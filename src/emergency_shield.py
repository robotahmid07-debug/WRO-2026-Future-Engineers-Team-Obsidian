"""
Smart Ultrasonic Emergency Shield with Traffic Rule Compliance.

Behaviour:
  - Follows traffic light rules (steer LEFT for RED, RIGHT for GREEN).
  - Reverses when critically close (< critical_stop) if rear is clear.
  - If rear is blocked, it hard‑steers in the intended direction (traffic rule)
    and crawls forward.
  - If no traffic rule is active, it uses LIDAR side distances to steer
    toward the clearer side.
  - Hard shield (5 cm) as last‑resort: full reverse + hard steer away.
  - Parking mode: uses relaxed thresholds and can disable reverse/hard steer.

All features can be enabled/disabled independently via YAML.
"""

import time
import logging
from typing import Dict, Any

logger = logging.getLogger(__name__)


class EmergencyShield:
    def __init__(self, config: dict, serial_bridge):
        self.config = config
        self.serial_bridge = serial_bridge

        # ---- Load configuration ----
        self.enabled = config.get('enabled', True)
        self.enable_reverse = config.get('enable_reverse', True)
        self.enable_rear_check = config.get('enable_rear_check', True)
        self.enable_hard_shield = config.get('enable_hard_shield', True)
        self.enable_hard_steer = config.get('enable_hard_steer', True)

        self.thresholds = config.get('thresholds_cm', {})

        # Core distances (from YAML)
        self.front_stop = self.thresholds.get('front_stop', 12.0)
        self.front_left_safety = self.thresholds.get('front_left_safety', 18.0)
        self.front_right_safety = self.thresholds.get('front_right_safety', 18.0)
        self.critical_stop = self.thresholds.get('critical_stop', 8.0)
        self.hard_shield_cm = self.thresholds.get('hard_shield_cm', 5.0)

        # Side safety (minimum of left and right)
        self.side_safety = min(self.front_left_safety, self.front_right_safety)

        self.dynamic_throttle = config.get('dynamic_throttle', {})
        self.dampening_enabled = self.dynamic_throttle.get(
            'enable_speed_dampening', True
        )
        self.dampened_factor = self.dynamic_throttle.get(
            'dampened_speed_factor', 0.60
        )

        # ---- Reverse parameters ----
        reverse_params = config.get('reverse_params', {})
        self.reverse_speed = reverse_params.get('speed_mps', -0.10)
        self.reverse_duration = reverse_params.get('duration_s', 1.0)
        self.rear_clearance_threshold = reverse_params.get(
            'rear_clearance_threshold_m', 0.15
        )

        # ---- Steering and Smoothing ----
        self.steer_gain = 0.4
        self.smoothing_factor = 0.3
        self.last_steer = 0.0
        self.last_throttle = 1.0

        # ---- Sensor data ----
        self.latest_distances = {
            'front': float('inf'),
            'front_left': float('inf'),
            'front_right': float('inf')
        }
        self.last_update = 0.0

        # ---- Traffic rule target (set by state_machine) ----
        self.target_steer_direction = 0.0   # -1 = right, +1 = left, 0 = straight

        # ---- LIDAR reference ----
        self.lidar = None

        # ---- Parking mode ----
        self.parking_mode = False
        self.parking_config = config.get('parking', {}).get('emergency_shield', {})
        self.use_parking_thresholds = self.parking_config.get(
            'use_parking_thresholds', True
        )

    def set_lidar(self, lidar_object):
        """Set the LIDAR object for rear and side clearance checks."""
        self.lidar = lidar_object

    def set_parking_mode(self, enabled: bool):
        """Enable/disable parking mode."""
        self.parking_mode = enabled
        if enabled:
            logger.info("Emergency shield: PARKING MODE ACTIVE")
        else:
            logger.info("Emergency shield: NORMAL MODE")

    def update(self) -> Dict[str, float]:
        """
        Poll the shared sensor state instead of consuming from the serial queue.
        Updates internal distances from the latest sensor data.
        """
        # Read from shared state (non‑consuming)
        sensor_data = self.serial_bridge.get_latest_sensor_data()
        if sensor_data and isinstance(sensor_data, dict):
            # Map ultrasonic fields to front, front_left, front_right
            # Assumes: ultrasonic1 = front, ultrasonic2 = left, ultrasonic3 = right
            # Adjust keys if your ESP32 uses different field names.
            self.latest_distances['front'] = sensor_data.get('ultrasonic1', float('inf'))
            self.latest_distances['front_left'] = sensor_data.get('ultrasonic2', float('inf'))
            self.latest_distances['front_right'] = sensor_data.get('ultrasonic3', float('inf'))
            self.last_update = time.time()
        else:
            # No new data – leave distances unchanged; log if stale
            if time.time() - self.last_update > 0.5:
                logger.warning("EmergencyShield: No sensor data for >0.5s")
        return self.latest_distances

    def set_target_steer_direction(self, direction: float):
        """Tell the emergency shield which way we want to go (traffic rule)."""
        self.target_steer_direction = direction

    def _is_rear_clear(self) -> bool:
        """
        Check if the rear path is clear using LIDAR (angle around 180°).
        Returns True if clear, False if blocked.
        """
        if not self.enable_rear_check:
            logger.warning("Rear check disabled – assuming clear.")
            return True

        if self.lidar is None:
            logger.warning("LIDAR not set for rear clearance check – assuming clear.")
            return True

        scan = self.lidar.get_scan_snapshot()
        if not scan:
            return True

        rear_dists = []
        for ang, dist in scan.items():
            if dist < 0.05 or dist > 5.0:
                continue
            # Rear sector: 180° ± 30°
            if abs(abs(ang) - 180) < 30:
                rear_dists.append(dist)

        if not rear_dists:
            return True

        avg_rear = sum(rear_dists) / len(rear_dists)
        return avg_rear > self.rear_clearance_threshold

    def _get_clearer_side(self) -> float:
        """
        Use LIDAR side distances to determine which side is clearer.
        Returns: +1.0 if left is clearer, -1.0 if right is clearer, 0.0 if unknown.
        """
        if self.lidar is None:
            return 0.0

        scan = self.lidar.get_scan_snapshot()
        if not scan:
            return 0.0

        left_dists = []
        right_dists = []
        for ang, dist in scan.items():
            if dist < 0.05 or dist > 5.0:
                continue
            if abs(ang - 90) < 10:
                left_dists.append(dist)
            elif abs(ang + 90) < 10:
                right_dists.append(dist)

        if not left_dists or not right_dists:
            return 0.0

        avg_left = sum(left_dists) / len(left_dists)
        avg_right = sum(right_dists) / len(right_dists)

        diff = avg_left - avg_right
        if diff > 0.2:
            return 1.0   # left is clearer
        elif diff < -0.2:
            return -1.0  # right is clearer
        else:
            return 0.0   # no clear preference

    def get_emergency_actions(self) -> Dict[str, Any]:
        """
        Returns actions based on current distances.

        Returns:
            dict with keys:
                brake (bool): True only if critically stuck.
                steer_offset (float): additional angular velocity (rad/s).
                throttle_factor (float): speed multiplier.
                reverse (bool): True if reversing.
                reverse_duration (float): seconds to reverse.
                reverse_speed (float): speed during reverse (m/s).
        """
        if not self.enabled:
            return {
                'brake': False,
                'steer_offset': 0.0,
                'throttle_factor': 1.0,
                'reverse': False,
                'reverse_duration': 0.0,
                'reverse_speed': 0.0
            }

        # ---- Get current distances ----
        dist_f = self.latest_distances.get('front', float('inf'))
        dist_fl = self.latest_distances.get('front_left', float('inf'))
        dist_fr = self.latest_distances.get('front_right', float('inf'))

        # ---- Determine thresholds ----
        if self.parking_mode and self.use_parking_thresholds:
            pth = self.parking_config.get('thresholds_cm', {})
            front_stop = pth.get('front_stop', self.front_stop)
            critical_stop = pth.get('critical_stop', self.critical_stop)
            front_left_safety = pth.get(
                'front_left_safety', self.front_left_safety
            )
            front_right_safety = pth.get(
                'front_right_safety', self.front_right_safety
            )
            hard_shield_cm = pth.get('hard_shield_cm', self.hard_shield_cm)
            enable_reverse = not self.parking_config.get('disable_reverse', False)
            enable_hard_steer = not self.parking_config.get(
                'disable_hard_steer', False
            )
            enable_hard_shield = not self.parking_config.get(
                'disable_hard_shield', False
            )
        else:
            front_stop = self.front_stop
            critical_stop = self.critical_stop
            front_left_safety = self.front_left_safety
            front_right_safety = self.front_right_safety
            hard_shield_cm = self.hard_shield_cm
            enable_reverse = self.enable_reverse
            enable_hard_steer = self.enable_hard_steer
            enable_hard_shield = self.enable_hard_shield

        # ---- 0. HARD SHIELD (Last-resort) ----
        if enable_hard_shield and (
            dist_f < hard_shield_cm or
            dist_fl < hard_shield_cm or
            dist_fr < hard_shield_cm
        ):
            # Determine steer direction: away from the closest side
            if dist_fl < dist_fr:
                steer_hard = self.steer_gain * 1.5   # steer right
            else:
                steer_hard = -self.steer_gain * 1.5  # steer left

            # If front is the only issue, use traffic rule direction
            if (
                dist_f < hard_shield_cm and
                dist_fl > hard_shield_cm and
                dist_fr > hard_shield_cm
            ):
                steer_hard = self.target_steer_direction * self.steer_gain * 1.5

            logger.warning(
                f"HARD SHIELD TRIGGERED: F={dist_f:.1f}, FL={dist_fl:.1f}, "
                f"FR={dist_fr:.1f} cm"
            )
            return {
                'brake': False,
                'steer_offset': steer_hard,
                'throttle_factor': -0.2,   # reverse
                'reverse': True,
                'reverse_duration': 0.5,
                'reverse_speed': -0.15
            }

        # ---- 1. CRITICAL: Reverse if front is critically close ----
        if dist_f < critical_stop:
            if enable_reverse:
                if self._is_rear_clear():
                    logger.warning(
                        f"CRITICAL: Front obstacle at {dist_f:.1f} cm! "
                        "Reversing (rear clear)."
                    )
                    return {
                        'brake': False,
                        'steer_offset': self.target_steer_direction * 0.5,
                        'throttle_factor': 0.0,
                        'reverse': True,
                        'reverse_duration': self.reverse_duration,
                        'reverse_speed': self.reverse_speed
                    }
                else:
                    if enable_hard_steer:
                        if abs(self.target_steer_direction) > 0.1:
                            steer_hard = (
                                self.target_steer_direction *
                                self.steer_gain * 1.5
                            )
                        else:
                            side = self._get_clearer_side()
                            if abs(side) > 0.1:
                                steer_hard = side * self.steer_gain * 1.5
                            else:
                                steer_hard = 0.0
                        steer_hard = max(-0.6, min(0.6, steer_hard))
                        logger.warning(
                            f"CRITICAL: Front obstacle at {dist_f:.1f} cm, "
                            f"rear blocked! Hard steer {steer_hard:.2f} rad/s, "
                            "crawl forward."
                        )
                        return {
                            'brake': False,
                            'steer_offset': steer_hard,
                            'throttle_factor': 0.3,
                            'reverse': False,
                            'reverse_duration': 0.0,
                            'reverse_speed': 0.0
                        }
                    else:
                        return {
                            'brake': True,
                            'steer_offset': 0.0,
                            'throttle_factor': 0.0,
                            'reverse': False,
                            'reverse_duration': 0.0,
                            'reverse_speed': 0.0
                        }
            else:
                if enable_hard_steer:
                    side = self._get_clearer_side()
                    steer_hard = side * self.steer_gain * 1.5 if abs(side) > 0.1 else 0.0
                    steer_hard = max(-0.6, min(0.6, steer_hard))
                    return {
                        'brake': False,
                        'steer_offset': steer_hard,
                        'throttle_factor': 0.3,
                        'reverse': False,
                        'reverse_duration': 0.0,
                        'reverse_speed': 0.0
                    }
                else:
                    return {
                        'brake': True,
                        'steer_offset': 0.0,
                        'throttle_factor': 0.0,
                        'reverse': False,
                        'reverse_duration': 0.0,
                        'reverse_speed': 0.0
                    }

        # ---- 2. FRONT OBSTACLE (but not critical) ----
        if dist_f < front_stop:
            steer = self.target_steer_direction * self.steer_gain
            throttle = 0.4
            logger.debug(
                f"Front obstacle at {dist_f:.1f} cm. "
                f"Steering {self.target_steer_direction:.2f}."
            )
            return {
                'brake': False,
                'steer_offset': steer,
                'throttle_factor': throttle,
                'reverse': False,
                'reverse_duration': 0.0,
                'reverse_speed': 0.0
            }

        # ---- 3. SIDE OBSTACLE: Steer away ----
        left_blocked = dist_fl < front_left_safety
        right_blocked = dist_fr < front_right_safety

        raw_steer = 0.0
        if left_blocked and not right_blocked:
            raw_steer = -self.steer_gain   # steer right
        elif right_blocked and not left_blocked:
            raw_steer = self.steer_gain    # steer left
        elif left_blocked and right_blocked:
            raw_steer = 0.0                # both blocked – go straight

        # ---- 4. Blend with traffic rule target ----
        if abs(self.target_steer_direction) > 0.1:
            combined_steer = (
                0.7 * self.target_steer_direction * self.steer_gain +
                0.3 * raw_steer
            )
        else:
            combined_steer = raw_steer

        # ---- 5. Proximity-based throttle ----
        side_safety = min(front_left_safety, front_right_safety)
        if dist_f < side_safety * 1.5:
            prox_factor = max(
                0.4,
                (dist_f - front_stop) / ((side_safety * 1.5) - front_stop)
            )
            prox_factor = min(1.0, prox_factor)
        else:
            prox_factor = 1.0

        if self.dampening_enabled and (
            left_blocked or right_blocked or dist_f < front_stop
        ):
            throttle = self.dampened_factor * prox_factor
        else:
            throttle = 1.0 * prox_factor

        # ---- 6. Smoothing ----
        smoothed_steer = (
            self.smoothing_factor * combined_steer +
            (1 - self.smoothing_factor) * self.last_steer
        )
        smoothed_throttle = (
            self.smoothing_factor * throttle +
            (1 - self.smoothing_factor) * self.last_throttle
        )

        self.last_steer = smoothed_steer
        self.last_throttle = smoothed_throttle

        return {
            'brake': False,
            'steer_offset': smoothed_steer,
            'throttle_factor': smoothed_throttle,
            'reverse': False,
            'reverse_duration': 0.0,
            'reverse_speed': 0.0
        }
