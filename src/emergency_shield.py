"""
Smart Ultrasonic Emergency Shield with Traffic Rule Compliance.
- Follows traffic light rules (steer LEFT for RED, RIGHT for GREEN).
- Reverses when critically close (< critical_stop) if rear is clear.
- If rear is blocked, it hard‑steers in the intended direction (traffic rule) and crawls forward.
- If no traffic rule is active (target_steer_direction = 0), it uses LIDAR side distances
  to steer toward the clearer side (parking lot escape).
- Uses LIDAR (360°) to check rear clearance before reversing.
"""

import time
import math
import logging
from typing import Dict, Optional

logger = logging.getLogger(__name__)


class EmergencyShield:
    def __init__(self, config: dict, serial_bridge):
        self.config = config
        self.serial_bridge = serial_bridge

        # Load configuration
        self.enabled = config.get('enabled', True)
        self.thresholds = config.get('thresholds_cm', {})

        # Core distances (from YAML)
        self.front_stop = self.thresholds.get('front_stop', 12.0)
        self.front_left_safety = self.thresholds.get('front_left_safety', 18.0)
        self.front_right_safety = self.thresholds.get('front_right_safety', 18.0)
        self.critical_stop = self.thresholds.get('critical_stop', 8.0)

        # Side safety (minimum of left and right)
        self.side_safety = min(self.front_left_safety, self.front_right_safety)

        self.dynamic_throttle = config.get('dynamic_throttle', {})
        self.dampening_enabled = self.dynamic_throttle.get('enable_speed_dampening', True)
        self.dampened_factor = self.dynamic_throttle.get('dampened_speed_factor', 0.60)

        # ---- Steering and Reverse Tuning ----
        self.steer_gain = 0.4

        # Reverse parameters – tuned for 10 cm backward movement
        # Distance = speed × duration → 0.1 × 1.0 = 0.1 m (10 cm)
        self.reverse_speed = -0.10      # m/s (negative = reverse)
        self.reverse_duration = 1.0     # seconds

        # Minimum rear clearance required before reversing (from LIDAR)
        self.rear_clearance_threshold = 0.15   # meters (15 cm)

        # Smoothing
        self.smoothing_factor = 0.3
        self.last_steer = 0.0
        self.last_throttle = 1.0

        # Sensor data
        self.latest_distances = {
            'front': float('inf'),
            'front_left': float('inf'),
            'front_right': float('inf')
        }
        self.last_update = 0.0

        # Traffic rule target (set by state_machine)
        self.target_steer_direction = 0.0   # -1 = right, +1 = left, 0 = straight

        # Reference to LIDAR (for rear clearance and side clearance checks)
        self.lidar = None

    def set_lidar(self, lidar_object):
        """Set the LIDAR object for rear and side clearance checks."""
        self.lidar = lidar_object

    def update(self) -> Dict[str, float]:
        """Poll serial bridge for latest ultrasonic data."""
        msg = self.serial_bridge.receive(block=False)
        if msg and msg.get('type') == 'ultrasonic':
            data = msg.get('data', {})
            self.latest_distances['front'] = data.get('front', float('inf'))
            self.latest_distances['front_left'] = data.get('front_left', float('inf'))
            self.latest_distances['front_right'] = data.get('front_right', float('inf'))
            self.last_update = time.time()
        return self.latest_distances

    def set_target_steer_direction(self, direction: float):
        """Tell the emergency shield which way we want to go (traffic rule)."""
        self.target_steer_direction = direction

    def _is_rear_clear(self) -> bool:
        """
        Check if the rear path is clear using LIDAR (angle around 180°).
        Returns True if clear, False if blocked.
        """
        if self.lidar is None:
            # No LIDAR provided – assume clear (but log a warning)
            logger.warning("LIDAR not set for rear clearance check – assuming clear.")
            return True

        scan = self.lidar.get_scan_snapshot()
        if not scan:
            # No scan available – assume clear
            return True

        rear_dists = []
        for ang, dist in scan.items():
            if dist < 0.05 or dist > 5.0:
                continue
            # Rear sector: 180° ± 30°
            if abs(abs(ang) - 180) < 30:
                rear_dists.append(dist)

        if not rear_dists:
            # No rear readings – assume clear
            return True

        avg_rear = sum(rear_dists) / len(rear_dists)
        return avg_rear > self.rear_clearance_threshold

    def _get_clearer_side(self) -> float:
        """
        Use LIDAR side distances to determine which side (left or right) is clearer.
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
            if abs(ang - 90) < 10:          # left side
                left_dists.append(dist)
            elif abs(ang + 90) < 10:        # right side
                right_dists.append(dist)

        if not left_dists or not right_dists:
            return 0.0

        avg_left = sum(left_dists) / len(left_dists)
        avg_right = sum(right_dists) / len(right_dists)

        # If one side is significantly clearer (e.g., > 0.2 m difference), steer that way
        diff = avg_left - avg_right
        if diff > 0.2:
            return 1.0   # left is clearer
        elif diff < -0.2:
            return -1.0  # right is clearer
        else:
            return 0.0   # no clear preference

    def get_emergency_actions(self) -> Dict[str, any]:
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
            return {'brake': False, 'steer_offset': 0.0, 'throttle_factor': 1.0,
                    'reverse': False, 'reverse_duration': 0.0, 'reverse_speed': 0.0}

        dist_f = self.latest_distances.get('front', float('inf'))
        dist_fl = self.latest_distances.get('front_left', float('inf'))
        dist_fr = self.latest_distances.get('front_right', float('inf'))

        # ---- 1. CRITICAL: Reverse if front is critically close ----
        if dist_f < self.critical_stop:
            # Check rear clearance before reversing
            if self._is_rear_clear():
                logger.warning(f"CRITICAL: Front obstacle at {dist_f:.1f} cm! Reversing (rear clear).")
                return {
                    'brake': False,
                    'steer_offset': self.target_steer_direction * self.steer_gain * 0.5,
                    'throttle_factor': 0.0,
                    'reverse': True,
                    'reverse_duration': self.reverse_duration,
                    'reverse_speed': self.reverse_speed
                }
            else:
                # Rear is blocked – cannot reverse.
                # Determine steering direction:
                # 1. If a traffic rule is active, use it (hard steer in that direction).
                # 2. Otherwise, use LIDAR to steer toward the clearer side.
                if abs(self.target_steer_direction) > 0.1:
                    # Use traffic rule direction
                    steer_hard = self.target_steer_direction * self.steer_gain * 1.5
                    logger.warning(f"CRITICAL: Front obstacle at {dist_f:.1f} cm, rear blocked! "
                                   f"Hard steer in traffic rule direction: {steer_hard:.2f} rad/s, crawl forward.")
                else:
                    # No traffic rule – use LIDAR to find the clearer side
                    side = self._get_clearer_side()
                    if abs(side) > 0.1:
                        steer_hard = side * self.steer_gain * 1.5
                        logger.warning(f"CRITICAL: Front obstacle at {dist_f:.1f} cm, rear blocked! "
                                       f"Hard steer toward clearer side: {steer_hard:.2f} rad/s, crawl forward.")
                    else:
                        # No clear side preference – go straight (avoid making a wrong guess)
                        steer_hard = 0.0
                        logger.warning(f"CRITICAL: Front obstacle at {dist_f:.1f} cm, rear blocked, "
                                       "no clear side preference. Going straight slowly.")

                # Clamp steer_hard to reasonable range (e.g., ±0.6 rad/s)
                steer_hard = max(-0.6, min(0.6, steer_hard))
                return {
                    'brake': False,
                    'steer_offset': steer_hard,
                    'throttle_factor': 0.3,      # Slow crawl (30% speed)
                    'reverse': False,
                    'reverse_duration': 0.0,
                    'reverse_speed': 0.0
                }

        # ---- 2. FRONT OBSTACLE (but not critical) ----
        if dist_f < self.front_stop:
            # Steer according to traffic rule, reduce speed
            steer = self.target_steer_direction * self.steer_gain
            throttle = 0.4
            logger.debug(f"Front obstacle at {dist_f:.1f} cm. Steering {self.target_steer_direction:.2f}.")
            return {
                'brake': False,
                'steer_offset': steer,
                'throttle_factor': throttle,
                'reverse': False,
                'reverse_duration': 0.0,
                'reverse_speed': 0.0
            }

        # ---- 3. SIDE OBSTACLE: Steer away ----
        left_blocked = dist_fl < self.front_left_safety
        right_blocked = dist_fr < self.front_right_safety

        raw_steer = 0.0
        if left_blocked and not right_blocked:
            raw_steer = -self.steer_gain   # steer right
        elif right_blocked and not left_blocked:
            raw_steer = self.steer_gain    # steer left
        elif left_blocked and right_blocked:
            raw_steer = 0.0                # both blocked – go straight

        # ---- 4. Blend with traffic rule target ----
        if abs(self.target_steer_direction) > 0.1:
            combined_steer = (0.7 * self.target_steer_direction * self.steer_gain +
                              0.3 * raw_steer)
        else:
            combined_steer = raw_steer

        # ---- 5. Proximity-based throttle ----
        if dist_f < self.side_safety * 1.5:
            prox_factor = max(0.4, (dist_f - self.front_stop) /
                              ((self.side_safety * 1.5) - self.front_stop))
            prox_factor = min(1.0, prox_factor)
        else:
            prox_factor = 1.0

        if self.dampening_enabled and (left_blocked or right_blocked or dist_f < self.front_stop):
            throttle = self.dampened_factor * prox_factor
        else:
            throttle = 1.0 * prox_factor

        # ---- 6. Smoothing ----
        smoothed_steer = (self.smoothing_factor * combined_steer +
                          (1 - self.smoothing_factor) * self.last_steer)
        smoothed_throttle = (self.smoothing_factor * throttle +
                             (1 - self.smoothing_factor) * self.last_throttle)

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
