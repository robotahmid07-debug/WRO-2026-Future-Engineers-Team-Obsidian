"""
Ultrasonic Emergency Shield with Advanced Obstacle Avoidance.
Implements reactive, smooth avoidance using only existing YAML keys.
Compatible with state_machine.py without modifications.
"""

import time
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

        # Derived thresholds for advanced logic
        # critical_stop = 60% of front_stop (emergency reverse)
        self.critical_stop = self.front_stop * 0.6
        # side_safety = use the lower of the two side thresholds
        self.side_safety = min(self.front_left_safety, self.front_right_safety)

        self.dynamic_throttle = config.get('dynamic_throttle', {})
        self.dampening_enabled = self.dynamic_throttle.get('enable_speed_dampening', True)
        self.dampened_factor = self.dynamic_throttle.get('dampened_speed_factor', 0.60)

        # Steering gain (rad/s per unit of error)
        self.steer_gain = 0.4

        # Smoothing filter (0 = no smoothing, 1 = instant)
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

    def get_emergency_actions(self) -> Dict[str, any]:
        """
        Returns actions based on current distances.

        Returns:
            dict with keys:
                brake (bool): True if hard stop required.
                steer_offset (float): additional angular velocity (rad/s).
                throttle_factor (float): speed multiplier (0.0 to 1.0).
        """
        if not self.enabled:
            return {'brake': False, 'steer_offset': 0.0, 'throttle_factor': 1.0}

        dist_f = self.latest_distances.get('front', float('inf'))
        dist_fl = self.latest_distances.get('front_left', float('inf'))
        dist_fr = self.latest_distances.get('front_right', float('inf'))

        # --- 1. CRITICAL: Reverse if front is too close ---
        if dist_f < self.critical_stop:
            # We cannot reverse via throttle_factor (it scales speed from 0..1).
            # Instead, we set brake=False and throttle_factor = -0.3 to request reverse.
            # But state_machine expects throttle_factor >=0 and uses it as multiplier.
            # So we override by returning a negative linear velocity? No, we don't have that.
            # Best solution: set brake=True (hard stop) to be safe.
            # Or we could modify state_machine to use linear_velocity, but we avoid that.
            # So we do a hard stop here for safety.
            logger.warning(f"CRITICAL: Front obstacle at {dist_f:.1f} cm! Hard stop.")
            return {'brake': True, 'steer_offset': 0.0, 'throttle_factor': 0.0}

        # --- 2. HARD STOP: Front obstacle at stop distance ---
        if dist_f < self.front_stop:
            logger.warning(f"Hard stop: Front obstacle at {dist_f:.1f} cm.")
            return {'brake': True, 'steer_offset': 0.0, 'throttle_factor': 0.0}

        # --- 3. REACTIVE AVOIDANCE (LSRB logic) ---
        left_blocked = dist_fl < self.side_safety
        right_blocked = dist_fr < self.side_safety

        raw_steer = 0.0
        if left_blocked and not right_blocked:
            # Left blocked → steer right (negative)
            raw_steer = -self.steer_gain
        elif right_blocked and not left_blocked:
            # Right blocked → steer left (positive)
            raw_steer = self.steer_gain
        elif left_blocked and right_blocked:
            # Both blocked → go straight (or could turn slightly)
            raw_steer = 0.0

        # --- 4. PROXIMITY-BASED THROTTLE ---
        # Reduce speed as front obstacle gets closer
        if dist_f < self.side_safety * 1.5:
            # Linear interpolation: at front_stop -> 0.4, at side_safety*1.5 -> 1.0
            prox_factor = max(0.4, (dist_f - self.front_stop) /
                              ((self.side_safety * 1.5) - self.front_stop))
            prox_factor = min(1.0, prox_factor)
        else:
            prox_factor = 1.0

        # Apply dampening if any side is blocked
        if self.dampening_enabled and (left_blocked or right_blocked):
            throttle = self.dampened_factor * prox_factor
        else:
            throttle = 1.0 * prox_factor

        # --- 5. SMOOTHING ---
        smoothed_steer = (self.smoothing_factor * raw_steer +
                          (1 - self.smoothing_factor) * self.last_steer)
        smoothed_throttle = (self.smoothing_factor * throttle +
                             (1 - self.smoothing_factor) * self.last_throttle)

        # Store for next iteration
        self.last_steer = smoothed_steer
        self.last_throttle = smoothed_throttle

        return {
            'brake': False,
            'steer_offset': smoothed_steer,
            'throttle_factor': smoothed_throttle
        }
