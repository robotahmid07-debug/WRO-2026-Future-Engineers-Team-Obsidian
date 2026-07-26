"""
Smart Ultrasonic Emergency Shield with Traffic Rule Compliance.
- Follows traffic light rules (steer LEFT for RED, RIGHT for GREEN).
- Only reverses when critically close (< 8 cm).
- Uses LIDAR for parking (360° coverage) instead of ultrasonic.
"""

import time
import logging
from typing import Dict, Optional, Tuple

logger = logging.getLogger(__name__)


class EmergencyShield:
    def __init__(self, config: dict, serial_bridge):
        self.config = config
        self.serial_bridge = serial_bridge

        # Load configuration
        self.enabled = config.get('enabled', True)
        self.thresholds = config.get('thresholds_cm', {})

        # Core distances
        self.front_stop = self.thresholds.get('front_stop', 12.0)       # Start reducing speed
        self.front_left_safety = self.thresholds.get('front_left_safety', 18.0)
        self.front_right_safety = self.thresholds.get('front_right_safety', 18.0)
        self.critical_stop = self.thresholds.get('critical_stop', 8.0)  # Emergency reverse

        self.dynamic_throttle = config.get('dynamic_throttle', {})
        self.dampening_enabled = self.dynamic_throttle.get('enable_speed_dampening', True)
        self.dampened_factor = self.dynamic_throttle.get('dampened_speed_factor', 0.60)

        # Steering gains
        self.steer_gain = 0.4
        self.reverse_speed = -0.1  # m/s when reversing

        # Smoothing
        self.smoothing_factor = 0.3
        self.last_steer = 0.0
        self.last_throttle = 1.0

        # State
        self.latest_distances = {
            'front': float('inf'),
            'front_left': float('inf'),
            'front_right': float('inf')
        }
        self.last_update = 0.0
        self.reverse_timer = 0.0
        self.is_reversing = False

        # Traffic rule state (set by state_machine)
        self.target_steer_direction = 0.0  # -1 = right, +1 = left, 0 = straight

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
        """
        Set the intended steering direction based on traffic rules.
        direction: -1 = right, +1 = left, 0 = straight
        """
        self.target_steer_direction = direction

    def get_emergency_actions(self) -> Dict[str, any]:
        """
        Returns actions based on current distances and traffic rules.

        Returns:
            dict with keys:
                brake (bool): True if hard stop required (only if critically stuck).
                steer_offset (float): additional angular velocity (rad/s).
                throttle_factor (float): speed multiplier (0.0 to 1.0).
                reverse (bool): True if reversing.
                reverse_duration (float): seconds to reverse.
        """
        if not self.enabled:
            return {'brake': False, 'steer_offset': 0.0, 'throttle_factor': 1.0,
                    'reverse': False, 'reverse_duration': 0.0}

        dist_f = self.latest_distances.get('front', float('inf'))
        dist_fl = self.latest_distances.get('front_left', float('inf'))
        dist_fr = self.latest_distances.get('front_right', float('inf'))

        # --- 1. CRITICAL: Reverse if front is critically close ---
        if dist_f < self.critical_stop:
            logger.warning(f"CRITICAL: Front obstacle at {dist_f:.1f} cm! Reversing.")
            return {
                'brake': False,
                'steer_offset': self.target_steer_direction * self.steer_gain * 0.5,
                'throttle_factor': 0.0,
                'reverse': True,
                'reverse_duration': 1.5  # reverse for 1.5 seconds
            }

        # --- 2. FRONT OBSTACLE: Follow traffic rules ---
        if dist_f < self.front_stop:
            # Steer in the intended direction (traffic rule), but reduce speed
            steer = self.target_steer_direction * self.steer_gain
            throttle = 0.4  # reduce speed to 40%
            logger.debug(f"Front obstacle at {dist_f:.1f} cm. Steering {self.target_steer_direction:.2f}.")
            return {
                'brake': False,
                'steer_offset': steer,
                'throttle_factor': throttle,
                'reverse': False,
                'reverse_duration': 0.0
            }

        # --- 3. SIDE OBSTACLE: Steer away ---
        left_blocked = dist_fl < self.front_left_safety
        right_blocked = dist_fr < self.front_right_safety

        raw_steer = 0.0
        if left_blocked and not right_blocked:
            # Left blocked -> steer right (negative)
            raw_steer = -self.steer_gain
        elif right_blocked and not left_blocked:
            # Right blocked -> steer left (positive)
            raw_steer = self.steer_gain
        elif left_blocked and right_blocked:
            # Both blocked -> go straight but slow
            raw_steer = 0.0

        # --- 4. Combine with traffic rule target ---
        # If a traffic rule is active, bias steering toward it
        if abs(self.target_steer_direction) > 0.1:
            # Blend: 70% traffic rule, 30% obstacle avoidance
            combined_steer = (0.7 * self.target_steer_direction * self.steer_gain +
                              0.3 * raw_steer)
        else:
            combined_steer = raw_steer

        # --- 5. PROXIMITY-BASED THROTTLE ---
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

        # --- 6. SMOOTHING ---
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
            'reverse_duration': 0.0
        }
