"""
Ultrasonic Emergency Shield with Advanced Obstacle Avoidance.
Implements a reactive, smooth, and fast obstacle avoidance algorithm.
Based on the LSRB (Left-Straight-Right-Back) decision logic with smooth
transitions to prevent jerky movements.
"""

import time
import logging
from typing import Dict, Tuple, Optional

logger = logging.getLogger(__name__)


class EmergencyShield:
    def __init__(self, config: dict, serial_bridge):
        self.config = config
        self.serial_bridge = serial_bridge

        # Load configuration
        self.enabled = config.get('enabled', True)
        self.thresholds = config.get('thresholds_cm', {})
        self.dynamic_throttle = config.get('dynamic_throttle', {})

        # --- Core Safety Thresholds ---
        # FRONT: Hard stop distance
        self.front_stop = self.thresholds.get('front_stop', 15.0)
        # SIDE: Distance at which we start turning away
        self.side_safety = self.thresholds.get('side_safety', 25.0)
        # CRITICAL: Distance at which we perform an emergency reverse
        self.critical_stop = self.thresholds.get('critical_stop', 8.0)

        # --- Smoothing & Speed Control ---
        self.dampening_enabled = self.dynamic_throttle.get('enable_speed_dampening', True)
        self.dampened_factor = self.dynamic_throttle.get('dampened_speed_factor', 0.60)

        # --- Reactive Navigation Parameters ---
        # How aggressively we turn away from obstacles (rad/s)
        self.steer_gain = 0.4
        # How fast we reverse in an emergency (m/s, negative)
        self.reverse_speed = -0.15
        # Speed reduction factor based on obstacle proximity
        self.proximity_speed_factor = 0.7

        # --- State Variables for Smoothing ---
        self.last_decision = {
            'steer_offset': 0.0,
            'throttle_factor': 1.0,
            'linear_velocity': 0.3  # base speed
        }
        self.smoothing_factor = 0.3  # Low-pass filter coefficient (0-1)

        # --- Sensor Data ---
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
        Returns actions based on current distances using a reactive algorithm.

        Actions: {'brake': bool, 'steer_offset': float (rad/s),
                  'throttle_factor': float, 'linear_velocity': float}
        """
        if not self.enabled:
            return {'brake': False, 'steer_offset': 0.0,
                    'throttle_factor': 1.0, 'linear_velocity': 0.3}

        dist_f = self.latest_distances.get('front', float('inf'))
        dist_fl = self.latest_distances.get('front_left', float('inf'))
        dist_fr = self.latest_distances.get('front_right', float('inf'))

        # --- 1. CRITICAL: Immediate Reverse if front is too close ---
        if dist_f < self.critical_stop:
            logger.warning(f"CRITICAL: Front obstacle at {dist_f:.1f} cm! Reversing.")
            return {
                'brake': False,
                'steer_offset': 0.0,
                'throttle_factor': 1.0,
                'linear_velocity': self.reverse_speed  # Go backwards
            }

        # --- 2. HARD STOP: Front obstacle at stop distance ---
        if dist_f < self.front_stop:
            logger.warning(f"Hard stop: Front obstacle at {dist_f:.1f} cm.")
            return {
                'brake': True,
                'steer_offset': 0.0,
                'throttle_factor': 0.0,
                'linear_velocity': 0.0
            }

        # --- 3. REACTIVE AVOIDANCE: LSRB-inspired decision logic ---
        # Determine the safest direction based on side distances
        # The robot will turn TOWARDS the clearer side.

        # Check if left or right side is blocked
        left_blocked = dist_fl < self.side_safety
        right_blocked = dist_fr < self.side_safety

        # Base steering: go straight if both sides are clear or blocked
        raw_steer = 0.0

        if left_blocked and not right_blocked:
            # Left is blocked, right is clear -> Turn RIGHT (negative)
            raw_steer = -self.steer_gain
            logger.debug(f"Left blocked ({dist_fl:.1f}cm). Turning RIGHT.")
        elif right_blocked and not left_blocked:
            # Right is blocked, left is clear -> Turn LEFT (positive)
            raw_steer = self.steer_gain
            logger.debug(f"Right blocked ({dist_fr:.1f}cm). Turning LEFT.")
        elif left_blocked and right_blocked:
            # Both sides blocked -> Go straight and slow down (or stop)
            # In a maze, this might mean the robot is in a dead end.
            # We'll continue forward slowly to re-evaluate.
            raw_steer = 0.0
            logger.debug("Both sides blocked. Moving forward slowly.")
        else:
            # Both sides clear -> Go straight at full speed
            raw_steer = 0.0
            logger.debug("Path clear. Going straight.")

        # --- 4. PROXIMITY-BASED SPEED CONTROL ---
        # Reduce speed as front obstacle gets closer (even before stop distance)
        proximity_factor = 1.0
        if dist_f < self.side_safety * 1.5:  # 1.5x side safety distance
            # Linear interpolation: at front_stop -> 0.4, at side_safety*1.5 -> 1.0
            # Clamp factor between 0.4 and 1.0
            proximity_factor = max(0.4, (dist_f - self.front_stop) /
                                   ((self.side_safety * 1.5) - self.front_stop))
            # Ensure it doesn't exceed 1.0
            proximity_factor = min(1.0, proximity_factor)

        # --- 5. APPLY DAMPENING (from config) ---
        if self.dampening_enabled and (left_blocked or right_blocked):
            throttle_factor = self.dampened_factor * proximity_factor
        else:
            throttle_factor = 1.0 * proximity_factor

        # --- 6. SMOOTH THE OUTPUT (Low-pass filter) ---
        # Prevents jerky movements and makes the robot behave more naturally
        smoothed_steer = (self.smoothing_factor * raw_steer +
                          (1 - self.smoothing_factor) * self.last_decision['steer_offset'])
        smoothed_throttle = (self.smoothing_factor * throttle_factor +
                             (1 - self.smoothing_factor) * self.last_decision['throttle_factor'])

        # --- 7. Base linear velocity (will be multiplied by throttle in state_machine) ---
        # We still return a base linear velocity; state_machine will apply throttle
        base_linear = 0.3  # m/s

        # Store decision for next smoothing step
        self.last_decision['steer_offset'] = smoothed_steer
        self.last_decision['throttle_factor'] = smoothed_throttle
        self.last_decision['linear_velocity'] = base_linear

        return {
            'brake': False,
            'steer_offset': smoothed_steer,
            'throttle_factor': smoothed_throttle,
            'linear_velocity': base_linear
        }
