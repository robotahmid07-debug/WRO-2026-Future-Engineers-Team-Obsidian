"""
Ackermann steering controller.
Converts (linear speed, yaw rate) -> (left rear speed, right rear speed) using Ackermann geometry.
"""

import math
import logging
from typing import Tuple

logger = logging.getLogger(__name__)


class SteeringController:
    def __init__(self, serial_bridge, wheelbase: float = 0.25, trackwidth: float = 0.15, max_speed: float = 0.5):
        """
        Args:
            serial_bridge: Serial bridge to ESP32.
            wheelbase: Distance between front and rear axles (meters).
            trackwidth: Distance between left and right rear wheels (meters).
            max_speed: Maximum linear speed (m/s).
        """
        self.serial_bridge = serial_bridge
        self.wheelbase = wheelbase
        self.trackwidth = trackwidth
        self.max_speed = max_speed
        self.current_left = 0.0
        self.current_right = 0.0

    def set_speed(self, linear: float, angular: float) -> None:
        """
        Set the robot's speed.

        Args:
            linear: Desired forward speed (m/s). Positive = forward.
            angular: Desired yaw rate (rad/s). Positive = turn left.
        """
        # 1. Clamp linear speed
        linear = max(-self.max_speed, min(self.max_speed, linear))

        # 2. Compute steering angle from yaw rate
        # Formula: angular = (v / L) * tan(delta) => delta = atan2(angular * L, v)
        if abs(linear) < 0.001:
            # If stationary, we cannot turn. Set both wheels to 0.
            self.current_left = 0.0
            self.current_right = 0.0
            self._send_command()
            return

        # Compute steering angle (front wheel)
        steering_angle = math.atan2(angular * self.wheelbase, linear)

        # Clamp steering angle to mechanical limits (e.g., ±30 degrees)
        max_steer_rad = math.radians(30.0)
        steering_angle = max(-max_steer_rad, min(max_steer_rad, steering_angle))

        # 3. Ackermann speed calculation for rear wheels
        # If steering angle is near zero, go straight.
        if abs(steering_angle) < 0.001:
            left_speed = linear
            right_speed = linear
        else:
            # Turning radius R = L / tan(delta)
            R = self.wheelbase / math.tan(steering_angle)

            # Speeds for left and right rear wheels
            # V_left = V * (R - T/2) / R
            # V_right = V * (R + T/2) / R
            left_speed = linear * (R - self.trackwidth / 2.0) / R
            right_speed = linear * (R + self.trackwidth / 2.0) / R

        # 4. Clamp individual wheel speeds to max_speed
        self.current_left = max(-self.max_speed, min(self.max_speed, left_speed))
        self.current_right = max(-self.max_speed, min(self.max_speed, right_speed))

        # 5. Send command to ESP32
        self._send_command()

        # Debug logging (optional)
        # logger.debug(f"Steering: linear={linear:.2f}, angular={angular:.2f} -> "
        #              f"left={self.current_left:.2f}, right={self.current_right:.2f}")

    def _send_command(self) -> None:
        """Send motor speeds to ESP32 via serial bridge."""
        msg = {
            'type': 'motor_command',
            'data': {
                'left': self.current_left,
                'right': self.current_right
            }
        }
        self.serial_bridge.send(msg)

    def stop(self) -> None:
        """Emergency stop: set both motors to 0."""
        self.current_left = 0.0
        self.current_right = 0.0
        self._send_command()
