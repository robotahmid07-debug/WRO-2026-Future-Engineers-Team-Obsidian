"""
Advanced Steering Controller for Single Motor + Steering Servo.

Features:
  - Low‑pass filter for smooth servo commands.
  - Speed‑dependent steering angle limiting (safer at high speeds).
  - Configurable max steering angle and smoothing factor.
  - Send speed + steering angle to ESP32 via serial bridge.
  - turn_around() method for 180° U‑turn (fixed‑time fallback).
  - turn_around_imu() method for 180° U‑turn using IMU heading feedback (closed‑loop).
  - set_localization() to pass localization object for IMU data.
  - All calibratable parameters are read from config (max_speed, max_steer_rad, etc.)
"""

import math
import time
import logging

logger = logging.getLogger(__name__)


class SteeringController:
    def __init__(self, serial_bridge,
                 max_speed: float = 0.5,
                 max_steer_rad: float = 0.524,   # ±30°
                 smoothing_alpha: float = 0.25,
                 steer_gain: float = 1.0):
        """
        Args:
            serial_bridge: Serial bridge to ESP32.
            max_speed: Maximum speed (m/s).
            max_steer_rad: Maximum steering angle (radians). Typical: 0.524 (30°).
            smoothing_alpha: Low‑pass filter coefficient (0 = no change, 1 = instant).
            steer_gain: Conversion factor from angular velocity (rad/s) to steering angle.
                        Default 1.0 assumes angular velocity is already close to steering angle.
                        Adjust if your car responds differently.
        """
        self.serial_bridge = serial_bridge
        self.max_speed = max_speed
        self.max_steer_rad = max_steer_rad
        self.smoothing_alpha = smoothing_alpha
        self.steer_gain = steer_gain

        # Internal state
        self.current_speed = 0.0
        self.current_steer = 0.0       # filtered steering angle
        self.last_raw_steer = 0.0      # for smoothing

        # Localization reference (for IMU-based U‑turn)
        self.localization = None

    def set_localization(self, localization):
        """Pass the localization object for IMU heading data."""
        self.localization = localization
        logger.info("Localization set for steering controller (IMU U‑turn available).")

    def set_speed(self, linear: float, angular: float) -> None:
        """
        Set the robot's speed and steering.

        Args:
            linear: Forward speed (m/s). Positive = forward, negative = reverse.
            angular: Angular velocity (rad/s). Positive = turn left.
        """
        # 1. Clamp speed
        linear = max(-self.max_speed, min(self.max_speed, linear))

        # 2. Speed‑dependent steering limiting (safer at high speed)
        # At low speed, allow full steering angle. At max speed, reduce to 60% of max.
        speed_fraction = abs(linear) / max(self.max_speed, 0.001)
        steer_limit_factor = max(0.6, 1.0 - speed_fraction * 0.4)
        current_max_steer = self.max_steer_rad * steer_limit_factor

        # 3. Map angular velocity to steering angle
        raw_steer = angular * self.steer_gain

        # 4. Clamp raw steer to the dynamic limit
        raw_steer = max(-current_max_steer, min(current_max_steer, raw_steer))

        # 5. Apply low‑pass filter (smoothing) to prevent jerky movements
        filtered_steer = (self.smoothing_alpha * raw_steer +
                          (1 - self.smoothing_alpha) * self.last_raw_steer)

        # 6. Store for next iteration
        self.last_raw_steer = filtered_steer
        self.current_speed = linear
        self.current_steer = filtered_steer

        # 7. Send command
        self._send_command()

    def _send_command(self) -> None:
        """Send speed and filtered steering angle to ESP32."""
        msg = {
            'type': 'cmd',
            'speed': self.current_speed,
            'steering': self.current_steer
        }
        self.serial_bridge.send(msg)

    def stop(self) -> None:
        """Emergency stop: set speed to 0 and center steering."""
        self.current_speed = 0.0
        self.current_steer = 0.0
        self.last_raw_steer = 0.0
        self._send_command()
        logger.info("Motors stopped, steering centered")

    def get_current_state(self):
        """Return current speed and steering angle (for debugging)."""
        return self.current_speed, self.current_steer

    # ============================================================
    # U‑Turn Methods
    # ============================================================

    def turn_around(self, speed: float = 0.1) -> None:
        """
        Execute a 180° turn in place (fixed‑time fallback).
        Steers fully to one side and reverses slightly, then centers steering.

        This is the fallback method if IMU is not available.
        """
        logger.info(f"Executing fixed‑time 180° turn at speed {speed} m/s")
        # 1. Stop and center steering first
        self.stop()
        time.sleep(0.2)

        # 2. Steer fully to one side (e.g., left)
        steer_angle = self.max_steer_rad * 0.9
        # Send a command with zero speed but full steering to set the servo
        self.set_speed(0.0, steer_angle / self.steer_gain)  # angular command
        time.sleep(0.3)  # allow servo to move

        # 3. Reverse slowly while maintaining the steering angle
        reverse_speed = -abs(speed)
        angular_cmd = steer_angle / self.steer_gain
        self.set_speed(reverse_speed, angular_cmd)
        time.sleep(1.5)  # duration – enough for a 180° turn

        # 4. Stop and center steering
        self.stop()
        logger.info("Fixed‑time 180° turn complete.")

    def turn_around_imu(self, speed: float = 0.1, target_angle_deg: float = 180.0):
        """
        Execute a 180° U‑turn using IMU heading feedback (closed‑loop).
        This is more reliable than the fixed‑time version because it stops
        when the heading has actually changed by 180°, regardless of terrain.

        Falls back to `turn_around()` if localization or IMU is not available.
        """
        # Check if IMU is available via localization
        if self.localization is None:
            logger.warning("Localization not set – using fallback turn_around()")
            self.turn_around(speed)
            return

        # Check if localization has a valid pose and IMU data
        # We'll read from localization.current_pose.theta (which is fused with IMU)
        # but we also need to ensure IMU is available; we can check a flag.
        if not hasattr(self.localization, 'imu_available') or not self.localization.imu_available:
            logger.warning("IMU not available – using fallback turn_around()")
            self.turn_around(speed)
            return

        logger.info(f"Executing IMU‑based 180° turn at speed {speed} m/s")

        # 1. Stop and center steering
        self.stop()
        time.sleep(0.2)

        # 2. Steer fully to one side (left)
        steer_angle = self.max_steer_rad * 0.9
        # Send a command with zero speed but full steering to set the servo
        self.set_speed(0.0, steer_angle / self.steer_gain)
        time.sleep(0.3)  # allow servo to move

        # 3. Get initial heading from localization
        start_heading = self.localization.current_pose.theta  # radians
        turned = 0.0
        target_angle_rad = math.radians(target_angle_deg)

        # 4. Reverse slowly while turning until heading change reaches target
        reverse_speed = -abs(speed)
        angular_cmd = steer_angle / self.steer_gain
        start_time = time.time()
        timeout = 5.0  # safety timeout (seconds)

        while abs(turned) < target_angle_rad:
            # Update heading
            current_heading = self.localization.current_pose.theta
            # Compute delta (handle wrap‑around)
            delta = current_heading - start_heading
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi
            turned = delta

            # Apply reverse speed (steering already set)
            self.set_speed(reverse_speed, angular_cmd)
            time.sleep(0.02)   # small loop delay (50 Hz)

            # Safety timeout
            if time.time() - start_time > timeout:
                logger.warning("IMU U‑turn timeout – stopping")
                break

        # 5. Stop and center
        self.stop()
        logger.info(f"IMU‑based U‑turn complete. Turned {math.degrees(turned):.1f}°")
