"""
Ackermann steering localization using the Bicycle Model.
Updates pose based on rear axle velocity and front steering angle.
"""

import math
import time
import logging
from dataclasses import dataclass
from typing import Optional, Dict, Any

logger = logging.getLogger(__name__)


@dataclass
class Pose2D:
    x: float  # meters
    y: float  # meters
    theta: float  # radians (0 = forward)


class Localization:
    def __init__(self, lidar_fusion, config: dict):
        self.lidar = lidar_fusion
        self.config = config
        self.wheelbase = 0.25  # Distance between front and rear axles (meters) - CALIBRATE THIS
        self.current_pose = Pose2D(0.0, 0.0, 0.0)
        self.initial_pose = Pose2D(0.0, 0.0, 0.0)
        self.parking_bay_geometry: Optional[Dict[str, float]] = None
        self.last_update_time = time.time()

    def register_start_pose(self) -> None:
        """
        Register the starting pose. 
        In production, use LIDAR wall distances to refine this.
        For now, we set to origin (0,0) heading forward.
        """
        self.initial_pose = Pose2D(0.0, 0.0, 0.0)
        self.current_pose = Pose2D(0.0, 0.0, 0.0)
        self.last_update_time = time.time()
        logger.info("Start pose registered at (0, 0, 0)")

    def register_parking_slot(self) -> None:
        """
        Register parking bay geometry.
        In production, use LIDAR to detect the empty bay dimensions.
        """
        self.parking_bay_geometry = {
            'x_min': -0.5,   # relative to robot
            'x_max': 0.5,
            'y_min': 1.0,
            'y_max': 1.8
        }
        logger.info(f"Parking bay geometry registered: {self.parking_bay_geometry}")

    def update_pose(self, linear_vel: float, steering_angle: float) -> None:
        """
        Update the robot's pose using the Ackermann Bicycle Model.

        Args:
            linear_vel: Velocity of the rear axle (m/s). Positive = forward.
            steering_angle: Front wheel steering angle (radians). Positive = left turn.
        """
        current_time = time.time()
        dt = current_time - self.last_update_time

        # Clamp dt to prevent insane jumps if the loop stalls
        if dt <= 0 or dt > 0.1:
            self.last_update_time = current_time
            return

        # Bicycle model kinematics
        # If linear velocity is near zero, the heading doesn't change.
        if abs(linear_vel) < 0.001:
            # Still update time, but no pose change
            self.last_update_time = current_time
            return

        # Compute heading rate (yaw)
        # theta_dot = (v / L) * tan(delta)
        theta_dot = (linear_vel / self.wheelbase) * math.tan(steering_angle)

        # Update pose
        self.current_pose.theta += theta_dot * dt
        self.current_pose.x += linear_vel * math.cos(self.current_pose.theta) * dt
        self.current_pose.y += linear_vel * math.sin(self.current_pose.theta) * dt

        # Normalize theta to [-pi, pi]
        if self.current_pose.theta > math.pi:
            self.current_pose.theta -= 2 * math.pi
        elif self.current_pose.theta < -math.pi:
            self.current_pose.theta += 2 * math.pi

        self.last_update_time = current_time

    def get_pose(self) -> Pose2D:
        """Return the current estimated pose."""
        return self.current_pose

    def get_start_pose(self) -> Optional[Pose2D]:
        """Return the saved starting pose."""
        return self.initial_pose
