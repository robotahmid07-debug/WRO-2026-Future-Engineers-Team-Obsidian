"""
Ackermann steering localization using the Bicycle Model.
Integrates WallMapper for map‑based pose correction using LIDAR wall distances.
The LIDAR is assumed to be mounted on top, scanning 360°.
"""

import math
import time
import logging
from dataclasses import dataclass
from typing import Optional, List, Tuple

from .wall_mapper import WallMapper

logger = logging.getLogger(__name__)


@dataclass
class Pose2D:
    """Represents a 2D pose (x, y, theta) in meters and radians."""
    x: float
    y: float
    theta: float


class Localization:
    def __init__(self, lidar_fusion, config: dict):
        """
        Initialize localization with Ackermann bicycle model.

        Args:
            lidar_fusion: LidarFusion object (used for getting LIDAR scans).
            config: Configuration dictionary (from YAML).
        """
        self.lidar = lidar_fusion
        self.config = config
        self.wheelbase = 0.25  # meters, distance between front and rear axles

        # Current and initial poses
        self.current_pose = Pose2D(0.0, 0.0, 0.0)
        self.initial_pose = Pose2D(0.0, 0.0, 0.0)

        # Parking bay geometry (for obstacle challenge)
        self.parking_bay_geometry: Optional[dict] = None

        # Time tracking for odometry integration
        self.last_update_time = time.time()

        # Initialize WallMapper for map building and pose correction
        self.wall_mapper = WallMapper(resolution=0.02, map_size=400, update_threshold=30)

        # Flag to enable/disable map correction (can be controlled from config)
        self.use_map_correction = True

        # Store last LIDAR scan for pose correction
        self.last_lidar_scan: List[Tuple[float, float]] = []

    def register_start_pose(self) -> None:
        """Register the starting pose. Defaults to (0,0,0) until map is built."""
        self.initial_pose = Pose2D(0.0, 0.0, 0.0)
        self.current_pose = Pose2D(0.0, 0.0, 0.0)
        self.last_update_time = time.time()
        logger.info("Start pose registered at (0, 0, 0)")

    def register_parking_slot(self) -> None:
        """
        Register parking bay geometry (for obstacle challenge).
        In practice, this would be measured from LIDAR; here we use a default.
        """
        self.parking_bay_geometry = {
            'x_min': -0.5,
            'x_max': 0.5,
            'y_min': 1.0,
            'y_max': 1.8
        }
        logger.info(f"Parking bay geometry registered: {self.parking_bay_geometry}")

    def update_pose(self, linear_vel: float, steering_angle: float,
                    lidar_points: Optional[List[Tuple[float, float]]] = None) -> None:
        """
        Update the robot's pose using the Ackermann Bicycle Model.
        Also updates the wall mapper if LIDAR points are provided.

        Args:
            linear_vel: Velocity of the rear axle (m/s). Positive = forward.
            steering_angle: Front wheel steering angle (radians). Positive = left turn.
            lidar_points: Optional list of (angle_degrees, distance_meters) from LIDAR.
        """
        current_time = time.time()
        dt = current_time - self.last_update_time

        # Prevent massive jumps if the loop stalls
        if dt <= 0 or dt > 0.1:
            self.last_update_time = current_time
            return

        # Bicycle model kinematics
        if abs(linear_vel) < 0.001:
            # Stationary – no pose change
            self.last_update_time = current_time
            return

        # Compute heading rate (yaw) from steering angle
        theta_dot = (linear_vel / self.wheelbase) * math.tan(steering_angle)

        # Update pose using odometry (dead reckoning)
        self.current_pose.theta += theta_dot * dt
        self.current_pose.x += linear_vel * math.cos(self.current_pose.theta) * dt
        self.current_pose.y += linear_vel * math.sin(self.current_pose.theta) * dt

        # Normalise theta to [-pi, pi]
        if self.current_pose.theta > math.pi:
            self.current_pose.theta -= 2 * math.pi
        elif self.current_pose.theta < -math.pi:
            self.current_pose.theta += 2 * math.pi

        # Store LIDAR scan for later use
        if lidar_points:
            self.last_lidar_scan = lidar_points

        # Update wall mapper if we have LIDAR points
        if lidar_points and self.wall_mapper:
            self.wall_mapper.update(self.current_pose.x, self.current_pose.y,
                                    self.current_pose.theta, lidar_points)

        # Apply map‑based pose correction if map is ready and enabled
        if self.use_map_correction and self.wall_mapper.is_mapped and lidar_points:
            map_pose = self.wall_mapper.get_pose_from_walls(lidar_points)
            # If map_pose is not (0,0,0) and is reasonable, correct the pose
            if map_pose != (0.0, 0.0, 0.0):
                # Simple correction: blend odometry with map pose (weighted average)
                # Weight: 0.6 odometry, 0.4 map (tunable)
                alpha = 0.4  # map weight
                self.current_pose.x = (1 - alpha) * self.current_pose.x + alpha * map_pose[0]
                self.current_pose.y = (1 - alpha) * self.current_pose.y + alpha * map_pose[1]
                # Theta correction: handle angle wrapping
                theta_diff = map_pose[2] - self.current_pose.theta
                if theta_diff > math.pi:
                    theta_diff -= 2 * math.pi
                elif theta_diff < -math.pi:
                    theta_diff += 2 * math.pi
                self.current_pose.theta += alpha * theta_diff
                # Normalise again
                if self.current_pose.theta > math.pi:
                    self.current_pose.theta -= 2 * math.pi
                elif self.current_pose.theta < -math.pi:
                    self.current_pose.theta += 2 * math.pi

                logger.debug(f"Pose corrected by map: x={self.current_pose.x:.3f}, y={self.current_pose.y:.3f}")

        self.last_update_time = current_time

    def get_pose(self) -> Pose2D:
        """Return the current estimated pose."""
        return self.current_pose

    def get_start_pose(self) -> Optional[Pose2D]:
        """Return the saved starting pose."""
        return self.initial_pose

    def get_map_pose(self) -> Optional[Pose2D]:
        """
        Use WallMapper to get pose from wall distances, if available.

        Returns:
            Pose2D if map is ready and scan exists, else None.
        """
        if not self.wall_mapper.is_mapped:
            return None
        if not self.last_lidar_scan:
            return None
        x, y, theta = self.wall_mapper.get_pose_from_walls(self.last_lidar_scan)
        if x == 0.0 and y == 0.0 and theta == 0.0:
            return None
        return Pose2D(x, y, theta)

    def is_map_ready(self) -> bool:
        """Check if the wall map has been built."""
        return self.wall_mapper.is_mapped

    def save_map(self, filename: str) -> None:
        """Convenience method to save the map."""
        self.wall_mapper.save_map(filename)

    def load_map(self, filename: str) -> None:
        """Convenience method to load a saved map."""
        self.wall_mapper.load_map(filename)
