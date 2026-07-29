"""
Ackermann steering localization using the Bicycle Model.
Integrates WallMapper for map‑based pose correction.
Receives IMU yaw rate from ESP32 via serial bridge.
All calibratable parameters are read from the config.
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
    def __init__(self, lidar_fusion, config: dict, use_map_correction: bool = True):
        """
        Initialize localization with Ackermann bicycle model.

        Args:
            lidar_fusion: LidarFusion object (used for getting LIDAR scans).
            config: SystemConfig object (from config_parser).
            use_map_correction: Whether to use WallMapper for pose correction.
        """
        self.lidar = lidar_fusion
        self.config = config

        # Read wheelbase from config (with fallback)
        if hasattr(config, 'vehicle') and hasattr(config.vehicle, 'wheelbase_m'):
            self.wheelbase = config.vehicle.wheelbase_m
        else:
            logger.warning("wheelbase_m not found in config, using default 0.25m")
            self.wheelbase = 0.25

        # Current and initial poses
        self.current_pose = Pose2D(0.0, 0.0, 0.0)
        self.initial_pose = Pose2D(0.0, 0.0, 0.0)

        # Parking bay geometry (for obstacle challenge)
        self.parking_bay_geometry: Optional[dict] = None

        # Time tracking for odometry integration
        self.last_update_time = time.time()

        # WallMapper for map building and pose correction
        self.wall_mapper = WallMapper(resolution=0.02, map_size=400, update_threshold=30)

        # Flag to enable/disable map correction (controlled by state machine)
        self.use_map_correction = use_map_correction

        # Store last LIDAR scan for pose correction
        self.last_lidar_scan: List[Tuple[float, float]] = []

        # ---- IMU fusion parameters ----
        # Latest yaw rate received from ESP32 (rad/s)
        self.latest_imu_yaw_rate: Optional[float] = None
        # Complementary filter weight: 0.8 = 80% IMU, 20% kinematic
        self.imu_alpha = 0.8
        # Low‑pass filter for IMU to reduce noise (optional)
        self.imu_filtered = 0.0
        self.imu_filter_alpha = 0.3

        # Flag indicating whether IMU data has been received at least once
        self.imu_available = False   # <-- ADDED

    # ==========================================================
    # IMU Data Update (called from state_machine)
    # ==========================================================

    def update_imu_data(self, yaw_rate: float) -> None:
        """
        Store the latest IMU yaw rate received from ESP32.
        Applies a simple low‑pass filter to reduce noise.
        """
        # Low‑pass filter: smooth the IMU reading
        if self.latest_imu_yaw_rate is not None:
            self.imu_filtered = (self.imu_filter_alpha * yaw_rate +
                                 (1 - self.imu_filter_alpha) * self.imu_filtered)
        else:
            self.imu_filtered = yaw_rate
        self.latest_imu_yaw_rate = self.imu_filtered
        self.imu_available = True   # <-- ADDED: mark IMU as active

    # ==========================================================
    # Pose Registration
    # ==========================================================

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

    # ==========================================================
    # Pose Update (Odometry + IMU Fusion + Map Correction)
    # ==========================================================

    def update_pose(self, linear_vel: float, steering_angle: float,
                    lidar_points: Optional[List[Tuple[float, float]]] = None) -> None:
        """
        Update the robot's pose using the Ackermann Bicycle Model.
        Fuses IMU yaw rate (if available) for heading.
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

        # Compute kinematic heading rate
        theta_dot_kinematic = (linear_vel / self.wheelbase) * math.tan(steering_angle)

        # ---- Fuse with IMU if available ----
        if self.latest_imu_yaw_rate is not None:
            # Complementary filter: alpha * IMU + (1-alpha) * kinematic
            alpha = self.imu_alpha
            theta_dot = alpha * self.latest_imu_yaw_rate + (1 - alpha) * theta_dot_kinematic
        else:
            theta_dot = theta_dot_kinematic

        # Update pose using the fused heading rate
        self.current_pose.theta += theta_dot * dt
        self.current_pose.x += linear_vel * math.cos(self.current_pose.theta) * dt
        self.current_pose.y += linear_vel * math.sin(self.current_pose.theta) * dt

        # Normalize theta to [-pi, pi]
        if self.current_pose.theta > math.pi:
            self.current_pose.theta -= 2 * math.pi
        elif self.current_pose.theta < -math.pi:
            self.current_pose.theta += 2 * math.pi

        # ---- LIDAR processing ----
        if lidar_points:
            self.last_lidar_scan = lidar_points

            # Update wall mapper for map building
            self.wall_mapper.update(self.current_pose.x, self.current_pose.y,
                                    self.current_pose.theta, lidar_points)

        # ---- Map correction (if enabled and map ready) ----
        if self.use_map_correction and self.wall_mapper.is_mapped and lidar_points:
            map_pose = self.wall_mapper.get_pose_from_walls(lidar_points)
            if map_pose != (0.0, 0.0, 0.0):
                # Blend odometry with map pose using a weighted average
                alpha = 0.4   # map weight (tunable)
                # Position correction
                self.current_pose.x = (1 - alpha) * self.current_pose.x + alpha * map_pose[0]
                self.current_pose.y = (1 - alpha) * self.current_pose.y + alpha * map_pose[1]
                # Heading correction (handle wrap‑around)
                theta_diff = map_pose[2] - self.current_pose.theta
                if theta_diff > math.pi:
                    theta_diff -= 2 * math.pi
                elif theta_diff < -math.pi:
                    theta_diff += 2 * math.pi
                self.current_pose.theta += alpha * theta_diff
                # Normalize again
                if self.current_pose.theta > math.pi:
                    self.current_pose.theta -= 2 * math.pi
                elif self.current_pose.theta < -math.pi:
                    self.current_pose.theta += 2 * math.pi

                logger.debug(f"Pose corrected by map: x={self.current_pose.x:.3f}, y={self.current_pose.y:.3f}")

        self.last_update_time = current_time

    # ==========================================================
    # Getters
    # ==========================================================

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

    # ==========================================================
    # Map Persistence (Save/Load/Clear)
    # ==========================================================

    def save_map(self, filename: str) -> None:
        """Convenience method to save the map."""
        self.wall_mapper.save_map(filename)

    def load_map(self, filename: str) -> None:
        """Convenience method to load a saved map."""
        self.wall_mapper.load_map(filename)

    def clear_map(self) -> None:
        """Convenience method to clear the map (force rebuild)."""
        self.wall_mapper.clear_map()
