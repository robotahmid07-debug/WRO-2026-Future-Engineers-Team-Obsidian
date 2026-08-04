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
        self.imu_available = False

        # Track the time of the last IMU update to detect staleness
        self.last_imu_time = 0.0

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
        self.imu_available = True
        self.last_imu_time = time.time()

    # ==========================================================
    # Pose Registration
    # ==========================================================

    def register_start_pose(self) -> None:
        """Register the starting pose. Defaults to (0,0,0) until map is built."""
        self.initial_pose = Pose2D(0.0, 0.0, 0.0)
        self.current_pose = Pose2D(0.0, 0.0, 0.0)
        self.last_update_time = time.time()
        logger.info("Start pose registered at (0, 0, 0)")

    # ==========================================================
    # PARKING DETECTION – ROBUST VERSION WITH BETTER FILTERING
    # ==========================================================

    def scan_for_parking_markers(self) -> Optional[dict]:
        """
        Perform a single LIDAR scan and detect the two magenta parking-lot
        boundary markers (200x20x100mm each, WRO rule 13.25).

        Improvements:
          - Uses 2 neighbours on each side (instead of 1) to reduce noise.
          - Stricter spacing tolerance: max 0.8m (real markers are ~0.3-0.5m apart).
          - Cluster quality check: rejects elongated or sparse clusters.
          - Outlier rejection: removes points far from cluster centroid.

        Returns:
            Dictionary with 'x_min', 'x_max', 'y_min', 'y_max' in world coords,
            or None if markers were not found in this scan.
        """
        robot_pose = self.current_pose
        scan = self.lidar.get_scan_snapshot()  # Dict[angle_deg, distance_m]

        if not scan:
            logger.debug("scan_for_parking_markers: No LIDAR scan available")
            return None

        sorted_pts = sorted(scan.items())
        n = len(sorted_pts)

        # Need at least 5 points to have 2 neighbours on each side
        if n < 5:
            logger.debug("scan_for_parking_markers: Too few LIDAR points")
            return None

        candidates = []

        for i in range(2, n - 2):  # Start at index 2 to have 2 neighbours before
            ang, dist = sorted_pts[i]
            if dist <= 0.02 or dist > 2.5:
                continue

            # Check 2 neighbours on each side
            prev1_dist = sorted_pts[i - 1][1]
            prev2_dist = sorted_pts[i - 2][1]
            next1_dist = sorted_pts[i + 1][1]
            next2_dist = sorted_pts[i + 2][1]

            # Marker sticks ~20mm proud of flat wall. Check if ALL neighbours are farther.
            # Using 2 neighbours on each side reduces false positives from noise spikes.
            # Fixed E129: continuation line indented further
            if (prev1_dist - dist > 0.02 and prev2_dist - dist > 0.02
                    and next1_dist - dist > 0.02 and next2_dist - dist > 0.02):

                world_ang = robot_pose.theta + math.radians(ang)
                wx = robot_pose.x + dist * math.cos(world_ang)
                wy = robot_pose.y + dist * math.sin(world_ang)
                candidates.append((wx, wy, dist))

        if len(candidates) < 4:  # Need at least 4 points for 2 clusters
            logger.debug(f"scan_for_parking_markers: Only {len(candidates)} candidates found")
            return None

        # ---- Cluster the candidate points ----
        clusters: List[List[Tuple[float, float]]] = []
        for pt in candidates:
            wx, wy, _ = pt
            placed = False
            for cluster in clusters:
                cx = sum(p[0] for p in cluster) / len(cluster)
                cy = sum(p[1] for p in cluster) / len(cluster)
                if math.hypot(wx - cx, wy - cy) < 0.06:  # Slightly tighter clustering
                    cluster.append((wx, wy))
                    placed = True
                    break
            if not placed:
                clusters.append([(wx, wy)])

        # Filter clusters: must have at least 2 points (A1's lower density)
        clusters = [c for c in clusters if len(c) >= 2]

        if len(clusters) < 2:
            logger.debug(f"scan_for_parking_markers: {len(clusters)} clusters found (<2)")
            return None

        # ---- Cluster quality check ----
        # Reject clusters that are too elongated (should be roughly circular)
        valid_clusters = []
        for cluster in clusters:
            if len(cluster) < 2:
                continue
            # Check cluster compactness
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)
            max_dist = max(math.hypot(p[0] - cx, p[1] - cy) for p in cluster)
            # Reject if points are spread out more than 8cm (marker is 200mm wide)
            if max_dist < 0.08:
                valid_clusters.append(cluster)

        if len(valid_clusters) < 2:
            logger.debug("scan_for_parking_markers: Invalid cluster quality")
            return None

        # Sort by size (largest first)
        valid_clusters.sort(key=len, reverse=True)
        m1, m2 = valid_clusters[0], valid_clusters[1]

        # Compute cluster centroids
        c1 = (sum(p[0] for p in m1) / len(m1), sum(p[1] for p in m1) / len(m1))
        c2 = (sum(p[0] for p in m2) / len(m2), sum(p[1] for p in m2) / len(m2))

        spacing = math.hypot(c1[0] - c2[0], c1[1] - c2[1])

        # Strict spacing: 0.15m minimum, 0.8m maximum (real markers are ~0.3-0.5m)
        if not (0.15 < spacing < 0.8):
            logger.debug(f"scan_for_parking_markers: Invalid spacing {spacing:.2f}m")
            return None

        x_min, x_max = sorted([c1[0], c2[0]])
        y_min, y_max = sorted([c1[1], c2[1]])

        result = {
            'x_min': x_min, 'x_max': x_max,
            'y_min': y_min, 'y_max': y_max,
            'spacing': spacing,
        }

        logger.info(f"scan_for_parking_markers: Found markers at spacing {spacing:.3f}m")
        return result

    def register_parking_slot(self) -> None:
        """
        Attempt to detect parking markers with up to 5 retries (100ms apart).
        Stores result in self.parking_bay_geometry or leaves as None.
        """
        for attempt in range(5):
            geometry = self.scan_for_parking_markers()
            if geometry:
                # Remove 'spacing' from stored geometry (it's just for logging)
                if 'spacing' in geometry:
                    del geometry['spacing']
                self.parking_bay_geometry = geometry
                logger.info(
                    f"Parking bay geometry measured (attempt {attempt + 1}): {geometry}"
                )
                return
            time.sleep(0.10)

        logger.warning(
            "Parking bay markers not found after 5 LIDAR scans; "
            "parking_bay_geometry left as None."
        )
        self.parking_bay_geometry = None

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

        # ---- Fuse with IMU if available and fresh ----
        use_imu = False
        if self.latest_imu_yaw_rate is not None and self.imu_available:
            # Check if IMU data is stale (>1s)
            if current_time - self.last_imu_time < 1.0:
                use_imu = True

        if use_imu:
            # Complementary filter: alpha * IMU + (1-alpha) * kinematic
            alpha = self.imu_alpha
            theta_dot = alpha * self.latest_imu_yaw_rate + (1 - alpha) * theta_dot_kinematic
        else:
            theta_dot = theta_dot_kinematic
            if self.imu_available and current_time - self.last_imu_time > 1.0:
                logger.debug("IMU data stale, falling back to kinematic")

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
            # Pass current theta for heading‑aware pose correction
            map_pose = self.wall_mapper.get_pose_from_walls(lidar_points, self.current_pose.theta)
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
        # Pass current theta for heading‑aware pose correction
        x, y, theta = self.wall_mapper.get_pose_from_walls(self.last_lidar_scan, self.current_pose.theta)
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

    # ==========================================================
    # Reset
    # ==========================================================

    def reset(self) -> None:
        """
        Reset the localization module to initial state.
        Clears pose, IMU state, wall mapper, and timers.
        """
        self.current_pose = Pose2D(0.0, 0.0, 0.0)
        self.initial_pose = Pose2D(0.0, 0.0, 0.0)
        self.last_update_time = time.time()
        self.latest_imu_yaw_rate = None
        self.imu_available = False
        self.imu_filtered = 0.0
        self.last_imu_time = 0.0
        self.last_lidar_scan = []
        self.wall_mapper.clear_map()
        logger.info("Localization reset")
