"""
WallMapper builds a 2D occupancy grid map from LIDAR data during the first lap.
After the map is built, it provides pose estimation by matching current wall distances
to the known map (rectangular track).

The LIDAR is assumed to be mounted on top of the car, scanning 360°.
The angles provided by the LIDAR are relative to the robot's forward heading.
"""

import math
import logging
from typing import List, Tuple, Optional
from dataclasses import dataclass

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class MapCell:
    """A single cell in the occupancy grid."""
    occupied: bool = False
    count: int = 0


class WallMapper:
    def __init__(self, resolution: float = 0.02, map_size: int = 400,
                 update_threshold: int = 50):
        """
        Initialize the wall mapper.

        Args:
            resolution: Size of each grid cell in meters.
            map_size: Number of cells per side (grid is square).
            update_threshold: Number of LIDAR updates required before extracting walls.
        """
        self.resolution = resolution
        self.map_size = map_size
        self.update_threshold = update_threshold

        # 2D grid of cells (row-major order)
        self.grid = [[MapCell() for _ in range(map_size)] for _ in range(map_size)]

        # Map origin in world coordinates (bottom-left corner)
        self.origin_x = -map_size * resolution / 2.0
        self.origin_y = -map_size * resolution / 2.0

        # Flag indicating whether the map is built
        self.is_mapped = False

        # List of wall segments extracted from the map (each segment: (x1,y1,x2,y2))
        self.wall_segments: List[Tuple[float, float, float, float]] = []

        # Counter for map building progress
        self.total_updates = 0

        # Store the last scan for pose estimation
        self.last_scan: List[Tuple[float, float]] = []

    def update(self, pose_x: float, pose_y: float, theta: float,
               lidar_points: List[Tuple[float, float]]) -> None:
        """
        Update the occupancy grid with a LIDAR scan.

        Args:
            pose_x, pose_y, theta: Robot pose in world coordinates (meters, radians).
            lidar_points: List of (angle_degrees, distance_meters) from the LIDAR.
                          Angles are relative to the robot's heading.
        """
        if not lidar_points:
            return

        self.last_scan = lidar_points
        self.total_updates += 1

        for ang_deg, dist in lidar_points:
            # Ignore invalid readings
            if dist < 0.05 or dist > 5.0:
                continue

            # Convert to world coordinates
            ang_rad = math.radians(ang_deg)
            wx = pose_x + dist * math.cos(theta + ang_rad)
            wy = pose_y + dist * math.sin(theta + ang_rad)

            # Convert to grid indices
            ix = int((wx - self.origin_x) / self.resolution)
            iy = int((wy - self.origin_y) / self.resolution)

            if 0 <= ix < self.map_size and 0 <= iy < self.map_size:
                self.grid[ix][iy].occupied = True
                self.grid[ix][iy].count += 1

        # If we have enough updates, extract wall segments and mark as mapped
        if not self.is_mapped and self.total_updates >= self.update_threshold:
            self._extract_wall_segments()
            self.is_mapped = True
            logger.info(f"Map built with {len(self.wall_segments)} wall segments.")

    def _extract_wall_segments(self) -> None:
        """
        Extract straight wall segments from the occupancy grid.
        Uses OpenCV's HoughLinesP if available; otherwise falls back to
        a simple rectangle approximation based on min/max occupied cells.
        """
        # Convert grid to a binary image (0=free, 255=occupied)
        img = np.zeros((self.map_size, self.map_size), dtype=np.uint8)
        for i in range(self.map_size):
            for j in range(self.map_size):
                if self.grid[i][j].occupied:
                    img[i, j] = 255

        try:
            import cv2
            # Use probabilistic Hough transform to find line segments
            lines = cv2.HoughLinesP(img, rho=1, theta=np.pi/180,
                                    threshold=30, minLineLength=50, maxLineGap=10)
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    # Convert pixel coordinates to world coordinates
                    wx1 = x1 * self.resolution + self.origin_x
                    wy1 = y1 * self.resolution + self.origin_y
                    wx2 = x2 * self.resolution + self.origin_x
                    wy2 = y2 * self.resolution + self.origin_y
                    self.wall_segments.append((wx1, wy1, wx2, wy2))
                return
        except ImportError:
            logger.warning("OpenCV not available; using rectangle approximation.")

        # Fallback: extract the bounding rectangle of all occupied cells
        xs, ys = [], []
        for i in range(self.map_size):
            for j in range(self.map_size):
                if self.grid[i][j].occupied:
                    xs.append(i * self.resolution + self.origin_x)
                    ys.append(j * self.resolution + self.origin_y)

        if not xs:
            logger.warning("No occupied cells found; cannot extract walls.")
            return

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        # Add four wall segments forming a rectangle (clockwise order)
        self.wall_segments = [
            (min_x, min_y, max_x, min_y),  # bottom wall
            (max_x, min_y, max_x, max_y),  # right wall
            (max_x, max_y, min_x, max_y),  # top wall
            (min_x, max_y, min_x, min_y)   # left wall
        ]

    def get_pose_from_walls(self, lidar_scan: Optional[List[Tuple[float, float]]] = None) -> Tuple[float, float, float]:
        """
        Estimate the robot's pose using distances to the four walls of the track.

        This uses the known rectangular track dimensions (from the wall segments)
        and the distances to the front, left, right, and rear walls measured
        by the LIDAR. Returns (x, y, theta) in world coordinates.

        Args:
            lidar_scan: List of (angle_degrees, distance_meters) from the LIDAR.
                        If None, uses the last stored scan.

        Returns:
            (x, y, theta) tuple. If map is not ready or distances are missing,
            returns (0,0,0) and logs a warning.
        """
        if not self.is_mapped or len(self.wall_segments) < 4:
            logger.warning("Map not ready; returning (0,0,0)")
            return (0.0, 0.0, 0.0)

        if lidar_scan is None:
            lidar_scan = self.last_scan

        if not lidar_scan:
            logger.warning("No LIDAR scan available; returning (0,0,0)")
            return (0.0, 0.0, 0.0)

        # Extract track bounding box from wall segments
        xs = [p[0] for seg in self.wall_segments for p in [(seg[0], seg[1]), (seg[2], seg[3])]]
        ys = [p[1] for seg in self.wall_segments for p in [(seg[0], seg[1]), (seg[2], seg[3])]]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)

        # Get distances to front, left, right, rear walls from LIDAR
        front = None
        left = None
        right = None
        rear = None

        for ang_deg, dist in lidar_scan:
            if dist < 0.05 or dist > 5.0:
                continue
            if abs(ang_deg) < 5:  # front
                if front is None or dist < front:
                    front = dist
            elif abs(ang_deg - 90) < 5:  # left
                if left is None or dist < left:
                    left = dist
            elif abs(ang_deg + 90) < 5:  # right
                if right is None or dist < right:
                    right = dist
            elif abs(abs(ang_deg) - 180) < 5:  # rear
                if rear is None or dist < rear:
                    rear = dist

        if None in (front, left, right, rear):
            logger.warning("Incomplete wall distances; using odometry.")
            return (0.0, 0.0, 0.0)

        # Compute robot position from wall distances and known track rectangle
        # x = (x_min + x_max - left + right) / 2
        # y = (y_min + y_max - rear + front) / 2
        robot_x = (x_min + x_max - left + right) / 2.0
        robot_y = (y_min + y_max - rear + front) / 2.0

        # Estimate heading (theta) from side wall distances
        track_width = x_max - x_min
        if track_width > 0.1:
            # Approximate theta = atan2(left - right, track_width * 0.5)
            theta = math.atan2(left - right, track_width * 0.5)
            # Clip to a reasonable range (±45°)
            theta = max(-math.pi/4, min(math.pi/4, theta))
        else:
            theta = 0.0  # fallback

        logger.debug(f"Pose from walls: x={robot_x:.3f}, y={robot_y:.3f}, theta={math.degrees(theta):.1f}°")
        return (robot_x, robot_y, theta)

    def save_map(self, filename: str) -> None:
        """
        Save the occupancy grid and wall segments to a file (using pickle).

        Args:
            filename: Path to the file where the map will be saved.
        """
        import pickle
        with open(filename, 'wb') as f:
            pickle.dump((self.grid, self.wall_segments), f)
        logger.info(f"Map saved to {filename}")

    def load_map(self, filename: str) -> None:
        """
        Load a previously saved map from a file.

        Args:
            filename: Path to the file containing the saved map.
        """
        import pickle
        with open(filename, 'rb') as f:
            self.grid, self.wall_segments = pickle.load(f)
        self.is_mapped = True
        logger.info(f"Map loaded from {filename}")

    # ------------------------------
    # NEW METHOD: clear_map()
    # ------------------------------
    def clear_map(self) -> None:
        """
        Reset the occupancy grid, wall segments, and all internal state.
        Used when `force_rebuild` is True to erase any existing map.
        """
        self.grid = [[MapCell() for _ in range(self.map_size)] for _ in range(self.map_size)]
        self.wall_segments = []
        self.is_mapped = False
        self.total_updates = 0
        self.last_scan = []
        logger.info("WallMapper: Map cleared for forced rebuild.")
