"""
WallMapper builds a 2D occupancy grid map from LIDAR data during the first lap.
After the map is built, it provides pose estimation by matching current wall distances
to the known map (rectangular track) using the robot's heading.

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
        self.resolution = resolution
        self.map_size = map_size
        self.update_threshold = update_threshold

        self.grid = [[MapCell() for _ in range(map_size)] for _ in range(map_size)]
        self.origin_x = -map_size * resolution / 2.0
        self.origin_y = -map_size * resolution / 2.0

        self.is_mapped = False
        self.wall_segments: List[Tuple[float, float, float, float]] = []
        self.total_updates = 0
        self.last_scan: List[Tuple[float, float]] = []

        self.x_min = 0.0
        self.x_max = 0.0
        self.y_min = 0.0
        self.y_max = 0.0

    def update(self, pose_x: float, pose_y: float, theta: float,
               lidar_points: List[Tuple[float, float]]) -> None:
        if not lidar_points:
            return

        self.last_scan = lidar_points
        self.total_updates += 1

        for ang_deg, dist in lidar_points:
            if dist < 0.05 or dist > 5.0:
                continue
            ang_rad = math.radians(ang_deg)
            wx = pose_x + dist * math.cos(theta + ang_rad)
            wy = pose_y + dist * math.sin(theta + ang_rad)

            ix = int((wx - self.origin_x) / self.resolution)
            iy = int((wy - self.origin_y) / self.resolution)

            if 0 <= ix < self.map_size and 0 <= iy < self.map_size:
                self.grid[ix][iy].occupied = True
                self.grid[ix][iy].count += 1

        if not self.is_mapped and self.total_updates >= self.update_threshold:
            self._extract_wall_segments()
            self.is_mapped = True
            xs = [p[0] for seg in self.wall_segments for p in [(seg[0], seg[1]), (seg[2], seg[3])]]
            ys = [p[1] for seg in self.wall_segments for p in [(seg[0], seg[1]), (seg[2], seg[3])]]
            if xs and ys:
                self.x_min, self.x_max = min(xs), max(xs)
                self.y_min, self.y_max = min(ys), max(ys)
                logger.info(f"Map built with bounding box: x=[{self.x_min:.2f}, {self.x_max:.2f}], "
                            f"y=[{self.y_min:.2f}, {self.y_max:.2f}]")

    def _extract_wall_segments(self) -> None:
        img = np.zeros((self.map_size, self.map_size), dtype=np.uint8)
        for i in range(self.map_size):
            for j in range(self.map_size):
                if self.grid[i][j].occupied:
                    img[i, j] = 255

        try:
            import cv2
            lines = cv2.HoughLinesP(img, rho=1, theta=np.pi/180,
                                    threshold=30, minLineLength=50, maxLineGap=10)
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    wx1 = x1 * self.resolution + self.origin_x
                    wy1 = y1 * self.resolution + self.origin_y
                    wx2 = x2 * self.resolution + self.origin_x
                    wy2 = y2 * self.resolution + self.origin_y
                    self.wall_segments.append((wx1, wy1, wx2, wy2))
                return
        except ImportError:
            logger.warning("OpenCV not available; using rectangle approximation.")

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

        self.wall_segments = [
            (min_x, min_y, max_x, min_y),
            (max_x, min_y, max_x, max_y),
            (max_x, max_y, min_x, max_y),
            (min_x, max_y, min_x, min_y)
        ]

    def _snap_to_cardinal(self, theta: float) -> int:
        theta = theta % (2 * math.pi)
        deg = math.degrees(theta)
        snapped_deg = round(deg / 90.0) * 90
        snapped_deg = snapped_deg % 360
        return int(snapped_deg // 90)

    def get_pose_from_walls(self, lidar_scan: Optional[List[Tuple[float, float]]] = None,
                            robot_theta: float = 0.0) -> Tuple[float, float, float]:
        if not self.is_mapped or len(self.wall_segments) < 4:
            logger.warning("Map not ready; returning (0,0,0)")
            return (0.0, 0.0, 0.0)

        if lidar_scan is None:
            lidar_scan = self.last_scan

        if not lidar_scan:
            logger.warning("No LIDAR scan available; returning (0,0,0)")
            return (0.0, 0.0, 0.0)

        x_min, x_max = self.x_min, self.x_max
        y_min, y_max = self.y_min, self.y_max

        front = None
        left = None
        right = None
        rear = None

        for ang_deg, dist in lidar_scan:
            if dist < 0.05 or dist > 5.0:
                continue
            if abs(ang_deg) < 5:
                if front is None or dist < front:
                    front = dist
            elif abs(ang_deg - 90) < 5:
                if left is None or dist < left:
                    left = dist
            elif abs(ang_deg + 90) < 5:
                if right is None or dist < right:
                    right = dist
            elif abs(abs(ang_deg) - 180) < 5:
                if rear is None or dist < rear:
                    rear = dist

        if None in (front, left, right, rear):
            logger.warning("Incomplete wall distances; using odometry.")
            return (0.0, 0.0, 0.0)

        cardinal = self._snap_to_cardinal(robot_theta)

        if cardinal == 0:   # facing +x
            robot_x = (x_min + x_max - rear + front) / 2.0
            robot_y = (y_min + y_max - right + left) / 2.0
        elif cardinal == 1:   # facing +y
            robot_x = (x_min + x_max + left - right) / 2.0
            robot_y = (y_min + y_max - rear + front) / 2.0
        elif cardinal == 2:   # facing -x
            robot_x = (x_min + x_max - front + rear) / 2.0
            robot_y = (y_min + y_max - left + right) / 2.0
        else:   # cardinal == 3, facing -y
            robot_x = (x_min + x_max - left + right) / 2.0
            robot_y = (y_min + y_max - front + rear) / 2.0

        theta_est = robot_theta

        logger.debug(
            f"Pose from walls (cardinal {cardinal*90}°): x={robot_x:.3f}, y={robot_y:.3f}, "
            f"theta={math.degrees(theta_est):.1f}°"
        )
        return (robot_x, robot_y, theta_est)

    def save_map(self, filename: str) -> None:
        import pickle
        with open(filename, 'wb') as f:
            pickle.dump((self.grid, self.wall_segments), f)
        logger.info(f"Map saved to {filename}")

    def load_map(self, filename: str) -> None:
        import pickle
        with open(filename, 'rb') as f:
            self.grid, self.wall_segments = pickle.load(f)
        self.is_mapped = True
        xs = [p[0] for seg in self.wall_segments for p in [(seg[0], seg[1]), (seg[2], seg[3])]]
        ys = [p[1] for seg in self.wall_segments for p in [(seg[0], seg[1]), (seg[2], seg[3])]]
        if xs and ys:
            self.x_min, self.x_max = min(xs), max(xs)
            self.y_min, self.y_max = min(ys), max(ys)
        logger.info(f"Map loaded from {filename}")

    def clear_map(self) -> None:
        self.grid = [[MapCell() for _ in range(self.map_size)] for _ in range(self.map_size)]
        self.wall_segments = []
        self.is_mapped = False
        self.total_updates = 0
        self.last_scan = []
        self.x_min = self.x_max = self.y_min = self.y_max = 0.0
        logger.info("WallMapper: Map cleared for forced rebuild.")
