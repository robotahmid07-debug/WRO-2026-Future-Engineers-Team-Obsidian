"""
Reads RPLIDAR A1 data via USB and provides range at a given angle.
Implements sector matching. Thread‑safe scan data access.
Adds 3‑sample median filtering to eliminate laser dropouts.
Provides front distance extraction for curvature estimation.

Globally masks out points from car's own structural pillars:
  - Angles: 45°, 135°, 225°, 315°
  - Distance threshold: 8 cm
  - Tolerance: ±2°
"""

import threading
import logging
from typing import Dict, List, Optional
from collections import deque

from rplidar import RPLidar

logger = logging.getLogger(__name__)


class LidarFusion:
    def __init__(self, port: str = '/dev/ttyUSB1', baudrate: int = 460800,
                 median_filter_size: int = 3):
        """
        Initialize LIDAR fusion.

        Args:
            port: Serial port of the RPLIDAR (e.g., '/dev/ttyUSB1').
            baudrate: Communication speed. RPLIDAR A1 uses 460800.
            median_filter_size: Size of median filter for each sector (default 3).
        """
        self.port = port
        self.baudrate = baudrate
        self.median_filter_size = median_filter_size
        self.lidar: Optional[RPLidar] = None
        self.running = False
        # Store raw scan data: angle (deg) -> distance (m)
        self.scan_data: Dict[float, float] = {}
        # Store history for median filtering: angle (rounded) -> deque of distances
        self.scan_history: Dict[float, deque] = {}
        self.lock = threading.Lock()
        self.read_thread = None

        # ---- Pillar mask configuration ----
        self.pillar_angles = [45.0, 135.0, 225.0, 315.0]   # degrees
        self.pillar_tolerance_deg = 2.0                     # ±2°
        self.pillar_distance_threshold_m = 0.08             # 8 cm

    def open(self):
        """Open the LIDAR connection and start the background scanning thread."""
        try:
            self.lidar = RPLidar(self.port, baudrate=self.baudrate)
            self.running = True
            self.read_thread = threading.Thread(target=self._scan_loop, daemon=True)
            self.read_thread.start()
            logger.info(f"LIDAR opened on {self.port} at {self.baudrate} baud")
        except Exception as e:
            logger.error(f"Failed to open LIDAR: {e}")
            raise

    def close(self):
        """Stop the scanning thread and disconnect the LIDAR."""
        self.running = False
        if self.lidar:
            self.lidar.stop()
            self.lidar.disconnect()
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        logger.info("LIDAR closed")

    def _scan_loop(self):
        """Background thread: continuously fetch LIDAR scans and store the latest."""
        try:
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                with self.lock:
                    self.scan_data.clear()
                    for _, angle, distance in scan:
                        if distance > 0:  # ignore invalid readings
                            self.scan_data[angle] = distance / 1000.0  # mm → m
        except Exception as e:
            logger.error(f"LIDAR scan error: {e}")

    def _apply_median_filter(self, angle: float, raw_dist: float) -> float:
        """
        Apply 3‑sample median filter to smooth raw LIDAR distances.
        Uses rounded angle as key to accumulate samples.
        """
        key = round(angle)
        if key not in self.scan_history:
            self.scan_history[key] = deque(maxlen=self.median_filter_size)
        self.scan_history[key].append(raw_dist)

        if len(self.scan_history[key]) >= self.median_filter_size:
            sorted_vals = sorted(self.scan_history[key])
            return sorted_vals[len(sorted_vals) // 2]
        else:
            return raw_dist

    def _is_pillar_point(self, angle: float, distance: float) -> bool:
        """
        Check if a point is from the car's own structural pillars.
        Returns True if it should be masked out.
        """
        if distance >= self.pillar_distance_threshold_m:
            return False

        # Check if angle falls within tolerance of any pillar angle
        for pa in self.pillar_angles:
            diff = abs(angle - pa)
            if diff > 180.0:
                diff = 360.0 - diff
            if diff <= self.pillar_tolerance_deg:
                return True
        return False

    def get_scan_snapshot(self) -> Dict[float, float]:
        """
        Return a thread‑safe copy of the latest LIDAR scan data,
        with median filtering applied and car‑structure pillar mask applied.
        """
        with self.lock:
            filtered = {}
            for angle, dist in self.scan_data.items():
                # ---- Pillar mask ----
                if self._is_pillar_point(angle, dist):
                    # Skip this point entirely – it's from the car itself
                    continue

                # Apply median filter to the remaining points
                filtered[angle] = self._apply_median_filter(angle, dist)

            return filtered

    def get_front_distances(self, half_width_deg: float = 30.0) -> List[float]:
        """
        Return distances in the front sector (±half_width_deg) for curvature estimation.
        """
        scan = self.get_scan_snapshot()
        if not scan:
            return []
        front_dists = []
        for ang, dist in scan.items():
            if dist < 0.05 or dist > 5.0:
                continue
            if abs(ang) < half_width_deg:
                front_dists.append(dist)
        return front_dists

    def get_range_at_angle(self, angle_deg: float, tolerance: float = 2.0) -> Optional[float]:
        """
        Returns the range (in meters) at the nearest angle within +/- tolerance degrees.

        Args:
            angle_deg: Target angle in degrees (0 = front).
            tolerance: Maximum angular difference to accept.

        Returns:
            Range in meters, or None if no point within tolerance.
        """
        scan = self.get_scan_snapshot()
        if not scan:
            return None

        best_angle = None
        best_diff = float('inf')
        for ang in scan.keys():
            diff = abs(ang - angle_deg)
            if diff > 180:
                diff = 360 - diff
            if diff < best_diff:
                best_diff = diff
                best_angle = ang

        if best_angle is not None and best_diff <= tolerance:
            return scan[best_angle]
        return None

    def get_range_in_sector(self, center_angle_deg: float, half_width_deg: float) -> Optional[float]:
        """
        Returns the minimum range in a sector centered at center_angle_deg with half-width.

        Args:
            center_angle_deg: Center angle of the sector (deg).
            half_width_deg: Half angular width of the sector (deg).

        Returns:
            Minimum range in meters, or None if no points in sector.
        """
        scan = self.get_scan_snapshot()
        if not scan:
            return None

        min_range = float('inf')
        for ang, dist in scan.items():
            diff = abs(ang - center_angle_deg)
            if diff > 180:
                diff = 360 - diff
            if diff <= half_width_deg:
                if dist < min_range:
                    min_range = dist

        return min_range if min_range != float('inf') else None
