"""
Reads RPLIDAR C1 data via USB and provides range at a given angle.
Implements sector matching.
"""

import math
import threading
import time
import logging
from typing import Optional, Tuple

# Use rplidar library (install via pip)
from rplidar import RPLidar

logger = logging.getLogger(__name__)


class LidarFusion:
    def __init__(self, port: str = '/dev/ttyUSB1', baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.lidar: Optional[RPLidar] = None
        self.running = False
        self.scan_data = {}  # angle -> distance (m)
        self.lock = threading.Lock()
        self.read_thread = None

    def open(self):
        try:
            self.lidar = RPLidar(self.port, baudrate=self.baudrate)
            self.running = True
            self.read_thread = threading.Thread(target=self._scan_loop, daemon=True)
            self.read_thread.start()
            logger.info(f"LIDAR opened on {self.port}")
        except Exception as e:
            logger.error(f"Failed to open LIDAR: {e}")
            raise

    def close(self):
        self.running = False
        if self.lidar:
            self.lidar.stop()
            self.lidar.disconnect()
        if self.read_thread:
            self.read_thread.join(timeout=1.0)

    def _scan_loop(self):
        # Use iterator
        try:
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                with self.lock:
                    self.scan_data.clear()
                    for _, angle, distance in scan:
                        # angle in degrees, distance in mm
                        if distance > 0:
                            self.scan_data[angle] = distance / 1000.0  # to meters
                # We don't need to sleep because iter_scans is blocking
        except Exception as e:
            logger.error(f"LIDAR scan error: {e}")

    def get_range_at_angle(self, angle_deg: float, tolerance: float = 2.0) -> Optional[float]:
        """
        Returns the range (in meters) at the nearest angle within +/- tolerance degrees.
        """
        with self.lock:
            if not self.scan_data:
                return None
            # Find closest angle
            best_angle = None
            best_diff = float('inf')
            for ang in self.scan_data.keys():
                diff = abs(ang - angle_deg)
                # Handle wrap-around for angles near 360/0
                if diff > 180:
                    diff = 360 - diff
                if diff < best_diff:
                    best_diff = diff
                    best_angle = ang
            if best_angle is not None and best_diff <= tolerance:
                return self.scan_data[best_angle]
            return None

    def get_range_in_sector(self, center_angle_deg: float, half_width_deg: float) -> Optional[float]:
        """
        Returns the minimum range in a sector centered at center_angle_deg with half-width.
        Used for spatial fusion.
        """
        min_range = float('inf')
        found = False
        with self.lock:
            for ang, dist in self.scan_data.items():
                diff = abs(ang - center_angle_deg)
                if diff > 180:
                    diff = 360 - diff
                if diff <= half_width_deg:
                    if dist < min_range:
                        min_range = dist
                        found = True
        return min_range if found else None
