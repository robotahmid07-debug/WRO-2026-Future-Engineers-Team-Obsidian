"""
LIDAR Fusion – Continuous Serial Acquisition (no rplidar library)
Reads RPLIDAR A1M8 data continuously, maintains a 360° buffer.
Thread-safe, with filters, median smoothing, and temporal filtering.

Architecture:
    LiDAR thread (continuous)
        ↓
    shared 360° buffer
        ↓
    50 Hz control loop
        ↓
    read latest geometry

Filters:
    1. Invalid distance rejection (< 0.01m or > 8m)
    2. Temporal filtering (ignore single bad points)
    3. Median filtering (3‑sample)
    4. Diagonal pillar mask (45°, 135°, 225°, 315° within 12 cm)
    5. Global minimum range (toggleable)
"""

import threading
import logging
import time
import serial
from typing import Dict, Optional, List
from collections import deque
import math

logger = logging.getLogger(__name__)


class LidarFusion:
    # ---- Diagonal pillar mask (always active) ----
    PILLAR_ANGLES = [45.0, 135.0, 225.0, 315.0]   # degrees
    PILLAR_TOLERANCE_DEG = 6.0
    PILLAR_DISTANCE_THRESHOLD_M = 0.12

    # ---- Global minimum range (toggleable) ----
    GLOBAL_MIN_RANGE_M = 0.08
    global_min_range_enabled = False

    # ---- Temporal filtering ----
    TEMPORAL_FRAMES = 3   # require same angle to be valid for N consecutive frames

    def __init__(self, port: str = '/dev/rplidar', baudrate: int = 115200,
                 median_filter_size: int = 3):
        self.port = port
        self.baudrate = baudrate
        self.median_filter_size = median_filter_size
        self.ser: Optional[serial.Serial] = None
        self.running = False
        self.read_thread = None

        # ---- 360° buffer (index = angle 0..359) ----
        self.scan_buffer: List[float] = [0.0] * 360   # distance in meters
        self.buffer_lock = threading.Lock()

        # ---- History for median filtering ----
        self.scan_history: Dict[int, deque] = {}   # angle -> deque of distances

        # ---- Temporal history (last N frames per angle) ----
        self.temporal_history: Dict[int, deque] = {}   # angle -> deque of distances

        # ---- Packet sync state ----
        self.packet_count = 0
        self.last_scan_time = 0.0

    def set_global_min_range(self, enabled: bool):
        self.global_min_range_enabled = enabled
        logger.info(f"Global min range {'enabled' if enabled else 'disabled'}")

    def open(self):
        """Open serial port and start background reader thread."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.5)
            self.running = True
            self.read_thread = threading.Thread(target=self._scan_loop, daemon=True)
            self.read_thread.start()
            logger.info(f"LIDAR opened on {self.port} at {self.baudrate} baud")
        except Exception as e:
            logger.error(f"Failed to open LIDAR: {e}")
            raise

    def close(self):
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        logger.info("LIDAR closed")

    def _send_command(self, cmd: bytes):
        """Send a command to the LIDAR."""
        if self.ser:
            self.ser.write(cmd)
            self.ser.flush()

    def _scan_loop(self):
        """
        Background thread: continuously read 5‑byte packets and maintain a 360° buffer.
        Uses sync‑byte (0xA5) alignment.
        """
        try:
            # Stop, reset, start scan
            self._send_command(b'\xA5\x25')   # stop
            time.sleep(0.3)
            self._send_command(b'\xA5\x40')   # reset
            time.sleep(1.0)
            self._send_command(b'\xA5\x20')   # start scan
            time.sleep(0.3)

            # Flush any pending data (firmware info, health packet)
            if self.ser and self.ser.is_open:
                self.ser.reset_input_buffer()

            self.last_scan_time = time.time()

            # ---- Continuous packet reader ----
            while self.running and self.ser and self.ser.is_open:
                # Find sync byte 0xA5
                byte = self.ser.read(1)
                if not byte:
                    continue
                if byte[0] != 0xA5:
                    continue

                # Read remaining 4 bytes
                packet = self.ser.read(4)
                if len(packet) < 4:
                    continue

                # Parse
                angle_low = packet[0]
                angle_high = packet[1]
                dist_low = packet[2]
                dist_high = packet[3]

                # Decode angle (0.1° per bit? Actually 1/64° per bit)
                angle_deg = (angle_high << 8 | angle_low) / 64.0
                distance_mm = (dist_high << 8 | dist_low) / 4.0
                distance_m = distance_mm / 1000.0

                # Reject invalid distances
                if distance_mm <= 0 or distance_mm > 8000:
                    continue

                # Apply masks
                if self._is_masked_point(angle_deg, distance_m):
                    continue

                # Apply temporal filtering
                if not self._temporal_filter(angle_deg, distance_m):
                    continue

                # Apply median filter
                filtered_dist = self._apply_median_filter(angle_deg, distance_m)

                # Store in 360° buffer (index = rounded angle)
                angle_idx = int(round(angle_deg)) % 360
                with self.buffer_lock:
                    self.scan_buffer[angle_idx] = filtered_dist

                self.packet_count += 1
                self.last_scan_time = time.time()

        except Exception as e:
            logger.error(f"LIDAR scan loop error: {e}")
            self.running = False

    def _is_masked_point(self, angle: float, distance: float) -> bool:
        """Return True if point should be ignored."""
        if self.global_min_range_enabled and distance < self.GLOBAL_MIN_RANGE_M:
            return True

        if distance >= self.PILLAR_DISTANCE_THRESHOLD_M:
            return False

        for pa in self.PILLAR_ANGLES:
            diff = abs(angle - pa)
            if diff > 180.0:
                diff = 360.0 - diff
            if diff <= self.PILLAR_TOLERANCE_DEG:
                return True
        return False

    def _temporal_filter(self, angle: float, distance: float) -> bool:
        """
        Temporal filtering: require the same angle to have a valid distance
        for N consecutive frames (ignores single bad points).
        """
        key = int(round(angle)) % 360

        if key not in self.temporal_history:
            self.temporal_history[key] = deque(maxlen=self.TEMPORAL_FRAMES)

        self.temporal_history[key].append(distance)

        # If we have enough frames, check if ALL are valid (> 0.01m)
        if len(self.temporal_history[key]) >= self.TEMPORAL_FRAMES:
            valid_count = sum(1 for d in self.temporal_history[key] if d > 0.01)
            if valid_count < self.TEMPORAL_FRAMES:
                return False   # not enough valid frames

        # Also reject if current distance is an outlier (e.g., >3x median)
        if len(self.temporal_history[key]) >= 3:
            sorted_vals = sorted(self.temporal_history[key])
            median = sorted_vals[len(sorted_vals) // 2]
            if distance > median * 3.0 and median > 0.01:
                return False   # outlier spike

        return True

    def _apply_median_filter(self, angle: float, raw_dist: float) -> float:
        """Apply 3‑sample median filter."""
        key = int(round(angle)) % 360
        if key not in self.scan_history:
            self.scan_history[key] = deque(maxlen=self.median_filter_size)
        self.scan_history[key].append(raw_dist)

        if len(self.scan_history[key]) >= self.median_filter_size:
            sorted_vals = sorted(self.scan_history[key])
            return sorted_vals[len(sorted_vals) // 2]
        return raw_dist

    def _interpolate_angle(self, angle_deg: float) -> float:
        """
        Get distance at a specific angle with interpolation between neighbouring
        valid measurements.
        """
        idx = int(round(angle_deg)) % 360
        with self.buffer_lock:
            # If exact angle has a valid reading, return it
            if self.scan_buffer[idx] > 0.01:
                return self.scan_buffer[idx]

            # Otherwise, look for nearest valid neighbours
            for offset in range(1, 5):
                left_idx = (idx - offset) % 360
                right_idx = (idx + offset) % 360
                left_val = self.scan_buffer[left_idx]
                right_val = self.scan_buffer[right_idx]

                if left_val > 0.01:
                    return left_val
                if right_val > 0.01:
                    return right_val

        return None

    # ==========================================================
    # Public interface – compatible with existing code
    # ==========================================================

    def get_scan_snapshot(self) -> Dict[float, float]:
        """
        Return a dictionary of angle → distance for all valid points.
        This maintains compatibility with existing code.
        """
        result = {}
        with self.buffer_lock:
            for idx, dist in enumerate(self.scan_buffer):
                if dist > 0.01:   # valid
                    result[float(idx)] = dist
        return result

    def get_front_distances(self, half_width_deg: float = 30.0) -> List[float]:
        """Return distances in the front sector."""
        result = []
        with self.buffer_lock:
            for idx, dist in enumerate(self.scan_buffer):
                # Normalize angle to [-180, 180]
                angle = idx
                if angle > 180:
                    angle = angle - 360
                if abs(angle) < half_width_deg and dist > 0.01:
                    result.append(dist)
        return result

    def get_range_at_angle(self, angle_deg: float, tolerance: float = 2.0) -> Optional[float]:
        """Get distance at a specific angle."""
        # Normalize angle to 0-359
        angle_idx = int(round(angle_deg)) % 360

        # Check if exact angle exists
        with self.buffer_lock:
            if self.scan_buffer[angle_idx] > 0.01:
                return self.scan_buffer[angle_idx]

            # Search within tolerance
            for offset in range(1, int(tolerance) + 1):
                left_idx = (angle_idx - offset) % 360
                right_idx = (angle_idx + offset) % 360

                if self.scan_buffer[left_idx] > 0.01:
                    return self.scan_buffer[left_idx]
                if self.scan_buffer[right_idx] > 0.01:
                    return self.scan_buffer[right_idx]

        return None

    def get_range_in_sector(self, center_angle_deg: float, half_width_deg: float) -> Optional[float]:
        """Get minimum distance in a sector."""
        min_dist = float('inf')
        center_idx = int(round(center_angle_deg)) % 360
        half = int(half_width_deg)

        with self.buffer_lock:
            for offset in range(-half, half + 1):
                idx = (center_idx + offset) % 360
                dist = self.scan_buffer[idx]
                if dist > 0.01 and dist < min_dist:
                    min_dist = dist

        return min_dist if min_dist != float('inf') else None

    def get_raw_buffer(self) -> List[float]:
        """Return the raw 360° buffer (for debugging)."""
        with self.buffer_lock:
            return self.scan_buffer.copy()

    def get_buffer_age(self) -> float:
        """Return the time since the last packet was received."""
        return time.time() - self.last_scan_time

    def is_data_fresh(self, max_age: float = 0.5) -> bool:
        """Check if the LIDAR data is fresh."""
        return self.get_buffer_age() < max_age
