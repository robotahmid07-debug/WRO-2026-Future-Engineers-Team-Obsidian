"""
LIDAR-based 4-Stage Parallel Parking.
Uses 360° LIDAR for precise distance measurements to rear and side walls.
"""
# Hi
import time
import logging
from enum import Enum
from typing import Optional, Dict, Any

logger = logging.getLogger(__name__)


class ParkingStage(Enum):
    IDLE = 0
    APPROACH = 1
    REVERSE_STEER = 2
    ALIGN_CENTER = 3
    FULL_STOP = 4
    COMPLETE = 5
    ABORTED = 6


class ParkingController:
    def __init__(self, localization, steering_controller, lidar_fusion,
                 config: Dict[str, Any]):
        self.localization = localization
        self.steering = steering_controller
        self.lidar = lidar_fusion
        self.config = config

        self.stage = ParkingStage.IDLE
        self.stage_start_time = 0.0
        self.stage_elapsed = 0.0

        self.parking_geometry: Optional[Dict[str, float]] = None
        self.target_pose = None

        # LIDAR-based parking targets
        self.rear_wall_target = 0.05   # 5 cm from rear wall
        self.side_wall_target = 0.10   # 10 cm from side wall
        self.alignment_tolerance = 0.02

        # PID gains for LIDAR-based alignment
        self.kp_rear = 0.5
        self.kp_side = 0.3

        # Stage durations (fallback if LIDAR fails)
        self.stage_durations = {
            ParkingStage.APPROACH: 2.0,
            ParkingStage.REVERSE_STEER: 2.5,
            ParkingStage.ALIGN_CENTER: 2.0,
            ParkingStage.FULL_STOP: 0.5,
        }

    def start(self, parking_geometry: Dict[str, float]):
        self.parking_geometry = parking_geometry
        self.stage = ParkingStage.APPROACH
        self.stage_start_time = time.time()
        self.stage_elapsed = 0.0
        logger.info("LIDAR-based parking STARTED")

    def abort(self):
        self.stage = ParkingStage.ABORTED
        self.steering.stop()
        logger.warning("Parking ABORTED")

    def update(self) -> ParkingStage:
        if self.stage == ParkingStage.IDLE:
            return self.stage
        if self.stage in (ParkingStage.COMPLETE, ParkingStage.ABORTED):
            return self.stage

        self.stage_elapsed = time.time() - self.stage_start_time

        if self.stage == ParkingStage.APPROACH:
            self._execute_approach()
        elif self.stage == ParkingStage.REVERSE_STEER:
            self._execute_reverse_steer()
        elif self.stage == ParkingStage.ALIGN_CENTER:
            self._execute_align_center()
        elif self.stage == ParkingStage.FULL_STOP:
            self._execute_full_stop()

        return self.stage

    def _get_rear_distance_from_lidar(self) -> Optional[float]:
        """Get distance to rear wall using LIDAR (180° / -180° sector)."""
        scan = self.lidar.get_scan_snapshot()
        if not scan:
            return None

        rear_dists = []
        for ang, dist in scan.items():
            if dist < 0.05 or dist > 5.0:
                continue
            if abs(abs(ang) - 180) < 15:  # rear sector (±15°)
                rear_dists.append(dist)

        if not rear_dists:
            return None
        return sum(rear_dists) / len(rear_dists)

    def _get_side_distance_from_lidar(self, side: str = 'left') -> Optional[float]:
        """Get distance to side wall using LIDAR (90° for left, -90° for right)."""
        scan = self.lidar.get_scan_snapshot()
        if not scan:
            return None

        target_angle = 90 if side == 'left' else -90
        side_dists = []
        for ang, dist in scan.items():
            if dist < 0.05 or dist > 5.0:
                continue
            if abs(ang - target_angle) < 15:
                side_dists.append(dist)

        if not side_dists:
            return None
        return sum(side_dists) / len(side_dists)

    def _execute_approach(self):
        """Stage 1: Approach parking bay using LIDAR rear distance."""
        rear_dist = self._get_rear_distance_from_lidar()

        if rear_dist is not None and rear_dist < self.rear_wall_target * 2:
            self._transition_to(ParkingStage.REVERSE_STEER)
            logger.info("Parking: APPROACH complete -> REVERSE_STEER")
            return

        self.steering.set_speed(0.10, 0.0)

        if self.stage_elapsed > self.stage_durations[ParkingStage.APPROACH]:
            self._transition_to(ParkingStage.REVERSE_STEER)

    def _execute_reverse_steer(self):
        """Stage 2: Reverse with steering into the bay."""
        rear_dist = self._get_rear_distance_from_lidar()

        if rear_dist is not None and rear_dist < self.rear_wall_target * 1.5:
            self._transition_to(ParkingStage.ALIGN_CENTER)
            logger.info("Parking: REVERSE_STEER complete -> ALIGN_CENTER")
            return

        left_dist = self._get_side_distance_from_lidar('left')
        right_dist = self._get_side_distance_from_lidar('right')

        steer_correction = 0.0
        if left_dist is not None and left_dist < self.side_wall_target:
            steer_correction = -0.3  # steer right
        elif right_dist is not None and right_dist < self.side_wall_target:
            steer_correction = 0.3   # steer left

        self.steering.set_speed(-0.12, 0.3 + steer_correction)

        if self.stage_elapsed > self.stage_durations[ParkingStage.REVERSE_STEER]:
            self._transition_to(ParkingStage.ALIGN_CENTER)

    def _execute_align_center(self):
        """Stage 3: Fine-tune position using LIDAR."""
        rear_dist = self._get_rear_distance_from_lidar()
        left_dist = self._get_side_distance_from_lidar('left')
        right_dist = self._get_side_distance_from_lidar('right')

        rear_error = (rear_dist - self.rear_wall_target) if rear_dist is not None else 0.0
        left_error = (left_dist - self.side_wall_target) if left_dist is not None else 0.0
        right_error = (right_dist - self.side_wall_target) if right_dist is not None else 0.0

        side_error = (left_error - right_error) / 2 if (left_dist and right_dist) else 0.0

        linear_correction = self.kp_rear * rear_error
        steering_correction = -self.kp_side * side_error

        linear_correction = max(-0.05, min(0.05, linear_correction))
        steering_correction = max(-0.2, min(0.2, steering_correction))

        if (abs(rear_error) < self.alignment_tolerance and
                abs(side_error) < self.alignment_tolerance):
            self._transition_to(ParkingStage.FULL_STOP)
            logger.info("Parking: ALIGN_CENTER complete -> FULL_STOP")
            return

        if self.stage_elapsed > self.stage_durations[ParkingStage.ALIGN_CENTER]:
            self._transition_to(ParkingStage.FULL_STOP)
            return

        self.steering.set_speed(linear_correction, steering_correction)

    def _execute_full_stop(self):
        """Stage 4: Stop and lock motors."""
        self.steering.stop()

        if self.stage_elapsed > self.stage_durations[ParkingStage.FULL_STOP]:
            self._transition_to(ParkingStage.COMPLETE)
            logger.info("Parking COMPLETE!")

    def _transition_to(self, new_stage: ParkingStage):
        self.stage = new_stage
        self.stage_start_time = time.time()
        self.stage_elapsed = 0.0

    def is_complete(self) -> bool:
        return self.stage == ParkingStage.COMPLETE

    def is_aborted(self) -> bool:
        return self.stage == ParkingStage.ABORTED

    def get_stage(self) -> ParkingStage:
        return self.stage
