"""
LIDAR-based 4-Stage Parallel Parking.
Uses 360° LIDAR for precise distance measurements to rear and side walls.
Implements LOGIC D, E, F, G for full 15‑point scoring.

LOGIC D – Direction‑aware reverse steer (side chosen by state machine)
LOGIC E – Fuse ultrasonics + LIDAR in ALIGN stage (ultrasonic < 25 cm)
LOGIC F – True parallel + inside check with stability counter (8 frames)
LOGIC G – Enhanced touch avoidance with forward escape (slow opposite linear + steer)
"""

import time
import math
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
        """
        Initialize parking controller.

        Args:
            localization: Localization object for pose data.
            steering_controller: SteeringController for motor/servo commands.
            lidar_fusion: LidarFusion object for LIDAR scans.
            config: Configuration dict (zone_management or parking section).
        """
        self.localization = localization
        self.steering = steering_controller
        self.lidar = lidar_fusion
        self.config = config

        # ---- State ----
        self.stage = ParkingStage.IDLE
        self.stage_start_time = 0.0
        self.stage_elapsed = 0.0

        self.parking_geometry: Optional[Dict[str, float]] = None
        self.target_pose = None

        # ---- LIDAR-based parking targets (tune these on the real track) ----
        self.rear_wall_target = 0.05   # 5 cm from rear wall
        self.side_wall_target = 0.10   # 10 cm from side wall
        self.alignment_tolerance = 0.02  # ±2 cm

        # ---- PID gains for LIDAR-based alignment ----
        self.kp_rear = 0.5
        self.kp_side = 0.3

        # ---- Stage durations (fallback if LIDAR fails) ----
        self.stage_durations = {
            ParkingStage.APPROACH: 2.0,
            ParkingStage.REVERSE_STEER: 3.0,   # extended for direction‑aware reverse
            ParkingStage.ALIGN_CENTER: 3.0,    # extended for stable check
            ParkingStage.FULL_STOP: 0.5,
        }

        # ---- LOGIC D: reverse steer direction ----
        self.reverse_steer_offset = 0.35   # positive = left, negative = right
        self.bay_side = 'left'             # 'left' or 'right' – set by state machine

        # ---- LOGIC F: stability counter ----
        self.stable_count = 0
        self.STABLE_FRAMES_REQUIRED = 8    # 8 frames at 50 ms = 0.4 s

        # ---- LOGIC G: Enhanced touch avoidance ----
        self.emergency = None              # reference to emergency shield (set via set_emergency_shield)
        self.touch_escape_active = False
        self.touch_escape_start_time = 0.0
        self.TOUCH_ESCAPE_DURATION = 0.5   # seconds to crawl forward
        self.TOUCH_ESCAPE_SPEED = 0.05     # m/s forward during escape
        self.TOUCH_CLEAR_THRESHOLD = 0.06  # 6 cm = clear

        logger.info("ParkingController initialized")

    # ==========================================================
    # LOGIC D: Direction‑aware reverse steer setters
    # ==========================================================

    def set_reverse_steer_offset(self, offset: float):
        """Set the reverse steer direction from state machine (LOGIC D)."""
        self.reverse_steer_offset = offset
        logger.debug(f"Reverse steer offset set to: {offset:.2f}")

    def set_bay_side(self, side: str):
        """Set which side the bay is on ('left' or 'right')."""
        self.bay_side = side
        logger.debug(f"Bay side set to: {side}")

    def set_emergency_shield(self, emergency):
        """
        Give the parking controller access to emergency shield for ultrasonics.
        Called from main.py before any parking stage runs.
        """
        self.emergency = emergency
        logger.info("Emergency shield reference set for parking controller")

    # ==========================================================
    # Public interface
    # ==========================================================

    def start(self, parking_geometry: Dict[str, float]):
        """Start the parking sequence with the given geometry."""
        self.parking_geometry = parking_geometry
        self.stage = ParkingStage.APPROACH
        self.stage_start_time = time.time()
        self.stage_elapsed = 0.0
        self.stable_count = 0
        self.touch_escape_active = False
        logger.info("LIDAR-based parking STARTED")

    def abort(self):
        """Abort parking and stop motors."""
        self.stage = ParkingStage.ABORTED
        self.steering.stop()
        self.touch_escape_active = False
        logger.warning("Parking ABORTED")

    def update(self) -> ParkingStage:
        """Run one cycle of the parking state machine. Call at ~20 Hz."""
        if self.stage == ParkingStage.IDLE:
            return self.stage
        if self.stage in (ParkingStage.COMPLETE, ParkingStage.ABORTED):
            return self.stage

        self.stage_elapsed = time.time() - self.stage_start_time

        # ---- Check if emergency shield is set ----
        if self.emergency is None:
            logger.warning("Parking: emergency shield not set – ultrasonic fusion disabled")

        if self.stage == ParkingStage.APPROACH:
            self._execute_approach()
        elif self.stage == ParkingStage.REVERSE_STEER:
            self._execute_reverse_steer()
        elif self.stage == ParkingStage.ALIGN_CENTER:
            self._execute_align_center()
        elif self.stage == ParkingStage.FULL_STOP:
            self._execute_full_stop()

        return self.stage

    # ==========================================================
    # LIDAR distance helpers
    # ==========================================================

    def _get_rear_distance_from_lidar(self) -> Optional[float]:
        """Get distance to rear wall using LIDAR (180° / -180° sector)."""
        scan = self.lidar.get_scan_snapshot()
        if not scan:
            return None

        rear_dists = []
        for ang, dist in scan.items():
            if dist < 0.05 or dist > 5.0:
                continue
            if abs(abs(ang) - 180) < 15:  # rear sector ±15°
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

    # ==========================================================
    # LOGIC E: Ultrasonic access for fusion
    # ==========================================================

    def _get_ultrasonic_distances(self) -> dict:
        """
        Get ultrasonic distances from emergency shield (cm → m).
        Returns dict with 'front', 'front_left', 'front_right' in meters.
        """
        if self.emergency is None:
            return {
                'front': float('inf'),
                'front_left': float('inf'),
                'front_right': float('inf')
            }

        return {
            'front': self.emergency.latest_distances.get('front', float('inf')) / 100.0,
            'front_left': self.emergency.latest_distances.get('front_left', float('inf')) / 100.0,
            'front_right': self.emergency.latest_distances.get('front_right', float('inf')) / 100.0,
        }

    # ==========================================================
    # LOGIC G: Enhanced touch avoidance with forward escape
    # ==========================================================

    def _check_touch_avoidance(self) -> tuple:
        """
        Enhanced LOGIC G: Check if any front ultrasonic < 4 cm.
        If triggered, do a short forward crawl with opposite steer.
        Returns (should_escape, steer_direction)
        """
        us = self._get_ultrasonic_distances()
        fl = us.get('front_left', 1.0)
        fr = us.get('front_right', 1.0)

        # If already in escape, check if clear
        if self.touch_escape_active:
            # Check elapsed time
            elapsed = time.time() - self.touch_escape_start_time
            if elapsed > self.TOUCH_ESCAPE_DURATION:
                self.touch_escape_active = False
                logger.info("Touch escape: duration complete, resuming normal stage")
                return False, 0.0

            # Also clear if both sensors > clear threshold (6 cm)
            if fl > self.TOUCH_CLEAR_THRESHOLD and fr > self.TOUCH_CLEAR_THRESHOLD:
                self.touch_escape_active = False
                logger.info("Touch escape: clear, resuming normal stage")
                return False, 0.0

            # Still in escape – continue moving forward with steer away
            if fl < fr:
                return True, 0.3   # steer right (away from left wall)
            else:
                return True, -0.3  # steer left (away from right wall)

        # Check if we need to trigger escape (any sensor < 4 cm)
        if fl < 0.04 or fr < 0.04:
            self.touch_escape_active = True
            self.touch_escape_start_time = time.time()
            logger.warning(f"Touch escape triggered! fl={fl:.3f}, fr={fr:.3f}")
            # Steer away from the closest side
            if fl < fr:
                return True, 0.3   # steer right (away from left wall)
            else:
                return True, -0.3  # steer left (away from right wall)

        return False, 0.0

    # ==========================================================
    # LOGIC F: Normalize angle helper
    # ==========================================================

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # ==========================================================
    # Stage execution
    # ==========================================================

    def _execute_approach(self):
        """Stage 1: Approach parking bay using LIDAR rear distance."""
        rear_dist = self._get_rear_distance_from_lidar()

        if rear_dist is not None and rear_dist < self.rear_wall_target * 2:
            self._transition_to(ParkingStage.REVERSE_STEER)
            logger.info("Parking: APPROACH complete -> REVERSE_STEER")
            return

        self.steering.set_speed(0.10, 0.0)

        if self.stage_elapsed > self.stage_durations[ParkingStage.APPROACH]:
            logger.warning("Parking: APPROACH timeout -> REVERSE_STEER")
            self._transition_to(ParkingStage.REVERSE_STEER)

    def _execute_reverse_steer(self):
        """
        Stage 2: Reverse with steering into the bay.
        LOGIC D – Direction‑aware reverse steer.
        LOGIC G – Touch avoidance with forward escape.
        """
        # ---- LOGIC G: Enhanced touch avoidance (escape) ----
        should_escape, steer_escape = self._check_touch_avoidance()
        if should_escape:
            # Move forward slowly + steer away (counter-direction)
            self.steering.set_speed(self.TOUCH_ESCAPE_SPEED, steer_escape)
            return  # Stay in this stage until escape complete

        rear_dist = self._get_rear_distance_from_lidar()

        # If rear is close, move to align
        if rear_dist is not None and rear_dist < self.rear_wall_target * 1.5:
            self._transition_to(ParkingStage.ALIGN_CENTER)
            logger.info("Parking: REVERSE_STEER complete -> ALIGN_CENTER")
            return

        # ---- LOGIC D: direction‑aware reverse steer ----
        steer = self.reverse_steer_offset

        # Fine‑tune with LIDAR side distances to avoid walls
        left_dist = self._get_side_distance_from_lidar('left')
        right_dist = self._get_side_distance_from_lidar('right')

        if left_dist is not None and left_dist < self.side_wall_target * 1.2:
            steer -= 0.15   # steer right (away from left wall)
        if right_dist is not None and right_dist < self.side_wall_target * 1.2:
            steer += 0.15   # steer left (away from right wall)

        # Clamp to safe range
        steer = max(-0.6, min(0.6, steer))

        self.steering.set_speed(-0.12, steer)

        if self.stage_elapsed > self.stage_durations[ParkingStage.REVERSE_STEER]:
            logger.warning("Parking: REVERSE_STEER timeout -> ALIGN_CENTER")
            self._transition_to(ParkingStage.ALIGN_CENTER)

    def _execute_align_center(self):
        """
        Stage 3: Fine-tune position using LIDAR + ultrasonics (LOGIC E).
        LOGIC F – Check parallel + inside before completing.
        LOGIC G – Touch avoidance during alignment.
        """
        # ---- LOGIC G: Touch avoidance during alignment ----
        should_escape, steer_escape = self._check_touch_avoidance()
        if should_escape:
            self.steering.set_speed(self.TOUCH_ESCAPE_SPEED, steer_escape)
            return  # Stay in this stage until escape complete

        # ---- LOGIC E: Fuse LIDAR + ultrasonics ----
        lidar_rear = self._get_rear_distance_from_lidar()
        us = self._get_ultrasonic_distances()

        # Rear: only LIDAR (no rear ultrasonic)
        rear_dist = lidar_rear if lidar_rear is not None else 0.10

        # Side: prefer ultrasonics when < 25 cm
        left_lidar = self._get_side_distance_from_lidar('left')
        right_lidar = self._get_side_distance_from_lidar('right')

        fl_us = us.get('front_left', 1.0)
        fr_us = us.get('front_right', 1.0)

        if fl_us < 0.25:
            left_dist = fl_us
        else:
            left_dist = left_lidar if left_lidar is not None else 0.20

        if fr_us < 0.25:
            right_dist = fr_us
        else:
            right_dist = right_lidar if right_lidar is not None else 0.20

        # Compute errors
        rear_error = rear_dist - self.rear_wall_target
        left_error = left_dist - self.side_wall_target
        right_error = right_dist - self.side_wall_target
        side_error = (left_error - right_error) / 2.0

        # PID corrections
        linear_correction = self.kp_rear * rear_error
        steering_correction = -self.kp_side * side_error

        linear_correction = max(-0.05, min(0.05, linear_correction))
        steering_correction = max(-0.2, min(0.2, steering_correction))

        # ---- LOGIC F: Check conditions for completion ----
        rear_ok = abs(rear_error) < self.alignment_tolerance
        side_ok = abs(side_error) < self.alignment_tolerance

        # Heading parallel to wall
        geometry = self.parking_geometry
        heading_parallel = False
        if geometry:
            # Wall direction: vector from marker1 to marker2 (along the wall)
            dx_wall = geometry['x_max'] - geometry['x_min']
            dy_wall = geometry['y_max'] - geometry['y_min']
            wall_heading = math.atan2(dy_wall, dx_wall)

            pose = self.localization.get_pose()
            heading_error = pose.theta - wall_heading
            heading_error = self._normalize_angle(heading_error)

            # Accept if parallel to wall (0° or 180°)
            heading_parallel = min(abs(heading_error), abs(heading_error - math.pi)) < 0.12  # ~7°

        # Fully inside rectangle
        inside = False
        if geometry:
            pose = self.localization.get_pose()
            half_L = 0.15   # half of robot length (tune)
            half_W = 0.10   # half of robot width
            cos_t = math.cos(pose.theta)
            sin_t = math.sin(pose.theta)

            corners = [
                (pose.x + half_L * cos_t - half_W * sin_t,
                 pose.y + half_L * sin_t + half_W * cos_t),
                (pose.x + half_L * cos_t + half_W * sin_t,
                 pose.y + half_L * sin_t - half_W * cos_t),
                (pose.x - half_L * cos_t - half_W * sin_t,
                 pose.y - half_L * sin_t + half_W * cos_t),
                (pose.x - half_L * cos_t + half_W * sin_t,
                 pose.y - half_L * sin_t - half_W * cos_t),
            ]

            inside = all(
                geometry['x_min'] <= cx <= geometry['x_max'] and
                geometry['y_min'] <= cy <= geometry['y_max']
                for cx, cy in corners
            )

        if rear_ok and side_ok and heading_parallel and inside:
            self.stable_count += 1
        else:
            self.stable_count = 0

        # If stable for enough frames, transition to FULL_STOP
        if self.stable_count >= self.STABLE_FRAMES_REQUIRED:
            self._transition_to(ParkingStage.FULL_STOP)
            logger.info("Parking: ALIGN_CENTER complete (stable) -> FULL_STOP")
            return

        # Apply corrections
        self.steering.set_speed(linear_correction, steering_correction)

        # Fallback timeout
        if self.stage_elapsed > self.stage_durations[ParkingStage.ALIGN_CENTER]:
            logger.warning("Parking: ALIGN_CENTER timeout -> FULL_STOP")
            self._transition_to(ParkingStage.FULL_STOP)

    def _execute_full_stop(self):
        """Stage 4: Stop and lock motors."""
        self.steering.stop()

        if self.stage_elapsed > self.stage_durations[ParkingStage.FULL_STOP]:
            self._transition_to(ParkingStage.COMPLETE)
            logger.info("Parking COMPLETE!")

    def _transition_to(self, new_stage: ParkingStage):
        """Transition to a new stage and reset timers."""
        self.stage = new_stage
        self.stage_start_time = time.time()
        self.stage_elapsed = 0.0
        if new_stage == ParkingStage.ALIGN_CENTER:
            self.stable_count = 0
        # Reset touch escape when transitioning
        self.touch_escape_active = False

    # ==========================================================
    # Status getters
    # ==========================================================

    def is_complete(self) -> bool:
        return self.stage == ParkingStage.COMPLETE

    def is_aborted(self) -> bool:
        return self.stage == ParkingStage.ABORTED

    def get_stage(self) -> ParkingStage:
        return self.stage

    def reset(self):
        """Reset the parking controller to IDLE state."""
        self.stage = ParkingStage.IDLE
        self.stage_start_time = 0.0
        self.stage_elapsed = 0.0
        self.parking_geometry = None
        self.stable_count = 0
        self.touch_escape_active = False
        logger.info("ParkingController reset")
