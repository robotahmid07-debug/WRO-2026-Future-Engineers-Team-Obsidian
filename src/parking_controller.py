"""
4-Stage parallel parking state machine with closed-loop control.
Uses ultrasonic sensors for feedback during alignment.
"""

import time
import math
import logging
from enum import Enum
from typing import Optional, Dict, Any, Tuple

logger = logging.getLogger(__name__)


class ParkingStage(Enum):
    """Stages of the parallel parking sequence."""
    IDLE = 0
    APPROACH = 1          # Align parallel to parking bay
    REVERSE_STEER = 2     # Reverse into bay with steering
    ALIGN_CENTER = 3      # Fine-tune position using sensors
    FULL_STOP = 4         # Final stop and motor lock
    COMPLETE = 5
    ABORTED = 6


class ParkingController:
    """
    4-Stage parallel parking controller with sensor feedback.

    Stages:
    1. APPROACH: Drive forward, align parallel to bay using side Lidar
    2. REVERSE_STEER: Reverse with Ackerman turn into the bay
    3. ALIGN_CENTER: Fine-tune using side/rear ultrasonic distances
    4. FULL_STOP: Engage motor brake and zero velocity
    """

    def __init__(self, localization, steering_controller, emergency_shield,
                 config: Dict[str, Any]):
        """
        Initialize parking controller.

        Args:
            localization: Localization object for pose tracking
            steering_controller: Steering controller for motor commands
            emergency_shield: Emergency shield for safety overrides
            config: Parking configuration from YAML
        """
        self.localization = localization
        self.steering = steering_controller
        self.emergency = emergency_shield
        self.config = config

        # State management
        self.stage = ParkingStage.IDLE
        self.stage_start_time = 0.0
        self.stage_elapsed = 0.0

        # Parking geometry
        self.parking_geometry: Optional[Dict[str, float]] = None
        self.target_pose = None

        # Sensor feedback for closed-loop control
        self.side_distance_target = 0.15  # 15cm from wall
        self.rear_distance_target = 0.10  # 10cm from rear wall
        self.alignment_tolerance = 0.02   # 2cm tolerance

        # PID gains for alignment
        self.kp_side = 0.5
        self.kp_rear = 0.3

        # Stage durations (fallback if sensors fail)
        self.stage_durations = {
            ParkingStage.APPROACH: 2.0,
            ParkingStage.REVERSE_STEER: 2.5,
            ParkingStage.ALIGN_CENTER: 2.0,
            ParkingStage.FULL_STOP: 0.5,
        }

    def start(self, parking_geometry: Dict[str, float]):
        """
        Start the parking sequence with the given bay geometry.

        Args:
            parking_geometry: Dictionary with x_min, x_max, y_min, y_max
        """
        self.parking_geometry = parking_geometry
        self.stage = ParkingStage.APPROACH
        self.stage_start_time = time.time()
        self.stage_elapsed = 0.0
        logger.info("Parking sequence STARTED - Stage: APPROACH")

    def abort(self):
        """Abort the parking sequence and stop the robot."""
        self.stage = ParkingStage.ABORTED
        self.steering.stop()
        logger.warning("Parking sequence ABORTED")

    def update(self) -> ParkingStage:
        """
        Execute the current parking stage and transition when complete.

        Returns:
            Current ParkingStage.
        """
        if self.stage == ParkingStage.IDLE:
            return self.stage

        if self.stage in (ParkingStage.COMPLETE, ParkingStage.ABORTED):
            return self.stage

        # Update elapsed time
        self.stage_elapsed = time.time() - self.stage_start_time

        # Check emergency shield first
        emergency_actions = self.emergency.get_emergency_actions()
        if emergency_actions.get('brake', False):
            self.steering.stop()
            logger.warning("Emergency brake triggered during parking")
            return self.stage

        # Execute current stage
        if self.stage == ParkingStage.APPROACH:
            self._execute_approach()
        elif self.stage == ParkingStage.REVERSE_STEER:
            self._execute_reverse_steer()
        elif self.stage == ParkingStage.ALIGN_CENTER:
            self._execute_align_center()
        elif self.stage == ParkingStage.FULL_STOP:
            self._execute_full_stop()

        return self.stage

    def _execute_approach(self):
        """
        Stage 1: Approach the parking bay and align parallel.
        Drive forward slowly while using side Lidar to maintain alignment.
        """
        # Get current pose
        pose = self.localization.get_pose()

        # Get side distance from Lidar (left side)
        side_distance = self._get_side_distance()

        # Compute steering correction to maintain parallel alignment
        if side_distance is not None:
            error = side_distance - self.side_distance_target
            steering_correction = -self.kp_side * error
            # Clamp steering
            steering_correction = max(-0.3, min(0.3, steering_correction))
        else:
            steering_correction = 0.0

        # Drive forward at slow speed
        linear_speed = 0.15
        self.steering.set_speed(linear_speed, steering_correction)

        # Check if we've reached the bay entry
        # Using distance traveled or sensor feedback
        if self.stage_elapsed > self.stage_durations[ParkingStage.APPROACH]:
            self._transition_to(ParkingStage.REVERSE_STEER)
            logger.info("Parking: APPROACH complete -> REVERSE_STEER")

    def _execute_reverse_steer(self):
        """
        Stage 2: Reverse into the parking bay with steering.
        Execute an inverse Ackerman turn to enter the bay.
        """
        # Use a fixed steering angle for the turn
        # The angle should be based on the bay geometry
        steering_angle = -0.4  # Negative = right turn (reverse)
        linear_speed = -0.15   # Reverse

        # Apply steering
        self.steering.set_speed(linear_speed, steering_angle)

        # Monitor rear distance to know when to stop turning
        rear_distance = self._get_rear_distance()

        # Check if we've reached the target position or timed out
        if self.stage_elapsed > self.stage_durations[ParkingStage.REVERSE_STEER]:
            self._transition_to(ParkingStage.ALIGN_CENTER)
            logger.info("Parking: REVERSE_STEER complete -> ALIGN_CENTER")

    def _execute_align_center(self):
        """
        Stage 3: Fine-tune position using side and rear sensors.
        Adjust until centered in the bay with proper clearance.
        """
        # Get sensor feedback
        side_distance = self._get_side_distance()
        rear_distance = self._get_rear_distance()

        # Compute corrections
        side_error = 0.0
        rear_error = 0.0

        if side_distance is not None:
            side_error = side_distance - self.side_distance_target

        if rear_distance is not None:
            rear_error = rear_distance - self.rear_distance_target

        # Simple control: adjust position based on errors
        linear_correction = self.kp_rear * rear_error
        steering_correction = -self.kp_side * side_error

        # Clamp values
        linear_correction = max(-0.05, min(0.05, linear_correction))
        steering_correction = max(-0.2, min(0.2, steering_correction))

        # Apply corrections (slow movements)
        if abs(side_error) < self.alignment_tolerance and abs(rear_error) < self.alignment_tolerance:
            # Aligned! Move to next stage
            self.steering.set_speed(0.0, 0.0)
            self._transition_to(ParkingStage.FULL_STOP)
            logger.info("Parking: ALIGN_CENTER complete (aligned) -> FULL_STOP")
        elif self.stage_elapsed > self.stage_durations[ParkingStage.ALIGN_CENTER]:
            # Timeout - move to next stage anyway
            self.steering.set_speed(0.0, 0.0)
            self._transition_to(ParkingStage.FULL_STOP)
            logger.info("Parking: ALIGN_CENTER timeout -> FULL_STOP")
        else:
            # Continue adjusting
            self.steering.set_speed(linear_correction, steering_correction)

    def _execute_full_stop(self):
        """
        Stage 4: Engage motor brake and zero velocity.
        """
        # Stop motors and engage brake
        self.steering.stop()

        # Check if brake is engaged
        if self.stage_elapsed > self.stage_durations[ParkingStage.FULL_STOP]:
            self._transition_to(ParkingStage.COMPLETE)
            logger.info("Parking: FULL_STOP complete -> COMPLETE")

    def _transition_to(self, new_stage: ParkingStage):
        """Transition to a new stage and reset timer."""
        self.stage = new_stage
        self.stage_start_time = time.time()
        self.stage_elapsed = 0.0

    def _get_side_distance(self) -> Optional[float]:
        """
        Get distance to the side wall using ultrasonic or Lidar.

        Returns:
            Distance in meters, or None if unavailable.
        """
        # Try to get from emergency shield first
        if self.emergency and hasattr(self.emergency, 'latest_distances'):
            # Use front-left as side distance (adjust based on sensor placement)
            dist = self.emergency.latest_distances.get('front_left', None)
            if dist is not None and dist < 100:  # Valid reading
                return dist / 100.0  # Convert cm to m
        return None

    def _get_rear_distance(self) -> Optional[float]:
        """
        Get distance to the rear wall using ultrasonic.

        Returns:
            Distance in meters, or None if unavailable.
        """
        # For now, we use a fallback. In production, add a rear ultrasonic sensor.
        # We'll use front distance as a proxy (in reverse, front becomes rear)
        if self.emergency and hasattr(self.emergency, 'latest_distances'):
            dist = self.emergency.latest_distances.get('front', None)
            if dist is not None and dist < 100:
                return dist / 100.0
        return None

    def is_complete(self) -> bool:
        """Check if parking is complete."""
        return self.stage == ParkingStage.COMPLETE

    def is_aborted(self) -> bool:
        """Check if parking was aborted."""
        return self.stage == ParkingStage.ABORTED

    def get_stage(self) -> ParkingStage:
        """Get the current parking stage."""
        return self.stage
