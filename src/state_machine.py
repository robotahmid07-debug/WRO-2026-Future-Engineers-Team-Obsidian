"""
Global execution FSM (INIT -> NAVIGATE -> TERMINATION).
Orchestrates all subsystems. 
Now reads turn direction dynamically from YAML config.
"""

import time
import math
import logging
from enum import Enum
from typing import Optional, List, Tuple

from .config_parser import SystemConfig
from .localization import Localization
from .vision_tracker import HuskyLensReader, ColorBlock
from .lidar_fusion import LidarFusion
from .spatial_map import SpatialMap, TrackedObject
from .emergency_shield import EmergencyShield
from .steering_controller import SteeringController
from .parking_controller import ParkingController

logger = logging.getLogger(__name__)


class RobotState(Enum):
    INIT = 1
    NAVIGATE = 2
    TERMINATION = 3
    EMERGENCY_STOP = 4


class StateMachine:
    """
    Main state machine for WRO Future Engineers 2026.
    Uses Ackermann steering geometry.
    All color-to-steering mappings are pulled from the YAML config.
    """

    # Color ID mapping (learned on HuskyLens V2)
    COLOR_RED = 1
    COLOR_GREEN = 2

    def __init__(self, config: SystemConfig, serial_bridge, localization: Localization,
                 vision_reader: HuskyLensReader, lidar_fusion: LidarFusion,
                 spatial_map: SpatialMap, emergency_shield: EmergencyShield,
                 steering: SteeringController, parking: ParkingController):
        self.config = config
        self.serial_bridge = serial_bridge
        self.localization = localization
        self.vision = vision_reader
        self.lidar = lidar_fusion
        self.spatial_map = spatial_map
        self.emergency = emergency_shield
        self.steering = steering
        self.parking = parking

        self.state = RobotState.INIT
        self.lap_count = 0
        self.is_open_challenge = not config.zone_management.use_parking_slot

        # For lap counting
        self.lap_start_pose = None
        self.track_length_estimate = 5.0  # meters per lap (calibrate)

        # Navigation constants
        self.BASE_SPEED = 0.3  # m/s
        self.STEER_MAGNITUDE = 0.3  # rad/s (yaw rate magnitude when turning)

    def run(self):
        """Main loop; call at ~20-50 Hz."""
        try:
            if self.state == RobotState.INIT:
                self._init_state()
            elif self.state == RobotState.NAVIGATE:
                self._navigate_state()
            elif self.state == RobotState.TERMINATION:
                self._termination_state()
            elif self.state == RobotState.EMERGENCY_STOP:
                self._emergency_stop()
        except Exception as e:
            logger.exception("State machine error: %s", e)
            self.steering.stop()

    def _init_state(self):
        """Initialize robot state: register pose, reset counters."""
        logger.info("State: INIT")

        # Register start pose
        self.localization.register_start_pose()
        self.lap_start_pose = self.localization.get_pose()

        # Register parking slot if obstacle challenge
        if not self.is_open_challenge:
            self.localization.register_parking_slot()
            parking_geometry = self.localization.parking_bay_geometry
            if parking_geometry:
                self.parking.start(parking_geometry)
                logger.info("Parking geometry registered: %s", parking_geometry)

        # Reset lap counter
        self.lap_count = 0
        logger.info("Lap counter reset to 0")

        # Transition to NAVIGATE
        self.state = RobotState.NAVIGATE
        logger.info("Transition to NAVIGATE")

    def _get_angular_velocity_from_color(self, color_id: int) -> float:
        """
        Determines the angular velocity (yaw rate) based on the color ID
        and the pass-side rules defined in the YAML config.

        Convention:
        - Positive angular velocity = Turn LEFT
        - Negative angular velocity = Turn RIGHT

        Args:
            color_id: The color ID detected by HuskyLens.

        Returns:
            Angular velocity in rad/s. 0.0 if color is unknown or should go straight.
        """
        # Get the pass side from config
        if color_id == self.COLOR_RED:
            pass_side = self.config.traffic_light_passing_rules.RED_BLOCK_PASS_SIDE.upper()
        elif color_id == self.COLOR_GREEN:
            pass_side = self.config.traffic_light_passing_rules.GREEN_BLOCK_PASS_SIDE.upper()
        else:
            # Unknown color -> go straight
            return 0.0

        # Map side to angular velocity sign
        if pass_side == "RIGHT":
            # Turning RIGHT means negative angular velocity
            return -self.STEER_MAGNITUDE
        elif pass_side == "LEFT":
            # Turning LEFT means positive angular velocity
            return self.STEER_MAGNITUDE
        else:
            # Invalid string in config? Default to straight.
            logger.warning(f"Invalid pass side '{pass_side}' in config. Defaulting to straight.")
            return 0.0

    def _navigate_state(self):
        """Main navigation loop with sensor fusion and Ackermann control."""
        # 1. Update emergency shield
        self.emergency.update()
        emergency_actions = self.emergency.get_emergency_actions()

        # 2. Check emergency stop
        if emergency_actions.get('brake', False):
            self.steering.stop()
            self.state = RobotState.EMERGENCY_STOP
            logger.warning("Emergency stop triggered")
            return

        # 3. Get color detections from HuskyLens V2
        color_blocks = self.vision.get_latest_colors()

        # 4. Process detections and update spatial map
        detections = []
        for color_block in color_blocks:
            # Map centroid to angle (HuskyLens V2 FOV ~60 degrees)
            # x ranges 0-319, center is 160
            angle_deg = ((color_block.x - 160) / 160.0) * 30.0  # -30 to +30 degrees

            # Get LIDAR range in that direction
            range_m = self.lidar.get_range_in_sector(angle_deg, 5.0)

            if range_m is not None and range_m > 0.1:
                # Compute local coordinates (forward = x, left = y)
                angle_rad = math.radians(angle_deg)
                x_local = range_m * math.cos(angle_rad)
                y_local = range_m * math.sin(angle_rad)

                # Only forward detections
                if x_local > 0:
                    detections.append((color_block.color_id, x_local, y_local))

        # Update spatial map with detections
        if detections:
            self.spatial_map.update(detections)

        # 5. Get confirmed objects
        confirmed_objects = self.spatial_map.get_confirmed_objects()

        # 6. Determine target and steering (fully config-driven)
        if confirmed_objects:
            target = confirmed_objects[0]  # Sorted by distance
            # Get angular velocity from YAML config
            angular = self._get_angular_velocity_from_color(target.color_id)
            logger.debug(f"Color ID {target.color_id} -> angular = {angular:.2f} rad/s")
        else:
            angular = 0.0

        # Apply emergency shield steering offset (adds to angular velocity)
        angular += emergency_actions.get('steer_offset', 0.0)

        # Throttle factor from emergency shield
        throttle = emergency_actions.get('throttle_factor', 1.0)
        linear = self.BASE_SPEED * throttle

        # 7. Apply steering (Ackermann)
        self.steering.set_speed(linear, angular)

        # 8. Update localization with Ackermann kinematics
        # Compute steering angle from angular velocity for odometry
        if abs(linear) > 0.001:
            steering_angle = math.atan2(angular * self.steering.wheelbase, linear)
        else:
            steering_angle = 0.0
        self.localization.update_pose(linear, steering_angle)

        # 9. Update lap counting
        self._update_lap_count()

    def _update_lap_count(self):
        """
        Update lap count based on odometry.
        In production, use wall detection or line crossing.
        """
        pose = self.localization.get_pose()

        if self.lap_start_pose is not None:
            distance_traveled = abs(pose.x - self.lap_start_pose.x)

            if distance_traveled > self.track_length_estimate:
                self.lap_count += 1
                self.lap_start_pose = pose
                logger.info("Lap %d/%d completed", self.lap_count,
                           self.config.navigation_matrix.TOTAL_REQUIRED_LAPS)

                if self.lap_count >= self.config.navigation_matrix.TOTAL_REQUIRED_LAPS:
                    logger.info("All %d laps completed!",
                               self.config.navigation_matrix.TOTAL_REQUIRED_LAPS)
                    self.state = RobotState.TERMINATION
                    logger.info("Transition to TERMINATION")

    def _termination_state(self):
        """Handle termination: open challenge stop or obstacle parking."""
        logger.info("State: TERMINATION")

        if self.is_open_challenge:
            # Open challenge: return to start pose and stop
            start_pose = self.localization.get_start_pose()
            if start_pose:
                self.steering.stop()
                logger.info("Open challenge complete - stopped at start zone")
            else:
                self.steering.stop()
        else:
            # Obstacle challenge: execute parallel parking
            if not self.parking.is_complete() and not self.parking.is_aborted():
                self.parking.update()
                logger.debug("Parking stage: %s", self.parking.get_stage())
            else:
                self.steering.stop()
                if self.parking.is_complete():
                    logger.info("Parking sequence COMPLETE! Robot parked successfully.")
                elif self.parking.is_aborted():
                    logger.warning("Parking sequence ABORTED.")

    def _emergency_stop(self):
        """Emergency stop state - robot is stopped."""
        self.steering.stop()
        logger.warning("Robot in EMERGENCY_STOP state. Manual reset required.")

    def reset(self):
        """Reset the state machine to INIT."""
        self.state = RobotState.INIT
        self.lap_count = 0
        self.lap_start_pose = None
        logger.info("State machine reset to INIT")
