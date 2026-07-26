"""
Global execution FSM (INIT -> NAVIGATE -> TERMINATION).
Supports both Open and Obstacle challenges.
Uses WallMapper for map building and LIDAR‑based lap counting.
"""

import time
import math
import logging
from enum import Enum
from typing import Optional, List, Tuple

from .config_parser import SystemConfig, ChallengeConfig
from .localization import Localization
from .vision_tracker import HuskyLensReader, ColorBlock
from .lidar_fusion import LidarFusion
from .spatial_map import SpatialMap, TrackedObject
from .emergency_shield import EmergencyShield
from .steering_controller import SteeringController
from .parking_controller import ParkingController
from .wall_mapper import WallMapper

logger = logging.getLogger(__name__)


class RobotState(Enum):
    INIT = 1
    NAVIGATE = 2
    TERMINATION = 3
    EMERGENCY_STOP = 4


class StateMachine:
    # HuskyLens color IDs (learned)
    COLOR_RED = 1
    COLOR_GREEN = 2

    def __init__(self, config: SystemConfig, serial_bridge, localization: Localization,
                 vision_reader: HuskyLensReader, lidar_fusion: LidarFusion,
                 spatial_map: SpatialMap, emergency_shield: EmergencyShield,
                 steering: SteeringController, parking: ParkingController,
                 challenge: str = "open"):
        """
        Initialize the state machine.

        Args:
            config: Parsed SystemConfig object.
            challenge: "open" or "obstacle" – selects which challenge mode to run.
        """
        self.config = config
        self.serial_bridge = serial_bridge
        self.localization = localization
        self.vision = vision_reader
        self.lidar = lidar_fusion
        self.spatial_map = spatial_map
        self.emergency = emergency_shield
        self.steering = steering
        self.parking = parking

        # Select active challenge config from nested YAML
        self.challenge = challenge.lower()
        if self.challenge == "open":
            self.active_config: ChallengeConfig = config.zone_management.open_challenge
        elif self.challenge == "obstacle":
            self.active_config = config.zone_management.obstacle_challenge
        else:
            raise ValueError(f"Invalid challenge: {challenge}. Must be 'open' or 'obstacle'.")

        self.is_open_challenge = not self.active_config.use_parking_slot

        # State tracking
        self.state = RobotState.INIT
        self.lap_count = 0
        self.lap_start_pose = None
        self.sections_passed = 0
        self.last_section_time = time.time()

        # Navigation constants
        self.BASE_SPEED = 0.3  # m/s
        self.STEER_MAGNITUDE = 0.3  # rad/s (yaw rate magnitude when turning)

        # Lap counting: distance fallback (meters) – calibrate this!
        self.track_length_estimate = 5.0  # full lap distance

        # For LIDAR‑based corner detection: store previous side distances
        self._prev_avg_left = None
        self._prev_avg_right = None

        logger.info(f"StateMachine initialized with challenge: {self.challenge} "
                    f"(Open: {self.is_open_challenge})")

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

        # Register parking slot only if obstacle challenge
        if not self.is_open_challenge:
            self.localization.register_parking_slot()
            parking_geometry = self.localization.parking_bay_geometry
            if parking_geometry:
                self.parking.start(parking_geometry)
                logger.info("Parking geometry registered: %s", parking_geometry)

        # Reset lap counter and section counter
        self.lap_count = 0
        self.sections_passed = 0
        self.last_section_time = time.time()

        # Reset LIDAR corner detection history
        self._prev_avg_left = None
        self._prev_avg_right = None

        logger.info("Lap counter reset to 0")
        self.state = RobotState.NAVIGATE
        logger.info("Transition to NAVIGATE")

    def _get_angular_velocity_from_color(self, color_id: int) -> float:
        """
        Determines the angular velocity (yaw rate) based on the color ID
        and the pass-side rules defined in the YAML config.

        Returns:
            Angular velocity in rad/s. 0.0 if unknown.
        """
        if color_id == self.COLOR_RED:
            pass_side = self.config.traffic_light_passing_rules.RED_BLOCK_PASS_SIDE.upper()
        elif color_id == self.COLOR_GREEN:
            pass_side = self.config.traffic_light_passing_rules.GREEN_BLOCK_PASS_SIDE.upper()
        else:
            return 0.0

        if pass_side == "RIGHT":
            return -self.STEER_MAGNITUDE
        elif pass_side == "LEFT":
            return self.STEER_MAGNITUDE
        else:
            logger.warning(f"Invalid pass side '{pass_side}' in config. Defaulting to straight.")
            return 0.0

    def _detected_corner_via_lidar(self) -> bool:
        """
        Detect if the robot is passing a corner by analyzing side wall distances.
        A corner is detected when the side distance changes suddenly (> 40% change).

        Returns:
            True if a corner passage is detected.
        """
        # Get thread‑safe LIDAR snapshot
        scan_data = self.lidar.get_scan_snapshot()
        if not scan_data:
            return False

        # Collect distances to left (90°) and right (-90°)
        left_dists = []
        right_dists = []
        for ang, dist in scan_data.items():
            if dist < 0.05 or dist > 5.0:
                continue
            if abs(ang - 90) < 10:
                left_dists.append(dist)
            elif abs(ang + 90) < 10:
                right_dists.append(dist)

        if not left_dists or not right_dists:
            return False

        avg_left = sum(left_dists) / len(left_dists)
        avg_right = sum(right_dists) / len(right_dists)

        # If no previous values, store and return false
        if self._prev_avg_left is None or self._prev_avg_right is None:
            self._prev_avg_left = avg_left
            self._prev_avg_right = avg_right
            return False

        # Compute percentage change (avoid division by zero)
        left_change = abs(avg_left - self._prev_avg_left) / max(self._prev_avg_left, 0.1)
        right_change = abs(avg_right - self._prev_avg_right) / max(self._prev_avg_right, 0.1)

        # Update stored values
        self._prev_avg_left = avg_left
        self._prev_avg_right = avg_right

        # If change > 40%, it's likely a corner
        if left_change > 0.4 or right_change > 0.4:
            logger.debug(f"Corner detected: left_change={left_change:.2f}, right_change={right_change:.2f}")
            return True

        return False

    def _update_lap_count(self):
        """
        Update lap count using LIDAR‑based corner detection with odometry fallback.
        A lap = 8 sections (4 corners + 4 straights).
        """
        # 1. Try LIDAR corner detection
        if self._detected_corner_via_lidar():
            self.sections_passed += 1
            self.last_section_time = time.time()
            logger.debug(f"Corner detected via LIDAR -> sections_passed={self.sections_passed}")

        # 2. Fallback: if no corner detected for > 3 seconds, use odometry distance
        if time.time() - self.last_section_time > 3.0:
            pose = self.localization.get_pose()
            if self.lap_start_pose is not None:
                distance_traveled = abs(pose.x - self.lap_start_pose.x)
                # Count a section every 1/8 of total lap distance
                section_distance = self.track_length_estimate / 8.0
                if distance_traveled > section_distance:
                    self.sections_passed += 1
                    self.last_section_time = time.time()
                    self.lap_start_pose = pose  # reset for next section
                    logger.debug(f"Section counted via odometry fallback -> sections_passed={self.sections_passed}")

        # 3. Check if a full lap (8 sections) has been completed
        if self.sections_passed >= 8:
            self.lap_count += 1
            self.sections_passed = 0
            self.lap_start_pose = self.localization.get_pose()
            logger.info(f"Lap {self.lap_count} completed (section-based)")

            if self.lap_count >= self.config.navigation_matrix.TOTAL_REQUIRED_LAPS:
                logger.info("All %d laps completed!", self.config.navigation_matrix.TOTAL_REQUIRED_LAPS)
                self.state = RobotState.TERMINATION

    def _navigate_state(self):
        """Main navigation loop with sensor fusion and control."""
        # 1. Get color detections from HuskyLens
        color_blocks = self.vision.get_latest_colors()
        detections = []
        for color_block in color_blocks:
            angle_deg = ((color_block.x - 160) / 160.0) * 30.0  # -30 to +30 deg
            range_m = self.lidar.get_range_in_sector(angle_deg, 5.0)
            if range_m is not None and range_m > 0.1:
                angle_rad = math.radians(angle_deg)
                x_local = range_m * math.cos(angle_rad)
                y_local = range_m * math.sin(angle_rad)
                if x_local > 0:
                    detections.append((color_block.color_id, x_local, y_local))

        if detections:
            self.spatial_map.update(detections)

        # 2. Get confirmed objects from spatial map
        confirmed_objects = self.spatial_map.get_confirmed_objects()

        # 3. Tell emergency shield the target direction based on traffic light
        if confirmed_objects:
            target = confirmed_objects[0]
            angular = self._get_angular_velocity_from_color(target.color_id)
            # Tell emergency shield which direction we want to go
            self.emergency.set_target_steer_direction(math.copysign(1.0, angular) if angular != 0 else 0.0)
        else:
            self.emergency.set_target_steer_direction(0.0)

        # 4. Update emergency shield
        self.emergency.update()
        emergency_actions = self.emergency.get_emergency_actions()

        # 5. Check emergency stop
        if emergency_actions.get('brake', False):
            self.steering.stop()
            self.state = RobotState.EMERGENCY_STOP
            logger.warning("Emergency stop triggered")
            return

        # 6. Calculate steering from emergency actions
        angular = emergency_actions.get('steer_offset', 0.0)
        throttle = emergency_actions.get('throttle_factor', 1.0)
        linear = self.BASE_SPEED * throttle

        # 7. Apply steering (Ackermann)
        self.steering.set_speed(linear, angular)

        # 8. Update localization with LIDAR points for mapping
        scan_data = self.lidar.get_scan_snapshot()
        lidar_scan = [(ang, dist) for ang, dist in scan_data.items()]
        # Use a subset to reduce processing (every 5 degrees)
        lidar_subset = [(ang, dist) for ang, dist in lidar_scan if abs(ang) % 5 < 1]

        steering_angle = 0.0 if abs(linear) < 0.001 else math.atan2(angular * self.steering.wheelbase, linear)
        self.localization.update_pose(linear, steering_angle, lidar_points=lidar_subset)

        # 9. Update lap counting
        self._update_lap_count()

    def _termination_state(self):
        """Handle termination: open challenge stop or obstacle parking."""
        logger.info("State: TERMINATION")

        if self.is_open_challenge:
            # Open challenge: stop at start zone
            start_pose = self.localization.get_start_pose()
            if start_pose:
                # For simplicity, just stop
                self.steering.stop()
                logger.info("Open challenge complete – stopped at start zone")
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
        """Emergency stop state – robot is stopped."""
        self.steering.stop()
        logger.warning("Robot in EMERGENCY_STOP state. Manual reset required.")

    def reset(self):
        """Reset the state machine to INIT."""
        self.state = RobotState.INIT
        self.lap_count = 0
        self.sections_passed = 0
        self.lap_start_pose = None
        self._prev_avg_left = None
        self._prev_avg_right = None
        logger.info("State machine reset to INIT")
