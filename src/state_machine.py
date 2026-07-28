"""
Global execution FSM (INIT -> NAVIGATE -> TERMINATION).
Supports both Open and Obstacle challenges.
Integrates:
  - Per‑lap direction switching (CW/CCW) from YAML.
  - Adaptive wall‑following (left/right reference based on direction).
  - Surprise rule (Lap X -> Lap X+1 direction change based on traffic light color).
  - Emergency lap fallback (force lap completion if LIDAR fails).
  - Map usage control (enable/disable mapping).
  - IMU fusion for heading stabilisation and corner confirmation (data from ESP32).
  - Reverse handling from emergency shield (with timer).
  - Traffic light filtering: area, aspect ratio, Y‑position (ROI) to reject false positives.
  - All calibratable parameters now read from config.
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
    # Traffic light detection filters (tunable)
    MIN_AREA = 200          # Minimum area in pixels (ignore tiny blobs)
    MAX_AREA = 15000        # Maximum area (ignore huge objects)
    MIN_ASPECT = 1.5        # Minimum height/width ratio (pillars are tall)
    MAX_ASPECT = 4.0        # Maximum height/width ratio
    ROI_Y_MIN = 30          # Ignore objects above this Y (top of frame)
    ROI_Y_MAX = 210         # Ignore objects below this Y (bottom of frame)

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
        self.distance_since_lap_start = 0.0
        self.lap_start_x = 0.0

        # Direction tracking
        self.current_direction = self.config.navigation_matrix.LAP_1_DIRECTION.upper()
        self.last_traffic_light_color = None
        self.surprise_rule_activated = False

        # Read all calibratable parameters from config
        self.BASE_SPEED = self.config.navigation.base_speed_mps
        self.STEER_MAGNITUDE = self.config.navigation.steer_magnitude_radps
        self.WALL_FOLLOW_GAIN = self.config.navigation.wall_follow_gain

        # Color IDs from config
        self.COLOR_RED = self.config.vision.color_red_id
        self.COLOR_GREEN = self.config.vision.color_green_id

        # Lap counting parameters
        self.lap_length = self.config.lap_counting.lap_length_m
        self.section_timeout = self.config.lap_counting.section_fallback_timeout_s
        self.emergency_margin = self.config.lap_counting.emergency_lap_margin_m

        # LIDAR corner detection history
        self._prev_avg_left = None
        self._prev_avg_right = None

        # Map usage flag
        self.use_mapping = self.config.mapping.use_mapping

        # Reverse timer
        self.reverse_end_time = 0.0
        self.reverse_speed = 0.0
        self.reverse_steer = 0.0

        # Give emergency shield access to LIDAR for rear clearance
        self.emergency.set_lidar(self.lidar)

        logger.info(f"StateMachine initialized with challenge: {self.challenge}")

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
        self.lap_start_x = self.lap_start_pose.x

        # Register parking slot only if obstacle challenge
        if not self.is_open_challenge:
            self.localization.register_parking_slot()
            parking_geometry = self.localization.parking_bay_geometry
            if parking_geometry:
                self.parking.start(parking_geometry)
                logger.info("Parking geometry registered: %s", parking_geometry)

        # Reset lap counters
        self.lap_count = 0
        self.sections_passed = 0
        self.last_section_time = time.time()
        self.distance_since_lap_start = 0.0
        self.last_traffic_light_color = None
        self.surprise_rule_activated = False

        # Set initial direction from YAML
        self.current_direction = self.config.navigation_matrix.LAP_1_DIRECTION.upper()
        logger.info(f"Lap 1 direction: {self.current_direction}")

        # Reset LIDAR corner detection history
        self._prev_avg_left = None
        self._prev_avg_right = None

        # Apply mapping enable/disable to localization
        self.localization.use_map_correction = self.use_mapping

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
            # ---- IMU confirmation (optional) ----
            # If IMU data is available, confirm that the robot is actually rotating.
            if hasattr(self.localization, 'latest_imu_yaw_rate') and self.localization.latest_imu_yaw_rate is not None:
                yaw_rate = self.localization.latest_imu_yaw_rate
                if abs(yaw_rate) > 0.3:   # rad/s threshold (tune)
                    return True
                else:
                    # LIDAR says corner, but IMU says no rotation → false positive
                    return False
            else:
                # No IMU – rely solely on LIDAR
                return True

        return False

    def _update_lap_count(self):
        """
        Update lap count using LIDAR‑based corner detection with odometry fallback.
        A lap = 8 sections (4 corners + 4 straights).
        """
        pose = self.localization.get_pose()
        if self.lap_start_pose is not None:
            self.distance_since_lap_start = abs(pose.x - self.lap_start_pose.x)

        # 1. Try LIDAR corner detection (now with IMU confirmation)
        if self._detected_corner_via_lidar():
            self.sections_passed += 1
            self.last_section_time = time.time()
            logger.debug(f"Corner detected via LIDAR -> sections_passed={self.sections_passed}")

        # 2. Fallback: if no corner detected for > timeout, use odometry distance
        elif time.time() - self.last_section_time > self.section_timeout:
            section_distance = self.lap_length / 8.0
            if self.distance_since_lap_start > section_distance:
                self.sections_passed += 1
                self.last_section_time = time.time()
                logger.debug(f"Section counted via odometry fallback -> sections_passed={self.sections_passed}")

        # 3. Emergency lap fallback: if sections_passed < 2 and distance is near full lap
        if self.sections_passed < 2 and self.distance_since_lap_start > (self.lap_length - self.emergency_margin):
            logger.warning("Emergency lap fallback triggered – force completing lap")
            self.sections_passed = 8  # force lap completion

        # 4. Check if a full lap (8 sections) has been completed
        if self.sections_passed >= 8:
            self.lap_count += 1
            self.sections_passed = 0
            self.lap_start_pose = pose
            self.lap_start_x = pose.x
            self.distance_since_lap_start = 0.0
            logger.info(f"Lap {self.lap_count} completed (section-based)")

            # Apply surprise rule after trigger lap
            if (self.lap_count == self.config.surprise_rule.trigger_lap and
                self.config.surprise_rule.enabled and not self.surprise_rule_activated):
                self._apply_surprise_rule()

            # Check if all laps completed
            if self.lap_count >= self.config.navigation_matrix.TOTAL_REQUIRED_LAPS:
                logger.info("All %d laps completed!", self.config.navigation_matrix.TOTAL_REQUIRED_LAPS)
                self.state = RobotState.TERMINATION

    def _apply_surprise_rule(self):
        """Apply the surprise rule based on last traffic light color."""
        rule = self.config.surprise_rule
        logger.info(f"Applying surprise rule (trigger lap {rule.trigger_lap})")
        self.surprise_rule_activated = True

        # Determine direction for next lap
        if self.last_traffic_light_color == self.COLOR_GREEN and rule.color_to_continue.upper() == "GREEN":
            # Keep same direction
            new_direction = self.current_direction
            logger.info(f"Last sign = GREEN -> same direction: {new_direction}")
        elif self.last_traffic_light_color == self.COLOR_RED and rule.color_to_reverse.upper() == "RED":
            # Reverse direction
            new_direction = "COUNTER_CLOCKWISE" if self.current_direction == "CLOCKWISE" else "CLOCKWISE"
            logger.info(f"Last sign = RED -> reverse direction: {new_direction}")
        else:
            # Fallback
            if rule.fallback_direction.upper() == "REVERSE":
                new_direction = "COUNTER_CLOCKWISE" if self.current_direction == "CLOCKWISE" else "CLOCKWISE"
            else:
                new_direction = self.current_direction
            logger.info(f"No matching sign or fallback -> direction: {new_direction}")

        self.current_direction = new_direction

        # Execute the 180° turn if direction changed
        if self.current_direction != self.config.navigation_matrix.LAP_1_DIRECTION:
            self.steering.turn_around(speed=rule.turnaround_speed)
        else:
            logger.info("Direction unchanged, no turn needed")

    def _navigate_state(self):
        """Main navigation loop with sensor fusion and control."""
        # ---- 0. Receive IMU data from ESP32 (if available) ----
        sensor_msg = self.serial_bridge.receive(block=False)
        if sensor_msg and sensor_msg.get('type') == 'sensor_data':
            data = sensor_msg.get('data', {})
            if 'imu_yaw_rate' in data:
                self.localization.update_imu_data(data['imu_yaw_rate'])

        # 1. Update emergency shield
        self.emergency.update()
        emergency_actions = self.emergency.get_emergency_actions()

        # ---- Check for reverse command ----
        if emergency_actions.get('reverse', False):
            # Set the reverse timer and store the reverse parameters
            self.reverse_end_time = time.time() + emergency_actions.get('reverse_duration', 1.0)
            self.reverse_speed = emergency_actions.get('reverse_speed', -0.10)
            self.reverse_steer = emergency_actions.get('steer_offset', 0.0)
            logger.info(f"Reverse commanded: speed={self.reverse_speed}, duration={emergency_actions.get('reverse_duration', 1.0)}s")
        elif emergency_actions.get('brake', False):
            self.steering.stop()
            self.state = RobotState.EMERGENCY_STOP
            logger.warning("Emergency stop triggered")
            return

        # ---- 2. If we are currently reversing, override steering ----
        current_time = time.time()
        if current_time < self.reverse_end_time:
            # Still reversing
            linear = self.reverse_speed
            angular = self.reverse_steer
            # Add a small smoothing to angular? Not needed for reverse.
        else:
            # Normal navigation – compute steering as usual
            # 2a. Get color detections from HuskyLens and apply filters
            color_blocks = self.vision.get_latest_colors()
            detections = []
            for color_block in color_blocks:
                # ============================================================
                # TRAFFIC LIGHT FILTERS – reject false positives
                # ============================================================
                # Filter 1: Area (ignore too small / too large)
                if color_block.area < self.MIN_AREA or color_block.area > self.MAX_AREA:
                    logger.debug(f"Block area {color_block.area} rejected (min={self.MIN_AREA}, max={self.MAX_AREA})")
                    continue

                # Filter 2: Aspect Ratio (pillars are tall and narrow)
                if color_block.width == 0:
                    continue
                aspect = color_block.height / color_block.width
                if aspect < self.MIN_ASPECT or aspect > self.MAX_ASPECT:
                    logger.debug(f"Block aspect {aspect:.2f} rejected (min={self.MIN_ASPECT}, max={self.MAX_ASPECT})")
                    continue

                # Filter 3: Y-position (ROI) – ignore top/bottom edges
                if color_block.y < self.ROI_Y_MIN or color_block.y > self.ROI_Y_MAX:
                    logger.debug(f"Block y={color_block.y} rejected (ROI min={self.ROI_Y_MIN}, max={self.ROI_Y_MAX})")
                    continue

                # ============================================================
                # FUSE WITH LIDAR (existing logic)
                # ============================================================
                angle_deg = ((color_block.x - 160) / 160.0) * 30.0
                range_m = self.lidar.get_range_in_sector(angle_deg, 5.0)
                if range_m is not None and range_m > 0.1:
                    angle_rad = math.radians(angle_deg)
                    x_local = range_m * math.cos(angle_rad)
                    y_local = range_m * math.sin(angle_rad)
                    if x_local > 0:
                        detections.append((color_block.color_id, x_local, y_local))

            if detections:
                self.spatial_map.update(detections)

            confirmed_objects = self.spatial_map.get_confirmed_objects()
            traffic_angular = 0.0
            if confirmed_objects:
                target = confirmed_objects[0]
                self.last_traffic_light_color = target.color_id
                traffic_angular = self._get_angular_velocity_from_color(target.color_id)

            # 2b. Wall‑following using LIDAR (adaptive to direction)
            scan = self.lidar.get_scan_snapshot()
            left_dist = None
            right_dist = None
            for ang, dist in scan.items():
                if dist < 0.05 or dist > 5.0:
                    continue
                if abs(ang - 90) < 10:
                    left_dist = dist if left_dist is None or dist < left_dist else left_dist
                elif abs(ang + 90) < 10:
                    right_dist = dist if right_dist is None or dist < right_dist else right_dist

            wall_steer = 0.0
            if left_dist is not None and right_dist is not None:
                error = left_dist - right_dist
                if self.current_direction == "CLOCKWISE":
                    wall_steer = -self.WALL_FOLLOW_GAIN * error
                else:
                    wall_steer = self.WALL_FOLLOW_GAIN * error
                wall_steer = max(-0.5, min(0.5, wall_steer))

            # 2c. Combine traffic light and wall‑following
            if abs(traffic_angular) > 0.01:
                raw_angular = traffic_angular
            else:
                raw_angular = wall_steer

            # 2d. Apply emergency shield corrections (non‑reverse)
            angular = raw_angular + emergency_actions.get('steer_offset', 0.0)
            throttle = emergency_actions.get('throttle_factor', 1.0)
            linear = self.BASE_SPEED * throttle

        # ---- 3. Apply steering ----
        self.steering.set_speed(linear, angular)

        # ---- 4. Update localization with LIDAR points for mapping ----
        scan_data = self.lidar.get_scan_snapshot()
        lidar_scan = [(ang, dist) for ang, dist in scan_data.items()]
        lidar_subset = [(ang, dist) for ang, dist in lidar_scan if abs(ang) % 5 < 1]

        steering_angle = 0.0 if abs(linear) < 0.001 else math.atan2(angular * self.config.vehicle.wheelbase_m, linear)
        self.localization.update_pose(linear, steering_angle, lidar_points=lidar_subset)

        # ---- 5. Update lap counting ----
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
        self.last_traffic_light_color = None
        self.surprise_rule_activated = False
        self.reverse_end_time = 0.0
        logger.info("State machine reset to INIT")
