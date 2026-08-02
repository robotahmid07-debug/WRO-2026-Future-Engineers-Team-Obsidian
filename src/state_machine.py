"""
Global execution FSM (INIT -> NAVIGATE -> TERMINATION).
Supports both Open and Obstacle challenges.

Integrates:
  - Challenge‑specific parameters (Open / Obstacle) from YAML.
  - PID wall‑following controller (KP, KI, KD) with LazyGo defaults.
  - Derivative + Percentage corner detection (dual validation).
  - Graded steering (partial → full steering based on corner strength).
  - Predictive speed control based on error derivative and absolute error.
  - Straight‑line boost (increased speed on straights).
  - IMU G‑force limiting (lateral G safety cap).
  - Surprise rule (Lap X -> Lap X+1 direction change).
  - Emergency lap fallback (force lap completion if LIDAR fails).
  - Map usage control (enable/disable mapping).
  - Reverse handling from emergency shield (with timer).
  - Parking mode activation for emergency shield (during parking).
  - Direct navigation to parking lot using LIDAR‑based localization.
  - All calibratable parameters read from config.
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
    # Traffic light detection filters (tunable for 1.5 m/s)
    MIN_AREA = 120
    MAX_AREA = 18000
    MIN_ASPECT = 1.5
    MAX_ASPECT = 4.0
    ROI_Y_MIN = 30
    ROI_Y_MAX = 210
    CONFIRMATION_FRAMES = 2

    # LIDAR sector tolerance (fixed 10° – efficient and sufficient)
    LIDAR_SECTOR_TOLERANCE = 10.0

    def __init__(self, config: SystemConfig, serial_bridge, localization: Localization,
                 vision_reader: HuskyLensReader, lidar_fusion: LidarFusion,
                 spatial_map: SpatialMap, emergency_shield: EmergencyShield,
                 steering: SteeringController, parking: ParkingController,
                 challenge: str = "open",
                 initial_direction: str = "CLOCKWISE"):
        """
        Initialize the state machine.

        Args:
            config: Parsed SystemConfig object.
            challenge: "open" or "obstacle" – selects which challenge mode to run.
            initial_direction: "CLOCKWISE" or "COUNTER_CLOCKWISE" – from hardware switch.
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

        # Select active challenge config
        self.challenge = challenge.lower()
        if self.challenge == "open":
            self.active_config: ChallengeConfig = config.zone_management.open_challenge
            self.nav_params = config.navigation.open_challenge
        elif self.challenge == "obstacle":
            self.active_config = config.zone_management.obstacle_challenge
            self.nav_params = config.navigation.obstacle_challenge
        else:
            raise ValueError(f"Invalid challenge: {challenge}")

        self.is_open_challenge = not self.active_config.use_parking_slot

        # ---- State tracking ----
        self.state = RobotState.INIT
        self.lap_count = 0
        self.lap_start_pose = None
        self.sections_passed = 0
        self.last_section_time = time.time()
        self.distance_since_lap_start = 0.0
        self.lap_start_x = 0.0

        # ---- Direction tracking ----
        self.current_direction = initial_direction.upper()
        self.last_traffic_light_color = None
        self.surprise_rule_activated = False
        logger.info(f"Direction set by hardware switch: {self.current_direction}")

        # ---- Read parameters from config (challenge‑specific) ----
        self.BASE_SPEED = self.nav_params.base_speed_mps
        self.WALL_FOLLOW_GAIN = self.nav_params.wall_follow_gain  # kept for reference (fallback if PID disabled)
        self.STEER_MAGNITUDE = self.nav_params.steer_magnitude_radps
        self.STRAIGHT_BOOST = self.nav_params.straight_boost_factor
        self.CORNER_SLOWDOWN_MAX = self.nav_params.corner_slowdown_max_reduction
        self.PREDICTIVE_GAIN = self.nav_params.predictive_slowdown_gain

        # ---- PID gains (LazyGo defaults) ----
        self.KP = self.nav_params.pid.kp
        self.KI = self.nav_params.pid.ki
        self.KD = self.nav_params.pid.kd
        self.pid_integral = 0.0
        self.prev_pid_error = 0.0
        self.pid_dt = 0.05  # 20 Hz loop

        # ---- Corner detection params ----
        self.cd = self.nav_params.corner_detection
        self.use_derivative = self.cd.use_derivative
        self.use_percentage = self.cd.use_percentage
        self.use_graded = self.cd.use_graded_steering
        self.use_imu = self.cd.use_imu_confirmation
        self.deriv_thresh = self.cd.lidar_derivative_threshold
        self.pct_thresh = self.cd.pct_threshold
        self.imu_thresh = self.cd.imu_confirm_threshold_radps

        # ---- G‑force params ----
        self.MAX_SAFE_G = self.nav_params.g_force.max_safe_g
        self.G_FILTER_ALPHA = self.nav_params.g_force.filter_alpha

        # ---- Traffic light params (Obstacle only) ----
        if self.challenge == "obstacle":
            self.tl = self.nav_params.traffic_light
        else:
            self.tl = None

        # ---- Color IDs ----
        self.COLOR_RED = self.config.vision.color_red_id
        self.COLOR_GREEN = self.config.vision.color_green_id

        # ---- Lap counting ----
        self.lap_length = self.config.lap_counting.lap_length_m
        self.section_timeout = self.config.lap_counting.section_fallback_timeout_s
        self.emergency_margin = self.config.lap_counting.emergency_lap_margin_m

        # ---- LIDAR corner detection history ----
        self._prev_avg_left = None
        self._prev_avg_right = None
        self.prev_error = 0.0
        self.error_derivative = 0.0
        self.prev_left_dist = None
        self.prev_right_dist = None

        # ---- Map usage ----
        self.use_mapping = self.config.mapping.use_mapping

        # ---- Reverse timer ----
        self.reverse_end_time = 0.0
        self.reverse_speed = 0.0
        self.reverse_steer = 0.0

        # ---- G‑force filter ----
        self.filtered_G = 0.0

        # Give emergency shield access to LIDAR
        self.emergency.set_lidar(self.lidar)

        # Override spatial map confirmation threshold
        self.spatial_map.confirmation_threshold = self.CONFIRMATION_FRAMES

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

        # Reset PID state
        self.pid_integral = 0.0
        self.prev_pid_error = 0.0
        self.prev_error = 0.0
        self.error_derivative = 0.0
        self.prev_left_dist = None
        self.prev_right_dist = None

        # Direction is already set from hardware switch
        logger.info(f"Lap 1 direction: {self.current_direction}")

        # Reset LIDAR corner detection history
        self._prev_avg_left = None
        self._prev_avg_right = None

        # Apply mapping enable/disable to localization
        self.localization.use_map_correction = self.use_mapping

        self.state = RobotState.NAVIGATE
        logger.info("Transition to NAVIGATE")

    def _get_angular_velocity_from_color(self, color_id: int) -> float:
        """Return angular velocity based on color ID and YAML rules."""
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
            return 0.0

    def _compute_wall_steer(self) -> float:
        """
        Compute wall‑following steering using PID (or fallback P‑only if PID not used).
        Uses LIDAR side distances to calculate error and apply PID.
        """
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

        if left_dist is not None and right_dist is not None:
            error = left_dist - right_dist

            # ---- PID controller ----
            error_derivative = (error - self.prev_pid_error) / self.pid_dt
            self.pid_integral += error * self.pid_dt
            # Anti‑windup clamp
            self.pid_integral = max(-0.5, min(0.5, self.pid_integral))
            wall_steer = (self.KP * error) + (self.KI * self.pid_integral) + (self.KD * error_derivative)
            wall_steer = max(-0.5, min(0.5, wall_steer))
            self.prev_pid_error = error

            return wall_steer
        else:
            return 0.0

    def _update_lap_count(self):
        """Update lap count using LIDAR‑based corner detection with odometry fallback."""
        pose = self.localization.get_pose()
        if self.lap_start_pose is not None:
            self.distance_since_lap_start = abs(pose.x - self.lap_start_pose.x)

        # Corner detection is handled inside _navigate_state()
        # The sections_passed is incremented there.

        # Emergency lap fallback
        if self.sections_passed < 2 and self.distance_since_lap_start > (self.lap_length - self.emergency_margin):
            logger.warning("Emergency lap fallback triggered – force completing lap")
            self.sections_passed = 8

        if self.sections_passed >= 8:
            self.lap_count += 1
            self.sections_passed = 0
            self.lap_start_pose = pose
            self.lap_start_x = pose.x
            self.distance_since_lap_start = 0.0
            logger.info(f"Lap {self.lap_count} completed")

            # Apply surprise rule after trigger lap
            if (self.lap_count == self.config.surprise_rule.trigger_lap and
                self.config.surprise_rule.enabled and not self.surprise_rule_activated):
                self._apply_surprise_rule()

            if self.lap_count >= self.config.navigation_matrix.TOTAL_REQUIRED_LAPS:
                logger.info("All laps completed!")
                self.state = RobotState.TERMINATION

    def _apply_surprise_rule(self):
        """Apply the surprise rule based on last traffic light color."""
        rule = self.config.surprise_rule
        logger.info(f"Applying surprise rule (trigger lap {rule.trigger_lap})")
        self.surprise_rule_activated = True

        if self.last_traffic_light_color == self.COLOR_GREEN and rule.color_to_continue.upper() == "GREEN":
            new_direction = self.current_direction
            logger.info(f"Last sign = GREEN -> same direction: {new_direction}")
        elif self.last_traffic_light_color == self.COLOR_RED and rule.color_to_reverse.upper() == "RED":
            new_direction = "COUNTER_CLOCKWISE" if self.current_direction == "CLOCKWISE" else "CLOCKWISE"
            logger.info(f"Last sign = RED -> reverse direction: {new_direction}")
        else:
            if rule.fallback_direction.upper() == "REVERSE":
                new_direction = "COUNTER_CLOCKWISE" if self.current_direction == "CLOCKWISE" else "CLOCKWISE"
            else:
                new_direction = self.current_direction
            logger.info(f"No matching sign -> direction: {new_direction}")

        self.current_direction = new_direction

        # Execute U‑turn if direction changed (use IMU if available)
        if self.current_direction != self.config.navigation_matrix.LAP_1_DIRECTION:
            self.steering.turn_around_imu(speed=rule.turnaround_speed)
        else:
            logger.info("Direction unchanged, no turn needed")

    def _navigate_state(self):
        """Main navigation loop with sensor fusion and control."""
        # ---- 0. Receive IMU data from ESP32 ----
        sensor_msg = self.serial_bridge.receive(block=False)
        if sensor_msg and sensor_msg.get('type') == 'sensor_data':
            data = sensor_msg.get('data', {})
            if 'imu_yaw_rate' in data:
                self.localization.update_imu_data(data['imu_yaw_rate'])

        # ---- 1. Emergency shield ----
        self.emergency.update()
        emergency_actions = self.emergency.get_emergency_actions()

        if emergency_actions.get('reverse', False):
            self.reverse_end_time = time.time() + emergency_actions.get('reverse_duration', 1.0)
            self.reverse_speed = emergency_actions.get('reverse_speed', -0.10)
            self.reverse_steer = emergency_actions.get('steer_offset', 0.0)
            logger.info(f"Reverse commanded: speed={self.reverse_speed}")
        elif emergency_actions.get('brake', False):
            self.steering.stop()
            self.state = RobotState.EMERGENCY_STOP
            logger.warning("Emergency stop triggered")
            return

        # ---- 2. Check if reversing ----
        current_time = time.time()
        if current_time < self.reverse_end_time:
            linear = self.reverse_speed
            angular = self.reverse_steer
            logger.debug(f"Reversing: speed={linear}, steer={angular}")
        else:
            # ---- Normal navigation ----

            # ---- 2a. HuskyLens detections with filters ----
            color_blocks = self.vision.get_latest_colors()
            detections = []
            for cb in color_blocks:
                if cb.area < self.MIN_AREA or cb.area > self.MAX_AREA:
                    continue
                if cb.width == 0:
                    continue
                aspect = cb.height / cb.width
                if cb.y < self.ROI_Y_MIN or cb.y > self.ROI_Y_MAX:
                    continue

                angle_deg = ((cb.x - 160) / 160.0) * 30.0
                range_m = self.lidar.get_range_in_sector(angle_deg, self.LIDAR_SECTOR_TOLERANCE)
                if range_m is not None and range_m > 0.1:
                    if range_m > 0.8:
                        min_aspect, max_aspect = 1.5, 4.0
                    elif range_m > 0.4:
                        min_aspect, max_aspect = 1.2, 4.5
                    else:
                        min_aspect, max_aspect = 1.0, 5.0
                    if aspect < min_aspect or aspect > max_aspect:
                        continue
                    angle_rad = math.radians(angle_deg)
                    x_local = range_m * math.cos(angle_rad)
                    y_local = range_m * math.sin(angle_rad)
                    if x_local > 0:
                        detections.append((cb.color_id, x_local, y_local))

            # ---- 2b. Confirmed objects & traffic angular ----
            confirmed = self.spatial_map.get_confirmed_objects()
            traffic_angular = 0.0
            dist_to_pillar = None
            if confirmed:
                target = confirmed[0]
                self.last_traffic_light_color = target.color_id
                dist_to_pillar = target.local_y
                traffic_angular = self._get_angular_velocity_from_color(target.color_id)
                self.emergency.set_target_steer_direction(
                    math.copysign(1.0, traffic_angular) if traffic_angular != 0 else 0.0
                )
            else:
                self.emergency.set_target_steer_direction(0.0)

            # ---- 2c. Wall‑following (PID) ----
            wall_steer = self._compute_wall_steer()
            # For corner detection, we also need the error and side deltas
            # We'll compute these inside _compute_wall_steer? We'll do it here.

            # Re‑compute LIDAR distances for corner detection
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

            error = 0.0
            if left_dist is not None and right_dist is not None:
                error = left_dist - right_dist

                # ---- Corner detection (dual validation) ----
                if self.prev_left_dist is not None and self.prev_right_dist is not None:
                    left_delta = left_dist - self.prev_left_dist
                    right_delta = right_dist - self.prev_right_dist

                    # Derivative trigger
                    deriv_trigger = False
                    if self.use_derivative:
                        deriv_trigger = (abs(left_delta) > self.deriv_thresh or
                                         abs(right_delta) > self.deriv_thresh)

                    # Percentage trigger (40%)
                    pct_trigger = False
                    if self.use_percentage:
                        left_pct = abs(left_delta) / max(self.prev_left_dist, 0.1)
                        right_pct = abs(right_delta) / max(self.prev_right_dist, 0.1)
                        pct_trigger = (left_pct > self.pct_thresh or right_pct > self.pct_thresh)

                    # IMU confirmation
                    imu_confirmed = False
                    if self.use_imu and hasattr(self.localization, 'imu_available') and self.localization.imu_available:
                        yaw_rate = self.localization.latest_imu_yaw_rate
                        imu_confirmed = (abs(yaw_rate) > self.imu_thresh)

                    # Combined trigger
                    corner_detected = deriv_trigger or pct_trigger

                    if corner_detected:
                        # Graded steering
                        if self.use_graded:
                            # Compute corner strength
                            corner_strength = max(abs(left_delta), abs(right_delta), left_pct, right_pct)
                            steer_multiplier = min(1.0, corner_strength / 0.5)
                            steer_multiplier = max(0.3, steer_multiplier)
                            if imu_confirmed:
                                steer_multiplier = 1.0
                            wall_steer = wall_steer * steer_multiplier

                        # Count the corner
                        self.sections_passed += 1
                        self.last_section_time = time.time()
                        logger.debug(f"Corner detected (deriv={deriv_trigger}, pct={pct_trigger}) -> sections={self.sections_passed}")

                # Store previous distances
                self.prev_left_dist = left_dist
                self.prev_right_dist = right_dist

                # Compute error derivative for predictive speed
                if self.prev_error is not None:
                    dt = 0.05  # fixed 20 Hz
                    self.error_derivative = (error - self.prev_error) / dt
                self.prev_error = error

            # ---- 2d. Combine traffic light and wall‑following ----
            if abs(traffic_angular) > 0.01:
                raw_angular = traffic_angular
            else:
                raw_angular = wall_steer

            # ---- 2e. Apply emergency shield corrections ----
            angular = raw_angular + emergency_actions.get('steer_offset', 0.0)
            throttle = emergency_actions.get('throttle_factor', 1.0)

            # ---- 2f. Speed reduction based on steering intensity ----
            steer_intensity = abs(raw_angular)
            steer_factor = 1.0 - (steer_intensity / 0.5) * self.CORNER_SLOWDOWN_MAX
            steer_factor = max(0.6, min(1.0, steer_factor))

            # ---- 2g. Predictive speed control (error derivative) ----
            abs_error = abs(error)
            deriv_factor = 1.0 - min(abs(self.error_derivative) / 0.5, 0.4)
            error_factor = 1.0 - min(abs_error / 0.4, 0.3)
            predictive_factor = min(deriv_factor, error_factor)
            predictive_factor = max(0.6, min(1.0, predictive_factor))

            # ---- 2h. Distance‑based slowdown (traffic light) ----
            distance_factor = 1.0
            if self.challenge == "obstacle" and dist_to_pillar is not None and dist_to_pillar > 0:
                if dist_to_pillar < self.tl.distance_slowdown_start_m:
                    distance_factor = dist_to_pillar / self.tl.distance_slowdown_start_m
                    distance_factor = max(self.tl.distance_slowdown_min_factor, distance_factor)
                    distance_factor = min(1.0, distance_factor)

                # LIDAR confirmation (safety)
                if dist_to_pillar > 0:
                    angle_deg = math.degrees(math.atan2(target.local_x, target.local_y))
                    range_m = self.lidar.get_range_in_sector(angle_deg, 10.0)
                    if range_m is None or range_m > self.tl.lidar_confirm_range_m:
                        distance_factor = 1.0
                        traffic_angular = 0.0
                        angular = raw_angular + emergency_actions.get('steer_offset', 0.0)

            # ---- 2i. Straight‑line boost ----
            boost_factor = 1.0
            if abs_error < 0.05 and abs(self.error_derivative) < 0.01:
                boost_factor = self.STRAIGHT_BOOST

            # ---- 2j. Final speed ----
            combined_factor = steer_factor * predictive_factor * distance_factor
            linear = self.BASE_SPEED * throttle * combined_factor * boost_factor

            # Clamp to max speed
            linear = min(linear, self.config.vehicle.max_speed_mps)

            # ---- 2k. IMU G‑force limiting ----
            if hasattr(self.localization, 'imu_available') and self.localization.imu_available:
                yaw_rate = self.localization.latest_imu_yaw_rate
                if yaw_rate is not None:
                    lateral_G = abs(linear * yaw_rate) / 9.81
                    self.filtered_G = self.G_FILTER_ALPHA * lateral_G + (1 - self.G_FILTER_ALPHA) * self.filtered_G
                    if self.filtered_G > self.MAX_SAFE_G:
                        linear *= self.MAX_SAFE_G / self.filtered_G
                        logger.debug(f"G‑force limiting: {self.filtered_G:.2f}G -> speed reduced")

            # ---- 2l. Update spatial map with velocity‑compensated pruning ----
            if detections:
                self.spatial_map.update(detections, robot_speed=linear, dt=0.05)

        # ---- 3. Apply steering ----
        self.steering.set_speed(linear, angular)

        # ---- 4. Update localization with LIDAR points for mapping ----
        scan_data = self.lidar.get_scan_snapshot()
        lidar_subset = [(ang, dist) for ang, dist in scan_data.items() if abs(ang) % 5 < 1]
        steering_angle = 0.0 if abs(linear) < 0.001 else math.atan2(angular * self.config.vehicle.wheelbase_m, linear)
        self.localization.update_pose(linear, steering_angle, lidar_points=lidar_subset)

        # ---- 5. Update lap counting ----
        self._update_lap_count()

    def _termination_state(self):
        """Handle termination: open challenge stop or obstacle parking with direct navigation."""
        logger.info("State: TERMINATION")

        if self.is_open_challenge:
            self.steering.stop()
            logger.info("Open challenge complete")
            return

        # ---- Obstacle challenge: navigate to parking spot ----
        current_pose = self.localization.get_pose()
        start_pose = self.localization.get_start_pose()

        if start_pose is None:
            logger.error("Start pose not registered – cannot park!")
            self.steering.stop()
            return

        # Distance and heading to target (parking spot)
        dx = start_pose.x - current_pose.x
        dy = start_pose.y - current_pose.y
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)

        # Heading error (normalised to [-pi, pi])
        heading_error = target_heading - current_pose.theta
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # If close enough and facing correct direction, start parking
        if distance < 0.3 and abs(heading_error) < math.pi / 6:
            logger.info(f"Arrived at parking spot (dist={distance:.2f}m) – starting parking")
            self._start_parking()
            return

        # ---- Drive toward parking spot ----
        logger.info(f"Driving to parking spot (dist={distance:.2f}m, heading error={math.degrees(heading_error):.1f}°)")

        # Compute wall‑following steering (PID)
        wall_steer = self._compute_wall_steer()

        # Steering correction to align with target heading
        # Simple proportional control
        steer_correction = 0.5 * heading_error
        steer_correction = max(-0.3, min(0.3, steer_correction))

        # Blend: 70% navigation, 30% wall‑following for stability
        final_steer = 0.7 * steer_correction + 0.3 * wall_steer
        final_steer = max(-0.4, min(0.4, final_steer))

        # Speed: reduce as we get closer
        if distance < 0.8:
            speed = 0.15   # crawl
        else:
            speed = 0.3

        self.steering.set_speed(speed, final_steer)

    def _start_parking(self):
        """Start the parking sequence."""
        self.emergency.set_parking_mode(True)
        if not self.parking.is_complete() and not self.parking.is_aborted():
            self.parking.update()
        else:
            self.steering.stop()
            self.emergency.set_parking_mode(False)
            if self.parking.is_complete():
                logger.info("Parking complete")
            elif self.parking.is_aborted():
                logger.warning("Parking aborted")

    def _emergency_stop(self):
        self.steering.stop()
        logger.warning("Emergency stop")

    def reset(self):
        self.state = RobotState.INIT
        self.lap_count = 0
        self.sections_passed = 0
        self.lap_start_pose = None
        self.prev_left_dist = None
        self.prev_right_dist = None
        self.prev_error = 0.0
        self.error_derivative = 0.0
        self.pid_integral = 0.0
        self.prev_pid_error = 0.0
        self.filtered_G = 0.0
        self.last_traffic_light_color = None
        self.surprise_rule_activated = False
        self.reverse_end_time = 0.0
        logger.info("State machine reset to INIT")
