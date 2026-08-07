"""
Global execution FSM (INIT -> NAVIGATE -> TERMINATION).
Supports both Open and Obstacle challenges.

Integrates all Phase 1 + Phase 2 features plus:
  - Corner / section snap (LOGIC A)
  - Lap‑end geometry refresh (LOGIC B)
  - Live geometry re‑scan during approach (LOGIC C)
  - Direction‑aware reverse steer (uses LOGIC D via parking controller)
  - Ultrasonic + LIDAR fusion (LOGIC E via parking controller)
  - True parallel + inside check (LOGIC F via parking controller)
  - Enhanced touch avoidance (LOGIC G via parking controller)
  - Camera FOV corrected (116° lens) using vision.pixel_to_angle()
  - Optional camera‑LIDAR offset correction (toggleable)
"""
# Hi
import time
import math
import logging
from enum import Enum
from typing import Optional

from .config_parser import SystemConfig, ChallengeConfig
from .localization import Localization
from .vision_tracker import HuskyLensReader
from .lidar_fusion import LidarFusion
from .spatial_map import SpatialMap
from .emergency_shield import EmergencyShield
from .steering_controller import SteeringController
from .parking_controller import ParkingController

logger = logging.getLogger(__name__)


class RobotState(Enum):
    INIT = 1
    NAVIGATE = 2
    TERMINATION = 3
    EMERGENCY_STOP = 4
    SEARCH_PARKING = 5


class StateMachine:
    # Traffic light detection filters
    MIN_AREA = 120
    MAX_AREA = 18000
    MIN_ASPECT = 1.5
    MAX_ASPECT = 4.0
    ROI_Y_MIN = 30
    ROI_Y_MAX = 210
    CONFIRMATION_FRAMES = 2
    LIDAR_SECTOR_TOLERANCE = 10.0

    # Parking fine‑tune thresholds
    PARKING_FINE_TUNE_THRESHOLD = 0.35   # 35 cm → start parking sequence
    PARKING_DRIVE_THRESHOLD = 1.2        # if farther than this, drive normally (and re‑scan)

    def __init__(self, config: SystemConfig, serial_bridge, localization: Localization,
                 vision_reader: HuskyLensReader, lidar_fusion: LidarFusion,
                 spatial_map: SpatialMap, emergency_shield: EmergencyShield,
                 steering: SteeringController, parking: ParkingController,
                 challenge: str = "open",
                 initial_direction: str = "CLOCKWISE"):
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
        logger.info(f"Direction set: {self.current_direction}")

        # ---- Read parameters from config ----
        self.BASE_SPEED = self.nav_params.base_speed_mps
        self.WALL_FOLLOW_GAIN = self.nav_params.wall_follow_gain
        self.STEER_MAGNITUDE = self.nav_params.steer_magnitude_radps
        self.STRAIGHT_BOOST = self.nav_params.straight_boost_factor
        self.CORNER_SLOWDOWN_MAX = self.nav_params.corner_slowdown_max_reduction
        self.PREDICTIVE_GAIN = self.nav_params.predictive_slowdown_gain

        # ---- PID gains ----
        self.KP = self.nav_params.pid.kp
        self.KI = self.nav_params.pid.ki
        self.KD = self.nav_params.pid.kd
        self.pid_integral = 0.0
        self.prev_pid_error = 0.0

        # ---- Corner detection ----
        self.cd = self.nav_params.corner_detection
        self.use_derivative = self.cd.use_derivative
        self.use_percentage = self.cd.use_percentage
        self.use_graded = self.cd.use_graded_steering
        self.use_imu = self.cd.use_imu_confirmation
        self.deriv_thresh = self.cd.lidar_derivative_threshold
        self.pct_thresh = self.cd.pct_threshold
        self.imu_thresh = self.cd.imu_confirm_threshold_radps

        # ---- G‑force ----
        self.MAX_SAFE_G = self.nav_params.g_force.max_safe_g
        self.G_FILTER_ALPHA = self.nav_params.g_force.filter_alpha

        # ---- Traffic light (Obstacle only) ----
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

        # ---- LIDAR history ----
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

        # ---- Dynamic PID dt & speed ----
        self.last_loop_time = time.time()
        self.current_speed = 0.0

        # ---- Speed control (from YAML) ----
        self.cruise_speed_mps = config.navigation.cruise_speed_mps
        self.max_accel = config.navigation.max_accel_mps2
        self.max_decel = config.navigation.max_decel_mps2
        self.enable_lookahead_braking = config.navigation.enable_lookahead_braking
        self.corner_strength_ref = config.navigation.corner_strength_ref

        # ---- Phase 2: Search parameters ----
        self.search_start_time = 0.0
        self.search_phase = "FORWARD"  # FORWARD, REVERSE, TURN
        self.last_lap_geometry_update = 0.0

        # Give emergency shield access to LIDAR
        self.emergency.set_lidar(self.lidar)

        # Pass emergency shield to parking controller for ultrasonic fusion
        self.parking.set_emergency_shield(self.emergency)

        self.spatial_map.confirmation_threshold = self.CONFIRMATION_FRAMES

        logger.info(f"StateMachine initialized with challenge: {self.challenge}")
        logger.info(f"Speed control: cruise={self.cruise_speed_mps}m/s, "
                    f"accel={self.max_accel}m/s², decel={self.max_decel}m/s²")

    # ==========================================================
    # Helpers
    # ==========================================================

    def _get_left_right_distances(self, scan):
        left_dist = None
        right_dist = None
        for ang, dist in scan.items():
            if dist < 0.05 or dist > 5.0:
                continue
            if abs(ang - 90) < 10:
                if left_dist is None or dist < left_dist:
                    left_dist = dist
            elif abs(ang + 90) < 10:
                if right_dist is None or dist < right_dist:
                    right_dist = dist
        return left_dist, right_dist

    def _get_front_distance(self, scan, half_width_deg: float = 10.0) -> Optional[float]:
        front_dists = []
        for ang, dist in scan.items():
            if dist < 0.05 or dist > 5.0:
                continue
            if abs(ang) < half_width_deg:
                front_dists.append(dist)
        return min(front_dists) if front_dists else None

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-π, π]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _apply_corner_snap(self):
        """LOGIC A – Corner / section snap."""
        if not self.use_imu:
            return
        map_pose = self.localization.get_map_pose()
        if map_pose is None:
            return
        # Hard snap position (70% map, 30% odometry)
        self.localization.current_pose.x = 0.7 * map_pose.x + 0.3 * self.localization.current_pose.x
        self.localization.current_pose.y = 0.7 * map_pose.y + 0.3 * self.localization.current_pose.y
        # Heading only if within 25°
        dtheta = self._normalize_angle(map_pose.theta - self.localization.current_pose.theta)
        if abs(dtheta) < 0.44:  # 25°
            self.localization.current_pose.theta += 0.5 * dtheta

    def _refresh_parking_geometry(self):
        """LOGIC B – Force geometry refresh at lap end."""
        if self.is_open_challenge:
            return
        # Force wall‑mapper correction with higher weight (α = 0.6)
        map_pose = self.localization.get_map_pose()
        if map_pose is not None:
            self.localization.current_pose.x = 0.6 * map_pose.x + 0.4 * self.localization.current_pose.x
            self.localization.current_pose.y = 0.6 * map_pose.y + 0.4 * self.localization.current_pose.y
            dtheta = self._normalize_angle(map_pose.theta - self.localization.current_pose.theta)
            self.localization.current_pose.theta += 0.5 * dtheta

        # Re‑detect parking geometry
        geometry = self.localization.scan_for_parking_markers()
        if geometry:
            self.localization.parking_bay_geometry = geometry
            logger.info(f"Parking geometry refreshed at lap end: {geometry}")
            # CRITICAL FIX: Only call parking.start() if we are actually in TERMINATION state
            # This prevents premature parking stage resets mid-race
            if self.state == RobotState.TERMINATION:
                self.parking.start(geometry)
                logger.info("Parking controller restarted with refreshed geometry")
            self.last_lap_geometry_update = time.time()

    # ==========================================================
    # Public interface
    # ==========================================================

    def run(self):
        try:
            if self.state == RobotState.INIT:
                self._init_state()
            elif self.state == RobotState.NAVIGATE:
                self._navigate_state()
            elif self.state == RobotState.TERMINATION:
                self._termination_state()
            elif self.state == RobotState.EMERGENCY_STOP:
                self._emergency_stop()
            elif self.state == RobotState.SEARCH_PARKING:
                self._search_parking_state()
        except Exception as e:
            logger.exception("State machine error: %s", e)
            self.steering.stop()

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
        self.last_loop_time = time.time()
        self.current_speed = 0.0
        self.search_phase = "FORWARD"
        self.search_start_time = 0.0
        self.last_lap_geometry_update = 0.0
        if hasattr(self.emergency, 'reset'):
            self.emergency.reset()
        if hasattr(self.parking, 'reset'):
            self.parking.reset()
        logger.info("State machine reset to INIT")

    # ==========================================================
    # State handlers
    # ==========================================================

    def _init_state(self):
        logger.info("State: INIT")
        self.localization.register_start_pose()
        self.lap_start_pose = self.localization.get_pose()
        self.lap_start_x = self.lap_start_pose.x

        if not self.is_open_challenge:
            self.localization.register_parking_slot()
            parking_geometry = self.localization.parking_bay_geometry
            if parking_geometry:
                self.parking.start(parking_geometry)
                logger.info("Parking geometry registered: %s", parking_geometry)

        self.lap_count = 0
        self.sections_passed = 0
        self.last_section_time = time.time()
        self.distance_since_lap_start = 0.0
        self.last_traffic_light_color = None
        self.surprise_rule_activated = False

        self.pid_integral = 0.0
        self.prev_pid_error = 0.0
        self.prev_error = 0.0
        self.error_derivative = 0.0
        self.prev_left_dist = None
        self.prev_right_dist = None

        logger.info(f"Lap 1 direction: {self.current_direction}")
        self._prev_avg_left = None
        self._prev_avg_right = None
        self.localization.use_map_correction = self.use_mapping

        self.state = RobotState.NAVIGATE
        logger.info("Transition to NAVIGATE")

    def _get_angular_velocity_from_color(self, color_id: int) -> float:
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

    def _compute_wall_steer(self, scan, dt: float):
        left_dist, right_dist = self._get_left_right_distances(scan)
        if left_dist is not None and right_dist is not None:
            corridor_width = max(left_dist + right_dist, 0.30)
            error = (left_dist - right_dist) / corridor_width
            error_derivative = (error - self.prev_pid_error) / dt if dt > 0 else 0.0
            if dt < 0.2:
                self.pid_integral += error * dt
            self.pid_integral = max(-0.5, min(0.5, self.pid_integral))
            wall_steer = (self.KP * error) + (self.KI * self.pid_integral) + (self.KD * error_derivative)
            wall_steer = max(-0.5, min(0.5, wall_steer))
            self.prev_pid_error = error
            return wall_steer, left_dist, right_dist
        return 0.0, None, None

    def _feedforward_steer(self, scan) -> float:
        left_dist, right_dist = self._get_left_right_distances(scan)
        if left_dist is None or right_dist is None:
            return 0.0
        diff = right_dist - left_dist
        corridor_width = max(left_dist + right_dist, 0.30)
        curvature = diff / (corridor_width * corridor_width)
        ff_steer = math.atan(self.config.vehicle.wheelbase_m * curvature)
        return max(-0.3, min(0.3, ff_steer))

    def _compute_target_speed(self, scan, corner_detected: bool, corner_strength: float) -> float:
        base_speed = self.cruise_speed_mps
        if not self.enable_lookahead_braking:
            return base_speed

        front_dist = self._get_front_distance(scan)
        if front_dist is None or front_dist < 0.05:
            return max(0.3, base_speed * 0.4)

        braking_dist = (self.current_speed ** 2) / (2 * max(self.max_decel, 0.1))
        if front_dist < braking_dist * 1.3:
            speed_factor = max(0.35, front_dist / (braking_dist * 1.3))
            target = base_speed * speed_factor
        elif corner_detected:
            severity = min(corner_strength / max(self.corner_strength_ref, 0.01), 1.0)
            target = base_speed * (1.0 - 0.5 * severity)
        else:
            target = base_speed

        max_delta = self.max_accel * 0.05
        if target > self.current_speed:
            target = min(target, self.current_speed + max_delta)
        else:
            target = max(target, self.current_speed - max_delta * 1.5)
        return max(0.3, min(base_speed * 1.1, target))

    def _update_parking_geometry(self):
        """Try to update parking geometry using a fresh LIDAR scan.
           Does NOT call parking.start() unless in TERMINATION state.
        """
        if self.is_open_challenge:
            return
        geometry = self.localization.scan_for_parking_markers()
        if geometry:
            self.localization.parking_bay_geometry = geometry
            logger.info(f"Parking geometry updated: {geometry}")
            # Only restart parking controller if we are actually in parking mode
            if self.state == RobotState.TERMINATION:
                self.parking.start(geometry)
                logger.info("Parking controller restarted with updated geometry")
            self.last_lap_geometry_update = time.time()

    def _update_lap_count(self):
        pose = self.localization.get_pose()
        if self.lap_start_pose is not None:
            self.distance_since_lap_start = abs(pose.x - self.lap_start_pose.x)

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

            # ---- LOGIC B: Lap‑end refresh ----
            if not self.is_open_challenge and self.lap_count >= 1:
                self._refresh_parking_geometry()

            if (self.lap_count == self.config.surprise_rule.trigger_lap and
                    self.config.surprise_rule.enabled and
                    not self.surprise_rule_activated):
                self._apply_surprise_rule()

            if self.lap_count >= self.config.navigation_matrix.TOTAL_REQUIRED_LAPS:
                logger.info("All laps completed!")
                self.state = RobotState.TERMINATION

    def _apply_surprise_rule(self):
        rule = self.config.surprise_rule
        logger.info(f"Applying surprise rule (trigger lap {rule.trigger_lap})")
        self.surprise_rule_activated = True

        if self.last_traffic_light_color == self.COLOR_GREEN and rule.color_to_continue.upper() == "GREEN":
            new_direction = self.current_direction
        elif self.last_traffic_light_color == self.COLOR_RED and rule.color_to_reverse.upper() == "RED":
            new_direction = "COUNTER_CLOCKWISE" if self.current_direction == "CLOCKWISE" else "CLOCKWISE"
        else:
            if rule.fallback_direction.upper() == "REVERSE":
                new_direction = "COUNTER_CLOCKWISE" if self.current_direction == "CLOCKWISE" else "CLOCKWISE"
            else:
                new_direction = self.current_direction

        self.current_direction = new_direction
        if self.current_direction != self.config.navigation_matrix.LAP_1_DIRECTION:
            self.steering.turn_around_imu(speed=rule.turnaround_speed)
        else:
            logger.info("Direction unchanged, no turn needed")

    def _navigate_state(self):
        current_time = time.time()
        dt = current_time - self.last_loop_time
        self.last_loop_time = current_time

        sensor_msg = self.serial_bridge.receive(block=False)
        if sensor_msg and sensor_msg.get('type') == 'sensor_data':
            data = sensor_msg.get('data', {})
            if 'imu_yaw_rate' in data:
                self.localization.update_imu_data(data['imu_yaw_rate'])
        else:
            latest = self.serial_bridge.get_latest_sensor_data()
            if latest and isinstance(latest, dict) and 'imu_yaw_rate' in latest:
                self.localization.update_imu_data(latest['imu_yaw_rate'])

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

        if current_time < self.reverse_end_time:
            linear = self.reverse_speed
            angular = self.reverse_steer
            self.current_speed = linear
        else:
            scan = self.lidar.get_scan_snapshot()

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

                # ---- FOV FIX: use camera's actual 116° FOV ----
                angle_deg = self.vision.pixel_to_angle(cb.x)

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

                    # ---- Optional camera-LIDAR offset correction ----
                    # Enable this if you want to compensate for the physical offset
                    # between the camera and LIDAR on your car.
                    ENABLE_OFFSET_CORRECTION = True   # Set to False to disable

                    # Camera is 0.10972 m AHEAD of LIDAR (forward offset)
                    CAMERA_OFFSET_X = 0.10972   # meters
                    CAMERA_OFFSET_Z = 0.0       # lateral offset is zero

                    if ENABLE_OFFSET_CORRECTION:
                        x_local_corrected = x_local - CAMERA_OFFSET_X
                        y_local_corrected = y_local   # no lateral correction
                    else:
                        x_local_corrected = x_local
                        y_local_corrected = y_local

                    if x_local_corrected > 0:
                        detections.append((cb.color_id, x_local_corrected, y_local_corrected))

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

            wall_steer, left_dist, right_dist = self._compute_wall_steer(scan, dt)

            error = 0.0
            corner_detected = False
            corner_strength = 0.0

            if left_dist is not None and right_dist is not None:
                error = left_dist - right_dist
                if self.prev_left_dist is not None and self.prev_right_dist is not None:
                    left_delta = left_dist - self.prev_left_dist
                    right_delta = right_dist - self.prev_right_dist
                    left_pct = 0.0
                    right_pct = 0.0
                    deriv_trigger = False
                    if self.use_derivative:
                        left_rate = left_delta / dt if dt > 0 else 0
                        right_rate = right_delta / dt if dt > 0 else 0
                        deriv_trigger = (abs(left_rate) > self.deriv_thresh or
                                         abs(right_rate) > self.deriv_thresh)
                    pct_trigger = False
                    if self.use_percentage:
                        left_pct = abs(left_delta) / max(self.prev_left_dist, 0.1)
                        right_pct = abs(right_delta) / max(self.prev_right_dist, 0.1)
                        pct_trigger = (left_pct > self.pct_thresh or right_pct > self.pct_thresh)
                    imu_confirmed = False
                    if self.use_imu and hasattr(self.localization, 'imu_available') and self.localization.imu_available:
                        yaw_rate = self.localization.latest_imu_yaw_rate
                        imu_confirmed = (abs(yaw_rate) > self.imu_thresh)
                    corner_detected = deriv_trigger or pct_trigger
                    corner_strength = max(abs(left_delta), abs(right_delta), left_pct, right_pct)

                    if corner_detected:
                        # ---- LOGIC A: Corner snap ----
                        if imu_confirmed:
                            self._apply_corner_snap()

                        if self.use_graded:
                            steer_multiplier = min(1.0, corner_strength / 0.5)
                            steer_multiplier = max(0.3, steer_multiplier)
                            if imu_confirmed:
                                steer_multiplier = 1.0
                            wall_steer = wall_steer * steer_multiplier
                        self.sections_passed += 1
                        self.last_section_time = time.time()
                        logger.debug(
                            f"Corner detected (deriv={deriv_trigger}, pct={pct_trigger}) "
                            f"-> sections={self.sections_passed}"
                        )
                self.prev_left_dist = left_dist
                self.prev_right_dist = right_dist
                if self.prev_error is not None and dt > 0:
                    self.error_derivative = (error - self.prev_error) / dt
                self.prev_error = error

            feedforward = self._feedforward_steer(scan)

            if abs(traffic_angular) > 0.01:
                raw_angular = traffic_angular
            else:
                raw_angular = wall_steer

            if abs(feedforward) > 0.01 and abs(traffic_angular) < 0.01:
                raw_angular = 0.7 * raw_angular + 0.3 * feedforward

            angular = raw_angular + emergency_actions.get('steer_offset', 0.0)
            throttle = emergency_actions.get('throttle_factor', 1.0)

            steer_intensity = abs(raw_angular)
            steer_factor = 1.0 - (steer_intensity / 0.5) * self.CORNER_SLOWDOWN_MAX
            steer_factor = max(0.6, min(1.0, steer_factor))

            abs_error = abs(error)
            deriv_factor = 1.0 - min(abs(self.error_derivative) / 0.5, 0.4)
            error_factor = 1.0 - min(abs_error / 0.4, 0.3)
            predictive_factor = min(deriv_factor, error_factor)
            predictive_factor = max(0.6, min(1.0, predictive_factor))

            distance_factor = 1.0
            if self.challenge == "obstacle" and dist_to_pillar is not None and dist_to_pillar > 0:
                if dist_to_pillar < self.tl.distance_slowdown_start_m:
                    distance_factor = dist_to_pillar / self.tl.distance_slowdown_start_m
                    distance_factor = max(self.tl.distance_slowdown_min_factor, distance_factor)
                    distance_factor = min(1.0, distance_factor)
                if dist_to_pillar > 0:
                    angle_deg = math.degrees(math.atan2(target.local_x, target.local_y))
                    range_m = self.lidar.get_range_in_sector(angle_deg, 10.0)
                    if range_m is None or range_m > self.tl.lidar_confirm_range_m:
                        distance_factor = 1.0
                        traffic_angular = 0.0
                        raw_angular = wall_steer
                        angular = raw_angular + emergency_actions.get('steer_offset', 0.0)

            boost_factor = 1.0
            if abs_error < 0.05 and abs(self.error_derivative) < 0.01:
                boost_factor = self.STRAIGHT_BOOST

            target_speed = self._compute_target_speed(scan, corner_detected, corner_strength)

            combined_factor = steer_factor * predictive_factor * distance_factor * boost_factor
            linear_from_pid = self.BASE_SPEED * throttle * combined_factor
            linear_from_lookahead = target_speed * throttle
            linear = min(linear_from_pid, linear_from_lookahead)
            linear = min(linear, self.config.vehicle.max_speed_mps)
            self.current_speed = linear

            if hasattr(self.localization, 'imu_available') and self.localization.imu_available:
                yaw_rate = self.localization.latest_imu_yaw_rate
                if yaw_rate is not None:
                    lateral_G = abs(linear * yaw_rate) / 9.81
                    self.filtered_G = self.G_FILTER_ALPHA * lateral_G + (1 - self.G_FILTER_ALPHA) * self.filtered_G
                    if self.filtered_G > self.MAX_SAFE_G:
                        linear *= self.MAX_SAFE_G / self.filtered_G
                        self.current_speed = linear
                        logger.debug(f"G‑force limiting: {self.filtered_G:.2f}G -> speed reduced")

            if detections:
                self.spatial_map.update(detections, robot_speed=linear, dt=dt)

        self.steering.set_speed(linear, angular)

        lidar_subset = [(ang, dist) for ang, dist in scan.items() if abs(ang) % 5 < 1]
        steering_angle = 0.0 if abs(linear) < 0.001 else math.atan2(
            angular * self.config.vehicle.wheelbase_m, linear
        )
        self.localization.update_pose(linear, steering_angle, lidar_points=lidar_subset)

        self._update_lap_count()

    # ==========================================================
    # Parking approach, fine‑tune, and search
    # ==========================================================

    def _drive_to_parking(self):
        """Drive toward the parking spot using stored geometry.
           LOGIC C: Re‑scan when within 1.2 m."""
        current_pose = self.localization.get_pose()
        geometry = self.localization.parking_bay_geometry

        if geometry is None:
            logger.warning("No parking geometry, entering search")
            self.state = RobotState.SEARCH_PARKING
            return

        target_x = (geometry['x_min'] + geometry['x_max']) / 2.0
        target_y = (geometry['y_min'] + geometry['y_max']) / 2.0

        dx = target_x - current_pose.x
        dy = target_y - current_pose.y
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)

        heading_error = target_heading - current_pose.theta
        heading_error = self._normalize_angle(heading_error)

        # ---- LOGIC C: Live re‑scan if within 1.2 m ----
        if distance < self.PARKING_DRIVE_THRESHOLD:
            fresh = self.localization.scan_for_parking_markers()
            if fresh is not None:
                stored_spacing = math.hypot(
                    geometry['x_max'] - geometry['x_min'],
                    geometry['y_max'] - geometry['y_min']
                )
                new_spacing = fresh.get('spacing', 0.0)
                if abs(new_spacing - stored_spacing) < 0.15:
                    # Accept new geometry
                    self.localization.parking_bay_geometry = fresh
                    # Only restart parking controller if in TERMINATION
                    if self.state == RobotState.TERMINATION:
                        self.parking.start(fresh)
                    geometry = fresh
                    logger.info(f"Geometry re‑scanned and updated: {fresh}")
                    # Recompute target
                    target_x = (geometry['x_min'] + geometry['x_max']) / 2.0
                    target_y = (geometry['y_min'] + geometry['y_max']) / 2.0
                    dx = target_x - current_pose.x
                    dy = target_y - current_pose.y
                    distance = math.hypot(dx, dy)

        # If we are very close, switch to fine‑tune (which will start parking)
        if distance < self.PARKING_FINE_TUNE_THRESHOLD:
            self._fine_tune_parking()
            return

        logger.info(f"Driving to parking: distance={distance:.2f}m, heading error={math.degrees(heading_error):.1f}°")

        wall_steer, _, _ = self._compute_wall_steer({}, 0.05)
        steer_correction = 0.5 * heading_error
        steer_correction = max(-0.3, min(0.3, steer_correction))

        final_steer = 0.7 * steer_correction + 0.3 * wall_steer
        final_steer = max(-0.4, min(0.4, final_steer))

        if distance < 0.8:
            speed = 0.15
        else:
            speed = 0.3

        self.steering.set_speed(speed, final_steer)

    def _fine_tune_parking(self):
        """
        Fine‑tune position when close to the parking spot.
        This will compute the side of the bay and pass it to the parking controller.
        """
        logger.info("Fine‑tuning parking position (small error correction)")

        self.steering.stop()
        time.sleep(0.2)

        current_pose = self.localization.get_pose()
        geometry = self.localization.parking_bay_geometry

        if geometry is None:
            logger.warning("No geometry for fine‑tune, falling back to search")
            self.state = RobotState.SEARCH_PARKING
            return

        # Compute side of the bay relative to car (for reverse steer direction)
        bay_cx = (geometry['x_min'] + geometry['x_max']) / 2.0
        bay_cy = (geometry['y_min'] + geometry['y_max']) / 2.0
        dx = bay_cx - current_pose.x
        dy = bay_cy - current_pose.y
        angle_to_bay = math.atan2(dy, dx)
        side_angle = self._normalize_angle(angle_to_bay - current_pose.theta)

        # Determine reverse steer direction (LOGIC D)
        if side_angle > 0:
            reverse_steer = 0.35
            side = 'left'
        else:
            reverse_steer = -0.35
            side = 'right'

        # Pass to parking controller
        self.parking.set_reverse_steer_offset(reverse_steer)
        self.parking.set_bay_side(side)

        # If already close, start parking
        distance = math.hypot(dx, dy)
        if distance < 0.15 and abs(self._normalize_angle(angle_to_bay - current_pose.theta)) < 0.1:
            logger.info("Already within fine‑tune tolerance, starting parking")
            self._start_parking()
            return

        # Drive slowly toward target
        target_heading = math.atan2(bay_cy - current_pose.y, bay_cx - current_pose.x)
        heading_error = self._normalize_angle(target_heading - current_pose.theta)
        speed = 0.05
        steer = 0.3 * heading_error
        steer = max(-0.3, min(0.3, steer))

        self.steering.set_speed(speed, steer)
        time.sleep(0.5)

        # Re‑check geometry
        fresh = self.localization.scan_for_parking_markers()
        if fresh:
            self.localization.parking_bay_geometry = fresh
            logger.info(f"Geometry updated during fine‑tune: {fresh}")

        if distance < 0.35:
            self._start_parking()
        else:
            logger.info("Fine‑tune failed to get close, resuming drive")
            self._drive_to_parking()

    def _search_parking_state(self):
        logger.info("State: SEARCH_PARKING")
        if self.localization.parking_bay_geometry is not None:
            logger.info("Parking geometry found – starting parking")
            self._start_parking()
            return

        if self.search_start_time == 0.0:
            self.search_start_time = time.time()
            self.search_phase = "FORWARD"
            logger.info("Search started: moving FORWARD")

        geometry = self.localization.scan_for_parking_markers()
        if geometry:
            self.localization.parking_bay_geometry = geometry
            logger.info(f"Parking markers found during search: {geometry}")
            self.parking.start(geometry)
            self._start_parking()
            return

        current_time = time.time()
        phase_duration = current_time - self.search_start_time

        if self.search_phase == "FORWARD":
            self.steering.set_speed(0.05, 0.0)
            if phase_duration > 3.0:
                self.search_phase = "REVERSE"
                self.search_start_time = current_time
                logger.info("Search phase: REVERSE")
        elif self.search_phase == "REVERSE":
            self.steering.set_speed(-0.05, 0.0)
            if phase_duration > 3.0:
                self.search_phase = "TURN"
                self.search_start_time = current_time
                logger.info("Search phase: TURN (U‑turn)")
        elif self.search_phase == "TURN":
            self.steering.set_speed(0.05, 0.4)
            if phase_duration > 4.0:
                self.search_phase = "FORWARD"
                self.search_start_time = current_time
                logger.info("Search phase: FORWARD (cycle repeat)")

        if time.time() - self.last_loop_time > 60.0:
            logger.error("Search timeout after 60 seconds – stopping")
            self.steering.stop()
            self.state = RobotState.EMERGENCY_STOP

    # ==========================================================
    # Termination state
    # ==========================================================

    def _termination_state(self):
        logger.info("State: TERMINATION")

        if self.is_open_challenge:
            self.steering.stop()
            logger.info("Open challenge complete")
            return

        if self.localization.parking_bay_geometry is None:
            logger.warning("Parking geometry missing – entering SEARCH_PARKING")
            self.state = RobotState.SEARCH_PARKING
            self.search_start_time = 0.0
            return

        current_pose = self.localization.get_pose()
        geometry = self.localization.parking_bay_geometry
        target_x = (geometry['x_min'] + geometry['x_max']) / 2.0
        target_y = (geometry['y_min'] + geometry['y_max']) / 2.0

        dx = target_x - current_pose.x
        dy = target_y - current_pose.y
        distance = math.hypot(dx, dy)

        if distance < self.PARKING_FINE_TUNE_THRESHOLD:
            self._fine_tune_parking()
            return

        if distance < self.PARKING_DRIVE_THRESHOLD:
            self._drive_to_parking()
            return

        logger.warning(f"Too far from parking spot ({distance:.2f}m), entering search")
        self.state = RobotState.SEARCH_PARKING
        self.search_start_time = 0.0

    # ==========================================================
    # Parking start and emergency stop
    # ==========================================================

    def _start_parking(self):
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
