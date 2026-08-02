"""
Unit tests for the State Machine.
Mocks hardware dependencies to test logic in isolation.
"""

from unittest.mock import Mock

import pytest

from src.config_parser import (
    SystemConfig, ZoneManagement, ChallengeConfig, SensorFusionTracking,
    UltrasonicShield, TrafficLightRules, NavigationMatrix, StateMachine as SM,
    MappingConfig, SurpriseRule, LapCounting, Vehicle, Vision,
    NavigationBehavior, PIDParams, CornerDetectionParams, GForceParams,
    TrafficLightParams, OpenChallengeParams, ObstacleChallengeParams,
    ParkingConfig, ParkingEmergencyShield
)
from src.state_machine import StateMachine, RobotState


@pytest.fixture
def mock_config():
    """Create a complete mock config for testing."""
    return SystemConfig(
        version="2026.3",
        competition="WRO Future Engineers",
        zone_management=ZoneManagement(
            open_challenge=ChallengeConfig(
                save_start_point=True,
                use_parking_slot=False,
                start_pose_registration={"method": "LIDAR_WALL_DISTANCES"},
                finish_behavior={"action": "CONTROLLED_STOP_AT_START_ZONE"}
            ),
            obstacle_challenge=ChallengeConfig(
                save_start_point=True,
                use_parking_slot=True,
                start_pose_registration={"method": "LIDAR_WALL_DISTANCES"},
                finish_behavior={"action": "PARALLEL_PARKING_SEQUENCE"}
            )
        ),
        sensor_fusion_and_tracking=SensorFusionTracking(
            huskylens_camera={
                "role": "RAW_BOUNDING_BOX_DETECTOR",
                "outputs": ["COLOR_ID", "CENTROID_X", "CENTROID_Y", "WIDTH", "HEIGHT"],
                "horizontal_fov_deg": 60.0
            },
            lidar_2d={"scan_arc_deg": 90.0, "update_rate_hz": 10},
            host_spatial_tracker={
                "processor_location": "MAIN_ONBOARD_COMPUTER",
                "coordinate_transformation": {
                    "method": "FUSE_CAMERA_ANGLE_WITH_LIDAR_DISTANCE",
                    "formula": ""
                },
                "spatial_gating": {"tolerance_radius_m": 0.20},
                "debouncing_and_persistence": {
                    "confirmation_threshold_frames": 3,
                    "frame_loss_tolerance_sec": 1.5,
                    "prune_condition": "Y_LOCAL < 0.0"
                },
                "passing_priority_queue": {
                    "sorting_rule": "SORT_BY_ASCENDING_FORWARD_DISTANCE",
                    "active_target": "QUEUE_INDEX_0"
                }
            }
        ),
        ultrasonic_emergency_shield=UltrasonicShield(
            enabled=True,
            enable_reverse=True,
            enable_rear_check=True,
            enable_hard_shield=True,
            enable_hard_steer=True,
            thresholds_cm={
                "front_stop": 12.0,
                "front_left_safety": 18.0,
                "front_right_safety": 18.0,
                "critical_stop": 8.0,
                "hard_shield_cm": 5.0
            },
            dynamic_throttle={
                "enable_speed_dampening": True,
                "dampened_speed_factor": 0.60
            },
            reverse_params={
                "speed_mps": -0.10,
                "duration_s": 1.0,
                "rear_clearance_threshold_m": 0.15
            }
        ),
        traffic_light_passing_rules=TrafficLightRules(
            RED_BLOCK_PASS_SIDE="RIGHT",
            GREEN_BLOCK_PASS_SIDE="LEFT",
            INVERT_COLORS_MID_RACE=False
        ),
        navigation_matrix=NavigationMatrix(
            TOTAL_REQUIRED_LAPS=3,
            LAP_1_DIRECTION="CLOCKWISE",
            LAP_2_DIRECTION="CLOCKWISE",
            LAP_3_DIRECTION="CLOCKWISE"
        ),
        state_machine=SM(
            STATE_1_INIT=["Register pose", "Reset lap counter"],
            STATE_2_NAVIGATE=["Run spatial tracker", "Check emergency shield"],
            STATE_3_TERMINATION=["Open: stop", "Obstacle: park"]
        ),
        mapping=MappingConfig(
            use_mapping=True,
            save_map_to_disk=True,
            load_map_from_disk=True,
            force_rebuild=False
        ),
        surprise_rule=SurpriseRule(
            enabled=True,
            trigger_lap=2,
            color_to_continue="GREEN",
            color_to_reverse="RED",
            fallback_direction="REVERSE",
            turnaround_speed=0.1
        ),
        lap_counting=LapCounting(
            lap_length_m=12.0,
            section_fallback_timeout_s=3.0,
            emergency_lap_margin_m=0.2
        ),
        vehicle=Vehicle(
            wheelbase_m=0.25,
            max_speed_mps=1.5,
            max_steer_rad=0.524
        ),
        vision=Vision(
            color_red_id=1,
            color_green_id=2
        ),
        navigation=NavigationBehavior(
            open_challenge=OpenChallengeParams(
                base_speed_mps=1.5,
                wall_follow_gain=0.30,
                steer_magnitude_radps=0.35,
                straight_boost_factor=1.20,
                corner_slowdown_max_reduction=0.3,
                predictive_slowdown_gain=0.3,
                pid=PIDParams(kp=25.0, ki=0.1, kd=8.0),
                corner_detection=CornerDetectionParams(
                    use_derivative=True,
                    use_percentage=True,
                    use_graded_steering=True,
                    use_imu_confirmation=True,
                    lidar_derivative_threshold=0.3,
                    pct_threshold=0.4,
                    imu_confirm_threshold_radps=0.3
                ),
                g_force=GForceParams(max_safe_g=0.30, filter_alpha=0.3)
            ),
            obstacle_challenge=ObstacleChallengeParams(
                base_speed_mps=1.3,
                wall_follow_gain=0.25,
                steer_magnitude_radps=0.30,
                straight_boost_factor=1.10,
                corner_slowdown_max_reduction=0.4,
                predictive_slowdown_gain=0.4,
                pid=PIDParams(kp=25.0, ki=0.1, kd=8.0),
                corner_detection=CornerDetectionParams(
                    use_derivative=True,
                    use_percentage=True,
                    use_graded_steering=True,
                    use_imu_confirmation=True,
                    lidar_derivative_threshold=0.25,
                    pct_threshold=0.4,
                    imu_confirm_threshold_radps=0.3
                ),
                g_force=GForceParams(max_safe_g=0.28, filter_alpha=0.3),
                traffic_light=TrafficLightParams(
                    distance_slowdown_start_m=2.0,
                    distance_slowdown_min_factor=0.4,
                    steering_slowdown_max_reduction=0.4,
                    lidar_confirm_range_m=2.5
                )
            )
        ),
        parking=ParkingConfig(
            emergency_shield=ParkingEmergencyShield(
                enabled=True,
                use_parking_thresholds=True,
                thresholds_cm={
                    "front_stop": 8.0,
                    "front_left_safety": 10.0,
                    "front_right_safety": 10.0,
                    "critical_stop": 4.0,
                    "hard_shield_cm": 3.0
                },
                dynamic_throttle={
                    "enable_speed_dampening": True,
                    "dampened_speed_factor": 0.60
                },
                disable_reverse=False,
                disable_hard_steer=False,
                disable_hard_shield=False
            )
        )
    )


@pytest.fixture
def mock_dependencies(mock_config):
    """Create mocked dependencies for the state machine."""
    serial_bridge = Mock()
    localization = Mock()
    localization.get_pose.return_value = Mock(x=0.0, y=0.0, theta=0.0)
    localization.get_start_pose.return_value = Mock(x=0.0, y=0.0, theta=0.0)
    vision_reader = Mock()
    lidar_fusion = Mock()
    lidar_fusion.get_scan_snapshot.return_value = {}
    spatial_map = Mock()
    spatial_map.get_confirmed_objects.return_value = []
    emergency_shield = Mock()
    emergency_shield.get_emergency_actions.return_value = {
        'brake': False,
        'steer_offset': 0.0,
        'throttle_factor': 1.0
    }
    steering = Mock()
    steering.wheelbase = 0.25
    parking = Mock()
    parking.is_complete.return_value = False
    parking.is_aborted.return_value = False

    return {
        'config': mock_config,
        'serial_bridge': serial_bridge,
        'localization': localization,
        'vision_reader': vision_reader,
        'lidar_fusion': lidar_fusion,
        'spatial_map': spatial_map,
        'emergency_shield': emergency_shield,
        'steering': steering,
        'parking': parking
    }


def test_initial_state_transition(mock_dependencies):
    """Test that state machine starts in INIT and transitions to NAVIGATE."""
    fsm = StateMachine(**mock_dependencies)
    assert fsm.state == RobotState.INIT

    fsm.run()
    assert fsm.state == RobotState.NAVIGATE
    mock_dependencies['localization'].register_start_pose.assert_called_once()


def test_color_to_angular_mapping_from_config(mock_dependencies):
    """Test that angular velocity is derived from YAML config."""
    fsm = StateMachine(**mock_dependencies)

    angular = fsm._get_angular_velocity_from_color(fsm.COLOR_RED)
    assert angular < 0  # RED -> RIGHT -> negative

    angular = fsm._get_angular_velocity_from_color(fsm.COLOR_GREEN)
    assert angular > 0  # GREEN -> LEFT -> positive

    angular = fsm._get_angular_velocity_from_color(99)
    assert angular == 0.0


def test_lap_counting_increment(mock_dependencies):
    """Test that lap count increments when sections_passed reaches 8."""
    fsm = StateMachine(**mock_dependencies)

    pose_mock = Mock()
    pose_mock.x = 0.0
    mock_dependencies['localization'].get_pose.return_value = pose_mock

    fsm.lap_start_pose = pose_mock
    fsm.lap_count = 0
    fsm.sections_passed = 8

    fsm._update_lap_count()
    assert fsm.lap_count == 1
    assert fsm.sections_passed == 0


def test_emergency_brake_override(mock_dependencies):
    """Test that emergency brake stops the robot."""
    fsm = StateMachine(**mock_dependencies)

    mock_dependencies['emergency_shield'].get_emergency_actions.return_value = {
        'brake': True,
        'steer_offset': 0.0,
        'throttle_factor': 0.0
    }

    fsm._navigate_state()
    assert fsm.state == RobotState.EMERGENCY_STOP
    mock_dependencies['steering'].stop.assert_called_once()


def test_surprise_rule_trigger(mock_dependencies):
    """Test that surprise rule is triggered at the configured lap."""
    fsm = StateMachine(**mock_dependencies)
    fsm.lap_count = 2
    fsm.surprise_rule_activated = False
    fsm.last_traffic_light_color = 1  # RED

    fsm.sections_passed = 8
    fsm._update_lap_count()

    assert fsm.surprise_rule_activated is True


def test_termination_transition(mock_dependencies):
    """Test that state transitions to TERMINATION after 3 laps."""
    fsm = StateMachine(**mock_dependencies)
    fsm.is_open_challenge = True

    fsm.lap_count = 3
    fsm.sections_passed = 8
    fsm._update_lap_count()

    assert fsm.state == RobotState.TERMINATION
