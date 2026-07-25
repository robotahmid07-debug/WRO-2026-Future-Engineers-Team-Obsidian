"""
Unit tests for the State Machine.
Mocks hardware dependencies to test logic in isolation.
"""

import pytest
import math
from unittest.mock import Mock, patch

from src.config_parser import SystemConfig, ZoneManagement, SensorFusionTracking
from src.config_parser import UltrasonicShield, TrafficLightRules, NavigationMatrix, StateMachine
from src.state_machine import StateMachine, RobotState


@pytest.fixture
def mock_config():
    """Create a minimal config for testing."""
    return SystemConfig(
        version="2026.3",
        competition="WRO Future Engineers",
        zone_management=ZoneManagement(
            save_start_point=True,
            use_parking_slot=False,
            start_pose_registration={"method": "LIDAR_WALL_DISTANCES_AND_ODOMETRY"},
            finish_behavior={"action": "CONTROLLED_STOP_AT_START_ZONE"}
        ),
        sensor_fusion_and_tracking=SensorFusionTracking(
            huskylens_camera={"role": "RAW_BOUNDING_BOX_DETECTOR"},
            lidar_2d={"scan_arc_deg": 90.0},
            host_spatial_tracker={"confirmation_threshold_frames": 3}
        ),
        ultrasonic_emergency_shield=UltrasonicShield(
            enabled=True,
            thresholds_cm={"front_stop": 12.0, "front_left_safety": 18.0, "front_right_safety": 18.0},
            dynamic_throttle={"enable_speed_dampening": True, "dampened_speed_factor": 0.6}
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
        state_machine=StateMachine(
            STATE_1_INIT=["Register pose", "Reset lap counter"],
            STATE_2_NAVIGATE=["Run spatial tracker", "Check emergency shield"],
            STATE_3_TERMINATION=["Open: stop", "Obstacle: park"]
        )
    )


@pytest.fixture
def mock_dependencies(mock_config):
    """Create mocked dependencies for the state machine."""
    serial_bridge = Mock()
    localization = Mock()
    # Make get_pose return a simple pose
    localization.get_pose.return_value = Mock(x=0.0, y=0.0, theta=0.0)
    vision_reader = Mock()
    lidar_fusion = Mock()
    spatial_map = Mock()
    emergency_shield = Mock()
    steering = Mock()
    steering.wheelbase = 0.25
    parking = Mock()

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

    # Run once to initialize
    fsm.run()
    # It should now be in NAVIGATE
    assert fsm.state == RobotState.NAVIGATE
    mock_dependencies['localization'].register_start_pose.assert_called_once()


def test_color_to_angular_mapping_from_config(mock_dependencies):
    """Test that angular velocity is derived from YAML config."""
    fsm = StateMachine(**mock_dependencies)

    # RED -> config says "RIGHT" -> should turn negative (right)
    angular = fsm._get_angular_velocity_from_color(fsm.COLOR_RED)
    assert angular < 0  # negative = turn right

    # GREEN -> config says "LEFT" -> should turn positive (left)
    angular = fsm._get_angular_velocity_from_color(fsm.COLOR_GREEN)
    assert angular > 0  # positive = turn left

    # Unknown color -> go straight
    angular = fsm._get_angular_velocity_from_color(99)
    assert angular == 0.0


def test_lap_counting_increment(mock_dependencies):
    """Test that lap count increments when distance traveled exceeds threshold."""
    fsm = StateMachine(**mock_dependencies)

    # Mock pose to simulate movement
    pose_mock = Mock()
    pose_mock.x = 0.0
    mock_dependencies['localization'].get_pose.return_value = pose_mock

    # Set initial start pose
    fsm.lap_start_pose = pose_mock
    fsm.track_length_estimate = 1.0  # Small for testing

    # Move 0.5m -> no lap
    pose_mock.x = 0.5
    fsm._update_lap_count()
    assert fsm.lap_count == 0

    # Move 1.2m -> lap completed
    pose_mock.x = 1.2
    fsm._update_lap_count()
    assert fsm.lap_count == 1

    # After 3 laps, state should transition to TERMINATION
    fsm.lap_count = 2
    fsm.track_length_estimate = 1.0
    pose_mock.x = 3.5
    fsm._update_lap_count()
    assert fsm.lap_count == 3
    assert fsm.state == RobotState.TERMINATION


def test_emergency_brake_override(mock_dependencies):
    """Test that emergency brake stops the robot and sets state to EMERGENCY_STOP."""
    fsm = StateMachine(**mock_dependencies)

    # Mock emergency shield to return brake=True
    mock_dependencies['emergency_shield'].get_emergency_actions.return_value = {
        'brake': True,
        'steer_offset': 0.0,
        'throttle_factor': 0.0
    }

    # Should transition to EMERGENCY_STOP
    fsm._navigate_state()
    assert fsm.state == RobotState.EMERGENCY_STOP
    mock_dependencies['steering'].stop.assert_called_once()
