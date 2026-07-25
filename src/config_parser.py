"""
Parses system_prompt_matrix.yaml into typed Python dataclasses.
All fields from the YAML are mapped correctly.
"""

import yaml
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any


@dataclass
class ZoneManagement:
    save_start_point: bool
    use_parking_slot: bool
    start_pose_registration: Dict[str, Any]
    finish_behavior: Dict[str, Any]


@dataclass
class SensorFusionTracking:
    huskylens_camera: Dict[str, Any]
    lidar_2d: Dict[str, Any]
    host_spatial_tracker: Dict[str, Any]


@dataclass
class UltrasonicShield:
    enabled: bool
    thresholds_cm: Dict[str, float]
    dynamic_throttle: Dict[str, Any]


@dataclass
class TrafficLightRules:
    RED_BLOCK_PASS_SIDE: str
    GREEN_BLOCK_PASS_SIDE: str
    INVERT_COLORS_MID_RACE: bool


@dataclass
class NavigationMatrix:
    TOTAL_REQUIRED_LAPS: int
    LAP_1_DIRECTION: str
    LAP_2_DIRECTION: str
    LAP_3_DIRECTION: str


@dataclass
class StateMachine:
    STATE_1_INIT: List[str]
    STATE_2_NAVIGATE: List[str]
    STATE_3_TERMINATION: List[str]


@dataclass
class SystemConfig:
    version: str
    competition: str
    zone_management: ZoneManagement
    sensor_fusion_and_tracking: SensorFusionTracking
    ultrasonic_emergency_shield: UltrasonicShield
    traffic_light_passing_rules: TrafficLightRules
    navigation_matrix: NavigationMatrix
    state_machine: StateMachine


def load_config(yaml_path: str) -> SystemConfig:
    """
    Load and parse the YAML configuration file.

    Args:
        yaml_path: Path to the system_prompt_matrix.yaml file.

    Returns:
        SystemConfig object with all fields populated.

    Raises:
        FileNotFoundError: If the YAML file does not exist.
        KeyError: If any required field is missing.
        yaml.YAMLError: If the YAML is malformed.
    """
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    # The YAML root is "system_prompt_matrix"
    sm = data['system_prompt_matrix']

    return SystemConfig(
        version=sm['version'],
        competition=sm['competition'],
        zone_management=ZoneManagement(**sm['zone_management']),
        sensor_fusion_and_tracking=SensorFusionTracking(**sm['sensor_fusion_and_tracking']),
        ultrasonic_emergency_shield=UltrasonicShield(**sm['ultrasonic_emergency_shield']),
        traffic_light_passing_rules=TrafficLightRules(**sm['traffic_light_passing_rules']),
        navigation_matrix=NavigationMatrix(**sm['navigation_matrix']),
        state_machine=StateMachine(**sm['state_machine'])
    )
