"""
Parses system_prompt_matrix.yaml into typed Python dataclasses.
Supports both open_challenge and obstacle_challenge nested structures.
"""

import yaml
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any


@dataclass
class ChallengeConfig:
    save_start_point: bool
    use_parking_slot: bool
    start_pose_registration: Dict[str, Any]
    finish_behavior: Dict[str, Any]


@dataclass
class ZoneManagement:
    open_challenge: ChallengeConfig
    obstacle_challenge: ChallengeConfig


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
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    sm = data['system_prompt_matrix']

    # Parse nested zone_management
    zm = sm['zone_management']
    open_challenge = ChallengeConfig(**zm['open_challenge'])
    obstacle_challenge = ChallengeConfig(**zm['obstacle_challenge'])
    zone_management = ZoneManagement(open_challenge=open_challenge,
                                     obstacle_challenge=obstacle_challenge)

    return SystemConfig(
        version=sm['version'],
        competition=sm['competition'],
        zone_management=zone_management,
        sensor_fusion_and_tracking=SensorFusionTracking(**sm['sensor_fusion_and_tracking']),
        ultrasonic_emergency_shield=UltrasonicShield(**sm['ultrasonic_emergency_shield']),
        traffic_light_passing_rules=TrafficLightRules(**sm['traffic_light_passing_rules']),
        navigation_matrix=NavigationMatrix(**sm['navigation_matrix']),
        state_machine=StateMachine(**sm['state_machine'])
    )
