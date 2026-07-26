"""
Parses system_prompt_matrix.yaml into typed Python dataclasses.
Includes all sections: zone_management, sensor fusion, emergency shield,
traffic lights, navigation, mapping, surprise rule, lap counting,
vehicle parameters, navigation behavior, and vision.
"""

import yaml
from dataclasses import dataclass
from typing import Optional, List, Dict, Any


# ------------------------------
# Existing Dataclasses (unchanged)
# ------------------------------

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


# ------------------------------
# New Dataclasses for Additional Sections
# ------------------------------

@dataclass
class MappingConfig:
    use_mapping: bool
    save_map_to_disk: bool
    load_map_from_disk: bool
    force_rebuild: bool


@dataclass
class SurpriseRule:
    enabled: bool
    trigger_lap: int
    color_to_continue: str
    color_to_reverse: str
    fallback_direction: str
    turnaround_speed: float


@dataclass
class LapCounting:
    lap_length_m: float
    section_fallback_timeout_s: float
    emergency_lap_margin_m: float


@dataclass
class Vehicle:
    wheelbase_m: float
    max_speed_mps: float
    max_steer_rad: float


@dataclass
class NavigationBehavior:
    base_speed_mps: float
    steer_magnitude_radps: float
    wall_follow_gain: float


@dataclass
class Vision:
    color_red_id: int
    color_green_id: int


# ------------------------------
# Main SystemConfig Dataclass
# ------------------------------

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
    mapping: MappingConfig
    surprise_rule: SurpriseRule
    lap_counting: LapCounting
    vehicle: Vehicle
    navigation: NavigationBehavior
    vision: Vision


# ------------------------------
# Parser Function
# ------------------------------

def load_config(yaml_path: str) -> SystemConfig:
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    sm = data['system_prompt_matrix']

    # Zone management (nested)
    zm = sm['zone_management']
    open_challenge = ChallengeConfig(**zm['open_challenge'])
    obstacle_challenge = ChallengeConfig(**zm['obstacle_challenge'])
    zone_management = ZoneManagement(open_challenge=open_challenge,
                                     obstacle_challenge=obstacle_challenge)

    # Mapping (with defaults if missing)
    mapping_defaults = {
        'use_mapping': True,
        'save_map_to_disk': True,
        'load_map_from_disk': True,
        'force_rebuild': False
    }
    mapping_raw = sm.get('mapping', {})
    mapping = MappingConfig(
        use_mapping=mapping_raw.get('use_mapping', mapping_defaults['use_mapping']),
        save_map_to_disk=mapping_raw.get('save_map_to_disk', mapping_defaults['save_map_to_disk']),
        load_map_from_disk=mapping_raw.get('load_map_from_disk', mapping_defaults['load_map_from_disk']),
        force_rebuild=mapping_raw.get('force_rebuild', mapping_defaults['force_rebuild'])
    )

    # Surprise Rule (with defaults)
    surprise_defaults = {
        'enabled': True,
        'trigger_lap': 2,
        'color_to_continue': 'GREEN',
        'color_to_reverse': 'RED',
        'fallback_direction': 'REVERSE',
        'turnaround_speed': 0.1
    }
    surprise_raw = sm.get('surprise_rule', {})
    surprise = SurpriseRule(
        enabled=surprise_raw.get('enabled', surprise_defaults['enabled']),
        trigger_lap=surprise_raw.get('trigger_lap', surprise_defaults['trigger_lap']),
        color_to_continue=surprise_raw.get('color_to_continue', surprise_defaults['color_to_continue']),
        color_to_reverse=surprise_raw.get('color_to_reverse', surprise_defaults['color_to_reverse']),
        fallback_direction=surprise_raw.get('fallback_direction', surprise_defaults['fallback_direction']),
        turnaround_speed=surprise_raw.get('turnaround_speed', surprise_defaults['turnaround_speed'])
    )

    # Lap Counting (with defaults)
    lap_defaults = {
        'lap_length_m': 12.0,
        'section_fallback_timeout_s': 3.0,
        'emergency_lap_margin_m': 0.2
    }
    lap_raw = sm.get('lap_counting', {})
    lap = LapCounting(
        lap_length_m=lap_raw.get('lap_length_m', lap_defaults['lap_length_m']),
        section_fallback_timeout_s=lap_raw.get('section_fallback_timeout_s', lap_defaults['section_fallback_timeout_s']),
        emergency_lap_margin_m=lap_raw.get('emergency_lap_margin_m', lap_defaults['emergency_lap_margin_m'])
    )

    # Vehicle (with defaults)
    vehicle_defaults = {
        'wheelbase_m': 0.25,
        'max_speed_mps': 0.5,
        'max_steer_rad': 0.524
    }
    vehicle_raw = sm.get('vehicle', {})
    vehicle = Vehicle(
        wheelbase_m=vehicle_raw.get('wheelbase_m', vehicle_defaults['wheelbase_m']),
        max_speed_mps=vehicle_raw.get('max_speed_mps', vehicle_defaults['max_speed_mps']),
        max_steer_rad=vehicle_raw.get('max_steer_rad', vehicle_defaults['max_steer_rad'])
    )

    # Navigation Behavior (with defaults)
    nav_defaults = {
        'base_speed_mps': 0.3,
        'steer_magnitude_radps': 0.3,
        'wall_follow_gain': 0.25
    }
    nav_raw = sm.get('navigation', {})
    nav = NavigationBehavior(
        base_speed_mps=nav_raw.get('base_speed_mps', nav_defaults['base_speed_mps']),
        steer_magnitude_radps=nav_raw.get('steer_magnitude_radps', nav_defaults['steer_magnitude_radps']),
        wall_follow_gain=nav_raw.get('wall_follow_gain', nav_defaults['wall_follow_gain'])
    )

    # Vision (with defaults)
    vision_defaults = {
        'color_red_id': 1,
        'color_green_id': 2
    }
    vision_raw = sm.get('vision', {})
    vision = Vision(
        color_red_id=vision_raw.get('color_red_id', vision_defaults['color_red_id']),
        color_green_id=vision_raw.get('color_green_id', vision_defaults['color_green_id'])
    )

    # Build and return the full config
    return SystemConfig(
        version=sm['version'],
        competition=sm['competition'],
        zone_management=zone_management,
        sensor_fusion_and_tracking=SensorFusionTracking(**sm['sensor_fusion_and_tracking']),
        ultrasonic_emergency_shield=UltrasonicShield(**sm['ultrasonic_emergency_shield']),
        traffic_light_passing_rules=TrafficLightRules(**sm['traffic_light_passing_rules']),
        navigation_matrix=NavigationMatrix(**sm['navigation_matrix']),
        state_machine=StateMachine(**sm['state_machine']),
        mapping=mapping,
        surprise_rule=surprise,
        lap_counting=lap,
        vehicle=vehicle,
        navigation=nav,
        vision=vision
    )
