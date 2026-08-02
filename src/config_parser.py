"""
Parses system_prompt_matrix.yaml into typed Python dataclasses.
Includes all new sections:
  - Challenge-specific navigation (Open/Obstacle)
  - PID, corner detection, G-force, traffic light parameters
  - Parking-specific emergency shield thresholds
  - Feature toggles for reverse, hard shield, rear check, etc.
"""

import yaml
from dataclasses import dataclass
from typing import List, Dict, Any

# ============================================================
# Existing Dataclasses (unchanged)
# ============================================================

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
class Vision:
    color_red_id: int
    color_green_id: int


# ============================================================
# NEW Dataclasses for Navigation & Speed Control
# ============================================================

@dataclass
class PIDParams:
    kp: float
    ki: float
    kd: float


@dataclass
class CornerDetectionParams:
    use_derivative: bool
    use_percentage: bool
    use_graded_steering: bool
    use_imu_confirmation: bool
    lidar_derivative_threshold: float
    pct_threshold: float
    imu_confirm_threshold_radps: float


@dataclass
class GForceParams:
    max_safe_g: float
    filter_alpha: float


@dataclass
class TrafficLightParams:
    distance_slowdown_start_m: float
    distance_slowdown_min_factor: float
    steering_slowdown_max_reduction: float
    lidar_confirm_range_m: float


@dataclass
class OpenChallengeParams:
    base_speed_mps: float
    wall_follow_gain: float
    steer_magnitude_radps: float
    straight_boost_factor: float
    corner_slowdown_max_reduction: float
    predictive_slowdown_gain: float
    pid: PIDParams
    corner_detection: CornerDetectionParams
    g_force: GForceParams


@dataclass
class ObstacleChallengeParams:
    base_speed_mps: float
    wall_follow_gain: float
    steer_magnitude_radps: float
    straight_boost_factor: float
    corner_slowdown_max_reduction: float
    predictive_slowdown_gain: float
    pid: PIDParams
    corner_detection: CornerDetectionParams
    g_force: GForceParams
    traffic_light: TrafficLightParams


@dataclass
class NavigationBehavior:
    open_challenge: OpenChallengeParams
    obstacle_challenge: ObstacleChallengeParams


# ============================================================
# NEW Dataclasses for Emergency Shield (updated)
# ============================================================

@dataclass
class ReverseParams:
    speed_mps: float
    duration_s: float
    rear_clearance_threshold_m: float


@dataclass
class UltrasonicShield:
    enabled: bool
    enable_reverse: bool
    enable_rear_check: bool
    enable_hard_shield: bool
    enable_hard_steer: bool
    thresholds_cm: Dict[str, float]
    dynamic_throttle: Dict[str, Any]
    reverse_params: ReverseParams


# ============================================================
# NEW Dataclasses for Parking
# ============================================================

@dataclass
class ParkingEmergencyShield:
    enabled: bool
    use_parking_thresholds: bool
    thresholds_cm: Dict[str, float]
    dynamic_throttle: Dict[str, Any]
    disable_reverse: bool
    disable_hard_steer: bool
    disable_hard_shield: bool


@dataclass
class ParkingConfig:
    emergency_shield: ParkingEmergencyShield


# ============================================================
# Main SystemConfig Dataclass
# ============================================================

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
    vision: Vision
    navigation: NavigationBehavior
    parking: ParkingConfig


# ============================================================
# Parser Function
# ============================================================

def load_config(yaml_path: str) -> SystemConfig:
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    sm = data['system_prompt_matrix']

    # ---- Zone management (nested) ----
    zm = sm['zone_management']
    open_challenge = ChallengeConfig(**zm['open_challenge'])
    obstacle_challenge = ChallengeConfig(**zm['obstacle_challenge'])
    zone_management = ZoneManagement(open_challenge=open_challenge,
                                     obstacle_challenge=obstacle_challenge)

    # ---- Ultrasonic shield (with new fields) ----
    us = sm['ultrasonic_emergency_shield']
    reverse_params = ReverseParams(**us.get('reverse_params', {
        'speed_mps': -0.10,
        'duration_s': 1.0,
        'rear_clearance_threshold_m': 0.15
    }))
    ultrasonic_shield = UltrasonicShield(
        enabled=us.get('enabled', True),
        enable_reverse=us.get('enable_reverse', True),
        enable_rear_check=us.get('enable_rear_check', True),
        enable_hard_shield=us.get('enable_hard_shield', True),
        enable_hard_steer=us.get('enable_hard_steer', True),
        thresholds_cm=us.get('thresholds_cm', {}),
        dynamic_throttle=us.get('dynamic_throttle', {}),
        reverse_params=reverse_params
    )

    # ---- Navigation (open_challenge + obstacle_challenge) ----
    nav = sm['navigation']

    def parse_challenge_params(data: dict, is_obstacle: bool = False):
        pid = PIDParams(**data.get('pid', {'kp': 25.0, 'ki': 0.1, 'kd': 8.0}))
        corner = CornerDetectionParams(**data.get('corner_detection', {
            'use_derivative': True,
            'use_percentage': True,
            'use_graded_steering': True,
            'use_imu_confirmation': True,
            'lidar_derivative_threshold': 0.3,
            'pct_threshold': 0.4,
            'imu_confirm_threshold_radps': 0.3
        }))
        g_force = GForceParams(**data.get('g_force', {
            'max_safe_g': 0.30,
            'filter_alpha': 0.3
        }))

        if is_obstacle:
            traffic = TrafficLightParams(**data.get('traffic_light', {
                'distance_slowdown_start_m': 2.0,
                'distance_slowdown_min_factor': 0.4,
                'steering_slowdown_max_reduction': 0.4,
                'lidar_confirm_range_m': 2.5
            }))
            return ObstacleChallengeParams(
                base_speed_mps=data.get('base_speed_mps', 1.3),
                wall_follow_gain=data.get('wall_follow_gain', 0.25),
                steer_magnitude_radps=data.get('steer_magnitude_radps', 0.30),
                straight_boost_factor=data.get('straight_boost_factor', 1.10),
                corner_slowdown_max_reduction=data.get(
                    'corner_slowdown_max_reduction', 0.4
                ),
                predictive_slowdown_gain=data.get(
                    'predictive_slowdown_gain', 0.4
                ),
                pid=pid,
                corner_detection=corner,
                g_force=g_force,
                traffic_light=traffic
            )
        else:
            return OpenChallengeParams(
                base_speed_mps=data.get('base_speed_mps', 1.5),
                wall_follow_gain=data.get('wall_follow_gain', 0.30),
                steer_magnitude_radps=data.get('steer_magnitude_radps', 0.35),
                straight_boost_factor=data.get('straight_boost_factor', 1.20),
                corner_slowdown_max_reduction=data.get(
                    'corner_slowdown_max_reduction', 0.3
                ),
                predictive_slowdown_gain=data.get(
                    'predictive_slowdown_gain', 0.3
                ),
                pid=pid,
                corner_detection=corner,
                g_force=g_force
            )

    open_params = parse_challenge_params(
        nav.get('open_challenge', {}), is_obstacle=False
    )
    obstacle_params = parse_challenge_params(
        nav.get('obstacle_challenge', {}), is_obstacle=True
    )
    navigation = NavigationBehavior(open_challenge=open_params,
                                    obstacle_challenge=obstacle_params)

    # ---- Parking ----
    park = sm.get('parking', {})
    park_es = park.get('emergency_shield', {})
    parking_emergency = ParkingEmergencyShield(
        enabled=park_es.get('enabled', True),
        use_parking_thresholds=park_es.get('use_parking_thresholds', True),
        thresholds_cm=park_es.get('thresholds_cm', {}),
        dynamic_throttle=park_es.get('dynamic_throttle', {}),
        disable_reverse=park_es.get('disable_reverse', False),
        disable_hard_steer=park_es.get('disable_hard_steer', False),
        disable_hard_shield=park_es.get('disable_hard_shield', False)
    )
    parking = ParkingConfig(emergency_shield=parking_emergency)

    # ---- Surprise rule ----
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
        color_to_continue=surprise_raw.get(
            'color_to_continue', surprise_defaults['color_to_continue']
        ),
        color_to_reverse=surprise_raw.get(
            'color_to_reverse', surprise_defaults['color_to_reverse']
        ),
        fallback_direction=surprise_raw.get(
            'fallback_direction', surprise_defaults['fallback_direction']
        ),
        turnaround_speed=surprise_raw.get(
            'turnaround_speed', surprise_defaults['turnaround_speed']
        )
    )

    # ---- Lap counting ----
    lap_defaults = {
        'lap_length_m': 12.0,
        'section_fallback_timeout_s': 3.0,
        'emergency_lap_margin_m': 0.2
    }
    lap_raw = sm.get('lap_counting', {})
    lap = LapCounting(
        lap_length_m=lap_raw.get('lap_length_m', lap_defaults['lap_length_m']),
        section_fallback_timeout_s=lap_raw.get(
            'section_fallback_timeout_s', lap_defaults['section_fallback_timeout_s']
        ),
        emergency_lap_margin_m=lap_raw.get(
            'emergency_lap_margin_m', lap_defaults['emergency_lap_margin_m']
        )
    )

    # ---- Vehicle ----
    vehicle_defaults = {
        'wheelbase_m': 0.25,
        'max_speed_mps': 1.5,
        'max_steer_rad': 0.524
    }
    vehicle_raw = sm.get('vehicle', {})
    vehicle = Vehicle(
        wheelbase_m=vehicle_raw.get('wheelbase_m', vehicle_defaults['wheelbase_m']),
        max_speed_mps=vehicle_raw.get('max_speed_mps', vehicle_defaults['max_speed_mps']),
        max_steer_rad=vehicle_raw.get('max_steer_rad', vehicle_defaults['max_steer_rad'])
    )

    # ---- Vision ----
    vision_defaults = {'color_red_id': 1, 'color_green_id': 2}
    vision_raw = sm.get('vision', {})
    vision = Vision(
        color_red_id=vision_raw.get('color_red_id', vision_defaults['color_red_id']),
        color_green_id=vision_raw.get('color_green_id', vision_defaults['color_green_id'])
    )

    # ---- Mapping ----
    mapping_defaults = {
        'use_mapping': True,
        'save_map_to_disk': True,
        'load_map_from_disk': True,
        'force_rebuild': False
    }
    mapping_raw = sm.get('mapping', {})
    mapping = MappingConfig(
        use_mapping=mapping_raw.get('use_mapping', mapping_defaults['use_mapping']),
        save_map_to_disk=mapping_raw.get(
            'save_map_to_disk', mapping_defaults['save_map_to_disk']
        ),
        load_map_from_disk=mapping_raw.get(
            'load_map_from_disk', mapping_defaults['load_map_from_disk']
        ),
        force_rebuild=mapping_raw.get(
            'force_rebuild', mapping_defaults['force_rebuild']
        )
    )

    # ---- Build and return SystemConfig ----
    return SystemConfig(
        version=sm['version'],
        competition=sm['competition'],
        zone_management=zone_management,
        sensor_fusion_and_tracking=SensorFusionTracking(
            **sm['sensor_fusion_and_tracking']
        ),
        ultrasonic_emergency_shield=ultrasonic_shield,
        traffic_light_passing_rules=TrafficLightRules(
            **sm['traffic_light_passing_rules']
        ),
        navigation_matrix=NavigationMatrix(**sm['navigation_matrix']),
        state_machine=StateMachine(**sm['state_machine']),
        mapping=mapping,
        surprise_rule=surprise,
        lap_counting=lap,
        vehicle=vehicle,
        vision=vision,
        navigation=navigation,
        parking=parking
    )
