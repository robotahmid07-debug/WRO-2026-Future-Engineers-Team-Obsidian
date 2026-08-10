#!/usr/bin/env python3
"""
Primary execution entrypoint for WRO Future Engineers 2026.
Reads hardware switches to select:
  1. Challenge mode (Open / Obstacle) – GPIO 22
  2. Driving direction (Clockwise / Counter-Clockwise) – either from GPIO 23
     or auto‑detected from LIDAR (config‑toggleable).
  3. Start button (GPIO 26) – must be pressed to begin the round

All calibratable parameters are read from config.

Hardware interfaces:
  - LIDAR: RPLIDAR A1 on /dev/rplidar (if udev rule exists) or /dev/ttyUSB1
  - HuskyLens V2 on I2C bus 1
  - ESP32-S3 on /dev/ttyAMA0 (UART)
  - GPIO switches for mode, direction, start
"""

import sys
import time
import signal
import logging
from pathlib import Path

# GPIO for mode selection and start button
import RPi.GPIO as GPIO  # noqa: E402

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / 'src'))

from src.config_parser import load_config  # noqa: E402
from src.hardware_pins import PiPins  # noqa: E402
from src.serial_bridge import SerialBridge  # noqa: E402
from src.vision_tracker import HuskyLensReader  # noqa: E402
from src.lidar_fusion import LidarFusion  # noqa: E402
from src.spatial_map import SpatialMap  # noqa: E402
from src.emergency_shield import EmergencyShield  # noqa: E402
from src.localization import Localization  # noqa: E402
from src.steering_controller import SteeringController  # noqa: E402
from src.parking_controller import ParkingController  # noqa: E402
from src.state_machine import StateMachine  # noqa: E402
from src.wall_mapper import detect_direction_from_scan  # noqa: E402

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Global flag for graceful shutdown
running = True


def signal_handler(sig, frame):
    """Handle SIGINT and SIGTERM for graceful shutdown."""
    global running
    logger.info("Caught interrupt signal, shutting down...")
    running = False


def main():
    # ------------------------------------------------------------------
    # 1. Load configuration
    # ------------------------------------------------------------------
    config_path = Path(__file__).parent / 'config' / 'system_prompt_matrix.yaml'
    config = load_config(config_path)
    logger.info("Configuration loaded: %s v%s", config.competition, config.version)

    # ------------------------------------------------------------------
    # 2. Read hardware switches to determine challenge and direction
    # ------------------------------------------------------------------
    GPIO.setmode(GPIO.BCM)

    # ---- 2a. Mode switch (GPIO 22) ----
    GPIO.setup(PiPins.MODE_SELECT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    if GPIO.input(PiPins.MODE_SELECT) == GPIO.LOW:
        challenge = "obstacle"
        logger.info("Hardware switch: OBSTACLE challenge (parallel parking)")
    else:
        challenge = "open"
        logger.info("Hardware switch: OPEN challenge (stop at start)")

    # ---- 2b. LIDAR initialization (moved earlier for auto‑direction) ----
    # Detect LIDAR port: prefer udev symlink /dev/rplidar, fallback to /dev/ttyUSB1
    lidar_port = '/dev/rplidar'
    if not Path(lidar_port).exists():
        lidar_port = '/dev/ttyUSB1'
        logger.info(f"Using LIDAR port: {lidar_port}")
    else:
        logger.info(f"Using LIDAR port: {lidar_port} (symlink)")

    lidar = LidarFusion(port=lidar_port)
    lidar.open()
    logger.info("LIDAR initialized")

    # ---- 2c. Direction: switch (manual) or auto (LIDAR‑based) ----
    direction_mode = config.navigation.direction_mode

    if direction_mode == "auto":
        logger.info("Direction mode: AUTO (LIDAR‑based detection)")
        initial_scan = lidar.get_scan_snapshot()
        initial_direction = detect_direction_from_scan(
            initial_scan,
            window_deg=config.navigation.auto_direction_scan_window_deg,
            min_confidence_m=config.navigation.auto_direction_min_confidence_m,
        )
        logger.info(f"Auto‑detected direction: {initial_direction}")
    else:
        logger.info("Direction mode: SWITCH (manual override)")
        GPIO.setup(PiPins.DIRECTION_SELECT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        if GPIO.input(PiPins.DIRECTION_SELECT) == GPIO.LOW:
            initial_direction = "COUNTER_CLOCKWISE"
            logger.info("Direction switch: COUNTER‑CLOCKWISE")
        else:
            initial_direction = "CLOCKWISE"
            logger.info("Direction switch: CLOCKWISE")

    # ---- 2d. Start button (GPIO 26) ----
    START_BUTTON_PIN = 26   # Physical pin 37
    GPIO.setup(START_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # ------------------------------------------------------------------
    # 3. Initialize remaining hardware interfaces
    # ------------------------------------------------------------------
    # ---- 3a. Serial bridge (Pi <-> ESP32) ----
    serial_bridge = SerialBridge(port='/dev/ttyAMA0', baudrate=460800)
    # The port is already opened in __init__, so no need for .open()
    logger.info("Serial bridge opened on /dev/ttyAMA0 at 460800 baud")

    # ---- 3b. HuskyLens V2 on I2C bus 1 ----
    vision = HuskyLensReader(i2c_bus=1, poll_interval=0.05)
    if not vision.open():
        logger.error("Failed to initialize HuskyLens V2. Check I2C connection.")
        # Continue anyway – vision is critical, but we don't exit to allow debugging.
    else:
        logger.info("HuskyLens V2 initialized on I2C bus 1")

    # ---- 3c. Spatial map for object tracking ----
    spatial_config = config.sensor_fusion_and_tracking.host_spatial_tracker
    spatial_map = SpatialMap(spatial_config)
    logger.info("Spatial map initialized")

    # ------------------------------------------------------------------
    # 4. Initialize localization with WallMapper
    # ------------------------------------------------------------------
    localization = Localization(
        lidar,
        config,
        use_map_correction=config.mapping.use_mapping
    )
    logger.info("Localization initialized (map correction: %s)",
                config.mapping.use_mapping)

    # ------------------------------------------------------------------
    # 5. Handle map persistence (save/load/force rebuild)
    # ------------------------------------------------------------------
    map_file = Path(__file__).parent / 'saved_map.pkl'

    # Force rebuild: delete existing map file
    if config.mapping.force_rebuild and map_file.exists():
        try:
            map_file.unlink()
            logger.info("Force rebuild: removed existing map file")
        except Exception as e:
            logger.warning(f"Could not delete map file: {e}")

    # Load map if enabled
    if config.mapping.load_map_from_disk and map_file.exists():
        try:
            localization.load_map(str(map_file))
            logger.info("Map loaded from disk")
        except Exception as e:
            logger.warning(f"Failed to load map: {e}")
    elif config.mapping.load_map_from_disk and not map_file.exists():
        logger.info("No saved map found; will build fresh during Lap 1")
    else:
        logger.info("Map loading disabled by configuration")

    # ------------------------------------------------------------------
    # 6. Initialize steering controller (Ackermann – one motor + servo)
    # ------------------------------------------------------------------
    steering = SteeringController(
        serial_bridge,
        max_speed=config.vehicle.max_speed_mps,
        max_steer_rad=config.vehicle.max_steer_rad,
        smoothing_alpha=0.25,
        steer_gain=1.0
    )
    logger.info("Steering controller initialized (Ackermann)")

    # Pass localization to steering for IMU U‑turn
    steering.set_localization(localization)

    # ------------------------------------------------------------------
    # 7. Emergency shield (smart avoidance)
    # ------------------------------------------------------------------
    emergency = EmergencyShield(config.ultrasonic_emergency_shield, serial_bridge)
    logger.info("Emergency shield initialized")

    # ------------------------------------------------------------------
    # 8. LIDAR‑based parking controller
    # ------------------------------------------------------------------
    parking = ParkingController(localization, steering, lidar, config.zone_management)

    # ---- IMPORTANT: Pass emergency shield reference to parking controller ----
    # This enables LOGIC E (ultrasonic + LIDAR fusion) and LOGIC G (touch avoidance)
    parking.set_emergency_shield(emergency)
    logger.info("Parking controller initialized with emergency shield reference")

    # ------------------------------------------------------------------
    # 9. State Machine with hardware-selected direction
    # ------------------------------------------------------------------
    fsm = StateMachine(
        config,
        serial_bridge,
        localization,
        vision,
        lidar,
        spatial_map,
        emergency,
        steering,
        parking,
        challenge=challenge,
        initial_direction=initial_direction
    )
    logger.info("State machine initialized with %s challenge, %s direction",
                challenge, initial_direction)

    # ------------------------------------------------------------------
    # 10. Wait for Start button (WRO rules)
    # ------------------------------------------------------------------
    logger.info("Waiting for Start button (GPIO 26)...")
    while GPIO.input(START_BUTTON_PIN) == GPIO.HIGH:
        time.sleep(0.01)   # Small delay to avoid busy-waiting
    logger.info("Start button pressed – starting the round!")

    # ------------------------------------------------------------------
    # 11. Setup signal handlers and main loop (20 Hz)
    # ------------------------------------------------------------------
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rate = 20.0  # Hz
    dt = 1.0 / rate
    last_time = time.time()
    logger.info("Starting main control loop at %.1f Hz", rate)

    while running:
        try:
            fsm.run()

            # Maintain loop rate
            elapsed = time.time() - last_time
            if elapsed < dt:
                time.sleep(dt - elapsed)
            last_time = time.time()

        except KeyboardInterrupt:
            break
        except Exception as e:
            logger.exception("Exception in main loop: %s", e)
            time.sleep(0.1)

    # ------------------------------------------------------------------
    # 12. Save map if enabled
    # ------------------------------------------------------------------
    if config.mapping.save_map_to_disk and localization.is_map_ready():
        try:
            localization.save_map(str(map_file))
            logger.info("Map saved to disk")
        except Exception as e:
            logger.warning(f"Failed to save map: {e}")
    elif not config.mapping.save_map_to_disk:
        logger.info("Map saving disabled by configuration")

    # ------------------------------------------------------------------
    # 13. Cleanup
    # ------------------------------------------------------------------
    logger.info("Shutting down...")
    steering.stop()
    vision.close()
    lidar.close()
    serial_bridge.close()
    GPIO.cleanup()
    logger.info("Shutdown complete.")


if __name__ == '__main__':
    main()
