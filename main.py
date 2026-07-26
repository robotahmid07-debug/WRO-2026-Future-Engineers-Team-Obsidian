#!/usr/bin/env python3
"""
Primary execution entrypoint for WRO Future Engineers 2026.
Reads a hardware switch to select Open or Obstacle challenge.
Uses WallMapper for map‑based localization, smart obstacle avoidance,
and LIDAR‑based parallel parking.
"""

import sys
import time
import signal
import logging
from pathlib import Path

# GPIO for mode selection
import RPi.GPIO as GPIO

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / 'src'))

from src.config_parser import load_config
from src.hardware_pins import PiPins
from src.serial_bridge import SerialBridge
from src.vision_tracker import HuskyLensReader
from src.lidar_fusion import LidarFusion
from src.spatial_map import SpatialMap
from src.emergency_shield import EmergencyShield
from src.localization import Localization
from src.steering_controller import SteeringController
from src.parking_controller import ParkingController
from src.state_machine import StateMachine

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Global flag for graceful shutdown
running = True


def signal_handler(sig, frame):
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
    # 2. Read hardware switch to determine challenge mode
    # ------------------------------------------------------------------
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PiPins.MODE_SELECT, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    if GPIO.input(PiPins.MODE_SELECT) == GPIO.LOW:
        challenge = "obstacle"
        logger.info("Hardware switch: OBSTACLE challenge (parallel parking)")
    else:
        challenge = "open"
        logger.info("Hardware switch: OPEN challenge (stop at start)")

    # ------------------------------------------------------------------
    # 3. Initialize hardware interfaces
    # ------------------------------------------------------------------
    # Serial bridge (Pi <-> ESP32)
    serial_bridge = SerialBridge(port='/dev/ttyAMA0', baudrate=460800)
    serial_bridge.open()
    logger.info("Serial bridge opened")

    # HuskyLens V2 on I2C bus 1
    vision = HuskyLensReader(i2c_bus=1, poll_interval=0.05)
    if not vision.open():
        logger.error("Failed to initialize HuskyLens V2. Check I2C connection.")
    else:
        logger.info("HuskyLens V2 initialized")

    # LIDAR – used for mapping, corner detection, traffic light fusion, and parking
    lidar = LidarFusion(port='/dev/ttyUSB1')
    lidar.open()
    logger.info("LIDAR initialized")

    # Spatial map for object tracking
    spatial_config = config.sensor_fusion_and_tracking.host_spatial_tracker
    spatial_map = SpatialMap(spatial_config)
    logger.info("Spatial map initialized")

    # ------------------------------------------------------------------
    # 4. Localization with WallMapper (map correction)
    # ------------------------------------------------------------------
    localization = Localization(lidar, config.zone_management, use_map_correction=True)
    logger.info("Localization initialized (map correction enabled)")

    # ------------------------------------------------------------------
    # 5. Ackermann steering controller
    # ------------------------------------------------------------------
    steering = SteeringController(
        serial_bridge,
        wheelbase=0.25,     # adjust to your car's wheelbase
        trackwidth=0.15,    # adjust to your car's track width
        max_speed=0.5       # max forward speed (m/s)
    )
    logger.info("Steering controller initialized (Ackermann)")

    # ------------------------------------------------------------------
    # 6. Smart Emergency Shield (traffic‑rule‑aware obstacle avoidance)
    # ------------------------------------------------------------------
    emergency = EmergencyShield(config.ultrasonic_emergency_shield, serial_bridge)
    logger.info("Emergency shield initialized (smart avoidance)")

    # ------------------------------------------------------------------
    # 7. LIDAR‑based Parking Controller (uses 360° LIDAR for precision)
    # ------------------------------------------------------------------
    # NOTE: We pass `lidar` here, NOT `emergency` – LIDAR gives accurate 360° distances.
    parking = ParkingController(localization, steering, lidar, config.zone_management)
    logger.info("Parking controller initialized (LIDAR‑based)")

    # ------------------------------------------------------------------
    # 8. State Machine
    # ------------------------------------------------------------------
    fsm = StateMachine(config, serial_bridge, localization, vision, lidar,
                       spatial_map, emergency, steering, parking,
                       challenge=challenge)
    logger.info("State machine initialized with %s challenge", challenge)

    # ------------------------------------------------------------------
    # 9. Optional: Load a previously saved map (if available)
    # ------------------------------------------------------------------
    map_file = Path(__file__).parent / 'saved_map.pkl'
    if map_file.exists() and localization.is_map_ready():
        try:
            localization.load_map(str(map_file))
            logger.info("Saved map loaded successfully")
        except Exception as e:
            logger.warning(f"Failed to load map: {e}")

    # ------------------------------------------------------------------
    # 10. Setup signal handlers and main loop (20 Hz)
    # ------------------------------------------------------------------
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rate = 20.0  # Hz
    dt = 1.0 / rate
    last_time = time.time()
    logger.info("Starting main control loop at %.1f Hz", rate)

    global running
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
    # 11. Optional: Save the map for next run (if built)
    # ------------------------------------------------------------------
    if localization.is_map_ready():
        try:
            localization.save_map(str(map_file))
            logger.info("Map saved for next run")
        except Exception as e:
            logger.warning(f"Failed to save map: {e}")

    # ------------------------------------------------------------------
    # 12. Cleanup
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
