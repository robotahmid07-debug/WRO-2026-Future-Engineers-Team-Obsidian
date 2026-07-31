"""
Centralized pin mapping for Raspberry Pi 5 and ESP32-S3.
All pins are conflict-free and adhere to the spec.

Hardware Configuration:
  - Raspberry Pi 5: UART to ESP32, I2C for HuskyLens, Mode Switch, Direction Switch
  - ESP32-S3: One drive motor (PWM + DIR), one steering servo (PWM),
              3 ultrasonic sensors, IMU (I2C)
  - Encoders: NOT USED (removed)
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class PiPins:
    # ============================================================
    # UART to ESP32-S3 (UART0)
    # ============================================================
    UART_TX = 14   # Physical pin 8
    UART_RX = 15   # Physical pin 10

    # ============================================================
    # I2C for HuskyLens and other sensors (I2C1)
    # ============================================================
    I2C_SDA = 2    # Physical pin 3
    I2C_SCL = 3    # Physical pin 5

    # ============================================================
    # Mode Selection Switch (Open / Obstacle Challenge)
    # ============================================================
    # Switch CLOSED (LOW)  = Obstacle Challenge
    # Switch OPEN  (HIGH) = Open Challenge
    MODE_SELECT = 22   # Physical pin 15

    # ============================================================
    # Direction Selection Switch (Clockwise / Counter-Clockwise)
    # ============================================================
    # Switch CLOSED (LOW)  = Counter-Clockwise
    # Switch OPEN  (HIGH) = Clockwise
    DIRECTION_SELECT = 23   # Physical pin 16

    # ============================================================
    # Start Button (GPIO 26)
    # ============================================================
    # Button pressed (LOW) starts the round
    START_BUTTON = 26   # Physical pin 37

    # ============================================================
    # USB is used for HuskyLens and RPLIDAR C1 (via USB serial)
    # No additional GPIO needed on Pi for this project.
    # ============================================================


@dataclass(frozen=True)
class ESP32Pins:
    # ============================================================
    # UART2 to Raspberry Pi (avoid UART0 for debugging)
    # ============================================================
    UART_TX = 17
    UART_RX = 18

    # ============================================================
    # Ultrasonic sensors (HC-SR04)
    # ============================================================
    FRONT_TRIG = 4
    FRONT_ECHO = 5
    FRONT_LEFT_TRIG = 6
    FRONT_LEFT_ECHO = 7
    FRONT_RIGHT_TRIG = 15
    FRONT_RIGHT_ECHO = 16

    # ============================================================
    # SINGLE DRIVE MOTOR (PWM + Direction)
    # ============================================================
    MOTOR_PWM = 1     # PWM speed control
    MOTOR_DIR = 2     # Direction (HIGH = forward, LOW = reverse)

    # ============================================================
    # STEERING SERVO (PWM)
    # ============================================================
    SERVO_PWM = 41    # PWM signal for steering servo

    # ============================================================
    # BNO086 IMU – I2C Bus
    # ============================================================
    # Connected to ESP32-S3 I2C peripheral
    IMU_SDA = 8
    IMU_SCL = 9

    # ============================================================
    # NOTE: Avoid strapping pins: GPIO0, 3, 45, 46
    #       Avoid SPI flash pins: GPIO26–32
    # ============================================================
