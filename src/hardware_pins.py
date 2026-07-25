"""
Centralized pin mapping for Raspberry Pi 5 and ESP32-S3.
All pins are conflict-free and adhere to the spec.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class PiPins:
    # UART to ESP32-S3 (UART0)
    UART_TX = 14   # Physical pin 8
    UART_RX = 15   # Physical pin 10
    # I2C for BNO086 and other sensors (I2C1)
    I2C_SDA = 2    # Physical pin 3
    I2C_SCL = 3    # Physical pin 5
    # USB is used for HuskyLens and RPLIDAR C1 (via USB serial)


@dataclass(frozen=True)
class ESP32Pins:
    # UART2 to Raspberry Pi (avoid UART0 for debugging)
    UART_TX = 17
    UART_RX = 18

    # Ultrasonic sensors
    FRONT_TRIG = 4
    FRONT_ECHO = 5
    FRONT_LEFT_TRIG = 6
    FRONT_LEFT_ECHO = 7
    FRONT_RIGHT_TRIG = 15
    FRONT_RIGHT_ECHO = 16

    # Motor driver (PWM + direction)
    MOTOR_PWM_A = 1
    MOTOR_DIR_A = 2
    MOTOR_PWM_B = 41
    MOTOR_DIR_B = 42

    # Quadrature encoders (two motors)
    ENC_A_A = 11
    ENC_A_B = 12
    ENC_B_A = 13
    ENC_B_B = 14

    # Avoid strapping pins: GPIO0,3,45,46 and SPI flash pins 26-32
