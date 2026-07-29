#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

// ============================================================
// UART2 to Raspberry Pi (Hardware Serial)
// ============================================================
// GPIO 17 (TX) → Pi GPIO 15 (RX)
// GPIO 18 (RX) ← Pi GPIO 14 (TX)
#define UART_TX 17
#define UART_RX 18

// ============================================================
// Ultrasonic Sensors (HC-SR04)
// ============================================================
#define FRONT_TRIG       4
#define FRONT_ECHO       5

#define FRONT_LEFT_TRIG  6
#define FRONT_LEFT_ECHO  7

#define FRONT_RIGHT_TRIG 15
#define FRONT_RIGHT_ECHO 16

// ============================================================
// Motor Driver (TB6612FNG / L298N / BTS 7960 compatible)
// ============================================================
// Single drive motor – PWM speed and Direction
#define MOTOR_PWM        1
#define MOTOR_DIR        2

// ============================================================
// Steering Servo (Ackermann steering)
// ============================================================
#define SERVO_PWM        41

// ============================================================
// BNO086 IMU – I2C Bus
// ============================================================
// Connected to ESP32-S3 I2C peripheral
#define IMU_SDA          8
#define IMU_SCL          9

#endif
