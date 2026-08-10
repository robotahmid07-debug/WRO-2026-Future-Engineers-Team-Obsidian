#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

// ============================================================
// UART2 to Raspberry Pi (Hardware Serial)
// ============================================================
// GPIO 17 (TX) → Pi GPIO 15 (RX)
// GPIO 18 (RX) ← Pi GPIO 14 (TX)
#define SERIAL_TX_PIN 17
#define SERIAL_RX_PIN 18

// ============================================================
// Ultrasonic Sensors (HC-SR04)
// ============================================================
#define TRIG1  4   // Front centre
#define ECHO1  5
#define TRIG2  6   // Front left
#define ECHO2  7
#define TRIG3  15  // Front right
#define ECHO3  16

// ============================================================
// Motor Driver (BTS 7960) – Enables tied to 5V
// ============================================================
// Single drive motor – forward PWM (RPWM) and reverse PWM (LPWM)
// R_EN and L_EN are tied directly to 5V – no GPIO control needed.
#define MOTOR_PWM1  1   // RPWM – forward PWM
#define MOTOR_PWM2  2   // LPWM – reverse PWM

// ============================================================
// Steering Servo (Ackermann steering)
// ============================================================
#define SERVO_PIN  41   // PWM output

// ============================================================
// BNO086 IMU – I2C Bus
// ============================================================
#define I2C_SDA  8
#define I2C_SCL  9

#endif
