#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

// UART2 to Raspberry Pi (avoid UART0 for debugging)
#define UART_TX 17
#define UART_RX 18

// Ultrasonic sensors (unchanged)
#define FRONT_TRIG 4
#define FRONT_ECHO 5
#define FRONT_LEFT_TRIG 6
#define FRONT_LEFT_ECHO 7
#define FRONT_RIGHT_TRIG 15
#define FRONT_RIGHT_ECHO 16

// ---- SINGLE DRIVE MOTOR ----
#define MOTOR_PWM 1
#define MOTOR_DIR 2

// ---- STEERING SERVO ----
#define SERVO_PWM 41   // PWM pin for servo

// ---- QUADRATURE ENCODERS (optional) ----
#define ENC_A_A 11
#define ENC_A_B 12
// Only one encoder needed (on the drive motor or axle)
// If you have two encoders, you can keep both, but they will read the same.

#endif
