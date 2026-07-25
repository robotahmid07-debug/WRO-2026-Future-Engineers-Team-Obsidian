#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

// UART2 to Raspberry Pi (avoid UART0 for debugging)
#define UART_TX 17
#define UART_RX 18

// Ultrasonic sensors
#define FRONT_TRIG 4
#define FRONT_ECHO 5
#define FRONT_LEFT_TRIG 6
#define FRONT_LEFT_ECHO 7
#define FRONT_RIGHT_TRIG 15
#define FRONT_RIGHT_ECHO 16

// Motor driver (PWM + direction)
#define MOTOR_PWM_A 1
#define MOTOR_DIR_A 2
#define MOTOR_PWM_B 41
#define MOTOR_DIR_B 42

// Quadrature encoders
#define ENC_A_A 11
#define ENC_A_B 12
#define ENC_B_A 13
#define ENC_B_B 14

#endif
