/**
 * ESP32-S3 Firmware for WRO Future Engineers 2026.
 * 
 * - Reads ultrasonic sensors and encoders.
 * - Controls motors via PWM.
 * - Communicates with Raspberry Pi over UART.
 */

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <ESP32Encoder.h>
#include "pin_definitions.h"

// Serial to Pi (use UART2)
#define UART_BAUD 460800

// Ultrasonic measurement timeout (us)
#define ULTRASONIC_TIMEOUT 30000

// Motor PWM frequency and resolution
#define PWM_FREQ 20000
#define PWM_RES 10  // 0-1023

// Motor max speed (duty cycle)
#define MAX_DUTY 1023

// Encoder objects
ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

// Ultrasonic distances (cm)
volatile float front_dist = 999.0;
volatile float front_left_dist = 999.0;
volatile float front_right_dist = 999.0;

// Last time sensor data sent
unsigned long last_sensor_send = 0;
const unsigned long SENSOR_SEND_INTERVAL = 50; // ms

// Motor speed commands (from Pi)
volatile float target_left = 0.0;
volatile float target_right = 0.0;
volatile bool new_motor_cmd = false;

// Function prototypes
void readUltrasonic();
void sendSensorData();
void processMotorCommand(String json);

void setup() {
    // Initialize serial for debugging (UART0)
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("ESP32-S3 WRO Robot Firmware Starting...");

    // Initialize UART2 for Pi communication
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
    Serial2.setTimeout(10);

    // Setup ultrasonic pins
    pinMode(FRONT_TRIG, OUTPUT);
    pinMode(FRONT_ECHO, INPUT);
    pinMode(FRONT_LEFT_TRIG, OUTPUT);
    pinMode(FRONT_LEFT_ECHO, INPUT);
    pinMode(FRONT_RIGHT_TRIG, OUTPUT);
    pinMode(FRONT_RIGHT_ECHO, INPUT);

    // Setup motor PWM
    ledcSetup(0, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_PWM_A, 0);
    ledcSetup(1, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_PWM_B, 1);
    pinMode(MOTOR_DIR_A, OUTPUT);
    pinMode(MOTOR_DIR_B, OUTPUT);

    // Setup encoders
    ESP32Encoder::useInternalWeakPullResistors = UP;
    encoderLeft.attachHalfQuad(ENC_A_A, ENC_A_B);
    encoderRight.attachHalfQuad(ENC_B_A, ENC_B_B);
    encoderLeft.clearCount();
    encoderRight.clearCount();

    Serial.println("Setup complete.");
}

void loop() {
    // Read ultrasonic sensors every 20 ms
    static unsigned long last_ultra = 0;
    if (millis() - last_ultra > 20) {
        last_ultra = millis();
        readUltrasonic();
    }

    // Send sensor data to Pi every 50 ms
    if (millis() - last_sensor_send > SENSOR_SEND_INTERVAL) {
        last_sensor_send = millis();
        sendSensorData();
    }

    // Process incoming motor commands from Pi
    while (Serial2.available()) {
        String line = Serial2.readStringUntil('\n');
        if (line.length() > 0) {
            processMotorCommand(line);
        }
    }

    // Apply motor speeds if new command received
    if (new_motor_cmd) {
        applyMotorSpeeds(target_left, target_right);
        new_motor_cmd = false;
    }

    // Small delay to yield
    delay(1);
}

void readUltrasonic() {
    // Front
    digitalWrite(FRONT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(FRONT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(FRONT_TRIG, LOW);
    long duration = pulseIn(FRONT_ECHO, HIGH, ULTRASONIC_TIMEOUT);
    if (duration > 0) {
        front_dist = duration * 0.034 / 2.0;
    } else {
        front_dist = 999.0;
    }

    // Front-left
    digitalWrite(FRONT_LEFT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(FRONT_LEFT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(FRONT_LEFT_TRIG, LOW);
    duration = pulseIn(FRONT_LEFT_ECHO, HIGH, ULTRASONIC_TIMEOUT);
    if (duration > 0) {
        front_left_dist = duration * 0.034 / 2.0;
    } else {
        front_left_dist = 999.0;
    }

    // Front-right
    digitalWrite(FRONT_RIGHT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(FRONT_RIGHT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(FRONT_RIGHT_TRIG, LOW);
    duration = pulseIn(FRONT_RIGHT_ECHO, HIGH, ULTRASONIC_TIMEOUT);
    if (duration > 0) {
        front_right_dist = duration * 0.034 / 2.0;
    } else {
        front_right_dist = 999.0;
    }
}

void sendSensorData() {
    // Build JSON: {"type":"ultrasonic","data":{"front":...,"front_left":...,"front_right":...},"checksum":...}
    // For simplicity, we send a comma-separated string with checksum.
    // But using JSON is better; we'll implement a simple JSON-like format.
    // We'll also include encoder counts.
    int64_t encL = encoderLeft.getCount();
    int64_t encR = encoderRight.getCount();

    String msg = "{";
    msg += "\"type\":\"ultrasonic\",";
    msg += "\"data\":{";
    msg += "\"front\":" + String(front_dist) + ",";
    msg += "\"front_left\":" + String(front_left_dist) + ",";
    msg += "\"front_right\":" + String(front_right_dist) + ",";
    msg += "\"enc_left\":" + String(encL) + ",";
    msg += "\"enc_right\":" + String(encR);
    msg += "}";
    // Checksum (simple sum of char codes)
    int sum = 0;
    for (int i = 0; i < msg.length(); i++) {
        sum += msg[i];
    }
    msg += ",\"checksum\":" + String(sum);
    msg += "}\n";
    Serial2.print(msg);
}

void processMotorCommand(String json) {
    // Expect: {"type":"motor_command","data":{"left":0.12,"right":0.34},"checksum":123}
    // Very basic parsing (assume format)
    int leftIdx = json.indexOf("\"left\":");
    int rightIdx = json.indexOf("\"right\":");
    if (leftIdx == -1 || rightIdx == -1) return;

    // Extract numbers
    float left_val = 0.0, right_val = 0.0;
    sscanf(json.substring(leftIdx + 7).c_str(), "%f", &left_val);
    sscanf(json.substring(rightIdx + 8).c_str(), "%f", &right_val);

    target_left = left_val;
    target_right = right_val;
    new_motor_cmd = true;

    // Optional: verify checksum
    // For production, implement proper checksum verification.
}

void applyMotorSpeeds(float left, float right) {
    // Convert speed (m/s) to PWM duty.
    // Map speed to PWM: max_speed = 0.5 m/s -> duty 1023.
    const float MAX_SPEED = 0.5;
    int duty_left = constrain(mapSpeedToDuty(left, MAX_SPEED), -MAX_DUTY, MAX_DUTY);
    int duty_right = constrain(mapSpeedToDuty(right, MAX_SPEED), -MAX_DUTY, MAX_DUTY);

    // Set direction and PWM
    setMotor(MOTOR_PWM_A, MOTOR_DIR_A, duty_left);
    setMotor(MOTOR_PWM_B, MOTOR_DIR_B, duty_right);
}

int mapSpeedToDuty(float speed, float max_speed) {
    if (speed > max_speed) speed = max_speed;
    if (speed < -max_speed) speed = -max_speed;
    return (int)((speed / max_speed) * MAX_DUTY);
}

void setMotor(int pwmPin, int dirPin, int duty) {
    if (duty >= 0) {
        digitalWrite(dirPin, HIGH);
        ledcWrite(pwmPin, duty);
    } else {
        digitalWrite(dirPin, LOW);
        ledcWrite(pwmPin, -duty);
    }
}
