/**
 * ESP32-S3 Firmware for WRO Future Engineers 2026
 * 
 * Hardware:
 *   - 1 DC drive motor with BTS 7960 driver (two PWM pins)
 *   - 1 steering servo
 *   - 3 ultrasonic sensors (front, front-left, front-right)
 *   - BNO086 IMU (I2C) for yaw rate (200 Hz)
 *   - UART communication with Raspberry Pi (JSON, 460800 baud)
 * 
 * Communication:
 *   - Received JSON: {"type":"cmd","speed":0.3,"steering":0.15}
 *   - Sent JSON: {"type":"sensor_data","data":{"front":12.5,...},"checksum":123}
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

#include "pin_definitions.h"

// ============================================================
// Constants & Configuration
// ============================================================

#define UART_BAUD 460800
#define PWM_FREQ 20000
#define PWM_RES 10                // 0-1023
#define MAX_DUTY 1023
#define MAX_SPEED_MPS 1.5         // Must match YAML vehicle.max_speed_mps
#define ULTRASONIC_TIMEOUT 15000  // 15 ms

#define SERVO_CENTER 90
#define MAX_STEER_RAD 0.524       // ±30° in radians

// ============================================================
// Objects & Variables
// ============================================================

Servo steeringServo;

// Ultrasonic distances (cm)
volatile float front_dist = 999.0;
volatile float front_left_dist = 999.0;
volatile float front_right_dist = 999.0;

// BNO086
Adafruit_BNO08x bno;
volatile float imu_yaw_rate = 0.0;
unsigned long last_imu_read = 0;
const unsigned long IMU_READ_INTERVAL = 5000;  // µs (200 Hz)

// Command variables (updated by UART from Pi)
volatile float target_speed = 0.0;       // m/s, positive = forward
volatile float target_steer = 0.0;       // radians, positive = left
volatile bool new_cmd = false;

unsigned long last_sensor_send = 0;
const unsigned long SENSOR_SEND_INTERVAL = 50;  // ms (20 Hz)

// ============================================================
// Setup
// ============================================================

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("ESP32-S3 WRO Firmware (BTS7960, BNO086)");

    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
    Serial2.setTimeout(10);

    // Ultrasonic pins
    pinMode(FRONT_TRIG, OUTPUT);
    pinMode(FRONT_ECHO, INPUT);
    pinMode(FRONT_LEFT_TRIG, OUTPUT);
    pinMode(FRONT_LEFT_ECHO, INPUT);
    pinMode(FRONT_RIGHT_TRIG, OUTPUT);
    pinMode(FRONT_RIGHT_ECHO, INPUT);

    // ---- Motor driver (BTS 7960) ----
    // Motor PWM pins
    ledcSetup(0, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_FWD_PWM, 0);
    ledcSetup(1, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_REV_PWM, 1);

    // Enable pins (pull HIGH to enable the driver)
    // If you hardwire EN pins to 3.3V, you can skip this
    pinMode(MOTOR_EN, OUTPUT);
    digitalWrite(MOTOR_EN, HIGH);

    // ---- Servo ----
    steeringServo.attach(SERVO_PWM);
    steeringServo.write(SERVO_CENTER);
    delay(200);

    // ---- BNO086 ----
    Wire.begin(IMU_SDA, IMU_SCL);
    if (!bno.begin_I2C(0x4B)) {
        if (!bno.begin_I2C(0x4A)) {
            Serial.println("BNO086 not found at 0x4B or 0x4A");
        } else {
            Serial.println("BNO086 found at 0x4A");
            bno.enableReport(SH2_GYROSCOPE_CALIBRATED, IMU_READ_INTERVAL);
        }
    } else {
        Serial.println("BNO086 found at 0x4B");
        bno.enableReport(SH2_GYROSCOPE_CALIBRATED, IMU_READ_INTERVAL);
    }

    Serial.println("Setup complete.");
}

// ============================================================
// Ultrasonic Reading (15 ms timeout)
// ============================================================

void readUltrasonic() {
    // Front
    digitalWrite(FRONT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(FRONT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(FRONT_TRIG, LOW);
    long duration = pulseIn(FRONT_ECHO, HIGH, ULTRASONIC_TIMEOUT);
    front_dist = (duration > 0) ? duration * 0.034 / 2.0 : 999.0;

    // Front-left
    digitalWrite(FRONT_LEFT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(FRONT_LEFT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(FRONT_LEFT_TRIG, LOW);
    duration = pulseIn(FRONT_LEFT_ECHO, HIGH, ULTRASONIC_TIMEOUT);
    front_left_dist = (duration > 0) ? duration * 0.034 / 2.0 : 999.0;

    // Front-right
    digitalWrite(FRONT_RIGHT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(FRONT_RIGHT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(FRONT_RIGHT_TRIG, LOW);
    duration = pulseIn(FRONT_RIGHT_ECHO, HIGH, ULTRASONIC_TIMEOUT);
    front_right_dist = (duration > 0) ? duration * 0.034 / 2.0 : 999.0;
}

// ============================================================
// IMU Reading (BNO086) – Calibrated Gyro
// ============================================================

void readIMU() {
    sh2_gyroscope_calibrated_t gyro;
    if (bno.getSensorEvent(&gyro)) {
        imu_yaw_rate = gyro.z;   // rad/s
    }
}

// ============================================================
// Motor Speed Control – BTS 7960 (Two PWM pins)
// ============================================================

int mapSpeedToDuty(float speed, float max_speed) {
    // Clamp speed
    if (speed > max_speed) speed = max_speed;
    if (speed < -max_speed) speed = -max_speed;
    // Map to duty cycle (0-1023)
    return (int)((speed / max_speed) * MAX_DUTY);
}

void applyMotorSpeed(float speed_mps) {
    int duty = mapSpeedToDuty(speed_mps, MAX_SPEED_MPS);
    // duty is between -MAX_DUTY and +MAX_DUTY
    if (duty > 0) {
        // Forward: RPWM = duty, LPWM = 0
        ledcWrite(0, duty);
        ledcWrite(1, 0);
    } else if (duty < 0) {
        // Reverse: RPWM = 0, LPWM = -duty
        ledcWrite(0, 0);
        ledcWrite(1, -duty);
    } else {
        // Stop: both 0
        ledcWrite(0, 0);
        ledcWrite(1, 0);
    }
}

// ============================================================
// Servo Steering Control
// ============================================================

void applyServoAngle(float angle_rad) {
    float clamped = constrain(angle_rad, -MAX_STEER_RAD, MAX_STEER_RAD);
    int angle_deg = SERVO_CENTER + (int)((clamped / MAX_STEER_RAD) * 90.0);
    angle_deg = constrain(angle_deg, 0, 180);
    steeringServo.write(angle_deg);
}

// ============================================================
// Send Sensor Data (JSON) to Raspberry Pi
// ============================================================

void sendSensorData() {
    String msg = "{";
    msg += "\"type\":\"sensor_data\",";
    msg += "\"data\":{";
    msg += "\"front\":" + String(front_dist) + ",";
    msg += "\"front_left\":" + String(front_left_dist) + ",";
    msg += "\"front_right\":" + String(front_right_dist) + ",";
    msg += "\"imu_yaw_rate\":" + String(imu_yaw_rate, 6);
    msg += "}";

    int sum = 0;
    for (int i = 0; i < msg.length(); i++) sum += msg[i];
    msg += ",\"checksum\":" + String(sum);
    msg += "}\n";

    Serial2.print(msg);
}

// ============================================================
// Command Processing (JSON from Raspberry Pi)
// ============================================================

void processCommand(String json) {
    float speed = 0.0, steer = 0.0;
    int idxSpeed = json.indexOf("\"speed\":");
    int idxSteer = json.indexOf("\"steering\":");
    if (idxSpeed != -1) {
        sscanf(json.substring(idxSpeed + 8).c_str(), "%f", &speed);
    }
    if (idxSteer != -1) {
        sscanf(json.substring(idxSteer + 11).c_str(), "%f", &steer);
    }

    // Clamp speed and steering
    if (speed > MAX_SPEED_MPS) speed = MAX_SPEED_MPS;
    if (speed < -MAX_SPEED_MPS) speed = -MAX_SPEED_MPS;
    if (steer > MAX_STEER_RAD) steer = MAX_STEER_RAD;
    if (steer < -MAX_STEER_RAD) steer = -MAX_STEER_RAD;

    target_speed = speed;
    target_steer = steer;
    new_cmd = true;
}

// ============================================================
// Main Loop (non‑blocking)
// ============================================================

void loop() {
    // Ultrasonics every 20 ms (50 Hz)
    static unsigned long last_ultra = 0;
    if (millis() - last_ultra > 20) {
        last_ultra = millis();
        readUltrasonic();
    }

    // IMU every 5 ms (200 Hz)
    if (micros() - last_imu_read >= IMU_READ_INTERVAL) {
        last_imu_read = micros();
        readIMU();
    }

    // Send sensor data every 50 ms (20 Hz) to Pi
    if (millis() - last_sensor_send > SENSOR_SEND_INTERVAL) {
        last_sensor_send = millis();
        sendSensorData();
    }

    // Process incoming commands from Pi
    while (Serial2.available()) {
        String line = Serial2.readStringUntil('\n');
        if (line.length() > 0) processCommand(line);
    }

    // Apply motor and servo if new command received
    if (new_cmd) {
        applyMotorSpeed(target_speed);
        applyServoAngle(target_steer);
        new_cmd = false;
    }

    delay(1);  // yield to other tasks
}
