/**
 * ESP32-S3 Firmware for WRO Future Engineers 2026
 * 
 * Hardware: single motor + servo, 3 ultrasonics, BNO086 IMU (I2C)
 * Communication: JSON with XOR checksum, type="sensor_data"
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "pin_definitions.h"

// ============================================================
// Constants
// ============================================================
#define UART_BAUD 460800
#define PWM_FREQ 20000
#define PWM_RES 10
#define MAX_DUTY 1023
#define MAX_SPEED_MPS 1.5
#define ULTRASONIC_TIMEOUT 15000  // 15 ms

#define SERVO_CENTER 90
#define MAX_STEER_RAD 0.524       // ±30°

// ============================================================
// Objects & Variables
// ============================================================
Servo steeringServo;

volatile float front_dist = 999.0;
volatile float front_left_dist = 999.0;
volatile float front_right_dist = 999.0;

Adafruit_BNO08x bno;
volatile float imu_yaw_rate = 0.0;
unsigned long last_imu_read = 0;
const unsigned long IMU_READ_INTERVAL = 5000;  // µs (200 Hz)

volatile float target_speed = 0.0;       // m/s
volatile float target_steer = 0.0;       // radians
volatile bool new_cmd = false;

unsigned long last_sensor_send = 0;
const unsigned long SENSOR_SEND_INTERVAL = 50;  // ms (20 Hz)

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("ESP32-S3 WRO Firmware (XOR checksum, sensor_data type)");

    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
    Serial2.setTimeout(10);

    // Ultrasonic pins
    pinMode(FRONT_TRIG, OUTPUT);
    pinMode(FRONT_ECHO, INPUT);
    pinMode(FRONT_LEFT_TRIG, OUTPUT);
    pinMode(FRONT_LEFT_ECHO, INPUT);
    pinMode(FRONT_RIGHT_TRIG, OUTPUT);
    pinMode(FRONT_RIGHT_ECHO, INPUT);

    // Motor PWM
    ledcSetup(0, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_PWM, 0);
    pinMode(MOTOR_DIR, OUTPUT);
    digitalWrite(MOTOR_DIR, LOW);

    // Servo
    steeringServo.attach(SERVO_PWM);
    steeringServo.write(SERVO_CENTER);
    delay(200);

    // BNO086 IMU
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
    digitalWrite(FRONT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(FRONT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(FRONT_TRIG, LOW);
    long duration = pulseIn(FRONT_ECHO, HIGH, ULTRASONIC_TIMEOUT);
    front_dist = (duration > 0) ? duration * 0.034 / 2.0 : 999.0;

    digitalWrite(FRONT_LEFT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(FRONT_LEFT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(FRONT_LEFT_TRIG, LOW);
    duration = pulseIn(FRONT_LEFT_ECHO, HIGH, ULTRASONIC_TIMEOUT);
    front_left_dist = (duration > 0) ? duration * 0.034 / 2.0 : 999.0;

    digitalWrite(FRONT_RIGHT_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(FRONT_RIGHT_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(FRONT_RIGHT_TRIG, LOW);
    duration = pulseIn(FRONT_RIGHT_ECHO, HIGH, ULTRASONIC_TIMEOUT);
    front_right_dist = (duration > 0) ? duration * 0.034 / 2.0 : 999.0;
}

// ============================================================
// IMU Reading (BNO086)
// ============================================================
void readIMU() {
    sh2_gyroscope_calibrated_t gyro;
    if (bno.getSensorEvent(&gyro)) {
        imu_yaw_rate = gyro.z;  // rad/s
    }
}

// ============================================================
// Send Sensor Data (JSON) – XOR Checksum
// ============================================================
void sendSensorData() {
    // Build JSON without checksum
    String msg = "{";
    msg += "\"type\":\"sensor_data\",";
    msg += "\"data\":{";
    msg += "\"front\":" + String(front_dist) + ",";
    msg += "\"front_left\":" + String(front_left_dist) + ",";
    msg += "\"front_right\":" + String(front_right_dist) + ",";
    msg += "\"imu_yaw_rate\":" + String(imu_yaw_rate, 6);
    msg += "}";

    // Compute XOR checksum over the string (excluding checksum)
    int checksum = 0;
    for (int i = 0; i < msg.length(); i++) {
        checksum ^= msg[i];
    }
    msg += ",\"checksum\":" + String(checksum);
    msg += "}\n";

    Serial2.print(msg);
}

// ============================================================
// Command Processing (JSON from Pi)
// ============================================================
void processCommand(String json) {
    float speed = 0.0, steer = 0.0;
    int idxSpeed = json.indexOf("\"speed\":");
    int idxSteer = json.indexOf("\"steering\":");
    if (idxSpeed != -1) sscanf(json.substring(idxSpeed + 8).c_str(), "%f", &speed);
    if (idxSteer != -1) sscanf(json.substring(idxSteer + 11).c_str(), "%f", &steer);
    // Clamp
    if (speed > MAX_SPEED_MPS) speed = MAX_SPEED_MPS;
    if (speed < -MAX_SPEED_MPS) speed = -MAX_SPEED_MPS;
    if (steer > MAX_STEER_RAD) steer = MAX_STEER_RAD;
    if (steer < -MAX_STEER_RAD) steer = -MAX_STEER_RAD;
    target_speed = speed;
    target_steer = steer;
    new_cmd = true;
}

// ============================================================
// Motor Speed Control
// ============================================================
void applyMotorSpeed(float speed_mps) {
    int duty = (int)((speed_mps / MAX_SPEED_MPS) * MAX_DUTY);
    if (duty > MAX_DUTY) duty = MAX_DUTY;
    if (duty < -MAX_DUTY) duty = -MAX_DUTY;
    if (duty >= 0) {
        digitalWrite(MOTOR_DIR, HIGH);
        ledcWrite(0, duty);
    } else {
        digitalWrite(MOTOR_DIR, LOW);
        ledcWrite(0, -duty);
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
// Main Loop (non‑blocking)
// ============================================================
void loop() {
    static unsigned long last_ultra = 0;
    if (millis() - last_ultra > 20) {
        last_ultra = millis();
        readUltrasonic();
    }
    if (micros() - last_imu_read >= IMU_READ_INTERVAL) {
        last_imu_read = micros();
        readIMU();
    }
    if (millis() - last_sensor_send > SENSOR_SEND_INTERVAL) {
        last_sensor_send = millis();
        sendSensorData();
    }
    while (Serial2.available()) {
        String line = Serial2.readStringUntil('\n');
        if (line.length() > 0) processCommand(line);
    }
    if (new_cmd) {
        applyMotorSpeed(target_speed);
        applyServoAngle(target_steer);
        new_cmd = false;
    }
    delay(1);
}
