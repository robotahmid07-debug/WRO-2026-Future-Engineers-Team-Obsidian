// ================================================================
// WRO 2026 Future Engineers - Team Obsidian
// ESP32-S3 Firmware (main.cpp)
// ================================================================
// Communicates with Raspberry Pi 5 via UART (JSON + XOR checksum).
// Reads 3 ultrasonic sensors, BNO086 IMU (I2C), and controls
// motor (PWM + direction) and steering servo.
// ================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Servo.h>

#include "pin_definitions.h"

// ---- Globals ----
Adafruit_BNO08x bno08x;
Servo steeringServo;

// Ultrasonic sensor pins
const int trigPins[3] = {TRIG1, TRIG2, TRIG3};
const int echoPins[3] = {ECHO1, ECHO2, ECHO3};

// Ultrasonic readings (cm)
float ultrasonic1 = 0.0;
float ultrasonic2 = 0.0;
float ultrasonic3 = 0.0;

// IMU yaw rate (rad/s)
float imu_yaw_rate = 0.0;

// Motor control
int motorSpeed = 0;       // -255..255 (negative = reverse)
int steeringAngle = 90;   // 0..180 (90 = centre)

// Timing
unsigned long lastIMURead = 0;
unsigned long lastUltrasonicRead = 0;
unsigned long lastSend = 0;

// ---- Utility: XOR checksum ----
uint8_t computeXOR(const String &str) {
    uint8_t checksum = 0;
    for (unsigned int i = 0; i < str.length(); i++) {
        checksum ^= (uint8_t)str[i];
    }
    return checksum;
}

// ---- Send sensor data to Pi ----
void sendSensorData() {
    // 1. Build the JSON payload (without the checksum field)
    String json = "{";
    json += "\"type\":\"sensor_data\",";
    json += "\"data\":{";
    json += "\"ultrasonic1\":" + String(ultrasonic1, 2) + ",";
    json += "\"ultrasonic2\":" + String(ultrasonic2, 2) + ",";
    json += "\"ultrasonic3\":" + String(ultrasonic3, 2) + ",";
    json += "\"imu_yaw_rate\":" + String(imu_yaw_rate, 4);
    json += "}";   // close "data"

    // 2. Compute checksum on this complete payload (no checksum field yet)
    uint8_t checksum = computeXOR(json);

    // 3. Append checksum field and close outer JSON
    json += ",\"checksum\":" + String(checksum);
    json += "}\n";   // now it's a valid complete JSON object

    // 4. Send over UART2
    Serial2.print(json);
}

// ---- Read ultrasonic sensors (cm) ----
void readUltrasonics() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(trigPins[i], LOW);
        delayMicroseconds(2);
        digitalWrite(trigPins[i], HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPins[i], LOW);
        long duration = pulseIn(echoPins[i], HIGH, 30000); // timeout 30ms
        float dist = duration * 0.034 / 2.0;
        if (dist > 0 && dist < 400) {
            switch (i) {
                case 0: ultrasonic1 = dist; break;
                case 1: ultrasonic2 = dist; break;
                case 2: ultrasonic3 = dist; break;
            }
        }
    }
}

// ---- Read IMU yaw rate ----
void readIMU() {
    // BNO08x uses SH-2 sensor hub. We read the rotational velocity.
    // This is a simplified polling; in a real implementation, you may use
    // a non‑blocking approach. For this firmware, we update yaw rate
    // every 50ms if new data is available.
    if (bno08x.wasReset()) {
        Serial.println("BNO08x reset detected");
        if (!bno08x.begin()) {
            Serial.println("BNO08x not found");
            return;
        }
        // Set up rotation vector report
        bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 50); // 20Hz
    }

    if (bno08x.getSensorEvent() && bno08x.getSensorEvent()->sensorId == SH2_GYROSCOPE_CALIBRATED) {
        imu_yaw_rate = bno08x.getSensorEvent()->gyroscope.z; // rad/s
    }
}

// ---- Handle incoming commands from Pi (motor, servo, etc.) ----
void handleCommand(const String &cmd) {
    // Commands are JSON strings: {"motor":150, "steer":45}
    // We'll parse simply (no full JSON parser for speed)
    int motorIdx = cmd.indexOf("\"motor\"");
    int steerIdx = cmd.indexOf("\"steer\"");
    if (motorIdx != -1) {
        int colon = cmd.indexOf(':', motorIdx);
        int comma = cmd.indexOf(',', colon);
        if (comma == -1) comma = cmd.length();
        String val = cmd.substring(colon + 1, comma);
        motorSpeed = val.toInt();
        motorSpeed = constrain(motorSpeed, -255, 255);
        // Update motor PWM
        if (motorSpeed >= 0) {
            digitalWrite(MOTOR_DIR1, LOW);
            digitalWrite(MOTOR_DIR2, HIGH);
            analogWrite(MOTOR_PWM1, motorSpeed);
        } else {
            digitalWrite(MOTOR_DIR1, HIGH);
            digitalWrite(MOTOR_DIR2, LOW);
            analogWrite(MOTOR_PWM1, -motorSpeed);
        }
    }
    if (steerIdx != -1) {
        int colon = cmd.indexOf(':', steerIdx);
        int comma = cmd.indexOf(',', colon);
        if (comma == -1) comma = cmd.length();
        String val = cmd.substring(colon + 1, comma);
        steeringAngle = val.toInt();
        steeringAngle = constrain(steeringAngle, 0, 180);
        steeringServo.write(steeringAngle);
    }
}

// ---- Setup ----
void setup() {
    // Serial for debugging (USB)
    Serial.begin(115200);
    // Serial2 for communication with Pi
    Serial2.begin(115200, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);

    // Ultrasonic pins
    for (int i = 0; i < 3; i++) {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
    }

    // Motor pins
    pinMode(MOTOR_PWM1, OUTPUT);
    pinMode(MOTOR_PWM2, OUTPUT);  // not used, but set
    pinMode(MOTOR_DIR1, OUTPUT);
    pinMode(MOTOR_DIR2, OUTPUT);
    analogWriteFrequency(MOTOR_PWM1, 20000); // 20kHz PWM

    // Servo
    steeringServo.attach(SERVO_PIN);
    steeringServo.write(90);

    // IMU (BNO086)
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!bno08x.begin_I2C()) {
        Serial.println("BNO08x I2C init failed");
    } else {
        bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 50);
    }

    Serial.println("ESP32-S3 ready");
}

// ---- Main loop ----
void loop() {
    unsigned long now = millis();

    // Read ultrasonics every 30ms (~33Hz)
    if (now - lastUltrasonicRead > 30) {
        readUltrasonics();
        lastUltrasonicRead = now;
    }

    // Read IMU every 50ms (20Hz)
    if (now - lastIMURead > 50) {
        readIMU();
        lastIMURead = now;
    }

    // Send sensor data to Pi every 50ms (20Hz)
    if (now - lastSend > 50) {
        sendSensorData();
        lastSend = now;
    }

    // Check for incoming commands from Pi
    while (Serial2.available()) {
        String line = Serial2.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            handleCommand(line);
        }
    }

    // Debug output (optional) – uncomment for debugging
    // static unsigned long lastDebug = 0;
    // if (now - lastDebug > 1000) {
    //     Serial.printf("U1:%.1f U2:%.1f U3:%.1f Yaw:%.2f\n",
    //                   ultrasonic1, ultrasonic2, ultrasonic3, imu_yaw_rate);
    //     lastDebug = now;
    // }
}
