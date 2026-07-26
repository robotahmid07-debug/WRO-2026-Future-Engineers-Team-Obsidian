/**
 * ESP32-S3 Firmware for WRO Future Engineers 2026
 * 
 * Hardware:
 *   - 1 DC drive motor (with encoder) + 1 steering servo
 *   - 3 ultrasonic sensors (front, front-left, front-right)
 *   - UART communication with Raspberry Pi (JSON)
 * 
 * Communication Protocol:
 *   - Received JSON: {"type":"cmd","speed":0.3,"steering":0.15}
 *   - Sent JSON: {"type":"ultrasonic","data":{"front":12.5,...},"checksum":123}
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <ESP32Encoder.h>
#include <Servo.h>
#include "pin_definitions.h"

// ============================================================
// Constants & Configuration
// ============================================================

#define UART_BAUD 460800
#define PWM_FREQ 20000
#define PWM_RES 10            // 0-1023
#define MAX_DUTY 1023
#define MAX_SPEED_MPS 0.5     // m/s, for mapping speed to duty
#define ULTRASONIC_TIMEOUT 30000  // µs

// Steering servo limits (mechanical)
#define SERVO_CENTER 90       // degrees (neutral)
#define SERVO_MAX_ANGLE 90    // ±90° from center (adjust if needed)
#define MAX_STEER_RAD 0.524   // ±30° in radians

// ============================================================
// Objects & Variables
// ============================================================

Servo steeringServo;
ESP32Encoder encoder;

// Ultrasonic sensor distances (cm)
volatile float front_dist = 999.0;
volatile float front_left_dist = 999.0;
volatile float front_right_dist = 999.0;

// Command variables (updated by UART)
volatile float target_speed = 0.0;       // m/s, positive = forward
volatile float target_steer = 0.0;       // radians, positive = left
volatile bool new_cmd = false;

// Timing
unsigned long last_sensor_send = 0;
const unsigned long SENSOR_SEND_INTERVAL = 50;  // ms

// ============================================================
// Setup
// ============================================================

void setup() {
    // Debug serial (USB)
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("ESP32-S3 WRO Robot Firmware v2.0 (Single Motor + Servo)");

    // UART2 to Raspberry Pi
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
    Serial2.setTimeout(10);

    // Ultrasonic pins
    pinMode(FRONT_TRIG, OUTPUT);
    pinMode(FRONT_ECHO, INPUT);
    pinMode(FRONT_LEFT_TRIG, OUTPUT);
    pinMode(FRONT_LEFT_ECHO, INPUT);
    pinMode(FRONT_RIGHT_TRIG, OUTPUT);
    pinMode(FRONT_RIGHT_ECHO, INPUT);

    // Motor PWM and Direction
    ledcSetup(0, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_PWM, 0);
    pinMode(MOTOR_DIR, OUTPUT);

    // Servo
    steeringServo.attach(SERVO_PWM);
    steeringServo.write(SERVO_CENTER);
    delay(200);

    // Encoder (only one, on the drive motor or axle)
    ESP32Encoder::useInternalWeakPullResistors = UP;
    encoder.attachHalfQuad(ENC_A_A, ENC_A_B);
    encoder.clearCount();

    Serial.println("Setup complete.");
}

// ============================================================
// Main Loop
// ============================================================

void loop() {
    // Read ultrasonics every 20 ms
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

    // Process incoming commands from Pi
    while (Serial2.available()) {
        String line = Serial2.readStringUntil('\n');
        if (line.length() > 0) {
            processCommand(line);
        }
    }

    // Apply motor and servo if new command received
    if (new_cmd) {
        applyMotorSpeed(target_speed);
        applyServoAngle(target_steer);
        new_cmd = false;
    }

    delay(1);  // yield to other tasks
}

// ============================================================
// Ultrasonic Reading
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
// Send Sensor Data (JSON)
// ============================================================

void sendSensorData() {
    int64_t enc = encoder.getCount();

    // Build JSON without checksum first
    String msg = "{";
    msg += "\"type\":\"ultrasonic\",";
    msg += "\"data\":{";
    msg += "\"front\":" + String(front_dist) + ",";
    msg += "\"front_left\":" + String(front_left_dist) + ",";
    msg += "\"front_right\":" + String(front_right_dist) + ",";
    msg += "\"enc\":" + String(enc);
    msg += "}";

    // Compute simple checksum (sum of char codes)
    int sum = 0;
    for (int i = 0; i < msg.length(); i++) sum += msg[i];
    msg += ",\"checksum\":" + String(sum);
    msg += "}\n";

    Serial2.print(msg);
}

// ============================================================
// Command Processing (JSON)
// ============================================================

void processCommand(String json) {
    // Expected format: {"type":"cmd","speed":0.30,"steering":0.15}
    // speed: m/s (negative = reverse)
    // steering: radians (positive = left turn)

    float speed = 0.0, steer = 0.0;
    int idxSpeed = json.indexOf("\"speed\":");
    int idxSteer = json.indexOf("\"steering\":");
    if (idxSpeed != -1) {
        sscanf(json.substring(idxSpeed + 8).c_str(), "%f", &speed);
    }
    if (idxSteer != -1) {
        sscanf(json.substring(idxSteer + 11).c_str(), "%f", &steer);
    }

    target_speed = speed;
    target_steer = steer;
    new_cmd = true;
}

// ============================================================
// Motor Control
// ============================================================

void applyMotorSpeed(float speed_mps) {
    int duty = mapSpeedToDuty(speed_mps, MAX_SPEED_MPS);
    if (duty >= 0) {
        digitalWrite(MOTOR_DIR, HIGH);
        ledcWrite(0, duty);
    } else {
        digitalWrite(MOTOR_DIR, LOW);
        ledcWrite(0, -duty);
    }
}

int mapSpeedToDuty(float speed, float max_speed) {
    // Clamp speed to ±max_speed
    if (speed > max_speed) speed = max_speed;
    if (speed < -max_speed) speed = -max_speed;
    return (int)((speed / max_speed) * MAX_DUTY);
}

// ============================================================
// Servo Steering Control
// ============================================================

void applyServoAngle(float angle_rad) {
    // Clamp steering angle to mechanical limits
    float clamped = constrain(angle_rad, -MAX_STEER_RAD, MAX_STEER_RAD);
    // Map radians to servo pulse (0-180)
    // angle_rad = 0 -> 90°, positive -> left, negative -> right
    int angle_deg = SERVO_CENTER + (int)((clamped / MAX_STEER_RAD) * 90.0);
    angle_deg = constrain(angle_deg, 0, 180);
    steeringServo.write(angle_deg);
}
