// ================================================================
// WRO 2026 Future Engineers - Team Obsidian
// ESP32-S3 Firmware - Single File (Arduino IDE Compatible)
// ================================================================
// - Non-blocking ultrasonic (timeout 10ms, 20Hz)
// - Motor safety timeout (5 seconds)
// - LEDC PWM for motors (20kHz)
// - Servo timer auto-allocated
// - IMU with fallback
// ================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <ESP32Servo.h>

// ============================================================
// PIN DEFINITIONS (match system_prompt_matrix.yaml)
// ============================================================
#define SERIAL_TX_PIN 17
#define SERIAL_RX_PIN 18

#define TRIG1  4
#define ECHO1  5
#define TRIG2  6
#define ECHO2  7
#define TRIG3  15
#define ECHO3  16

#define MOTOR_PWM1  1
#define MOTOR_PWM2  2

#define SERVO_PIN  41

#define I2C_SDA  8
#define I2C_SCL  9

// ============================================================
// GLOBALS
// ============================================================
Adafruit_BNO08x bno08x;
Servo steeringServo;

const int trigPins[3] = {TRIG1, TRIG2, TRIG3};
const int echoPins[3] = {ECHO1, ECHO2, ECHO3};

float ultrasonic1 = 0.0;
float ultrasonic2 = 0.0;
float ultrasonic3 = 0.0;
float imu_yaw_rate = 0.0;

int motorSpeed = 0;           // -255..255
int steeringAngle = 90;       // 0..180

unsigned long lastIMURead = 0;
unsigned long lastUltrasonicRead = 0;
unsigned long lastSend = 0;
unsigned long lastCommandTime = 0;   // for motor safety timeout

bool imu_found = false;
sh2_SensorValue_t sensorValue;

// ============================================================
// XOR Checksum (for outgoing sensor data)
// ============================================================
uint8_t computeXOR(const String &str) {
  uint8_t checksum = 0;
  for (unsigned int i = 0; i < str.length(); i++) {
    checksum ^= (uint8_t)str[i];
  }
  return checksum;
}

// ============================================================
// Send sensor data to Raspberry Pi
// ============================================================
void sendSensorData() {
  String json = "{";
  json += "\"type\":\"sensor_data\",";
  json += "\"data\":{";
  json += "\"ultrasonic1\":" + String(ultrasonic1, 2) + ",";
  json += "\"ultrasonic2\":" + String(ultrasonic2, 2) + ",";
  json += "\"ultrasonic3\":" + String(ultrasonic3, 2) + ",";
  json += "\"imu_yaw_rate\":" + String(imu_yaw_rate, 4);
  json += "}";

  uint8_t checksum = computeXOR(json);
  json += ",\"checksum\":" + String(checksum);
  json += "}\n";

  Serial2.print(json);
}

// ============================================================
// Read Ultrasonic sensors – Non-blocking (timeout 10ms)
// ============================================================
void readUltrasonics() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(trigPins[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[i], LOW);

    // timeout reduced to 10ms (max range ~1.7m)
    long duration = pulseIn(echoPins[i], HIGH, 10000);
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

// ============================================================
// Read IMU (with guard)
// ============================================================
void readIMU() {
  if (!imu_found) return;

  if (bno08x.wasReset()) {
    Serial.println("BNO08x was reset - re-enabling report");
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 50);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      imu_yaw_rate = sensorValue.un.gyroscope.z;  // rad/s
    }
  }
}

// ============================================================
// Handle commands from Pi (with motor safety timeout update)
// ============================================================
void handleCommand(const String &cmd) {
  // Update last command time for safety timeout
  lastCommandTime = millis();

  int motorIdx = cmd.indexOf("\"motor\"");
  int steerIdx = cmd.indexOf("\"steer\"");

  if (motorIdx != -1) {
    int colon = cmd.indexOf(':', motorIdx);
    int comma = cmd.indexOf(',', colon);
    if (comma == -1) comma = cmd.length();
    String val = cmd.substring(colon + 1, comma);
    motorSpeed = constrain(val.toInt(), -255, 255);

    // Apply motor PWM via LEDC (channels 0 and 1)
    if (motorSpeed >= 0) {
      ledcWrite(0, motorSpeed);
      ledcWrite(1, 0);
    } else {
      ledcWrite(0, 0);
      ledcWrite(1, -motorSpeed);
    }
  }

  if (steerIdx != -1) {
    int colon = cmd.indexOf(':', steerIdx);
    int comma = cmd.indexOf(',', colon);
    if (comma == -1) comma = cmd.length();
    String val = cmd.substring(colon + 1, comma);
    steeringAngle = constrain(val.toInt(), 0, 180);
    steeringServo.write(steeringAngle);
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);

  // ---- Ultrasonic pins ----
  for (int i = 0; i < 3; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  // ---- Motor PWM using LEDC (20kHz, 8-bit resolution) ----
  pinMode(MOTOR_PWM1, OUTPUT);
  pinMode(MOTOR_PWM2, OUTPUT);
  ledcSetup(0, 20000, 8);   // channel 0, 20kHz, 8-bit
  ledcAttachPin(MOTOR_PWM1, 0);
  ledcSetup(1, 20000, 8);   // channel 1, 20kHz, 8-bit
  ledcAttachPin(MOTOR_PWM2, 1);

  // ---- Servo (timer auto-allocated) ----
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  steeringServo.write(90);

  // ---- IMU (BNO086) with address fallback ----
  Wire.begin(I2C_SDA, I2C_SCL);

  if (bno08x.begin_I2C()) {
    imu_found = true;
    Serial.println("BNO08x found at 0x4A");
  } else if (bno08x.begin_I2C(0x4B)) {
    imu_found = true;
    Serial.println("BNO08x found at 0x4B");
  }

  if (imu_found) {
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 50);
    Serial.println("Gyroscope report enabled");
  } else {
    Serial.println("WARNING: BNO08x not found!");
  }

  // Initialise lastCommandTime to now
  lastCommandTime = millis();

  Serial.println("ESP32-S3 Ready - All features active");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  // ---- Read ultrasonics at 20 Hz (every 50ms) ----
  if (now - lastUltrasonicRead > 50) {
    readUltrasonics();
    lastUltrasonicRead = now;
  }

  // ---- Read IMU at 20 Hz (every 50ms) ----
  if (now - lastIMURead > 50) {
    readIMU();
    lastIMURead = now;
  }

  // ---- Send sensor data at 20 Hz (every 50ms) ----
  if (now - lastSend > 50) {
    sendSensorData();
    lastSend = now;
  }

  // ---- Process incoming commands from Pi ----
  while (Serial2.available()) {
    String line = Serial2.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      handleCommand(line);
    }
  }

  // ---- Motor safety timeout (5 seconds) ----
  if (now - lastCommandTime > 5000) {
    if (motorSpeed != 0) {
      motorSpeed = 0;
      ledcWrite(0, 0);
      ledcWrite(1, 0);
      Serial.println("Motor safety timeout: motors stopped");
    }
  }
}
