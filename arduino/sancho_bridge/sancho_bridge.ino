#include <ArduinoBridge.h>
#include <Wire.h>

// L298N motor driver pins — TODO: confirm pin assignments on hardware
#define MOTOR_LEFT_EN   9
#define MOTOR_LEFT_IN1  8
#define MOTOR_LEFT_IN2  7
#define MOTOR_RIGHT_EN  6
#define MOTOR_RIGHT_IN1 5
#define MOTOR_RIGHT_IN2 4

// HC-SR04 pins — TODO: confirm pin assignments on hardware
#define TRIG_PIN 12
#define ECHO_PIN 11

// MPU-6050 I2C address (default, AD0 = LOW)
#define MPU6050_ADDR 0x68

// Hardware emergency stop distance in cm — TODO: tune this value
// This check runs in loop() and is completely independent of ROS 2
#define EMERGENCY_STOP_THRESHOLD_CM 15

// Sensor notification interval
#define SENSOR_INTERVAL_MS 100

unsigned long lastSensorMs = 0;

// ── Setup ─────────────────────────────────────────────────────────────────────

void setup() {
    Bridge.begin();

    pinMode(MOTOR_LEFT_EN,   OUTPUT);
    pinMode(MOTOR_LEFT_IN1,  OUTPUT);
    pinMode(MOTOR_LEFT_IN2,  OUTPUT);
    pinMode(MOTOR_RIGHT_EN,  OUTPUT);
    pinMode(MOTOR_RIGHT_IN1, OUTPUT);
    pinMode(MOTOR_RIGHT_IN2, OUTPUT);

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    Wire.begin();
    initMPU6050();

    stopMotors();  // safe default before any ROS 2 command arrives

    Bridge.expose("set_motors",    setMotors);
    Bridge.expose("emergency_stop", emergencyStop);
}

// ── Main loop ─────────────────────────────────────────────────────────────────

void loop() {
    int distance = readUltrasonic();  // returns -1 if no echo

    // Hardware emergency stop — always active, independent of ROS 2
    if (distance > 0 && distance < EMERGENCY_STOP_THRESHOLD_CM) {
        stopMotors();
    }

    if (millis() - lastSensorMs >= SENSOR_INTERVAL_MS) {
        lastSensorMs = millis();

        float gyroZ = readGyroZ();

        ArduinoBridgeMessage msg;
        msg.put("distance", (float)distance);
        msg.put("gyro_z",   gyroZ);
        Bridge.notify("sensor_data", msg);
    }

    Bridge.process();
}

// ── Bridge-exposed functions ───────────────────────────────────────────────────

void setMotors(ArduinoBridgeRequest &req) {
    int left  = req.getInt(0);
    int right = req.getInt(1);
    applyMotor(MOTOR_LEFT_EN,  MOTOR_LEFT_IN1,  MOTOR_LEFT_IN2,  left);
    applyMotor(MOTOR_RIGHT_EN, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, right);
}

void emergencyStop(ArduinoBridgeRequest &req) {
    stopMotors();
}

// ── Motor helpers ──────────────────────────────────────────────────────────────

void applyMotor(int enPin, int in1, int in2, int pwm) {
    pwm = constrain(pwm, -255, 255);
    if (pwm >= 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        pwm = -pwm;
    }
    analogWrite(enPin, pwm);
}

void stopMotors() {
    analogWrite(MOTOR_LEFT_EN,  0);
    analogWrite(MOTOR_RIGHT_EN, 0);
}

// ── Sensor helpers ─────────────────────────────────────────────────────────────

int readUltrasonic() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);  // 30 ms timeout
    if (duration == 0) return -1;
    return (int)(duration * 0.034f / 2.0f);  // microseconds → cm
}

void initMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1
    Wire.write(0x00);  // wake up (clear sleep bit)
    Wire.endTransmission(true);
}

float readGyroZ() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x47);  // GYRO_ZOUT_H register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    int16_t raw = ((int16_t)Wire.read() << 8) | Wire.read();
    return raw / 131.0f;  // ±250 deg/s full scale → deg/s
}
