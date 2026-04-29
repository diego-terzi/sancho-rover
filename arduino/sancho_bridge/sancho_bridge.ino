// SANCHO MCU firmware — runs on the STM32U585 of the Arduino UNO Q.
//
// Responsibility:
//   Receive set_motors(left, right) RPC calls from the MPU (Linux + ROS 2)
//   over the Arduino Bridge UART, and drive two BTS7960 (IBT-2) motor-driver
//   modules accordingly.
//
// Safety: three independent layers prevent runaway motion.
//   1) motor_bridge_node (Python)  — software watchdog, ~500 ms
//   2) MCU watchdog here           — independent watchdog at the firmware level
//   3) BTS7960 with PWM = 0        — half-bridges off → motors coast
//
// Wiring strategy (Strategy A, simplest):
//   On each BTS7960 module, R_EN and L_EN are tied directly to the module's
//   +5 V supply (always enabled). The MCU only drives the two PWM inputs per
//   module. R_IS and L_IS (current sense) are left unconnected for now.
//   Total MCU pins used: 4 (all PWM-capable).
//
// Sensors: not yet integrated in this revision. HC-SR04 ultrasonic and
// MPU-6050 IMU will be added in a later pass.

#include <ArduinoBridge.h>

// ── BTS7960 control pins ──────────────────────────────────────────────────────
// All four must be PWM-capable. On Arduino UNO Q, pins 5/6/9/10 are PWM.
// 11/12 are intentionally left free for the upcoming HC-SR04, and A4/A5 for I²C.
#define LEFT_RPWM    5      // PWM forward, left  motor
#define LEFT_LPWM    6      // PWM reverse, left  motor
#define RIGHT_RPWM   9      // PWM forward, right motor
#define RIGHT_LPWM  10      // PWM reverse, right motor

// ── MCU-side watchdog ─────────────────────────────────────────────────────────
// If the MPU stops calling set_motors() for this long, the MCU autonomously
// zeroes the PWMs. This is the second of the three safety layers and protects
// against a frozen / crashed Python side that the MPU-side watchdog could miss.
#define MOTOR_WATCHDOG_MS  500UL

unsigned long lastSetMotorsMs = 0;

// ── Setup ─────────────────────────────────────────────────────────────────────

void setup() {
    Bridge.begin();

    pinMode(LEFT_RPWM,  OUTPUT);
    pinMode(LEFT_LPWM,  OUTPUT);
    pinMode(RIGHT_RPWM, OUTPUT);
    pinMode(RIGHT_LPWM, OUTPUT);

    stopMotors();  // safe default before the first MPU command arrives

    Bridge.expose("set_motors",     setMotors);
    Bridge.expose("emergency_stop", emergencyStop);
}

// ── Main loop ─────────────────────────────────────────────────────────────────

void loop() {
    Bridge.process();

    // MCU-side watchdog. millis() wraps every ~49 days; the subtraction is
    // wrap-safe because both operands are unsigned long.
    if (millis() - lastSetMotorsMs > MOTOR_WATCHDOG_MS) {
        stopMotors();
    }
}

// ── Bridge-exposed functions ───────────────────────────────────────────────────

void setMotors(ArduinoBridgeRequest &req) {
    int left  = req.getInt(0);
    int right = req.getInt(1);
    applyMotor(LEFT_RPWM,  LEFT_LPWM,  left);
    applyMotor(RIGHT_RPWM, RIGHT_LPWM, right);
    lastSetMotorsMs = millis();
}

void emergencyStop(ArduinoBridgeRequest &req) {
    stopMotors();
    // Reset the watchdog timestamp so the loop continues to enforce stop
    // until a fresh set_motors() call explicitly resumes operation.
    lastSetMotorsMs = 0;
}

// ── Motor helpers ─────────────────────────────────────────────────────────────

// BTS7960 dual-PWM control. Convention:
//   FORWARD (pwm > 0) : RPWM = |pwm|, LPWM = 0
//   REVERSE (pwm < 0) : RPWM = 0,     LPWM = |pwm|
//   STOP    (pwm = 0) : both pins 0  (coast — half-bridges off, no braking)
void applyMotor(int rpwm_pin, int lpwm_pin, int pwm) {
    pwm = constrain(pwm, -255, 255);
    if (pwm >= 0) {
        analogWrite(rpwm_pin, pwm);
        analogWrite(lpwm_pin, 0);
    } else {
        analogWrite(rpwm_pin, 0);
        analogWrite(lpwm_pin, -pwm);
    }
}

void stopMotors() {
    analogWrite(LEFT_RPWM,  0);
    analogWrite(LEFT_LPWM,  0);
    analogWrite(RIGHT_RPWM, 0);
    analogWrite(RIGHT_LPWM, 0);
}
