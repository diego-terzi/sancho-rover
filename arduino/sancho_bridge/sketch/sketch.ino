// SANCHO MCU firmware — runs on the STM32U585 of the Arduino UNO Q.
//
// Responsibility:
//   Receive set_motors(left, right) RPC calls from the MPU (Linux + Python)
//   over the Arduino_RouterBridge (arduino-router service on /dev/ttyHS1),
//   and drive two BTS7960 (IBT-2) motor-driver modules accordingly.
//
// Safety: three independent layers prevent runaway motion.
//   1) motor_bridge_node (Python)  — software watchdog, ~500 ms
//   2) MCU watchdog here           — independent watchdog at the firmware level
//   3) BTS7960 with PWM = 0        — half-bridges off → motors coast
//
// Wiring (Strategy A): on each BTS7960 module, R_EN and L_EN are tied directly
// to the module's +5 V (always enabled). The MCU only drives the two PWM
// inputs per module. R_IS / L_IS (current sense) left unconnected.
//
// Pin assignments — naming convention:
//   The macros are named for the *direction* the motor moves when that pin
//   is driven, NOT for the BTS7960 input name (RPWM/LPWM). On this rover the
//   right motor is wired with reversed polarity (M+/M- swapped on the
//   BTS7960), so its "forward" pin happens to be the LPWM input of the right
//   module. The macros hide that detail from the rest of the firmware.
//
//   Pin numbers verified with the bench-test sketch (3-second forward / pivot
//   cycle) on the actual rover.

#include "Arduino_RouterBridge.h"

// ── Direction-named pin assignments ──────────────────────────────────────────
#define LEFT_FWD_PIN   10   // LEFT  motor → BTS7960 RPWM input
#define LEFT_REV_PIN    9   // LEFT  motor → BTS7960 LPWM input
#define RIGHT_FWD_PIN   6   // RIGHT motor → BTS7960 LPWM input  (motor wired reversed)
#define RIGHT_REV_PIN   5   // RIGHT motor → BTS7960 RPWM input  (motor wired reversed)

// ── HC-SR04 ultrasonic (front) ───────────────────────────────────────────────
#define ULTRASONIC_TRIG_PIN     7
#define ULTRASONIC_ECHO_PIN     8
#define ULTRASONIC_PERIOD_MS    50UL    // ~20 Hz sampling
#define ULTRASONIC_TIMEOUT_US   25000UL // ~4 m max round-trip; pulseIn returns 0 on timeout

// ── MCU-side watchdog ─────────────────────────────────────────────────────────
#define MOTOR_WATCHDOG_MS  500UL

unsigned long lastSetMotorsMs    = 0;
unsigned long lastUltrasonicMs   = 0;

// Forward declarations
void applyMotor(int fwd_pin, int rev_pin, int pwm);
void stopMotors();
uint16_t readUltrasonicCm();

// ── RPC handlers (called by Bridge.provide_safe) ─────────────────────────────
void setMotors(int left, int right) {
    applyMotor(LEFT_FWD_PIN,  LEFT_REV_PIN,  left);
    applyMotor(RIGHT_FWD_PIN, RIGHT_REV_PIN, right);
    lastSetMotorsMs = millis();
}

void emergencyStop() {
    stopMotors();
    lastSetMotorsMs = 0;
}

// ── Setup ─────────────────────────────────────────────────────────────────────

void setup() {
    Bridge.begin();
    Monitor.begin();

    pinMode(LEFT_FWD_PIN,  OUTPUT);
    pinMode(LEFT_REV_PIN,  OUTPUT);
    pinMode(RIGHT_FWD_PIN, OUTPUT);
    pinMode(RIGHT_REV_PIN, OUTPUT);

    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

    stopMotors();  // safe default before the first MPU command arrives

    Bridge.provide_safe("set_motors",     setMotors);
    Bridge.provide_safe("emergency_stop", emergencyStop);

    Monitor.println("[sancho_bridge] MCU ready, waiting for set_motors()");
}

// ── Main loop ─────────────────────────────────────────────────────────────────

void loop() {
    // MCU-side watchdog. millis() wraps every ~49 days; the subtraction is
    // wrap-safe because both operands are unsigned long.
    if (millis() - lastSetMotorsMs > MOTOR_WATCHDOG_MS) {
        stopMotors();
    }

    // Ultrasonic sampling, ~20 Hz. pulseIn() can block up to ULTRASONIC_TIMEOUT_US
    // (~25 ms) on no-echo, but the Bridge serial RX is interrupt-driven so this
    // does not delay incoming RPC calls. The motor watchdog tolerance is 500 ms,
    // ten times longer, so a few stalled cycles are still safe.
    if (millis() - lastUltrasonicMs > ULTRASONIC_PERIOD_MS) {
        lastUltrasonicMs = millis();
        uint16_t cm = readUltrasonicCm();
        Bridge.notify("distance_cm", cm);
    }
}

// ── Motor helpers ─────────────────────────────────────────────────────────────

// Direction-symmetric drive.
//   FORWARD (pwm > 0): fwd_pin = |pwm|, rev_pin = 0
//   REVERSE (pwm < 0): fwd_pin = 0,     rev_pin = |pwm|
//   STOP    (pwm = 0): both pins 0  (coast — half-bridges off, no braking)
void applyMotor(int fwd_pin, int rev_pin, int pwm) {
    pwm = constrain(pwm, -255, 255);
    if (pwm >= 0) {
        analogWrite(fwd_pin, pwm);
        analogWrite(rev_pin, 0);
    } else {
        analogWrite(fwd_pin, 0);
        analogWrite(rev_pin, -pwm);
    }
}

void stopMotors() {
    analogWrite(LEFT_FWD_PIN,  0);
    analogWrite(LEFT_REV_PIN,  0);
    analogWrite(RIGHT_FWD_PIN, 0);
    analogWrite(RIGHT_REV_PIN, 0);
}

// ── Ultrasonic helper ─────────────────────────────────────────────────────────

// Returns the front distance in centimetres, or 0 if no echo arrived within
// ULTRASONIC_TIMEOUT_US. The 0-as-out-of-range convention is interpreted by
// sensor_node on the ROS side as "free space ahead" (mapped to max_range).
uint16_t readUltrasonicCm() {
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

    unsigned long us = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
    if (us == 0) return 0;
    // Speed of sound ≈ 343 m/s → 58 µs per cm round-trip.
    return (uint16_t)(us / 58UL);
}
