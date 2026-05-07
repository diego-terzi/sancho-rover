// SANCHO MCU firmware — runs on the STM32U585 of the Arduino UNO Q.
//
// Responsibilities:
//   1) Receive set_motors(left, right) RPC calls from the MPU (Linux + Python)
//      over the Arduino_RouterBridge and drive four BTS7960 (IBT-2) modules
//      in 4WD skid-steer configuration (two motors per side, forward-only).
//   2) Stream HC-SR04 ultrasonic distance readings to the MPU at ~20 Hz via
//      Bridge.notify("distance_cm", cm).
//
// IMPORTANT — UNO Q PWM gotcha:
//   On the UNO Q's Zephyr-based core, calling pinMode(pin, OUTPUT) BEFORE
//   analogWrite(pin, val) breaks PWM on that pin (it gets stuck as plain
//   digital). analogWrite() configures the pin internally. So: NO pinMode()
//   on any pin used with analogWrite(). We still call pinMode() on TRIG/ECHO
//   of the HC-SR04 because those use digitalWrite/digitalRead, not PWM.
//
// Forward-only drive:
//   The BTS7960 modules are wired with only their RPWM input connected;
//   LPWM is left floating, so the rover only moves forward at the hardware
//   level. Negative PWM coming from motor_bridge_node is clamped to 0
//   (motor stops). This is fine for SANCHO's mission: FOLLOWING /
//   TRAIL_LOST / OBSTACLE_STOP never need reverse. A very tight pivot that
//   would request reverse on the inner wheel just turns wider instead.
//
// 4WD scaling:
//   Both motors of one side share the side's logical PWM, with per-position
//   scale factors so all four wheels reach the same ground speed despite
//   different no-load RPM:
//     - front: JGB37-520, 333 RPM nominal → FRONT_SCALE = 0.90 (throttled)
//     - back:  JGB37-545, 300 RPM nominal → BACK_SCALE  = 1.00 (full)
//   Scaling is local to the MCU; the MPU still sends two ints (left, right).
//
// Safety: three independent layers prevent runaway motion.
//   1) motor_bridge_node (Python) — software watchdog, 500 ms
//   2) MCU watchdog here          — independent firmware-level watchdog
//   3) BTS7960 with PWM = 0       — half-bridges off → motors coast

#include "Arduino_RouterBridge.h"

// ── Forward-only PWM pins (BTS7960 RPWM input, one per motor) ────────────────
#define LEFT_FRONT_RPWM_PIN    9
#define RIGHT_FRONT_RPWM_PIN   6
#define LEFT_BACK_RPWM_PIN    10
#define RIGHT_BACK_RPWM_PIN    3

// ── Per-position PWM scaling (4WD) ───────────────────────────────────────────
#define FRONT_SCALE  0.90f    // ≈ 300 / 333
#define BACK_SCALE   1.00f

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
void applyMotor(int rpwm_pin, int pwm);
void stopMotors();
uint16_t readUltrasonicCm();

// ── RPC handlers (called by Bridge.provide_safe) ─────────────────────────────
void setMotors(int left, int right) {
    int left_front  = (int)(left  * FRONT_SCALE);
    int right_front = (int)(right * FRONT_SCALE);
    int left_back   = (int)(left  * BACK_SCALE);
    int right_back  = (int)(right * BACK_SCALE);

    applyMotor(LEFT_FRONT_RPWM_PIN,  left_front);
    applyMotor(RIGHT_FRONT_RPWM_PIN, right_front);
    applyMotor(LEFT_BACK_RPWM_PIN,   left_back);
    applyMotor(RIGHT_BACK_RPWM_PIN,  right_back);

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

    // No pinMode() on motor RPWM pins — analogWrite() configures them itself
    // and any prior pinMode() would break PWM (Zephyr core gotcha).

    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

    stopMotors();  // safe default before the first MPU command arrives

    Bridge.provide_safe("set_motors",     setMotors);
    Bridge.provide_safe("emergency_stop", emergencyStop);

    Monitor.println("[sancho_bridge] MCU ready (4WD forward-only + ultrasonic)");
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

// Forward-only drive of one motor. Negative PWM clamps to 0 (motor coasts).
// PWM > 255 clamps to 255. The hardware has no LPWM connected, so reverse
// is impossible.
void applyMotor(int rpwm_pin, int pwm) {
    if (pwm < 0)   pwm = 0;
    if (pwm > 255) pwm = 255;
    analogWrite(rpwm_pin, pwm);
}

void stopMotors() {
    analogWrite(LEFT_FRONT_RPWM_PIN,  0);
    analogWrite(RIGHT_FRONT_RPWM_PIN, 0);
    analogWrite(LEFT_BACK_RPWM_PIN,   0);
    analogWrite(RIGHT_BACK_RPWM_PIN,  0);
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
