// SANCHO MCU firmware — runs on the STM32U585 of the Arduino UNO Q.
//
// Responsibilities:
//   1) Receive set_motors(left, right) RPC calls from the MPU (Linux + Python)
//      over the Arduino_RouterBridge and drive four BTS7960 (IBT-2) modules
//      in 4WD skid-steer configuration (two motors per side).
//   2) Stream HC-SR04 ultrasonic distance readings to the MPU at ~20 Hz via
//      Bridge.notify("distance_cm", cm).
//
// 4WD scaling:
//   Both motors on a given side share the side's logical PWM command but use
//   per-position scale factors so all four wheels reach the same ground speed
//   despite different no-load RPM:
//     - front: JGB37-520, 333 RPM nominal → FRONT_SCALE = 0.90 (throttled)
//     - back:  JGB37-545, 300 RPM nominal → BACK_SCALE  = 1.00 (full)
//   The MPU still sends only two ints (left, right) — scaling is local to
//   the MCU, transparent to ROS and to the App Lab Python shim.
//
// Safety: three independent layers prevent runaway motion.
//   1) motor_bridge_node (Python) — software watchdog, 500 ms
//   2) MCU watchdog here          — independent firmware-level watchdog
//   3) BTS7960 with PWM = 0       — half-bridges off → motors coast
//
// Wiring (Strategy A): on each BTS7960 module, R_EN and L_EN are tied directly
// to the module's +5 V (always enabled). The MCU only drives the two PWM
// inputs per module. R_IS / L_IS (current sense) left unconnected.
//
// Pin naming convention:
//   Macros are named for the *direction* the motor moves when that pin is
//   driven, NOT for the BTS7960 input name (RPWM/LPWM). On the original 2WD
//   build the right motor was wired with reversed polarity (M+/M- swapped on
//   the BTS7960), so its FORWARD pin maps to LPWM. Verify the polarity of
//   the new back-side motors on the first bench test and flip the FWD/REV
//   mapping if a wheel turns backward.

#include "Arduino_RouterBridge.h"

// ── Front-side motors (JGB37-520, 333 RPM nominal) ───────────────────────────
#define LEFT_FRONT_FWD_PIN   10
#define LEFT_FRONT_REV_PIN    9
#define RIGHT_FRONT_FWD_PIN   6   // RIGHT motor wired reversed on original chassis
#define RIGHT_FRONT_REV_PIN   5

// ── Back-side motors (JGB37-545, 300 RPM nominal) ────────────────────────────
// D3, D11 are Uno-R3 PWM pins (always safe). D2, D4 rely on the UNO Q's STM32
// extending analogWrite() coverage beyond the Uno-R3 set; verify on first
// flash. If they don't PWM, easiest swap is A2/A3.
#define LEFT_BACK_FWD_PIN     3
#define LEFT_BACK_REV_PIN    11
#define RIGHT_BACK_FWD_PIN    2
#define RIGHT_BACK_REV_PIN    4

// ── Per-position PWM scaling (4WD) ───────────────────────────────────────────
// Front 520 nominal 333 RPM, back 545 nominal 300 RPM. Throttle the faster
// motor down so the slower one sets the pace and the side as a whole rolls
// at a single ground speed without internal fighting.
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
void applyMotor(int fwd_pin, int rev_pin, int pwm);
void stopMotors();
uint16_t readUltrasonicCm();

// ── RPC handlers (called by Bridge.provide_safe) ─────────────────────────────
void setMotors(int left, int right) {
    int left_front  = (int)(left  * FRONT_SCALE);
    int right_front = (int)(right * FRONT_SCALE);
    int left_back   = (int)(left  * BACK_SCALE);
    int right_back  = (int)(right * BACK_SCALE);

    applyMotor(LEFT_FRONT_FWD_PIN,  LEFT_FRONT_REV_PIN,  left_front);
    applyMotor(RIGHT_FRONT_FWD_PIN, RIGHT_FRONT_REV_PIN, right_front);
    applyMotor(LEFT_BACK_FWD_PIN,   LEFT_BACK_REV_PIN,   left_back);
    applyMotor(RIGHT_BACK_FWD_PIN,  RIGHT_BACK_REV_PIN,  right_back);

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

    pinMode(LEFT_FRONT_FWD_PIN,  OUTPUT);
    pinMode(LEFT_FRONT_REV_PIN,  OUTPUT);
    pinMode(LEFT_BACK_FWD_PIN,   OUTPUT);
    pinMode(LEFT_BACK_REV_PIN,   OUTPUT);
    pinMode(RIGHT_FRONT_FWD_PIN, OUTPUT);
    pinMode(RIGHT_FRONT_REV_PIN, OUTPUT);
    pinMode(RIGHT_BACK_FWD_PIN,  OUTPUT);
    pinMode(RIGHT_BACK_REV_PIN,  OUTPUT);

    pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

    stopMotors();  // safe default before the first MPU command arrives

    Bridge.provide_safe("set_motors",     setMotors);
    Bridge.provide_safe("emergency_stop", emergencyStop);

    Monitor.println("[sancho_bridge] MCU ready (4WD + ultrasonic), waiting for set_motors()");
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

// Direction-symmetric drive of one motor.
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
    analogWrite(LEFT_FRONT_FWD_PIN,  0);
    analogWrite(LEFT_FRONT_REV_PIN,  0);
    analogWrite(LEFT_BACK_FWD_PIN,   0);
    analogWrite(LEFT_BACK_REV_PIN,   0);
    analogWrite(RIGHT_FRONT_FWD_PIN, 0);
    analogWrite(RIGHT_FRONT_REV_PIN, 0);
    analogWrite(RIGHT_BACK_FWD_PIN,  0);
    analogWrite(RIGHT_BACK_REV_PIN,  0);
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
