// SANCHO MCU firmware — 4WD motor control + end-of-mission signal.
//
// Motors: 4 BTS7960 modules driven on their RPWM input only (forward-only).
//   Per-side scaling so the 333 RPM front motors don't outrun the 300 RPM
//   rear ones (FRONT_SCALE = 300/333 ≈ 0.901, BACK_SCALE = 1.00).
//
// End-of-mission signal (LED + buzzer):
//   When the rover has moved at least once and then stops (no non-zero motor
//   command for CELEB_STOP_DEBOUNCE_MS), it plays a short riff one time.
//   If the rover starts moving again, the riff arms again for the next stop.
//   The trigger logic lives entirely on the MCU — no extra Bridge traffic
//   needed; we infer "mission complete" from the motor commands.
//
// IMPORTANT — UNO Q PWM gotcha:
//   On Zephyr, pinMode(pin, OUTPUT) BEFORE the first analogWrite(pin, val)
//   leaves the PWM peripheral inert. We do NOT pinMode the motor pins;
//   we force-init each with a brief PWM=1 pulse instead.
//   pinMode on the LED pin (12) is fine — it's a plain digital out.
//   tone() handles its own pin config, so no pinMode on the buzzer either.

#include "Arduino_RouterBridge.h"

// ── Motor pins (RPWM only, forward-only) ─────────────────────────────────────
#define LEFT_FRONT_RPWM_PIN    9
#define RIGHT_FRONT_RPWM_PIN   6
#define LEFT_BACK_RPWM_PIN    10
#define RIGHT_BACK_RPWM_PIN    3

#define FRONT_SCALE  (300.0f / 333.0f)
#define BACK_SCALE   1.00f

// ── End-of-mission signal pins ──────────────────────────────────────────────
#define LED_PIN       12
#define BUZZER_PIN     5

// ── Timing ──────────────────────────────────────────────────────────────────
#define MOTOR_WATCHDOG_MS         500UL
#define CELEB_STOP_DEBOUNCE_MS   2000UL  // motors stopped this long → trigger riff

// ── Celebration riff ────────────────────────────────────────────────────────
const int REST     = 0;
const int C_SHARP  = 277;  // C#4
const int B_NOTE   = 247;  // B3
const int E_NOTE   = 330;  // E4

struct RiffStep { int freq; int duration_ms; };
const RiffStep riff[] = {
    {C_SHARP, 150}, {REST,  50},
    {B_NOTE,  150}, {REST,  50},
    {C_SHARP, 200}, {REST, 300},
    {E_NOTE,  400}, {REST, 100},
    {C_SHARP, 250}, {REST,  50},
    {C_SHARP, 200}, {REST, 100},
};
const int RIFF_STEPS = sizeof(riff) / sizeof(RiffStep);

// ── State ───────────────────────────────────────────────────────────────────
unsigned long lastSetMotorsMs    = 0;
unsigned long lastSetMotorsLogMs = 0;
uint32_t      setMotorsCallCount = 0;

unsigned long lastNonZeroMotorsMs = 0;
bool          everMoved            = false;
bool          armed                = false;  // ready to trigger on next stop
bool          celebrating          = false;
int           celebStepIdx         = 0;
unsigned long celebStepStartMs     = 0;

void applyMotor(int rpwm_pin, int pwm);
void stopMotors();
void updateCelebration();

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

    // Mission-state tracking. Any non-zero command counts as "moving".
    if (left > 0 || right > 0) {
        lastNonZeroMotorsMs = millis();
        everMoved = true;
        armed = true;
        // If we were celebrating and the rover starts moving again, abort.
        if (celebrating) {
            celebrating = false;
            noTone(BUZZER_PIN);
            digitalWrite(LED_PIN, LOW);
        }
    }

    // Diagnostic heartbeat — once every 5 s
    setMotorsCallCount++;
    if (millis() - lastSetMotorsLogMs > 5000UL) {
        lastSetMotorsLogMs = millis();
        Monitor.print("[setMotors #");
        Monitor.print(setMotorsCallCount);
        Monitor.print("] L=");
        Monitor.print(left);
        Monitor.print(" R=");
        Monitor.println(right);
    }
}

void emergencyStop() {
    stopMotors();
    lastSetMotorsMs = 0;
}

// ── Setup ───────────────────────────────────────────────────────────────────
void setup() {
    Bridge.begin();
    Monitor.begin();

    // Force PWM peripheral init: brief PWM=1 pulse then 0. Without this, the
    // first analogWrite(0) on a fresh pin can leave the peripheral inert.
    analogWrite(LEFT_FRONT_RPWM_PIN,  1);
    analogWrite(RIGHT_FRONT_RPWM_PIN, 1);
    analogWrite(LEFT_BACK_RPWM_PIN,   1);
    analogWrite(RIGHT_BACK_RPWM_PIN,  1);
    delay(50);
    stopMotors();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);

    Bridge.provide_safe("set_motors",     setMotors);
    Bridge.provide_safe("emergency_stop", emergencyStop);

    Monitor.println("[sancho_bridge] MCU ready (4WD + end-of-mission signal)");
}

// ── Main loop ───────────────────────────────────────────────────────────────
void loop() {
    if (millis() - lastSetMotorsMs > MOTOR_WATCHDOG_MS) {
        stopMotors();
    }
    updateCelebration();
}

// ── Motor helpers ───────────────────────────────────────────────────────────
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

// Apply a step's buzzer output — called exactly once per step transition so
// tone() is not restarted on every loop iteration (that would click/distort).
static void applyRiffStep(int idx) {
    const RiffStep& step = riff[idx];
    if (step.freq == REST) {
        noTone(BUZZER_PIN);
    } else {
        tone(BUZZER_PIN, step.freq);
    }
}

// ── End-of-mission celebration (LED + buzzer) ───────────────────────────────
void updateCelebration() {
    // Arm-and-fire: rover must have moved this cycle (armed=true), and
    // the motors must have been zero for CELEB_STOP_DEBOUNCE_MS.
    if (!celebrating && armed && everMoved &&
        (millis() - lastNonZeroMotorsMs > CELEB_STOP_DEBOUNCE_MS)) {
        celebrating = true;
        armed = false;                 // disarm until next motion
        celebStepIdx = 0;
        celebStepStartMs = millis();
        digitalWrite(LED_PIN, HIGH);   // solid on for the whole riff
        applyRiffStep(0);              // start the first note immediately
        Monitor.println("[celebration] mission complete — playing riff");
        return;
    }

    if (!celebrating) return;

    // Has the current step's duration elapsed?
    if (millis() - celebStepStartMs < (unsigned long)riff[celebStepIdx].duration_ms) {
        return;  // still inside the current step, nothing to do
    }

    // Advance to next step
    celebStepIdx++;
    celebStepStartMs = millis();

    if (celebStepIdx >= RIFF_STEPS) {
        // Riff finished — loop it from the start. LED stays on the whole
        // time; the riff keeps replaying until the rover starts moving again
        // (setMotors detects non-zero PWM and aborts celebration).
        celebStepIdx = 0;
    }

    applyRiffStep(celebStepIdx);
}
