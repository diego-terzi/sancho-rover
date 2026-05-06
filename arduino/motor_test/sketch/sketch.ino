// SANCHO — standalone bench-test sketch for the 4WD motor wiring.
//
// Purpose: drive each motor individually, then in combinations, to verify
// the BTS7960 wiring and the chosen pin assignments BEFORE integrating with
// the main sancho_bridge firmware. No Bridge, no UDP, no sensors — only
// motors. If something is wrong here, you isolate the bug to wiring or pins.
//
// SAFETY: run with the wheels OFF the ground (rover on blocks). The phases
// include direction reversals and a pivot-in-place — they would jolt a rover
// sitting on a table. Make sure all 4 BTS7960 modules are powered by the
// motor battery (12V) AND share GND with the UNO Q.
//
// What you should see, phase by phase:
//   Phase 1: each wheel spins individually for 2 s forward, 2 s reverse.
//            ✓ identifies which physical motor is connected to which pin.
//            ✗ if a motor doesn't spin: bad wiring, dead BTS7960, or wrong pin.
//            ✗ if a motor spins backward when "forward": swap FWD↔REV in the
//              #defines for that motor and re-flash.
//   Phase 2: all 4 forward together.   (rover would move forward)
//   Phase 3: all 4 reverse together.   (rover would move backward)
//   Phase 4: pivot LEFT  → left rev, right fwd. (rover rotates CCW from above)
//   Phase 5: pivot RIGHT → left fwd, right rev. (rover rotates CW from above)

// ── Pin assignments (match the main firmware exactly) ───────────────────────
#define LEFT_FRONT_FWD_PIN   10
#define LEFT_FRONT_REV_PIN    9
#define RIGHT_FRONT_FWD_PIN   6
#define RIGHT_FRONT_REV_PIN   5
#define LEFT_BACK_FWD_PIN    12
#define LEFT_BACK_REV_PIN    11
#define RIGHT_BACK_FWD_PIN    3
#define RIGHT_BACK_REV_PIN    4

#define TEST_PWM      120UL     // moderate speed (~47% of full); safe for bench
#define PHASE_MS      2000UL    // duration of each spin phase
#define STOP_MS       1000UL    // pause between phases
#define CYCLE_GAP_MS  5000UL    // pause between full cycles

// ── Low-level motor primitives ──────────────────────────────────────────────

// Drive one motor at signed PWM (positive = forward, negative = reverse).
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

void stopAll() {
    analogWrite(LEFT_FRONT_FWD_PIN,  0);
    analogWrite(LEFT_FRONT_REV_PIN,  0);
    analogWrite(RIGHT_FRONT_FWD_PIN, 0);
    analogWrite(RIGHT_FRONT_REV_PIN, 0);
    analogWrite(LEFT_BACK_FWD_PIN,   0);
    analogWrite(LEFT_BACK_REV_PIN,   0);
    analogWrite(RIGHT_BACK_FWD_PIN,  0);
    analogWrite(RIGHT_BACK_REV_PIN,  0);
}

// Drive all 4 motors at the same signed PWM.
void allDrive(int pwm) {
    applyMotor(LEFT_FRONT_FWD_PIN,  LEFT_FRONT_REV_PIN,  pwm);
    applyMotor(RIGHT_FRONT_FWD_PIN, RIGHT_FRONT_REV_PIN, pwm);
    applyMotor(LEFT_BACK_FWD_PIN,   LEFT_BACK_REV_PIN,   pwm);
    applyMotor(RIGHT_BACK_FWD_PIN,  RIGHT_BACK_REV_PIN,  pwm);
}

// Drive left side at l_pwm, right side at r_pwm (each side has 2 motors).
void sideDrive(int l_pwm, int r_pwm) {
    applyMotor(LEFT_FRONT_FWD_PIN,  LEFT_FRONT_REV_PIN,  l_pwm);
    applyMotor(LEFT_BACK_FWD_PIN,   LEFT_BACK_REV_PIN,   l_pwm);
    applyMotor(RIGHT_FRONT_FWD_PIN, RIGHT_FRONT_REV_PIN, r_pwm);
    applyMotor(RIGHT_BACK_FWD_PIN,  RIGHT_BACK_REV_PIN,  r_pwm);
}

// ── Test phases ─────────────────────────────────────────────────────────────

// One motor: 2 s forward, 1 s stop, 2 s reverse, 1 s stop.
void testOneMotor(const char* label, int fwd_pin, int rev_pin) {
    Serial.print("[motor_test] "); Serial.print(label); Serial.println(" forward");
    applyMotor(fwd_pin, rev_pin, TEST_PWM);
    delay(PHASE_MS);
    stopAll();
    delay(STOP_MS);

    Serial.print("[motor_test] "); Serial.print(label); Serial.println(" reverse");
    applyMotor(fwd_pin, rev_pin, -(int)TEST_PWM);
    delay(PHASE_MS);
    stopAll();
    delay(STOP_MS);
}

// ── Setup / loop ────────────────────────────────────────────────────────────

void setup() {
    pinMode(LEFT_FRONT_FWD_PIN,  OUTPUT);
    pinMode(LEFT_FRONT_REV_PIN,  OUTPUT);
    pinMode(RIGHT_FRONT_FWD_PIN, OUTPUT);
    pinMode(RIGHT_FRONT_REV_PIN, OUTPUT);
    pinMode(LEFT_BACK_FWD_PIN,   OUTPUT);
    pinMode(LEFT_BACK_REV_PIN,   OUTPUT);
    pinMode(RIGHT_BACK_FWD_PIN,  OUTPUT);
    pinMode(RIGHT_BACK_REV_PIN,  OUTPUT);

    stopAll();

    Serial.begin(9600);
    while (!Serial && millis() < 2000) {}  // give USB serial up to 2 s to come up
    Serial.println("[motor_test] starting in 3 seconds — wheels OFF the ground!");
    delay(3000);
}

void loop() {
    Serial.println();
    Serial.println("=== Phase 1: identify each motor ===");
    testOneMotor("LEFT  FRONT", LEFT_FRONT_FWD_PIN,  LEFT_FRONT_REV_PIN);
    testOneMotor("RIGHT FRONT", RIGHT_FRONT_FWD_PIN, RIGHT_FRONT_REV_PIN);
    testOneMotor("LEFT  BACK",  LEFT_BACK_FWD_PIN,   LEFT_BACK_REV_PIN);
    testOneMotor("RIGHT BACK",  RIGHT_BACK_FWD_PIN,  RIGHT_BACK_REV_PIN);

    Serial.println("=== Phase 2: ALL FORWARD ===");
    allDrive(TEST_PWM);
    delay(PHASE_MS);
    stopAll();
    delay(STOP_MS);

    Serial.println("=== Phase 3: ALL REVERSE ===");
    allDrive(-(int)TEST_PWM);
    delay(PHASE_MS);
    stopAll();
    delay(STOP_MS);

    Serial.println("=== Phase 4: pivot LEFT (CCW from above) ===");
    sideDrive(-(int)TEST_PWM, TEST_PWM);
    delay(PHASE_MS);
    stopAll();
    delay(STOP_MS);

    Serial.println("=== Phase 5: pivot RIGHT (CW from above) ===");
    sideDrive(TEST_PWM, -(int)TEST_PWM);
    delay(PHASE_MS);
    stopAll();
    delay(STOP_MS);

    Serial.println("=== cycle complete — restarting in 5 s ===");
    delay(CYCLE_GAP_MS);
}
