// SANCHO — slow forward bench test for rough-terrain runs.
//
// Same pattern as the straight_test (5 s forward, 5 s stop) but at lower
// PWM to keep the rover slow and observable on uneven ground while still
// having enough torque to roll over small obstacles.
//
// SAFETY: verify wheels turn correctly off the ground first, then place on
// the test terrain.

#define LEFT_FRONT_RPWM   10
#define RIGHT_FRONT_RPWM   6
#define LEFT_BACK_RPWM     9
#define RIGHT_BACK_RPWM    3

// Rear (300 RPM) sets the target speed; front (333 RPM) scaled to match.
#define BACK_PWM   100                                       // ~39% duty
#define FRONT_PWM  ((int)(BACK_PWM * 300.0 / 333.0 + 0.5))   // = 90

#define ON_MS   5000UL
#define OFF_MS  5000UL

void setup() {
    Serial.begin(9600);
    while (!Serial && millis() < 2000) {}
    Serial.print("[slow_test] FRONT_PWM=");
    Serial.print(FRONT_PWM);
    Serial.print("  BACK_PWM=");
    Serial.println(BACK_PWM);
    Serial.println("[slow_test] starting — 5 s forward, 5 s stop");
}

void loop() {
    Serial.println("[slow_test] FORWARD");
    analogWrite(LEFT_FRONT_RPWM,  FRONT_PWM);
    analogWrite(RIGHT_FRONT_RPWM, FRONT_PWM);
    analogWrite(LEFT_BACK_RPWM,   BACK_PWM);
    analogWrite(RIGHT_BACK_RPWM,  BACK_PWM);
    delay(ON_MS);

    Serial.println("[slow_test] STOP");
    analogWrite(LEFT_FRONT_RPWM,  0);
    analogWrite(RIGHT_FRONT_RPWM, 0);
    analogWrite(LEFT_BACK_RPWM,   0);
    analogWrite(RIGHT_BACK_RPWM,  0);
    delay(OFF_MS);
}
