// SANCHO — straight-line drive test.
//
// Drives all four motors forward at equal linear speed.
// Front motors (333 RPM nominal) are scaled down relative to rear motors
// (300 RPM nominal) so that all tracks advance at the same rate:
//
//   FRONT_SCALE = 300 / 333 = 0.9009...
//
//   BACK_PWM  = TEST_PWM
//   FRONT_PWM = round(TEST_PWM * 300.0 / 333.0)
//
// With TEST_PWM = 150: FRONT_PWM = 135, BACK_PWM = 150.
//
// Pattern: 5 s forward → 5 s stop → repeat.
// Watch for drift — if the rover pulls left or right, one side has a
// mechanical asymmetry that requires per-side calibration in sancho_params.yaml
// (left_scale / right_scale).
//
// IMPORTANT — UNO Q PWM gotcha:
//   On Zephyr, calling pinMode(pin, OUTPUT) BEFORE analogWrite() breaks PWM
//   (pin gets stuck as plain digital). analogWrite() configures the pin itself.
//   So: NO pinMode() on motor pins.
//
// SAFETY: wheels OFF the ground for the first run; motor battery 12 V; GND
// of battery shared with GND of UNO Q.

#define LEFT_FRONT_RPWM   9
#define RIGHT_FRONT_RPWM  6
#define LEFT_BACK_RPWM   10
#define RIGHT_BACK_RPWM   3

// Rear (300 RPM) sets the target speed; front (333 RPM) scaled to match.
#define BACK_PWM   150                          // ~58% duty on rear motors
#define FRONT_PWM  ((int)(BACK_PWM * 300.0 / 333.0 + 0.5))  // = 135

#define ON_MS   5000UL
#define OFF_MS  5000UL

void setup() {
    Serial.begin(9600);
    while (!Serial && millis() < 2000) {}
    Serial.print("[straight_test] FRONT_PWM=");
    Serial.print(FRONT_PWM);
    Serial.print("  BACK_PWM=");
    Serial.println(BACK_PWM);
    Serial.println("[straight_test] starting loop — 5 s forward, 5 s stop");
}

void loop() {
    Serial.println("[straight_test] FORWARD");
    analogWrite(LEFT_FRONT_RPWM,  FRONT_PWM);
    analogWrite(RIGHT_FRONT_RPWM, FRONT_PWM);
    analogWrite(LEFT_BACK_RPWM,   BACK_PWM);
    analogWrite(RIGHT_BACK_RPWM,  BACK_PWM);
    delay(ON_MS);

    Serial.println("[straight_test] STOP");
    analogWrite(LEFT_FRONT_RPWM,  0);
    analogWrite(RIGHT_FRONT_RPWM, 0);
    analogWrite(LEFT_BACK_RPWM,   0);
    analogWrite(RIGHT_BACK_RPWM,  0);
    delay(OFF_MS);
}
