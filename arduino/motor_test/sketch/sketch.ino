// SANCHO — minimal motor bench test (forward only).
//
// Drives all four motors forward via the RPWM input of each BTS7960 only.
// 5 s on, 5 s stop, repeat — easy to spot if any motor is silent.
//
// IMPORTANT — UNO Q PWM gotcha:
//   On the UNO Q's Zephyr core, calling pinMode(pin, OUTPUT) BEFORE
//   analogWrite(pin, val) breaks PWM on that pin (it gets stuck as plain
//   digital). analogWrite() configures the pin itself. So: no pinMode()
//   for any pin used with analogWrite().
//
// SAFETY: wheels OFF the ground; all 4 BTS7960 modules powered by the motor
// battery (12V); GND of the battery shared with GND of the UNO Q.

#define LEFT_FRONT_RPWM   9
#define RIGHT_FRONT_RPWM  6
#define LEFT_BACK_RPWM   10
#define RIGHT_BACK_RPWM   3

#define TEST_PWM   150     // ~58% duty (≈ 7 V on motor average)
#define ON_MS    5000UL
#define OFF_MS   5000UL

void setup() {
    // Intentionally no pinMode() — see header comment.
    Serial.begin(9600);
    while (!Serial && millis() < 2000) {}
    Serial.println("[motor_test] forward-only loop starting");
}

void loop() {
    Serial.println("[motor_test] FORWARD");
    analogWrite(LEFT_FRONT_RPWM,  TEST_PWM);
    analogWrite(RIGHT_FRONT_RPWM, TEST_PWM);
    analogWrite(LEFT_BACK_RPWM,   TEST_PWM);
    analogWrite(RIGHT_BACK_RPWM,  TEST_PWM);
    delay(ON_MS);

    Serial.println("[motor_test] STOP");
    analogWrite(LEFT_FRONT_RPWM,  0);
    analogWrite(RIGHT_FRONT_RPWM, 0);
    analogWrite(LEFT_BACK_RPWM,   0);
    analogWrite(RIGHT_BACK_RPWM,  0);
    delay(OFF_MS);
}
