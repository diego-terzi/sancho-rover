// SANCHO — simple bench-test sketch for the 4WD motors.
//
// Sequence: all forward → all reverse → each motor individually.
//
// SAFETY: wheels OFF the ground, all 4 BTS7960 modules powered by the motor
// battery (12V), GND of the battery shared with GND of the UNO Q.

#define LEFT_FRONT_FWD_PIN   10
#define LEFT_FRONT_REV_PIN    9
#define RIGHT_FRONT_FWD_PIN   6
#define RIGHT_FRONT_REV_PIN   5
#define LEFT_BACK_FWD_PIN    12
#define LEFT_BACK_REV_PIN    11
#define RIGHT_BACK_FWD_PIN    3
#define RIGHT_BACK_REV_PIN    4

#define TEST_PWM   120
#define PHASE_MS   3000UL
#define STOP_MS    1500UL

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

void allDrive(int pwm) {
    applyMotor(LEFT_FRONT_FWD_PIN,  LEFT_FRONT_REV_PIN,  pwm);
    applyMotor(RIGHT_FRONT_FWD_PIN, RIGHT_FRONT_REV_PIN, pwm);
    applyMotor(LEFT_BACK_FWD_PIN,   LEFT_BACK_REV_PIN,   pwm);
    applyMotor(RIGHT_BACK_FWD_PIN,  RIGHT_BACK_REV_PIN,  pwm);
}

void testOneMotor(const char* label, int fwd_pin, int rev_pin) {
    Serial.print("[motor_test] "); Serial.println(label);
    applyMotor(fwd_pin, rev_pin, TEST_PWM);
    delay(PHASE_MS);
    stopAll();
    delay(STOP_MS);
}

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
    while (!Serial && millis() < 2000) {}
    Serial.println("[motor_test] starting in 3 s — wheels OFF the ground!");
    delay(3000);
}

void loop() {
    Serial.println();
    Serial.println("=== ALL FORWARD ===");
    allDrive(TEST_PWM);
    delay(PHASE_MS);
    stopAll();
    delay(STOP_MS);

    Serial.println("=== ALL REVERSE ===");
    allDrive(-TEST_PWM);
    delay(PHASE_MS);
    stopAll();
    delay(STOP_MS);

    Serial.println("=== ONE MOTOR AT A TIME ===");
    testOneMotor("LEFT  FRONT", LEFT_FRONT_FWD_PIN,  LEFT_FRONT_REV_PIN);
    testOneMotor("RIGHT FRONT", RIGHT_FRONT_FWD_PIN, RIGHT_FRONT_REV_PIN);
    testOneMotor("LEFT  BACK",  LEFT_BACK_FWD_PIN,   LEFT_BACK_REV_PIN);
    testOneMotor("RIGHT BACK",  RIGHT_BACK_FWD_PIN,  RIGHT_BACK_REV_PIN);

    Serial.println("=== cycle complete, restarting in 5 s ===");
    delay(5000);
}
