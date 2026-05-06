// SANCHO — simplest motor bench test: all 4 motors forward, continuously.
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

#define TEST_PWM 120

void setup() {
    pinMode(LEFT_FRONT_FWD_PIN,  OUTPUT);
    pinMode(LEFT_FRONT_REV_PIN,  OUTPUT);
    pinMode(RIGHT_FRONT_FWD_PIN, OUTPUT);
    pinMode(RIGHT_FRONT_REV_PIN, OUTPUT);
    pinMode(LEFT_BACK_FWD_PIN,   OUTPUT);
    pinMode(LEFT_BACK_REV_PIN,   OUTPUT);
    pinMode(RIGHT_BACK_FWD_PIN,  OUTPUT);
    pinMode(RIGHT_BACK_REV_PIN,  OUTPUT);

    analogWrite(LEFT_FRONT_REV_PIN,  0);
    analogWrite(RIGHT_FRONT_REV_PIN, 0);
    analogWrite(LEFT_BACK_REV_PIN,   0);
    analogWrite(RIGHT_BACK_REV_PIN,  0);

    analogWrite(LEFT_FRONT_FWD_PIN,  TEST_PWM);
    analogWrite(RIGHT_FRONT_FWD_PIN, TEST_PWM);
    analogWrite(LEFT_BACK_FWD_PIN,   TEST_PWM);
    analogWrite(RIGHT_BACK_FWD_PIN,  TEST_PWM);
}

void loop() {
}
