// SANCHO MCU firmware — runs on the STM32U585 of the Arduino UNO Q.
//
// Stripped to MOTORS-ONLY for the moment: outgoing Bridge.notify traffic
// (ultrasonic) was saturating the bridge and starving the incoming
// set_motors RPC. We removed the HC-SR04 sampling entirely; obstacle
// detection will come back once basic 4WD motion works end-to-end.
//
// IMPORTANT — UNO Q PWM gotcha:
//   On Zephyr, calling pinMode(pin, OUTPUT) BEFORE analogWrite(pin, val)
//   breaks PWM (the pin gets stuck as plain digital). analogWrite()
//   configures the pin internally. So: NO pinMode() on motor pins.
//   Also: the very first analogWrite() configures the timer with the
//   requested duty cycle, so if it's 0 the peripheral stays inert.
//   We force-init each pin with a brief PWM=1 pulse before stopMotors().
//
// Forward-only drive: BTS7960 LPWM is left floating. Negative PWM is
// clamped to 0 (motor coasts). 4WD scaling: front 520 motors throttled
// to 0.90 to match back 545s' lower no-load RPM.

#include "Arduino_RouterBridge.h"

// ── Forward-only PWM pins ────────────────────────────────────────────────────
#define LEFT_FRONT_RPWM_PIN    9
#define RIGHT_FRONT_RPWM_PIN   6
#define LEFT_BACK_RPWM_PIN    10
#define RIGHT_BACK_RPWM_PIN    3

#define FRONT_SCALE  0.90f
#define BACK_SCALE   1.00f

// ── Watchdog ─────────────────────────────────────────────────────────────────
#define MOTOR_WATCHDOG_MS  500UL

unsigned long lastSetMotorsMs    = 0;
unsigned long lastSetMotorsLogMs = 0;
uint32_t      setMotorsCallCount = 0;

void applyMotor(int rpwm_pin, int pwm);
void stopMotors();

// ── RPC handlers ─────────────────────────────────────────────────────────────
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

    // Diagnostic — once every 5 s
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

// ── Setup ────────────────────────────────────────────────────────────────────
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

    Bridge.provide_safe("set_motors",     setMotors);
    Bridge.provide_safe("emergency_stop", emergencyStop);

    Monitor.println("[sancho_bridge] MCU ready (motors-only)");
}

// ── Main loop ────────────────────────────────────────────────────────────────
void loop() {
    if (millis() - lastSetMotorsMs > MOTOR_WATCHDOG_MS) {
        stopMotors();
    }
}

// ── Motor helpers ────────────────────────────────────────────────────────────
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
