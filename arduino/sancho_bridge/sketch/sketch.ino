// SANCHO MCU firmware — trail following demo
//
// Riceve set_motors(L, R) via Bridge dal Python shim e pilota i 4 motori.
// 4 LED indicano lo stato del rover:
//   verde  (14) — acceso quando i motori girano (rover segue la traccia)
//   blu    (15) — sempre acceso (modalità trail following attiva)
//   giallo (16) — sempre spento (riservato a human following)
//   rosso  (17) — lampeggia quando il rover è fermo / traccia persa
//
// Watchdog MCU: se non arriva set_motors per 500 ms, motori a 0.
//
// UNO Q PWM quirk (Zephyr): non usare pinMode sui pin motori.
// Inizializzare con analogWrite(pin, 1) poi subito stopMotors().

#include "Arduino_RouterBridge.h"

// ── Pin motori (RPWM only, forward-only, BTS7960) ───────────────────────────
#define LEFT_FRONT_RPWM_PIN    9
#define RIGHT_FRONT_RPWM_PIN   6
#define LEFT_BACK_RPWM_PIN    10
#define RIGHT_BACK_RPWM_PIN    3

// ── Compensazione RPM (front 333 RPM, rear 300 RPM) ─────────────────────────
#define FRONT_SCALE  (300.0f / 333.0f)

// ── Pin LED ──────────────────────────────────────────────────────────────────
#define LED_GREEN   14
#define LED_BLUE    15
#define LED_YELLOW  16
#define LED_RED     17

// ── Timing ───────────────────────────────────────────────────────────────────
#define MOTOR_WATCHDOG_MS  500UL
#define RED_BLINK_MS       300UL

// ── Stato ────────────────────────────────────────────────────────────────────
static int           _last_left        = 0;
static int           _last_right       = 0;
static unsigned long _last_cmd_ms      = 0;
static unsigned long _last_log_ms      = 0;
static uint32_t      _cmd_count        = 0;

static unsigned long _last_blink_ms   = 0;
static bool          _red_state       = false;

// ── Helpers (dichiarati prima dei callback Bridge) ───────────────────────────
void applyMotor(int pin, int pwm) {
    if (pwm < 0)   pwm = 0;
    if (pwm > 255) pwm = 255;
    analogWrite(pin, pwm);
}

void stopMotors() {
    analogWrite(LEFT_FRONT_RPWM_PIN,  0);
    analogWrite(RIGHT_FRONT_RPWM_PIN, 0);
    analogWrite(LEFT_BACK_RPWM_PIN,   0);
    analogWrite(RIGHT_BACK_RPWM_PIN,  0);
}

// ── Bridge callbacks ─────────────────────────────────────────────────────────
void setMotors(int left, int right) {
    applyMotor(LEFT_FRONT_RPWM_PIN,  (int)(left  * FRONT_SCALE));
    applyMotor(RIGHT_FRONT_RPWM_PIN, (int)(right * FRONT_SCALE));
    applyMotor(LEFT_BACK_RPWM_PIN,   left);
    applyMotor(RIGHT_BACK_RPWM_PIN,  right);

    _last_left  = left;
    _last_right = right;
    _last_cmd_ms = millis();

    _cmd_count++;
    if (millis() - _last_log_ms > 5000UL) {
        _last_log_ms = millis();
        Monitor.print("[setMotors #");
        Monitor.print(_cmd_count);
        Monitor.print("] L=");
        Monitor.print(left);
        Monitor.print(" R=");
        Monitor.println(right);
    }
}

void emergencyStop() {
    stopMotors();
    _last_left  = 0;
    _last_right = 0;
    _last_cmd_ms = 0;
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Bridge.begin();
    Monitor.begin();

    // LED
    pinMode(LED_GREEN,  OUTPUT);
    pinMode(LED_BLUE,   OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_RED,    OUTPUT);

    digitalWrite(LED_GREEN,  LOW);
    digitalWrite(LED_BLUE,   HIGH);  // sempre acceso
    digitalWrite(LED_YELLOW, LOW);   // sempre spento
    digitalWrite(LED_RED,    LOW);

    // Forza init PWM (Zephyr quirk — senza questo il primo analogWrite(0) è inerte)
    analogWrite(LEFT_FRONT_RPWM_PIN,  1);
    analogWrite(RIGHT_FRONT_RPWM_PIN, 1);
    analogWrite(LEFT_BACK_RPWM_PIN,   1);
    analogWrite(RIGHT_BACK_RPWM_PIN,  1);
    delay(50);
    stopMotors();

    Bridge.provide_safe("set_motors",     setMotors);
    Bridge.provide_safe("emergency_stop", emergencyStop);

    Monitor.println("[sancho_bridge] MCU ready — trail following demo");
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    // Watchdog: niente set_motors da troppo tempo → stop
    if (_last_cmd_ms > 0 && (now - _last_cmd_ms) > MOTOR_WATCHDOG_MS) {
        stopMotors();
        _last_left  = 0;
        _last_right = 0;
    }

    // LED: verde se in movimento, rosso lampeggiante se fermo
    bool moving = (_last_left != 0 || _last_right != 0);

    if (moving) {
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_RED,   LOW);
        _red_state = false;
    } else {
        digitalWrite(LED_GREEN, LOW);
        if (now - _last_blink_ms >= RED_BLINK_MS) {
            _red_state = !_red_state;
            digitalWrite(LED_RED, _red_state ? HIGH : LOW);
            _last_blink_ms = now;
        }
    }
}
