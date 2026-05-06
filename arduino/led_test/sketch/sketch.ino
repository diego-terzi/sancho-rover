// SANCHO — standalone test sketch for the 6-LED Audi-style turn signals.
//
// Purpose: flash this onto the UNO Q *before* integrating LEDs into the main
// sancho_bridge firmware, to verify wiring and pin assignments visually.
//
// Behaviour: alternates LEFT and RIGHT signal sweeps every ~1.5 s.
//   Sweep phases (each phase = SWEEP_PHASE_MS):
//     1) outermost LED on
//     2) outer + middle on
//     3) all three on
//     4) all off (gap before next cycle)
//
// Wiring (per LED):
//   GPIO ── [220 Ω] ── anode (long leg) ── cathode (short leg) ── GND
//
// Once this sketch lights all six LEDs in the right order, copy the pin
// macros into the main sancho_bridge sketch.ino and add the non-blocking
// updateLeds() function we'll write next.

#define LEFT_LED_INNER    A0
#define LEFT_LED_MIDDLE   13
#define LEFT_LED_OUTER    12

#define RIGHT_LED_INNER   A1
#define RIGHT_LED_MIDDLE  A2
#define RIGHT_LED_OUTER   A3

#define SWEEP_PHASE_MS    150
#define SWEEPS_PER_BURST  3
#define BURST_GAP_MS      500

const int LEFT_LEDS[3]  = { LEFT_LED_INNER,  LEFT_LED_MIDDLE,  LEFT_LED_OUTER };
const int RIGHT_LEDS[3] = { RIGHT_LED_INNER, RIGHT_LED_MIDDLE, RIGHT_LED_OUTER };

void allOff() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(LEFT_LEDS[i],  LOW);
        digitalWrite(RIGHT_LEDS[i], LOW);
    }
}

// One Audi sweep on the given side. pins[] runs from inner to outer.
void sweepOnce(const int pins[3]) {
    digitalWrite(pins[2], HIGH);  // outer
    delay(SWEEP_PHASE_MS);
    digitalWrite(pins[1], HIGH);  // outer + middle
    delay(SWEEP_PHASE_MS);
    digitalWrite(pins[0], HIGH);  // all three
    delay(SWEEP_PHASE_MS);
    digitalWrite(pins[0], LOW);
    digitalWrite(pins[1], LOW);
    digitalWrite(pins[2], LOW);
    delay(SWEEP_PHASE_MS);
}

void setup() {
    for (int i = 0; i < 3; i++) {
        pinMode(LEFT_LEDS[i],  OUTPUT);
        pinMode(RIGHT_LEDS[i], OUTPUT);
    }
    allOff();

    Serial.begin(9600);
    Serial.println("[led_test] sweep demo started");
}

void loop() {
    Serial.println("[led_test] LEFT");
    for (int i = 0; i < SWEEPS_PER_BURST; i++) sweepOnce(LEFT_LEDS);
    delay(BURST_GAP_MS);

    Serial.println("[led_test] RIGHT");
    for (int i = 0; i < SWEEPS_PER_BURST; i++) sweepOnce(RIGHT_LEDS);
    delay(BURST_GAP_MS);
}
