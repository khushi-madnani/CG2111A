#include "arm_driver.h"
#include <avr/interrupt.h>

// Pins on PORTK (Mega Pins 62-65)
#define B_BIT (1 << 0)
#define S_BIT (1 << 1)
#define E_BIT (1 << 2)
#define G_BIT (1 << 3)

// Internals
volatile int actualTicks[4] = {3000, 3000, 3000, 3000}; 
volatile int targetTicks[4] = {3000, 3000, 3000, 3000};
volatile int stepSpeed = 30; // Ticks per 20ms (higher = faster)

const int limits[4][2] = {{0, 180}, {20, 100}, {75, 180}, {0, 90}};

// --- TIMER 5 ISR ---
ISR(TIMER5_COMPA_vect) {
    static uint8_t slot = 0;
    PORTK &= ~0x0F; // Reset pins at start of slot

    if (slot < 4) {
        // Move actual pulse width toward target pulse width (Smoothness)
        if (actualTicks[slot] < targetTicks[slot]) {
            actualTicks[slot] += stepSpeed;
            if (actualTicks[slot] > targetTicks[slot]) actualTicks[slot] = targetTicks[slot];
        } else if (actualTicks[slot] > targetTicks[slot]) {
            actualTicks[slot] -= stepSpeed;
            if (actualTicks[slot] < targetTicks[slot]) actualTicks[slot] = targetTicks[slot];
        }

        PORTK |= (1 << slot);            // Turn pin ON
        OCR5B = TCNT5 + actualTicks[slot]; // Set OFF time
    }
    slot = (slot + 1) & 0x07; // Cycle 0-7 (20ms total)
}

ISR(TIMER5_COMPB_vect) {
    PORTK &= ~0x0F; // Turn pin OFF
}

void armInit() {
    // 1. Set Port K 0-3 as outputs without affecting pins 4-7
    DDRK |= 0x0F; 

    cli(); // Disable interrupts during configuration

    // 2. TCCR5A: Clear only the Waveform Generation Mode bits (WGM50, WGM51)
    // and Compare Output Mode bits, leaving other bits as they were.
    TCCR5A &= ~((1 << WGM51) | (1 << WGM50) | (1 << COM5A1) | (1 << COM5A0) | (1 << COM5B1) | (1 << COM5B0));

    // 3. TCCR5B: Clear WGM53, WGM52 and Clock Select bits first, 
    // then set CTC mode (WGM52) and Prescaler 8 (CS51).
    TCCR5B &= ~((1 << WGM53) | (1 << WGM52) | (1 << CS52) | (1 << CS51) | (1 << CS50));
    TCCR5B |= (1 << WGM52) | (1 << CS51);

    // 4. Set the Compare Registers
    OCR5A = 5000; // 2.5ms interval
    OCR5B = 3000; // Default neutral pulse

    // 5. Enable Timer 5 Compare Match A and B interrupts
    // Use |= so we don't disable other Timer 5 interrupts (like Overflow)
    TIMSK5 |= (1 << OCIE5A) | (1 << OCIE5B);

    sei(); // Re-enable interrupts
}
void handleArmCommand(char joint, int val) {
    int idx = -1;
    if (joint == 'V') { stepSpeed = val; return; }
    else if (joint == 'B') idx = 0;
    else if (joint == 'S') idx = 1;
    else if (joint == 'E') idx = 2;
    else if (joint == 'G') idx = 3;

    if (idx != -1) {
        val = constrain(val, limits[idx][0], limits[idx][1]);
        targetTicks[idx] = 2000 + ((long)val * 2000 / 180);
    }
}

void armStop() {
    // Lock arm in current physical position immediately
    for(int i=0; i<4; i++) {
        targetTicks[i] = actualTicks[i];
    }
}