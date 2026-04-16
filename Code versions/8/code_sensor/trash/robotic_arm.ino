#include "Arduino.h"
#include <avr/interrupt.h>

// ---------------- PIN DEFINITIONS ----------------
#define BASE_PIN (1 << 0)
#define SHOULDER_PIN (1 << 1)
#define ELBOW_PIN (1 << 2)
#define GRIPPER_PIN (1 << 3)

// ---------------- LIMITS ----------------
#define BASE_MIN 0
#define BASE_MAX 180
#define SHOULDER_MIN 20
#define SHOULDER_MAX 100
#define ELBOW_MIN 75
#define ELBOW_MAX 180
#define GRIPPER_MIN 0
#define GRIPPER_MAX 90

// ---------------- GLOBALS ----------------
char c = '\0';
int basePos = 90, shoulderPos = 90, elbowPos = 75, gripperPos = 90;
volatile int msPerDeg = 10;
volatile uint8_t currentPin = 0;

// ---------------- FUNCTIONS ----------------
int angleToPulse(int angle) {
  return 2000 + ((long)angle * 2000 / 180);
}

ISR(TIMER5_COMPA_vect) {
  if (currentPin != 0) PORTK |= currentPin;
}

ISR(TIMER5_COMPB_vect) {
  if (currentPin != 0) PORTK &= ~currentPin;
}

void moveSmooth(uint8_t servoPin, int *cur, int target) {
  PORTK &= ~currentPin;
  currentPin = servoPin;

  int minV = 0, maxV = 180;

  if (servoPin == BASE_PIN) { minV = BASE_MIN; maxV = BASE_MAX; }
  else if (servoPin == SHOULDER_PIN) { minV = SHOULDER_MIN; maxV = SHOULDER_MAX; }
  else if (servoPin == ELBOW_PIN) { minV = ELBOW_MIN; maxV = ELBOW_MAX; }
  else { minV = GRIPPER_MIN; maxV = GRIPPER_MAX; }

  target = constrain(target, minV, maxV);

  int curTicks = angleToPulse(*cur);
  int tgtTicks = angleToPulse(target);

  if (tgtTicks > curTicks) {
    for (int t = curTicks; t <= tgtTicks; t += (4000 / 180)) {
      OCR5B = t;
      delay(msPerDeg);
    }
  } else {
    for (int t = curTicks; t >= tgtTicks; t -= (4000 / 180)) {
      OCR5B = t;
      delay(msPerDeg);
    }
  }

  OCR5B = tgtTicks;
  *cur = target;
}

void homeAll() {
  moveSmooth(BASE_PIN, &basePos, 90);
  moveSmooth(SHOULDER_PIN, &shoulderPos, 15);
  moveSmooth(ELBOW_PIN, &elbowPos, 75);
  moveSmooth(GRIPPER_PIN, &gripperPos, 90);
}

void armInit() {
  DDRK |= (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);

  cli();
  TCCR5A = 0;
  TCCR5B = (1 << WGM52) | (1 << CS51);
  OCR5A = 40000;
  OCR5B = 3000;
  TIMSK5 = (1 << OCIE5A) | (1 << OCIE5B);
  sei();

  delay(1500);
  homeAll();
}

void handleArmCommand(char joint, int val) {
  if (joint == 'V') {
    msPerDeg = val;
  }
  else if (joint == 'B') {
    moveSmooth(BASE_PIN, &basePos, val);
  }
  else if (joint == 'S') {
    moveSmooth(SHOULDER_PIN, &shoulderPos, val);
  }
  else if (joint == 'E') {
    moveSmooth(ELBOW_PIN, &elbowPos, val);
  }
  else if (joint == 'G') {
    moveSmooth(GRIPPER_PIN, &gripperPos, val);
  }
}

