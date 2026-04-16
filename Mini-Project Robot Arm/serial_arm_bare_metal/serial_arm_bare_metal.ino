#include "Arduino.h"
#include <avr/interrupt.h>

#define BASE_PIN (1 << 0)      // A0
#define SHOULDER_PIN (1 << 1)  // A1
#define ELBOW_PIN (1 << 2)     // A2
#define GRIPPER_PIN (1 << 3)   // A3

// Constrain limits for the 4 servos
#define BASE_MIN 0
#define BASE_MAX 180
#define SHOULDER_MIN 20
#define SHOULDER_MAX 100
#define ELBOW_MIN 75
#define ELBOW_MAX 180
#define GRIPPER_MIN 0
#define GRIPPER_MAX 90

char c = '\u0000'; //fancy way to say null
int basePos = 90, shoulderPos = 90, elbowPos = 75, gripperPos = 90;
volatile int msPerDeg = 10;
volatile uint8_t currentPin = 0;

// Helper to convert angle to ticks (0.5us resolution)
// 0 deg = 1000 ticks (0.5ms), 180 deg = 5000 ticks (2.5ms) oliver: i changed the range from 1ms to 2ms since sg90 expects that range, anything beyond might cause it to punch, if you want a wider range u can do 0.9-2.1ms 
int angleToPulse(int angle) { //this function ensures that we sorta stay within the range of pulses based on what angle we want(0-180 deg)
  return 2000 + ((long)angle * 2000 / 180);
}

int parse3(const String *s) { //converts s digit input into an int
  if (!s || s->length() != 3) return -1;
  if (!isDigit(s->charAt(0)) || !isDigit(s->charAt(1)) || !isDigit(s->charAt(2))) return -1;
  return s->toInt();
}

ISR(TIMER1_COMPA_vect) { //fires every 20ms upon ocr1a compare match
  if (currentPin != 0) {
    PORTC |= currentPin; //runs if currentPin is high and if it is, opens portc setting that pin to high. basically sets selected servo output to high
  } 
}

ISR(TIMER1_COMPB_vect) {
  if (currentPin != 0) { 
    PORTC &= ~currentPin; //sets pulse low after ocr1b ticks after start
  } //ocr1b < ocr1a, ocr1b sets the finish line for the pulse
} //since servo ranges from 0.5ms to 2.5ms, ocr1b range should only stay within this range and adjust pulse(high) to last within 0.5ms to 2.5ms

void moveSmooth(uint8_t servoPin, int *cur, int target) {
  PORTC &= ~currentPin;  // Force old pin low immediately
  currentPin = servoPin;

  // Constrain target to prevent physical damage
  int currentMin = 0;
  int currentMax = 180;
  if (servoPin == BASE_PIN) {
    currentMin = BASE_MIN;
    currentMax = BASE_MAX;
  } else if (servoPin == SHOULDER_PIN) {
    currentMin = SHOULDER_MIN;
    currentMax = SHOULDER_MAX;
  } else if (servoPin == ELBOW_PIN) {
    currentMin = ELBOW_MIN;
    currentMax = ELBOW_MAX;
  } else {
    currentMin = GRIPPER_MIN;
    currentMax = GRIPPER_MAX;
  }
  target = constrain(target, currentMin, currentMax); //constrains target to be within the range(currentMin and currentMax)

  int current_ticks = angleToPulse(*cur);
  int target_ticks = angleToPulse(target); //angle to pulse keeps us in the correct range for pulses

  // Move gradually by updating OCR1B, ocr1b adjusts the duty cycle/pulse width, frequency doesnt change only duty cycle
  if (target_ticks > current_ticks) {
    for (int ticks = current_ticks; ticks <= target_ticks; ticks += (4000 / 180)) { //4000/180 moves by 1 degree per iteration, 1 step/1 deg is 22 ticks -> 1 deg per loop iteration
      OCR1B = ticks;  // Update pulse width, finihs line changes so servo reacts to increased pulse width and changes angle
      delay(msPerDeg); //delay for smooth movement
    }
  } else {
    for (int ticks = current_ticks; ticks >= target_ticks; ticks -= (4000 / 180)) { //move back if desired angle is less than current angle
      OCR1B = ticks; //same idea, update pulse widTh
      delay(msPerDeg);
    }
  }

  // Ensure we land exactly on target, this is incase loop didnt land perfectly, this is a backup to snap the servo to the right position.
  OCR1B = target_ticks; //maybe this causes the punching? maybe replace with for loop for smooth movement back?
  *cur = target;
}

void homeAll() { //just uses movesmooth to go back to all default positions
  moveSmooth(BASE_PIN, &basePos, 90);
  moveSmooth(SHOULDER_PIN, &shoulderPos, 15);
  moveSmooth(ELBOW_PIN, &elbowPos, 75);
  moveSmooth(GRIPPER_PIN, &gripperPos, 90);
}

void setup() {
  Serial.begin(115200);

  DDRC |= (BASE_PIN | SHOULDER_PIN | ELBOW_PIN | GRIPPER_PIN);
  cli();

  // Mode: CTC with OCR1A as TOP
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);  // Prescaler 8

  // Set Period to 20ms (40,000 ticks)
  OCR1A = 40000;

  // Set Initial Pulse to 1.5ms (Midpoint)
  OCR1B = 3000;

  // Enable Interrupts for Match A (Period) and Match B (Pulse)
  TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B);
  sei();
  delay(1500);
  homeAll();
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "H") {
    Serial.println("Homing all servos...");
    homeAll();
    return;
  }

  if (cmd.length() != 4) {
    Serial.println("ERROR: Command is not 4 characters long");
    return;
  }

  c = cmd.charAt(0);
  int val = parse3(&cmd.substring(1));
  if (val < 0) {
    Serial.println("ERROR: Argument not valid");
    return;
  }


  if (c == 'V') {
    Serial.print("Setting velocity to ");
    Serial.println(val);
    msPerDeg = val;
    return;
  } else {
    if (c == 'B') {
      Serial.print("Moving base to ");
      Serial.println(val);
      moveSmooth(BASE_PIN, &basePos, val);
      return;
    } else if (c == 'S') {
      Serial.print("Moving shoulder to ");
      Serial.println(val);
      moveSmooth(SHOULDER_PIN, &shoulderPos, val);
      return;
    } else if (c == 'E') {
      Serial.print("Moving shoulder to ");
      Serial.println(val);
      moveSmooth(ELBOW_PIN, &elbowPos, val);
      return;
    } else if (c == 'G') {
      Serial.print("Moving gripper to ");
      Serial.println(val);
      moveSmooth(GRIPPER_PIN, &gripperPos, val);
      return;
    } else {
      Serial.println("ERROR: Unknown command");
      return;
    }
  }
}
