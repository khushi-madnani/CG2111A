#ifndef ARM_DRIVER_H
#define ARM_DRIVER_H

#include <Arduino.h>

// Initialize Timer 5 and Port K
void armInit();

// Non-blocking command handler
void handleArmCommand(char joint, int val);

// Emergency stop for the arm
void armStop();

#endif