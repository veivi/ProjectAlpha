#ifndef PPM_H
#define PPM_H

#include <Arduino.h>

//

extern uint8_t ppmNumChannels;
extern boolean ppmWarn;

struct RxInputRecord {
  uint8_t pin, portBit;
  boolean freqOnly, alive;
  uint32_t pulseStart;
  uint32_t pulseCount;
  uint32_t pulseWidthAcc;
};

void ppmInputInit(struct RxInputRecord *inputs[], int num);
  
#endif

