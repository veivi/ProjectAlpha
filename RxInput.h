#ifndef RXINPUT_H
#define RXINPUT_H

#include <Arduino.h>

//

struct RxInputRecord {
  uint8_t pin, index;
  boolean freqOnly, alive;
  uint32_t pulseStart;
  uint32_t pulseCount;
  uint32_t pulseWidthAcc;
};
  
boolean inputValid(struct RxInputRecord *record);
float inputValue(struct RxInputRecord *record);

#endif

