#ifndef PPM_H
#define PPM_H

#include <Arduino.h>
#include "RxInput.h"

//

extern uint8_t ppmNumChannels;
extern uint32_t ppmFrames;
extern boolean ppmWarn;

void ppmInputInit(struct RxInputRecord *inputs[], int num);
  
#endif

