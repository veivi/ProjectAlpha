#ifndef PPM_H
#define PPM_H

#include <Arduino.h>
#include "RxInput.h"

//

extern uint8_t ppmNumChannels;
extern boolean ppmWarn;

void ppmInputInit(struct RxInputRecord *inputs[], int num);
  
#endif

