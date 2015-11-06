#include "RxInput.h"
#include "Interrupt.h"

float decodePWM(float pulse) {
  const float txRange = 0.81;
  return (pulse - 1500)/500.0/txRange;
}

boolean inputValid(struct RxInputRecord *record)
{
  return record->pulseCount > 0;
}

float inputValue(struct RxInputRecord *record)
{
  FORBID;
  
  uint32_t count = record->pulseCount, acc = record->pulseWidthAcc;
  record->pulseWidthAcc = record->pulseCount = 0;
  
  PERMIT;
  
  return (float) acc / count;
}

