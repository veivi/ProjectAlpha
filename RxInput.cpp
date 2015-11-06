#include "RxInput.h"
#include "Interrupt.h"

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

