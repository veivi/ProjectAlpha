#include <Arduino.h>
#include "NewI2C.h"

#define EXT_EEPROM_SIZE (1L<<17)
#define EXT_EEPROM_LATENCY 5000

extern int logBytesCum;
extern float logBandWidth;
extern boolean eepromWarn, eepromFailed;

void waitEEPROM(uint32_t addr);
void writeEEPROM(uint32_t addr, const uint8_t *data, int bytes);
boolean readEEPROM(uint32_t addr, uint8_t *data, int size);
void cacheFlush(void);
void cacheWrite(uint32_t addr, const uint8_t *value, int size);
void cacheRead(uint32_t addr, uint8_t *value, int size);

