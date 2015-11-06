#include <EEPROM.h>
#include "NVState.h"
#include "Console.h"

// NV store layout

const int paramOffset = 0;
const int stateOffset = MAX_MODELS*sizeof(ParamRecord);

struct ParamRecord paramRecord[MAX_MODELS];
struct NVStateRecord stateRecord;

static uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  crc ^= a;
  for (int i = 0; i < 8; ++i) {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001U;
    else
      crc = (crc >> 1);
  }
  
  return crc;
}

static uint16_t crc16(uint16_t initial, const uint8_t *data, int len)
{
  uint16_t crc = initial;
  
  for(int i = 0; i < len; i++)
    crc = crc16_update(crc, data[i]);
    
  return crc;
}

static uint16_t crc16OfRecord(uint16_t initial, const uint8_t *record, int size)
{
  return crc16(initial, &record[sizeof(uint16_t)], size - sizeof(uint16_t));
}

static uint16_t paramRecordCrc(struct ParamRecord *record)
{
  return crc16OfRecord(0xFFFF, (uint8_t*) record, sizeof(*record));
}

static uint16_t stateRecordCrc(struct NVStateRecord *record)
{
  return crc16OfRecord(0xFFFF, (uint8_t*) record, sizeof(*record));
}

struct ParamRecord paramDefaults = {
      0,
      64, 0x50, 
      12, 12,
      0,
      -25.0/90, -50.0/90,
      0, 0, 0, 0,
      -3.0/360,  12.0/360,  0,
      0.65, 0.35, 0.04, 35.0, 
      0.5, 1.2, 0.01,
      2,
       0.5, -0.3,
      -15.0/90, -50.0/90 };

struct NVStateRecord stateDefaults = { 0, 400, 0, false, 0 };

static void readBlock(char *ptr, uint16_t addr, int size)
{
  for(int i = 0; i < size; i++)
    ptr[i] = EEPROM.read(addr+i);
}

static void writeBlock(const char *ptr, uint16_t addr, int size)
{
  for(int i = 0; i < size; i++)
    EEPROM.write(addr+i, ptr[i]);
}

void readParams(void)
{
  readBlock((char*) paramRecord, paramOffset, sizeof(paramRecord));

  for(int i = 0; i < MAX_MODELS; i++) {
    if(paramRecordCrc(&paramRecord[i]) != paramRecord[i].crc) {
      consoleNote("MODEL ");
      consolePrintLn(i);    
      consolePrintLn("  Param record corrupt, using defaults"); 
      paramRecord[i] = paramDefaults;
    }
  }
}

void storeParams(void)
{
  paramRecord[stateRecord.model].crc = paramRecordCrc(&paramRecord[stateRecord.model]);
  writeBlock((const char*) &paramRecord[stateRecord.model], paramOffset+stateRecord.model*sizeof(struct ParamRecord), sizeof(struct ParamRecord));
}

void readNVState(void)
{
  readBlock((char*) &stateRecord, stateOffset, sizeof(stateRecord));
  
  consoleNote("  State record CRC = ");
  consolePrint(stateRecord.crc);
    
  if(stateRecord.crc != stateRecordCrc(&stateRecord)) {
    consolePrintLn(" CORRUPT, using defaults"); 
    stateRecord = stateDefaults;
  } else
    consolePrintLn(" OK");

}

void storeNVState(void)
{
  stateRecord.crc = stateRecordCrc(&stateRecord);
  writeBlock((const char*) &stateRecord, stateOffset, sizeof(stateRecord));
}

