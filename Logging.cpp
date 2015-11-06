#include "Logging.h"
#include "NVState.h"
#include "Filter.h"

struct LogChannel {
  ChannelId_t ch;
  const char *name;
  float small, large;
  boolean tick;
  uint16_t value;
};

// Must match the order with logChannelId_t declaration!!

struct LogChannel logChannels[] = {
   { l_alpha, "ALPH", -180, 180, true },
   { l_dynpressure, "PRES", -100, 10000 },
   { l_acc, "G", 0, 10 },
   { l_roll, "ROLL", -180, 180 },
   { l_rollrate, "RRTE", -360, 360 },
   { l_pitch, "PTCH", -90, 90 },
   { l_pitchrate, "PRTE", -360, 360 },
   { l_heading, "HEAD", -180, 180},
   { l_ailestick, "ASTK", -1, 1 },
   { l_elevstick, "ESTK", -1, 1 },
   { l_aileron, "AILE", -1, 1 },
   { l_elevator, "ELEV", -1, 1 },
   { l_mode, "MODE", 0, 255 },
   { l_target, "TARG", -180, 180 },
   { l_trim, "TRIM", -180, 180 },
   { l_gain, "GAIN", 0, 50},
   { l_test, "TEST", 0, 255},
   { l_rpm, "RPM", 0, 50000 },
   { l_speed, "VELO", 0, 300 },
   { l_track, "TRAK", 0, 360 },
   { l_altgps, "ALTG", -10, 300 },
   { l_altbaro, "ALTB", -10, 300 } };

#define TOKEN_MASK (1U<<15)
#define VALUE_MASK (~TOKEN_MASK)
#define DELTA_MASK (VALUE_MASK>>1)
#define ENTRY_TOKEN(t) (TOKEN_MASK | (t))
#define ENTRY_VALUE(v) (((uint16_t) v) & VALUE_MASK)
#define ENTRY_IS_TOKEN(e) ((e) & TOKEN_MASK)

typedef enum { t_stamp,
               t_start, 
               t_mark, 
               t_channel = t_stamp + (1<<8),
               t_delta = t_stamp + (1<<14)
            } LogToken_t;

#define logIndex(i) ((logPtr + logSize + (i)) % logSize)

typedef enum { invalid_c, init_c, stop_c, run_c, failed_c } logState_t;

logState_t logState;
int32_t logPtr, logLen, logSize;
uint16_t logEndStamp;
boolean logEnabled = false;
  
static void logWrite(int32_t index, const uint16_t *value, int count)
{
  if(logSize < 1)
    return;
    
  cacheWrite(index*sizeof(uint16_t), (const uint8_t*) value, count*sizeof(uint16_t));
  logLen = -1L;
}

static void logWrite(int32_t index, const uint16_t value)
{
  logWrite(index, &value, 1);
}


static boolean logReady(void)
{
  if(logState == stop_c || logState == run_c)
    return true;

  consoleNoteLn("Log not ready");
  return false;
}

static uint16_t logRead(int32_t index)
{
  uint16_t entry = 0;
  cacheRead(index*sizeof(entry), (uint8_t*) &entry, sizeof(entry));
  return entry;
}

static void logCommit(int bytes)
{
  if(!logReady())
    return;
    
  logPtr = logIndex(bytes);
}

boolean logDirty = false;

static void logEnter(const uint16_t *value, int count)
{
  if(!logReady())
    return;
    
  logWrite(logIndex(0), value, count);
  uint16_t buffer[2] = { ENTRY_TOKEN(t_stamp), logEndStamp };
  logWrite(logIndex(count), buffer, 2);
  logCommit(count);
  
  logDirty = true;
}

static void logEnter(uint16_t value)
{
  logEnter(&value, 1);
}

void logClear(void)
{
  if(!logReady())
    return;
    
  consoleNoteLn("Log CLEARED");
    
  logEnter(ENTRY_TOKEN(t_start));

  stateRecord.logStamp++;
  storeNVState();
}
  
int prevCh = -1;

static void logWithCh(int ch, uint16_t value)
{
  if(logState != run_c)
    return;

  value = ENTRY_VALUE(value);    // Constrain to valid range
  
  if(!logChannels[ch].tick && value == logChannels[ch].value)
    // Repeat value, not stored
    
    return;
            
  if(ch == prevCh) {
    // Same channel as previous, store as delta
    
    logEnter(ENTRY_TOKEN(t_delta) | (DELTA_MASK & ((value - logChannels[ch].value)>>1)));
    
  } else if(prevCh > -1 && ch == prevCh + 1) {
    // Channel prediction good, just store value
    
    logEnter(value);
  } else {
    // Set channel first
    
    uint16_t buffer[2] = { ENTRY_TOKEN(t_channel + ch), value };
    logEnter(buffer, 2);
  }
    
  logChannels[ch].value = value;
  prevCh = ch;
}

void logGeneric(int ch, float value)
{
  float small = logChannels[ch].small, large = logChannels[ch].large;
  
  logWithCh(ch, (uint16_t) ((float) VALUE_MASK*clamp((value-small)/(large-small), 0, 1)));
}

void logMark(void)
{
  logEnter(ENTRY_TOKEN(t_mark));
}

void logEnable()
{
  if(logEnabled)
    return;
    
  logEnabled = true;
  
  for(int i = 0; i < l_channels; i++)
    logChannels[i].value = TOKEN_MASK;
  
  prevCh = -1;  
  
  consoleNoteLn("Logging ENABLED");
}

void logDisable()
{
  if(!logEnabled)
    return;
    
  logMark();
  logEnabled = false;
  
  consoleNoteLn("Logging DISABLED");
}

int col;
  
static void logPrintValue(float v)
{
  float av = abs(v);
  
  if(col > 72) {
    consolePrintLn("");
    col = 0;
  }
  
  if(av < 0.001) {
    col++;
    consolePrint(0);
    } else if(abs(av - 1) < 0.001){
    consolePrint(v < 0.0 ? -1 : 1);
    col += v < 0.0 ? 2 : 1;
    } else {
      int decimals = av < 1 ? 3 : av < 10 ? 2 : av < 100 ? 1 : 0;
    consolePrint(v, decimals);
    col += 3 + (v < 0.0 ? 1 : 0) + (decimals > 0 ? 1 : 0) + (av < 1.0 ? 1 : 0)
      + (av >= 1000.0 ? 1 : 0) + (av >= 10000.0 ? 1 : 0);
  }
  
  col++; // account for the comma
}

void logDump(int ch)
{
  if(!logReady())
    return;
  
  if(ch < 0) {
    for(ch = 0; ch < l_channels; ch++)
      logDump(ch);

    consolePrint("fdr_");
    consolePrint(stateRecord.logStamp);
    consolePrint("_matrix = [ ");

    for(ch = 0; ch < l_channels; ch++) {
      if(ch > 0) 
          consolePrint("; ");

      consolePrint("fdr_");
      consolePrint(stateRecord.logStamp);
      consolePrint("_");
      consolePrint(logChannels[ch].name);
    }
    
    consolePrint(" ]\n");

    consoleNoteLn("FLIGHT DATA RECORD WITH INDEX");
    
    consolePrint("fdr_");
    consolePrint(stateRecord.logStamp);
    consolePrint(" = { fdr_");
    consolePrint(stateRecord.logStamp);
    consolePrint("_matrix, ");
    
    for(ch = 0; ch < l_channels; ch++) {
      consolePrint("\"");
      consolePrint(logChannels[ch].name);
      consolePrint("\"");
      
      if(ch < l_channels-1)
        consolePrint(", ");
    }
    
    consolePrintLn(" }");
    
    return;
  }
  
  int32_t len = logLen, notFirst = 0;

  if(len < 0) {
    consoleNote("Looking for log start...");
    
    len = 0;
    
    while(len < logSize-1 && logRead(logIndex(-len-1)) != ENTRY_TOKEN(t_start)) {
      if(len % 5000 == 0)
        consolePrint(".");
      len++;
    }
        
    consolePrint(" found, log length = ");
    consolePrintLn(len);
  
    logLen = len;
  }
  
  consoleNote("CHANNEL ");
  consolePrint(ch);
  consolePrint(" (");
  consolePrint(logChannels[ch].name);
  consolePrintLn(") DATA");
  
  consolePrint("fdr_");
  consolePrint(stateRecord.logStamp);
  consolePrint("_");
  consolePrint(logChannels[ch].name);
  consolePrint(" = [ ");

  float small = logChannels[ch].small, large = logChannels[ch].large;
  int currentCh = -1, nextCh = -1;
  uint16_t valueRaw = 0;
  boolean valueValid = false;
  float value = 0.0;

  col = 20;
    
  for(int32_t i = 0; i < logLen; i++) {
    uint16_t entry = logRead(logIndex(-logLen+i));

    if(ENTRY_IS_TOKEN(entry) && ENTRY_VALUE(entry) < t_delta) {
      LogToken_t token = (LogToken_t) ENTRY_VALUE(entry);
      
      switch(token) {
        case t_stamp:
          // End marker, a count follows, ignore both
          i++;
          break;
          
        case t_start:
          break;
          
        case t_mark:
          // Mark
                      
          for(int j = 0; j < 5; j++) {
            if(notFirst)
              consolePrint(",");
            logPrintValue(large); 
            consolePrint(",");
            logPrintValue(small);
            notFirst++;
          }
          break;
          
        default:
          if(token >= t_channel && token < t_channel+l_channels) {
            // Valid channel id
      
            nextCh = token - t_channel;
          } else {
            // Invalid token
      
            if(notFirst)
              consolePrint(",");
            consolePrint("\n -360 // *** Invalid entry ");
            consolePrintLn(entry);
            notFirst++;
            break;
          }
      }
    } else {
      // A log value entry

      if(!ENTRY_IS_TOKEN(entry))
        currentCh = nextCh;
      
      if(logChannels[currentCh].tick) {
        if(notFirst)
          consolePrint(",");
            
        if(valueValid)
          logPrintValue(value);
        else
          logPrintValue((notFirst & 1) ? small : large);
            
        notFirst++;
      }

      if(currentCh == ch) {
        if(ENTRY_IS_TOKEN(entry)) {
          // Delta value
                  
          valueRaw = ENTRY_VALUE(valueRaw + ((ENTRY_VALUE(entry) - t_delta) << 1));
                    
        } else {
          // Absolute value
                    
          valueRaw = entry;
        }
        
        value = (float) (large - small) * valueRaw / VALUE_MASK + small;
        valueValid = true;
      }

      if(currentCh > -1)
        nextCh = currentCh + 1;
    }
  }

  consolePrintLn(" ]");
}

#define WORD6_CHAR(v, s)  (' ' + (((v)>>(s*6)) & 0x3F))

void logDumpBinary(void)
{
  int32_t len = 0;

  consoleNote("Looking for log start... ");

  while(len < logSize-1 && logRead(logIndex(-len-1)) != ENTRY_TOKEN(t_start))
    len++;
  
  consolePrint("found, log length = ");
  consolePrintLn(len);
  
  int lineLen = 0;
  
  uint16_t buf[3];
  int count = 0;
  
  for(int32_t i = 0; i < len; i++) {
    buf[count++] = logRead(logIndex(-len+i));
    
    if(count == 3) {
      uint64_t tmp = *((uint64_t*) buf);
      
      char string[] = {
        WORD6_CHAR(tmp, 0),      
        WORD6_CHAR(tmp, 1),      
        WORD6_CHAR(tmp, 2),      
        WORD6_CHAR(tmp, 3),      
        WORD6_CHAR(tmp, 4),      
        WORD6_CHAR(tmp, 5),      
        WORD6_CHAR(tmp, 6),      
        WORD6_CHAR(tmp, 7),     
        '\0' };
        
      consolePrint(string);
    
      lineLen += 8;
      if(lineLen >= 72) {
        consolePrintLn("");
        lineLen = 0;
      }
      
      count = 0;
    }
  }
}

boolean logInit(long maxDuration)
{
  long current = micros();
  static int32_t endPtr = -1, searchPtr = 0;
  static boolean endFound = false;
  uint32_t eepromSize = 0;
  uint8_t dummy;
  
  switch(logState) {
  case invalid_c:
    logSize = 0;
    
    while(!readEEPROM(eepromSize, &dummy, 1))
      eepromSize += 1<<10;
    
    if(!readEEPROM(eepromSize-1, &dummy, 1)) {
      logSize = eepromSize/sizeof(uint16_t);

      consoleNote("Inferred log size = ");
      consolePrint(logSize/(1<<10));
      consolePrintLn("k entries");
      
      logState = init_c;
      logLen = -1;

        return false;
    } else {
      consoleNoteLn("Log EEPROM failed");
      logState = failed_c;
    }
    break;

  case init_c:
    while(searchPtr < logSize) {
      if(logRead(searchPtr) == ENTRY_TOKEN(t_stamp)) {
        uint16_t stamp = logRead(logIndex(searchPtr+1));

        if(stamp % 500 == 0) {
          consoleNote("  Searching for log end at ");
          consolePrint(searchPtr);
          consolePrintLn("...");
        }
        
        if(endFound && stamp != ENTRY_VALUE(logEndStamp+1))
          break;
            
        endPtr = searchPtr;
        endFound = true;
        logEndStamp = stamp;

        searchPtr++;
      }
      
      searchPtr++;
      
      if(micros() - current > maxDuration)
        // Stop for now
        return false;
    }

    logState = stop_c;
      
    if(endFound && endPtr < logSize) {
      logPtr = endPtr;
    
      consoleNote("End of log found at ");
      consolePrint(logPtr);
      consolePrint(", stamp = ");
      consolePrintLn(logEndStamp);
    } else {
      consoleNoteLn("*** The log is corrupt and is being cleared ***");
      logEndStamp = 0;
      logPtr = logSize-1;  
      logClear();
    }
    break;
  }

  return true;
}

void logSave(void (*logStartCB)())
{
  switch(logState) {
    case stop_c:
      if(logEnabled) {
        consoleNoteLn("Logging STARTED");
      
        logState = run_c;
    
        for(int i = 0; i < 4; i++)
          logMark();

	(*logStartCB)();
      }
      break;
      
    case run_c:
      if(!logEnabled) {
        consoleNoteLn("Logging STOPPED");
        logState = stop_c;
      } else if(logDirty) {
        logDirty = false;
        logCommit(2);
        logEndStamp = ENTRY_VALUE(logEndStamp + 1);
      }
      break;
  }
}
