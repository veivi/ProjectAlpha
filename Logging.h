#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include "Storage.h"
#include "Console.h"

typedef enum {  l_alpha, 
                l_dynpressure, 
                l_acc, 
                l_roll, 
                l_rollrate, 
                l_pitch, 
                l_pitchrate, 
                l_heading, 
                l_ailestick, 
                l_elevstick, 
                l_aileron, 
                l_elevator, 
                l_mode, 
                l_target, 
                l_trim, 
                l_gain, 
                l_test, 
                l_rpm,
                l_speed,
                l_track,
                l_altgps, 
                l_altbaro, 
                l_channels } ChannelId_t;

extern boolean logEnabled;

boolean logInit(long);
void logClear(void);
void logDump(int ch);
void logDumpBinary(void);

void logGeneric(int ch, float value);
void logMark();

void logEnable();
void logDisable();

void logSave(void (*logStartCB)());
  
#endif
