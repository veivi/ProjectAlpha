#ifndef NVSTATE_H
#define NVSTATE_H

#include <Arduino.h>

#define MAX_MODELS   4

// Parameters and non-volatile state

struct ParamRecord {
  uint16_t crc;
  uint8_t i2c_5048B, i2c_24L256;
  uint8_t clk_5048B, clk_24L256;
  uint16_t alphaRef;
  float elevDefl, aileDefl;
  float elevZero, aileZero;
  float elevNeutral, aileNeutral;
  float alphaMin, alphaMax, alphaNeutral;
  float i_Kp, i_Ki, i_Kd, o_P;
  float s_Kp, s_Ki, s_Kd;
  int filtLen;
  float flapNeutral, flapStep;
  float brakeNeutral, brakeDefl;
};

struct NVStateRecord {
  uint16_t crc;
  int logStamp;
  int model;
  boolean logRPM;
  int testChannel;
};

extern struct ParamRecord paramRecord[];
extern struct NVStateRecord stateRecord;

void readParams(void);
void storeParams(void);
void readNVState(void);
void storeNVState(void);

#endif
