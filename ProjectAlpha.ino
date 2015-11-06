#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <EEPROM.h>
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Sensors.h"
#include "IMU.h"
#include "Filter.h"
#include "Console.h"
#include "Controller.h"
#include "NewI2C.h"
#include "Storage.h"
#include "Interrupt.h"
#include "RxInput.h"
#include "PPM.h"
#include "Logging.h"
#include "NVState.h"

// #define MEGAMINI

NewI2C I2c = NewI2C();

uint8_t nestCount;

struct ModeRecord {
  boolean sensorFailSafe;
  boolean rxFailSafe;
  boolean autoStick;
  boolean autoAlpha;
  boolean autoTrim;
  boolean stabilizer;
  boolean wingLeveler;
  boolean bankLimiter;
};

struct ModeRecord mode;
boolean testMode = false;
float testGain = 0;
boolean calibrate, switchState = false, switchStateLazy = false, echoEnabled = true;
boolean iasFailed = false, iasWarn = false, alphaFailed = false, alphaWarn = false, pciWarn = false;
boolean calibrateStart = false, calibrateStop = false;
float controlCycle = 5.0;
boolean rxElevatorAlive = true, rxAileronAlive = true, rpmAlive = 0;
const int cycleTimeWindow = 31;
float cycleTimeStore[cycleTimeWindow];
int cycleTimePtr = 0, cycleTimesValid = 0;
float cycleMin = -1.0, cycleMax = -1.0, cycleMean = -1.0, cycleCum = -1.0;
const float tau = 0.1;
float dynPressure, alpha, aileStick, elevStick, aileStickRaw, elevStickRaw;
RunningAvgFilter aileFilter, elevFilter;
float controlCycleEnded;
int initCount = 5;
boolean armed = false, talk = true;
float neutralStick = 0.0, neutralAlpha, targetAlpha;
float switchValue, tuningKnobValue, rpmOutput;
Controller elevController, aileController, pusher;
float autoAlphaP, maxAlpha;
float acc,altitude,  heading, rollAngle, pitchAngle, rollRate, pitchRate;
int cycleTimeCounter = 0;
Median3Filter cycleTimeFilter;
boolean cycleTimesDone = false;
float prevMeasurement;
float parameter;  

RunningAvgFilter alphaFilter;
AlphaBuffer alphaBuffer, pressureBuffer;

float elevOutput = 0, aileOutput = 0, flapOutput = 0, gearOutput = 1, brakeOutput = 0;
  
struct GPSFix {
  float altitude;
  float track;
  float lat;
  float lon;
  float speed;
};

struct GPSFix gpsFix;

#define BAUDRATE 115200

#define INT_EEPROM_SIZE (1<<12)

#define buttonPin 35

#ifdef MEGAMINI

#include "PWMOutput.h"

#define ppmInputPin 48

struct RxInputRecord aileInput, elevInput, switchInput, tuningKnobInput;

struct RxInputRecord *ppmInputs[] = 
{ &aileInput, &elevInput, NULL, NULL, &switchInput, &tuningKnobInput };

struct HWTimer hwTimer1 =
       { &TCCR1A, &TCCR1B, &ICR1, { &OCR1A, &OCR1B, &OCR1C } };
struct HWTimer hwTimer3 =
       { &TCCR3A, &TCCR3B, &ICR3, { &OCR3A, &OCR3B, &OCR3C } };
struct HWTimer hwTimer4 =
       { &TCCR4A, &TCCR4B, &ICR4, { &OCR4A, &OCR4B, &OCR4C } };

struct PWMOutput pwmOutput[] = {
       { 12, &hwTimer1, COMnB },
       { 11, &hwTimer1, COMnA },
       { 8, &hwTimer4, COMnC },
       { 7, &hwTimer4, COMnB },
       { 6, &hwTimer4, COMnA },
       { 3, &hwTimer3, COMnC },
       { 2, &hwTimer3, COMnB },
       { 5, &hwTimer3, COMnA } };

void *aileHandle = (void*) &pwmOutput[0];
void *elevatorHandle = (void*) &pwmOutput[1];
void *flapHandle = (void*) &pwmOutput[2];
void *gearHandle = (void*) &pwmOutput[3];
void *brakeHandle = (void*) &pwmOutput[4];

void servoOutput(void *handle, uint16_t value)
{
  pwmOutputWrite((struct PWMOutput*) handle, value);
}

#else

#define aileRxPin	A8
#define elevatorRxPin 	A9
#define switchPin 	A11
#define tuningKnobPin 	A13

struct RxInputRecord aileInput = { aileRxPin }; 
struct RxInputRecord elevInput = { elevatorRxPin };
struct RxInputRecord switchInput = { switchPin };
struct RxInputRecord tuningKnobInput = { tuningKnobPin };

struct RxInputRecord *rxInputs[8] =
  { &aileInput, &elevInput, NULL, &switchInput, NULL, &tuningKnobInput };

#include <Servo.h>

#define aileServoPin		2
#define elevatorServoPin  	3
#define flapServoPin		5
#define gearServoPin  		6
#define brakeServoPin  		7

Servo elevatorServo, aileServo, flapServo, gearServo, brakeServo;  

void *elevatorHandle = (void*) &elevatorServo;
void *aileHandle = (void*) &aileServo;
void *flapHandle = (void*) &flapServo;
void *gearHandle = (void*) &gearServo;
void *brakeHandle = (void*) &brakeServo;

void servoOutput(void *handle, uint16_t value)
{
  ((Servo*) handle)->writeMicroseconds(value);
}

#endif

#define rpmPin 		A14

struct RxInputRecord rpmInput = { rpmPin };

#define minAlpha paramRecord[stateRecord.model].alphaMin

// #define min(a,b) ((a) < (b) ? (a) : (b))
// #define max(a,b) ((a) > (b) ? (a) : (b))

void printParams(struct ParamRecord *p)
{
  consoleNote("  24L256 addr = ");
  consolePrint(p->i2c_24L256);
  consolePrint(" clk div = ");
  consolePrintLn(p->clk_24L256);
  consoleNote("  AS5048B addr = ");
  consolePrint(p->i2c_5048B);
  consolePrint(" ref = ");
  consolePrint(p->alphaRef);
  consolePrint(" clk div = ");
  consolePrintLn(p->clk_5048B);
  consoleNoteLn("  Autostick/pusher");
  consoleNote("    Inner P = ");
  consolePrint(p->i_Kp, 4);
  consolePrint(" I = ");
  consolePrint(p->i_Ki, 4);;
  consolePrint(" D = ");
  consolePrint(p->i_Kd, 4);
  consolePrint(" Outer P = ");
  consolePrintLn(p->o_P, 4);
  consoleNoteLn("  Stabilizer");
  consoleNote("    P = ");
  consolePrint(p->s_Kp, 4);
  consolePrint(" I = ");
  consolePrint(p->s_Ki, 4);
  consolePrint(" D = ");
  consolePrintLn(p->s_Kd, 4);
  consoleNote("  Alpha min = ");
  consolePrint(p->alphaMin*360);
  consolePrint("  max = ");
  consolePrintLn(p->alphaMax*360);
  consoleNoteLn("  Elevator");
  consoleNote("    deflection = ");
  consolePrint(p->elevDefl*90);
  consolePrint(" neutral = ");
  consolePrintLn(p->elevNeutral*90);
  consoleNoteLn("  Aileron");
  consoleNote("    deflection = ");
  consolePrint(p->aileDefl*90);
  consolePrint(" neutral = ");
  consolePrintLn(p->aileNeutral*90);
}

void dumpParams(struct ParamRecord *p)
{
  consolePrint("24l256_addr ");
  consolePrint(p->i2c_24L256);
  consolePrint("; 24l256_clk ");
  consolePrint(p->clk_24L256);
  consolePrint("; 5048b_addr ");
  consolePrint(p->i2c_5048B);
  consolePrint("; 5048b_clk ");
  consolePrint(p->clk_5048B);
  consolePrint("; 5048b_ref ");
  consolePrint(p->alphaRef);
  consolePrint("; inner_pid "); consolePrint(p->i_Kp, 4); consolePrint(" "); consolePrint(p->i_Ki, 4);  consolePrint(" "); consolePrint(p->i_Kd, 4);
  consolePrint("; outer_p "); consolePrint(p->o_P);
  consolePrint("; stabilizer_pid "); consolePrint(p->s_Kp, 4); consolePrint(" "); consolePrint(p->s_Ki, 4);  consolePrint(" "); consolePrint(p->s_Kd, 4);
  consolePrint("; min "); consolePrint(p->alphaMin*360);
  consolePrint("; max "); consolePrint(p->alphaMax*360);
  consolePrint("; edefl "); consolePrint(p->elevDefl*90);
  consolePrint("; eneutral "); consolePrint(p->elevNeutral*90);
  consolePrint("; ezero "); consolePrint(p->elevZero*90);
  consolePrint("; adefl "); consolePrint(p->aileDefl*90);
  consolePrint("; aneutral "); consolePrint(p->aileNeutral*90);
  consolePrint("; azero "); consolePrint(p->aileZero*90);
  consolePrint("; fstep "); consolePrint(p->flapStep*90);
  consolePrint("; fneutral "); consolePrint(p->flapNeutral*90);
  consolePrint("; bdefl "); consolePrint(p->brakeDefl*90);
  consolePrint("; bneutral "); consolePrint(p->brakeNeutral*90);
  consolePrint("; filtlen "); consolePrint(p->filtLen);
}

const uint8_t addr5048B_c = 0x40;
const uint8_t addr4525_c = 0x28;

boolean read5048B(uint8_t addr, uint8_t *storage, int bytes) 
{
  return I2c.read(addr5048B_c, addr, storage, bytes) == 0;
}

boolean read5048B_byte(uint8_t addr, byte *result)
{
  return read5048B(addr, result, sizeof(*result));
}

boolean read5048B_word14(uint8_t addr, uint16_t *result)
{
  uint8_t buf[sizeof(uint16_t)];
  boolean success = false;
  
  success = read5048B(addr, buf, sizeof(buf));
  
  if(success)
    *result = ((((uint16_t) buf[0]) << 6) + (buf[1] & 0x3F))<<2;

  return success;
}

boolean read4525DO(uint8_t *storage, int bytes) 
{
  return I2c.read(addr4525_c, NULL, 0, storage, bytes) == 0;
}

boolean read4525DO_word14(uint16_t *result)
{
  uint8_t buf[sizeof(uint16_t)];
  boolean success = false;
  
  success = read4525DO(buf, sizeof(buf));
  
  if(success)
    *result = (((uint16_t) (buf[0] & 0x3F)) << 8) + buf[1];

  return success && (buf[0]>>6) == 0;
}

float decodePWM(float pulse) {
  const float txRange = 0.81;
  return (pulse - 1500)/500.0/txRange;
}

void logAlpha(void)
{
  logGeneric(l_alpha, alpha*360);
}

void logConfig(void)
{
  logGeneric(l_mode, 
    (mode.rxFailSafe ? 32 : 0) 
    + (mode.sensorFailSafe ? 16 : 0) 
    + (mode.wingLeveler ? 8 : 0) 
    + (mode.bankLimiter ? 4 : 0) 
    + (mode.autoStick ? 2 : 0) 
    + (mode.autoAlpha ? 1 : 0)); 
    
  logGeneric(l_target, targetAlpha*360);
  logGeneric(l_trim, neutralAlpha*360);

  if(testMode) {
    logGeneric(l_gain, testGain);
    logGeneric(l_test, stateRecord.testChannel);
  } else {
    logGeneric(l_gain, 0);
    logGeneric(l_test, 0);
  }
}

void logPosition(void)
{
  logGeneric(l_speed, gpsFix.speed);
  logGeneric(l_track, gpsFix.track);
  logGeneric(l_altgps, gpsFix.altitude);
  logGeneric(l_altbaro, altitude);
}
  
void logInput(void)
{
  logGeneric(l_ailestick, aileStick);
  logGeneric(l_elevstick, elevStick);
}

void logActuator(void)
{
  logGeneric(l_aileron, aileOutput);
  logGeneric(l_elevator, elevOutput);
}

void logRPM(void)
{
  logGeneric(l_rpm, rpmOutput);
}

void logAttitude(void)
{
  logGeneric(l_dynpressure, dynPressure);
  logGeneric(l_acc, acc);
  logGeneric(l_roll, rollAngle);
  logGeneric(l_rollrate, rollRate*360);
  logGeneric(l_pitch, pitchAngle);
  logGeneric(l_pitchrate, pitchRate*360);
  logGeneric(l_heading, heading);
}

ISR(BADISR_vect)
{
   sei();
   consoleNoteLn("PASKA KESKEYTYS.");

   if(!armed)
     abort();
}

#ifndef MEGAMINI

uint8_t log2Table[1<<8];

ISR(PCINT2_vect)
{
  static uint8_t prevState;
  uint8_t state = PINK, event = (state ^ prevState) & PCMSK2;

  prevState = state;
  
  uint32_t current = micros();
  
  while(event) {
    uint8_t i = log2Table[event];
    uint8_t mask = 1U<<i;
  
    if(!rxInputs[i]) {
      pciWarn = true;
    } else if(rxInputs[i]->freqOnly) {
      rxInputs[i]->pulseCount += (state & mask) ? 1 : 0;
    } else if(state & mask) {
      rxInputs[i]->pulseStart = current;
    } else if(rxInputs[i]->pulseStart > 0) {
      uint32_t width = current - rxInputs[i]->pulseStart;
      rxInputs[i]->pulseWidthAcc += width;
      rxInputs[i]->pulseCount++;      
      rxInputs[i]->alive = true;
    }
    
    event &= ~mask;
  }
}

#endif

void setup() {
  // Serial comms

  Serial.begin(BAUDRATE); 
  consoleNoteLn("Project | Alpha"); 
  
  Serial1.begin(38400); 
  
  // Read the non-volatile state

  readNVState();
    
  consoleNote("Current model is ");
  consolePrintLn(stateRecord.model);
  
  // I2C
  
  I2c.begin();
  I2c.setSpeed(true);
  I2c.pullup(true);
  I2c.timeOut(2+EXT_EEPROM_LATENCY/1000);

  // Param record
  
  readParams();

  for(int i = 0; i < MAX_MODELS; i++) {
    consoleNote("MODEL ");
    consolePrintLn(i);
    printParams(&paramRecord[i]);    
  }

  // Set I2C speed
  
  TWBR = max(paramRecord[stateRecord.model].clk_24L256,
              paramRecord[stateRecord.model].clk_5048B);
              
  // Rx input

#ifdef MEGAMINI
  // PPM input
  
  consoleNoteLn("Initializing PPM receiver");
  
  pinMode(48, INPUT_PULLUP);
  
  ppmInputInit(ppmInputs, sizeof(ppmInputs)/sizeof(struct RxInputRecord*));
  
#else
  consoleNoteLn("Initializing individual PWM inputs");

  pinMode(elevatorRxPin, INPUT_PULLUP);
  pinMode(aileRxPin, INPUT_PULLUP);
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(tuningKnobPin, INPUT_PULLUP);
  pinMode(rpmPin, INPUT_PULLUP);  

  for(int i = 1; i < (1<<8); i++) {
    int j = 7;
    while(((1<<j) & i) == 0 && j > 0)
      j--;
    log2Table[i] = j;
  }

  FORBID;

  PCMSK2 = 0;
  
  for(int i = 0; i < 8; i++) {
    if(rxInputs[i]) {
       rxInputs[i]->index = i;
       PCMSK2 |= 1<<i;
    }
  }
  
  PCICR |= 1<<PCIE2;

  PERMIT;

#endif
  
  aileFilter.setWindowLen(1);
  elevFilter.setWindowLen(1);

  // RPM sensor int control
  
  // rpmMeasure(stateRecord.logRPM);
  
  // Servos

  consoleNoteLn("Initializing servos");

#ifdef MEGAMINI
  pwmOutputInitList(pwmOutput, sizeof(pwmOutput)/sizeof(struct PWMOutput));
#else

  aileServo.attach(aileServoPin);
  elevatorServo.attach(elevatorServoPin);
  flapServo.attach(flapServoPin);
  gearServo.attach(gearServoPin);
  brakeServo.attach(brakeServoPin);

#endif

  //
  
  alphaFilter.setWindowLen(-1);

  // Misc sensors
  
  consoleNote("Initializing sensors...");
  
  initSensors();

  calibratingG = 512;
  calibratingB = 200;

  consolePrintLn(" done");
  
  // Configuration input

  pinMode(buttonPin, INPUT_PULLUP);

  // LED output
  
  LEDPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_PINMODE;
  BUZZERPIN_PINMODE;
}

const int alphaBits = 14;

boolean readSwitch() {
  return decodePWM(switchValue) < 0.0;
}

float readParameter() {
  return (decodePWM(tuningKnobValue) + 1.0) / 2;
}

float readRPM() {
  return rpmOutput;
}

void rpmMeasure(boolean on)
{
#if defined(rpmPin)
  if(on)
    PCMSK2 |= 1<<rpmInput.index;
  else
    PCMSK2 &= ~(1<<rpmInput.index);
#endif
}

#define AS5048_ADDRESS 0x40 
#define AS5048B_PROG_REG 0x03
#define AS5048B_ADDR_REG 0x15
#define AS5048B_ZEROMSB_REG 0x16 //bits 0..7
#define AS5048B_ZEROLSB_REG 0x17 //bits 0..5
#define AS5048B_GAIN_REG 0xFA
#define AS5048B_DIAG_REG 0xFB
#define AS5048B_MAGNMSB_REG 0xFC //bits 0..7
#define AS5048B_MAGNLSB_REG 0xFD //bits 0..5
#define AS5048B_ANGLMSB_REG 0xFE //bits 0..7
#define AS5048B_ANGLLSB_REG 0xFF //bits 0..5
#define AS5048B_RESOLUTION 16384.0 //14 bits

boolean readAlpha_5048B(int16_t *result) {
  uint16_t raw = 0;
  
  if(alphaFailed)
    // Stop trying
    
    return false;
  
  if(!read5048B_word14(AS5048B_ANGLMSB_REG, &raw))
    // Failed
    return false;

  // The value is good, use it

  if(result)
    *result = (int16_t) (raw - paramRecord[stateRecord.model].alphaRef);
  
  return true;
}

boolean readPressure(int16_t *result) 
{
  uint16_t raw = 0;
  
  if(iasFailed)
    // Stop trying
    
    return false;
  
  if(!read4525DO_word14(&raw))
    return false;

//  consoleNote("pressure = ");
//  consolePrintLn(raw);

  if(result)
    *result = (raw - 8176)<<2;
  
  return true;
}

typedef enum { c_, c_adefl, c_edefl, c_clear, c_dump, c_min, c_max, c_zero,
c_eneutral, c_aneutral, c_store, c_report, c_stop, c_cycle,
           c_read, c_write, c_5048b_addr, c_5048b_read, c_start, c_init, c_filtlen,
            c_params, c_defaults, c_reset, c_24l256_addr, c_24l256_clk, c_5048b_clk, c_center,
           c_loop, c_stamp, c_model, c_alpha, c_flapneutral, c_flapstep, c_backup, c_echo,
           c_ezero, c_azero, c_5048b_ref, c_bdefl, c_bneutral, c_rpm, c_baud, c_dumpz,
            c_stabilizer_pid, c_stabilizer_pid_zn, c_stabilizer_pi_zn, c_outer_p, 
          c_inner_pid, c_inner_pid_zn, c_inner_pi_zn, c_arm, c_disarm, c_test, c_talk } command_t;

struct command {
  const char *c_string;
  command_t c_token;
};

const struct command commands[] = {
  { "edefl", c_edefl },
  { "adefl", c_adefl },
  { "eneutral", c_eneutral },
  { "aneutral", c_aneutral },
  { "ezero", c_ezero },
  { "azero", c_azero },
  { "zero", c_zero },
  { "alpha", c_alpha },
  { "max", c_max },
  { "min", c_min },
  { "dump", c_dump },
  { "clear", c_clear },
  { "store", c_store },
  { "report", c_report },
  { "stop", c_stop },
  { "cycle", c_cycle },
  { "read", c_read },
  { "write", c_write },
  { "5048b_addr", c_5048b_addr },
  { "5048b_ref", c_5048b_ref },
  { "24l256_addr", c_24l256_addr },
  { "5048b_read", c_5048b_read },
  { "start", c_start },
  { "init", c_init },
  { "filtlen", c_filtlen },
  { "defaults", c_defaults },
  { "params", c_params },
  { "reset", c_reset },
  { "5048b_clk", c_5048b_clk },
  { "24l256_clk", c_24l256_clk },
  { "center", c_center },
  { "loop", c_loop },
  { "stamp", c_stamp },
  { "model", c_model },
  { "alpha", c_alpha },
  { "fneutral", c_flapneutral },
  { "fstep", c_flapstep },
  { "backup", c_backup },
  { "echo", c_echo },
  { "bdefl", c_bdefl },
  { "bneutral", c_bneutral },
  { "rpm", c_rpm },
  { "baud", c_baud },
  { "dumpz", c_dumpz },
  { "stabilizer_pid", c_stabilizer_pid },
  { "stabilizer_pid_zn", c_stabilizer_pid_zn },
  { "stabilizer_pi_zn", c_stabilizer_pi_zn },
  { "outer_p", c_outer_p },
  { "inner_pid", c_inner_pid },
  { "inner_pid_zn", c_inner_pid_zn },
  { "inner_pi_zn", c_inner_pi_zn },
  { "arm", c_arm },
  { "disarm", c_disarm },
  { "test", c_test },
  { "cycle", c_cycle },
  { "talk", c_talk },
  { "", c_ }
};

boolean looping;
    
const int maxCmdLen = 100;
char cmdBuf[maxCmdLen];
int cmdBufLen;

int indexOf(const char *s, const char c, int index)
{
  while(s[index] != '\0') {
    if(s[index] == c)
      return index;
  
    index++;
  }
    
  return -1;
}

int indexOf(const char *s, const char c)
{
  return indexOf(s, c, 0);
}
    
void executeCommand(const char *cmdBuf, int cmdBufLen)
{
  if(echoEnabled) {
    consoleNote("% ");
    consolePrintLn(cmdBuf);  
  }
  
  const int maxParams = 8;

  int index = 0, prevIndex = 0, numParams = 0, tokenLen = cmdBufLen;
  float param[maxParams];

  for(int i = 0; i < maxParams; i++)
    param[i] = 0.0;

  if((index = indexOf(cmdBuf, ' ')) > 0) {
    tokenLen = index;
    
    do {
      prevIndex = index;
    
      index = indexOf(cmdBuf, ' ', index+1);
      
      if(index < 0)
        index = cmdBufLen;
        
      float value = 0.0, fract = 0.0;
      int exponent = 0;
      boolean sign = false, deci = false;

      for(int i = prevIndex+1; i < index; i++) {
        char c = cmdBuf[i];

        switch(c) {
          case '-':
            sign = true;
            break;
            
          case '.':
            deci = true;
            break;
            
          case '0':
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
          case '8':
          case '9':
            if(deci) {
              value += (float) (c - '0') / pow(10, ++exponent);
            } else {
               value *= (float) 10;
               value += (float) (c - '0');
            }
            break;
            
          default:
            consoleNote("weird char = ");
            consolePrintLn(c);
        }
      }
            
      if(sign)
        value = -value;
    
      if(numParams < maxParams)
        param[numParams++] = value;
    } while(index < cmdBufLen);
  }
  
  int j = 0;
  
  while(commands[j].c_token != c_) {
    if(!strncmp(cmdBuf, commands[j].c_string, tokenLen))
      break;
    j++;
  }

  switch(commands[j].c_token) {
  case c_arm:
    armed = true;
    break;
    
  case c_disarm:
    armed = false;
    break;
    
  case c_talk:
    talk = true;
    consoleNoteLn("Hello world");
    break;
    
  case c_test:
    if(numParams > 0)
      stateRecord.testChannel = param[0];
    else {
      consoleNote("Current test channel = ");
      consolePrintLn(stateRecord.testChannel);
    }
    break;
    
  case c_stabilizer_pid:
    if(numParams > 0)
      paramRecord[stateRecord.model].s_Kp = param[0];
    if(numParams > 1)
      paramRecord[stateRecord.model].s_Ki = param[1];
    if(numParams > 2)
      paramRecord[stateRecord.model].s_Kd = param[2];

    aileController.setPID(paramRecord[stateRecord.model].s_Kp, paramRecord[stateRecord.model].s_Ki, paramRecord[stateRecord.model].s_Kd);

    consoleNote("Stabilizer P = ");
    consolePrint(paramRecord[stateRecord.model].s_Kp);
    consolePrint(", I = ");
    consolePrint(paramRecord[stateRecord.model].s_Ki);
    consolePrint(", D = ");
    consolePrintLn(paramRecord[stateRecord.model].s_Kd);    
    break;
    
  case c_stabilizer_pid_zn:
    if(numParams > 1) {
      consoleNote("Stabilizer Z-N PID Ku, Tu = ");
      consolePrint(param[0]);
      consolePrint(", ");
      consolePrintLn(param[1]);
    
      aileController.setZieglerNicholsPID(param[0], param[1]);

      paramRecord[stateRecord.model].s_Kp = aileController.getP();
      paramRecord[stateRecord.model].s_Ki = aileController.getI();
      paramRecord[stateRecord.model].s_Kd = aileController.getD();

      consoleNote("  Resulting P = ");
      consolePrint(paramRecord[stateRecord.model].s_Kp);
      consolePrint(", I = ");
      consolePrint(paramRecord[stateRecord.model].s_Ki);
      consolePrint(", D = ");
      consolePrintLn(paramRecord[stateRecord.model].s_Kd);
    } else {
      float Ku, Tu;
      aileController.getZieglerNicholsPID(&Ku, &Tu);
      consoleNote("Current stabilizer ");
      consolePrint(aileController.getD() > 0.0 ? "PID" : "PI");
      consolePrint(" Ku, Tu = ");
      consolePrint(Ku, 4);
      consolePrint(", ");
      consolePrintLn(Tu, 4);      
    }
    break;
    
  case c_stabilizer_pi_zn:
    if(numParams > 1) {
      consoleNote("Stabilizer Z-N PI Ku, Tu = ");
      consolePrint(param[0]);
      consolePrint(", ");
      consolePrintLn(param[1]);
    
      aileController.setZieglerNicholsPI(param[0], param[1]);

      paramRecord[stateRecord.model].s_Kp = aileController.getP();
      paramRecord[stateRecord.model].s_Ki = aileController.getI();
      paramRecord[stateRecord.model].s_Kd = aileController.getD();

      consoleNote("  Resulting P = ");
      consolePrint(paramRecord[stateRecord.model].s_Kp);
      consolePrint(", I = ");
      consolePrint(paramRecord[stateRecord.model].s_Ki);
      consolePrint(", D = ");
      consolePrintLn(paramRecord[stateRecord.model].s_Kd);
    }
    break;
    
  case c_inner_pid:
    if(numParams > 0)
      paramRecord[stateRecord.model].i_Kp = param[0];
    if(numParams > 1)
      paramRecord[stateRecord.model].i_Ki = param[1];
    if(numParams > 2)
      paramRecord[stateRecord.model].i_Kd = param[2];

    consoleNote("Autostick/Pusher inner P = ");
    consolePrint(paramRecord[stateRecord.model].i_Kp);
    consolePrint(", I = ");
    consolePrint(paramRecord[stateRecord.model].i_Ki);
    consolePrint(", D = ");
    consolePrintLn(paramRecord[stateRecord.model].i_Kd);
    
    elevController.setPID(paramRecord[stateRecord.model].i_Kp, paramRecord[stateRecord.model].i_Ki, paramRecord[stateRecord.model].i_Kd);
    pusher.setPID(paramRecord[stateRecord.model].i_Kp, paramRecord[stateRecord.model].i_Ki, paramRecord[stateRecord.model].i_Kd);
    break;
    
  case c_inner_pid_zn:
    if(numParams > 1) {
      consoleNote("Autostick/Pusher inner Z-N PID Ku, Tu = ");
      consolePrint(param[0]);
      consolePrint(", ");
      consolePrintLn(param[1]);
    
      elevController.setZieglerNicholsPID(param[0], param[1]);
      pusher.setZieglerNicholsPID(param[0], param[1]);

      paramRecord[stateRecord.model].i_Kp = elevController.getP();
      paramRecord[stateRecord.model].i_Ki = elevController.getI();
      paramRecord[stateRecord.model].i_Kd = elevController.getD();

      consoleNote("  Resulting P = ");
      consolePrint(paramRecord[stateRecord.model].i_Kp);
      consolePrint(", I = ");
      consolePrint(paramRecord[stateRecord.model].i_Ki);
      consolePrint(", D = ");
      consolePrintLn(paramRecord[stateRecord.model].i_Kd);
    } else {
      float Ku, Tu;
      elevController.getZieglerNicholsPID(&Ku, &Tu);
      consoleNote("Current autostick ");
      consolePrint(elevController.getD() > 0.0 ? "PID" : "PI");
      consolePrint(" Ku, Tu = ");
      consolePrint(Ku, 4);
      consolePrint(", ");
      consolePrintLn(Tu, 4);      
    }
    break;
    
  case c_inner_pi_zn:
    if(numParams > 1) {
      consoleNote("Autostick/Pusher inner Z-N PI Ku, Tu = ");
      consolePrint(param[0]);
      consolePrint(", ");
      consolePrintLn(param[1]);
    
      elevController.setZieglerNicholsPI(param[0], param[1]);
      pusher.setZieglerNicholsPI(param[0], param[1]);

      paramRecord[stateRecord.model].i_Kp = elevController.getP();
      paramRecord[stateRecord.model].i_Ki = elevController.getI();
      paramRecord[stateRecord.model].i_Kd = elevController.getD();

      consoleNote("  Resulting P = ");
      consolePrint(paramRecord[stateRecord.model].i_Kp);
      consolePrint(", I = ");
      consolePrint(paramRecord[stateRecord.model].i_Ki);
      consolePrint(", D = ");
      consolePrintLn(paramRecord[stateRecord.model].i_Kd);
    }
    break;
    
  case c_outer_p:
    if(numParams > 0)
      autoAlphaP = paramRecord[stateRecord.model].o_P = param[0];

    consoleNote("Autostick outer P = ");
    consolePrintLn(paramRecord[stateRecord.model].o_P);
    break;
    
  case c_edefl:
    if(numParams > 0)
      paramRecord[stateRecord.model].elevDefl = param[0] / 90.0;
    break;
    
  case c_adefl:
    if(numParams > 0)
      paramRecord[stateRecord.model].aileDefl = param[0] / 90.0;
    break;
    
  case c_bdefl:
    if(numParams > 0)
      paramRecord[stateRecord.model].brakeDefl = param[0] / 90.0;
    break;
    
  case c_ezero:
    if(numParams > 0)
      paramRecord[stateRecord.model].elevZero = param[0] / 90.0;
    break;
    
  case c_azero:
    if(numParams > 0)
      paramRecord[stateRecord.model].aileZero = param[0] / 90.0;
    break;
    
  case c_flapneutral:
    if(numParams > 0)
      paramRecord[stateRecord.model].flapNeutral = param[0] / 90.0;
    break;
    
  case c_flapstep:
    if(numParams > 0)
      paramRecord[stateRecord.model].flapStep = param[0] / 90.0;
    break;
    
  case c_min:
    if(numParams > 0)
      paramRecord[stateRecord.model].alphaMin = param[0]/360.0;
    break;

  case c_max:
    if(numParams > 0)
      maxAlpha = paramRecord[stateRecord.model].alphaMax = param[0]/360.0;
    break;

  case c_zero:
    paramRecord[stateRecord.model].alphaRef += (int16_t) ((1L<<16) * alpha);
    break;

  case c_5048b_ref:
    if(numParams > 0)
      paramRecord[stateRecord.model].alphaRef = (int16_t) param[0];
    break;

  case c_alpha:
    if(numParams > 0)
      paramRecord[stateRecord.model].alphaRef += (int16_t) ((1L<<16) * (alpha - (float) param[0] / 360));
    break;

  case c_loop:
    looping = true;
    rpmMeasure(true);
    break;
    
  case c_store:
    consoleNoteLn("Params & NV state stored");
    storeParams();
    storeNVState();
    break;

  case c_dump:
    consoleNoteLn("Log contents:");
    if(numParams > 0)
      logDump(param[0]);
    else
      logDump(-1);
    break;
    
  case c_dumpz:
    consoleNoteLn("Compressed log contents:");
    logDumpBinary();
    break;
    
  case c_backup:
    consoleNoteLn("Param backup");
    for(int i = 0; i < MAX_MODELS; i++) {
      consolePrintLn("//");
      consoleNote("MODEL ");
      consolePrintLn(i);
      consolePrintLn("//");
      consolePrintLn("");
      consolePrint("echo 0; model ");
      consolePrint(i);
      consolePrint("; ");
      dumpParams(&paramRecord[i]);
      consolePrintLn("; echo 1; store");
      consolePrintLn("");
    }
    break;

  case c_stamp:
    if(numParams > 0) {
      consoleNote("Log stamp set to ");
      consolePrintLn(param[0]);  
      stateRecord.logStamp = param[0];
      storeNVState();
    } else {
      consoleNote("Current log stamp is ");
      consolePrintLn(stateRecord.logStamp);  
    }
    break;

  case c_model:
    if(numParams < 1) {
      consoleNote("Current model is ");
      consolePrintLn(stateRecord.model); 
    } else { 
      if(param[0] > MAX_MODELS-1)
        param[0] = MAX_MODELS-1;
      consoleNote("Model set to ");
      consolePrintLn(param[0]);  
      stateRecord.model = param[0];
      storeNVState();
    }
    break;

  case c_echo:
    if(numParams > 0 && param[0] < 1.0) 
      echoEnabled = false;
    else {
      consoleNoteLn("Echo enabled");
      echoEnabled = true;
    }
    break;
    
  case c_init:
//    logEndStamp = ENTRY_VALUE(-100);
//    logPtr = logSize - 1;

  case c_clear:
    logClear();
    cycleMin = cycleMax = cycleCum = cycleMean = -1;
    cycleTimesValid = false;
    cycleTimePtr = 0;
    break;

  case c_stop:
    logDisable();
    break;

  case c_start:
    logEnable();
    break;

  case c_cycle:
    cycleTimeCounter = 0;
    cycleTimesDone = false;
    break;

  case c_report:
    consoleNote("Alpha = ");
    consolePrint(360*alpha);
    if(alphaFailed)
      consolePrintLn(" FAIL");
    else
      consolePrintLn(" OK");
      
    consoleNoteLn("Cycle time (ms)");
    consoleNote("  median     = ");
    consolePrintLn(controlCycle * 1000.0);
    consoleNote("  min        = ");
    consolePrintLn(cycleMin * 1000.0);
    consoleNote("  max        = ");
    consolePrintLn(cycleMax * 1000.0);
    consoleNote("  mean       = ");
    consolePrintLn(cycleMean * 1000.0);
    consoleNote("  cum. value = ");
    consolePrintLn(cycleCum * 1000.0);
    consoleNote("Warning flags :");
    if(pciWarn)
      consolePrint(" SPURIOUS_PCINT");
    if(alphaWarn)
      consolePrint(" ALPHA_SENSOR");
    if(ppmWarnShort)
      consolePrint(" PPM_SHORT");
    if(ppmWarnSlow)
      consolePrint(" PPM_SLOW");
    if(eepromWarn)
      consolePrint(" EEPROM");
    if(eepromFailed)
      consolePrint(" EEPROM_FAILED");
    if(alphaBuffer.warn)
      consolePrint(" ALPHA_BUFFER");
    if(pusher.warn)
      consolePrint(" PUSHER");
    if(elevController.warn)
      consolePrint(" AUTOSTICK");
      
    consolePrintLn("");

    consoleNote("Log write bandwidth = ");
    consolePrint(logBandWidth);
    consolePrintLn(" bytes/sec");
    break;

  case c_reset:
    pciWarn = alphaWarn = alphaFailed = pusher.warn = elevController.warn
      = alphaBuffer.warn = eepromWarn = eepromFailed = ppmWarnShort
      = ppmWarnSlow = false;
    consoleNoteLn("Warning flags reset");
    break;
    
  case c_5048b_addr:
    paramRecord[stateRecord.model].i2c_5048B = param[0];
    break;
    
  case c_24l256_addr:
    paramRecord[stateRecord.model].i2c_24L256 = param[0];
    break;
    
  case c_5048b_read:
//    consolePrintLn(read5048B_byte((int) param), DEC);
    break;
    
  case c_rpm:
    stateRecord.logRPM = param[0] > 0.5 ? true : false;
    consoleNote("RPM logging ");
    consolePrintLn(stateRecord.logRPM ? "ENABLED" : "DISABLED");
    rpmMeasure(stateRecord.logRPM);
    storeNVState();
    break;
    
   case c_defaults:
//      paramRecord[stateRecord.model] = paramDefaults;
//      consoleNoteLn("Defaults restored");
      break;
      
   case c_params:
      consoleNote("SETTINGS (MODEL ");
      consolePrint(stateRecord.model);
      consolePrintLn(")");
      printParams(&paramRecord[stateRecord.model]);
      break;
  
  case c_5048b_clk:
    paramRecord[stateRecord.model].clk_5048B = param[0];
    break;
    
  case c_24l256_clk:
    paramRecord[stateRecord.model].clk_24L256 = param[0];
    break;
    
  case c_center:
    paramRecord[stateRecord.model].elevZero = elevStickRaw;
    paramRecord[stateRecord.model].aileZero = aileStickRaw;
    break;
    
  case c_eneutral:
    paramRecord[stateRecord.model].elevNeutral = param[0]/90.0;
    break;
    
  case c_aneutral:
    paramRecord[stateRecord.model].aileNeutral = param[0]/90.0;
    break;
    
  case c_bneutral:
    paramRecord[stateRecord.model].brakeNeutral = param[0]/90.0;
    break;
    
  default:
    consolePrintLn("Command not recognized");
    break;
  }
}

void executeCommandSeries(const char *buffer, int len)
{
  int index = 0;
    
  while(index < len) {
    cmdBufLen = 0;
    
    while(index < len && buffer[index] != ';') {
      if(cmdBufLen < maxCmdLen-1 && (buffer[index] != ' ' || cmdBufLen > 0)) {
        cmdBuf[cmdBufLen++] = buffer[index];
      }
      index++;
    }
        
    cmdBuf[cmdBufLen] = '\0';
    
    if(cmdBufLen > 0)
      executeCommand(cmdBuf, cmdBufLen);

    index++;    
  }
}

void annexCode() {} 

#define CONTROL_HZ 100
#define ALPHA_HZ (CONTROL_HZ*6)
#define ACTUATOR_HZ CONTROL_HZ/2
#define TRIM_HZ 10
#define LED_HZ 3
#define LED_TICK 100

struct Task {
  float period;
  float lastExecuted;
  void (*code)(float time);
};

void cacheTask(float currentTime)
{
  cacheFlush();
}

void logStartCallback()
{
  logAlpha();
  logAttitude();
  logInput();
  logActuator();
  logConfig();
  logPosition();
  logRPM();
}

void logSaveTask(float currentTime)
{
  logSave(logStartCallback);
}

void alphaTask(float currentTime)
{
  int16_t raw = 0;
  static int failCount = 0;
  
  if(!handleFailure("alpha", !readAlpha_5048B(&raw), &alphaWarn, &alphaFailed, &failCount))
    alphaBuffer.input((float) raw / (1L<<(8*sizeof(raw))));
}

void airspeedReadTask(float currentTime)
{
  int16_t raw = 0;
  static int failCount = 0;
  
  if(!handleFailure("airspeed", !readPressure(&raw), &iasWarn, &iasFailed, &failCount))
    pressureBuffer.input((float) raw);
}

void airspeedUpdateTask(float currentTime)
{
  const float pascalsPerPSI_c = 6894.7573, range_c = 2*1.1;
  const float factor_c = pascalsPerPSI_c * range_c / (1L<<(8*sizeof(uint16_t)));
    
  dynPressure = pressureBuffer.output() * factor_c;
}

#define NULLZONE 0.075

float applyNullZone(float value)
{
  if(value < -NULLZONE)
    return (value + NULLZONE) / (1.0 - NULLZONE);
  else if(value > NULLZONE)
    return (value - NULLZONE) / (1.0 - NULLZONE);

  return 0.0;
}

void receiverTask(float currentTime)
{
  if(inputValid(&aileInput)) {
    aileStickRaw = aileFilter.input(decodePWM(inputValue(&aileInput)));
    aileStick = applyNullZone(clamp(aileStickRaw - paramRecord[stateRecord.model].aileZero, -1, 1));
  }
  
  if(inputValid(&elevInput)) {
    elevStickRaw = elevFilter.input(decodePWM(inputValue(&elevInput)));
    elevStick = clamp(elevStickRaw - paramRecord[stateRecord.model].elevZero, -1, 1);

    if(mode.autoStick) {
      elevStick = applyNullZone(elevStick - neutralStick);
    }
  }

  if(inputValid(&switchInput))
    switchValue = inputValue(&switchInput);
    
  if(inputValid(&tuningKnobInput))
    tuningKnobValue = inputValue(&tuningKnobInput);
}

void sensorTask(float currentTime)
{
  // Altitude
    
  Baro_update();
  getEstimatedAltitude();

  altitude = (float) alt.EstAlt / 100;

  // Attitude

  computeIMU();

  acc = (float) imu.accSmooth[2] / (1<<9);
  rollRate = (float) imu.gyroData[0] * 2000 / (1<<13) / 360;
  pitchRate = (float) -imu.gyroData[1] * 2000 / (1<<13) / 360;
  rollAngle = (float) att.angle[0] / 10;
  pitchAngle = (float) -att.angle[1] / 10;
  heading = (float) att.heading;
}

const int numPoles = 4;

void rpmTask(float currentTime)
{
  static float prev;

  FORBID;
  
  uint32_t count = rpmInput.pulseCount;
  rpmInput.pulseCount = 0;
  
  PERMIT;
  
  float delta = currentTime - prev;
  
  prev = currentTime;
  
  if(prev > 0)
    rpmOutput = 2.0*60*count/numPoles/delta;
}

void alphaLogTask(float currentTime)
{
  logAlpha();  
}

void controlLogTask(float currentTime)
{
  logAttitude();
  logInput();
  logActuator();
  logConfig();
  logRPM();
}

void positionLogTask(float currentTime)
{
  logPosition();
}

void cycleTimeMonitor(float value)
{
//  consolePrint("ct = ");
//  consolePrintLn(value*1000);
  
  cycleTimeStore[cycleTimePtr] = value;
  
  if(cycleTimePtr < cycleTimeWindow-1)
    cycleTimePtr++;
  else {
    cycleTimePtr = 0;
    cycleTimesValid = true;
  }
  
  if(cycleMin < 0.0) {
    cycleMin = cycleMax = cycleCum = value;
  } else {
    cycleMin = min(cycleMin, value);
    cycleMax = max(cycleMax, value);
    cycleCum = cycleCum*(1-tau) + value*tau;
  }
}

int compareFloat(const void *a, const void *b)
{
  if(*(float*)a < *(float*)b)
    return -1;
  else if(*(float*)a > *(float*)b)
    return 1;
  else return 0;    
}

float ppmFreq;

void measurementTask(float currentTime)
{
  FORBID;
  ppmFreq = ppmFrames / (currentTime - prevMeasurement);
  ppmFrames = 0;
  PERMIT;

  logBandWidth = logBytesCum / (currentTime - prevMeasurement);
  logBytesCum = 0;
  prevMeasurement = currentTime;
  
  if(cycleTimesDone)
    return;
    
  if(cycleTimeCounter > 5) {
    controlCycle = cycleTimeFilter.output();
    consoleNote("Effective cycle time is ");
    consolePrintLn(controlCycle*1000);
    cycleTimesDone = true;
    return;
  }

  if(cycleTimesValid) {
    qsort((void*) cycleTimeStore, cycleTimeWindow, sizeof(float), compareFloat);
    controlCycle = cycleTimeStore[cycleTimeWindow/2];
    
    consoleNote("Cycle time (min, median, max) = ");
    consolePrint(cycleTimeStore[0]*1000);
    consolePrint(", ");
    consolePrint(controlCycle*1000);
    consolePrint(", ");
    consolePrintLn(cycleTimeStore[cycleTimeWindow-1]*1000);
    
    float sum = 0;
    for(int i = 0; i < cycleTimeWindow; i++)
      sum += cycleTimeStore[i];
    cycleMean = sum / cycleTimeWindow;
    cycleTimesValid = false;
    cycleTimePtr = 0;

    cycleTimeFilter.input(controlCycle);    
    cycleTimeCounter++;
  }
}

float testGainExpo(float range)
{
  return exp(3*(parameter-1))*range;
}

float testGainLinear(float start, float stop)
{
  return start + parameter*(stop - start);
}

void configurationTask(float currentTime)
{   
  static boolean pulseArmed = false, pulsePolarity = false;
  static int pulseCount = 0; 
   
  int prev = switchState;
  switchState = readSwitch();
  boolean switchStateChange = switchState != prev;

  if(initCount > 0) {
    if(rxElevatorAlive && rxAileronAlive)
      initCount--;
      
    return;
  }

  static uint32_t lastUpdate;
          
  if(switchStateChange) {
    if(switchState != switchStateLazy) {
      pulseArmed = true;
      pulsePolarity = switchState;
    } else if(pulseArmed) {
      // We detected a pulse
            
      pulseCount++;            
      
      if(pulseCount > 1) {
        pulseCount = 0;
             
        if(armed) {   
          if(!pulsePolarity) {
            if(!mode.sensorFailSafe) {
              consoleNoteLn("Failsafe ENABLED");
              mode.sensorFailSafe = true;
            } else {
              logDisable();
            }
          } else {
            consoleNoteLn("Climbing out");
            gearOutput = 1;
            if(flapOutput > 2)
              flapOutput = 2;
          }
        }
      }
      
      pulseArmed = false;
    }
          
    lastUpdate = micros();
  } else if(micros() - lastUpdate > 1e6/3) {
    if(switchState != switchStateLazy) {
      switchStateLazy = switchState;
        
      consoleNote("Lazy switch ");
      consolePrintLn(switchState ? "ON" : "OFF");

      if(switchState) {
        if(armed && !logEnabled)
          logEnable();
          
        consoleNoteLn("Wing leveler ENABLED");
        mode.wingLeveler = true;
      } else {
        if(armed && logEnabled)
          logMark();
      }
    }
          
    if(pulseCount > 0) {
      if(pulsePolarity) {
        if(mode.sensorFailSafe) {
            mode.sensorFailSafe = false;
            consoleNoteLn("Failsafe DISABLED");
            
        } else if(!armed) {
          consoleNoteLn("We're now ARMED");
          armed = true;
          talk = false;
          
        } else if(testMode) {
          consoleNote("Test channel incremented to ");
          consolePrintLn(++stateRecord.testChannel);

        } else if(gearOutput > 0) {
          if(flapOutput > 0) {
            flapOutput--;
            consoleNote("Flaps RETRACTED to ");
            consolePrintLn(flapOutput);
          }
        } else  {
          consoleNoteLn("Gear UP");
          gearOutput = 1;
        }
      } else {
        if(testMode) {
          consoleNoteLn("Test channel RESET");
          // stateRecord.testChannel = 0;
        } else if(gearOutput > 0) {
          consoleNoteLn("Gear DOWN");
          gearOutput = 0;
        } else if(flapOutput < 3) {
          flapOutput++;
          consoleNote("Flaps EXTENDED to ");
          consolePrintLn(flapOutput);
        }
      }
    }
          
    pulseArmed = false;
    pulseCount = 0;
  }

  // Test parameter

  parameter = readParameter();

  if(!testMode && parameter > 0.5) {
    testMode = true;

    consoleNoteLn("Test mode ENABLED");
    
  } else if(testMode && parameter < 0.1) {
    testMode = false;
    
    consoleNoteLn("Test mode DISABLED");
  }

  if(mode.wingLeveler && abs(aileStick) > 0.2)
    mode.wingLeveler = false;
  
  // Mode-to-feature mapping: first nominal values
      
  mode.stabilizer = true;
  mode.bankLimiter = switchStateLazy;
  mode.autoStick = mode.autoAlpha = mode.autoTrim = false;

/*  
  if(!gearOutput || flapOutput > 0)
    // Auto stick 
    mode.autoStick = true;

  if(!gearOutput && flapOutput > 0)
    // Gear down and flaps out, auto alpha    
    mode.autoAlpha = mode.autoTrim = true;
   */
  
  mode.autoStick = mode.autoAlpha = mode.autoTrim = !gearOutput;
   
  // Detect transmitter fail

  if(mode.bankLimiter && aileStick < -0.90 && elevStick > 0.90) {
    if(!mode.rxFailSafe) {
      consoleNoteLn("Receiver failsafe mode ENABLED");
      mode.rxFailSafe = true;
      mode.sensorFailSafe = false;
    }
  } else if(mode.rxFailSafe) {
    consoleNoteLn("Receiver failsafe mode DISABLED");
    mode.rxFailSafe = false;
  }
      
  // Default controller settings
     
  elevController.setPID(
    paramRecord[stateRecord.model].i_Kp, paramRecord[stateRecord.model].i_Ki, paramRecord[stateRecord.model].i_Kd);
  pusher.setPID(
    paramRecord[stateRecord.model].i_Kp, paramRecord[stateRecord.model].i_Ki, paramRecord[stateRecord.model].i_Kd);
  aileController.setPID(
    paramRecord[stateRecord.model].s_Kp, paramRecord[stateRecord.model].s_Ki, paramRecord[stateRecord.model].s_Kd);

  autoAlphaP = paramRecord[stateRecord.model].o_P;
  maxAlpha = paramRecord[stateRecord.model].alphaMax;
 
  // Then apply test modes
  
  if(testMode) {
     switch(stateRecord.testChannel) {
       case 1:
         // Wing stabilizer gain
         
         mode.stabilizer = mode.bankLimiter = mode.wingLeveler = true;
         aileController.setPID(testGain = testGainExpo(5), 0, 0);
         break;
            
       case 2:
         // Elevator stabilizer gain, outer loop disabled
         
         mode.autoStick = true;
         mode.autoTrim = mode.autoAlpha = false;
         elevController.setPID(testGain = testGainExpo(6), 0, 0);
         break;
         
       case 3:
         // Elevator stabilizer gain, outer loop enabled
         
         mode.autoStick = mode.autoTrim = mode.autoAlpha = true;
         elevController.setPID(testGain = testGainExpo(6), 0, 0);
         break;
         
       case 4:
         // Auto alpha outer loop gain
         
         mode.autoStick = mode.autoTrim = mode.autoAlpha = true;
         autoAlphaP = testGain = testGainExpo(21);
         break;
         
       case 11:
         // Elevator stabilizer gain, mode depends on config
         
         testGain = testGainExpo(4);
         elevController.setPID(testGain, 0, 0);
         pusher.setPID(testGain, 0, 0);
         break;
         
       case 12:
         // Auto alpha outer loop gain, mode depends on config
         
         autoAlphaP = testGain = testGainExpo(15);
         break;

       case 21:
         // Elevator stabilizer ZN gain = 0... 1.5, fixed period = 0.5
         
         testGain = testGainExpo(1.5);
         elevController.setZieglerNicholsPID(testGain, 0.5);
         pusher.setZieglerNicholsPID(testGain, 0.5);
         break;
         
       case 22:
         // Elevator stabilizer ZN period = 0... 1, fixed gain = 0.9
         
         testGain = testGainExpo(1);
         elevController.setZieglerNicholsPID(0.9, testGain);
         pusher.setZieglerNicholsPID(0.9, testGain);
         break;
         
       case 31:
         // Elevator stabilizer ZN empirical period

         float Ku, Tu;
         elevController.getZieglerNicholsPID(&Ku, &Tu);
         parameter *= neutralAlpha/maxAlpha;
         testGain = testGainLinear(Tu, 0.5);
         elevController.setZieglerNicholsPID(Ku, testGain);
         pusher.setZieglerNicholsPID(Ku, testGain);
         break;
         
       case 7:
         // Max alpha
         
         maxAlpha = testGain = testGainLinear(10, 20);
         break;         
     }
  }
      
  if(!mode.autoStick)
    neutralStick = elevStick;

  alphaFilter.input(alpha);
  
  if(!mode.autoAlpha)
    neutralAlpha = clamp(alphaFilter.output(), minAlpha, maxAlpha);

  // Calibration button

  if(digitalRead(buttonPin) == 0) {
    calibrateStart = !calibrate;
    calibrate = true;
  } else {
    calibrateStop = calibrate;
    calibrate = false;
   }
}

void loopTask(float currentTime)
{
  if(looping) {
    consolePrint("alpha = ");
    consolePrint(alpha*360);
/*    consolePrint(" IAS = ");
    consolePrint(sqrt(2*dynPressure));
*/

    consolePrint(" ppmFreq = ");
    consolePrint(ppmFreq);
    consolePrint(" aileStick = ");
    consolePrint(aileStick);
    consolePrint(" elevStick = ");
    consolePrint(elevStick);
    
/*
    consolePrint(" roll = ");
    consolePrint(rollAngle);
    consolePrint(" (rate = ");
    consolePrint(rollRate);
    consolePrint(") pitch = ");
    consolePrint(pitchAngle);
    consolePrint(" (rate = ");
    consolePrint(pitchRate);
*/
    /*
    consolePrint(") rpm = ");
    consolePrint(readRPM());
    */
/*    consolePrint(" heading = ");
    consolePrint(heading);
*/    consolePrint(" alt(GPS) = ");
    consolePrint(altitude);
    consolePrint(" m (");
    consolePrint(gpsFix.altitude);
    consolePrint(" m) speed = ");
    consolePrint(gpsFix.speed);
/*    consolePrint(" target = ");
    consolePrint(targetAlpha*360);
    consolePrint(" trim = ");
    consolePrint(neutralAlpha*360);
*/    consolePrint(" testGain = ");
    consolePrint(testGain);
    consolePrintLn("");
  }
}

const int serialBufLen = 1<<7;
char serialBuf[serialBufLen];
int serialBufIndex = 0;

void communicationTask(float currentTime)
{
  int len = 0;
  boolean dirty = false;
       
  while((len = Serial.available()) > 0) {
    dirty = true;
    
    int spaceLeft = serialBufLen - serialBufIndex;
    
    if(len > spaceLeft) {
      for(int i = 0; i < len - spaceLeft; i++)
        Serial.read();
    }
    
    len = min(len, spaceLeft);
    Serial.readBytes(&serialBuf[serialBufIndex], len);
    serialBufIndex += len;
  }

  if(dirty && serialBufIndex > 0 && serialBuf[serialBufIndex-1] == '\n') {
    if(looping) {
      looping = false;
      rpmMeasure(stateRecord.logRPM);
    }

    executeCommandSeries(serialBuf, serialBufIndex-1);
    serialBufIndex = 0;

    controlCycleEnded = -1.0;
  }    
}

const int gpsBufLen = 1<<7, gpsMaxParam = 1<<5;
char gpsBuf[gpsBufLen], gpsMsg[gpsBufLen], gpsParam[gpsMaxParam+1];
int gpsBufIndex = 0, gpsMsgLen = 0;

const char *gpsParamIndex(int n)
{
  int start = 0, end = -1;
  int i = 0;

  do {
    if(end > -1)
      start = end+1;
      
    end = start;
    
    while(gpsMsg[end] != '\0' && gpsMsg[end] != ',')
      end++;
          
    if(i == n) {
      int len = min(end-start, gpsMaxParam);
      strncpy(gpsParam, &gpsMsg[start], len);
      gpsParam[len] = '\0';      
      break;
    }
      
    i++;
  } while(gpsMsg[end] != '\0');
  
  return gpsParam;
}

int hexDigit(char c)
{
  if(isdigit(c))
    return c - '0';
  else if(c >= 'A' && c <= 'F')
    return 10 + c - 'A';
  else if(c >= 'a' && c <= 'f')
    return 10 + c - 'a';
  else
    return -1;
}

boolean gpsChecksum(void)
{
  if(gpsMsgLen < 3 || gpsMsg[gpsMsgLen-3] != '*')
    return false;
    
  uint8_t chkSum = hexDigit(gpsMsg[gpsMsgLen-2])*16+
    hexDigit(gpsMsg[gpsMsgLen-1]);
    
  uint8_t sum = 0;
  
  for(int i = 1; i < gpsMsgLen - 3; i++)
    sum ^= gpsMsg[i];
    
  return sum == chkSum;
}

void gpsSentence(const char *type)
{
  if(!strncmp("RMC", type, 3)) {
    if(!strcmp("A", gpsParamIndex(2))) {
      gpsFix.speed = atof(gpsParamIndex(7));
      gpsFix.track = atof(gpsParamIndex(8));
//      consoleNote("GPS speed = ");
//      consolePrintLn(gpsFix.speed);
    }    
  } else if(!strncmp("GGA", type, 3)) {
    if(atoi(gpsParamIndex(6)) > 0) {
      gpsFix.lat = atof(gpsParamIndex(2));
      gpsFix.lon = atof(gpsParamIndex(4));
      gpsFix.altitude = atof(gpsParamIndex(9));
//      consoleNote("GPS fix = ");
//      consolePrint(gpsFix.lat);
//      consolePrint(", ");
//      consolePrintLn(gpsFix.lon);
    }
  }
}

void gpsInput(const char *buf, int len)
{
  for(int i = 0; i < len; i++) {
    if(buf[i] == '\r') {
//      consoleNote("GPS : ");
//      consolePrintLn(gpsMsg);
      
      if(!strncmp(gpsMsg, "$GP", 3) && gpsChecksum()) {
        gpsSentence(&gpsMsg[3]);
      } else
        consoleNoteLn("Corrupt GPS sentence");

      gpsMsgLen = 0;        
      gpsMsg[0] = '\0';
    } else if(buf[i] != '\n' && gpsMsgLen < gpsBufLen-1) {
//      consoleNote("GPS : ");
//      consolePrintLn(buf[i]);
      gpsMsg[gpsMsgLen++] = buf[i];
      gpsMsg[gpsMsgLen] = '\0';
    }
  }
}

float sign(float x)
{
  return x < 0.0 ? -1.0 : 1.0;
}

void gpsTask(float currentTime)
{
  int len = 0;
  boolean dirty = false;
       
  while((len = Serial1.available()) > 0) {
    dirty = true;
    
    int spaceLeft = gpsBufLen - gpsBufIndex;
    
    if(len > spaceLeft) {
      for(int i = 0; i < len - spaceLeft; i++)
        Serial1.read();
      consoleNote("Lost ");
      consolePrintLn(len - spaceLeft);
    }
    
    len = min(len, spaceLeft);
    
    if(len > 0) {
      Serial1.readBytes(&gpsBuf[gpsBufIndex], len);
      gpsBufIndex += len;
    }
    
    if(gpsBufIndex > 0) {
//      consoleNote("Proc ");
//      consolePrintLn(gpsBufIndex);
      gpsInput(gpsBuf, gpsBufIndex);
      gpsBufIndex = 0;
    }        
  }
}

void controlTask(float currentTime)
{
  // Cycle time bookkeeping 
  
  if(controlCycleEnded > 0.0)
    cycleTimeMonitor(currentTime - controlCycleEnded);

  controlCycleEnded = currentTime;
  
  // Alpha input
  
  alpha = alphaBuffer.output();
  
  if(calibrateStart) {
    paramRecord[stateRecord.model].elevZero += elevStick;
    paramRecord[stateRecord.model].aileZero += aileStick;
    paramRecord[stateRecord.model].elevNeutral = paramRecord[stateRecord.model].aileNeutral = 0.0;
    calibrateStart = false;
  } else if(calibrateStop) {       
    paramRecord[stateRecord.model].elevNeutral = elevStick;
    paramRecord[stateRecord.model].aileNeutral = aileStick;
    calibrateStop = false;
  } else if(calibrate) {
    elevOutput = elevStick;
    aileOutput = aileStick;
  } else {
    // Elevator control
    
    targetAlpha = 0.0;

    if(mode.autoStick) {
      float targetRate = clamp(elevStick, -0.5, 0.5);

      if(mode.autoAlpha) {  
        float maxAutoAlpha = maxAlpha/square(1.1);
        
        targetAlpha = clamp(neutralAlpha + elevStick*maxAutoAlpha,
          minAlpha, maxAutoAlpha);
 
        targetRate = (targetAlpha - alpha) * autoAlphaP;
        
      } else
        targetRate = min(targetRate, (maxAlpha - alpha) * autoAlphaP);
      
      elevController.input(targetRate - pitchRate, controlCycle);
    } else {
      elevController.reset(elevStick, 0.0);
    }
    
    if(mode.autoStick && !mode.sensorFailSafe && !alphaFailed) {
      const float fract_c = 1.0/3;
      float strongStick = 
        sign(elevStick)*max(abs(elevStick)-(1.0-fract_c), 0)/fract_c;

      elevOutput = mixValue(square(strongStick), elevController.output(), elevStick);
    } else
      elevOutput = elevStick;   
    
    // Pusher

    pusher.input((maxAlpha - alpha)*paramRecord[stateRecord.model].o_P - pitchRate, controlCycle);

    if(!mode.sensorFailSafe && !alphaFailed)
      elevOutput = min(elevOutput, pusher.output());
  
    // Aileron
    
    float maxBank = 45.0;
    
    float targetRate = 270.0/360*aileStick;
    
    if(mode.rxFailSafe)
      maxBank = 15.0;

    else if(mode.autoTrim)
      maxBank -= 30.0*(neutralAlpha / maxAlpha);

    if(mode.sensorFailSafe)
      aileOutput = aileStick;
    
    else if(!armed || !mode.stabilizer) {
      // Simple proportional wing leveler
        
      aileOutput = (aileStick*maxBank - rollAngle) / 90;
      aileController.reset(aileOutput, targetRate - rollRate);
      
    } else {
      // Roll stabilizer enabled
      
      if(mode.wingLeveler)
        // Wing leveler enabled
        
        targetRate = clamp((aileStick*maxBank - rollAngle) / 90, -0.5, 0.5);
      else if(mode.bankLimiter) {
        // No leveling but limit bank
                
        targetRate = clamp(targetRate, (-maxBank - rollAngle) / 90, (maxBank - rollAngle) / 90);
      }
      
      aileController.input(targetRate - rollRate, controlCycle);
      aileOutput = aileController.output();
    }
 
    // Brake
    
    if(!mode.sensorFailSafe && gearOutput == 1)
      brakeOutput = 0;
    else
      brakeOutput = max(-elevStick, 0);
  } 
}

void actuatorTask(float currentTime)
{
  // Actuators
 
  if(armed) {
    servoOutput(aileHandle, 1500 + 500*clamp(paramRecord[stateRecord.model].aileDefl*aileOutput 
      + paramRecord[stateRecord.model].aileNeutral, -1, 1));

    servoOutput(elevatorHandle, 1500 + 500*clamp(paramRecord[stateRecord.model].elevDefl*elevOutput 
      + paramRecord[stateRecord.model].elevNeutral, -1, 1));
                              
    servoOutput(flapHandle, 1500 + 500*(clamp(paramRecord[stateRecord.model].flapNeutral 
                        + flapOutput*paramRecord[stateRecord.model].flapStep, -1, 1)));                              

    servoOutput(gearHandle, 1500 - 500*(gearOutput*2-1));

    servoOutput(brakeHandle, 1500 + 500*clamp(paramRecord[stateRecord.model].brakeNeutral + 
                                paramRecord[stateRecord.model].brakeDefl*brakeOutput, -1, 1));                        
  }
}

void trimTask(float currentTime)
{
  if(mode.autoTrim && abs(rollAngle) < 30) {
    neutralAlpha += clamp((targetAlpha - neutralAlpha)/2/TRIM_HZ,
      -1.5/360/TRIM_HZ, 1.5/360/TRIM_HZ);
//    neutralAlpha = clamp(neutralAlpha, minAlpha, maxAlpha*0.9);
  }
}

boolean logInitialized;

void backgroundTask(long durationMicros)
{
  logInitialized = logInit(durationMicros);
  
  if(logInitialized)
    delayMicroseconds(durationMicros);
}

void blinkTask(float currentTime)
{
  float ledRatio = testMode ? 0.0 : !logInitialized ? 1.0 : (mode.sensorFailSafe || !armed) ? 0.5 : alpha > 0.0 ? 0.90 : 0.10;
  static int tick = 0;
  
  tick = (tick + 1) % (LED_TICK/LED_HZ);
    
  if(tick < ledRatio*LED_TICK/LED_HZ) {
    STABLEPIN_ON;
  } else {
    STABLEPIN_OFF;
  }
}

struct Task taskList[] = {
  { 1.0/100, 0, communicationTask },
//  { 1.0/100, 0, gpsTask },
  { 1.0/ALPHA_HZ, 0, alphaTask },
  { 1.0/30/2, 0, airspeedReadTask },
  { 1.0/30, 0, airspeedUpdateTask },
  { 1.0/LED_TICK, 0, blinkTask },
  { 1.0/CONTROL_HZ, 0, receiverTask },
  { 1.0/CONTROL_HZ, 0, sensorTask },
  { 1.0/CONTROL_HZ, 0, controlTask },
  { 1.0/ACTUATOR_HZ, 0, actuatorTask },
  { 1.0/TRIM_HZ, 0, trimTask },
  { 1.0/50, 0, configurationTask },
  { 1.0/4, 0, cacheTask },
  { 1.0/10, 0, rpmTask },
  { 1.0/45, 0, alphaLogTask },
  { 1.0/15, 0, controlLogTask },
  { 1.0/2, 0, positionLogTask },
  { 1.0/2, 0, logSaveTask },
  { 1.0, 0, measurementTask },
  { 1.0/10, 0, loopTask },
  { 0, 0, NULL } };

int scheduler(float currentTime)
{
  struct Task *task = taskList;
  
  while(task->code) {
    if(task->lastExecuted + task->period < currentTime
      || task->lastExecuted > currentTime) {
      task->code(currentTime);
      task->lastExecuted = currentTime;
      
      if(task->period > 0.0)
        // Staggered execution for all but the critical tasks
        return 1;
    }
    
    task++;
  }
  
  return 0;
}

void loop() 
{
  while(1) {
    // Invoke scheduler
  
    currentTime = micros();
    float currentTimeF = (float) currentTime / 1e6;
    
    if(!scheduler(currentTimeF))
      // Idle
      
      backgroundTask(250);
  }
}

/* 

// Param backup 2015/10/23
//
// MODEL 0
//

echo 0; model 0; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 48220; inner_pid 1.3200 15.8654 0.0275; outer_p 7.50; stabilizer_pid 0.6600 5.6410 0.0193; min -3.00; max 13.50; edefl -35.00; eneutral 3.00; ezero 4.34; adefl 60.00; aneutral -20.00; azero 4.34; fstep -30.00; fneutral 52.50; bdefl 45.00; bneutral 0.00; filtlen 1; echo 1; store

//
// MODEL 3
//

echo 0; model 3; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 56799; inner_pid 0.4980 3.9840 0.0156; outer_p 7.50; stabilizer_pid 0.7200 8.0990 0.0160; min -5.00; max 12.50; edefl -90.00; eneutral -5.00; ezero 3.56; adefl -40.00; aneutral -5.00; azero 8.00; fstep -27.00; fneutral 45.00; bdefl -50.00; bneutral -15.00; filtlen 2; echo 1; store


// Param backup 2015/10/03
//
// MODEL 0
//

echo 0; model 0; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 48220; inner_pid 1.4400 17.3077 0.0300; outer_p 7.50; stabilizer_pid 0.6600 5.6410 0.0193; min -3.00; max 13.50; edefl -35.00; eneutral 3.00; ezero 4.34; adefl 60.00; aneutral -20.00; azero 4.34; fstep -30.00; fneutral 52.50; bdefl 45.00; bneutral 0.00; filtlen 1; echo 1; store

//
// MODEL 1
//

echo 0; model 1; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner_pid 1.4400 17.3077 0.0300; outer_p 20.00; stabilizer_pid 0.6600 5.6410 0.0193; min -3.00; max 12.00; edefl -37.50; eneutral -90.00; ezero 0.00; adefl -75.00; aneutral -90.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl 45.00; bneutral 0.00; filtlen 2; echo 1; store

//
// MODEL 2
//

echo 0; model 2; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner_pid 1.4400 17.3077 0.0300; outer_p 20.00; stabilizer_pid 0.6600 5.6410 0.0193; min -3.00; max 12.00; edefl -25.00; eneutral 0.00; ezero 0.00; adefl -50.00; aneutral 0.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl -50.00; bneutral -15.00; filtlen 2; echo 1; store

//
// MODEL 3
//

echo 0; model 3; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner_pid 1.4400 17.3077 0.0300; outer_p 20.00; stabilizer_pid 0.6600 5.6410 0.0193; min -3.00; max 12.00; edefl -25.00; eneutral 0.00; ezero 0.00; adefl -50.00; aneutral 0.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl -50.00; bneutral -15.00; filtlen 2; echo 1; store


// Param backup 2015/10/2
//
// MODEL 0
//

echo 0; model 0; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 48436; inner_pid 1.8000 19.2000 0.0422; outer_p 7.50; stabilizer_pid 0.6840 5.8462 0.0200; min -3.00; max 14.00; edefl -35.00; eneutral 3.00; ezero 4.34; adefl 60.00; aneutral -20.00; azero 4.34; fstep -30.00; fneutral 52.50; bdefl 45.00; bneutral 0.00; filtlen 1; echo 1; store

//
// MODEL 1
//

echo 0; model 1; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner_pid 1.8000 19.2000 0.0422; outer_p 20.00; stabilizer_pid 0.6840 5.8462 0.0200; min -3.00; max 12.00; edefl -37.50; eneutral -90.00; ezero 0.00; adefl -75.00; aneutral -90.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl 45.00; bneutral 0.00; filtlen 2; echo 1; store

//
// MODEL 2
//

echo 0; model 2; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner_pid 1.8000 19.2000 0.0422; outer_p 20.00; stabilizer_pid 0.6840 5.8462 0.0200; min -3.00; max 12.00; edefl -25.00; eneutral 0.00; ezero 0.00; adefl -50.00; aneutral 0.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl -50.00; bneutral -15.00; filtlen 2; echo 1; store

//
// MODEL 3
//

echo 0; model 3; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner_pid 1.8000 19.2000 0.0422; outer_p 20.00; stabilizer_pid 0.6840 5.8462 0.0200; min -3.00; max 12.00; edefl -25.00; eneutral 0.00; ezero 0.00; adefl -50.00; aneutral 0.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl -50.00; bneutral -15.00; filtlen 2; echo 1; store

// Param backup
//
// MODEL 0 KYOSHO 2015/9/24
//

echo 0; model 0; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 35472; inner_pid 1.8000 19.2000 0.0422; outer_p 7.50; stabilizer_pid 0.8400 8.4000 0.0210; min -3.00; max 14.00; edefl -35.00; eneutral 0.00; ezero 4.34; adefl 60.00; aneutral -20.00; azero 4.34; fstep -30.00; fneutral 52.50; bdefl 45.00; bneutral 0.00; filtlen 1; echo 1; store

// Param backup KYOSHO 2015/9/22
//
// MODEL 0
//

echo 0; model 0; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 34894; inner_pid 1.9800 21.1200 0.0464; outer_p 7.50; stabilizer_pid 0.9000 9.0000 0.0225; min -3.00; max 15.00; edefl -35.00; eneutral 0.00; ezero 4.34; adefl 60.00; aneutral -20.00; azero 4.34; fstep -30.00; fneutral 52.50; bdefl 45.00; bneutral 0.00; filtlen 1; echo 1; store

//
// MODEL 2 VIPER backup 2015(9(9
//

echo 0; model 2; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 28636; inner_pid 1.2000 8.0000 0.0450; outer_p 7.50; stabilizer_pid 0.6000 4.0000 0.0225; min -3.00; max 12.00; edefl -30.00; eneutral 5.00; ezero 5.04; adefl -50.00; aneutral 0.00; azero 5.04; fstep -27.00; fneutral 45.00; bdefl -50.00; bneutral -15.00; filtlen 2; echo 1; store

//
// MODEL 1 L-59 backup 2015/9/8
//

echo 0; model 1; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 28120; inner_pid 1.2000 7.2727 0.0495; outer_p 10.00; stabilizer_pid 0.6000 3.6364 0.0248; min -3.00; max 12.00; edefl 40.00; eneutral 0.00; ezero 6.05; adefl -50.00; aneutral 0.00; azero 11.09; fstep -27.00; fneutral 45.00; bdefl -50.00; bneutral -15.00; filtlen 2; echo 1; store

//
// MODEL 0 KYOSHO Backup 2015/9/7
//

echo 0; model 0; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 24315; inner_pid 1.9800 21.1200 0.0464; outer_p 7.50; stabilizer_pid 0.9000 9.0000 0.0225; min -3.00; max 14.00; edefl -30.00; eneutral -5.00; ezero 5.04; adefl 50.00; aneutral 10.00; azero 5.04; fstep -30.00; fneutral 52.50; bdefl 45.00; bneutral 0.00; filtlen 1; echo 1; store

// Param backup 2015/9/3

//
// MODEL 0
//

echo 0; model 0; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 24315; inner_pid 1.97 21.85 0.04; outer_p 6.60; stabilizer_pid 0.93 8.30 0.03; min -3.00; max 14.00; edefl -30.00; eneutral -5.00; ezero 5.04; adefl 50.00; aneutral 10.00; azero 5.04; fstep -30.00; fneutral 52.50; bdefl 45.00; bneutral 0.00; filtlen 1; echo 1; store
// Param backup 2015/9/2

//
// MODEL 0
//

echo 0; model 0; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 24315; inner_pid 1.97 21.85 0.04; outer_p 6.60; stabilizer_pid 0.93 8.30 0.03; min -3.00; max 14.00; edefl -30.00; eneutral -5.00; ezero 5.04; adefl 50.00; aneutral 10.00; azero 10.08; fstep -30.00; fneutral 52.50; bdefl 45.00; bneutral 0.00; filtlen 1; echo 1; store


// Backup 2015/08/30

echo 0; model 0; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 24315; inner_pid 1.26 14.20 0.03; outer_p 10.00; stabilizer_pid 0.24 2.13 0.01; min -3.00; max 12.00; edefl -30.00; eneutral -15.00; ezero 4.03; adefl -50.00; aneutral 10.00; azero 4.03; fstep -30.00; fneutral 52.50; bdefl 45.00; bneutral 0.00; filtlen 1; echo 1; store

// Param backup 2015/08/28

//
// MODEL 0
//

echo 0; model 0; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 24764; inner_pid 0.21 2.10 0.01; outer_p 35.00; stabilizer_pid 0.24 2.13 0.01; min -3.00; max 12.00; edefl -30.00; eneutral -15.00; ezero 4.03; adefl -50.00; aneutral 10.00; azero 4.03; fstep -30.00; fneutral 52.50; bdefl 45.00; bneutral 0.00; filtlen 1; echo 1; store

//
// MODEL 1
//

echo 0; model 1; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner_pid 0.21 2.10 0.01; outer_p 20.00; stabilizer_pid 0.24 2.13 0.01; min -3.00; max 12.00; edefl -37.50; eneutral -90.00; ezero 0.00; adefl -75.00; aneutral -90.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl 45.00; bneutral 0.00; filtlen 2; echo 1; store

//
// MODEL 2
//

echo 0; model 2; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner_pid 0.21 2.10 0.01; outer_p 20.00; stabilizer_pid 0.24 2.13 0.01; min -3.00; max 12.00; edefl -25.00; eneutral 0.00; ezero 0.00; adefl -50.00; aneutral 0.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl -50.00; bneutral -15.00; filtlen 2; echo 1; store

//
// MODEL 3
//

echo 0; model 3; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner_pid 0.21 2.10 0.01; outer_p 20.00; stabilizer_pid 0.24 2.13 0.01; min -3.00; max 12.00; edefl -25.00; eneutral 0.00; ezero 0.00; adefl -50.00; aneutral 0.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl -50.00; bneutral -15.00; filtlen 2; echo 1; store

// Param backup 2015/08/27 no 2

//
// MODEL 0
//

echo 0; model 0; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 24764; inner_pid 0.21 2.10 0.01; outer_p 35.00; stabilizer_pid 0.50 1.20 0.01; min -3.00; max 12.00; edefl -30.00; eneutral -15.00; ezero 4.03; adefl -50.00; aneutral 10.00; azero 4.03; fstep -30.00; fneutral 52.50; bdefl 45.00; bneutral 0.00; filtlen 1; echo 1; store

//
// MODEL 1
//

echo 0; model 1; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner_pid 0.21 2.10 0.01; outer_p 20.00; stabilizer_pid 0.50 1.20 0.01; min -3.00; max 12.00; edefl -37.50; eneutral -90.00; ezero 0.00; adefl -75.00; aneutral -90.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl 45.00; bneutral 0.00; filtlen 2; echo 1; store

//
// MODEL 2
//

echo 0; model 2; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner_pid 0.21 2.10 0.01; outer_p 20.00; stabilizer_pid 0.50 1.20 0.01; min -3.00; max 12.00; edefl -25.00; eneutral 0.00; ezero 0.00; adefl -50.00; aneutral 0.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl -50.00; bneutral -15.00; filtlen 2; echo 1; store

//
// MODEL 3
//

echo 0; model 3; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner_pid 0.21 2.10 0.01; outer_p 20.00; stabilizer_pid 0.50 1.20 0.01; min -3.00; max 12.00; edefl -25.00; eneutral 0.00; ezero 0.00; adefl -50.00; aneutral 0.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl -50.00; bneutral -15.00; filtlen 2; echo 1; store

// Param backup 2015/08/27

//
// MODEL 0
//

echo 0; model 0; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 24764; inner_pid 0.21 2.10 0.01; outer_p 35.00; stabilizer_pid 0.50 1.20 0.01; min -3.00; max 12.00; edefl -30.00; eneutral -15.00; ezero 4.03; adefl -50.00; aneutral 10.00; azero 4.03; fstep -30.00; fneutral 52.50; bdefl 45.00; bneutral 0.00; filtlen 1; echo 1; store

//
// MODEL 1
//

echo 0; model 1; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner_pid 0.21 2.10 0.01; outer_p 20.00; stabilizer_pid 0.50 1.20 0.01; min -3.00; max 12.00; edefl -37.50; eneutral -90.00; ezero 0.00; adefl -75.00; aneutral -90.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl 45.00; bneutral 0.00; filtlen 2; echo 1; store

//
// MODEL 2
//

echo 0; model 2; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner_pid 0.21 2.10 0.01; outer_p 20.00; stabilizer_pid 0.50 1.20 0.01; min -3.00; max 12.00; edefl -25.00; eneutral 0.00; ezero 0.00; adefl -50.00; aneutral 0.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl -50.00; bneutral -15.00; filtlen 2; echo 1; store

//
// MODEL 3
//

echo 0; model 3; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner_pid 0.21 2.10 0.01; outer_p 20.00; stabilizer_pid 0.50 1.20 0.01; min -3.00; max 12.00; edefl -25.00; eneutral 0.00; ezero 0.00; adefl -50.00; aneutral 0.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl -50.00; bneutral -15.00; filtlen 2; echo 1; store

// param backup 26/8/2015

//
// MODEL 0
//

echo 0; model 0; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 24764; inner 0.35 0.20; outer 35.00; stabilizer 0.50 0.20 0.00; min -3.00; max 12.00; edefl -30.00; eneutral -15.00; ezero 4.03; adefl -50.00; aneutral 10.00; azero 4.03; fstep -30.00; fneutral 52.50; bdefl 45.00; bneutral 0.00; filtlen 1; echo 1; store

//
// MODEL 1
//

echo 0; model 1; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner 2.00 0.50; outer 20.00; stabilizer 1.00 0.10 0.10; min -3.00; max 12.00; edefl -37.50; eneutral -90.00; ezero 0.00; adefl -75.00; aneutral -90.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl 45.00; bneutral 0.00; filtlen 2; echo 1; store

//
// MODEL 2
//

echo 0; model 2; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner 2.00 0.50; outer 20.00; stabilizer 1.00 0.10 0.10; min -3.00; max 12.00; edefl -25.00; eneutral 0.00; ezero 0.00; adefl -50.00; aneutral 0.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl -50.00; bneutral -15.00; filtlen 2; echo 1; store

//
// MODEL 3
//

echo 0; model 3; 24l256_addr 80; 24l256_clk 12; 5048b_addr 64; 5048b_clk 12; 5048b_ref 0; inner 2.00 0.50; outer 20.00; stabilizer 1.00 0.10 0.10; min -3.00; max 12.00; edefl -25.00; eneutral 0.00; ezero 0.00; adefl -50.00; aneutral 0.00; azero 0.00; fstep -27.00; fneutral 45.00; bdefl -50.00; bneutral -15.00; filtlen 2; echo 1; store


*/
