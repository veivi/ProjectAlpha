#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Servo.h>
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

#define MEGAMINI

NewI2C I2c = NewI2C();

#define FORBID if(!nestCount++) cli()
#define PERMIT if(!--nestCount) sei()

uint8_t nestCount;

typedef enum { init_c, stop_c, run_c } logState_t;

int32_t logPtr, logLen, logSize;
uint16_t logEndStamp;

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
boolean calibrate, switchState = false, switchStateLazy = false, echoEnabled = true, logEnabled = false;
boolean iasFailed = false, iasWarn = false, alphaFailed = false, alphaWarn = false, pciWarn = false, eepromWarn = false, eepromFailed = false;
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
logState_t logState;
int initCount = 5;
boolean armed = false, talk = true;
float neutralStick = 0.0, neutralAlpha, targetAlpha;
float switchValue, tuningKnobValue, rpmOutput;
Controller elevController, aileController, pusher;
float autoAlphaP, maxAlpha;
float acc,altitude,  heading, rollAngle, pitchAngle, rollRate, pitchRate;
int logBytesCum;
float logBandWidth;
int cycleTimeCounter = 0;
Median3Filter cycleTimeFilter;
boolean cycleTimesDone = false;
float prevMeasurement;
float parameter;  

struct GPSFix {
  float altitude;
  float track;
  float lat;
  float lon;
  float speed;
};

struct GPSFix gpsFix;

#define BAUDRATE 115200

#define EXT_EEPROM_SIZE (1L<<17)
#define EXT_EEPROM_PAGE (1L<<7)
#define EXT_EEPROM_LATENCY 5000
#define PAGE_MASK ~(EXT_EEPROM_PAGE-1)

#define INT_EEPROM_SIZE (1<<12)

#define buttonPin 35

#define aileServoPin  12
#define elevatorServoPin  11
#define flapServoPin  8
#define gearServoPin  7
#define brakeServoPin  7

#ifdef MEGAMINI
#define ppmInputPin 48
#endif

// Pin change stuff

struct RxInputRecord {
  int pin, port;
  boolean freqOnly, alive;
  uint32_t pulseStart;
  uint32_t pulseCount;
  uint32_t pulseWidthAcc;
};

#define rpmPin A14

#ifdef MEGAMINI

#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define CH_9 8
#define CH_10 9
#define CH_11 10
#define CH_12 11
#define CH_13 12
#define CH_14 13
#define CH_15 14
#define CH_16 15
#define CH_17 16
#define CH_18 17

struct RxInputRecord aileInput, elevInput, switchInput, modeInput;

#define AVR_RC_INPUT_NUM_CHANNELS 8
#define AVR_RC_INPUT_MIN_CHANNELS 8

/*
  mininum pulse width in microseconds to signal end of a PPM-SUM
  frame. This value is chosen to be smaller than the default 3000 sync
  pulse width for OpenLRSng. Note that this is the total pulse with
  (typically 300us low followed by a long high pulse)
 */
#define AVR_RC_INPUT_MIN_SYNC_PULSE_WIDTH 2500

uint16_t _pulse_capt[AVR_RC_INPUT_NUM_CHANNELS];

struct RxInputRecord *ppmInputRecord[AVR_RC_INPUT_NUM_CHANNELS] = 
{ &aileInput, &elevInput, NULL, NULL, &switchInput, &modeInput };

uint8_t ppmNumChannels;

void handlePPMInput(const uint16_t *pulse, int numCh)
{
  ppmNumChannels = numCh;

  for(int i = 0; i < numCh; i++) {
    if(ppmInputRecord[i]) {
       ppmInputRecord[i]->alive = true;
       ppmInputRecord[i]->pulseCount++;
       ppmInputRecord[i]->pulseWidthAcc += pulse[i]/2;              
     }
  }
}

ISR(TIMER5_CAPT_vect) {
    static uint16_t icr5_prev;
    static uint8_t  channel_ctr;

    const uint16_t icr5_current = ICR5;
    uint16_t pulse_width;
    if (icr5_current < icr5_prev) {
        /* ICR5 rolls over at TOP=40000 */
        pulse_width = icr5_current + 40000 - icr5_prev;
    } else {
        pulse_width = icr5_current - icr5_prev;
    }

    if (pulse_width > AVR_RC_INPUT_MIN_SYNC_PULSE_WIDTH*2) {
        // sync pulse detected.  Pass through values if at least a minimum number of channels received
        if( channel_ctr >= AVR_RC_INPUT_MIN_CHANNELS ) {
             handlePPMInput(_pulse_capt, channel_ctr);
        }
        channel_ctr = 0;
    } else {
        if (channel_ctr < AVR_RC_INPUT_NUM_CHANNELS) {
            _pulse_capt[channel_ctr] = pulse_width;
            channel_ctr++;
            if (channel_ctr == AVR_RC_INPUT_NUM_CHANNELS) {
               handlePPMInput(_pulse_capt, channel_ctr);
            }
        }
    }
    icr5_prev = icr5_current;
}

#define HAL_GPIO_INPUT INPUT

void ppm_input_init() {
 //   isrregistry->register_signal(ISR_REGISTRY_TIMER5_CAPT, _timer5_capt_cb);

    /* initialize overrides */
    /* Arduino pin 48 is ICP5 / PL1,  timer 5 input capture */

    /**
     * WGM: 1 1 1 1. Fast WPM, TOP is in OCR5A
     * COM all disabled
     * CS51: prescale by 8 => 0.5us tick
     * ICES5: input capture on rising edge
     * OCR5A: 40000, 0.5us tick => 2ms period / 50hz freq for outbound
     * fast PWM.
     */

    FORBID;
    
    /* Timer cleanup before configuring */
    TCNT5 = 0;
    TIFR5 = 0;
    
    /* Set timer 8x prescaler fast PWM mode toggle compare at OCRA with rising edge input capture */
    TCCR5A = 0;
    TCCR5B = (1<<ICES5) | (1<<CS51);
    OCR5A  = 40000 - 1; // -1 to correct for wrap

    /* Enable input capture interrupt */
    TIMSK5 = 1<<ICIE5;
     
    PERMIT;
  }

#define HAL_GPIO_OUTPUT OUTPUT

#define RC_OUTPUT_MIN_PULSEWIDTH 400
#define RC_OUTPUT_MAX_PULSEWIDTH 2100

struct HWTimer {
  volatile uint8_t *TCCRA, *TCCRB;
  volatile uint16_t *ICR;
  volatile uint16_t *OCR[3]; // 0... 2 = A ... C
  boolean initDone;
};

typedef enum { COMnA = 0, COMnB = 1, COMnC = 2, COMnInvalid } PWM_Ch_t;

const uint8_t outputModeMask[3] = { 1<<COM1A1, 1<<COM1B1, 1<<COM1C1 };

struct PWMOutput {
  int pin;
  struct HWTimer *timer;
  PWM_Ch_t pwmCh; // COMnA / COMnB / COMnC
};

struct HWTimer hwTimer1 = { &TCCR1A, &TCCR1B, &ICR1, { &OCR1A, &OCR1B, &OCR1C } };
struct HWTimer hwTimer3 = { &TCCR3A, &TCCR3B, &ICR3, { &OCR3A, &OCR3B, &OCR3C } };
struct HWTimer hwTimer4 = { &TCCR4A, &TCCR4B, &ICR4, { &OCR4A, &OCR4B, &OCR4C } };

struct PWMOutput pwmOutput[] = {
       { 12, &hwTimer1, COMnB },
       { 11, &hwTimer1, COMnA },
       { 8, &hwTimer4, COMnC },
       { 7, &hwTimer4, COMnB },
       { 6, &hwTimer4, COMnA },
       { 3, &hwTimer3, COMnC },
       { 2, &hwTimer3, COMnB },
       { 5, &hwTimer3, COMnA } };

#define PWM_HZ 50
#define TIMER_HZ (16e6/8)

void pwmTimerInit(struct HWTimer *timer)
{
   if(timer->initDone)
      return;
      
   // WGM, prescaling
   
   *(timer->TCCRA) = 1<<WGM11;
   *(timer->TCCRB) = (1<<WGM13) | (1<<WGM12) | (1<<CS11);

   // PWM frequency
   
   *(timer->ICR) = TIMER_HZ/PWM_HZ - 1;

   // Output set to nil by default

   for(int i = 0; i < 3; i++)
      *(timer->OCR[i]) = 0xFFFF;

   timer->initDone = true;
}

void pwmOutputInit(struct PWMOutput *output)
{
   pwmTimerInit(output->timer);
   pinMode(output->pin, HAL_GPIO_OUTPUT);
   *(output->timer->TCCRA) |= outputModeMask[output->pwmCh];
}

void servo_init_new(void)
{
   for(int i = 0; i < sizeof(pwmOutput)/sizeof(struct PWMOutput); i++)
      pwmOutputInit(&pwmOutput[i]);
}

void servo_write_new(uint8_t ch, uint16_t value)
{
   *(pwmOutput[ch].timer->OCR[pwmOutput[ch].pwmCh])
      = constrain_period(value) << 1;
}

void servo_init(void) {
    // --------------------- TIMER1: CH_1 and CH_2 -----------------------
    pinMode(12,HAL_GPIO_OUTPUT); // CH_1 (PB6/OC1B)
    pinMode(11,HAL_GPIO_OUTPUT); // CH_2 (PB5/OC1A)

    // WGM: 1 1 1 0. Clear Timer on Compare, TOP is ICR1.
    // CS11: prescale by 8 => 0.5us tick
    TCCR1A =(1<<COM1B1) | (1<<COM1A1) | ((1<<WGM11));
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
    ICR1 = 40000; // 0.5us tick => 50hz freq
    OCR1A = 0xFFFF; // Init OCR registers to nil output signal
    OCR1B = 0xFFFF;

    // --------------- TIMER4: CH_3, CH_4, and CH_5 ---------------------
    pinMode(8,HAL_GPIO_OUTPUT); // CH_3 (PH5/OC4C)
    pinMode(7,HAL_GPIO_OUTPUT); // CH_4 (PH4/OC4B)
    pinMode(6,HAL_GPIO_OUTPUT); // CH_5 (PH3/OC4A)

    // WGM: 1 1 1 0. Clear Timer on Compare, TOP is ICR4.
    // CS41: prescale by 8 => 0.5us tick
    TCCR4A = (1<<COM4C1) | (1<<COM4B1) | (1<<COM4A1) | ((1<<WGM41));
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    OCR4A = 0xFFFF; // Init OCR registers to nil output signal
    OCR4B = 0xFFFF;
    OCR4C = 0xFFFF;
    ICR4 = 40000; // 0.5us tick => 50hz freq

    //--------------- TIMER3: CH_6, CH_7, and CH_8 ----------------------
    pinMode(3,HAL_GPIO_OUTPUT); // CH_6 (PE5/OC3C)
    pinMode(2,HAL_GPIO_OUTPUT); // CH_7 (PE4/OC3B)
    pinMode(5,HAL_GPIO_OUTPUT); // CH_8 (PE3/OC3A)

    // WGM: 1 1 1 0. Clear timer on Compare, TOP is ICR3
    // CS31: prescale by 8 => 0.5us tick
    TCCR3A = (1<<COM3C1) | (1<<COM3B1) | (1<<COM3A1) | ((1<<WGM31));
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
    OCR3A = 0xFFFF; // Init OCR registers to nil output signal
    OCR3B = 0xFFFF;
    OCR3C = 0xFFFF;
    ICR3 = 40000; // 0.5us tick => 50hz freq
}

/* Output freq (1/period) control */
void servo_set_freq(uint32_t chmask, uint16_t freq_hz) {
    uint16_t icr = servo_timer_period(freq_hz);

    if ((chmask & ( _BV(CH_1) | _BV(CH_2))) != 0) {
        ICR1 = icr;
    }

    if ((chmask & ( _BV(CH_3) | _BV(CH_4) | _BV(CH_5))) != 0) {
        ICR4 = icr;
    }

    if ((chmask & ( _BV(CH_6) | _BV(CH_7) | _BV(CH_8))) != 0) {
        ICR3 = icr;
    }
}

uint16_t servo_get_freq(uint8_t ch) {
    uint16_t icr;
    switch (ch) {
        case CH_1:
        case CH_2:
            icr = ICR1;
            break;
        case CH_3:
        case CH_4:
        case CH_5:
            icr = ICR4;
            break; 
        case CH_6:
        case CH_7:
        case CH_8:
            icr = ICR3;
            break;
        default:
            return 0;
    }
    /* transform to period by inverse of _time_period(icr). */
    return (2000000UL / icr);
}

/* Output active/highZ control, either by single channel at a time
 * or a mask of channels */
 
void servo_enable_ch(uint8_t ch) {
    switch(ch) {
    case 0: TCCR1A |= (1<<COM1B1); break; // CH_1 : OC1B
    case 1: TCCR1A |= (1<<COM1A1); break; // CH_2 : OC1A
    case 2: TCCR4A |= (1<<COM4C1); break; // CH_3 : OC4C
    case 3: TCCR4A |= (1<<COM4B1); break; // CH_4 : OC4B
    case 4: TCCR4A |= (1<<COM4A1); break; // CH_5 : OC4A
    case 5: TCCR3A |= (1<<COM3C1); break; // CH_6 : OC3C
    case 6: TCCR3A |= (1<<COM3B1); break; // CH_7 : OC3B
    case 7: TCCR3A |= (1<<COM3A1); break; // CH_8 : OC3A
    }
}

void servo_disable_ch(uint8_t ch) {
    switch(ch) {
    case 0: TCCR1A &= ~(1<<COM1B1); break; // CH_1 : OC1B
    case 1: TCCR1A &= ~(1<<COM1A1); break; // CH_2 : OC1A
    case 2: TCCR4A &= ~(1<<COM4C1); break; // CH_3 : OC4C
    case 3: TCCR4A &= ~(1<<COM4B1); break; // CH_4 : OC4B
    case 4: TCCR4A &= ~(1<<COM4A1); break; // CH_5 : OC4A
    case 5: TCCR3A &= ~(1<<COM3C1); break; // CH_6 : OC3C
    case 6: TCCR3A &= ~(1<<COM3B1); break; // CH_7 : OC3B
    case 7: TCCR3A &= ~(1<<COM3A1); break; // CH_8 : OC3A
    }
}

/* constrain pwm to be between min and max pulsewidth. */
uint16_t constrain_period(uint16_t p) {
    if (p > RC_OUTPUT_MAX_PULSEWIDTH) return RC_OUTPUT_MAX_PULSEWIDTH;
    if (p < RC_OUTPUT_MIN_PULSEWIDTH) return RC_OUTPUT_MIN_PULSEWIDTH;
    return p;
}

/* Output, either single channel or bulk array of channels */
void servo_write(uint8_t ch, uint16_t period_us) {
    /* constrain, then scale from 1us resolution (input units)
     * to 0.5us (timer units) */
    uint16_t pwm = constrain_period(period_us) << 1;
    switch(ch)
    {
    case 0:  OCR1B=pwm; break;  // out1
    case 1:  OCR1A=pwm; break;  // out2
    case 2:  OCR4C=pwm; break;  // out3
    case 3:  OCR4B=pwm; break;  // out4
    case 4:  OCR4A=pwm; break;  // out5
    case 5:  OCR3C=pwm; break;  // out6
    case 6:  OCR3B=pwm; break;  // out7
    case 7:  OCR3A=pwm; break;  // out8
    case 9:  OCR5B=pwm; break;  // out10
    case 10: OCR5C=pwm; break;  // out11
    }
}

/* Read back current output state, as either single channel or
 * array of channels. */
uint16_t servo_read(uint8_t ch) {
    uint16_t pwm=0;
    switch(ch) {
    case 0:  pwm=OCR1B; break;      // out1
    case 1:  pwm=OCR1A; break;      // out2
    case 2:  pwm=OCR4C; break;      // out3
    case 3:  pwm=OCR4B; break;      // out4
    case 4:  pwm=OCR4A; break;      // out5
    case 5:  pwm=OCR3C; break;      // out6
    case 6:  pwm=OCR3B; break;      // out7
    case 7:  pwm=OCR3A; break;      // out8
    case 9:  pwm=OCR5B; break;      // out10
    case 10: pwm=OCR5C; break;      // out11
    }
    /* scale from 0.5us resolution (timer units) to 1us units */
    return pwm>>1;

}

void servo_read(uint16_t* period_us, uint8_t len) {
    for (int i = 0; i < len; i++) {
        period_us[i] = servo_read(i);
    }
}

uint16_t servo_timer_period(uint16_t speed_hz) {
    return 2000000UL / speed_hz;
}


#else

#define aileRxPin A8
#define elevatorRxPin A9
#define switchPin A11
#define tuningKnobPin A13
#define rpmPin A14

#define RX_INPUT_PORT PINK
#define RX_INPUT_PCMASK PCMSK2
#define RX_INPUT_PCI 2
#define RX_INPUT_PCI_VECT PCIINT2_vect

struct RxInputRecord aileInput = { aileRxPin, 0 }; 
struct RxInputRecord elevInput = { elevatorRxPin, 1 };
struct RxInputRecord switchInput = { switchPin, 3 };
struct RxInputRecord modeInput = { tuningKnobPin, 5 };

#endif

struct RxInputRecord rpmInput = { rpmPin, 6, false, true };

#define minAlpha paramRecord[stateRecord.model].alphaMin

// #define min(a,b) ((a) < (b) ? (a) : (b))
// #define max(a,b) ((a) > (b) ? (a) : (b))

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

struct RxInputRecord *rxInputRecord[] = 
  { &aileInput, &elevInput, &switchInput, &modeInput, NULL, NULL };

struct RxInputRecord *rxInputPin[8];

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

uint16_t crc16_update(uint16_t crc, uint8_t a)
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

uint16_t crc16(uint16_t initial, const uint8_t *data, int len)
{
  uint16_t crc = initial;
  
  for(int i = 0; i < len; i++)
    crc = crc16_update(crc, data[i]);
    
  return crc;
}

uint16_t crc16OfRecord(uint16_t initial, const uint8_t *record, int size)
{
  return crc16(initial, &record[sizeof(uint16_t)], size - sizeof(uint16_t));
}

uint16_t paramRecordCrc(struct ParamRecord *record)
{
  return crc16OfRecord(0xFFFF, (uint8_t*) record, sizeof(*record));
}

struct NVStateRecord stateDefaults = { 0, 400, 0, false, 0 };

uint16_t stateRecordCrc(struct NVStateRecord *record)
{
  return crc16OfRecord(0xFFFF, (uint8_t*) record, sizeof(*record));
}

// NV store layout

#define MAX_MODELS   4

const int paramOffset = 0;
const int stateOffset = MAX_MODELS*sizeof(ParamRecord);
const int logOffset = stateOffset+sizeof(NVStateRecord);

struct ParamRecord paramRecord[MAX_MODELS];
struct NVStateRecord stateRecord;

boolean handleFailure(const char *name, boolean fail, boolean *warn, boolean *failed, int *count)
{
  if(*failed)
    return true;
    
  if(fail) {
    *warn = true;
    
    consoleNote("Bad ");
    consolePrintLn(name);
    
    if(++(*count) > 10) {
      consoleNote("");
      consolePrint(name);
      consolePrintLn(" failed");
      *failed = true;
    }
  } else {    
    if(*count > 0) {
      consoleNote("");
      consolePrint(name);
      consolePrintLn(" recovered");
      *count = 0;
    }
  }
  
  return fail;
}

int eepromFailCount = 0;
uint32_t lastWriteTime;

void waitEEPROM(uint32_t addr)
{
  if(micros() - lastWriteTime > EXT_EEPROM_LATENCY)
    // We're cool
    return;
    
  // Write latency not met, wait for acknowledge

  handleFailure("EEPROM wait",
     I2c.wait((uint8_t) (paramRecord[stateRecord.model].i2c_24L256
     			+ (uint8_t) ((addr>>16) & 0x7))) != 0, 
                  	&eepromWarn, &eepromFailed, &eepromFailCount);
}

void writeEEPROM(uint32_t addr, const uint8_t *data, int bytes) 
{
  if(eepromFailed)
    return;
    
  waitEEPROM(addr);
  boolean fail = I2c.write(  (uint8_t) paramRecord[stateRecord.model].i2c_24L256 + (uint8_t) ((addr>>16) & 0x7), 
                             (uint16_t) (addr & 0xFFFFL), 
                             data, bytes) != 0;
  handleFailure("EEPROM write", fail, &eepromWarn, &eepromFailed, &eepromFailCount);
  lastWriteTime = micros();
}
 
boolean readEEPROM(uint32_t addr, uint8_t *data, int size) 
{
  if(eepromFailed)
    return true;
    
  waitEEPROM(addr);

  boolean fail = I2c.read((uint8_t) paramRecord[stateRecord.model].i2c_24L256 + (uint8_t) ((addr>>16) & 0x7), (uint16_t) (addr & 0xFFFFL), data, size) != 0;
  
  return handleFailure("EEPROM read", fail, &eepromWarn, &eepromFailed, &eepromFailCount);

}

uint8_t cacheData[EXT_EEPROM_PAGE];
boolean cacheFlag[EXT_EEPROM_PAGE];
boolean cacheValid, cacheModified;
uint32_t cacheTag;

void cacheFlush(void)
{
  if(!cacheModified)
    return;
    
  int i = 0;
  
  do {
    while(!cacheFlag[i] && i < EXT_EEPROM_PAGE)
      i++;
    
    if(i < EXT_EEPROM_PAGE) {
      int l = 0;
      
      while(cacheFlag[i+l] && i+l < EXT_EEPROM_PAGE) {
        cacheFlag[i+l] = false;
        l++;
      }

      writeEEPROM(cacheTag + i, &cacheData[i], l);
        
      i += l;
    }
  } while(i < EXT_EEPROM_PAGE);
  
  cacheModified = false;
}

#define CACHE_TAG(a) ((a) & PAGE_MASK)

boolean cacheHit(uint32_t addr)
{
  return CACHE_TAG(addr) == cacheTag;
}

void cacheAlloc(uint32_t addr)
{
  cacheFlush();
  cacheTag = CACHE_TAG(addr);
  cacheValid = false;
}

void cacheWriteLine(uint32_t addr, const uint8_t *value, int size)
{  
  if(!cacheHit(addr))
    cacheAlloc(addr);
    
  for(int i = 0; i < size; i++) {
    cacheData[(addr & ~PAGE_MASK) + i] = value[i];
    cacheFlag[(addr & ~PAGE_MASK) + i] = true;
  }
  
  cacheValid = false;
  cacheModified = true;
  logBytesCum += size;
}

void cacheWrite(uint32_t addr, const uint8_t *value, int size)
{
  if(CACHE_TAG(addr) != CACHE_TAG(addr+size-1)) {
    uint32_t split = CACHE_TAG(addr+size-1);
    
    cacheWriteLine(addr, value, split - addr);
    cacheWriteLine(split, &value[split - addr], size - (split - addr));
  } else
    cacheWriteLine(addr, value, size);
}

void cacheRead(uint32_t addr, uint8_t *value, int size) 
{
  if(cacheModified || !cacheHit(addr))
    cacheAlloc(addr);
  
  if(!cacheValid) {
    if(readEEPROM(cacheTag, cacheData, EXT_EEPROM_PAGE)) {
      for(int i = 0; i < EXT_EEPROM_PAGE; i++)
        cacheData[i] = 0xFF;
    }
    cacheValid = true;
  }
  
  for(int i = 0; i < size; i++)
    value[i] = cacheData[(addr & ~PAGE_MASK) + i];  
}

boolean read5048B(uint8_t addr, uint8_t *storage, int bytes) 
{
#ifdef USE_FUCKING_CRAP_WIRE_I2C_LIBRARY

  Wire.beginTransmission((uint8_t) paramRecord[stateRecord.model].i2c_5048B);
  Wire.write(addr);
  Wire.endTransmission(false);
  
  Wire.requestFrom((uint8_t) paramRecord[stateRecord.model].i2c_5048B,  (uint8_t) bytes);

  int count = Wire.available();

  for(int i = 0; i < count; i++)
    storage[i] = Wire.read();

  return count == bytes;   
#else
  return I2c.read(paramRecord[stateRecord.model].i2c_5048B, addr, storage, bytes) == 0;
#endif
}

const uint8_t addr4525_c = 0x28;

boolean read4525DO(uint8_t *storage, int bytes) 
{
  return I2c.read(addr4525_c, NULL, 0, storage, bytes) == 0;
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

void readBlock(char *ptr, uint16_t addr, int size)
{
  for(int i = 0; i < size; i++)
    ptr[i] = EEPROM.read(addr+i);
}

void writeBlock(const char *ptr, uint16_t addr, int size)
{
  for(int i = 0; i < size; i++)
    EEPROM.write(addr+i, ptr[i]);
}

void readParams(void)
{
  readBlock((char*) paramRecord, paramOffset, sizeof(paramRecord));
}

void storeParams(void)
{
  paramRecord[stateRecord.model].crc = paramRecordCrc(&paramRecord[stateRecord.model]);
  writeBlock((const char*) &paramRecord[stateRecord.model], paramOffset+stateRecord.model*sizeof(struct ParamRecord), sizeof(struct ParamRecord));
}

void readNVState(void)
{
  readBlock((char*) &stateRecord, stateOffset, sizeof(stateRecord));
}

void storeNVState(void)
{
  stateRecord.crc = stateRecordCrc(&stateRecord);
  writeBlock((const char*) &stateRecord, stateOffset, sizeof(stateRecord));
}

void logWrite(int32_t index, const uint16_t *value, int count)
{
  if(logState == init_c || logSize < 1)
    return;
    
  cacheWrite(index*sizeof(uint16_t), (const uint8_t*) value, count*sizeof(uint16_t));
  logLen = -1L;
}

void logWrite(int32_t index, const uint16_t value)
{
  logWrite(index, &value, 1);
}

uint16_t logRead(int32_t index)
{
  uint16_t entry = 0;
  cacheRead(index*sizeof(entry), (uint8_t*) &entry, sizeof(entry));
  return entry;
}

RunningAvgFilter alphaFilter;
AlphaBuffer alphaBuffer, pressureBuffer;

float elevOutput = 0, aileOutput = 0, flapOutput = 0, gearOutput = 1, brakeOutput = 0;
Servo elevatorServo, aileServo, flapServo, gearServo, brakeServo;  
  
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
#define uint16_tOKEN(t) (TOKEN_MASK | (t))
#define ENTRY_VALUE(v) (((uint16_t) v) & VALUE_MASK)
#define ENTRY_IS_TOKEN(e) ((e) & TOKEN_MASK)

typedef enum { t_stamp,
               t_start, 
               t_mark, 
               t_channel = t_stamp + (1<<8),
               t_delta = t_stamp + (1<<14)
            } LogToken_t;

#define logIndex(i) ((logPtr + logSize + (i)) % logSize)

void logInit(void)
{
  uint32_t eepromSize = 1;
  uint8_t dummy;
  
  logSize = 0;
    
  while(!readEEPROM(eepromSize, &dummy, 1))
    eepromSize *= 2;
    
  if(!readEEPROM(eepromSize-1, &dummy, 1)) {
    logSize = eepromSize/sizeof(uint16_t);

    consoleNote("Inferred log size = ");
    consolePrint(logSize/(1<<10));
    consolePrintLn("k entries");
  } else
    consoleNoteLn("Log EEPROM failed");
  
  logState = init_c;
  logLen = -1;
}

void logCommit(int bytes)
{
  if(logState == init_c)
    return;
    
  logPtr = logIndex(bytes);
}

boolean logDirty = false;

void logEnter(const uint16_t *value, int count)
{
  if(logState == init_c)
    return;
    
  logWrite(logIndex(0), value, count);
  uint16_t buffer[2] = { uint16_tOKEN(t_stamp), logEndStamp };
  logWrite(logIndex(count), buffer, 2);
  logCommit(count);
  
  logDirty = true;
}

void logEnter(uint16_t value)
{
  logEnter(&value, 1);
}

void logClear(void)
{
  if(logState == init_c)
    return;
    
  consoleNoteLn("Log CLEARED");
    
  stateRecord.logStamp++;
  logEnter(uint16_tOKEN(t_start));
  storeNVState();
}
  
  int prevCh = -1;

void logWithCh(int ch, uint16_t value)
{
  if(!logEnabled || logState != run_c)
    return;

  value = ENTRY_VALUE(value);    // Constrain to valid range
  
  if(!logChannels[ch].tick && value == logChannels[ch].value)
    // Repeat value, not stored
    
    return;
            
  if(ch == prevCh) {
    // Same channel as previous, store as delta
    
    logEnter(uint16_tOKEN(t_delta) | (DELTA_MASK & ((value - logChannels[ch].value)>>1)));
    
  } else if(prevCh > -1 && ch == prevCh + 1) {
    // Channel prediction good, just store value
    
    logEnter(value);
  } else {
    // Set channel first
    
    uint16_t buffer[2] = { uint16_tOKEN(t_channel + ch), value };
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

void logMark(void)
{
  logEnter(uint16_tOKEN(t_mark));
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
  
void logPrintValue(float v)
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
  if(logState == init_c) {
    consoleNoteLn("Log initialization not completed yet");
    return;
  }
  
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
    
    while(len < logSize-1 && logRead(logIndex(-len-1)) != uint16_tOKEN(t_start)) {
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

  while(len < logSize-1 && logRead(logIndex(-len-1)) != uint16_tOKEN(t_start))
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

#ifndef MEGAMINI
uint8_t log2Table[1<<8];
#endif

#ifdef RX_INPUT_PCI_VECT

ISR(RX_INPUT_PCI_VECT)
{
  static uint8_t prevState;
  uint8_t state = RX_INPUT_PORT, event = (state ^ prevState) & RX_INPUT_PCMASK;
  prevState = state;

  uint32_t current = micros();
  
  while(event) {
    uint8_t i = log2Table[event];
    uint8_t mask = 1<<i;
  
    if(!rxInputPin[i]) {
      pciWarn = true;
    } else if(rxInputPin[i]->freqOnly) {
      rxInputPin[i]->pulseCount += (state & mask) ? 1 : 0;
    } else if(state & mask) {
      rxInputPin[i]->pulseStart = current;
    } else if(rxInputPin[i]->pulseStart > 0) {
      uint32_t width = current - rxInputPin[i]->pulseStart;
      rxInputPin[i]->pulseWidthAcc += width;
      rxInputPin[i]->pulseCount++;      
      rxInputPin[i]->alive = true;
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
    
  consoleNote("  State record CRC = ");
  consolePrint(stateRecord.crc);
    
  if(stateRecord.crc != stateRecordCrc(&stateRecord)) {
    consolePrintLn(" CORRUPT, using defaults"); 
    stateRecord = stateDefaults;
  } else
    consolePrintLn(" OK");

  consoleNote("Current model is ");
  consolePrintLn(stateRecord.model);
  
  // I2C
  
#ifdef USE_FUCKING_CRAP_WIRE_I2C_LIBRARY
  Wire.begin();  
#else
  I2c.begin();
  I2c.setSpeed(true);
  I2c.pullup(true);
  I2c.timeOut(2+EXT_EEPROM_LATENCY/1000);
#endif

  // Param record
  
  readParams();

  for(int i = 0; i < MAX_MODELS; i++) {
    consoleNote("MODEL ");
    consolePrintLn(i);
    
    if(paramRecordCrc(&paramRecord[i]) != paramRecord[i].crc) {
      consoleNoteLn("  Param record corrupt, using defaults"); 
      paramRecord[i] = paramDefaults;
    }
    
    printParams(&paramRecord[i]);    
  }

  // Set I2C speed
  
  TWBR = max(paramRecord[stateRecord.model].clk_24L256,
              paramRecord[stateRecord.model].clk_5048B);
              
  // Rx input

  consoleNoteLn("Initializing RX inputs");

#ifdef MEGAMINI
  // PPM input
  
  pinMode(48, INPUT);
  
  ppm_input_init();
  
#else
  pinMode(elevatorRxPin, INPUT);
  pinMode(aileRxPin, INPUT);
  pinMode(switchPin, INPUT);
  pinMode(tuningKnobPin, INPUT);
  pinMode(rpmPin, INPUT_PULLUP);  

  for(int i = 1; i < (1<<8); i++) {
    int j = 7;
    while(((1<<j) & i) == 0 && j > 0)
      j--;
    log2Table[i] = j;
  }

  PCICR |= 1<<RX_INPUT_PCI;
  
  for(int i = 0; rxInputRecord[i]; i++) {
    RX_INPUT_PCMASK |= (1<<rxInputRecord[i]->port);
    rxInputPin[rxInputRecord[i]->port] = rxInputRecord[i];
  }
#endif
  
  aileFilter.setWindowLen(3);
  elevFilter.setWindowLen(3);

  // RPM sensor int control
  
  // rpmMeasure(stateRecord.logRPM);
  
  // Servos

  consoleNoteLn("Initializing servos");

#ifdef MEGAMINI
  servo_init_new();
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

  // Initialise log
  
  logInit();
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
#if defined(RX_INPUT_PCMASK) && defined(rpmPin)
  if(on)
    RX_INPUT_PCMASK |= 1<<rpmInput.port;
  else
    RX_INPUT_PCMASK &= ~(1<<rpmInput.port);
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
    logEndStamp = ENTRY_VALUE(-100);
    logPtr = logSize - 1;

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
    consoleNoteLn("");

    consoleNote("Log write bandwidth = ");
    consolePrint(logBandWidth);
    consolePrintLn(" bytes/sec");
    break;

  case c_reset:
    pciWarn = alphaWarn = alphaFailed = pusher.warn = elevController.warn = alphaBuffer.warn
      = eepromWarn = eepromFailed = false;
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
      paramRecord[stateRecord.model] = paramDefaults;
      consoleNoteLn("Defaults restored");
      printParams(&paramDefaults);
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

#define CONTROL_HZ 150
#define ALPHA_HZ (CONTROL_HZ*4)
#define ACTUATOR_HZ CONTROL_HZ
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

void logSaveTask(float currentTime)
{
  switch(logState) {
    case stop_c:
      if(logEnabled) {
        consoleNoteLn("Logging STARTED");
      
        logState = run_c;
    
        for(int i = 0; i < 4; i++)
          logMark();
  
        logAlpha();
        logAttitude();
        logInput();
        logActuator();
        logConfig();
        logPosition();
        logRPM();
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
    
  if(inputValid(&modeInput))
    tuningKnobValue = inputValue(&modeInput);
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

void measurementTask(float currentTime)
{
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
   /*
  if(initCount > 0) {
    if(rxElevatorAlive && rxAileronAlive)
      initCount--;
      
    return;
  }
*/
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
/*
    consolePrint(" ppm_ch = ");
    consolePrint(ppmNumChannels);
*/    
/*
    consolePrint(" aileStick = ");
    consolePrint(aileStick);
    consolePrint(" elevStick = ");
    consolePrint(elevStick);
*/    
    consolePrint(" roll = ");
    consolePrint(rollAngle);
    consolePrint(" (rate = ");
    consolePrint(rollRate);
    consolePrint(") pitch = ");
    consolePrint(pitchAngle);
    consolePrint(" (rate = ");
    consolePrint(pitchRate);
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
#ifdef MEGAMINI
    servo_write_new(0, 1500 + 500*clamp(paramRecord[stateRecord.model].aileDefl*aileOutput 
      + paramRecord[stateRecord.model].aileNeutral, -1, 1));

    servo_write_new(1, 1500 + 500*clamp(paramRecord[stateRecord.model].elevDefl*elevOutput 
      + paramRecord[stateRecord.model].elevNeutral, -1, 1));
                              
    servo_write_new(2, 1500 + 500*(clamp(paramRecord[stateRecord.model].flapNeutral 
                        + flapOutput*paramRecord[stateRecord.model].flapStep, -1, 1)));                              

    servo_write_new(3, 1500 - 500*(gearOutput*2-1));

    servo_write_new(4, 1500 + 500*clamp(paramRecord[stateRecord.model].brakeNeutral + 
                                paramRecord[stateRecord.model].brakeDefl*brakeOutput, -1, 1));
#else      
    aileServo.writeMicroseconds(1500 + 500*clamp(paramRecord[stateRecord.model].aileDefl*aileOutput 
      + paramRecord[stateRecord.model].aileNeutral, -1, 1));
      
    elevatorServo.writeMicroseconds(1500 + 500*clamp(paramRecord[stateRecord.model].elevDefl*elevOutput 
      + paramRecord[stateRecord.model].elevNeutral, -1, 1));
                              
    flapServo.writeMicroseconds(1500 + 500*(clamp(paramRecord[stateRecord.model].flapNeutral 
                        + flapOutput*paramRecord[stateRecord.model].flapStep, -1, 1)));                              

    gearServo.writeMicroseconds(1500 - 500*(gearOutput*2-1));

    brakeServo.writeMicroseconds(1500 + 500*clamp(paramRecord[stateRecord.model].brakeNeutral + 
                                paramRecord[stateRecord.model].brakeDefl*brakeOutput, -1, 1));
#endif                                
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

void blinkTask(float currentTime)
{
  float ledRatio = testMode ? 0.0 : logState == init_c ? 1.0 : (mode.sensorFailSafe || !armed) ? 0.5 : alpha > 0.0 ? 0.90 : 0.10;
  static int tick = 0;
  
  tick = (tick + 1) % (LED_TICK/LED_HZ);
    
  if(tick < ledRatio*LED_TICK/LED_HZ) {
    STABLEPIN_ON;
  } else {
    STABLEPIN_OFF;
  }
/*  
  if(abs(elevStick) < 0.2)
    elevStick = 0.5;
  
  if(tick == 0)
    elevStick = -elevStick;
    */
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

void backgroundTask(long durationMicros)
{
  long current = micros();
  static int32_t endPtr = -1, searchPtr = 0;
  static boolean endFound = false;
  
  if(logState == init_c) {
    while(searchPtr < logSize) {
      if(logRead(searchPtr) == uint16_tOKEN(t_stamp)) {
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
      
      if(micros() - current > durationMicros)
        // Stop for now
        return;
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
  } else
    delayMicroseconds(durationMicros);
}

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
