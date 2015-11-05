#include <Arduino.h>

#define RC_OUTPUT_MIN_PULSEWIDTH 400
#define RC_OUTPUT_MAX_PULSEWIDTH 2100

typedef enum { COMnA = 0, COMnB = 1, COMnC = 2 } PWM_Ch_t;

struct HWTimer {
  volatile uint8_t *TCCRA, *TCCRB;
  volatile uint16_t *ICR;
  volatile uint16_t *OCR[3]; // 0... 2 = A ... C
  boolean initDone;
};

struct PWMOutput {
  int pin;
  struct HWTimer *timer;
  PWM_Ch_t pwmCh; // COMnA / COMnB / COMnC
};

void pwmTimerInit(struct HWTimer *timer);
void pwmEnable(struct PWMOutput *output);
void pwmDisable(struct PWMOutput *output);
void pwmOutputInit(struct PWMOutput *output);
void pwmOutputInitList(struct PWMOutput output[], int num);
void pwmOutputWrite(struct PWMOutput *output, uint16_t value);


