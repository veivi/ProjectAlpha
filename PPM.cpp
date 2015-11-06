#include "PPM.h"
#include "Interrupt.h"

#define AVR_RC_INPUT_MAX_CHANNELS 10
#define AVR_RC_INPUT_MIN_CHANNELS 5

/*
  mininum pulse width in microseconds to signal end of a PPM-SUM
  frame. This value is chosen to be smaller than the default 3000 sync
  pulse width for OpenLRSng. Note that this is the total pulse with
  (typically 300us low followed by a long high pulse)
 */

#define AVR_RC_INPUT_MIN_SYNC_PULSE_WIDTH 2700

static uint16_t _pulse_capt[AVR_RC_INPUT_MAX_CHANNELS];

uint8_t ppmNumChannels;
uint32_t ppmFrames;
boolean ppmWarn;

static struct RxInputRecord **inputRecords;
static int numInputs;

static void handlePPMInput(const uint16_t *pulse, int numCh)
{
  ppmNumChannels = numCh;
  ppmFrames++;

  for(int i = 0; i < numCh; i++) {
    if(inputRecords[i]) {
       inputRecords[i]->alive = true;
       inputRecords[i]->pulseCount++;
       inputRecords[i]->pulseWidthAcc += pulse[i]/2;              
     }
  }
}

ISR(TIMER5_CAPT_vect) {
    static uint16_t icr5_prev;
    static uint8_t  channel_ctr;

    const uint16_t icr5_current = ICR5;
    uint16_t pulse_width;
    
    if (icr5_current < icr5_prev) {
        pulse_width =  0x10000L + icr5_current - icr5_prev;
    } else {
        pulse_width = icr5_current - icr5_prev;
    }

    if (pulse_width > AVR_RC_INPUT_MIN_SYNC_PULSE_WIDTH*2) {
        // sync pulse
	
        if( channel_ctr < AVR_RC_INPUT_MIN_CHANNELS )
	   ppmWarn = true;
	else
           handlePPMInput(_pulse_capt, channel_ctr);
 
        channel_ctr = 0;
    } else if (channel_ctr < numInputs)
      	_pulse_capt[channel_ctr++] = pulse_width;

    icr5_prev = icr5_current;
}

void ppmInputInit(struct RxInputRecord *inputs[], int num)
{

  inputRecords = inputs;
  numInputs = min(num, AVR_RC_INPUT_MAX_CHANNELS);
  
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
  OCR5A  = 0xFFFF;

  /* Enable input capture interrupt */
  TIMSK5 = 1<<ICIE5;
     
  PERMIT;
}
