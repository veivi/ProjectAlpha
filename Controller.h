#include <Arduino.h>
#include "Filter.h"
#include "Console.h"

class Controller {
public:
  void setPID(float P, float I, float D);
  float getP(void);
  float getI(void);
  float getD(void);
  void setZieglerNicholsPID(float Ku, float Tu);
  void setZieglerNicholsPI(float Ku, float Tu);
  void getZieglerNicholsPID(float *Ku, float *Tu);
  void reset(float value, float err);
  void input(float err, float d);

  float output(void);
  
  boolean warn;
private:
  float I, delta, prevErr, Kp, Ki, Kd;
  Median3Filter errorFilter;
};

