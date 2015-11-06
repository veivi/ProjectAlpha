#ifndef FILTER_H
#define FILTER_H

#include <Arduino.h>

const int windowLenMax = 8;

float sign(float x);
float clamp(float value, float a, float b);
float mixValue(float mixRatio, float a, float b);

class RunningAvgFilter {
  public:
    void setWindowLen(int l);
    float input(float v);
    float output();
    
  private:
    float memory[windowLenMax], sum;
    int windowLen;
    int ptr;
};

const int MedianWindow_c = 3;
  
class Median3Filter {  
  public:
    void input(float v);
    float output();
    
  private:
    float memory[MedianWindow_c];
    int ptr;
};

class AlphaBuffer {
public:
  float output(void);
  void input(float v);
  boolean warn;
  
private:
  float sum, value;
  int length;
};

#endif
