#include "Console.h"

static void newline(void);

void consoleNote(const char *s)
{
  consolePrint("// ");
  consolePrint(s);
}

void consoleNoteLn(const char *s)
{
  consoleNote(s);
  newline();
}

void consolePrint(const char *s)
{
  if(talk)
    Serial.print(s);
}

void consolePrint(float v, int p)
{
  if(talk)
    Serial.print(v, p);
}

void consolePrint(float v)
{
  consolePrint(v, 2);
}

void consolePrint(double v, int p)
{
  if(talk)
    Serial.print(v, p);
}

void consolePrint(double v)
{
  consolePrint(v, 2);
}

void consolePrint(int v)
{
  if(talk)
    Serial.print(v);
}

void consolePrint(unsigned int v)
{
  if(talk)
    Serial.print(v);
}

void consolePrint(long v)
{
  if(talk)
    Serial.print(v);
}

void consolePrint(unsigned long v)
{
  if(talk)
    Serial.print(v);
}

void consolePrint(uint8_t v)
{
  if(talk)
    Serial.print(v);
}

void newline(void)
{
  consolePrint("\n");
}

void consolePrintLn(const char *s)
{
  consolePrint(s);
  newline();
}

void consolePrintLn(float v)
{
  consolePrint(v);
  newline();
}

void consolePrintLn(float v, int p)
{
  consolePrint(v, p);
  newline();
}

void consolePrintLn(double v)
{
  consolePrint(v);
  newline();
}

void consolePrintLn(double v, int p)
{
  consolePrint(v, p);
  newline();
}

void consolePrintLn(int v)
{
  consolePrint(v);
  newline();
}

void consolePrintLn(unsigned int v)
{
  consolePrint(v);
  newline();
}

void consolePrintLn(uint8_t v)
{
  consolePrint(v);
  newline();
}

void consolePrintLn(long v)
{
  consolePrint(v);
  newline();
}

void consolePrintLn(unsigned long v)
{
  consolePrint(v);
  newline();
}

