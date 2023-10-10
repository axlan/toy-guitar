
#include "button_interface.h"

#include <Arduino.h>

#define PIN_COM_OUT2 32 // Used to differentiate buttons with common output.
#define PIN_IN4 33      // Connected to VCC through button 1. Connected to PIN_COM_OUT2 through Button 5.
#define PIN_IN3 13      // Connected to VCC through button 2. Connected to PIN_COM_OUT2 through Button 6.
#define PIN_IN2 14      // Connected to VCC through button 3.
#define PIN_IN1 12      // Connected to VCC through button 4.

void InitButtonPins()
{
  pinMode(PIN_COM_OUT2, OUTPUT);
  pinMode(PIN_IN1, INPUT_PULLDOWN);
  pinMode(PIN_IN2, INPUT_PULLDOWN);
  pinMode(PIN_IN3, INPUT_PULLDOWN);
  pinMode(PIN_IN4, INPUT_PULLDOWN);

  // Only get HIGH from presses to buttons 1-4.
  digitalWrite(PIN_COM_OUT2, LOW);
}

uint8_t ReadButtons()
{
  uint8_t ret = 0;
  digitalWrite(PIN_COM_OUT2, LOW);
  // Read buttons connected to VCC.
  ret |= digitalRead(PIN_IN4) << 0;
  ret |= digitalRead(PIN_IN3) << 1;
  ret |= digitalRead(PIN_IN2) << 2;
  ret |= digitalRead(PIN_IN1) << 3;

  // Check if buttons 5,6 are pressed. This can't differentiate between the other button these
  // share a pin with. I could use another digital pin instead of VCC, but it didn't matter
  // for this application. In the case where the VCC connected button is pressed, only it will register.
  digitalWrite(PIN_COM_OUT2, HIGH);
  if (!(ret & 0x1))
  {
    ret |= digitalRead(PIN_IN2) << 2;
  }
  if (!(ret & 0x2))
  {
    ret |= digitalRead(PIN_IN1) << 3;
  }
  digitalWrite(PIN_COM_OUT2, LOW);
  return ret;
}
