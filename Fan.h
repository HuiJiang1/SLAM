#ifndef Fan_h
#define Fan_h

#include "Phototransistor.h"

class Fan
{
  private:
    int pin;
    bool on;

  public:
    Fan(const int pin) {
      this->pin = pin;
    }

    int getPin() {
      return pin;
    }
    
    void needsToBeOn(bool yes) {
      if (yes)
        digitalWrite(pin, LOW);
      else
        digitalWrite(pin, HIGH);
    }
};

#endif
