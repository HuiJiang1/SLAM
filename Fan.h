#ifndef FAN
#define FAN

#include "Phototransistor.h"

class Fan
{
  private:
    int pin;

  public:
    Fan(const int pin) {
      this->pin = pin;
    }

    int getPin() {
      return pin;
    }
    
    void needsToBeOn(bool yes) {
      if (yes){
        digitalWrite(pin, LOW);
      else
        digitalWrite(pin, HIGH);
      }
    }
};

#endif
