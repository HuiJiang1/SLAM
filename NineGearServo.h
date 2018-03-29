#ifndef NineGearServo_h
#define NineGearServo_h

#include <Servo.h>
#include "Phototransistor.h"

class NineGear : public Servo
{
  private:
    int pin;
    bool clockwise;
    int anglePosition;
  
  public:
    NineGear(const int pin) {
      this->pin = pin;
      anglePosition = 90;
    }

    int getPin() {
      return pin;
    }

    void setPosition(char ) {
      
    }

    void direct(bool clockwise) {
      clockwise = clockwise;
    }

    void sweep() {
      if (clockwise) {
        if (anglePosition < 180) {
          anglePosition++;
        }
        else {
          anglePosition--;
          clockwise = false;
        }
      }
      else {
        if (anglePosition > 0) {
          anglePosition--;
        }
        else {
          anglePosition++;
          clockwise = true;
        }
      }
      
      write(anglePosition);
    }
};

#endif
