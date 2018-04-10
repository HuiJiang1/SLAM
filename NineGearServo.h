#ifndef NineGearServo
#define NineGearServo

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

    void directCW(bool clockwise) {
      clockwise = clockwise;
    }

    // Basically this function sweep the servo by 1 degreee for each iteration
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
