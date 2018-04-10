#ifndef Phototransistor_h
#define Phototransistor_h

#include "Arduino.h"

class Phototransistor
{
  private:
    char type;
    int pin;
    float total;
    float average;
    
    public:
    
    Phototransistor(char type, const int pin) {
      type = type;
      this->pin = pin;
    }

    int getPin(){
      return pin;
    }

    float readVoltage() {
      float voltage = analogRead(pin) * (5.0/1023.0);
      return voltage;
    }
    
   void addToTotal(float voltage) {
      total += voltage;
    }

   float getTotal() {
      return total;
    }

    void resetTotal() {
      total = 0;
    }

    void calcAvg(float noOfReads) {
      average = total/noOfReads;
    }

    float getAvg() {
      return average;
    }

    void resetAvg() {
      average = 0;
    }

    void tenMillisecFunc() {
      float voltage = readVoltage();
      addToTotal(voltage);
    }
	
    // This function require previous running of tenMillisecFunc() <50> times 
    bool fiveHundredMillisecFunc() {
      calcAvg(50);
      resetTotal();
      switch(type) {
        case 'B':
          if (average > 1.0)
            return true;
          else
            return false;
          break;
        case 'L':
          if (average > 1.5)
            return true;
          else
            return false;
          break;
        default:
          break;
      }
    }
};

#endif

