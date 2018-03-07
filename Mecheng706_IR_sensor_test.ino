/* To control the Rover, copy and paste the code below into the Arduino
 software. Ensure the motors are connected to the correct pins. The
 code does not factor in encoders at this time*/

#include <Servo.h>
#include "main.h"
#include <FaBo9Axis_MPU9250.h> // have to include this library

STATE initialising();
STATE running();
STATE stopped();
//float read_sharp_IR_sensor(SHARP , int);

//Servo servo_test(4);  // create servo object to control a servo

void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);  //Open serial comms to bluetooth module
  //Serial1.begin(115200);  //Open serial comms to bluetooth module
}

void loop(void) //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING:
      machine_state =  running();
      break;
    case STOPPED:
      machine_state =  stopped();
      break;
  };

}


STATE initialising() {
  //Do some initialising
  Serial.println("INITIALISING....");
 // Serial1.println("INITIALISING bluetooth....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  return RUNNING;
}

STATE running() {
  int level_bar = 0;
  static unsigned long previous_millis;
  static unsigned long sprevious_millis;
  static float avg;
  static int stotal;

  int i;
  static int avgcount;
  if (millis() - sprevious_millis > 10) { // read the IR sensor every 10 seconds
    sprevious_millis = millis();
  avgcount++;
  if (avgcount > 30) {
    avgcount = 0;
    avg = stotal / 30.0;
    stotal= 0;
  }
  else
    stotal +=  read_sharp_IR_sensor(SHARP_DX, A5);
  }

  if (millis() - previous_millis > 500) {
    previous_millis = millis();

    //int raw = analogRead(A1);
    Serial.print("s1:");
    Serial.println(avg);
    
    //Serial1.print("s1:");
    //Serial1.println(avg);
    /* Serial1.print(" s2:");
     Serial1.print(read_sharp_IR_sensor(SHARP_Ya, A2));
     Serial1.print(" s3:");
     Serial1.print(read_sharp_IR_sensor(SHARP_Ya, A3));
     Serial1.print(" s4:");
     Serial1.println(read_sharp_IR_sensor(SHARP_YA, A4));*/


    //2D120X 40mm - 300mm http://www.phidgets.com/products.php?product_id=3520
    //Distance = 2076.0 / (raw - 11) + 2;
    //
    //2Y0A21 100mm - 800mm http://www.phidgets.com/products.php?product_id=3521
    //Distance = 4800.0 / (raw - 20);
    //
    //2Y0A02 200mm - 1500mm http://www.phidgets.com/products.php?product_id=3522
    //Distance = 9462 / (raw - 16.92);
    //

    /*Serial1.print("\r[");
    for (i = 0; i < 10; i++) {
      if (level_bar > i)
        Serial1.print("#");
      else
        Serial1.print("-");
    }
    //Serial.print(char(219));

    Serial1.print("]");
    Serial1.print(puntualDistance, 1);*/
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    level_bar++;
    if (level_bar > 10)
      level_bar = 0;
  }
  return RUNNING;
}

STATE stopped() {

}

int read_sharp_IR_sensor(SHARP which_one, int which_analog_pin) {
  int temp_dis;
  switch (which_one) {
    case SHARP_DX:
      //2D120X 4cm - 30cm http://www.phidgets.com/products.php?product_id=3520
      temp_dis = 2076.0 / (analogRead(which_analog_pin) - 11) + 2;
      if (temp_dis < 0.0)
        return 30.1;
      if (temp_dis >= 0.0 && temp_dis <= 4.0)
        return 3.9;
      if (temp_dis > 30.0)
        return 30.1;
      return temp_dis;
      break;
    case SHARP_Ya:
      //2Y0A21 10cm - 80cm http://www.phidgets.com/products.php?product_id=3521
      temp_dis =  4800.0 / (analogRead(which_analog_pin) - 20);
      if (temp_dis < 0.0)
        return 80.1;
      if (temp_dis >= 0.0 && temp_dis <= 10.0)
        return 9.9;
      if (temp_dis > 80.0)
        return 80.1;
      return temp_dis;
      break;
    case SHARP_YA:
      //2Y0A02 20cm - 150cm http://www.phidgets.com/products.php?product_id=3522
      temp_dis =  9462.0 / (analogRead(which_analog_pin) - 16.92);
      if (temp_dis < 0.0)
        return 150.1;
      if (temp_dis >= 0.0 && temp_dis <= 20.0)
        return 19.9;
      if (temp_dis > 150.0)
        return 150.1;
      return temp_dis;
      break;
  }
}



