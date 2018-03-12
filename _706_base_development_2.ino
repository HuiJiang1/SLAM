#include <MPU9250_RegisterMap.h>
#include <SparkFunMPU9250-DMP.h>

/*
  MechEng 706 Base Code that is undergoing construction..... 12/03/18 GRRRRRRR

  This code provides basic movement and sensor reading for the MechEng 706 Mecanum Wheel Robot Project

  Hardware: 
  * Arduino Mega2560 https://www.arduino.cc/en/Guide/ArduinoMega2560
  * MPU-9250 https://www.sparkfun.com/products/13762
  * Ultrasonic Sensor - HC-SR04 https://www.sparkfun.com/products/13959
  * Infrared Proximity Sensor - Sharp https://www.sparkfun.com/products/242
  * Infrared Proximity Sensor Short Range - Sharp https://www.sparkfun.com/products/12728
  * Servo - Generic (Sub-Micro Size) https://www.sparkfun.com/products/9065
  * Vex Motor Controller 29 https://www.vexrobotics.com/276-2193.html 
  * Vex Motors https://www.vexrobotics.com/motors.html
  * Turnigy nano-tech 2200mah 2S https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html

  Date: 11/11/2016
  Author: Logan Stuart 
  Modified: 15/02/2018
  Author: Logan Stuart
*/
#include <Servo.h>  //Need for Servo pulse output
//#include <FaBo9Axis_MPU9250.h> //refer to Installing9250Lib.png to install library

//#define NO_READ_MPU //Uncomment if MPU is not attached.
//#define NO_HC-SR04 //Uncomment if HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

// Instance of the MPU9250 class created
MPU9250_DMP imu;

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;


//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;
const unsigned int MAX_DISTANCE = 400; // Maximum distance (cm)

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
//const unsigned int MAX_DIST = 23200;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29

/*#ifndef NO_READ_MPU // THIS SAID 'NPU' BEFORE
FaBo9Axis fabo_9axis;
bool axis_OK;
#endif */

// Speed Variables
  int speed_val = 100;
  int speed_change;

// Sonar Sensor variables
  unsigned long sonar_t1;
  unsigned long sonar_t2;
  float sonar_cm; // calculated from converting the pulse width to a distance. Constants in datasheet

// Battery variables
  static byte Low_voltage_counter;
  static unsigned long battery_previous_millis;
  int Lipo_level_cal;
  int raw_lipo;

// Declarations and initialisations
Servo fan_servo;
int fan_pos = 0;
int fan_delay_counter = 0;
bool fan_cw = true;

//Serial Pointer
HardwareSerial *SerialCom;

void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

// Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

// MPU9250 set up
// Note all error-returning functions return a 0 on success so check for INV_SUCCESS for a more verbose error check
if (imu.begin() == INV_SUCCESS) {
  SerialCom->println("MPU is all good to go!");
} else {
  SerialCom->println("Yeah Nah, MPU not good...");
}

// Initialise DMP of MPU9250. DMP rate = 10Hz
imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL |DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_6X_LP_QUAT, 10);

// MPU9250 enabling all sensors
imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
// Full scale ranges for accelerometer and gyro
imu.setGyroFSR(2000); //200dps
imu.setAccelFSR(2); // +- 2g

// MPU9250 Filters and sample rates
imu.setLPF(5); // LPF corner freq (Hz)
imu.setSampleRate(10); // Hz
imu.setCompassSampleRate(10); // Hz

delay(1000); //settling time but no really needed

}

void loop(void) //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING: //Lipo Battery Volage OK 
      machine_state =  running();
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state =  stopped();
      break;
  };
}


STATE initialising() {
  //initialising
  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

STATE running() {
  static unsigned long previous_millis;

  read_serial_command();
  fast_flash_double_LED_builtin();

  sweep_fan(); 

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement 
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();
    //Analog_Range_A4(); // This prints analog stuff

/*
#ifndef NO_READ_MPU
    if (axis_OK)
      MPU9250_reading(&ax, &ay, &az, &gy, &gx, &gz);
#endif
*/

/*
#ifndef NO_HC-SR04
    HC_SR04_range();
#endif
*/

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
#endif
  }
  return RUNNING;

// Trying to read form the DMP
if (imu.fifoAvailable) // check for new data in FIFO
{
  if(imu.dmpUpdateFifo() == INV_SUCCESS) {
    printIMUData();
  }
}
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;

  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
    SerialCom->println("Please Re-charge Lipo");

    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK Counter:");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else counter_lipo_voltage_ok = 0;
  }
  return STOPPED;
}

void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

// Turn below into function so it only outputs info when asked. Could assign to interrupt?

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
//  static byte Low_voltage_counter;
//  static unsigned long previous_millis;
//
//  int Lipo_level_cal;
//  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    battery_previous_millis = millis();
//    SerialCom->print("Lipo level:");
//    SerialCom->print(Lipo_level_cal);
//    SerialCom->print("%");
//    SerialCom->print(" : Raw Lipo:");
//    SerialCom->println(raw_lipo);
    Low_voltage_counter = 0;
    return true;
  } else {
    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }

}
#endif

/* // Want to put this into a function so it isnt constantly spitting stuff out at us
#ifndef NO_HC-SR04
void HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }
  
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    SerialCom->println("HC-SR04: Out of range");
  } else {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
  }
}
#endif
*/

void Analog_Range_A4()
{
  SerialCom->print("Analog Range A4:");
  SerialCom->println(analogRead(A4));
}

/*
#ifndef NO_READ_MPU
void MPU9250_reading(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
  // do these need to be declared in setup? Have moved these
  //float ax, ay, az;
  //float gx, gy, gz;

  // Print Acceleration Measurements 
  fabo_9axis.readAccelXYZ(ax, ay, az);
  SerialCom->print("MPU Acceleration (g):");
  SerialCom->println(*ax);
  SerialCom->println(*ay);
  SerialCom->println(*az);

  // Print Gyroscope Measurements (deg/s)
  fabo_9axis.readGyroXYZ(gx, gy, gz);
  SerialCom->print("MPU Rotation (deg/s):");
  SerialCom->println(*gx);
  SerialCom->println(*gy);
  SerialCom->println(*gz);
}
#endif
*/

//Serial command pasing
void read_serial_command()
{
  if (SerialCom->available()) {
    char val = SerialCom->read();
//    SerialCom->print("Speed:");
//    SerialCom->print(speed_val);
//    SerialCom->print(" ms ");

    //Perform an action depending on the command
    switch (val) {
      case 'w'://Move Forward
      case 'W':
        forward ();
        SerialCom->println("Forward");
        break;
      case 's'://Move Backwards
      case 'S':
        reverse ();
        SerialCom->println("Backwards");
        break;
      case 'q'://Turn Left
      case 'Q':
        strafe_left();
        SerialCom->println("Strafe Left");
        break;
      case 'e'://Turn Right
      case 'E':
        strafe_right();
        SerialCom->println("Strafe Right");
        break;
      case 'a'://Turn Right
      case 'A':
        ccw();
        SerialCom->println("ccw");
        break;
      case 'd'://Turn Right
      case 'D':
        cw();
        SerialCom->println("cw");
        break;
      case '-'://Turn Right
      case '_':
        speed_change = -100;
        SerialCom->println("-100");
        break;
      case '=':
      case '+':
        speed_change = 100;
        SerialCom->println("+");
        break;

      //MPU9250 Test
      case 'k':
        MPU_measurements();
        break;

      // SONAR Test
      case 'l':
        sonar_reading();
        break;

      // Battery Voltage Information
      case 'b':
        battery_voltage();
        break;

       // Speed info
       case 'o':
        speed_read();
        break;
        
      default:
        stop();
        SerialCom->println("stop");
        break;
    }

  }

}

//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control 

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On

  fan_servo.attach(9);
}
void stop() //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void reverse ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void sonar_reading()
{
  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  sonar_t1 = micros();
  while (digitalRead(ECHO_PIN) == 0 ) {
    sonar_t2 = micros();
    sonar_cm = (sonar_t2 - sonar_t1)/58.0;
    if (sonar_cm > MAX_DISTANCE) {
      SerialCom->println("Sonar Sensor: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width). Counter overflow after ~70min
  sonar_t1 = micros();
  while (digitalRead(ECHO_PIN) == 1)
  {
    sonar_t2 = micros();
    sonar_cm = (sonar_t2 - sonar_t1)/58.0;
    if (sonar_cm > (MAX_DISTANCE) ) {
      SerialCom->println("Sonar Sensor: Out of range");
      return;
    }
  }
  
  sonar_t2 = micros();
  sonar_cm = (sonar_t2 - sonar_t1)/58.0;

  // Print measurements
  if ( sonar_cm > MAX_DISTANCE ) {
    SerialCom->println("Sonar Sensor: Out of range");
  } else {
    SerialCom->print("Sonar Sensor:");
    SerialCom->print(sonar_cm);
    SerialCom->println("cm");
  }

}

void reset_gyro(float *gx, float *gy, float *gz)
{
  // Reset coordinates?
  *gx = 0;
  *gy = 0;
  *gz = 0; 

  // Reconfigure the gyro?
   //fabo_9axis.configMPU9250();
}

void MPU_measurements()
{

  imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
  
  // Print Acceleration Measurements 
  SerialCom->print("MPU Acceleration (g):");
  SerialCom->println(imu.calcAccel(imu.ax));
  SerialCom->println(imu.calcAccel(imu.ay));
  SerialCom->println(imu.calcAccel(imu.az));

  // Print Gyroscope Measurements (deg/s)
  SerialCom->print("MPU Rotation (deg/s):");
  SerialCom->println(imu.calcGyro(imu.gx));
  SerialCom->println(imu.calcGyro(imu.gy));
  SerialCom->println(imu.calcGyro(imu.gz));

  // Print Magnetometer Measurements (uT)
  SerialCom->print("MPU Magnetic Field (uT):");
  SerialCom->println(imu.calcMag(imu.mx));
  SerialCom->println(imu.calcMag(imu.my));
  SerialCom->println(imu.calcMag(imu.mz));
  
}

void battery_voltage()
{
  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    battery_previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    SerialCom->print(" : Raw Lipo:");
    SerialCom->println(raw_lipo);
    Low_voltage_counter = 0;
    return true;
  } else {
    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}

void speed_read() 
{
  SerialCom->print("Speed:");
  SerialCom->print(speed_val);
  SerialCom->print(" ms ");
}

void sweep_fan()
{
  //Servo position only changed every 1000 counts
  if (fan_delay_counter >= 1000) {
    fan_delay_counter = 0;

    //When servo is turning clockwise
    if (fan_cw) {
      if (fan_pos < 180) {
        fan_pos++;
      }
      //Reached max of clockwise turn, changes to anticlockwise
      else {
        fan_pos--;
        fan_cw = false;
      }
    }

    //When servo is turning anticlock
    else {
      if (fan_pos > 0) {
        fan_pos--;
      }
      //Reached max of anticlockwise turn, changes to clockwise
      else {
        fan_pos++;
        fan_cw = true;
      }
    }
    
    fan_servo.write(fan_pos);
  }
  else {
    fan_delay_counter++;
  }
}

void printIMUData(void)
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  float gyroX = imu.calcGyro(imu.gx);
  float gyroY = imu.calcGyro(imu.gy);
  float gyroZ = imu.calcGyro(imu.gz);
  
  SerialPort.println("Gyro: " + String(gyroX) + ", " +
              String(gyroY) + ", " + String(gyroZ) + " dps");
  SerialPort.println("Time: " + String(imu.time) + " ms");
  SerialPort.println();
}

void calculateHeading() 
{
  // Need to determine offset (output at angular velocity = 0) Currently outputting something close to half of supply.
  // Involts. Could use multimeter.
  // sensitivity (scale factor) trial and error?
}


