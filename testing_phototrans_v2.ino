// This code should turn on the fan if the phototransistors dectect a fire.
// Here the phototransistors are all just pointint forward and the fan is not on a servo

// Set of pins, include this with other pin setups
const int LEFTMOST_PHOTO = A4;
const int MIDLEFT_PHOTO = A5;
const int MIDRIGHT_PHOTO = A6;
const int RIGHTMOST_PHOTO = A7;

const int FAN_PIN = 8;

 // Modify the function "running()" to look somewhat like this, just copy and paste the lines needed
 STATE running() {
  static unsigned long previous_millis; // For every 500ms
  static unsigned long sprevious_millis; // For every 10ms
  static float avg1; // SFH 309 FA, "Little Boy", The Little Phototransistor
  static float avg2; // SFH 300 FA, "Big Boy", The Big Phototransistor
  static float avg3; // SFH 300 FA, "Big Boy", The Big Phototransistor
  static float avg4; // SFH 309 FA, "Little Boy", The Little Phototransistor
  static float total1; // SFH 309 FA
  static float total2; // SFH 300 FA
  static float total3; // SFH 300 FA
  static float total4; // SFH 309 FA

  // Reads the four phototransistors every 10ms
  if (millis() - sprevious_millis > 10) {
    sprevious_millis = millis();

    // Adds the 10ms readings of the phototransistors for each one
    total1 += read_phototransistor(LEFTMOST_PHOTO); // SFH 309 FA
    total2 += read_phototransistor(MIDLEFT_PHOTO); // SFH 300 FA
    total3 += read_phototransistor(MIDRIGHT_PHOTO); // SFH 300 FA
    total4 += read_phototransistor(RIGHTMOST_PHOTO); // SFH 309 FA
  }

  //Arduino style 500ms timed execution statement
  if (millis() - previous_millis > 500) { 
    previous_millis = millis();

    // Calculates the average reading from each phototransistor taken over a 500ms period and resets the totals
    avg1 = total1/50.0;
    total1 = 0;
    avg2 = total2/50.0;
    total2 = 0;
    avg3 = total3/50.0;
    total3 = 0;
    avg4 = total4/50.0;
    total4 = 0;

    if (is_there_a_fire(avg1, avg2, avg3, avg4))
      digitalWrite(FAN_PIN, HIGH);
    else
      digitalWrite (FAN_PIN, LOW);
    
    // Prints the phototransistor readings
    SerialCom->println("RUNNING---------");
    SerialCom->print("photo 1: ");
    SerialCom->println(avg1);
    SerialCom->print("photo 2: ");
    SerialCom->println(avg2);
    SerialCom->print("photo 3: ");
    SerialCom->println(avg3);
    SerialCom->print("photo 4: ");
    SerialCom->println(avg4);  
  }

  return RUNNING;
}

// Function to read from phototransistor attached to pin A_ (e.g. pin A4 or A5)
float read_phototransistor(int photo_pin_number)
{
  float temp_volt;
  
  temp_volt = analogRead(photo_pin_number)*(5.0/1023.0); // Takes reading from pin and converts it to the voltage value
  
  return temp_volt;
}

// Function to turn on fan if there is a fire
bool is_there_a_fire(float avg1, float avg2, float avg3, float avg4)
{
  bool call_the_firedepartment = false;
  
  if ((avg1 >= 1.8) || (avg2 >= 1.1) || (avg3 >= 1.1) || (avg4 >= 1.8) 
    return call_the_firedepartment = true;
  else
    return call_the_fiedepartment;
}

