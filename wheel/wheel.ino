// Main wheel code

#include <Arduino_DebugUtils.h> // Logging

#include <CAN.h> // CAN
/*
CAN default pins
CAN | ESP32
3V3	| 3V3
GND	| GND
CTX	| GPIO_5
CRX	| GPIO_4
*/

#include "Mapped_Encoder.h" // Encoder and throttle

// Debug mode
// Options in lowest to highest priority:
// DBG_NONE, DBG_ERROR, DBG_WARNING, DBG_INFO (default), DBG_DEBUG, DBG_VERBOSE
#define debuggingLevel DBG_INFO

// Set whether to disable CAN (useful for debugging)
#define DISABLECAN true

// Calibration settings
#define calTol 10
#define calNumTrials 2

// Can bitrate
#define bitrateCAN 500E3

// CAN Ids
#define encoderId  1999
#define throttleId 1998
#define buttonsId  2000
#define LEDsId     2001

/*
Steering Wheel Buttons
Momentary buttons
All should be set to pullup, using the ESP32's internal pullup resistors

E-stop (A-BRB) is intentionally not included
*/
#define CommsPin 12    // Top Right Button (B-TRB)
#define TrimUpPin 14    // Right Paddle (C-RPD) // may output PWM signal at boot
#define TrimDownPin 27  // Left Paddle (E-LPD)
#define GainUpPin 26    // Top Left Button (F-TLB)
#define GainDownPin 25  // Bottom Left Button (G-BLB)

// Switchboard Green LEDs
// Connected to the LED's positive leads, i.e. the red wire
#define LED1 23
#define LED2 22
#define LED3 21
#define LED4 19
#define LED5 18
#define LED6 17

// Lists that contain the steering wheel buttons and LED pins
#define NUM_BUTTONS 5
#define NUM_LEDS 6

int buttons[NUM_BUTTONS] = {CommsPin, TrimUpPin, TrimDownPin, GainUpPin, GainDownPin};
const String button_names[NUM_BUTTONS] = {"Comms", "Trim Up", "Trim Down", "Gain Up", "Gain Down"};

int LEDs[NUM_LEDS] = {LED1, LED2, LED3, LED4, LED5, LED6};

// Steering wheel encoder
Mapped_Encoder encoder(32, BITMAX12, 0, 360, "Encoder"); // Scale => [0,360]

// Throttle potentiometer
Mapped_Encoder throttle(34, BITMAX12, 0, 100, "Throttle"); // Scale => [0,100]


// Calibration Button Combos
int encoderCalCombo[NUM_BUTTONS] = {1, 0, 0, 0, 0};
int throttleCalCombo[NUM_BUTTONS] = {1, 0, 0, 1, 1};


void setup() {

  Serial.begin(9600); // Can have Debug send to another stream if we want

  Debug.setDebugLevel(debuggingLevel);
  Debug.timestampOn();
  Debug.newlineOn(); // Send a new line after every message

  while(debuggingLevel >= DBG_DEBUG && !Serial) {
    yield(); // Wait for serial only if in debug mode or higher
  }
  DEBUG_INFO("CAN mcu");

  // Steering momentary buttons
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttons[i], INPUT_PULLUP);
  }
  DEBUG_VERBOSE("Setup Buttons");

  // Encoder & Potentiometer SHOULD NOT be set to input
  
  // Switchboard LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(LEDs[i], OUTPUT);
  }
  DEBUG_VERBOSE("Setup LEDs");

  // Wait for CAN to begin
  while(!CAN.begin(bitrateCAN)){
    DEBUG_WARNING("Starting CAN failure");
  }
  DEBUG_VERBOSE("Setup CAN");
}

void loop() {

  encoderToCAN(encoderId, encoder.read(), encoder.name());

  encoderToCAN(throttleId, throttle.read(), throttle.name());

  buttonsSend();

  LEDReceive();

  delay(1000);  // For testing to make it easier to read values in serial

}

// Encoder and Throttle to CAN
void encoderToCAN(int id, int val, const String name) {
  if (DISABLECAN) {return;}
  size_t size = sizeof(val);
  CAN.beginPacket(id);
  uint8_t data[size];                   // Create uint8_t array
  memcpy(data, &val, size);             // Store bytes of val to array
  size_t sent = CAN.write(data, size);  // Write the buffer to CAN
  int end = CAN.endPacket();

  DEBUG_INFO("%s - %d", name, val);
  if (sent < size) {
    DEBUG_WARNING("Size of message sent < size of data");
  } else {
    DEBUG_INFO("All data sent");
  }
  DEBUG_VERBOSE("endPacket output: %d", end);

}

void buttonsSend() {
  // Check if a calibration button combo is pressed
  checkButtonCombo();

  if (DISABLECAN) {return;}
  CAN.beginPacket(buttonsId);
  for (int i = 0; i < NUM_BUTTONS; i++) {
    CAN.write(!digitalRead(buttons[i]));

    DEBUG_INFO("Button %s - %s", button_names[i], !digitalRead(buttons[i]) ? "On" : "Off");
  }
  
  int end = CAN.endPacket();

  DEBUG_INFO("Sent buttons");

  DEBUG_VERBOSE("endPacket output: %d", end);
}

void LEDReceive() {
  DEBUG_INFO("Receiving LED messages");

  // try to parse packet
  int packetSize = CAN.parsePacket();

  if (packetSize && CAN.packetId() == LEDsId) {

    DEBUG_DEBUG("Received...");

    if (CAN.packetExtended()) {
      DEBUG_DEBUG("...extended...");
    }

    DEBUG_DEBUG("...packet with id 0x%a...", CAN.packetId());

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      DEBUG_WARNING("(No Data Received for LEDs)");
      DEBUG_DEBUG("...RTR and requested length %d", CAN.packetDlc());
    
    } else {
      DEBUG_DEBUG("...and length %d", packetSize);
      uint8_t data[packetSize];

      // only print packet data for non-RTR packets
      int size = 0;
      while (CAN.available()) {
        data[size] = CAN.read();
        size++;
      
      }

      int count = NUM_LEDS; // Default value

      if (packetSize > NUM_LEDS) {
        DEBUG_ERROR("More data received than number of LEDs, continuing with first %d", count);
      }

      if (packetSize < NUM_LEDS) {
        count = packetSize;
        DEBUG_ERROR("Less data received than number of LEDs, continuing with %d given", count);
      }

      DEBUG_INFO("LEDS:");

      // Iterate through the LEDs
      for (int i = 0; i < count; i++) {
        DEBUG_INFO("LED %d - %s", i+1, data[i] ? "On" : "Off");
        
        if (data[i] > 1) {
          DEBUG_ERROR("Non-binary value: %d, defaulting to on", data[i]);
        }

        digitalWrite(LEDs[i], data[i] ? HIGH : LOW);
      } 
    }
  }
  else {
    DEBUG_WARNING("No LED CAN messages received.");
  }
}

/*

-----------
Calibration
-----------

(Cal is short for Calibration)

*/

// EFFECTS  : starts calibration if one of the button combos is pressed
int checkButtonCombo() {
  DEBUG_DEBUG("Checking button combos");
  bool encoderMatch, throttleMatch;
  encoderMatch = throttleMatch = true;
  int val;

  for (int i = 0; i < NUM_BUTTONS; i++) {
    val = !digitalRead(buttons[i]);
    DEBUG_DEBUG("Button %s - %s", button_names[i], val ? "On" : "Off");

    if (encoderMatch && val != encoderCalCombo[i]) {
      encoderMatch = false;
      DEBUG_DEBUG("Encoder combo button non match");
    }

    if (throttleMatch && val != throttleCalCombo[i]) {
      throttleMatch = false;
      DEBUG_DEBUG("Throttle combo button non match");
    }
  }

  DEBUG_DEBUG("Done checking button combos");
  DEBUG_DEBUG("EncoderMatch: %d", encoderMatch);
  DEBUG_DEBUG("ThrottleMatch: %d", throttleMatch);

  if (encoderMatch) {
    return encoderCal(encoder, calNumTrials, calTol);
  }

  else if (throttleMatch) {
    return encoderCal(throttle, calNumTrials, calTol);
  }

  // Else, return 0
  return 0;
}

// FIXME change to display on screen
/*
Steps:
1. Start calibration
2. Indicate to user to move to min
3. User moves to min position
4. User keeps it there for lockTime number of milliseconds
5. Indicate to user to move to max
6. User moves to max postion
7. User keeps it there for lockTime number of milliseconds
8. Repeat steps 2-7 for the number of trials
9. If the trial means are within tolerance of each other, continue
10. Update the encoder's value both in the object and in EEPROM
*/
int encoderCal(Mapped_Encoder &encoder, int numTrials, float tol) {
  Serial.printf("%s Calibration:\n", encoder.name());

  //  Max number of data points to hold for each trial
  const int numData = 100;

  // Number of seconds to wait for locking in the value (in milliseconds)
  const int lockTime = 10000; // 1000 milliseconds = 1 second

  // Create two 2D arrays, one for each extrema
  float mins[numTrials];
  float maxes[numTrials];

  for (size_t trial = 0; trial < numTrials; trial++) {
    Serial.printf("Trial #%d \n", trial+1);

    Serial.println("Move the wheel clockwise to its right maximum / take your foot all the way off the throttle");

    mins[trial] = encoder_find_extrema(encoder, numData, lockTime, tol);

    Serial.println("Now move the wheel counterclockwise to its left maximum / press the throttle to its max");

    maxes[trial] = encoder_find_extrema(encoder, numData, lockTime, tol);
  }

  int issue = 0;
  for (size_t i = 1; i < numTrials; i++) {
    if (!check_tol(mins[i-1], mins[i], tol, encoder.get_analogMax())) {
      issue += 1;
    }
    else if (!check_tol(maxes[i-1], maxes[i], tol, encoder.get_analogMax())) { // Doesn't check max if min is true
      issue += 2;

    }

    if (issue > 0) {
      Serial.printf("Values for %s trials are not within tolerance of each other, \
      please try calibration again\n", issue == 1 ? "min" : "max");

      // Don't change the encoder's values
      return issue;
    }
    

  }

  // If all the values are generally within tolerance of each other,
  // find the mean min and max of all trials since all trials have equal sizes
  encoder.set_analog_vals(round(avg_analog(mins, numTrials, tol, encoder.get_analogMax())), 
          round(avg_analog(maxes, numTrials, tol, encoder.get_analogMax())));

  Serial.printf("%s Calibration completed and values stored to EEPROM\n", encoder.name());

  return issue;
}

// EFFECTS  : collects data once the wheel is stationary for lockTime milliseconds
float encoder_find_extrema(Mapped_Encoder &encoder, const int numData, const int lockTime, float tol) {
  // Make an array to hold the values
  float data[numData]; // If there are memory space issues, we could make this uint16_t
  data[0] = 0;

  size_t index = 1;
  float val;

  while (index < numData) {
    val = encoder.readRaw();

    if (!check_tol(data[index-1], val, tol, encoder.get_analogMax())) { // Alternative solution: compare to moving average
      index = 0;
      Serial.println("Restart, no longer stationary");
    }

    else {
      Serial.print(".");
    }

    data[index] = val;
    index++;

    delay(lockTime / numData);
  }

  float average = avg_analog(data, numData, tol, encoder.get_analogMax());

  Serial.printf("Found extrema: %f\n", average);

  return average;

}

// REQUIRES : arr is properly initialized and has length >= 1
float avg_analog(float * arr, size_t numData, float tol, int analogMax) {
  int numAboveMax, numBelowMax = 0; // In special case where values wrap around

  int tolMax = analogMax - tol;

  float average = 0;

  for (size_t i = 0; i < numData; i++) {
    average += arr[i];

    if (arr[i] >= tolMax) { // 0 <= val <= tol
      numBelowMax++;
    }
    else if (arr[i] <= tol) { // max - tol <= val <= max
      numAboveMax++;
    }
  }

  // Special cases near analogMax

  // If most values lean to above the max
  if (numBelowMax <= numAboveMax) {
    average -= numBelowMax * analogMax;
  }
  // If most values lean to below the max
  else if (numAboveMax < numBelowMax) {
    average += numAboveMax * analogMax;
  }

  // Divide by the number and return
  return average / numData;
}

// EFFECTS  : returns true if two values are within tolerance
// considering both the base case and when values pass over analogMax
bool check_tol(float lhs, float rhs, float tol, int analogMax) {
  // In the case of values near 0 / analogMax
  int tolMax = analogMax - tol;
  if ((lhs >= tolMax && rhs <= tol) || (rhs >= tolMax && lhs <= tol)) {
    return true;
  }

  // Base case
  else {
    DEBUG_VERBOSE("Check tolerance: %f, %f : %d", lhs, rhs, abs(lhs - rhs) <= tol);
    return abs(lhs - rhs) <= tol;
  }
}
