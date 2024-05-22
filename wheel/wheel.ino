/*
All the quotes are from here: https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
*/

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

#include <Mapped_Encoder.h> // Ecoder and throttle

// Debug mode
// Options in lowest to highest priority:
// DBG_NONE, DBG_ERROR, DBG_WARNING, DBG_INFO (default), DBG_DEBUG, DBG_VERBOSE
#define debuggingLevel DBG_DEBUG

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
#define CommsPin 12    // Top Right Button (B-TRB) // "boot fails if pulled high, strapping pin" = works fine
#define TrimUpPin 14    // Right Paddle (C-RPD) // "outputs PWM signal at boot"
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

#define NUM_BUTTONS 5
#define NUM_LEDS 6

// Lists that contain the Steering Wheel buttons and the LED pins
int buttons[NUM_BUTTONS] = {CommsPin, TrimUpPin, TrimDownPin, GainUpPin, GainDownPin};
const String button_names[NUM_BUTTONS] = {"Comms", "Trim Up", "Trim Down", "Gain Up", "Gain Down"};

int LEDs[NUM_LEDS] = {LED1, LED2, LED3, LED4, LED5, LED6};

// Steering wheel encoder
Mapped_Encoder encoder(32,0,4095,0,360, BITMAX12, "Encoder"); // Scale => [0,360]

// Throttle Potentiometer
Mapped_Encoder throttle(34,0,4095,0,100, BITMAX12, "Throttle"); // Scale => [0,100]

void setup() {

  Serial.begin(9600); // Can have Debug send to another stream if we want

  Debug.setDebugLevel(debuggingLevel);
  Debug.timestampOn();
  Debug.newlineOn(); // Send a new line after every message

  while(debuggingLevel >= DBG_DEBUG && !Serial){} // Wait for serial only if in debug mode or higher
  DEBUG_INFO("CAN mcu");

  // Steering momentary buttons
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttons[i], INPUT_PULLUP);
  }

  // Encoder & Potentiometer SHOULD NOT be set to input
  
  // Switchboard LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(LEDs[i], OUTPUT);
  }

  // Wait for CAN to begin
  while(!CAN.begin(bitrateCAN)){
    DEBUG_WARNING("Starting CAN failure");
  }
}

void loop() {

  potToCAN(encoderId, encoder.read(), encoder.name());

  potToCAN(throttleId, throttle.read(), throttle.name());

  buttonsSend();

  LEDReceive();
}


// Encoder and Throttle to CAN
void potToCAN(int id, int val, const String name) {
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
  CAN.beginPacket(buttonsId);
  for (int i = 0; i < NUM_BUTTONS; i++) {
    CAN.write(digitalRead(buttons[i]));

    DEBUG_INFO("Button %s - %s", button_names[i], digitalRead(buttons[i]) ? "On" : "Off");
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

// FIXME change to display on screen
/*
Steps:
1. Start calibration
2. Indicate to user to start
3. User moves to min position
4. keep it there for lockTime number of milliseconds
5. 

*/

bool pot_cal(Mapped_Encoder &pot, int numTrials) {
  DEBUG_INFO("%s Calibration:", pot.name());

  // Max number of data points to hold for each trial
  const size_t numData = 500;

  // Number of seconds to wait for locking in the value (in milliseconds)
  const int lockTime = 10000; // 1000 milliseconds = 1 second

  // Create two 2D arrays, one for each extrema
  uint16_t mins[numTrials][numData];
  uint16_t maxes[numTrials][numData];

  return false; // XXX

}
