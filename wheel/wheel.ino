/*
All the quotes are from here: https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
*/

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


// Steering wheel encoder
#define EncoderPin 32 // Two away from the throttle pin

// Throttle Potentiometer
#define ThrottlePin 34 // Should not be set to input, "input only"

// Switchboard Green LEDs
// Connected to the LED's positive leads, i.e. the red wire
#define LED1 23
#define LED2 22
#define LED3 21
#define LED4 19
#define LED5 18
#define LED6 17

// Lists that contain the Steering Wheel buttons and the LED pins
#define NUM_BUTTONS 5
#define NUM_LEDS 6

int buttons[NUM_BUTTONS] = {CommsPin, TrimUpPin, TrimDownPin, GainUpPin, GainDownPin};
const String button_names[NUM_BUTTONS] = {"Comms", "Trim Up", "Trim Down", "Gain Up", "Gain Down"};

int LEDs[NUM_LEDS] = {LED1, LED2, LED3, LED4, LED5, LED6};

// For serial
const String dash = " - ";
const String LED_ON = "ON; ";
const String LED_OFF = "OFF; ";


#include <CAN.h>

/*
CAN default pins
CAN | ESP32
3V3	| 3V3
GND	| GND
CTX	| GPIO_5
CRX	| GPIO_4
*/

// Can bitrate
#define bitrateCAN 500E3

// CAN Ids
#define encoderId  1999
#define throttleId 1998
#define buttonsId  2000
#define LEDsId     2001


int encoderVal = 0;
int throttleVal = 0;
// uint16_t i = 0; // FIXME not sure why this is here, should delete it

// FIXME instead use ESP32Logger or Arduino_DebugUtilis, or EasyLogger = can be compiled out
#include "debug.h"

// Debug mode
#define debugging true

void setup() {
  debug.debugMode(debugging);

  Serial.begin(9600);

  while(debug && !Serial){} // Wait for serial if in debug mode
  debugPrintln("CAN mcu");

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
    debugPrintln("Starting CAN fail");
  }
}

void loop() {

  encoderSend();

  throttleSend();

  buttonsSend();

  LEDReceive();
}


// Encoder and Throttle to CAN
void potToCAN(const int id, const int val, const String pot) {
  size_t size = sizeof(val);
  CAN.beginPacket(id);
  const uint8_t data[size];           // Create char array
  memcpy(data, &val, size);  // Store bytes of val to array
  CAN.write(data, size);     // Write the buffer to CAN
  CAN.endPacket();

  debugPrintln(pot + dash + val);
}

// From https://esp32io.com/tutorials/esp32-potentiometer
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void encoderSend() {
  int val = floatMap(analogRead(EncoderPin), 0, 4095, 0, 360); //Scale => [0,360]
  potToCAN(encoderId, val, "Encoder");
}

void throttleSend() {
  int val = floatMap(analogRead(ThrottlePin), 0, 4095, 0, 100); //Scale => [0,100]
  potToCAN(throttleId, val, "Throttle");
}


void buttonsSend() {
  CAN.beginPacket(buttonsId);
  for (int i = 0; i < NUM_BUTTONS; i++) {
    CAN.write(bool(digitalRead(buttons[i])));

    debugPrintln(button_names[i] + dash + digitalRead(buttons[i]));
  }
  
  CAN.endPacket();
  debugPrintln("Sent buttons");
}


void LEDReceive() {
  // try to parse packet
  int packetSize = CAN.parsePacket();

  if (packetSize && CAN.packetId() == LEDsId) {
    // received a packet
    debugPrint("Received ");

    if (CAN.packetExtended()) {
      debugPrint("extended ");
    }

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      debugPrint("RTR ");
    }

    debugPrint("packet with id 0x");
    debugPrint(CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
      debugPrint(" and requested length ");
      debugPrintln(CAN.packetDlc());
    } else {
      debugPrint(" and length ");
      debugPrintln(packetSize);

      char data[packetSize];

      // only print packet data for non-RTR packets
      int i = 0;
      while (CAN.available()) {
        data[i] = CAN.read();
        i++;
      }

      debugPrint("LEDS: ");

      // Iterate through the LEDs
      for (int i = 0; i < NUM_LEDS; i++) {
        debugPrint((i+1) + dash);

        if (data[i]) { // 1 evaluates to true, 0 evaluates to false
  
          debugPrintln(LED_ON);

          digitalWrite(LEDs[i], HIGH);
        }

        else {
          debugPrint(LED_OFF);

          digitalWrite(LEDs[i], LOW);
        }
      }

      debugPrintln();
    }

    debugPrintln();
  }
}
