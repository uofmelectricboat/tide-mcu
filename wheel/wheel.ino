/*
All the quotes are from here: https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
*/

class Pot {
  private:
    int pin;
    int in_min;
    int in_max;
    int out_min;
    int out_max;
    const String pot_name;

    // From https://esp32io.com/tutorials/esp32-potentiometer
    float floatMap(float x) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

  public:
    Pot(int pin, int in_min, int in_max, int out_min, int out_max, const String name) : 
      pin(pin), in_min(in_min), in_max(in_max), out_min(out_max), pot_name(name) {}

    float read() {
      floatMap(readRaw());
    }

    uint16_t readRaw() {
      return analogRead(pin);
    }

    const String & name() {
      return pot_name;
    }

    // MODIFIES : analog min value
    void set_min(int analogVal) {
      in_min = analogVal;
    }

    // MODIFIES : analog max value
    void set_max(int analogVal) {
      in_max = analogVal;
    }
};

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


// uint16_t i = 0; // FIXME not sure why this is here, should delete it

// FIXME instead use ESP32Logger or Arduino_DebugUtilis, or EasyLogger = can be compiled out

// Debug mode
#define debugging true


  // Steering wheel encoder
  Pot encoder(32,0,4095,0,360, "Encoder"); // pin, analog min & max, out min & max Scale => [0,360]

  // Throttle Potentiometer
  Pot throttle(34,0,4095,0,100, "Throttle"); // pin, analog min & max, out min & max  Scale => [0,100]

void setup() {
  // debugMode(debugging);


  Serial.begin(9600);

  while(debugging && !Serial){} // Wait for serial if in debug mode
  // debugPrintln("CAN mcu");

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
    // debugPrintln("Starting CAN fail");
  }
}

void loop() {

  encoderSend();

  throttleSend();

  buttonsSend();

  LEDReceive();
}


// Encoder and Throttle to CAN
void potToCAN(int id, int val, const String name) {
  size_t size = sizeof(val);
  CAN.beginPacket(id);
  uint8_t data[size];           // Create uint8_t array
  memcpy(data, &val, size);  // Store bytes of val to array
  CAN.write(data, size);     // Write the buffer to CAN
  CAN.endPacket();
  delay

  // debugPrintln(pot + dash + val);
}

void encoderSend() {
  potToCAN(encoderId, encoder.read(), encoder.name());
}

void throttleSend() {
  potToCAN(throttleId, throttle.read(), throttle.name());
}


void buttonsSend() {
  CAN.beginPacket(buttonsId);
  for (int i = 0; i < NUM_BUTTONS; i++) {
    CAN.write(bool(digitalRead(buttons[i])));

    // debugPrintln(button_names[i] + dash + digitalRead(buttons[i]));
  }
  
  CAN.endPacket();
  // debugPrintln("Sent buttons");
}


void LEDReceive() {
  // try to parse packet
  int packetSize = CAN.parsePacket();

  if (packetSize && CAN.packetId() == LEDsId) {
    // received a packet
    // debugPrint("Received ");

    if (CAN.packetExtended()) {
      // debugPrint("extended ");
    }

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      // debugPrint("RTR ");
    }

    // debugPrint("packet with id 0x");
    // debugPrint(CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
      // debugPrint(" and requested length ");
      // debugPrintln(CAN.packetDlc());
    } else {
      // debugPrint(" and length ");
      // debugPrintln(packetSize);

      char data[packetSize];

      // only print packet data for non-RTR packets
      int i = 0;
      while (CAN.available()) {
        data[i] = CAN.read();
        i++;
      }

      // debugPrint("LEDS: ");

      // Iterate through the LEDs
      for (int i = 0; i < NUM_LEDS; i++) {
        // debugPrint((i+1) + dash);

        if (data[i]) { // 1 evaluates to true, 0 evaluates to false
  
          // debugPrintln(LED_ON);

          digitalWrite(LEDs[i], HIGH);
        }

        else {
          // debugPrint(LED_OFF);

          digitalWrite(LEDs[i], LOW);
        }
      }

      // debugPrintln();
    }

    // debugPrintln();
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
4. User moves to min position
3. keep it there for 3 seconds
3.  

*/
bool pot_cal(Pot &pot) {
  Serial.print(pot.name());
  Serial.println(" Calibration:");


  // Number of data points to collect
  const size_t numData = 300;

  // Delay between data points (in FIXME-seconds)
  const interval = 100;

  


  Serial.println("")
}