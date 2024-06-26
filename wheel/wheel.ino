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

/*
CAN default pins
CAN | ESP32
3V3	| 3V3
GND	| GND
CTX	| GPIO_5
CRX	| GPIO_4
*/
#include <CAN.h>

int encoderVal = 0;
int throttleVal = 0;
uint16_t i = 0;

void setup() {
  Serial.begin(9600);

  while(!Serial){}

  Serial.println("CAN mcu");

  // Steering momentary buttons
  pinMode(CommsPin, INPUT_PULLUP);
  pinMode(GainUpPin, INPUT_PULLUP);
  pinMode(TrimUpPin, INPUT_PULLUP);
  pinMode(TrimDownPin, INPUT_PULLUP);
  pinMode(GainDownPin, INPUT_PULLUP);

  // Encoder & Potentiometer should NOT be set to input
  
  // Switchboard LEDs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);

  if(!CAN.begin(500E3)){
    Serial.println("Starting CAN fail");
    while(1);
  }


}

void loop() {

  encoderVal = readEncoder();

  // Encoder testing
  Serial.print("Encoder: ");
  Serial.println(encoderVal);

  encodertoCAN(encoderVal);

  throttleVal = readThrottle();

  // Throttle testing
  Serial.print("Throttle: ");
  Serial.println(throttleVal);
  delay(500);

  throttletoCAN(throttleVal);

  buttonstoCAN();

  LEDReceive();
}

// From https://esp32io.com/tutorials/esp32-potentiometer
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float readEncoder(){
    return floatMap(analogRead(ThrottlePin), 0, 4095, 0, 360); //Scale => [0,360]

}

float readThrottle(){
  return floatMap(analogRead(ThrottlePin), 0, 4095, 0, 100); //Scale => [0,100]
}

void encodertoCAN(int val){
  CAN.beginPacket(1999);
  char data[sizeof(val)];               //Create char array
  memcpy(data, &val, sizeof(val));       //Store bytes of val to array
  for(int j = 0; j < sizeof(val); j++){  //Write bytes one by one to CAN
    CAN.write(data[j]);
  }
  CAN.endPacket();
}

void throttletoCAN(int val){
  CAN.beginPacket(1998);
  char data[sizeof(val)];                //Create char array
  memcpy(data, &val, sizeof(val));       //Store bytes of val to array
  for(int j = 0; j < sizeof(val); j++){  //Write bytes one by one to CAN
    CAN.write(data[j]);
  }
  CAN.endPacket();
}

/*----------------------------------
IDs
2000
-----------------------------------*/
void buttonstoCAN(){
  CAN.beginPacket(2000);
  CAN.write(bool(digitalRead(CommsPin)));
  CAN.write(bool(digitalRead(TrimUpPin)));
  CAN.write(bool(digitalRead(TrimDownPin)));
  CAN.write(bool(digitalRead(GainUpPin)));
  CAN.write(bool(digitalRead(GainDownPin)));

  //  Button testing
  Serial.print("Comms: ");
  Serial.println(bool(digitalRead(CommsPin)));
  Serial.print("Trim Up: ");
  Serial.println(bool(digitalRead(TrimUpPin)));
  Serial.print("Trim Down: ");
  Serial.println(bool(digitalRead(TrimDownPin)));
  Serial.print("Gain Up: ");
  Serial.println(bool(digitalRead(GainUpPin)));
  Serial.print("Gain Down: ");
  Serial.println(bool(digitalRead(GainDownPin)));

  CAN.endPacket();
  Serial.println("sent");
}


void LEDReceive() {

    // try to parse packet
  int packetSize = CAN.parsePacket();

  if (packetSize && CAN.packetId() == 2001) {
    // received a packet
    Serial.print("Received ");

    if (CAN.packetExtended()) {
      Serial.print("extended ");
    }

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      Serial.print("RTR ");
    }

    Serial.print("packet with id 0x");
    Serial.print(CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
      Serial.print(" and requested length ");
      Serial.println(CAN.packetDlc());
    } else {
      Serial.print(" and length ");
      Serial.println(packetSize);

      char data[packetSize];

      // only print packet data for non-RTR packets
      int i = 0;
      while (CAN.available()) {
        data[i] = CAN.read();
        i++;
      }

      if(data[0] == 1){
        digitalWrite(LED1, HIGH);
        Serial.print("recieve yay");
      }else{
        digitalWrite(LED1, LOW);
      }

      if(data[1] == 1){
        digitalWrite(LED2, HIGH);
        Serial.print("LED2");
      }else{
        digitalWrite(LED2, LOW);
      }

      if(data[2] == 1){
        digitalWrite(LED3, HIGH);
      }else{
        digitalWrite(LED3, LOW);
      }

      if(data[3] == 1){
        digitalWrite(LED4, HIGH);
      }else{
        digitalWrite(LED4, LOW);
      }

      if(data[4] == 1){
        digitalWrite(LED5, HIGH);
      }else{
        digitalWrite(LED5, LOW);
      }

      if(data[5] == 1){
        digitalWrite(LED6, HIGH);
      }else{
        digitalWrite(LED6, LOW);
      }
      Serial.println();
    }

    Serial.println();
  }
}
