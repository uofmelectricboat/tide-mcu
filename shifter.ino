/* 
Direction Shifter
Toggle switches
All should be pullup

When neither shifter forward or reverse are active,
the shifter is in the neutral position
*/
#define ShifterForwardPin 15
#define ShifterReversePin 2
#define ShifterSwitchPin 13


/*
CAN default pins
CAN | ESP32
3V3	| 3V3
GND	| GND
CTX	| GPIO_5
CRX	| GPIO_4
*/
#include <CAN.h>

void setup() {
  Serial.begin(9600);

  while(!Serial){}

  Serial.println("shifter mcu");

  // Shifter toggle switches
  pinMode(ShifterForwardPin, INPUT_PULLUP);
  pinMode(ShifterReversePin, INPUT_PULLUP);
  pinMode(ShifterSwitchPin, INPUT_PULLUP);


  if(!CAN.begin(500E3)){
    Serial.println("Starting CAN fail");
    while(1);
  }

}

void loop() {
    //TODO

}