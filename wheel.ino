#include <Arduino.h>
#define TrimUpPin 26
#define TrimDownPin 25
#define GainUpPin 32
#define GainDownPin 33
#define EStopPin 34
#define EncoderPin 36
#define ThrottlePin 39
#define LED1 23
#define LED2 22
#define LED3 18
#define LED4 17
#define LED5 21
#define LED6 19

#include <CAN.h>

int encoderVal = 0;
int throttleVal = 0;
uint16_t i = 0;

void setup() {
  Serial.begin(9600);

  while(!Serial){}

  Serial.println("CAN mcu");

  pinMode(GainUpPin, INPUT);
  pinMode(TrimUpPin, INPUT);
  pinMode(TrimDownPin,INPUT);
  pinMode(GainDownPin, INPUT);
  pinMode(EStopPin, INPUT);
  pinMode(EncoderPin, INPUT);
  pinMode(ThrottlePin, INPUT);
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
  encodertoCAN(encoderVal);

  throttleVal = readThrottle();
  throttletoCAN(throttleVal);

  buttonstoCAN();

  LEDReceive();
}

float readEncoder(){
  return (int)analogRead(EncoderPin)*360./4095.; //Scale => [0,360]
}

float readThrottle(){
  return (int)analogRead(ThrottlePin)*100./4095.; //Scale => [0,100]
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
  CAN.write(bool(analogRead(TrimUpPin)));
  CAN.write(bool(analogRead(TrimDownPin)));
  CAN.write(bool(analogRead(GainUpPin)));
  CAN.write(bool(analogRead(GainDownPin)));
  CAN.write(bool(analogRead(EStopPin)));
  CAN.endPacket();
  //cSerial.println("sent");
}


void LEDReceive() {

    // try to parse packet
  int packetSize = CAN.parsePacket();

  if (packetSize && CAN.packetId() == 2001) {
    // received a packet
    Serial.print("Received ");

    if (CAN.packetExtended()) {
      //Serial.print("extended ");
    }

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      //Serial.print("RTR ");
    }

    //Serial.print("packet with id 0x");
    //Serial.print(CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
      //Serial.print(" and requested length ");
      //Serial.println(CAN.packetDlc());
    } else {
      //Serial.print(" and length ");
      //Serial.println(packetSize);

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
      //Serial.println();
    }

    Serial.println();
  }
}
