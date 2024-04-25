#include <Arduino.h>
#define Man1 13
#define Man2 12
#define Man3 14
#define Man4 27
#define Man5 26
#define Man6 25

#include <CAN.h>

void setup() {
  Serial.begin(9600);

  while(!Serial){}

  Serial.println("CAN mcu 2");

  pinMode(Man1, OUTPUT);
  pinMode(Man2, OUTPUT);
  pinMode(Man3, OUTPUT);
  pinMode(Man4, OUTPUT);
  pinMode(Man5, OUTPUT);
  pinMode(Man6, OUTPUT);

  if(!CAN.begin(500E3)){
    Serial.println("Starting CAN fail");
    while(1);
  }
}

void loop() {

  ManifoldReceive();
}



void ManifoldReceive() {

    // try to parse packet
  int packetSize = CAN.parsePacket();

  if (packetSize && CAN.packetId() == 0x7D2) {
    // received a packet
    //Serial.print("Received ");

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
        Serial.print(CAN.read());
        data[i] = (char)CAN.peek();
        //Serial.println((int)data[i]);
        i++;
      }

      if(data[0] == 1){
        digitalWrite(Man1, HIGH);
        //Serial.print("hi");
      }else{
        digitalWrite(Man1, LOW);
      }
      
      if(data[1] == 1){
        digitalWrite(Man2, HIGH);
      }else{
        digitalWrite(Man2, LOW);
      }

      if(data[2] == 1){
        digitalWrite(Man3, HIGH);
      }else{
        digitalWrite(Man3, LOW);
      }

      if(data[3] == 1){
        digitalWrite(Man4, HIGH);
      }else{
        digitalWrite(Man4, LOW);
      }

      if(data[4] == 1){
        digitalWrite(Man5, HIGH);
      }else{
        digitalWrite(Man5, LOW);
      }
  
      if(data[5] == 1){
        digitalWrite(Man6, HIGH);
      }else{
        digitalWrite(Man6, LOW);
      }
    }

    Serial.println();
  }
}
