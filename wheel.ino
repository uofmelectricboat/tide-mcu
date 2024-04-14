#include <Arduino.h>
#include <SPI.h>

//#define CAN_2515
#define CAN_2518FD


// Set SPI CS Pin according to your hardware
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;

#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN); // Set CS pin
#endif

#define TrimUpPin 1
#define TrimDownPin 2
#define GainUpPin 3
#define GainDownPin 4
#define EStopPin 5
#define EncoderPin 6
#define ThrottlePin 7
#define LED1 10

float encoderVal = 0;
float throttleVal = 0;
uint16_t i = 0;

void setup() {
  // put your setup code here, to run once:
    SERIAL_PORT_MONITOR.begin(115200);
    while(!Serial){};

    while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(100);
    }
    SERIAL_PORT_MONITOR.println("CAN init ok!");
  
  pinMode(TrimUpPin, INPUT);
  pinMode(TrimDownPin, INPUT);
  pinMode(GainUpPin, INPUT);
  pinMode(GainDownPin, INPUT);
  pinMode(EStopPin, INPUT);
  pinMode(EncoderPin, INPUT);
  pinMode(ThrottlePin, INPUT);
  pinMode(LED1, OUTPUT);
}

void loop() {
  encoderVal = readEncoder();
  encodertoCAN(encoderVal);

  throttleVal = readThrottle();
  throttletoCAN(throttleVal);

  buttonstoCAN();
}

float readEncoder(){
  return (float)analogRead(EncoderPin)*360./4095.; //Scale => [0,360]
}

float readThrottle(){
  return (float)analogRead(ThrottlePin)*360./4095.; //Scale => [0,360]
}

void encodertoCAN(float val){
  char data[sizeof(val)];                //Create char array
  memcpy(data, &val, sizeof(val));       //Store bytes of val to array
  CAN.sendMsgBuf(1999, 0, sizeof(val), data);
}

void throttletoCAN(float val){
  CAN.beginPacket(1998);
  char data[sizeof(val)];                //Create char array
  memcpy(data, &val, sizeof(val));       //Store bytes of val to array
  CAN.sendMsgBuf(1998, 0, sizeof(val), data);
}

/*----------------------------------
IDs
2000
-----------------------------------*/
void buttonstoCAN(){
  boolean trimU = digitalRead(TrimUpPin);
  boolean trimD = digitalRead(TrimDownPin);
  boolean gainU = digitalRead(GainUpPin);
  boolean gainD = digitalRead(GainDownPin);
  boolean stop = analogRead(EStopPin);

  char data[5];
  data[0] = trimU;
  data[1] = trimD;
  data[2] = gainU;
  data[3] = gainD;
  data[4] = stop;

  CAN.sendMsgBuf(2000, 0, 5, data);
}
