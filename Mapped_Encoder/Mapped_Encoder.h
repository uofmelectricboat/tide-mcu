/*
  Mapped_Encoder.h - Library for getting input from a potentiometer or encoder and mapping it
  Created by David Anapolsky, May 21, 2024.
*/

#ifndef Mapped_Encoder_h
#define Mapped_Encoder_h

#include "Arduino.h"

#define BITMAX10 1023
#define BITMAX12 4095

class Mapped_Encoder {
  private:
    int pin;
    int in_min;
    int in_max;
    int out_min;
    int out_max;

    const int analogMax;
    const String _name;
    bool passes_zero;

    void set_passes_zero();
  
  public:
    Mapped_Encoder(int pin, int in_min, int in_max, int out_min, int out_max, const int analogMax, const String name);

    float read();
    uint16_t readRaw();
    const String & name();
    void set_min(int analogVal);
    void set_max(int analogVal);

    float floatMap(float x);

    int get_analogMax();
};

#endif
