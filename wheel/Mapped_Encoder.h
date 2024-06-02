/*
  Mapped_Encoder.h - Library for getting input from a potentiometer or encoder, mapping it, 
  and storing its settings in EEPROM
  Created by David Anapolsky, May 21, 2024.
*/

#ifndef Mapped_Encoder_h
#define Mapped_Encoder_h

#include "Arduino.h"

#include <Preferences.h>

#define BITMAX10 1023
#define BITMAX12 4095

class Mapped_Encoder {
  private:
    void set_passes_zero();

    void update_preferences();

    struct EEPROMVals { 
      uint16_t min;
      uint16_t max;
      bool passes_zero;
    };

    Preferences pref;
    int pin;
    const uint16_t analogMax;

    uint32_t out_min;
    uint32_t out_max;

    const char * _name;

    EEPROMVals inVals;

  public:
    Mapped_Encoder(int pin, const uint16_t analogMax, uint32_t out_min, uint32_t out_max, const char * name);

    float read() const;
    uint16_t readRaw() const;

    const char * name() const;

    void set_analog_vals(uint16_t min, uint16_t max);

    float floatMap(float x) const;

    uint16_t get_analogMax() const;
};

#endif
