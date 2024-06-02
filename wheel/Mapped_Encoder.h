/*
  Mapped_Encoder.h - Library for getting input from a potentiometer or encoder and mapping it
  Created by David Anapolsky, May 21, 2024.
*/

#ifndef Mapped_Encoder_h
#define Mapped_Encoder_h

#include "Arduino.h"

#include <EEPROM.h>

#define BITMAX10 1023
#define BITMAX12 4095

// To check if the EEPROM is empty
#define __signature 602

class Mapped_Encoder {
  private:
    int pin;
    const uint16_t analogMax;

    uint32_t out_min;
    uint32_t out_max;

    const String _name;

    void set_passes_zero();

    void update_EEPROM();

    struct EEPROMVals {
      uint8_t signature;
      uint16_t min;
      uint16_t max;
      bool passes_zero;
    };

    EEPROMVals inVals;
    int eeAddress;

  public:
    Mapped_Encoder(int pin, int eeAddress, const uint16_t analogMax, uint32_t out_min, uint32_t out_max, const String name);

    float read() const;
    uint16_t readRaw() const;

    const String & name() const;

    void set_analog_vals(uint16_t min, uint16_t max);

    float floatMap(float x) const;

    uint16_t get_analogMax() const;

    int get_EEPROM_usage() const;
};

#endif
