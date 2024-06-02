/*
  Mapped_Encoder.cpp - Library for getting input from a potentiometer or encoder, mapping it, 
  and storing its settings in EEPROM
  Created by David Anapolsky, May 21, 2024.
*/

#include "Arduino.h"
#include "Mapped_Encoder.h"
#include <EEPROM.h>

// MODIFIES : passes zero boolean
// EFFECTS  : sets the passes zero state for use by the floatMap
void Mapped_Encoder::set_passes_zero() {
  inVals.passes_zero = (inVals.min > inVals.max);
}

// MODIFIES : EEPROM
void Mapped_Encoder::update_EEPROM() {
  EEPROM.put(eeAddress, inVals);
}

Mapped_Encoder::Mapped_Encoder(int pin, int eeAddress, const uint16_t analogMax, uint32_t out_min, uint32_t out_max, const String name) : 
  pin(pin), eeAddress(eeAddress), analogMax(analogMax), out_min(out_min), out_max(out_max), _name(name) {
    
    EEPROM.get(eeAddress, inVals);

    // Set to default values if the signature doesn't match
    if (inVals.signature != __signature) {
      inVals.signature = __signature;
      inVals.min = 0;
      inVals.max = analogMax;

      set_passes_zero();
      update_EEPROM();
    }
  }

float Mapped_Encoder::read() const {
  return floatMap(readRaw());
}

uint16_t Mapped_Encoder::readRaw() const {
  return analogRead(pin);
}

const String & Mapped_Encoder::name() const{
  return _name;
}

// REQUIRES : min and max are unsigned integers
// MODIFIES : analog min and max values, passes zero, and EEPROM
// EFFECTS  : mapping of analog reading
void Mapped_Encoder::set_analog_vals(uint16_t min, uint16_t max) {
  inVals.min = min;
  inVals.max = max;

  set_passes_zero();

  update_EEPROM();
}

// REQUIRES : val is a valid analog value
// EFFECTS  : returns the analog values mapped to the range of the output values
float Mapped_Encoder::floatMap(float val) const {
  if (inVals.passes_zero && val < inVals.max) {
    // Add the analogMax to val
    val += analogMax;
  }
  
  return (val - inVals.min) * (out_max - out_min) / abs(inVals.max - inVals.min) + out_min;
}

uint16_t Mapped_Encoder::get_analogMax() const {
  return analogMax;
}

// EFFECTS  : returns the number of bytes used in EEPROM
int Mapped_Encoder::get_EEPROM_usage() const {
  return sizeof(inVals);
}
