/*
  Mapped_Encoder.cpp - Library for getting input from a potentiometer or encoder and mapping it
  Created by David Anapolsky, May 21, 2024.
*/

#include "Arduino.h"
#include "Mapped_Encoder.h"

// EFFECTS: sets the passes zero state for use by the floatMap
void Mapped_Encoder::set_passes_zero() {
  passes_zero = in_min > in_max;
}

Mapped_Encoder::Mapped_Encoder(int pin, int in_min, int in_max, int out_min, int out_max, const int analogMax, const String name) : 
  pin(pin), in_min(in_min), in_max(in_max), out_min(out_max), analogMax(analogMax), _name(name) {
    set_passes_zero();
  }

float Mapped_Encoder::read() {
  return floatMap(readRaw());
}

uint16_t Mapped_Encoder::readRaw() {
  return analogRead(pin);
}

const String & Mapped_Encoder::name() {
  return _name;
}

// MODIFIES : analog min value
void Mapped_Encoder::set_min(int analogVal) {
  in_min = analogVal;

  set_passes_zero();
}

// MODIFIES : analog max value
void Mapped_Encoder::set_max(int analogVal) {
  in_max = analogVal;

  set_passes_zero();
}

float Mapped_Encoder::floatMap(float x) {
  if (passes_zero && x < in_max) {
    // Add the absolute max in to x
    x += analogMax;
  }
  
  return (x - in_min) * (out_max - out_min) / abs(in_max - in_min) + out_min;
}

int Mapped_Encoder::get_analogMax() {
  return analogMax;
}

