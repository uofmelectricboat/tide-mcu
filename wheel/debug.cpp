#include "debug.h"

// Debugging printing to only send to Serial when in DEBUG mode


bool DEBUG = true;

void mode(bool mode) {
    DEBUG = mode;
}

void print(const String val = "") {
  if (DEBUG) {
    Serial.print(val);
  }
}

void println(const String val = "") {
  print(val);
  print("\n");
}

void print(const uint8_t val) {
  if (DEBUG) {
    Serial.print(val);
  }
}

void println(const uint8_t val) {
  print(val);
  print("\n");
}

void print(const uint8_t val, const int format) {
  if (DEBUG) {
    Serial.print(val, format);
  }
}

void println(const uint8_t val, const int format) {
    print(val, format);
    print("\n");
}
