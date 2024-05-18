#include "debug.h"
// Debugging printing to only send to Serial when in DEBUG mode


// TODO consider passing by reference

bool DEBUG = true;

void debugMode(bool mode) {
    DEBUG = mode;
}

void debugPrint(const String val) {
  if (DEBUG) {
    Serial.print(val);
  }
}

void debugPrintln(const String val) {
  debugPrint(val);
  debugPrint("\n");
}

void debugPrint(const uint8_t val) {
  if (DEBUG) {
    Serial.print(val);
  }
}

void debugPrintln(const uint8_t val) {
  debugPrint(val);
  debugPrint("\n");
}

void debugPrint(const uint8_t val, const int format) {
  if (DEBUG) {
    Serial.print(val, format);
  }
}

void debugPrintln(const uint8_t val, const int format) {
    debugPrint(val, format);
    debugPrint("\n");
}



