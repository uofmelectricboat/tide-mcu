/*
 * Based on the following:
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-potentiometer
 */

#define PotPin 34

int floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  analogReadResolution(12); // set the resolution to 12 bits
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin GPIO36:
  int analogValue = analogRead(PotPin);
  // Rescale to potentiometer's voltage (from 0V to 3.3V):
  float voltage = floatMap(analogValue, 0, 4095, 0, 3.3);

  // // Try to read using PWM
  // // Did not work
  // unsigned long duration = pulseIn(PotPin,LOW);
  // Serial.print("Duration: ");
  // Serial.println(duration);

  // print out the value you read:
  Serial.print("Analog: ");
  Serial.print(analogValue);
  Serial.print(", Voltage: ");
  Serial.println(voltage);
  delay(500);
}
