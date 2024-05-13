/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-button-debounce
 */

#define BUTTON_PIN     34 // GIOP21 pin connected to button
#define DEBOUNCE_TIME  50 // the debounce time in millisecond, increase this time if it still chatters

// Variables will change:
int lastSteadyState = HIGH;       // the previous steady state from the input pin
int lastFlickerableState = HIGH;  // the previous flickerable state from the input pin
int currentState;                // the current reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  //while(!Serial){}
  // initialize the pushbutton pin as an pull-up input
  // the pull-up input pin will be HIGH when the switch is open and LOW when the switch is closed.
  pinMode(BUTTON_PIN, INPUT);

  Serial.println("Begin");
}

void loop() {
  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_PIN);

  if (currentState == HIGH) {
     Serial.println("High");
  }
  else {
     Serial.println("Low");
  }

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch/button changed, due to noise or pressing:
  if (currentState != lastFlickerableState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
    // save the the last flickerable state
    lastFlickerableState = currentState;
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_TIME) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if(lastSteadyState == HIGH && currentState == LOW)
      Serial.println("The button is pressed");
    else if(lastSteadyState == LOW && currentState == HIGH)
      Serial.println("The button is released");

    // save the the last steady state
    lastSteadyState = currentState;
  }
}
