/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

// Button and LED pins
// const int btn = BUTTON; // user button
const int powPin = D3; // target LED pin to toggle
const int ledPin = D4; // target LED pin to toggle


// Button state tracking for edge detection + debounce
bool ledState = false;
int btnState = HIGH;
int lastBtnState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// setup() runs once, when the device is first turned on
void setup() {
  // Put initialization like pinMode and begin functions here
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(powPin, OUTPUT);
  digitalWrite(powPin, HIGH);

  // The on-board `BUTTON` is already configured by Device OS; don't reconfigure it here.
  // Initialize button state tracking from the actual pin state.
  lastBtnState = System.buttonPushed() > 0 ? LOW : HIGH; // active-low button
  btnState = lastBtnState;
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // The core of your code will likely live here.

  int reading = System.buttonPushed() > 0 ? LOW : HIGH; // active-low button

  // If the button reading changed, reset the debounce timer
  if (reading != lastBtnState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the button state has changed (and is stable), take action
    if (reading != btnState) {
      btnState = reading;

      // Button is active-low (INPUT_PULLUP): press -> LOW
      if (btnState == LOW) {
        ledState = !ledState;
        digitalWrite(ledPin, ledState ? HIGH : LOW);
        Log.info("D4 toggled %s", ledState ? "HIGH" : "LOW");
      }
    }
  }

  lastBtnState = reading;

  delay(10);
}
