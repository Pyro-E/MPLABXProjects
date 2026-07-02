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

const pin_t powPin = D3;
const pin_t pulsePin = D6;
const unsigned long pulseOnMs = 30UL * 1000UL;
const unsigned long pulseOffMs = 5UL * 60UL * 1000UL;


// Button state tracking for edge detection + debounce
int btnState = HIGH;
int lastBtnState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

bool pulseOn = true;
unsigned long pulsePhaseStartMs = 0;

// setup() runs once, when the device is first turned on
void setup() {
  pinMode(powPin, OUTPUT);
  digitalWrite(powPin, HIGH);
  pinMode(pulsePin, OUTPUT);
  digitalWrite(pulsePin, HIGH);
  pulsePhaseStartMs = millis();

  // The on-board `BUTTON` is already configured by Device OS; don't reconfigure it here.
  // Initialize button state tracking from the actual pin state.
  lastBtnState = System.buttonPushed() > 0 ? LOW : HIGH; // active-low button
  btnState = lastBtnState;
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  unsigned long now = millis();

  if (pulseOn && (now - pulsePhaseStartMs >= pulseOnMs)) {
    pulseOn = false;
    pulsePhaseStartMs = now;
    digitalWrite(pulsePin, LOW);
    Log.info("D6 LOW for 20 minutes");
  } else if (!pulseOn && (now - pulsePhaseStartMs >= pulseOffMs)) {
    pulseOn = true;
    pulsePhaseStartMs = now;
    digitalWrite(pulsePin, HIGH);
    Log.info("D6 HIGH for 5 seconds (periodic)");
  }

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
        pulseOn = true;
        pulsePhaseStartMs = now;
        digitalWrite(pulsePin, HIGH);
        Log.info("D6 HIGH for 5 seconds (button)");
      }
    }
  }

  lastBtnState = reading;
}
