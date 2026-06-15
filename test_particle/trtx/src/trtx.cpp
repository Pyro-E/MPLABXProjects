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

// // setup() runs once, when the device is first turned on
// void setup() {
//   // Put initialization like pinMode and begin functions here
// }

// // loop() runs over and over again, as quickly as it can execute.
// void loop() {
//   // The core of your code will likely live here.

//   // Example: Publish event to cloud every 10 seconds. Uncomment the next 3 lines to try it!
//   // Log.info("Sending Hello World to the cloud!");
//   // Particle.publish("Hello world!");
//   // delay( 10 * 1000 ); // milliseconds and blocking - see docs for more info!
// }


// #include "Particle.h"

// SYSTEM_THREAD(ENABLED);

// UART from PIC:
// PIC TX -> Photon RX
// PIC GND -> Photon GND
// Baud: 9600, 8N1

void setup() {
    Serial.begin(115200);     // USB debug
    Serial1.begin(9600);      // UART from PIC

    waitFor(Serial.isConnected, 4000);
    Serial.println("Photon UART receiver started.");
}

void loop() {
    while (Serial1.available() > 0) {
        char c = Serial1.read();

        if (c == '1') {
            Serial.println("Received 1 from PIC");

            // Example action:
            // digitalWrite(D7, HIGH);
            // Particle.publish("pic_msg", "1", PRIVATE);
        } else {
            Serial.print("Received other char: ");
            Serial.println(c);
        }
    }
}