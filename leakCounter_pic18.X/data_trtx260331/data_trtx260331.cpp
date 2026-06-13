/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// // Include Particle Device OS APIs
#include "Particle.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

const pin_t PIC_DATA = D3;
const pin_t PIC_CLK  = D2;
const pin_t PIC_WAKE = D4; //

static const uint16_t CLK_HIGH_US = 120;
static const uint16_t CLK_LOW_US  = 120;
static const uint16_t FIRST_BIT_SETUP_US = 600;
static const uint8_t EXPECTED_FLAGS = 0xA5;
static const uint8_t TOGGLE_DATA_MARKER = 0xB5;
static const uint16_t MAX_TOGGLE_PAIRS = 180;  // 6 hours worth

void printPicPinStates() {
    Serial.printf(
        "PIC pins: DATA=%d CLK=%d WAKE=%d\r\n",
        digitalRead(PIC_DATA),
        digitalRead(PIC_CLK),
        digitalRead(PIC_WAKE)
    );
}

uint8_t readByteFromPIC() {
    uint8_t v = 0;

    // Hold CLK low before the first edge so PIC can present bit 7.
    digitalWrite(PIC_CLK, LOW);
    delayMicroseconds(FIRST_BIT_SETUP_US);

    for (int i = 0; i < 8; i++) {
        digitalWrite(PIC_CLK, HIGH);
        delayMicroseconds(CLK_HIGH_US / 3);

        // Majority sample while CLK is high to reduce single-point noise.
        int s0 = digitalRead(PIC_DATA);
        delayMicroseconds(CLK_HIGH_US / 3);
        int s1 = digitalRead(PIC_DATA);
        delayMicroseconds(CLK_HIGH_US / 3);
        int s2 = digitalRead(PIC_DATA);
        int bit = (s0 + s1 + s2) >= 2;

        v <<= 1;
        if (bit) {
            v |= 1;
        }

        digitalWrite(PIC_CLK, LOW);
        delayMicroseconds(CLK_LOW_US);
    }
    return v;
}

void sendAck8Clocks() {
    // PIC polls for ACK using __delay_ms(1) intervals in wait_for_ack_8clocks.
    // Each pulse must be long enough (>1ms per phase) to be detected reliably.
    for (int i = 0; i < 8; i++) {
        digitalWrite(PIC_CLK, HIGH);
        delay(3);   // 3ms high — well above 1ms poll interval
        digitalWrite(PIC_CLK, LOW);
        delay(3);   // 3ms low
    }
}

void sendStartPulse() {
    // PIC polls CLK every 1ms in wait_for_photon_start, so hold HIGH
    // for at least 3ms to guarantee it is detected on at least one poll.
    digitalWrite(PIC_CLK, HIGH);
    delay(5);                   // 5ms — well above 1ms poll interval
    digitalWrite(PIC_CLK, LOW);
    delay(3);                   // 3ms post-pulse: PIC exits wait, sets up bit 7
}

bool readMinuteFrame(uint16_t &count, uint8_t &flags) {
    uint8_t hi = readByteFromPIC();
    uint8_t lo = readByteFromPIC();
    flags = readByteFromPIC();

    // Give PIC time to exit send_byte, clear DATA, and enter wait_for_ack_8clocks.
    delay(5);
    sendAck8Clocks();

    count = ((uint16_t)hi << 8) | lo;
    return true; 
}

// Unpack 20 bits (10-bit time + 10-bit data) from 3 bytes
void unpackTogglePair(const uint8_t *buf, uint16_t &time, uint16_t &data) {
    uint16_t combined = ((uint16_t)buf[0] << 10) | ((uint16_t)buf[1] << 2) | ((uint16_t)buf[2] >> 6);
    time = combined >> 10;  // upper 10 bits (0-1023 minutes)
    data = combined & 0x3FF; // lower 10 bits (state/event data)
}

// Read toggle meter data frame
bool readToggleDataFrame(uint16_t &pairCount, uint8_t *toggleBuffer, uint16_t maxBytes) {
    uint8_t countHi = readByteFromPIC();
    uint8_t countLo = readByteFromPIC();
    uint8_t marker = readByteFromPIC();
    
    pairCount = ((uint16_t)countHi << 8) | countLo;
    
    if (marker != TOGGLE_DATA_MARKER) {
        Serial.printf("ERROR: Invalid toggle marker 0x%02X (expected 0x%02X)\r\n", marker, TOGGLE_DATA_MARKER);
        return false;
    }
    
    // Read all the bit-packed pairs
    uint16_t byteCount = pairCount * 3;
    if (byteCount > maxBytes) {
        Serial.printf("ERROR: Toggle buffer too large (%u bytes requested, %u available)\r\n", byteCount, maxBytes);
        pairCount = maxBytes / 3;  // Truncate to fit
        byteCount = pairCount * 3;
    }
    
    for (uint16_t i = 0; i < byteCount; i++) {
        toggleBuffer[i] = readByteFromPIC();
    }
    
    // Give PIC time to exit send_byte, clear DATA, and enter wait_for_ack_8clocks.
    delay(5);
    sendAck8Clocks();
    
    return true;
}

// Format toggle meter data into readable string (compact format: HH:MM count; HH:MM count)
String formatToggleData(uint16_t pairCount, const uint8_t *toggleBuffer) {
    String result = "";
    result.reserve(512);  // Pre-allocate for efficiency
    
    for (uint16_t i = 0; i < pairCount; i++) {
        uint16_t minute = 0;
        uint16_t state = 0;
        unpackTogglePair(&toggleBuffer[i * 3], minute, state);
        
        // Convert minute to HH:MM format
        uint16_t hours = minute / 60;
        uint16_t mins = minute % 60;
        
        // Format: "HH:MM count; HH:MM count"
        if (i > 0) {
            result += "; ";
        }
        result += String::format("%02u:%02u %u", hours, mins, state);
    }
    
    return result;
}

void setup() {
    pinMode(PIC_DATA, INPUT);
    pinMode(PIC_CLK, OUTPUT);
    digitalWrite(PIC_CLK, LOW);
    pinMode(PIC_WAKE, INPUT_PULLDOWN);
    pinMode(BTN, INPUT_PULLUP);

    Serial.begin(115200);
    waitFor(Serial.isConnected, 3000);
}
    
static uint16_t lastMinuteCount = 0;
static uint8_t toggleDataBuffer[540];  // 180 pairs * 3 bytes each

void loop() {
    static unsigned long wakeTime = 0;
    static const unsigned long AWAKE_DURATION = 100000; // in milliseconds
    static bool lastBtnState = HIGH;
    static bool lastWakeState = LOW;

    bool wakeState = (digitalRead(PIC_WAKE) == HIGH);
    bool wakeRising = (wakeState && !lastWakeState);

    // Debounce: require WAKE to stay HIGH for 50ms before trusting the edge.
    // This rejects spurious glitches during PIC reset (RA4 floats before gpio_init).
    if (wakeRising) {
        delay(50);
        wakeRising = (digitalRead(PIC_WAKE) == HIGH);
    }

    if (wakeState) {
        // Record the time when device first wakes up
        if (wakeTime == 0) {
            wakeTime = millis();
        }
    }

    if (wakeRising) {
        // PIC raises WAKE, then sleeps 2ms before entering wait_for_photon_start.
        // Wait here so the start pulse arrives after PIC is polling CLK.
        delay(10);

        sendStartPulse();

        // Read header to determine frame type
        uint8_t header[3];
        header[0] = readByteFromPIC();
        header[1] = readByteFromPIC();
        uint8_t frameType = readByteFromPIC();
        
        if (frameType == EXPECTED_FLAGS) {
            // Standard minute count frame
            uint16_t minuteCount = ((uint16_t)header[0] << 8) | header[1];
            lastMinuteCount = minuteCount;
            
            Serial.printf("Frame raw: HI=0x%02X LO=0x%02X FLAGS=0x%02X\r\n",
                header[0], header[1], frameType);
            Serial.printf("Minute pulses: %u\r\n", minuteCount);
            
            // Give PIC time to exit send_byte, clear DATA, and enter wait_for_ack_8clocks.
            delay(5);
            sendAck8Clocks();
            
            // Example conversion if your meter has K pulses per gallon:
            // float gallons = minuteCount / K;
            // Particle.publish("flow_minute_count", String(minuteCount), PRIVATE);
        }
        else if (frameType == TOGGLE_DATA_MARKER) {
            // Toggle data frame with bit-packed pairs
            uint16_t pairCount = ((uint16_t)header[0] << 8) | header[1];
            
            // Read all the bit-packed pairs
            uint16_t byteCount = pairCount * 3;
            if (byteCount <= sizeof(toggleDataBuffer)) {
                for (uint16_t i = 0; i < byteCount; i++) {
                    toggleDataBuffer[i] = readByteFromPIC();
                }
                
                // Give PIC time to exit send_byte, clear DATA, and enter wait_for_ack_8clocks.
                delay(5);
                sendAck8Clocks();
                
                // Format and display toggle data in compact format
                String formattedData = formatToggleData(pairCount, toggleDataBuffer);
                Serial.printf("Toggle Meter (%u events): ", pairCount);
                Serial.println(formattedData);
                
                // Optionally publish to Particle cloud
                // Particle.publish("toggle_meter_data", formattedData, PRIVATE);
            }
            else {
                Serial.printf("ERROR: Toggle buffer too large (%u bytes)\r\n", byteCount);
                // Still need to read and ack the data to keep protocol in sync
                for (uint16_t i = 0; i < byteCount && i < sizeof(toggleDataBuffer); i++) {
                    toggleDataBuffer[i] = readByteFromPIC();
                }
                delay(5);
                sendAck8Clocks();
            }
        }
        else {
            Serial.printf("ERROR: Unknown frame type 0x%02X\r\n", frameType);
            // Try to recover by sending ACK
            delay(5);
            sendAck8Clocks();
        }
    }
    lastWakeState = wakeState;

    bool btnState = digitalRead(BTN);
    if (lastBtnState == HIGH && btnState == LOW) {
        // Active-low built-in Particle button.
        Serial.printf("Button pressed - Last minute pulses: %u\r\n", lastMinuteCount);
        printPicPinStates();
        
        // Display cached toggle data if available
        if (sizeof(toggleDataBuffer) > 0) {
            Serial.printf("\r\n===== Cached Toggle Meter Data =====\r\n");
            // Note: You could request new data here or just display what's cached
            // For now, this displays the format as a reminder
            Serial.printf("(Use button with data transmission to populate)\r\n");
            Serial.printf("=======================================\r\n\r\n");
        }
    }
    lastBtnState = btnState;

    // Only sleep if 30 seconds have passed since waking up, or if not currently woken
    if (wakeTime != 0 && (millis() - wakeTime) >= AWAKE_DURATION) {
        wakeTime = 0; // Reset wake time before sleeping
        Serial.printf("Sleep after awake %lu seconds\r\n", AWAKE_DURATION / 1000);  

        SystemSleepConfiguration config;
        config.mode(SystemSleepMode::HIBERNATE)
              .gpio(PIC_WAKE, RISING);

        System.sleep(config);
    }
}

