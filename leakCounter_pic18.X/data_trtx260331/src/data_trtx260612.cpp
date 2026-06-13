/*
 * Project myProject
 * Author: Your Name
 * Date:
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

#include "Particle.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

const pin_t PIC_DATA = D3;
const pin_t PIC_CLK  = D2;
const pin_t PIC_WAKE = D4;

static const uint16_t CLK_HIGH_US        = 120;
static const uint16_t CLK_LOW_US         = 120;
static const uint16_t FIRST_BIT_SETUP_US = 600;
static const uint8_t  METER_WINDOW_SIZE  = 10;

void printPicPinStates() {
    Serial.printf(
        "PIC pins: DATA=%d CLK=%d WAKE=%d\r\n",
        digitalRead(PIC_DATA),
        digitalRead(PIC_CLK),
        digitalRead(PIC_WAKE)
    );
}

// Read one byte: Boron drives CLK, PIC drives DATA
uint8_t readByteFromPIC() {
    uint8_t v = 0;

    digitalWrite(PIC_CLK, LOW);
    delayMicroseconds(FIRST_BIT_SETUP_US);

    for (int i = 0; i < 8; i++) {
        digitalWrite(PIC_CLK, HIGH);
        delayMicroseconds(CLK_HIGH_US / 3);

        // Majority-vote sample to reduce noise
        int s0 = digitalRead(PIC_DATA);
        delayMicroseconds(CLK_HIGH_US / 3);
        int s1 = digitalRead(PIC_DATA);
        delayMicroseconds(CLK_HIGH_US / 3);
        int s2 = digitalRead(PIC_DATA);
        int bit = (s0 + s1 + s2) >= 2;

        v = (v << 1) | bit;

        digitalWrite(PIC_CLK, LOW);
        delayMicroseconds(CLK_LOW_US);
    }
    return v;
}

void sendAck8Clocks() {
    for (int i = 0; i < 8; i++) {
        digitalWrite(PIC_CLK, HIGH);
        delay(3);
        digitalWrite(PIC_CLK, LOW);
        delay(3);
    }
}

void sendStartPulse() {
    digitalWrite(PIC_CLK, HIGH);
    delay(5);
    digitalWrite(PIC_CLK, LOW);
    delay(3);
}

// Unpack 3-byte packed pair: 4-bit interval (upper nibble of b0) + 16-bit count (b1:b2)
void unpackMeterPair(const uint8_t *buf, uint8_t &interval, uint16_t &count) {
    interval = buf[0] >> 4;
    count    = ((uint16_t)buf[1] << 8) | buf[2];
}

// Read pair_count + full buffer from PIC, then print the complete window.
// Wire format: 1 byte pair_count, then pair_count*3 bytes of packed pairs.
// Buffer and counting are managed on the PIC; Boron just receives and displays.
void readAndPrintWindow() {
    uint8_t pair_count = readByteFromPIC();

    if (pair_count == 0 || pair_count > METER_WINDOW_SIZE) {
        Serial.printf("ERROR: bad pair_count=%u\r\n", pair_count);
        delay(5);
        sendAck8Clocks();
        return;
    }

    uint8_t buf[30];  // 10 pairs * 3 bytes max
    for (uint8_t i = 0; i < pair_count * 3; i++) {
        buf[i] = readByteFromPIC();
    }
    delay(5);
    sendAck8Clocks();

    uint8_t  latest_interval;
    uint16_t latest_count;
    unpackMeterPair(&buf[(pair_count - 1) * 3], latest_interval, latest_count);

    Serial.printf("Frame: interval=%u/9  count=%u\r\n", latest_interval, latest_count);
    Serial.printf("--- 10-min window (interval %u received) ---\r\n", latest_interval);
    for (uint8_t i = 0; i < pair_count; i++) {
        uint8_t  interval;
        uint16_t count;
        unpackMeterPair(&buf[i * 3], interval, count);
        Serial.printf("  [%u/9] %u pulses\r\n", interval, count);
    }
    Serial.printf("--------------------------------------------\r\n");
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

void loop() {
    static bool lastBtnState = HIGH;

    // Only act on PIC trigger; WAKE is HIGH on wakeup from ULP sleep or first power-on
    if (digitalRead(PIC_WAKE) == HIGH) {
        delay(10);
        sendStartPulse();
        readAndPrintWindow();
        Serial.flush();

        // Wait for PIC to lower WAKE before sleeping (up to 500 ms)
        for (unsigned long t = millis(); digitalRead(PIC_WAKE) && millis() - t < 500; delay(1));
    }

    bool btnState = digitalRead(BTN);
    if (lastBtnState == HIGH && btnState == LOW) {
        printPicPinStates();
    }
    lastBtnState = btnState;

    // ULTRA_LOW_POWER preserves statics across sleep; wakes only on PIC trigger or button
    SystemSleepConfiguration cfg;
    cfg.mode(SystemSleepMode::HIBERNATE)
       .gpio(PIC_WAKE, RISING)
       .gpio(BTN, FALLING);
    System.sleep(cfg);
}
