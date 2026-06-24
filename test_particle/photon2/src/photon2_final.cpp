/*
 * Project: PIC meter reader (Photon 2) - UART version (HIBERNATE, USB-CDC safe)
 *
 * Based on the known-good HIBERNATE build (USB-CDC debug works because HIBERNATE
 * re-runs setup() -> Serial.begin() on every wake). On top of that:
 *   - Compile-time decoder select (DECODE_METHOD), default 10-10-4.
 *   - COUNT == 0 treated as a valid empty upload.
 *   - 0xAA rate limit: at most once per second, enforced with millis() while
 *     awake (NO RTC / retained memory). Each exchange is followed by a 1 s
 *     busy period, so a held button or repeated WAKE cannot fire another 0xAA
 *     until it elapses. This works even with no cloud time available.
 *
 * Protocol (per PIC <-> Photon2 spec):
 *   - UART 38400 8N1 on the physical TX/RX pins (Serial1).
 *   - Photon2 sends 0xAA; PIC replies with COUNT (4 bytes, big-endian) + COUNT
 *     samples back-to-back.
 *       Method 0 "2B-2B"  : 4 bytes/sample, index/pulses each 16-bit big-endian.
 *       Method 1 "10-10-4": 3 bytes/sample, 24-bit BE word:
 *                           index = bits[23:14], pulses = bits[13:4], low 4 bits = 0.
 */

#include "Particle.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

// =====================================================================
//  COMPILE-TIME METHOD SELECT — set this to match the PIC build.
//    0 = 2B-2B   (4 bytes per sample)
//    1 = 10-10-4 (3 bytes per sample)   <-- default
// =====================================================================
#define DECODE_METHOD 1

#if   DECODE_METHOD == 0
  static const uint8_t BYTES_PER_SAMPLE = 4;
#elif DECODE_METHOD == 1
  static const uint8_t BYTES_PER_SAMPLE = 3;
#else
  #error "DECODE_METHOD must be 0 (2B-2B) or 1 (10-10-4)"
#endif

// PIC drives this line HIGH to wake the Photon 2.
const pin_t PIC_WAKE = D10;
const pin_t VALV_CNTR = D4;
const pin_t VALV_POWR = D3;


static const unsigned long PIC_BAUD          = 38400;  // Must match the PIC UART baud rate
static const uint32_t      MAX_RECORDS       = 4096;   // Sanity cap (PIC ring buffer ~360 slots)
static const uint32_t      READ_TIMEOUT_MS   = 200;    // Per-byte receive timeout
static const uint32_t      WAKE_LOW_WAIT_MS  = 500;    // Max wait for PIC to release WAKE
static const uint32_t      MIN_CYCLE_MS      = 1000;   // Minimum time per exchange cycle (1 s)

// Print current pin / link status (debug helper).
void printPicPinStates() {
    Serial.printf("PIC WAKE=%d  Serial1 avail=%d\r\n",
                  digitalRead(PIC_WAKE), Serial1.available());
}

// Receive one byte from the PIC over Serial1, with a timeout.
bool readByteFromPIC(uint8_t &out) {
    uint32_t start = millis();
    while (!Serial1.available()) {
        if (millis() - start >= READ_TIMEOUT_MS) return false;
    }
    out = (uint8_t)Serial1.read();
    return true;
}

// Read exactly n bytes into buf (wire order preserved). Returns false on timeout.
bool readBytesFromPIC(uint8_t *buf, uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        if (!readByteFromPIC(buf[i])) return false;
    }
    return true;
}

// Assemble the 4-byte big-endian COUNT: 01 02 03 04 -> (01<<24)|...|04.
bool readCount(uint32_t &val) {
    uint8_t b[4];
    if (!readBytesFromPIC(b, 4)) return false;   // b[0] = MSB ... b[3] = LSB
    val = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
          ((uint32_t)b[2] << 8)  |  (uint32_t)b[3];
    return true;
}

// Decode one sample into index/pulses, per DECODE_METHOD.
void decodeSample(const uint8_t *b, uint16_t &index, uint16_t &pulses) {
#if DECODE_METHOD == 0
    index  = ((uint16_t)b[0] << 8) | b[1];
    pulses = ((uint16_t)b[2] << 8) | b[3];
#else
    uint32_t word = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    index  = (uint16_t)((word >> 14) & 0x3FF);
    pulses = (uint16_t)((word >> 4)  & 0x3FF);
#endif
}

// Send 0xAA, then receive one upload block (COUNT + samples) and print it.
void readAndPrintWindow() {
    // Flush any stale bytes left in the RX buffer before requesting.
    while (Serial1.available()) Serial1.read();

    Serial1.write(0xAA);   // Request data
    Serial1.flush();       // Block until the request byte is physically transmitted

    uint32_t count;
    if (!readCount(count)) {
        Serial.printf("ERROR: timeout reading count\r\n");
        return;
    }

    // COUNT == 0 is normal: 0xAA arrived with nothing new buffered.
    if (count == 0) {
        Serial.printf("--- empty upload (no new samples) ---\r\n");
        return;
    }
    if (count > MAX_RECORDS) {
        Serial.printf("ERROR: bad count=%lu (cap=%lu)\r\n",
                      (unsigned long)count, (unsigned long)MAX_RECORDS);
        return;
    }

    Serial.printf("--- upload: %lu samples (method %d, %u B/sample) ---\r\n",
                  (unsigned long)count, DECODE_METHOD, BYTES_PER_SAMPLE);

    uint16_t lastIndex = 0, lastPulses = 0;
    for (uint32_t i = 0; i < count; i++) {
        uint8_t b[4];   // large enough for either method
        if (!readBytesFromPIC(b, BYTES_PER_SAMPLE)) {
            Serial.printf("ERROR: timeout at sample %lu\r\n", (unsigned long)i);
            break;
        }
        uint16_t index, pulses;
        decodeSample(b, index, pulses);
        Serial.printf("  [%lu] index=%u  pulses=%u\r\n",
                      (unsigned long)i, index, pulses);
        lastIndex  = index;
        lastPulses = pulses;
    }

    Serial.printf("Latest: index=%u  pulses=%u\r\n", lastIndex, lastPulses);
    Serial.printf("------------------------------------------\r\n");
}

void setup() {
    pinMode(PIC_WAKE, INPUT_PULLDOWN);
    pinMode(BTN, INPUT_PULLUP);
    pinMode(VALV_CNTR, OUTPUT);
    pinMode(VALV_POWR, OUTPUT);
    digitalWrite(VALV_CNTR, HIGH);
    digitalWrite(VALV_POWR, HIGH);

    Serial.begin();                        // USB CDC: baud argument is ignored
    Serial1.begin(PIC_BAUD, SERIAL_8N1);   // PIC link on physical TX/RX pins, 38400 8N1
    waitFor(Serial.isConnected, 300);      // Short, bounded wait so it never stalls the handshake
}

void loop() {
    // After a wake, the device has just reset (HIBERNATE). Check current levels:
    // start a PIC exchange if the PIC raised WAKE, OR if the MODE button is pressed.
    static bool lastButtonState = HIGH;
    bool currentButtonState = digitalRead(BTN);
    bool buttonPressed = (currentButtonState == LOW && lastButtonState == HIGH);
    bool wokeByPic    = (digitalRead(PIC_WAKE) == HIGH);
    bool wokeByButton = (currentButtonState == LOW);

    if (buttonPressed) {
        bool currentValvState = digitalRead(VALV_CNTR);
        digitalWrite(VALV_CNTR, currentValvState ? LOW : HIGH);
    } else if (wokeByButton) {
        printPicPinStates();     // optional debug snapshot
    }

    lastButtonState = currentButtonState;

    if (wokeByPic || wokeByButton) {
        uint32_t cycleStart = millis();

        delay(10);
        readAndPrintWindow();    // sends 0xAA, then receives and prints
        Serial.flush();

        // Wait for the PIC to release WAKE before sleeping (bounded wait).
        for (unsigned long t = millis();
             digitalRead(PIC_WAKE) && millis() - t < WAKE_LOW_WAIT_MS;
             delay(1));

        // Enforce a minimum 1-second cycle: stay busy until 1 s has elapsed
        // since this exchange started. A held button or repeated WAKE cannot
        // trigger another 0xAA until then. Uses millis() only (no RTC).
        while (millis() - cycleStart < MIN_CYCLE_MS) delay(1);
    }

    // HIBERNATE: wake on a PIC trigger (RISING) or a button press (FALLING).
    // HIBERNATE re-runs setup() on wake, which keeps the USB-CDC debug port alive.
    SystemSleepConfiguration cfg;
    cfg.mode(SystemSleepMode::HIBERNATE)
       .gpio(PIC_WAKE, RISING)
       .gpio(BTN, FALLING);
    System.sleep(cfg);
}
