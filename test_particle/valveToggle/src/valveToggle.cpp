/*
 * Project: PIC meter reader (Photon 2) - UART version (HIBERNATE, USB-CDC safe)
 *
 * FIX (RX overrun): reception and printing are split into two phases so the
 * per-sample printf can never stall reception and overflow the RX FIFO.
 *   Phase 1: read the ENTIRE upload into RAM with NO printing.
 *   Phase 2: after the PIC finishes sending, decode and print at leisure.
 *
 * ADDED: Valve control outputs (non-blocking, millis()-based)
 *   D3 = VALVE_POWER   : 30 s ON / 10 s OFF  (period 40 s)
 *   D4 = VALVE_CONTROL :  5 s ON /  4 s OFF  (period  9 s)
 *   Both run continuously and independently while the loop is running.
 *   IMPORTANT: continuous valve timing only works while the device is AWAKE.
 *   If you re-enable HIBERNATE (System.sleep) the MCU stops counting and
 *   setup() re-runs on wake, so the valve cycle restarts each wake. Keep
 *   HIBERNATE disabled if the valves must cycle on a continuous schedule.
 *
 * Protocol:
 *   - UART 38400 8N1 on Serial1 (physical TX/RX pins).
 *   - Photon2 sends 0xAA; PIC replies with COUNT (4 bytes BE) + COUNT samples.
 *       Method 0 "2B-2B"  : 4 bytes/sample (index/pulses each 16-bit BE).
 *       Method 1 "10-10-4": 3 bytes/sample (24-bit BE: index[23:14], pulses[13:4]).
 */

#include "Particle.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

/* ====================================================================
 *  COMPILE-TIME METHOD SELECT — set this to match the PIC build.
 *    0 = 2B-2B   (4 bytes per sample)
 *    1 = 10-10-4 (3 bytes per sample)   <-- default
 * ==================================================================== */
#define DECODE_METHOD 1

#if   DECODE_METHOD == 0
  static const uint8_t BYTES_PER_SAMPLE = 4;
#elif DECODE_METHOD == 1
  static const uint8_t BYTES_PER_SAMPLE = 3;
#else
  #error "DECODE_METHOD must be 0 (2B-2B) or 1 (10-10-4)"
#endif

/* PIC drives this line HIGH to wake the Photon 2. */
const pin_t PIC_WAKE = D10;

/* ====================================================================
 *  VALVE OUTPUTS
 *    D3 = valve power, D4 = valve control.
 *    on  = output HIGH, off = output LOW (active-high). If your relay /
 *    solenoid driver is active-low, swap HIGH/LOW in valveService()/valveInit().
 * ==================================================================== */
const pin_t VALVE_POWER   = D3;   /* valve power   */
const pin_t VALVE_CONTROL = D4;   /* valve control */

static const uint32_t VPOWER_ON_MS  = 30000;  /* D3 ON time  = 30 s */
static const uint32_t VPOWER_OFF_MS = 10000;  /* D3 OFF time = 10 s */
static const uint32_t VCTRL_ON_MS   =  5000;  /* D4 ON time  =  5 s */
static const uint32_t VCTRL_OFF_MS  =  4000;  /* D4 OFF time =  4 s */

static const unsigned long PIC_BAUD         = 38400;  /* Must match PIC UART baud */
static const uint32_t      MAX_RECORDS      = 4096;   /* Sanity cap (PIC ring ~360) */
static const uint32_t      READ_TIMEOUT_MS  = 200;    /* Per-byte receive timeout */
static const uint32_t      WAKE_LOW_WAIT_MS = 500;    /* Max wait for PIC to drop WAKE */
static const uint32_t      MIN_CYCLE_MS     = 600000;   /* Minimum time per exchange (10 min) */

/* Whole-upload receive buffer (worst case MAX_RECORDS * 4 B = 16 KB).
 * Filled in phase 1, drained in phase 2, so printf can never stall reception. */
static uint8_t g_rxBuf[MAX_RECORDS * 4];

/* --------------------------------------------------------------------
 *  Non-blocking valve timer.
 *    state == true  -> output ON  (HIGH), use onMs  before toggling
 *    state == false -> output OFF (LOW),  use offMs before toggling
 *  Call valveService() as often as possible (every loop iteration and
 *  inside any long wait loops) so toggles happen near their deadlines.
 * ------------------------------------------------------------------ */
struct ValveTimer {
    pin_t    pin;
    uint32_t onMs;
    uint32_t offMs;
    bool     state;        /* current output state: true = ON  */
    uint32_t lastToggle;   /* millis() at the last state change */
};

static ValveTimer g_valvePower = { VALVE_POWER,   VPOWER_ON_MS, VPOWER_OFF_MS, false, 0 };
static ValveTimer g_valveCtrl  = { VALVE_CONTROL, VCTRL_ON_MS,  VCTRL_OFF_MS,  false, 0 };

/* Initialise a valve: configure pin, start in the ON state. */
void valveInit(ValveTimer *v) {
    pinMode(v->pin, OUTPUT);
    v->state      = true;
    digitalWrite(v->pin, HIGH);   /* start ON; flip to LOW for active-low drivers */
    v->lastToggle = millis();
}

/* Service a valve: toggle when its current interval has elapsed.
 * Uses unsigned subtraction so it is safe across the millis() rollover. */
void valveService(ValveTimer *v) {
    uint32_t now      = millis();
    uint32_t interval = v->state ? v->onMs : v->offMs;
    if (now - v->lastToggle >= interval) {
        v->state = !v->state;
        digitalWrite(v->pin, v->state ? HIGH : LOW);   /* swap for active-low */
        v->lastToggle = now;
    }
}

/* Service all valves in one call (convenience). */
void valvesService(void) {
    valveService(&g_valvePower);
    valveService(&g_valveCtrl);
}

/* Grow the Serial1 hardware RX buffer for extra headroom (default ~64 B).
 * If this fails to compile on your Device OS version, delete this whole
 * function — phases 1/2 alone already fix the overrun.
 * NOTE: designated initializers MUST follow the struct's declaration order:
 *       rx_buffer, rx_buffer_size, tx_buffer, tx_buffer_size. */
hal_usart_buffer_config_t acquireSerial1Buffer(void) {
    static uint8_t txBuf[256];
    static uint8_t rxBuf[4096];
    hal_usart_buffer_config_t cfg = {
        .size           = sizeof(hal_usart_buffer_config_t),
        .rx_buffer      = rxBuf,
        .rx_buffer_size = sizeof(rxBuf),
        .tx_buffer      = txBuf,
        .tx_buffer_size = sizeof(txBuf)
    };
    return cfg;
}

/* Print current pin / link status (debug helper).
 * digitalRead() returns int32_t on Photon 2 -> cast to int for %d. */
void printPicPinStates(void) {
    Serial.printf("PIC WAKE=%d  Serial1 avail=%d  VPWR=%d  VCTL=%d\r\n",
                  (int)digitalRead(PIC_WAKE), (int)Serial1.available(),
                  (int)g_valvePower.state, (int)g_valveCtrl.state);
}

/* Receive one byte from the PIC over Serial1, with a timeout.
 * Keep servicing the valves while waiting so their schedule does not stall. */
bool readByteFromPIC(uint8_t *out) {
    uint32_t start = millis();
    while (!Serial1.available()) {
        valvesService();
        if (millis() - start >= READ_TIMEOUT_MS) return false;
    }
    *out = (uint8_t)Serial1.read();
    return true;
}

/* Read exactly n bytes into buf (wire order preserved). false on timeout.
 * NOTE: n is uint32_t — a uint8_t counter would overflow past 255 bytes. */
bool readBytesFromPIC(uint8_t *buf, uint32_t n) {
    uint32_t i;
    for (i = 0; i < n; i++) {
        if (!readByteFromPIC(&buf[i])) return false;
    }
    return true;
}

/* Assemble the 4-byte big-endian COUNT. */
bool readCount(uint32_t *val) {
    uint8_t b[4];
    if (!readBytesFromPIC(b, 4)) return false;   /* b[0]=MSB ... b[3]=LSB */
    *val = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] << 8)  |  (uint32_t)b[3];
    return true;
}

/* Decode one sample into index/pulses, per DECODE_METHOD. */
void decodeSample(const uint8_t *b, uint16_t *index, uint16_t *pulses) {
#if DECODE_METHOD == 0
    *index  = ((uint16_t)b[0] << 8) | b[1];
    *pulses = ((uint16_t)b[2] << 8) | b[3];
#else
    uint32_t word = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    *index  = (uint16_t)((word >> 14) & 0x3FF);
    *pulses = (uint16_t)((word >> 4)  & 0x3FF);
#endif
}

/* Send 0xAA, then receive one upload block (COUNT + samples) and print it. */
void readAndPrintWindow(void) {
    uint32_t count;
    uint32_t totalBytes;
    uint32_t i;
    uint16_t lastIndex = 0, lastPulses = 0;

    /* Flush any stale bytes left in the RX buffer before requesting. */
    while (Serial1.available()) Serial1.read();

    Serial1.write(0xAA);   /* Request data */
    Serial1.flush();       /* Block until the request byte is physically sent */

    if (!readCount(&count)) {
        Serial.printf("ERROR: timeout reading count\r\n");
        return;
    }

    /* COUNT == 0 is normal: 0xAA arrived with nothing new buffered. */
    if (count == 0) {
        Serial.printf("--- empty upload (no new samples) ---\r\n");
        return;
    }
    if (count > MAX_RECORDS) {
        Serial.printf("ERROR: bad count=%lu (cap=%lu)\r\n",
                      (unsigned long)count, (unsigned long)MAX_RECORDS);
        return;
    }

    /* ----------------------------------------------------------------
     * PHASE 1: receive the ENTIRE upload with NO printing.
     * Tight read+store loop (microseconds/byte) outpaces the ~0.26 ms/byte
     * wire rate, so the RX FIFO never overflows. NOTHING blocking in here.
     * ---------------------------------------------------------------- */
    totalBytes = count * BYTES_PER_SAMPLE;
    if (!readBytesFromPIC(g_rxBuf, totalBytes)) {
        Serial.printf("ERROR: timeout during receive (expected %lu bytes)\r\n",
                      (unsigned long)totalBytes);
        return;
    }

    /* ----------------------------------------------------------------
     * PHASE 2: PIC has finished sending. Decode and print at leisure.
     * ---------------------------------------------------------------- */
    Serial.printf("--- upload: %lu samples (method %d, %u B/sample) ---\r\n",
                  (unsigned long)count, DECODE_METHOD, BYTES_PER_SAMPLE);

    for (i = 0; i < count; i++) {
        uint16_t index, pulses;
        decodeSample(&g_rxBuf[i * BYTES_PER_SAMPLE], &index, &pulses);
        Serial.printf("  [%lu] index=%u  pulses=%u\r\n",
                      (unsigned long)i, index, pulses);
        lastIndex  = index;
        lastPulses = pulses;
        valvesService();   /* keep valve schedule alive during long prints */
    }

    Serial.printf("Latest: index=%u  pulses=%u\r\n", lastIndex, lastPulses);
    Serial.printf("------------------------------------------\r\n");
}

void setup(void) {
    pinMode(PIC_WAKE, INPUT_PULLDOWN);
    pinMode(BTN, INPUT_PULLUP);

    /* Initialise valve outputs (both start ON). */
    valveInit(&g_valvePower);
    valveInit(&g_valveCtrl);

    Serial.begin();                        /* USB CDC: baud argument ignored */
    Serial1.begin(PIC_BAUD, SERIAL_8N1);   /* PIC link, 38400 8N1 */
    waitFor(Serial.isConnected, 300);      /* Short bounded wait */
}

void loop(void) {
    /* Drive the valve schedules every iteration (non-blocking). */
    // valvesService();

    bool wokeByPic    = (digitalRead(PIC_WAKE) == HIGH);
    bool wokeByButton = (digitalRead(BTN) == LOW);

    if (wokeByButton) {
        printPicPinStates();   /* optional debug snapshot */
    }

    if (wokeByPic || wokeByButton) {
        uint32_t cycleStart = millis();

        delay(10);
        // readAndPrintWindow();  /* sends 0xAA, then receives and prints */
        // Serial.flush();

        // /* Wait for the PIC to release WAKE before sleeping (bounded). */
        // {
        //     unsigned long t = millis();
        //     while (digitalRead(PIC_WAKE) && millis() - t < WAKE_LOW_WAIT_MS) {
        //         valvesService();
        //         delay(1);
        //     }
        // }

        /* Enforce a minimum 1-second cycle (millis() only, no RTC). */
        while (millis() - cycleStart < MIN_CYCLE_MS) {
            valvesService();
            delay(1);
        }
    }

    /* HIBERNATE: wake on PIC trigger (RISING) or button press (FALLING).
     * HIBERNATE re-runs setup() on wake, keeping USB-CDC debug alive.
     * WARNING: while hibernating, millis() stops and the valve timers do
     * NOT advance; setup() restarts both valves in the ON state on wake.
     * Keep this disabled if the valves must follow a continuous schedule. */
    SystemSleepConfiguration cfg;
    cfg.mode(SystemSleepMode::HIBERNATE)
       .gpio(PIC_WAKE, RISING)
       .gpio(BTN, FALLING);
    System.sleep(cfg);
}