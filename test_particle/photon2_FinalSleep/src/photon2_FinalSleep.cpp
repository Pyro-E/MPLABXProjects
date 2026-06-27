/*
 * Project: PIC meter reader (Photon 2) - UART version  [C-struct, .cpp source]
 *          STOP-mode sleep, USB-CDC kept alive (continuous USB comms with PC)
 *
 * Aligned to "PIC18F06Q40 <-> Photon2 Protocol Guide".
 *
 * Protocol summary (per guide):
 *   - UART 38400 8N1 on Serial1 (physical TX/RX pins).
 *   - WAKE: PIC drives D10 HIGH ("PIC ready / ACK"). 500 ms pulse, but the PIC
 *     keeps waiting for 0xAA up to 3000 ms after that rising edge.
 *   - Bytes Photon2 -> PIC:  0xF0 = wake/trigger,  0xAA = start upload.
 *   - CORE RULE (guide section 3): send 0xAA ONLY after WAKE has been seen HIGH.
 *   - Case A (Photon-initiated): 0xF0 -> wait WAKE HIGH -> 0xAA -> receive.
 *   - Case B (PIC batch-full):   WAKE HIGH -> 0xAA -> receive.
 *   - On 0xAA: PIC replies with COUNT (4 bytes big-endian) + COUNT samples.
 *
 * Sample format (guide section 6 -- "10-14", MSB first, 3 bytes/sample):
 *       v = (sample# << 14) | pulses
 *         sample# : upper 10 bits  (0..1023)
 *         pulses  : lower 14 bits  (0..16383, the flow value)
 *
 * Structure:
 *   Sample      - one decoded sample (index, pulses)
 *   Upload      - one upload transaction (count + raw byte buffer)
 *   WakeInfo    - classified wake reason after STOP sleep
 *   ProtoConfig - all pins / baud / timing constants in one place
 *
 * Fixes carried over:
 *   [FIX 1] 10-14 decode (pulses = word & 0x3FFF), not the old 10-10-4.
 *   [FIX 2] 0xF0 trigger + Case A.
 *   [FIX 3] 0xAA sent ONLY after WAKE HIGH (button path too).
 *   [FIX 4] UART exchange runs BEFORE waiting for USB, so 0xAA never misses
 *           the PIC's 3000 ms window due to USB re-enumeration.
 */

#include "Particle.h"

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

/* photon2_serial enable.cpp ====================================================================
 *  TEST TOGGLE - temporarily bypass STOP-mode sleep for bench testing.
 *    1 = sleep DISABLED: poll WAKE/BTN in a loop, USB stays connected  <-- test
 *    0 = normal STOP-mode sleep (production / low power)
 * ==================================================================== */
#define TEST_DISABLE_SLEEP 0

/* ====================================================================
 *  COMPILE-TIME METHOD SELECT - set this to match the PIC build.
 *    0 = 2B-2B   (4 bytes per sample: index/pulses each 16-bit BE)
 *    1 = 10-14   (3 bytes per sample, matches protocol guide)  <-- default
 * ==================================================================== */
#define DECODE_METHOD 1

#if   DECODE_METHOD == 0
  #define BYTES_PER_SAMPLE  4
#elif DECODE_METHOD == 1
  #define BYTES_PER_SAMPLE  3
#else
  #error "DECODE_METHOD must be 0 (2B-2B) or 1 (10-14)"
#endif

#define MAX_RECORDS  4096u   /* Sanity cap (PIC slots <= 1000) */

/* Protocol bytes (Photon2 -> PIC). */
enum {
    PROTO_F0 = 0xF0,   /* wake / trigger */
    PROTO_AA = 0xAA    /* start upload   */
};

/* Upload result codes (Upload.count < 0 means error). */
enum {
    UP_ERR_COUNT_TIMEOUT = -2,   /* timed out reading COUNT */
    UP_ERR_BAD_COUNT     = -3,   /* COUNT exceeded the cap */
    UP_ERR_BODY_TIMEOUT  = -4,   /* timed out reading sample body */
    UP_ERR_NO_WAKE       = -10   /* PIC never drove WAKE HIGH */
};

/* ---- Structures --------------------------------------------------- */

/* All pins / baud / timing parameters in one place. */
typedef struct {
    pin_t         wakePin;          /* PIC drives this HIGH to wake/ACK */
    unsigned long baud;             /* Serial1 baud (must match PIC) */
    uint8_t       bytesPerSample;
    uint32_t      readTimeoutMs;    /* per-byte receive timeout */
    uint32_t      wakeLowWaitMs;    /* max wait for WAKE to drop LOW */
    uint32_t      wakeHighWaitMs;   /* Case A: wait for WAKE HIGH after 0xF0 */
    uint32_t      minCycleMs;       /* minimum time per exchange */
    uint32_t      usbReconnectMs;   /* max wait for USB after wake */
    uint32_t      pollIntervalMs;   /* TEST mode: poll period for WAKE/BTN */
} ProtoConfig;

static const ProtoConfig PROTO = {
    .wakePin        = D10,
    .baud           = 38400,
    .bytesPerSample = BYTES_PER_SAMPLE,
    .readTimeoutMs  = 200,
    .wakeLowWaitMs  = 3000,
    .wakeHighWaitMs = 2000,
    .minCycleMs     = 1000,
    .usbReconnectMs = 5000,
    .pollIntervalMs = 2
};

/* One decoded sample. */
typedef struct {
    uint16_t index;    /* sequence number 0..1023 */
    uint16_t pulses;   /* flow value 0..16383 */
} Sample;

/* One upload transaction: count + raw wire bytes. */
typedef struct {
    long    count;                    /* >=0 sample count, <0 = UP_ERR_* */
    uint8_t buf[MAX_RECORDS * 4];     /* raw bytes as received */
} Upload;

/* Classified wake reason after STOP sleep (or after pollForWake in test mode). */
typedef struct {
    bool  byGpio;
    pin_t pin;
    bool  byPic;
    bool  byButton;
    int   err;
    int   reason;
} WakeInfo;

/* Single upload buffer (16 KB worst case) lives as one global instance. */
static Upload g_upload;

/* ---- Serial1 RX buffer growth ------------------------------------- */

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

/* ---- Low-level UART helpers --------------------------------------- */

void waitUsbReady(uint32_t timeout_ms) {
    uint32_t t = millis();
    while (!Serial.isConnected() && (millis() - t < timeout_ms)) delay(10);
    delay(300);   /* host port settling margin */
}

void printPicPinStates(void) {
    Serial.printf("PIC WAKE=%d  BTN=%d  Serial1 avail=%d\r\n",
                  (int)digitalRead(PROTO.wakePin), (int)digitalRead(BTN),
                  (int)Serial1.available());
}

bool readByteFromPIC(uint8_t *out) {
    uint32_t start = millis();
    while (!Serial1.available()) {
        if (millis() - start >= PROTO.readTimeoutMs) return false;
    }
    *out = (uint8_t)Serial1.read();
    return true;
}

bool readBytesFromPIC(uint8_t *buf, uint32_t n) {
    for (uint32_t i = 0; i < n; i++)
        if (!readByteFromPIC(&buf[i])) return false;
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

/* Wait for the PIC WAKE line to read HIGH, up to timeout_ms. */
bool waitWakeHigh(uint32_t timeout_ms) {
    uint32_t t = millis();
    while (millis() - t < timeout_ms) {
        if (digitalRead(PROTO.wakePin) == HIGH) return true;
        delay(2);
    }
    return digitalRead(PROTO.wakePin) == HIGH;
}

/* ---- Upload operations (operate on an Upload struct) -------------- */

/* Decode the i-th sample out of u->buf into *s, per DECODE_METHOD. */
void uploadDecode(const Upload *u, long i, Sample *s) {
    const uint8_t *b = &u->buf[i * PROTO.bytesPerSample];
#if DECODE_METHOD == 0
    s->index  = ((uint16_t)b[0] << 8) | b[1];
    s->pulses = ((uint16_t)b[2] << 8) | b[3];
#else
    /* [FIX 1] 10-14, MSB-first (guide section 6.3). */
    uint32_t word = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    s->index  = (uint16_t)((word >> 14) & 0x03FF);
    s->pulses = (uint16_t)( word        & 0x3FFF);
#endif
}

/* Send 0xAA and receive one upload block into *u.
 * CALLER MUST have confirmed WAKE was HIGH first (core rule, section 3).
 * Fills u->count (>=0 on success, <0 = UP_ERR_*). Does NOT print. */
void uploadRequest(Upload *u) {
    uint32_t count, totalBytes;

    while (Serial1.available()) Serial1.read();   /* flush stale */

    Serial1.write(PROTO_AA);
    Serial1.flush();                              /* block until sent */

    if (!readCount(&count))  { u->count = UP_ERR_COUNT_TIMEOUT; return; }
    if (count == 0)          { u->count = 0;                    return; }
    if (count > MAX_RECORDS) { u->count = UP_ERR_BAD_COUNT;     return; }

    totalBytes = count * PROTO.bytesPerSample;
    if (!readBytesFromPIC(u->buf, totalBytes)) { u->count = UP_ERR_BODY_TIMEOUT; return; }

    u->count = (long)count;
}

/* [FIX 2/3] Case A -- Photon-initiated:
 * 0xF0 -> wait for WAKE HIGH (PIC ready/ACK) -> 0xAA -> receive. */
void uploadInitiate(Upload *u) {
    while (Serial1.available()) Serial1.read();   /* flush stale */

    Serial1.write(PROTO_F0);   /* may be lost on PIC clock start; OK */
    Serial1.flush();

    if (!waitWakeHigh(PROTO.wakeHighWaitMs)) {     /* PIC never acked */
        u->count = UP_ERR_NO_WAKE;
        return;
    }
    uploadRequest(u);          /* WAKE HIGH confirmed -> request */
}

/* Decode + print a received upload (USB must be ready by now). */
void uploadPrint(const Upload *u) {
    if (u->count == 0) {
        Serial.printf("--- empty upload (no new samples) ---\r\n");
        return;
    }
    if (u->count < 0) {
        const char *why = "unknown";
        switch (u->count) {
            case UP_ERR_COUNT_TIMEOUT: why = "timeout reading COUNT";       break;
            case UP_ERR_BAD_COUNT:     why = "bad COUNT (exceeds cap)";     break;
            case UP_ERR_BODY_TIMEOUT:  why = "timeout reading sample body"; break;
            case UP_ERR_NO_WAKE:       why = "no WAKE HIGH after 0xF0";     break;
        }
        Serial.printf("ERROR: upload failed (code %ld: %s)\r\n", u->count, why);
        return;
    }

    Serial.printf("--- upload: %ld samples (method %d, %u B/sample) ---\r\n",
                  u->count, DECODE_METHOD, PROTO.bytesPerSample);

    Sample last = { 0, 0 };
    for (long i = 0; i < u->count; i++) {
        Sample s;
        uploadDecode(u, i, &s);
        Serial.printf("  [%ld] index=%u  pulses=%u\r\n", i, s.index, s.pulses);
        last = s;
    }
    Serial.printf("Latest: index=%u  pulses=%u\r\n", last.index, last.pulses);
    Serial.printf("------------------------------------------\r\n");
}

/* ---- Wake-reason classification ----------------------------------- */

/* Real STOP-sleep path: classify from a SystemSleepResult. */
void classifyWake(const SystemSleepResult *r, WakeInfo *w) {
    w->byGpio   = (r->wakeupReason() == SystemSleepWakeupReason::BY_GPIO);
    w->pin      = r->wakeupPin();
    w->byPic    = w->byGpio && (w->pin == PROTO.wakePin);
    w->byButton = w->byGpio && (w->pin == BTN);
    w->err      = (int)r->error();
    w->reason   = (int)r->wakeupReason();
}

/* TEST path (sleep disabled): block here polling the same two lines the
 * sleep config would have armed, then fill WakeInfo the same way
 * classifyWake() would have. WAKE rising => Case B; BTN falling => Case A.
 * Note: this never returns until one of those events is seen. */
void pollForWake(WakeInfo *w) {
    w->byGpio   = false;
    w->byPic    = false;
    w->byButton = false;
    w->pin      = (pin_t)PIN_INVALID;
    w->err      = 0;
    w->reason   = 0;   /* 0 stands in for "not a real sleep wake" */

    int prevWake = digitalRead(PROTO.wakePin);   /* expected LOW at entry */
    int prevBtn  = digitalRead(BTN);             /* HIGH when idle (pullup) */

    /* If WAKE is already HIGH on entry (e.g. wakeLowWaitMs expired while it
     * was still pulsing), treat it as a PIC batch-full event right away. */
    if (prevWake == HIGH) {
        w->byGpio = true;
        w->byPic  = true;
        w->pin    = PROTO.wakePin;
        return;
    }

    for (;;) {
        int curWake = digitalRead(PROTO.wakePin);
        int curBtn  = digitalRead(BTN);

        /* WAKE rising edge (LOW->HIGH) -> Case B (PIC batch-full). */
        if (prevWake == LOW && curWake == HIGH) {
            w->byGpio = true;
            w->byPic  = true;
            w->pin    = PROTO.wakePin;
            return;
        }
        /* BTN falling edge (HIGH->LOW) -> Case A (Photon-initiated). */
        if (prevBtn == HIGH && curBtn == LOW) {
            w->byGpio   = true;
            w->byButton = true;
            w->pin      = BTN;
            return;
        }

        prevWake = curWake;
        prevBtn  = curBtn;
        delay(PROTO.pollIntervalMs);
    }
}

/* ---- Arduino entry points ----------------------------------------- */

void setup(void) {
    WiFi.off();
    RGB.control(true);
    RGB.brightness(0);

    pinMode(D7, OUTPUT);
    for (int i = 0; i < 3; i++) { digitalWrite(D7, HIGH); delay(120); digitalWrite(D7, LOW); delay(120); }

    pinMode(PROTO.wakePin, INPUT_PULLDOWN);
    pinMode(BTN, INPUT_PULLUP);

    pinMode(D3, OUTPUT);
    digitalWrite(D3, LOW);
    pinMode(D4, OUTPUT);
    digitalWrite(D4, LOW);
    
    Serial.begin();
    Serial1.begin(PROTO.baud, SERIAL_8N1);
    waitFor(Serial.isConnected, 5000);

    Serial.printf(">>> BOOT  millis=%lu  D10=%d  BTN=%d  [sleep=%s]\r\n",
                  millis(), (int)digitalRead(PROTO.wakePin), (int)digitalRead(BTN),
                  TEST_DISABLE_SLEEP ? "DISABLED(test)" : "ENABLED");
}

void loop(void) {
    /* Let WAKE return LOW before re-arming sleep / re-polling. */
    {
        unsigned long t = millis();
        while (digitalRead(PROTO.wakePin) == HIGH && millis() - t < PROTO.wakeLowWaitMs) delay(5);
    }

    WakeInfo w;
    uint32_t cycleStart;

#if TEST_DISABLE_SLEEP
    /* -------- TEST MODE: no STOP sleep, busy-poll WAKE/BTN -------- */
    Serial.printf(">>> [TEST] sleep DISABLED - polling WAKE/BTN  millis=%lu  D10=%d  BTN=%d\r\n",
                  millis(), (int)digitalRead(PROTO.wakePin), (int)digitalRead(BTN));
    Serial.flush();

    pollForWake(&w);             /* blocks until WAKE rises or BTN falls */
    cycleStart = millis();
#else
    /* -------- PRODUCTION: real STOP-mode sleep ------------------- */
    Serial.printf(">>> Goto STOP sleep  millis=%lu  D10=%d  BTN=%d\r\n",
                  millis(), (int)digitalRead(PROTO.wakePin), (int)digitalRead(BTN));
    Serial.flush();

    SystemSleepConfiguration cfg;
    cfg.mode(SystemSleepMode::STOP)
       .gpio(PROTO.wakePin, RISING)   /* Case B: WAKE rising edge = ACK */
       .gpio(BTN, FALLING);           /* Case A trigger: external event */
    SystemSleepResult r = System.sleep(cfg);

    cycleStart = millis();
    classifyWake(&r, &w);
#endif

    /* [FIX 4] Time-critical UART exchange BEFORE waiting for USB.
     * (In test mode USB never drops, so the wait below is a no-op.) */
    bool didExchange = false;
    if (w.byPic) {
        delay(2);                 /* WAKE already HIGH (rising-edge / poll) */
        uploadRequest(&g_upload); /* Case B */
        didExchange = true;
    } else if (w.byButton) {
        uploadInitiate(&g_upload);/* Case A: 0xF0 -> WAKE HIGH -> 0xAA */
        didExchange = true;
    }

    /* Now safe to (re)wait for USB and print. */
    waitUsbReady(PROTO.usbReconnectMs);

    Serial.printf(">>> WAKE  millis=%lu  err=%d  reason=%d  pin=%d  D10=%d  BTN=%d\r\n",
                  millis(), w.err, w.reason, (int)w.pin,
                  (int)digitalRead(PROTO.wakePin), (int)digitalRead(BTN));

    if (w.byButton) printPicPinStates();

    if (didExchange) {
        uploadPrint(&g_upload);
        Serial.flush();
    }

    while (millis() - cycleStart < PROTO.minCycleMs) delay(1);
    delay(1000);
}
