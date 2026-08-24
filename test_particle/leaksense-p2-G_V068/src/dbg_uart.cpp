/*
 * dbg_uart.cpp  -  See dbg_uart.h for the overview.
 */

#include "dbg_uart.h"

// ============================================================ 1) UART log sink
#if DBG_UART_ENABLE

// Guard 1: the TX/RX pins are fixed in silicon per port and cannot be remapped.
// These macros only DOCUMENT the port's pins, so if someone edits one by hand
// the boot banner and the collision guards below would silently lie. Catch that
// here instead. To move the debug output, change DBG_UART_SELECT in app_config.h.
#if DBG_UART_SELECT == 2
static_assert((int)DBG_UART_TX_PIN == (int)D4 && (int)DBG_UART_RX_PIN == (int)D5,
              "Serial2 is fixed at TX=D4 / RX=D5 on the RTL872x and cannot be remapped");
#elif DBG_UART_SELECT == 3
static_assert((int)DBG_UART_TX_PIN == (int)D15 && (int)DBG_UART_RX_PIN == (int)D16,
              "Serial3 is fixed at TX=D15(MOSI) / RX=D16(MISO) and cannot be remapped");
#endif

// Guard 2: the debug UART owns BOTH of its pins once begin() runs, so nothing
// else in the pin map may claim them. If either assert fires, either move the
// offending signal or set DBG_UART_ENABLE to 0 in app_config.h.
static_assert((int)DBG_UART_TX_PIN != (int)SHUTOFF_SWITCH_PIN,
              "Debug UART TX collides with SHUTOFF_SWITCH_PIN");
static_assert((int)DBG_UART_TX_PIN != (int)SHUTOFF_SSR_PIN,
              "Debug UART TX collides with SHUTOFF_SSR_PIN");
static_assert((int)DBG_UART_TX_PIN != (int)LED1_PIN,
              "Debug UART TX collides with LED1_PIN");
static_assert((int)DBG_UART_TX_PIN != (int)PIC_WAKE_PIN,
              "Debug UART TX collides with PIC_WAKE_PIN");
static_assert((int)DBG_UART_RX_PIN != (int)SHUTOFF_SWITCH_PIN,
              "Debug UART RX collides with SHUTOFF_SWITCH_PIN");
static_assert((int)DBG_UART_RX_PIN != (int)SHUTOFF_SSR_PIN,
              "Debug UART RX collides with SHUTOFF_SSR_PIN");
static_assert((int)DBG_UART_RX_PIN != (int)LED1_PIN,
              "Debug UART RX collides with LED1_PIN");
static_assert((int)DBG_UART_RX_PIN != (int)PIC_WAKE_PIN,
              "Debug UART RX collides with PIC_WAKE_PIN");

static bool s_uartReady = false;   // true once the extra log handler is registered

#endif // DBG_UART_ENABLE

// Appendix H.3: this diagnostic logger deliberately reports the RAW clock state,
// including when it is unsynced, so it cannot use the application data-path gate
// (clockNowUtc() in leaksense.cpp, a different translation unit). It has its own
// single guarded read instead - the ONLY Time.now() in this file - which, like
// the app gate, returns 0 (never a fabricated 2000-01-01 epoch) when invalid.
// These values are logged only; they never enter buckets/prevReportEnd/flash.
static inline uint32_t dbgClockUtc() {
  return Time.isValid() ? (uint32_t)Time.now() : 0UL;
}

// ============================================================ 2) time ticker
static uint32_t s_lastTimeLogMs = 0;      // millis() of the last emitted TIME line
static uint32_t s_syncStartedMs = 0;      // millis() when the pending sync was requested
static uint32_t s_syncMark      = 0;      // Particle.timeSyncedLast() before that request
static bool     s_syncPending   = false;  // a syncTime() request is outstanding
static bool     s_firstLogDone  = false;  // the boot-time TIME line has been emitted
static uint32_t s_timeLogSeq    = 0;      // running counter, makes gaps obvious in a capture

// ---------------------------------------------------------------------------
// Emit one TIME line. 'why' explains where the value came from:
//   "boot"          first valid clock after power-up
//   "cloud"         a fresh Particle.syncTime() round trip completed
//   "rtc-offline"   no cloud link, so this is the local clock, not a cloud read
//   "rtc-timeout"   a sync was requested but the cloud did not answer in time
void dbgTimeLogNow(const char *why) {
  const bool     valid   = Time.isValid();
  const uint32_t epoch   = dbgClockUtc();          // H.3: single guarded read; 0 when unsynced
  const uint32_t syncAge = (uint32_t)(millis() - (uint32_t)Particle.timeSyncedLast());

  // Time.format() returns a String; keep it in a named local so the buffer it
  // points at is still alive while Log.info() formats the line.
  String stamp = valid ? Time.format(epoch, TIME_FORMAT_ISO8601_FULL) : String("unsynced");

  Log.info("TIME #%lu src=%s local=%s epoch=%lu valid=%d cloud=%d syncAgeMs=%lu",
           (unsigned long)(++s_timeLogSeq),
           why,
           stamp.c_str(),
           (unsigned long)epoch,
           (int)valid,
           (int)Particle.connected(),
           (unsigned long)syncAge);
}

// ============================================================ public API
void dbgUartBegin() {
#if DBG_UART_ENABLE
  // Bring up the port first: the handler writes into it as soon as it is added.
  DBG_UART_PORT.begin(DBG_UART_BAUD, SERIAL_8N1);

  // Function-local static: constructed on the first call (i.e. from setup()),
  // never destroyed, so the log manager's pointer stays valid for the whole run.
  // This also sidesteps any static-initialisation-order question about Serial2.
  static spark::StreamLogHandler dbgHandler(DBG_UART_PORT, DBG_UART_LOG_LEVEL);

  s_uartReady = spark::LogManager::instance()->addHandler(&dbgHandler);

  // First line out of the new port; also proves the wiring/baud on a scope.
  Log.info("DBG_UART: mirror active on TX=%s @ %lu baud 8N1 (handler=%s)",
           DBG_UART_TX_NAME, (unsigned long)DBG_UART_BAUD,
           s_uartReady ? "ok" : "FAILED");
#else
  Log.info("DBG_UART: mirror disabled on this platform (no spare hardware UART); USB log only");
#endif
}

void dbgUartService() {
  const uint32_t nowMs = millis();

  // ---- one immediate line as soon as the clock is first valid --------------
  // Device OS syncs time automatically on cloud connect, so this normally fires
  // a second or two after the cloud handshake, well before the first minute tick.
  if (!s_firstLogDone && Time.isValid()) {
    s_firstLogDone  = true;
    s_lastTimeLogMs = nowMs;
    dbgTimeLogNow("boot");
    return;
  }

  // ---- finish an outstanding sync -----------------------------------------
  if (s_syncPending) {
    // Completion is detected by the sync timestamp changing rather than by
    // syncTimeDone() alone: timeSyncedLast() only moves on an actual cloud
    // answer, so this cannot report success on a request that never landed.
    if ((uint32_t)Particle.timeSyncedLast() != s_syncMark && Particle.syncTimeDone()) {
      s_syncPending   = false;
      s_lastTimeLogMs = nowMs;
      dbgTimeLogNow("cloud");
    } else if ((uint32_t)(nowMs - s_syncStartedMs) >= DBG_TIME_SYNC_TIMEOUT_MS) {
      s_syncPending   = false;
      s_lastTimeLogMs = nowMs;
      dbgTimeLogNow("rtc-timeout");
    }
    return;
  }

  // ---- minute boundary -----------------------------------------------------
  if ((uint32_t)(nowMs - s_lastTimeLogMs) < DBG_TIME_LOG_INTERVAL_MS) return;

  if (Particle.connected()) {
    s_syncMark      = (uint32_t)Particle.timeSyncedLast();  // remember the "before" value
    s_syncStartedMs = nowMs;
    s_syncPending   = true;
    Particle.syncTime();                                    // non-blocking request
  } else {
    // No cloud link: still emit a line every minute, but label it honestly as
    // the local clock so a capture never implies a cloud read that never happened.
    s_firstLogDone  = true;
    s_lastTimeLogMs = nowMs;
    dbgTimeLogNow("rtc-offline");
  }
}

// ============================================================ 3) fn tracing
void dbgLogCloudFnCall(const char *name, const String &arg, int rc) {
  Log.info("CLOUDFN %s(\"%s\") -> %d  [epoch=%lu]",
           name,
           arg.c_str(),
           rc,
           (unsigned long)dbgClockUtc());          // H.3: single guarded read

  // Then dump the resulting settings so a changed value is visible, not just
  // the fact that a call happened.
  dbgLogSettingsSnapshot(name);
}
