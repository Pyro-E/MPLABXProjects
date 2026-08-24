/*
 * Particle.h  -  HOST-ONLY STUB. NOT part of the firmware build.
 *
 * Purpose: let `g++ -fsyntax-only` parse leaksense.cpp / pic_link.cpp /
 * dbg_uart.cpp on a normal Linux box, so syntax, types, missing declarations,
 * printf format mismatches and the static_asserts in dbg_uart.cpp are all
 * checked without the Particle toolchain. It intentionally implements NOTHING:
 * every symbol here exists only so the compiler can type-check the real code.
 *
 * The Particle build never sees this file (tools/ is outside src/).
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>          // V068: the real JsonParserGeneratorRK uses va_start/va_end
#include <string>
#include <deque>
#include <vector>

// ------------------------------------------------------------------ platform
#define PLATFORM_P2      32
#define PLATFORM_ARGON   12
#define PLATFORM_BORON   13
#ifndef PLATFORM_ID
#define PLATFORM_ID      PLATFORM_P2
#endif

// ------------------------------------------------------------------ pins
typedef uint16_t pin_t;
enum {
  D0 = 0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15, D16,
  A0 = 100, A1, A2, A3, A4, A5, A6,
  PIN_INVALID = 0xFFFF
};

enum { LOW = 0, HIGH = 1 };
enum { INPUT = 0, OUTPUT = 1, INPUT_PULLUP = 2 };
enum { CHANGE = 1, RISING = 2, FALLING = 3 };

inline void     pinMode(pin_t, int) {}
inline void     digitalWrite(pin_t, int) {}
inline int      digitalRead(pin_t) { return 0; }
inline int      analogRead(pin_t) { return 0; }
inline void     attachInterrupt(pin_t, void (*)(), int) {}
// Host test clock. Monotonic and advanced by one tick per call, so the
// firmware's "while (!available()) { if (millis()-start >= timeout) ... }"
// loops terminate deterministically instead of spinning forever. Tests read
// the current value with hostClock() without disturbing it.
inline uint32_t &hostClock() { static uint32_t t = 0; return t; }
inline uint32_t millis() { return hostClock()++; }
inline void     delay(uint32_t) {}
inline long     random(long lo, long hi) { return lo + (hi - lo) / 2; }
inline void     randomSeed(unsigned) {}

// ------------------------------------------------------------------ String
class String {
public:
  String() {}
  String(const char *s) : v(s ? s : "") {}
  String(const std::string &s) : v(s) {}
  const char *c_str() const { return v.c_str(); }
  size_t      length() const { return v.size(); }
  void        trim() {}
  void        toLowerCase() {}
  int         indexOf(char) const { return -1; }
  int         indexOf(char, int) const { return -1; }
  String      substring(int) const { return String(); }
  String      substring(int, int) const { return String(); }
  bool        operator==(const char *s) const { return v == (s ? s : ""); }
  // V068: used by the vendored JsonParserGeneratorRK, which tools/hostcheck/
  // payload_test.cpp compiles for real (the stub JsonWriter cannot answer a
  // question about byte counts).
  void        reserve(size_t n) { v.reserve(n); }
  void        concat(char c) { v += c; }
  void        concat(const char *s) { if (s) v += s; }
private:
  std::string v;
};

// ------------------------------------------------------------------ streams
#define SERIAL_8N1 0

class Print {
public:
  size_t write(const uint8_t *, size_t) { return 0; }
  size_t write(uint8_t) { return 0; }
};

// A loopback UART. Byte-level link tests drive the firmware through this:
//   pending  - the peer's reply, held back until we finish transmitting, so a
//              flushRx() before the write cannot swallow it (which is exactly
//              what the real wire does: the PIC answers after it has listened)
//   rx       - bytes on the wire and readable now
//   tx       - everything the firmware has transmitted, for byte comparison
//   trickleMs - if >0, release at most one byte per that many ticks, to model a
//              peer that stalls mid-frame
class Stream : public Print {
public:
  std::deque<uint8_t>  pending;
  std::deque<uint8_t>  rx;
  std::vector<uint8_t> tx;
  uint32_t trickleMs = 0;
  long     freeBytes = 0;    // first N bytes always arrive at line rate
  long     nRead     = 0;
  uint32_t lastRel   = 0;
  // How many bytes each flush() may release from `pending`. Empty = release
  // everything at once (the old behaviour). One entry per transmitted frame
  // lets a test stage a MULTI-transaction exchange: without it the whole reply
  // queue lands in rx on the first flush and the next sendFrame's flushRx()
  // throws the later replies away. A 0 entry models a peer that stays silent
  // for that exchange, which is how a dropped request looks from this side.
  std::deque<size_t> releaseChunks;

  void begin(unsigned long) {}
  void begin(unsigned long, uint32_t) {}
  int  available() {
    if (trickleMs == 0 || nRead < freeBytes) return (int)rx.size();
    if (rx.empty()) return 0;
    if ((uint32_t)(hostClock() - lastRel) < trickleMs) return 0;
    lastRel = hostClock();
    return 1;
  }
  int  read() {
    if (rx.empty()) return -1;
    int v = rx.front(); rx.pop_front(); nRead++; return v;
  }
  void flush() {                       // transmit complete -> the peer may answer
    size_t n = pending.size();
    if (!releaseChunks.empty()) {
      n = releaseChunks.front();
      releaseChunks.pop_front();
      if (n > pending.size()) n = pending.size();
    }
    for (size_t i = 0; i < n; i++) { rx.push_back(pending.front()); pending.pop_front(); }
  }
  bool isConnected() { return false; }
  size_t write(const uint8_t *p, size_t n) { for (size_t i = 0; i < n; i++) tx.push_back(p[i]); return n; }
  size_t write(uint8_t b) { tx.push_back(b); return 1; }
  void reset() { pending.clear(); rx.clear(); tx.clear(); releaseChunks.clear();
                 trickleMs = 0; freeBytes = 0; nRead = 0; lastRel = 0; }
};

extern Stream Serial;    // USB CDC
extern Stream Serial1;   // PIC link
extern Stream Serial2;
extern Stream Serial3;   // debug mirror

// Device OS hook that lets the app supply bigger UART buffers.
typedef struct {
  uint16_t size;
  uint8_t *rx_buffer;
  uint32_t rx_buffer_size;
  uint8_t *tx_buffer;
  uint32_t tx_buffer_size;
} hal_usart_buffer_config_t;

// ------------------------------------------------------------------ logging
enum LogLevel { LOG_LEVEL_ALL = 1, LOG_LEVEL_INFO = 30, LOG_LEVEL_WARN = 40, LOG_LEVEL_ERROR = 50 };

class Logger {
public:
  __attribute__((format(printf, 2, 3))) void info(const char *, ...) {}
  __attribute__((format(printf, 2, 3))) void warn(const char *, ...) {}
  __attribute__((format(printf, 2, 3))) void error(const char *, ...) {}
  __attribute__((format(printf, 2, 3))) void trace(const char *, ...) {}
};
extern Logger Log;

namespace spark {
class LogHandler {};
class StreamLogHandler : public LogHandler {
public:
  StreamLogHandler(Stream &, LogLevel) {}
};
class LogManager {
public:
  static LogManager *instance() { static LogManager m; return &m; }
  bool addHandler(LogHandler *) { return true; }
};
}  // namespace spark

class SerialLogHandler {
public:
  explicit SerialLogHandler(LogLevel) {}
};

// ------------------------------------------------------------------ time
#define TIME_FORMAT_ISO8601_FULL "iso8601"

// Controllable on the host so Appendix H.3.3 / H.15 tests can force the clock
// valid or invalid. Defaults to the historical stub behaviour (unsynced: now()=0,
// isValid()=false), so existing tests see no change. Set s_valid / s_now to model
// a synced clock or the Particle 2000-01-01 default (946684800).
class TimeClass {
public:
  static bool     s_valid;
  static uint32_t s_now;
  void     zone(int) {}
  uint32_t now() { return s_now; }
  bool     isValid() { return s_valid; }
  int      hour() { return 0; }
  int      hour(uint32_t) { return 0; }
  int      day() { return 1; }
  int      day(uint32_t) { return 1; }
  void     setTime(uint32_t t) { s_now = t; s_valid = true; }
  String   format(uint32_t, const char *) { return String(); }
};
extern TimeClass Time;

// ------------------------------------------------------------------ cloud
class IPAddress {
public:
  uint8_t operator[](int) const { return 0; }
};

class ParticleClass {
public:
  bool publish(const char *, const char *) { return true; }
  bool connected() { return false; }
  void connect() {}
  void process() {}
  void syncTime() {}
  bool syncTimeDone() { return true; }
  uint32_t timeSyncedLast() { return 0; }
  bool function(const char *, int (*)(String)) { return true; }
};
extern ParticleClass Particle;

class NetworkSignal {
public:
  int getStrength() const { return 0; }
};

class NetworkClass {
public:
  static bool   ready() { return false; }   // static in Device OS -> waitFor(WiFi.ready, ...) compiles
  void          connect() {}
  void          connect(unsigned) {}          // WiFi.connect(WIFI_CONNECT_SKIP_LISTEN)
  void          on() {}
  bool          hasCredentials() { return false; }
  void          disconnect() {}
  NetworkSignal RSSI() { return NetworkSignal(); }
  IPAddress     localIP() { return IPAddress(); }
  bool          setCredentials(String, String) { return true; }
  void          clearCredentials() {}
};
// Device OS connect() flags (system_network.h). Stubbed so the host check can
// parse the credential-aware boot path in setup().
#ifndef WIFI_CONNECT_SKIP_LISTEN
  #define WIFI_CONNECT_SKIP_LISTEN 1u
#endif
extern NetworkClass WiFi;
extern NetworkClass Cellular;

class FuelGauge {
public:
  float getSoC() { return 0.0f; }
};

class SystemClass {
public:
  uint32_t freeMemory() { return 0; }
  uint32_t uptime() { return 0; }
  void     backupRamSync() {}
  int      enableFeature(int) { return 0; }
};
extern SystemClass System;
#define FEATURE_RETAINED_MEMORY 1

class EEPROMClass {
public:
  template <typename T> T &get(int, T &t) { return t; }
  template <typename T> const T &put(int, const T &t) { return t; }
};
extern EEPROMClass EEPROM;

class Timer {
public:
  Timer(unsigned, void (*)(), bool) {}
  void start() {}
  void stop() {}
};

template <typename F>
inline bool waitFor(F, uint32_t) { return false; }

// ------------------------------------------------------------------ macros
#define retained
#define STARTUP(x)
#define PRODUCT_VERSION(x)
#define SYSTEM_MODE(x)
#define MANUAL 0
