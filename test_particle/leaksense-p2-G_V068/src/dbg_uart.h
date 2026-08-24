/*
 * dbg_uart.h  -  Debug log mirror on a D0..D7 UART TX pin, 1-minute cloud-time
 *                logger, and cloud-function call tracing for LeakSense P2.
 *
 * WHAT THIS MODULE ADDS (all three requested items):
 *
 *   1) A second log sink on a hardware UART, ADDED alongside the existing USB
 *      one. The SerialLogHandler in leaksense.cpp is never removed or replaced:
 *      LogManager broadcasts each message to every registered handler, so every
 *      Log.info() / Log.warn() / Log.error() line comes out BOTH the USB serial
 *      port and the debug UART, byte for byte, at the same time.
 *
 *   2) A ticker that, once per minute for as long as the board is powered,
 *      asks the Particle cloud for the current time (Particle.syncTime()) and
 *      writes one "TIME ..." log line with the result.
 *
 *   3) A thin wrapper around every registered Particle.function() so that any
 *      value changed from the cloud is echoed to the log: the function name,
 *      the raw argument string, the return code, and a snapshot of the settings
 *      that the call may have modified.
 *
 * PORT CHOICE (Photon 2 / P2). TX pins are fixed in silicon and cannot be
 * remapped, so the port choice IS the pin choice:
 *      Serial1 : TX = D8  , RX = D9    -> already owned by the PIC link
 *      Serial2 : TX = D4  , RX = D5    -> the only UART TX inside D0..D7
 *      Serial3 : TX = D15 , RX = D16   -> header pins marked MOSI / MISO
 *   Select with DBG_UART_SELECT (2 or 3) in app_config.h -- never by editing a
 *   pin macro. dbg_uart.cpp static_asserts that the two stay consistent.
 */

#pragma once

#include "Particle.h"
#include "app_config.h"

// ---------------------------------------------------------------- lifecycle
// Start the debug UART and register it as an additional log handler.
// Call ONCE, as early as possible in setup(), so boot messages are mirrored too.
void dbgUartBegin();

// Drive the 1-minute cloud-time logger. Call on EVERY loop() pass, in every
// state, so the tick keeps running after the session has ended and the board is
// simply waiting for the PIC to remove power. Non-blocking.
void dbgUartService();

// Emit one time line immediately (used at boot and available for manual probes).
void dbgTimeLogNow(const char *why);

// Log one cloud-function invocation. Called by the wrappers defined below.
void dbgLogCloudFnCall(const char *name, const String &arg, int rc);

// Log the current value of every cloud-settable setting. DEFINED IN
// leaksense.cpp because that translation unit owns the configuration globals.
void dbgLogSettingsSnapshot(const char *tag);

// ---------------------------------------------------------------- fn tracing
// DEFINE_LOGGED_CLOUD_FN(setConfig) generates:
//
//     static int setConfig_logged(String arg) {
//       int rc = setConfig(arg);
//       dbgLogCloudFnCall("setConfig", arg, rc);
//       return rc;
//     }
//
// The original handler is not touched; the wrapper only observes it. Because
// the handler takes its argument BY VALUE, any trim()/toLowerCase() it performs
// happens on its own copy, so the wrapper still logs exactly what the cloud
// sent. Every cloud name in this project already matches its C++ function name,
// so #FN is both the registered key and the log tag.
#define DEFINE_LOGGED_CLOUD_FN(FN)                     \
  static int FN##_logged(String arg) {                 \
    int rc = FN(arg);                                  \
    dbgLogCloudFnCall(#FN, arg, rc);                   \
    return rc;                                         \
  }

// Same wrapper, but the argument is NOT printed. Use it for any function whose
// argument carries a secret -- setWiFi takes "ssid,password", and that password
// must not end up on a debug wire or in a serial capture.
#define DEFINE_LOGGED_CLOUD_FN_REDACTED(FN)            \
  static int FN##_logged(String arg) {                 \
    int rc = FN(arg);                                  \
    dbgLogCloudFnCall(#FN, String("<redacted>"), rc);  \
    return rc;                                         \
  }

// REGISTER_LOGGED_CLOUD_FN(setConfig) -> Particle.function("setConfig", setConfig_logged)
#define REGISTER_LOGGED_CLOUD_FN(FN)  Particle.function(#FN, FN##_logged)
