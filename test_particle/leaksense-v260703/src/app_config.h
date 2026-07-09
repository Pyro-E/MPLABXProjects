/*
 * app_config.h  -  Shared configuration for LeakSense P2
 *
 * Targets Particle Photon 2 (P2). Boron guards are preserved so Kevin can
 * build the cellular variant unchanged (TODO #3).
 *
 * BEGINNER NOTE:
 *   A ".h" file is a "header" file. It holds shared settings, constants, and
 *   declarations that several ".cpp" source files can all "#include" and reuse.
 *   Think of it as a list of definitions everyone agrees on. Almost nothing in
 *   this file runs by itself; it just gives names to numbers and on/off switches.
 */

#pragma once             // Tell the compiler: include this file only ONCE per build,
                         // even if many files #include it. Prevents "duplicate" errors.

#include "Particle.h"    // Pull in the Particle device-OS library. This is what gives
                         // us pinMode(), digitalWrite(), Serial, Time, etc.

// ------------------------------------------------------------------ Platform
// "#if / #elif / #else / #endif" is the C++ PREPROCESSOR choosing code BEFORE
// compiling. Depending on which board we build for, a different block is kept
// and the others are thrown away. PLATFORM_ID is set automatically by the build.
#if PLATFORM_ID == PLATFORM_P2          // If we are building for the Photon 2 board...
  #define PLATFORM_STR    "P2"          //   ...give this build a human-readable name "P2".
  #define USE_WIFI        1             //   This board talks over Wi-Fi (1 = yes).
  #define USE_CELLULAR    0             //   It does NOT use a cellular modem (0 = no).
  #define HAS_FUEL_GAUGE  0             // P2 has no fuel gauge -> read ADC divider
                                        //   (so we will measure battery a different way later).
  #define PIC_WAKE_PIN        D10    // UNUSED / RESERVED under power-gating (PIC V048).
                                   //   The old WAKE handshake is gone: the PIC now switches the
                                   //   Photon's SUPPLY via a P-MOS on RC4, so D10 carries no signal.
                                   //   Kept defined (and held HIGH via INPUT_PULLUP) so the pin never
                                   //   floats and remaining references compile. Do not gate comms on it.

#elif PLATFORM_ID == PLATFORM_ARGON     // Otherwise, if building for the Argon board...
  #define PLATFORM_STR    "Argon"       //   ...name it "Argon".
  #define USE_WIFI        1             //   Argon also uses Wi-Fi.
  #define USE_CELLULAR    0             //   No cellular.
  #define HAS_FUEL_GAUGE  1             // Gen3 on-board fuel gauge (a chip that reports battery %).
#elif PLATFORM_ID == PLATFORM_BORON     // Otherwise, if building for the Boron board...
  #define PLATFORM_STR    "Boron"       //   ...name it "Boron".
  #define USE_WIFI        0             //   Boron has no Wi-Fi.
  #define USE_CELLULAR    1             //   Boron talks over the cellular network.
  #define HAS_FUEL_GAUGE  1             //   Boron also has the on-board fuel gauge.
 #define PIC_WAKE_PIN     D8            // UNUSED / RESERVED under power-gating (PIC V048).

#else                                   // If it is none of the boards we support...
  #error "Unsupported platform. Supported: P2, Argon, Boron."  // ...stop the build with this message.
#endif                                  // End of the platform selection.

// IMU (LSM6DS) is OPTIONAL telemetry (temp / accel / gyro). It is OFF by default
// because the Adafruit LSM6DS library is NOT in the Particle registry and will
// not cloud-resolve. To use it, vendor the libraries into lib/ (see README) and
// set this to 1. The core leak / PIC / cloud-config paths do not need it.
#define USE_IMU 0        // 0 = the motion/temperature sensor (IMU) is turned OFF.
                         //     Set to 1 only after adding its library manually.

// ------------------------------------------------------------------ Pin map
// Merged from LeakSense (doc 1) + PIC reader (doc 2). No conflicts.
// "#define NAME value" creates a text nickname. Everywhere we write MODE_PIN the
// compiler substitutes A1. Using names instead of raw pin numbers avoids mistakes.
#define MODE_PIN            A1     // The push button (also called "mode" button) is wired to pin A1.
#define LED1_PIN            D7     // A status LED is on pin D7 (D7 is the small on-board LED).
#define SHUTOFF_SWITCH_PIN  D4     // Controls valve direction (open vs close) on pin D4.
#define SHUTOFF_SSR_PIN     D3     // Controls the solid-state relay that powers the valve, on pin D3.
#define METER_PIN           A2     // local hall sensor (only if USE_LOCAL_METER)
                                   //   A "hall sensor" counts water-flow pulses; only used if enabled.

// The battery-measuring pin differs by board, so we pick it per platform.
#if PLATFORM_ID == PLATFORM_P2
  #define BATTERY_PIN A6           // On P2 we read the battery voltage from analog pin A6.
#elif PLATFORM_ID == PLATFORM_ARGON
  #define BATTERY_PIN A6           // unused (fuel gauge), defined to satisfy pinMode
                                   //   Argon uses its fuel-gauge chip instead, but we still
                                   //   define a pin so pinMode() has something valid to set.
#elif PLATFORM_ID == PLATFORM_BORON
  #define BATTERY_PIN A5           // On Boron the battery pin is A5.
#endif

// ------------------------------------------------------------------ Flow source
// 0 = flow data comes from the PIC over UART (Kevin's TODO #1)  <-- default
// 1 = also keep the local hall-effect interrupt path as a fallback
#define USE_LOCAL_METER 0          // 0 = trust the external PIC chip for water-flow data.
                                   //     1 = ALSO read a local sensor on this board.

// ------------------------------------------------------------------ Bench test
// OPTIONAL fast bench-test mode (see Photon_Addendum_FastBenchTest.md).
// When defined, the Photon SKIPS all cloud/publish work and instead just does the
// PIC UART exchange (REQ_DATA -> RSP_DATA, valve status) and logs it over USB via
// the existing Log.info() lines, then ends the session with PKT_PHOTON_OFF_REQ
// (func 0x07, OFF_REASON_DONE) exactly like production -- so the PIC still cuts
// power and the cycle repeats, minus the ~20-40 s cloud connect per session.
//
// SHIP BUILD = the normal cloud path, so this macro is left COMMENTED OUT (the default).
// Uncomment it ONLY to rebuild the no-cloud bench firmware (PIC UART + USB log, no cloud
// connect). NOTE: the D10-dependency removal from the main porting guide is STILL required
// either way -- neither build re-enables any wakeIsHigh()/D10 gating.

// #define FAST_BENCH_TEST 1   // uncomment ONLY for no-cloud bench bring-up (PIC UART + USB
                            //  log only); leave commented for the cloud build we ship.

// Bench only: after the USB CDC is enumerated (Serial.isConnected), wait this long so a
// serial-monitor app has time to (re)open the COM port before we start logging. Needed
// because power-gating re-enumerates USB every session. Not used in the cloud build.
constexpr uint32_t POST_USB_OPEN_MS = 5000;   // 5 s

// ------------------------------------------------------------------ Timing
// TODO #4: cloud connection only every 24 h.
// "constexpr" means: a constant value the compiler can compute at build time.
// "uint32_t" is an unsigned 32-bit integer (whole numbers 0 .. ~4.29 billion).
// The "UL" suffix means "unsigned long" so the multiplication stays a big integer.
constexpr uint32_t SLEEP_DURATION_S = 24UL * 60UL * 60UL;  // 24 h publish cadence
                                   //   24 hours * 60 min * 60 sec = 86400 seconds between cloud uploads.
constexpr uint32_t WAKE_WINDOW_MS   = 60UL * 1000UL;       // stay awake 1 min
                                   //   After waking, stay awake 60,000 milliseconds (= 60 s = 1 minute).

// P2 / Photon 2 hard limit: a single System.sleep() duration may be at most
// 546 min (~9.1 h). A longer .duration() is rejected with SYSTEM_ERROR_NOT_SUPPORTED
// (-120) and the device wakes immediately. So the 24 h period is slept in chunks
// no longer than this, and the cloud is contacted only when the 24 h publish
// boundary (nextPublishEpoch) is actually reached.
constexpr uint32_t SLEEP_CHUNK_MAX_S = 8UL * 60UL * 60UL;  // 8 h (< 9.1 h limit)
                                   //   We sleep in pieces of at most 8 hours so the hardware accepts it.

// "Flow in progress" latch (drives triggerState / runSleep's no-sleep gate).
// A PIC sample at/above FLOW_ACTIVE_GPM marks water as moving; the latch is
// auto-released once FLOW_IDLE_TIMEOUT_S elapses with no further flow, so it
// can never pin the device awake forever (incl. on stale retained RAM).
// "float" is a number with a decimal point. The "f" suffix marks it as a float.
constexpr float    FLOW_ACTIVE_GPM     = 0.10f;            // > this = flowing
                                   //   If water flow exceeds 0.10 gallons-per-minute, we say "water is moving".
constexpr uint32_t FLOW_IDLE_TIMEOUT_S = 5UL * 60UL;       // 5 min idle -> clear
                                   //   After 5 minutes (300 s) of no flow, drop the "flowing" flag.

// One logger sample per minute. PIC sample interval is assumed equal to this
// so each decoded PIC sample maps 1:1 to one logger slot. If the PIC firmware
// uses a different cadence, change PIC_SAMPLE_INTERVAL_SEC to match it.
constexpr uint32_t METER_INTERVAL_SEC      = 60;   // We store one flow reading every 60 seconds.
constexpr uint32_t PIC_SAMPLE_INTERVAL_SEC = 60;   // We assume the PIC also samples every 60 seconds.
constexpr uint32_t LEAK_MODEL_INTERVAL_SEC = 300;  // 5 min leak slots
                                                   //   The leak-learning model uses 5-minute (300 s) buckets.

// "uint16_t" = unsigned 16-bit integer (0 .. 65535).
constexpr uint16_t SAMPLES_PER_DAY = 24 * 60 * 60 / METER_INTERVAL_SEC; // 1440
                                   //   86400 seconds in a day / 60 s per sample = 1440 samples per day.
constexpr uint16_t MAX_SAMPLES     = SAMPLES_PER_DAY;                   // 1440
                                   //   The biggest buffer we keep is one full day = 1440 readings.

// ------------------------------------------------------------------ Flow calibration
// Identical to doc 1. Hz -> GPM polynomial + trickle correction.
// These constants tune the math that turns a sensor frequency (Hz) into a real
// water-flow value (gallons per minute). They come from physical testing.
constexpr float FLOW_C0 = 0.023f;          // Coefficient 0 of the calibration formula.
constexpr float FLOW_C1 = 0.35f;           // Coefficient 1.
constexpr float FLOW_C2 = -0.46f;          // Coefficient 2 (negative).
constexpr float FLOW_C3 = -0.034f;         // Coefficient 3 (negative).
constexpr float FLOW_C4 = 1.11f * 1.24f;   // trickle correction divisor
                                           //   1.11 * 1.24 is pre-multiplied; used to fix slow "trickle" flow.
constexpr float FLOW_C5 = -0.0043f;        // Coefficient 5.
constexpr float FLOW_C6 = 0.065f;          // Coefficient 6.

// Limits and default for the user-adjustable calibration scale.
constexpr float    FLOW_CAL_MIN   = 0.500f;  // The smallest calibration scale we will accept.
constexpr float    FLOW_CAL_MAX   = 2.000f;  // The largest calibration scale we will accept.
constexpr float    FLOW_CAL_DFLT  = 1.255f;  // The default scale used if none is saved yet.

// ------------------------------------------------------------------ Leak model
// This block tunes the statistical "is this a leak?" detector.
constexpr float ALPHA      = 0.02f;   // EWMA mean rate
                                      //   How fast the running AVERAGE adapts (small = slow, smooth).
constexpr float BETA       = 0.02f;   // EWMA variance rate
                                      //   How fast the running SPREAD (variance) adapts.
constexpr float EPS        = 0.02f;   // div0 guard / min sigma
                                      //   A tiny floor value so we never divide by zero.
constexpr float ABS_GPM_MIN = 0.5f;   // absolute floor for "large leak"
                                      //   Any flow above 0.5 GPM is always treated as suspicious.
constexpr float SIGMA_MULT  = 4.0f;   // mu + 4*sigma
                                      //   Alarm threshold = average + 4 * standard-deviation.
constexpr int   N_CONSEC    = 3;      // 3 leak-model intervals = 15 min
                                      //   Need 3 high readings in a row (3 * 5 min = 15 min) to call it a leak.
constexpr int   VOL_WIN     = 6;      // 6 intervals = 30 min volume window
                                      //   Track the last 6 buckets (6 * 5 min = 30 min) for total volume.

// ============================================================================
//  Host-side leak/valve analytics config (host only; NOT sent to the PIC).
//  In V040 the PIC runs its own independent leak detection + valve; these
//  values drive the P2's own cloud analytics, dashboard echo, and the local
//  SSR valve (D3/D4). They are no longer transmitted to the PIC -- the old
//  0xC0 downlink is gone (see app_config / pic_link headers). Kept so the
//  existing dashboard `setConfig` control and host leak model keep working.
// ============================================================================
// A "struct" groups several related variables into one named bundle.
struct AppConfig {
  float   leakThreshGpm;   // (1) instantaneous flow that raises a host leak alert
                           //     If flow jumps above this many GPM, warn about a leak.
  float   shutoffVolGal;   // (2) 30-min volume that forces a host shutoff
                           //     If total gallons over 30 min exceeds this, force the valve shut.
  uint8_t autoShutoff;     // (3) 0/1  auto-close the LOCAL valve on a host leak
                           //     0 = just warn; 1 = also close the valve automatically.
  uint8_t alertMode;       // (4) 0=off, 1=publish, 2=publish + extra alert
                           //     How loudly to report a leak to the cloud.
};                         // (Note: the ";" after a struct's closing brace is REQUIRED in C++.)

// Defaults (used on first boot / corrupt EEPROM)
// These are the starting values when the device has never been configured.
constexpr float   CFG_LEAK_GPM_DFLT  = 5.0f;   // Default leak threshold: 5 GPM.
constexpr float   CFG_SHUTOFF_DFLT   = 30.0f;  // Default 30-min shutoff volume: 30 gallons.
constexpr uint8_t CFG_AUTOSHUT_DFLT  = 0;      // Default: do NOT auto-close the valve.
constexpr uint8_t CFG_ALERTMODE_DFLT = 1;      // Default: publish alerts to the cloud.

// Accepted ranges for the cloud setter
// When a user sends new settings from the cloud, we reject anything outside these bounds.
constexpr float CFG_LEAK_GPM_MIN = 0.1f,  CFG_LEAK_GPM_MAX = 200.0f;  // Leak threshold must be 0.1 .. 200 GPM.
constexpr float CFG_SHUTOFF_MIN  = 1.0f,  CFG_SHUTOFF_MAX  = 1000.0f; // Shutoff volume must be 1 .. 1000 gal.

// ============================================================================
//  PIC framed-protocol host parameters (V040 spec section 7).
//  These live only on the P2 -- the PIC does not know them.
// ============================================================================
// These tune HOW we talk to the PIC chip over the serial wire (retries/timeouts).
constexpr uint8_t  PHOTON_RETRY_COUNT     = 3;     // resend a REQ on no/bad RSP
                                                   //   If a reply is missing or corrupt, try up to 3 more times.
constexpr uint32_t PHOTON_TIMEOUT_DATA_MS = 4000;  // waiting for RSP_DATA (big)
                                                   //   Wait up to 4000 ms for a big data reply.
constexpr uint32_t PHOTON_TIMEOUT_READ_MS = 1000;  // small reply (PARAM/VALVE/ACK)
                                                   //   Wait up to 1000 ms for a small reply.
// The two wake constants below are UNUSED under power-gating (there is no 0xF0
// handshake anymore -- being powered already means the PIC is listening). Kept
// only so any lingering references keep compiling.
constexpr uint8_t  PHOTON_WAKE_RETRIES    = 5;     // (unused) former 0xF0 resend count.
constexpr uint32_t PHOTON_WAKE_WAIT_MS    = 100;   // (unused) former per-0xF0 wait.

// ============================================================================
//  Power-gating session timing (PIC firmware V048).
//  The PIC switches the Photon's supply: it powers us only when a report is due,
//  and cuts power when we send PKT_PHOTON_OFF_REQ (func 0x07). Safety nets on the
//  PIC: it powers us off if it hears NO valid packet within 90 s of power-on, and
//  20 s after our LAST packet. So the Photon must finish and send 0x07 well
//  inside those windows.
// ============================================================================
// Cloud-connect give-up. MUST be < the PIC's 90 s "no valid packet" cutoff so the
// OFF_REASON_CLOUD_FAIL 0x07 reaches the PIC before it powers us off blindly.
// 80 s leaves a 10 s margin. (An earlier draft's 800000 ms was a typo for 80000.)
constexpr uint32_t TIMEOUT_CANNOT_FIND_CLOUD_MS = 80UL * 1000UL;   // 80 s

// How many REQ_DATA rounds to issue per session. The first upload after a cold
// boot may carry 0-2 capture periods (COUNT can be 0 -> treated as "nothing
// new", not an error). We re-request a few times in case the PIC had more than
// one chunk pending, stopping early once it reports an empty batch.
constexpr uint8_t  PIC_DATA_MAX_REQUESTS = 4;

// ============================================================================
//  The FOUR PIC leak parameters (Kevin's TODO #2: cloud -> P2 -> PIC).
//  These ARE the 4xu16 payload of REQ_SET_PARAM / RSP_PARAM (spec 4.1), in
//  this fixed order. Sent big-endian by pic_link. Read-modify-write only:
//  GET all four -> change what you want -> SET all four (no per-field write).
//    alert 1 -> TEMPORARY valve lock (auto-clears after 10 min)
//    alert 2 -> PERMANENT valve lock (cleared by UNLOCK or reset)
// ============================================================================
// Defaults match spec 4.1 / the verified REQ_SET_PARAM example frame.
// "counts" = number of flow pulses; "window" = the time span those pulses are counted over.
constexpr uint16_t PIC_LEAK1_COUNTS_DFLT = 100;    // counts
                                                   //   Alert-1 trips at 100 pulses within its window.
constexpr uint16_t PIC_LEAK1_WINDOW_DFLT = 480;    // seconds (8 min)
                                                   //   Alert-1 window is 480 s (8 minutes).
constexpr uint16_t PIC_LEAK2_COUNTS_DFLT = 400;    // counts
                                                   //   Alert-2 trips at 400 pulses within its window.
constexpr uint16_t PIC_LEAK2_WINDOW_DFLT = 180;    // seconds (3 min)
                                                   //   Alert-2 window is 180 s (3 minutes).

// Sanity ranges for the cloud setter (u16 fields).
// New PIC parameters from the cloud must fall inside these limits, or we reject them.
constexpr uint16_t PIC_COUNTS_MIN = 1,   PIC_COUNTS_MAX = 65000;   // "counts" must be 1 .. 65000.
constexpr uint16_t PIC_WINDOW_MIN = 1,   PIC_WINDOW_MAX = 65000;   // "window" seconds must be 1 .. 65000.