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

// Firmware version string, logged at boot so a capture always identifies the
// build it came from. Bump this with every functional change.
#define FW_VERSION_STR "P2-V068"

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
#define PIC_WAKE_PIN        D10    // UNUSED / RESERVED under power-gating (PIC V048).
                                   //   The old WAKE handshake is gone: the PIC now switches the
                                   //   Photon's SUPPLY via a P-MOS on RC4, so D10 carries no signal.
                                   //   Kept defined (and held HIGH via INPUT_PULLUP) so the pin never
                                   //   floats and remaining references compile. Do not gate comms on it.

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
// ============================================================================
//  *** BUILD MODE - the ONE switch that selects the whole build ***
//  ----------------------------------------------------------------------------
//  This is a SINGLE codebase (no separate D / Pro projects). Pick exactly ONE
//  mode with BUILD_MODE below; every other test/production value (PIC timing,
//  bucket width, virtual clock, off-reason) is derived from it automatically, so
//  nothing else has to be edited to switch builds.
//
//    BUILD_MODE_PRODUCTION : normal cloud path (Wi-Fi + Particle cloud + cloud
//                            time). Ships to the field.
//    BUILD_MODE_CLOUD_FAST : Appendix H.13 - the fourth mode. REAL cloud + REAL
//                            cloud time, but the fast benchmark cadence (3-min
//                            report / 60-s buckets) is kept, so the live cloud
//                            path (publish, sec 6.7 replay on a real 'now', the
//                            Time.isValid 0->1 transition) can be exercised in
//                            minutes instead of the 48-h production cadence. Ends
//                            the session with OFF_REASON_DONE. CLOUD_FAIL_EVERY_N
//                            can inject periodic cloud failures.
//    BUILD_MODE_FAST_BENCH : no cloud, but a local VIRTUAL clock is seeded so
//                            Time.isValid() is true. Skips the ~20-40 s cloud
//                            connect per session; PIC UART + USB/debug-UART log
//                            only. Ends the session with OFF_REASON_DONE.
//    BUILD_MODE_CLOUD_FAIL : no cloud AND no clock. Simulates a device that can
//                            talk to the PIC but never reaches the cloud, so the
//                            absolute time never arrives (Time.isValid() stays
//                            false). Wi-Fi is not even attempted and there is NO
//                            connect delay -- the "failure" is immediate, to keep
//                            bench rotations fast. The received data still flows
//                            through the normal time-less path (TIME_SYNC valid=0,
//                            flash storage, no-time hourly rules). Ends the session
//                            with OFF_REASON_CLOUD_FAIL. See docs/CHANGES_V059_to_V060.md.
//
//  Appendix H.13.1 - TWO INDEPENDENT AXES. The old FAST_BENCH_TEST conflated
//  "fast cadence" with "no cloud". They are now separate so the fourth mode above
//  can have fast cadence WITH cloud:
//     CADENCE_FAST  : PIC period / bucket width / initial-hold - the "speed" axis.
//     CLOUD_ENABLED : Wi-Fi + Particle + cloud time - the "cloud" axis.
//  The mapping keeps the three existing modes byte-identical (their axes match the
//  old FAST_BENCH_TEST exactly); only CLOUD_FAST differs.
//
//  The D10-dependency removal from the porting guide applies to ALL modes.
// ============================================================================
#define BUILD_MODE_PRODUCTION 0
#define BUILD_MODE_FAST_BENCH 1
#define BUILD_MODE_CLOUD_FAIL 2
#define BUILD_MODE_CLOUD_FAST 3

#ifndef BUILD_MODE
  #define BUILD_MODE  BUILD_MODE_CLOUD_FAST   // <-- the ONE switch. Set to one of the four above.
#endif                                        //     V065 (req 4.갑): was CLOUD_FAIL. The 9600 campaign
                                              //     proved the metering accounting; CLOUD_FAST turns the
                                              //     time axis on (real Wi-Fi, real cloud time) while
                                              //     keeping the fast bench cadence, so the whole
                                              //     never-executed path -- bucket placement, sec 6.7
                                              //     replay, missed-fill, LOCO second-stage calibration,
                                              //     publish -- runs in minutes.
                                              //     This automatically gives CLOUD_ENABLED=1 and
                                              //     BENCH_VIRTUAL_CLOCK=0 (see the derived block below),
                                              //     which is exactly what req 4.갑 asks for. CADENCE_FAST
                                              //     stays 1 and is independent of the PIC's
                                              //     PCFG_FAST_BENCH, as the request notes.

// ---- Derived axis flags (do not edit; they follow BUILD_MODE) --------------
// CADENCE_FAST and CLOUD_ENABLED are the two axes (Appendix H.13.1). BENCH_VIRTUAL
// _CLOCK separates the two no-cloud modes (FAST_BENCH seeds a virtual clock;
// CLOUD_FAIL does not). Every mode defines all three so they can be tested with
// plain "#if".
#if BUILD_MODE == BUILD_MODE_PRODUCTION
  #define CADENCE_FAST        0
  #define CLOUD_ENABLED       1
  #define BENCH_VIRTUAL_CLOCK 0
#elif BUILD_MODE == BUILD_MODE_CLOUD_FAST
  #define CADENCE_FAST        1
  #define CLOUD_ENABLED       1
  #define BENCH_VIRTUAL_CLOCK 0
#elif BUILD_MODE == BUILD_MODE_FAST_BENCH
  #define CADENCE_FAST        1
  #define CLOUD_ENABLED       0
  #define BENCH_VIRTUAL_CLOCK 1
#elif BUILD_MODE == BUILD_MODE_CLOUD_FAIL
  #define CADENCE_FAST        1
  #define CLOUD_ENABLED       0
  #define BENCH_VIRTUAL_CLOCK 0
#else
  #error "BUILD_MODE must be PRODUCTION, FAST_BENCH, CLOUD_FAIL, or CLOUD_FAST"
#endif

// ---- Legacy compatibility names (unchanged meaning) ------------------------
// FAST_BENCH_TEST historically meant "fast cadence AND no cloud, PIC UART only".
// It is retained for the compile-time sites that are purely CADENCE concerns
// (PIC_MODE_TEST, BUCKET_SEC default), where it equals the cadence axis. Sites
// that were really CLOUD concerns are moved to CLOUD_ENABLED individually.
// BENCH_CLOUD_FAIL keeps its exact meaning: the no-cloud, no-virtual-clock mode
// that reports OFF_REASON_CLOUD_FAIL (only BUILD_MODE_CLOUD_FAIL).
#define FAST_BENCH_TEST   CADENCE_FAST
#define BENCH_CLOUD_FAIL  ((CLOUD_ENABLED == 0) && (BENCH_VIRTUAL_CLOCK == 0))

// Appendix H.13.3: inject periodic cloud failures in a cloud-enabled build so the
// "cloud works intermittently" condition can be verified in one continuous run.
// 0 = always succeed. N = deliberately skip the connect/publish once every N
// sessions, so blocks accumulate (endValid recorded) and the NEXT success session
// replays them on a real 'now' - the positive confirmation of the H.2/H.3 fixes.
// Only meaningful when CLOUD_ENABLED; ignored otherwise.
//
// *** V065: DELIBERATELY 0 FOR THE FIRST CLOUD_FAST CAMPAIGN (req 3 + 7.3) ***
//
// The default used to be 3 here, and that default is incompatible with the run
// this build exists for. Request section 3 states the constraint plainly: the
// ring holds 4 blocks, a report goes out roughly every 3 minutes, so cloud time
// has to arrive WITHIN 4 REPORTS for all four held blocks to be replayed -- the
// ~12-minute window. Checklist item 3 ("RECOVERY: 4 block(s) replayed") is the
// item that has been deferred five campaigns running, and the ring is right now
// holding four clean n=34 blocks (seq 55-58): the ideal starting state, and one
// that is not cheap to reproduce.
//
// An injected failure inside those first four sessions costs one of the four
// report slots AND stores another block, pushing the oldest of the four out of
// the ring. And g_sessionCounter is RETAINED across the power cuts, so it does
// not restart at 1 for this campaign -- it continues from the CLOUD_FAIL run
// (five sessions). With N=3, one session in every three fails, so the first
// four sessions cannot all succeed no matter what phase the counter is in. The
// primary objective would be lost before the run started.
//
// So: 0 for the first campaign. The mechanism is untouched; the request's
// mixed-state interest (section 7 "기록 요령": a failed session mixed in is
// itself an important item) is served by the SECOND run -- set this back to 3,
// or pass -DCLOUD_FAIL_EVERY_N=3 at compile time, once items 1-5 have been
// confirmed once cleanly. Do not raise it before then.
#ifndef CLOUD_FAIL_EVERY_N
  #define CLOUD_FAIL_EVERY_N 0
#endif

// Both bench modes and CLOUD_FAST use the PIC's fast test cadence for their
// pre-handshake estimates, so PIC_MODE_TEST follows CADENCE_FAST automatically
// (see the PIC timing block below). Production keeps the 48 h cadence.
#if CADENCE_FAST
  #define PIC_MODE_TEST 1
#endif

// When there is no cloud clock AND this build seeds a virtual one (FAST_BENCH),
// we set a local clock at boot (see setup()). The absolute value is arbitrary on
// the bench -- only the sample spacing matters -- but it must make Time.isValid()
// true so batches are ingested and logged. 1735689600 = 2025-01-01 00:00:00 UTC.
#define FAST_BENCH_TEST_EPOCH  1735689600UL

// ---- DEBUG: stream the PIC data-series over USB-CDC ----
// Defined   -> ingestPicBatch() prints the per-sample flow series and a compact
//              one-line pulse dump to the USB-CDC log (Log.info). Use this in
//              TEST mode to watch the actual samples.
// Undefined -> those data-series lines are suppressed (CDC output stays quiet);
//              the compact "PIC report: imp/cap/span/ovf/n" summary and other
//              status lines still print.
// We are currently in test mode, so leave this DEFINED to stream the series.
#define DEBUG_CDC_DATASERIES 1

// Bench only: after the USB CDC is enumerated (Serial.isConnected), wait this long so a
// serial-monitor app has time to (re)open the COM port before we start logging. Needed
// because power-gating re-enumerates USB every session. Not used in the cloud build.
// After the USB-CDC is (re)opened at boot, wait this long BEFORE sending log data,
// so the PC's serial monitor has surely re-acquired the COM port (USB re-enumerates
// every time the PIC re-powers us). Measured ~3 s needed for reliable OS COM
// re-recognition. Adjust this value as your host/OS requires.
#define BENCH_SERIAL_SEND_DELAY_MS 10000  // <-- tune here. 10 s: the USB re-enumerates every
                                          //  power-up and the PC monitor auto-connects ~10-13 s
                                          //  later; this holds off the log burst until AFTER the
                                          //  monitor is reading. MUST stay well under the PIC's
                                          //  INITIAL_POWER_HOLD_MS / session length.

// ------------------------------------------------------------------ Timing
// OBSOLETE UNDER POWER-GATING (kept only so old references still compile).
// The Photon no longer owns its own sleep/wake: the PIC switches the supply and
// decides the report cadence. SLEEP_DURATION_S / WAKE_WINDOW_MS /
// SLEEP_CHUNK_MAX_S are referenced by nothing in this project -- do not build
// new logic on them. Report cadence lives on the PIC (App_Config.h,
// APP_SAMPLES_PER_REPORT x APP_CAPTURE_PERIOD_MS).
// Historic note: requirement #4 was "cloud connection only every 24 h".
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

// ---- PIC sample cadence : MUST MATCH the PIC firmware ----
// The PIC (V051) captures one sample every APP_CAPTURE_PERIOD_MS = wake x
// wakes_per_sample:  test = 3 s x 4 = 12 s,  production = 60 s x 4 = 240 s.
// The Photon stores one logger entry per PIC sample (1:1), so METER_INTERVAL_SEC
// and PIC_SAMPLE_INTERVAL_SEC BOTH equal that capture period. Keeping them wrong
// makes GPM rates, leak timing and hourly placement wrong (the daily gallon TOTAL
// still cancels out, but the rate/timeline do not).
//
// ===== PIC TIMING MODE - MUST match the PIC's build (App_Config.h) =====
// PIC_MODE_TEST is now DERIVED from BUILD_MODE (defined above whenever
// FAST_BENCH_TEST is 1), so it is not set by hand here. Build the PIC WITH
// REPORT_CONFIG_DEBUG for the fast-test cadence. These are the only place the
// numbers live; the precise _F value below is the PIC's real capture window.
//   PRODUCTION : wake 114 counts x 0.5285 s = 60.25 s; capture x4 = 241.004 s;
//                720 samples => ~48 h report.
//   FAST TEST  : PIC measured on the bench (Appendix D): wake 0.53 s, capture
//                5.29 s (5290 ms, jitter 0), ~34 samples => ~3 min report.
// NOTE: both values below are only PRE-HANDSHAKE ESTIMATES. RSP_PHOTON_CFG (0x89)
// overwrites captureIntervalSecF and samplesPerReport at runtime with the PIC's
// real numbers, so the header's sample_count -- not this 34 -- is authoritative.
// They are set close to the measured values only so the first session or two do
// not compute wildly off before the handshake lands.
#ifdef PIC_MODE_TEST
  #define PIC_CAPTURE_PERIOD_SEC     5           // integer, for timestamp spacing (measured 5.29 s)
  #define PIC_SAMPLE_INTERVAL_SEC_F  5.29f       // precise window (rate/gallons); measured 5290 ms
  #define PIC_SAMPLES_PER_REPORT     34          // nominal samples per report (Appendix D)
#else
  #define PIC_CAPTURE_PERIOD_SEC     240         // integer, for timestamp spacing
  #define PIC_SAMPLE_INTERVAL_SEC_F  241.004f    // precise window (rate/gallons)
  #define PIC_SAMPLES_PER_REPORT     720
#endif
#define REPORT_PERIOD_SEC  ((uint32_t)PIC_SAMPLES_PER_REPORT * (uint32_t)PIC_CAPTURE_PERIOD_SEC)

// FAST_BENCH_TEST virtual clock jitter: each session's advance is REPORT_PERIOD_SEC
// plus/minus up to this, so the sample count vs the nominal period drifts by a
// sample or two -- deliberately exercising the boundary-cut / zero-fill path.
// Kept well under the report period so the clock always moves forward.
#define BENCH_TIME_JITTER_SEC (2u * (uint32_t)PIC_CAPTURE_PERIOD_SEC)   // ~2 samples

constexpr uint32_t METER_INTERVAL_SEC      = PIC_CAPTURE_PERIOD_SEC; // FALLBACK only (see note below)
constexpr uint32_t PIC_SAMPLE_INTERVAL_SEC = PIC_CAPTURE_PERIOD_SEC; // FALLBACK only (see note below)

// V056 NOTE - the capture interval is a RUNTIME value.
// RSP_PHOTON_CFG (0x89) is the single source of truth for the capture interval
// (PIC App_Config_Photon.h derives it from APP_CAPTURE_PERIOD_MS). The two
// constants above are used ONLY until that exchange succeeds, and for the
// nominal-interval fallback inside ingestPicBatch(). Nothing that must match the
// PIC may be derived from them at compile time - see METER_LOG_SLOTS below.

// Method A note: PIC_SAMPLE_INTERVAL_SEC_F (set in the mode block above) is the
// PIC's REAL count-multiple capture window and is used for the RATE (freq/gallons)
// so the count-quantization error is removed. The integer PIC_SAMPLE_INTERVAL_SEC
// is kept for timestamp spacing only.
constexpr uint32_t LEAK_MODEL_INTERVAL_SEC = 300;  // 5 min leak slots
                                                   //   The leak-learning model uses 5-minute (300 s) buckets.

// ---- How to handle flow BEFORE the received series ----
// When the PIC's ring safety policy truncates a long backlog (a report was
// skipped, e.g. cloud woke after 3 days but the series only carries the last
// day), the RSP_DATA header still reports the TRUE totals (impulseSinceReport /
// capturesSinceReport). Choose how the Photon treats the un-received front span:
//   ZERO    = ignore it. Only the received (latest) samples count; the older
//             volume is dropped. Matches "front-series loss is by design."
//   AVERAGE = reconstruct it as an average flow computed from
//             (impulseSinceReport - received pulses) over (captures - n), so the
//             daily TOTAL gallons are preserved (the impulse count stays right).
#define PIC_MISSED_FILL_ZERO     0
#define PIC_MISSED_FILL_AVERAGE  1
#define PIC_MISSED_FILL  PIC_MISSED_FILL_AVERAGE   // <-- select ZERO or AVERAGE here

// ---- Interval-logger capacity (V056) --------------------------------------
// The logger capacity must NOT be derived from a compile-time capture interval:
// the real interval arrives at runtime in RSP_PHOTON_CFG and can be much shorter
// than the compiled guess (PIC bench build = 12.684 s vs the 241.004 s
// production value). Sizing from the wrong number is what made the logger
// silently stop storing after a fraction of a day.
//
// METER_LOG_SLOTS is therefore a FIXED capacity, independent of the interval:
//   240 s interval -> 1440 slots = 96 h of history (more than a day)
//    12 s interval -> 1440 slots =  4.8 h of history
// When it is full the logger keeps the NEWEST samples and drops the oldest
// (see appendIntervalSample()), so a session always publishes recent data
// instead of a stale prefix. RAM cost = 1440 * 2 B = 2.88 kB.
constexpr uint16_t METER_LOG_SLOTS = 1440;
constexpr uint16_t MAX_SAMPLES     = METER_LOG_SLOTS;   // legacy name kept for existing code

// Publishing 1 chunk per 120 samples with a 1.1 s cloud rate-limit gap means a
// full logger takes ~13 s of publishing during which NO packet reaches the PIC.
// The PIC cuts our power after TIMEOUT_NO_MORE_MSG_MS (20 s) of silence, so the
// publish path pumps a KEEPALIVE (0x0A) whenever the link has been quiet this
// long. Keep well under the PIC's 20 s.
constexpr uint32_t PIC_BUSY_KEEPALIVE_MS = 6000;   // 6 s (PIC idle backstop = 20 s)

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

// Appendix H.10.5 (1) - INTERIM domain measure. The FLOW_C* coefficients above are
// FROZEN (Appendix H.10: they are the client's reference from physical testing;
// do NOT change them). This flag governs only how the APPLICATION treats a
// frequency ABOVE the calibration's usable peak: the raw polynomial falls and
// then collapses to 0 there, so a real over-peak flow would read 0 GPM. With this
// set, freqToGpmUsable() clamps such a result to the peak GPM (a LOWER BOUND) and
// the code emits an explicit interim WARN, instead of silently losing the volume.
// The pure polynomial in flow_cal.h is untouched (the G.6.3 collapse tests still
// pin it). Set to 0 to restore the raw curve once the client confirms the domain.
#ifndef FLOW_CLAMP_ABOVE_PEAK
  #define FLOW_CLAMP_ABOVE_PEAK 1
#endif

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
// OBSOLETE: the 0xF0 wake handshake no longer exists (being powered already
// means the PIC is listening). Nothing references these; kept so an old branch
// or snippet still compiles. Do not reintroduce a wake handshake.
constexpr uint8_t  PHOTON_WAKE_RETRIES    = 5;     // (unused) former 0xF0 resend count.
constexpr uint32_t PHOTON_WAKE_WAIT_MS    = 100;   // (unused) former per-0xF0 wait.

// ---- V064 P-10: RX quiesce before a RETRY ---------------------------------
// A timed-out RSP_DATA can still be streaming when the timeout expires: 2023
// bytes at 9600 8N1 is ~2107 ms of wire time (it was ~527 ms at 38400), which
// is a large fraction of PHOTON_TIMEOUT_DATA_MS in the worst case. sendFrame() flushes the RX buffer at the instant it sends,
// which discards what has ALREADY arrived but not the tail still in flight -
// those bytes land after the flush and become the leading garbage of the next
// reply. readFrame() resynchronises on AA 55, so this is usually survivable,
// but a false AA 55 inside sample data is not: the parser then locks onto a
// bogus length and the attempt is lost too.
//
// So before a retry we wait for the line to go QUIET, then flush. Only retries
// pay this cost, and only until the tail finishes.
//
// V065 (req 4.을): THE LINK IS 9600 NOW, and this cap was still sized for 38400.
// The whole point of the cap is that it must outlast the longest tail that can
// still be in flight, and wire time scales with the baud rate:
//     2030 B x 10 bits / 9600 = ~2115 ms      (was ~527 ms at 38400)
// At 700 ms the wait would have expired with roughly 1.4 s of the tail still
// coming, and those bytes become the leading garbage of the next reply -- the
// exact failure mode P-10 was written to remove, silently reintroduced by the
// baud change.
//
// This did NOT show up in the 9600 campaign, and could not have: a bench frame
// is 99 B = ~103 ms, so 700 ms covered it with room to spare. It only bites on
// a LARGE frame, which is precisely what the CLOUD_FAST replay test is about to
// generate. Hence the request's "지금 확인해 주십시오".
//
// PHOTON_RX_IDLE_MS is left at 5 ms and is still correct: one byte at 9600 is
// ~1.04 ms, so a 5 ms silence is ~4.8 byte-times -- unambiguously a gap, not the
// spacing inside a continuous stream.
constexpr uint32_t PHOTON_RX_IDLE_MS      = 5;     // line counts as quiet after this gap
constexpr uint32_t PHOTON_RX_QUIESCE_MAX_MS = 2500; // hard cap (> 2030 B at 9600 = ~2115 ms)

// ---- V065: the baud rate now lives HERE, and the cap is checked against it --
//
// The 700 ms bug above happened because the baud rate was a bare literal at the
// picLink.begin() call site in leaksense.cpp while the constant that DEPENDS on
// it sat here, several hundred lines away in another file. Changing one could
// not remind anyone to change the other. Moving the baud into the same block as
// the constants derived from it, and making the compiler check the relationship,
// is what stops the next baud change from reintroducing this silently.
//
// Worst-case RSP_DATA on the wire: PIC_MAX_SAMPLES (1000) x 2 bytes + a 24-byte
// header + 6 bytes of framing (AA 55, func, len u16, CRC16) = 2030 bytes. 8N1
// puts 10 bits on the wire per byte.
constexpr uint32_t PIC_UART_BAUD          = 9600UL;   // Serial1 to the PIC, 8N1
constexpr uint32_t PIC_MAX_FRAME_BYTES    = 2030UL;   // worst-case RSP_DATA incl. framing
constexpr uint32_t PIC_MAX_FRAME_WIRE_MS  =
    (PIC_MAX_FRAME_BYTES * 10UL * 1000UL) / PIC_UART_BAUD;   // 9600 -> 2114 ms

static_assert(PHOTON_RX_QUIESCE_MAX_MS > PIC_MAX_FRAME_WIRE_MS,
              "PHOTON_RX_QUIESCE_MAX_MS must outlast the longest frame that can "
              "still be in flight at PIC_UART_BAUD, or a retry starts on top of "
              "a tail it never drained (V064 P-10 / V065 req 4.b). Raise the cap "
              "or raise the baud.");

// The frame timeout must also cover the wire time, or a good large reply is
// abandoned mid-stream and every attempt times out identically. At 9600 the
// margin is 4000/2114 = 1.9x, down from 7.6x at 38400 - adequate, but this is
// now the tightest ratio in the link layer, so it is pinned rather than assumed.
static_assert(PHOTON_TIMEOUT_DATA_MS > PIC_MAX_FRAME_WIRE_MS,
              "PHOTON_TIMEOUT_DATA_MS is shorter than the wire time of a "
              "worst-case RSP_DATA at PIC_UART_BAUD: a large backlog frame could "
              "never be received, at any retry count.");

// ---- V064 P-5: RSP_DATA batch_seq (24-byte header) -------------------------
// PIC V023 retransmits an un-ACKed batch unchanged, across sessions. That closes
// the data-loss hole but opens a double-counting one: a lost 0x0B makes the PIC
// resend a batch we already stored, and with identical sample VALUES there is no
// way to tell "the same batch again" from "new data that happens to match".
// batch_seq - one byte appended to the RSP_DATA header - is that identifier.
//
// PAIRED SWITCH. This must track the PIC's PCFG_BATCH_SEQ_ENABLE exactly:
//    0 = 23-byte RSP_DATA header (PIC V022, and V023 with the switch off)
//    1 = 24-byte RSP_DATA header, byte 23 = batch_seq; 0x0B carries the echo
// A mismatch is a HARD failure, by design: requestData() rejects every frame as
// PIC_ERR_BAD_FRAME rather than silently misreading a length. Where metering
// data is at stake, stopping loudly beats a quiet disagreement.
//
// Default 0 mirrors the PIC default, so the V064 stage-1 fixes (P-1/P-2/P-3/P-10)
// can be deployed and bench-tested against a V022 or switch-off V023 PIC first.
// Flip both sides together for stage 3.
#ifndef PHOTON_BATCH_SEQ_ENABLE
#define PHOTON_BATCH_SEQ_ENABLE 0
#endif

// ---- V064 P-6: the forbidden 2000-01-01 epoch band -------------------------
// An unsynced Particle clock reads 946684800 (2000-01-01 UTC). H.3 made
// clockNowUtc() the single gate so no NEW code path can write that value, but a
// value already sitting in a persisted blob from an older build survives a
// firmware update. Anything inside this band is not a real measurement time.
constexpr uint32_t EPOCH_2000_UTC      = 946684800UL;
constexpr uint32_t EPOCH_FORBIDDEN_LO  = EPOCH_2000_UTC - 86400UL;
constexpr uint32_t EPOCH_FORBIDDEN_HI  = EPOCH_2000_UTC + 86400UL;
static inline bool epochInForbiddenBand(uint32_t e) {
  return (e >= EPOCH_FORBIDDEN_LO && e <= EPOCH_FORBIDDEN_HI);
}

// ---- V066 req 갑/을 : PIC 첫 교환 전 침묵 구간 --------------------------------
// 전원 인가 직후의 과도현상(전압 천이, 입력핀 ESD 경로 안정화)이 가라앉을 때까지
// PIC와 아무것도 주고받지 않는 구간.
//
// 안전 마진: PIC는 첫 패킷을 TIMEOUT_NO_MSG_PHOTON2PIC_MS (90,000 ms) 까지
// 기다린다. 3000 ms 침묵은 그 한계의 3.3%에 불과하므로 PIC 측에 영향 없음.
//
// 이 상수는 leaksense.cpp setup() 안 picLink.begin() 바로 뒤,
// requestPicConfig() 바로 앞에서 단순 delay()로 소비된다.
// SYSTEM_MODE(MANUAL)이지만 Particle의 delay()는 내부에서 Particle.process()를
// 호출하므로 시스템은 계속 동작한다.
// 이 구간이 끝난 뒤 picLink.flushRx()로 전원 천이가 밀어넣은 쓰레기를 제거한다.
constexpr uint32_t PIC_QUIET_MS = 3000;   // ms — 전원 인가 후 첫 PIC 교환 전 대기

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

// Keepalive cadence while waiting for the cloud (STATE_CONNECTING). During that
// window the Photon sends no requests to the PIC, so without this the PIC's 20 s
// "no more message" backstop would cut our power ~31 s after boot -- before we
// can connect (or spend the full 80 s budget above). A zero-payload PKT_KEEPALIVE
// every interval resets that timer, so a LIVE-but-connecting Photon is never cut;
// a truly-dead Photon stops sending them and the 20 s backstop still fires (safety
// net preserved). MUST be well under the PIC's TIMEOUT_NO_MORE_MSG_MS (20 s):
// 5 s = 4 keepalives per window, tolerant of a lost frame or two.
constexpr uint32_t KEEPALIVE_INTERVAL_MS = 5000;   // 5 s (keep << PIC's 20 s idle timeout)

// How many REQ_DATA rounds to issue per session. The first upload after a cold
// boot may carry 0-2 capture periods (COUNT can be 0 -> treated as "nothing
// new", not an error). We re-request a few times in case the PIC had more than
// one chunk pending, stopping early once it reports an empty batch.
constexpr uint8_t  PIC_DATA_MAX_REQUESTS = 4;

// ============================================================================
//  The FOUR PIC leak parameters (Kevin's TODO #2: cloud -> P2 -> PIC).
//  These ARE the 4xu16 payload of REQ_SET_LEAK / leak part of RSP_PARAM (spec
//  4.1), in this fixed order. Sent big-endian by pic_link. Read-modify-write
//  only: GET all four -> change what you want -> SET all four (no per-field write).
//    alert 1 -> TEMPORARY valve lock (auto-clears after 10 min)
//    alert 2 -> PERMANENT valve lock (cleared by UNLOCK or reset)
// ============================================================================
// Defaults match spec 4.1 / the verified REQ_SET_LEAK example frame.
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
// ---- Bucket width and bucket COUNT (V057) ---------------------------------
// bucketSec is the width of one reported bucket. It is deliberately independent
// of the report anchor and the report interval: {midnight, 48 h report, 1 h
// bucket} is only one combination among many, and {21:00, 3 h, 10 min} must work
// just as well.
constexpr uint32_t BUCKET_SEC_PROD = 3600u;   // PRODUCTION: 1-hour buckets = true hourlyGallons
constexpr uint32_t BUCKET_SEC_TEST = 60u;     // TEST: 1-min buckets. With a ~3-min report this yields
                                              //   ~3 closed buckets per report; a 5-min bucket vs a 3-min
                                              //   report would close none and the series would look empty.

// *** The number of buckets a report produces is NOT fixed. ***
// A 48 h report with 1 h buckets typically yields 47, 48 or 49 depending on the
// PIC's clock error and where the grid alignment landed, and after a cloud
// outage a merged report can yield hundreds. Nothing may assume 24 or 48.
// HOURLY_MAX_BUCKETS is the working array size: five production reports' worth
// (5 x 48 = 240) plus margin. Buckets beyond it are dropped OLDEST-first and the
// loss is reported as (totalMakeable - count), i.e. the "MM of which NN" pair.
constexpr uint16_t HOURLY_MAX_BUCKETS = 250;

// How many buckets fit in one cloud event. A Particle publish carries at most
// 1024 bytes, and one bucket costs about 6 characters ("123.4,"), so the bucket
// array is published in chunks with a repeated header rather than truncated.
//
// V068: lowered 96 -> 64. Two things changed under this number at once. The
// audit array now publishes at HOURLY_BUCKET_DECIMALS (3) places instead of one,
// so a bucket costs up to 9 characters ("1234.567,") rather than 6; and the
// chunk header grew by the UTC/local epoch pair (section 4 of the request). At
// 96 buckets the worst case is 96*9 = 864 bytes of array against a ~330 byte
// header - past both JsonWriterStatic<1024> and the Particle event limit, which
// would have truncated the payload silently. 64 keeps the worst case near 900
// bytes total, and a production 48-bucket report still goes out in ONE chunk, so
// the published event count is unchanged (see the section 11 count).
constexpr uint16_t HOURLY_PUBLISH_PER_CHUNK = 64;

// ---- V068 contract publish (Project Agreement 2026-07-18 sec 2 req 3/4/5) ---
//
// The agreement names a fixed 48-slot "hourlyGallons" array. Our own bucket
// series is NOT 48 long and that is correct behaviour, not a defect: the PIC has
// no precision RTC, so a 48 h report routinely closes 47 or 49 buckets, and a
// flash-ring recovery can deliver 96 or more at once. Forcing that series into
// 48 slots would throw the older half of a recovery away.
//
// So the two are published as two separate events with two different jobs:
//   hourlyGallons  - the contract's fixed 48-slot sliding window. DISPLAY. What
//                    the dashboard draws. One decimal is enough for it.
//   hourlyBuckets  - the variable-length series we already produced (this is the
//                    event that was called "hourlyGallons" up to V067). AUDIT.
//                    What the conservation checks are compared against, so it
//                    keeps three decimals.
// Splitting them is what lets each be right on its own terms.
constexpr uint8_t  ROLL48_COUNT    = 48;   // contract array length. Fixed by the agreement.
constexpr uint8_t  ROLL48_DECIMALS = 1;    // display array: 0.1 gal = 0.8 % at a production 12 gal/h
constexpr uint8_t  HOURLY_BUCKET_DECIMALS = 3;  // audit array: the bench runs 0.3 gal/h, where 0.1 gal
                                                //   granularity would be a 7 % error and the
                                                //   conservation check could not close.

// Particle's event data limit, and the point at which we complain. The margin is
// deliberate: a payload that grows past the warn line still publishes, but it
// says so in the log. Silent truncation is the failure this is here to prevent.
constexpr uint16_t PARTICLE_EVENT_MAX_BYTES = 1024u;
constexpr uint16_t CONTRACT_EVENT_WARN_BYTES = 600u;

// Requirement 3: the contract's 4-column UART diagnostic table
// (Timestamp / sample_ID / sample_count / GPM), emitted per sample.
//
// DIAG_TABLE_ENABLE is deliberately NOT gated on debugHourly. The agreement asks
// for this table to be readable "using Photon serial without the cloud", and a
// line hidden behind a debug level does not meet that.
//
// DIAG_VERBOSE_ENABLE keeps the existing detailed [DAT] sample line alongside it.
// The 4 columns cannot express bin, dt, freq/g0/poly or gal, and dt is the field
// that exposed the 2x Timer0 period error in the 2026-08 campaign. The agreement
// specifies what must be present, not what must be absent; the submission
// screenshot filters the terminal to [DIAG].
constexpr bool DIAG_TABLE_ENABLE   = true;
constexpr bool DIAG_VERBOSE_ENABLE = true;

// Legacy 24-slot rolling view, kept only for the existing dashboard schema and
// for the contract JSON (bin0 = oldest completed, bin47 = newest). The authoritative
// output is the variable-length bucket list above.
constexpr uint8_t  BUCKET_COUNT    = 24;      // legacy hourlyGallons[] length

// Bucket alignment scheme (feature K / doc 05 section 5.3). Both are implemented;
// pick one here.
//   BUCKET_ALIGN_CLOCK    : boundaries on absolute clock multiples of bucketSec,
//                           so 3600 s buckets read "22:00-23:00". Only the newest
//                           bucket is ever partial. Preferred for display.
//   BUCKET_ALIGN_FROM_NOW : boundaries measured back from the end of the span, so
//                           the newest bucket closes exactly at T_end and the
//                           partial one is the oldest. Simpler, off-clock labels.
#define BUCKET_ALIGN_CLOCK     1
#define BUCKET_ALIGN_FROM_NOW  2
#ifndef BUCKET_ALIGN_MODE
  #define BUCKET_ALIGN_MODE    BUCKET_ALIGN_CLOCK
#endif

// Compile-time default, used before RSP_PHOTON_CFG is answered. Bucket width is a
// CADENCE concern (Appendix H.13.1), so it follows CADENCE_FAST, not the cloud axis.
#ifndef BUCKET_SEC
  #if CADENCE_FAST
    #define BUCKET_SEC   BUCKET_SEC_TEST
  #else
    #define BUCKET_SEC   BUCKET_SEC_PROD
  #endif
#endif

// ---- Local time (V057, addendum A.1) --------------------------------------
// DESIGN A: the cloud gives UTC only; the Photon adds a fixed offset and hands
// the PIC a LOCAL epoch in TIME_SYNC. The PIC compares epoch % 86400 against its
// anchor, so giving it local time is what makes its report grid land on local
// midnight. The offset is therefore applied in exactly ONE place - the TIME_SYNC
// send - and every downstream epoch (start_time echoed back, bucket boundaries,
// carry seams) is already local. Do not convert anywhere else.
//
// Fixed offset only: daylight saving is deliberately NOT handled, matching the
// contract. Changing the offset shifts the whole local timebase, so it may only
// be changed through the reboot path (see A.3) - never applied mid-run.
//
// NOTE (open question, see docs/CHANGES_V056_to_V057.md): the addendum specifies
// the default as "the US Washington region", which is ambiguous - Washington
// State is UTC-8 and Washington DC is UTC-5. V056 ran at -8 and that is kept, so
// behaviour does not change silently. Confirm before shipping.
constexpr int      TIME_ZONE_HOURS    = -8;                            // display + default offset
constexpr int32_t  TZ_OFFSET_SEC_DFLT = (int32_t)TIME_ZONE_HOURS * 3600;
constexpr int32_t  TZ_OFFSET_SEC_MIN  = -12 * 3600;                    // accepted range for the
constexpr int32_t  TZ_OFFSET_SEC_MAX  =  14 * 3600;                    // cloud setter

// ---- Report grid the PIC uses (anchor + interval) -------------------------
// anchor   = seconds-of-day of the grid origin (midnight = 0), LOCAL.
// interval = seconds between report grid points.
// These live on the PIC but it has no non-volatile memory, so the Photon stores
// them and writes them back after every cold boot.
constexpr uint32_t GRID_ANCHOR_SEC_DFLT   = 0u;               // local midnight
constexpr uint32_t GRID_INTERVAL_SEC_PROD = 48UL * 3600UL;    // 48 h
constexpr uint32_t GRID_INTERVAL_SEC_TEST = 1800UL;           // 30 min bench
constexpr uint32_t GRID_INTERVAL_SEC_MIN  = 60u;              // sanity bounds for the
constexpr uint32_t GRID_INTERVAL_SEC_MAX  = 7UL * 24UL * 3600UL;  // cloud setter

// Master switch for pushing anchor/interval/offset changes down to the PIC.
// OFF = the Photon still accepts and stores the values but never reboots the
// system to apply them, so bring-up cannot be disturbed by a stray cloud call.
#define ALLOW_PARAM_CHANGE 1

// Read the grid back after REQ_SET_GRID (0x0F) was ACKed (Appendix E).
// An ACK proves the PIC PARSED the frame; it does not prove the PIC APPLIED it.
// A PIC built with PIC_USE_OWN_TIMING ACKs the write but keeps its own grid
// (Appendix E.2). Without this read-back the Photon would clear its "grid
// pending" flag on an ACK the PIC ignored, and the two sides would silently
// disagree about when reports are due. Costs one extra REQ_GET_PARAM, and only
// on the sessions that actually write the grid.
#define PIC_VERIFY_SET_GRID 1

// Bench verification log detail for the hourly (bucket) engine (Appendix E.4).
// 0 = summary: with more than 12 buckets, print the first 3 and last 3 and a
//     "... N more" line. This is the default and is plenty for a normal report.
// 1 = verbose: print every bucket, however many there are. Turn this on to
//     hand-verify a long-outage recovery where hundreds of buckets are produced.
// With 12 or fewer buckets every bucket is printed regardless of this setting.
#ifndef APP_DEBUG_VERBOSE_HOURLY
  #define APP_DEBUG_VERBOSE_HOURLY 0
#endif

// hourlyGallon calculation-verification detail (Appendix G.4, extended by H.21).
// This is the verdict-oriented level control the calibration review asked for; it
// is a LOCAL setting (not on the PIC wire, so the PIC is unchanged - Appendix F.5).
//   0 = off      : pre-V062 output only (existing E.4 lines), plus the always-on
//                  WARN lines below. Nothing extra is printed.
//   1 = summary  : + coefficient/valid-range banner (G.3.1), grid line (G.3.2),
//                  missed-fill intermediates (G.3.4), split bucket lines (G.3.5),
//                  the two self-checks (G.3.6/G.3.7), and the H.20 session summary.
//   2 = detailed : summary + the per-sample conversion intermediates (G.3.3), and
//                  the Appendix H.15-H.18 cloud / flash / time / seam lines.
//   3 = full     : detailed + the H.19 leak-model [LEAK] block and any internal
//                  state dumps. Recommended for the CLOUD_FAST verification run.
//
// Regardless of level, the collapse / mismatch / carry-loss WARN lines ALWAYS
// print (Appendix H.6) - a silent 0.0000 is exactly what Appendix G exists to
// prevent. Default: full on CLOUD_FAST, detailed on the other bench modes,
// summary in production.
#ifndef APP_DEBUG_HOURLY_LEVEL
  #if BUILD_MODE == BUILD_MODE_PRODUCTION
    #define APP_DEBUG_HOURLY_LEVEL 1
  #elif BUILD_MODE == BUILD_MODE_CLOUD_FAST
    #define APP_DEBUG_HOURLY_LEVEL 3
  #else
    #define APP_DEBUG_HOURLY_LEVEL 2
  #endif
#endif

// ---- Time-axis continuity --------------------------------------------------
// How far apart the resolved span start and the previous report's end may be and
// still be treated as the same seam. Wider than one bucket would let a real gap
// masquerade as continuity; a couple of capture intervals is the right scale.
constexpr uint32_t SPAN_CONTINUITY_TOL_SEC = 120u;

// ---- Flash buffer for cloud outages (feature G / doc 05 section 6) ---------
// Number of stored raw-report blocks. 0 disables buffering entirely: every
// report stands alone and a failed publish is simply lost. N blocks allow a
// recovery to merge up to N+1 reports.
#ifndef FLASH_BUFFER_BLOCKS
  #define FLASH_BUFFER_BLOCKS 4
#endif
#if FLASH_BUFFER_BLOCKS < 0 || FLASH_BUFFER_BLOCKS > 4
  #error "FLASH_BUFFER_BLOCKS must be 0..4"
#endif

// Bound the boot power-state handshake (REQ_POWER_STATE). V055 retried forever,
// which parked the Photon in setup() with no keepalives if the PIC never
// answered - the PIC then killed power at its 90 s no-packet cutoff with nothing
// logged. We now retry a bounded number of times, then assume NORMAL and let the
// normal session (and its CLOUD_FAIL path) run.
constexpr uint8_t  POWER_STATE_MAX_TRIES = 10;    // 10 x 1 s = 10 s worst case

// ============================================================================
//  Debug UART log mirror + cloud-time logger (ported from tested dbg_uart demo)
//  ----------------------------------------------------------------------------
//  Adds a SECOND log sink on a hardware UART TX pin: every Log.*() line comes
//  out BOTH the USB serial port AND this UART, at the same time (the USB handler
//  is never removed). Also logs the cloud time once per minute. No resource
//  conflict: Serial1 stays dedicated to the PIC link. See dbg_uart.h/.cpp.
//
//  PORT CHOICE (Photon 2 / P2), TX pins are FIXED in silicon:
//    Serial1 : TX=D8  RX=D9    -> PIC link (off limits)
//    Serial2 : TX=D4  RX=D5    -> collides with SHUTOFF_SWITCH_PIN (D4) here!
//    Serial3 : TX=D15 RX=D16   -> free on this board  <-- default
//  Use Serial3 so nothing moves. (To use Serial2, set DBG_UART_SELECT 2 AND
//  move SHUTOFF_SWITCH_PIN off D4.)
#define DBG_UART_SELECT 3        // 2=Serial2(D4/D5), 3=Serial3(D15/D16, default here)

#if PLATFORM_ID == PLATFORM_P2
  #define DBG_UART_ENABLE   1
  #if DBG_UART_SELECT == 2
    #define DBG_UART_PORT   Serial2
    #define DBG_UART_TX_PIN D4
    #define DBG_UART_RX_PIN D5
    #define DBG_UART_TX_NAME "D4"
  #elif DBG_UART_SELECT == 3
    #define DBG_UART_PORT   Serial3
    #define DBG_UART_TX_PIN D15
    #define DBG_UART_RX_PIN D16
    #define DBG_UART_TX_NAME "D15 (MOSI)"
  #else
    #error "DBG_UART_SELECT must be 2 (Serial2, D4/D5) or 3 (Serial3, D15/D16)"
  #endif
#else
  #define DBG_UART_ENABLE   0        // Gen3 exposes Serial1 only, and it is taken.
  #define DBG_UART_PORT     Serial1
  #define DBG_UART_TX_PIN   PIN_INVALID
  #define DBG_UART_RX_PIN   PIN_INVALID
  #define DBG_UART_TX_NAME  "none"
#endif

#define DBG_UART_BAUD       115200            // 8N1. Match this in the terminal app.
#define DBG_UART_LOG_LEVEL  LOG_LEVEL_INFO    // Same level as the USB handler.

constexpr uint32_t DBG_TIME_LOG_INTERVAL_MS = 60UL * 1000UL;   // 1 minute cloud-time log
constexpr uint32_t DBG_TIME_SYNC_TIMEOUT_MS = 10UL * 1000UL;   // 10 s sync wait
