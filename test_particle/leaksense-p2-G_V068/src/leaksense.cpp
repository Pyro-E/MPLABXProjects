/*
 * LeakSense P2 firmware  -  V056 (Photon side of the PIC V055 contract)
 * Integrates:
 *   - LeakSense flow/leak/publish pipeline (doc 1)
 *   - PIC18F06Q40 UART meter source (doc 2)  -> pic_link.*
 *
 * Kevin's requirements:
 *   #1 Ingest PIC data and align it to hourly buckets + the interval logger.
 *   #2 Push 4 cloud-set variables (alert + shutoff settings) down to the PIC.
 *   #3 Boron build kept compatible via PLATFORM guards (Kevin verifies HW).
 *   #4 Report cadence: OWNED BY THE PIC under power-gating. The PIC powers this
 *      board only when a report is due; the session ends with PHOTON_OFF_REQ.
 *      (The original "connect every 24 h" self-sleep model is gone.)
 *
 * Target: Particle Photon 2 (P2), Device OS 5.6.0+.
 *
 * BEGINNER NOTE:
 *   This is the MAIN program. In Particle/Arduino firmware, execution always
 *   starts with setup() (run once at power-on) and then loop() (run over and
 *   over forever). Find those two functions near the BOTTOM of this file to see
 *   the big picture; everything above them is helpers that setup()/loop() call.
 */

#include "Particle.h"                 // Core device-OS API (pins, Serial, Time, sleep, cloud, etc.).
#include "app_config.h"               // Our shared settings (pins, timeouts, defaults).
#include "pic_link.h"          // The PicLink class for talking to the PIC chip.
#include "hourly.h"          // Time reconstruction + bucket binning (V057).
#include "roll48.h"          // Contract 48-slot sliding window + lifetime total (V068).
#include "flow_cal.h"        // Hz -> GPM calibration polynomial + valid-range scan (V062, Appendix G).
#include "flash_buffer.h"    // Raw-report ring for cloud outages (V057).
#include "JsonParserGeneratorRK.h"    // A helper library to build JSON text we publish to the cloud.
#if USE_IMU                            // Only if the optional motion sensor is enabled (it is OFF by default)...
#include "Adafruit_LSM6DS33.h"         // ...include the IMU driver library.
#endif                                 // End of the optional include.

#include <fcntl.h>      // open() flags (O_WRONLY, O_CREAT, ...) for reading/writing files.
#include <unistd.h>     // read(), write(), close() for files.
#include <sys/stat.h>   // File status/permission helpers.
#include <string.h>     // memset(), strlen(), and similar C string/memory tools.
#include <math.h>       // sqrtf(), floorf(), and other math functions.
#include <stdlib.h>     // atol() for parsing the cloud function arguments.

PRODUCT_VERSION(2);                            // Tag this firmware as product version 2 (for the Particle cloud).
SYSTEM_MODE(MANUAL);                           // We control Wi-Fi/cloud connection ourselves (not automatic).
SerialLogHandler logHandler(LOG_LEVEL_INFO);   // Send Log.info()/warn()/error() messages to the USB serial port.

#include "dbg_uart.h"    // Second log sink on a hardware UART (Serial3 D15) + 1-min cloud-time logger.

// ============================================================================
//  LOG TAG TAXONOMY  (every line is time-stamped by the Log handler)
//    [SYS]  boot / init / config          e.g. [SYS] BOOT ...
//    [PWR]  power + session lifecycle      e.g. [PWR] PIC power-state = NORMAL
//    [PKT]  PIC<->Photon UART frames       e.g. [PKT] TX REQ_DATA(0x01) len=0 ...
//    [EVT]  notable protocol events        e.g. [EVT] BATCH_STORED_ACKED n=12
//    [CLD]  cloud comms                    UP / DOWN / WAIT / REQ / RESP / FUNC / PUB / SIM
//    [DAT]  decoded meter data             RSP_DATA hdr, per-sample, hourlyGallons
//    [LEAK] leak / valve actions           (add as needed)
//  Rich, greppable, and self-explaining so a capture is easy to interpret and
//  to debug from. Add a new [XXX] tag when a new subsystem deserves one.
// ============================================================================

// dbg_uart.cpp references this to dump settings after a cloud-function call.
// Kept dependency-free (no globals) so it can live near the top; extend later
// if you wire REGISTER_LOGGED_CLOUD_FN and want values echoed.
void dbgLogSettingsSnapshot(const char *tag) {
  Log.info("SETTINGS[%s] (snapshot)", tag ? tag : "?");
}

// On P2/Photon 2 retained memory must be explicitly enabled and is only
// persisted to flash when System.backupRamSync() is called (Device OS 5.3.1+).
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));   // Turn on "retained" RAM (survives sleep) very early at boot.

// ============================================================ Timing constants
// "const unsigned long" = a fixed whole-number value that never changes at run time.
const unsigned long DEBOUNCE_TIME            = 300;    // Ignore button presses closer than 300 ms apart (debounce).
const unsigned long SHUTOFF_TIMER_MS         = 10000;  // The valve power stays on for 10,000 ms (10 s) per action.
const unsigned long WIFI_CONNECT_TIMEOUT_MS  = 20000;  // Give Wi-Fi up to 20 s to connect.
const unsigned long CELL_CONNECT_TIMEOUT_MS  = 90000;  // Give cellular up to 90 s to connect (it's slower).
const unsigned long PARTICLE_DISCONNECT_MS   = 5000;   // Wait up to 5 s for a clean cloud disconnect.
const unsigned long PARTICLE_CONNECT_MS      = 30000;  // Wait up to 30 s for the cloud connection.
const unsigned long SERIAL_CONNECT_MS        = 8000;   // Wait up to 8 s for the USB serial monitor at boot.
const unsigned long INITIAL_HOLD_POLL_MS     = 2000;   // In the PIC's cold-boot hold: poll the meter every 2 s (spec: ~1-3 s).
const unsigned long STATE_CHANGE_DELAY_MS    = 500;    // A short 0.5 s pause used around state changes.
// "Magic numbers" are unique tags we store in EEPROM to recognize OUR saved data.
const uint32_t      FLOW_CAL_MAGIC           = 0x4643414CUL; // "FCAL"  (the bytes spell FCAL in ASCII).
const uint32_t      CFG_MAGIC                = 0x4C434647UL; // "GFCL"
const uint32_t      PICP_MAGIC               = 0x50494350UL; // "PICP"
// EEPROM is tiny non-volatile storage; these are the byte addresses we save each block at.
const int           FLOW_CAL_EEPROM_ADDR     = 0;      // Flow calibration saved starting at address 0.
const int           CFG_EEPROM_ADDR          = 32;     // App config saved starting at address 32.
const int           PICP_EEPROM_ADDR         = 64;     // PIC params saved starting at address 64.

// =============================================================== IMU plumbing
#if USE_IMU                                    // Everything in here only exists when the IMU is enabled.
#define LSM6DS3_ADDR_A 0x6A                      // One possible I2C address of the IMU chip.
#define LSM6DS3_ADDR_B 0x6B                      // The other possible I2C address.
#define GYRO_CAL_SAMPLES 200                     // Take 200 readings to measure the gyro's resting bias.
Adafruit_LSM6DS33 lsm6ds;                        // The IMU driver object.
Adafruit_Sensor *lsm_temp = nullptr, *lsm_accel = nullptr, *lsm_gyro = nullptr;  // Sub-sensors (start as "none").
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;   // Measured resting offsets to subtract from gyro readings.
#endif                                          // End of IMU-only block.

// ImuData is kept even without a physical IMU: leaking/shutoff/overflow flags
// are used by the core leak/valve logic. Sensor fields stay 0 when USE_IMU=0.
struct ImuData {                                // A bundle of current device status + sensor readings.
  int   sensor = 0;                             // The IMU's I2C address if found, or 0 if absent.
  bool  leaking = false;                        // true while a leak is currently flagged.
  bool  shutoff = false;                        // true while the valve has been shut off.
  bool  overflow = false;                       // true if an overflow condition was seen.
  float temperature = 0;                        // Last temperature reading (stays 0 without an IMU).
  float accelX = 0, accelY = 0, accelZ = 0;     // Last accelerometer readings (X/Y/Z).
  float gyroX = 0,  gyroY = 0,  gyroZ = 0;      // Last gyroscope readings (X/Y/Z).
  float meterFrequency = 0;                     // Last computed flow-sensor frequency (Hz).
  float dailyGallons = 0;                       // Running total of gallons used today (mirror of the retained value).
} imu_data;                                     // Create one global instance named 'imu_data'.

// =============================================================== Meter logger
#pragma pack(push, 1)                           // Tell the compiler: pack the next struct tightly (no padding gaps).
typedef struct {                                // The interval logger: a day's worth of flow readings.
  uint32_t day0_utc_midnight;                   // UTC epoch of the start of this logging WINDOW (set at each roll).
  uint16_t start_slot;                          // Cumulative count of oldest samples dropped by the ring.
  uint16_t count;                               // How many readings are currently stored.
  uint16_t raw[METER_LOG_SLOTS];                // each 0..999 (0.01 GPM units)
                                                //   The readings themselves: up to 1440, each stored as GPM*100.
} MeterLog;                                      // Name this struct type "MeterLog".
#pragma pack(pop)                               // Stop the tight-packing rule (restore normal alignment).

// gMeter and the leak model are LARGE and live in normal RAM (preserved across
// ULTRA_LOW_POWER sleep). They are persisted to LittleFS so they also survive a
// reset/power loss -- this is the P2-safe alternative to putting ~21 KB in the
// 3 KB retained block.
static MeterLog gMeter;                          // The single global logger buffer ("static" = file-private).

struct SlotModel {                               // The statistical model that learns a normal weekly flow pattern.
  static constexpr int SLOTS_PER_DAY  = 24 * 60 * 60 / LEAK_MODEL_INTERVAL_SEC; // 288
                                                //   86400 s/day / 300 s per slot = 288 five-minute slots per day.
  static constexpr int SLOTS_PER_WEEK = SLOTS_PER_DAY * 7;                       // 2016
                                                //   288 * 7 days = 2016 slots covering a full week.
  float   mu[SLOTS_PER_WEEK];                    // mu  = the learned AVERAGE flow for each weekly slot.
  float   var[SLOTS_PER_WEEK];                   // var = the learned VARIANCE (spread) for each slot.
  uint8_t init[SLOTS_PER_WEEK];                  // 1 if a slot has been initialized/learned yet, else 0.
};
static SlotModel model;                          // The single global leak-learning model.

// =============================================================== Retained state
// All small -> comfortably inside the retained block; synced via backupRamSync.
// "retained" variables keep their value across sleep (and, after backupRamSync, resets).
retained float        flowCalScale       = FLOW_CAL_DFLT;   // User's flow calibration scale (default 1.255).
retained AppConfig    appConfig          = {CFG_LEAK_GPM_DFLT, CFG_SHUTOFF_DFLT,    // The 4 host analytics settings,
                                            CFG_AUTOSHUT_DFLT, CFG_ALERTMODE_DFLT}; // initialized to their defaults.
// Cached copy of the PIC's 4 leak parameters (REQ_GET/SET_PARAM payload). The
// host caches what it last wrote so it can re-push after a reset/power loss.
retained PicParams    picParams          = {PIC_LEAK1_COUNTS_DFLT, PIC_LEAK1_WINDOW_DFLT,   // The 4 PIC leak params,
                                            PIC_LEAK2_COUNTS_DFLT, PIC_LEAK2_WINDOW_DFLT};  // initialized to defaults.
retained bool         picParamsDirty     = true;   // SET_PARAM to PIC on next contact
                                                   //   true = we still owe the PIC a fresh write of these params.
retained float        dailyGallons       = 0.0f;   // Total gallons used so far today.
retained float        hourlyData[24]     = {0.0f}; // Leftover "today" totals. Not published (req 5:
                                                   //   the contract array is hourlyGallons[48]).
                                                   //   Bucket width = g_bucketSec (3600 s production).
retained int          lastBucketIngested = -1;     // Index of the last bucket we wrote (-1 = none yet).
retained uint32_t     lastBucketDay      = 0;      // Day counter (local epoch / (24 * bucketSec)).
// End epoch (T_now) of the last batch we ingested. This is T_prev for the next
// batch's equalization (handoff spec 4.2). It MUST survive the power cut between
// sessions, so it is retained (flushed to flash by persistAll -> syncBackupRam).
// V055 kept this in a plain static, which is wiped every time the PIC removes
// power - so equalization silently fell back to the nominal interval on EVERY
// session and never actually equalized anything.
// End of the last span we placed on the time axis, as a LOCAL epoch. This is the
// seam the next report continues from, so it must survive the power cut between
// sessions (retained + flushed to flash by persistAll -> syncBackupRam).
retained uint32_t     prevReportEnd      = 0;

// Appendix F.1-C: prevReportEnd is only trustworthy as an absolute seam once we
// have actually placed a batch on a real cloud-time axis. A device that ran the
// no-cloud build can hold a stale prevReportEnd written by an older firmware
// against a fake 1970/2000 grid; comparing that as a real epoch is what made the
// overlap defence fire falsely and killed the no-time path. This flag is set
// true only on a genuine absolute-time placement, and the overlap/continuity
// logic consults it: when false, prevReportEnd is treated as "no seam recorded".
retained bool         prevReportEndValid = false;

// Appendix H.2-C: a device that ran V062 can carry a prevReportEnd poisoned by the
// old sec 6.7 reconstruction (946656003 / 946656017 - a 2000-01-01-derived value).
// The H.2 code fixes stop new poisoning, but a value already in retained RAM
// survives a firmware update. This schema marker is bumped whenever the seam
// state must be discarded on the first boot of a new build; seamSchemaReset()
// (called at the very start of setup) clears the seam once when it does not match.
#define SEAM_SCHEMA_V063  0x48325343UL   /* "H2SC" - Appendix H.2-C one-time seam wipe */
retained uint32_t     seamSchemaMagic    = 0u;

// V059: the header totals we have ALREADY credited against the PIC's current
// consume-on-ACK mark. The PIC only advances that mark when it sees 0x0B, so a
// retransmitted report carries totals measured from the SAME origin as the one
// we already placed - they are cumulative, not incremental. Subtracting these
// is what makes the second delivery credit only the genuinely new water.
// Cleared whenever the mark is known to have moved (see ingestReport()).
retained uint32_t     prevCreditedImpulses = 0;
retained uint32_t     prevCreditedCaptures = 0;

// ---- V064 P-5: idempotent storage of a retransmitted batch -----------------
// PIC V023 holds an un-ACKed batch and resends it unchanged, preferring it even
// across a session boundary. That removes the loss hole but creates a
// double-counting one: our 0x0B can be lost AFTER we have durably stored the
// batch, and the PIC then legitimately resends data we already hold. The sample
// values are byte-identical, so nothing in the payload can distinguish "the same
// batch again" from "new data that happens to match" - only an identifier can.
//
// This is that identifier, and it must be NON-VOLATILE: the whole failure mode
// spans the power cut between sessions, so a plain static would be wiped exactly
// when it is needed. Kept in retained RAM (flushed by persistAll -> syncBackupRam)
// rather than in the flash ring's RingMeta as rev2 Appendix C suggested - the
// ring only exists when FLASH_BUFFER_BLOCKS > 0, and de-duplication must hold in
// every configuration, including the 0-block one.
retained uint8_t      g_lastStoredBatchSeq      = 0;
retained bool         g_lastStoredBatchSeqValid = false;

// ---- V057 persisted time-axis state ---------------------------------------
// The unfinished bucket left over from the previous report. Without it, every
// report boundary would slice a bucket in half and the volume in that slice
// would be reported against the wrong hour (or lost).
retained HourlyCarry  hourlyCarry        = {HOURLY_CARRY_MAGIC, 0u, 0u, 0u, 0.0f, 0u, 0u};

// ---- V068: the contract's fixed 48-slot sliding window --------------------
// The window state and its arithmetic live in src/roll48.{h,cpp}. They are a
// separate translation unit so tools/hostcheck/roll48_test.cpp can link and
// exercise them directly - the gap accounting is the part most likely to be
// wrong, and it is not reachable from any test that links leaksense.cpp.
// See roll48.h for why this is a third view of the buckets rather than a
// reshaping of hourlyData[24] or of the variable-length series.

// ---- V068: leak-event counting over a publish window ----------------------
// The agreement asks for DAILY ALERT1/ALERT2 counts. They cannot be produced.
// The PIC reports temp_lock_count, a cumulative total with no timestamps, so a
// rise of 8 across a 48 h report cannot be split into day 1 and day 2 - the
// information does not exist on the wire, and no amount of Photon-side
// arithmetic creates it. Rather than fabricate a daily split, the count is
// reported over the window it actually covers, and the window length is
// published beside it as a1WindowSec. Set the report grid to 24 h and
// a1WindowSec becomes 86400, which is the contract reading exactly; at 48 h the
// consequence of the client's own interval choice is visible in the data instead
// of being hidden by a divided-by-two guess.
#define LEAK_EVT_INVALID 0xFFFFFFFFu
retained uint32_t     g_a1LockPrev        = LEAK_EVT_INVALID;  // PIC cumulative count at the last look
retained uint32_t     g_a1EventsWindow    = 0;   // ALERT1 rises accumulated in the open window
retained uint32_t     g_a1WindowStartUtc  = 0;   // when the window opened (0 = not open)
retained uint8_t      g_a1WindowValid     = 0;   // 0 -> publish -1, meaning "could not count"
retained uint8_t      g_shutoffSeen       = 0;   // a lock was observed since the last publish (T/F)

// Local-time offset in seconds, applied in exactly one place: the TIME_SYNC we
// hand the PIC. Everything downstream is already local. Changing it moves the
// whole timebase, so it is only ever applied through the reboot path.
retained int32_t      g_tzOffsetSec      = TZ_OFFSET_SEC_DFLT;

// The PIC's report grid. The PIC has no non-volatile memory, so we hold the
// authoritative copy and write it back after each of its cold boots.
retained PicGridParams g_grid            = {GRID_ANCHOR_SEC_DFLT, GRID_INTERVAL_SEC_PROD};
retained bool         g_gridDirty        = true;   // owe the PIC a grid write

// Set when a cloud command changed the timebase and the system must be restarted
// to adopt it cleanly (addendum A.3). Survives the reboot so the Photon knows,
// on the way back up, that it is the one that asked for this.
retained bool         g_pendingRestart   = false;
retained uint32_t     leakingEventCount  = 0;      // How many leak events happened since the last publish.
retained uint32_t     shutoffEventCount  = 0;      // How many valve-shutoff events since the last publish.
retained uint32_t     overflowEventCount = 0;      // How many overflow events since the last publish.
retained unsigned long nextPublishEpoch  = 0;      // UTC time (seconds) of the next scheduled 24 h cloud upload.
retained uint32_t     nextSampleAtUtc    = 0;      // UTC time of the next expected sampling boundary.
retained bool         triggerState       = false;  // true while water is actively flowing (keeps us awake).
retained unsigned long lastTriggerTime   = 0;      // Timestamp of the most recent flowing sample. Appendix H.9:
                                                   //   this is a LOCAL epoch on the absolute-time path, but the
                                                   //   no-time path (Appendix E.5) stamps a RELATIVE value
                                                   //   (millis()/1000, ~5..185), NOT a UTC epoch. Nothing reads
                                                   //   it as an epoch today; any future reader must gate on the
                                                   //   clock (H.3) before treating it as absolute time.

#if USE_LOCAL_METER                              // Only when the optional local flow sensor is enabled...
retained volatile unsigned long retainedPulseCount = 0;   // Pulse counter updated by an interrupt ("volatile" = can change anytime).
retained unsigned long lastMeterWakeTime = 0;             // Last time we computed flow from the local sensor.
#endif                                           // End of local-meter block.

// =============================================================== Leak detector state
static int   leakRunLen = 0;                     // How many consecutive high-flow readings we have seen.
static float volWin[VOL_WIN] = {0};              // A sliding window of the last few flow readings (for 30-min volume).
static int   volIdx = 0, volCount = 0;           // volIdx = where to write next; volCount = how many are filled.

// =============================================================== Globals
PicLink  picLink;                                // The single PicLink object used to talk to the PIC chip.

// ---- runtime config (defaults = compile-time; overwritten by the PIC at boot) ----
// The PIC is the single source of truth (see PicPhotonCfg / RSP_PHOTON_CFG). We
// start from the compiled defaults so the unit still works if the PIC provides
// nothing or is unreachable, then requestPicConfig() may overwrite these.
struct RuntimeCfg {
  bool     fromPic          = false;                       // did the PIC supply these?
  float    captureIntervalSecF = PIC_SAMPLE_INTERVAL_SEC_F;// rate divisor (seconds)
  uint16_t samplesPerReport = (uint16_t)PIC_SAMPLES_PER_REPORT;
  // Appendix H.13.1: two independent axes.
  //   cadenceFast  - the PIC's "speed" (test 3-min vs production 48-h). Its value
  //                  can be refined by the PIC's RSP_PHOTON_CFG.fastBench bit,
  //                  which is a CADENCE bit only.
  //   cloudEnabled - whether this build uses Wi-Fi/Particle/cloud time. Comes from
  //                  the BUILD_MODE (compile-time CLOUD_ENABLED); the PIC does NOT
  //                  get to override it, so a bench PIC reporting fastBench=0 can
  //                  never drag a no-cloud build into a real connect attempt.
  bool     cadenceFast      = (CADENCE_FAST != 0);        // test cadence / bucket width
  bool     cloudEnabled     = (CLOUD_ENABLED != 0);       // Wi-Fi + Particle + cloud time
#ifdef DEBUG_CDC_DATASERIES
  bool     debugDataseries  = true;
#else
  bool     debugDataseries  = false;
#endif
  uint8_t  missedFillMode   = (uint8_t)PIC_MISSED_FILL;    // 0=ZERO 1=AVERAGE
  uint16_t serialDelayMs    = (uint16_t)BENCH_SERIAL_SEND_DELAY_MS;
  // Appendix G.4: hourlyGallon calculation-verification detail (LOCAL, not on the
  // PIC wire). 0=off, 1=summary, 2=detailed(+per-sample). WARN lines print at any
  // level. Defaults from APP_DEBUG_HOURLY_LEVEL (detailed on bench, summary prod).
  uint8_t  debugHourly      = (uint8_t)APP_DEBUG_HOURLY_LEVEL;
};
RuntimeCfg g_cfg;                                // the live settings the code reads

// Active bucket width for hourlyGallons[], in seconds. Chosen at runtime once
// RSP_PHOTON_CFG has told us whether this is a bench or a cloud session; the
// compile-time BUCKET_SEC is only the pre-handshake default.
static uint32_t g_bucketSec = BUCKET_SEC;

// ---- V057 session time state ----------------------------------------------
// The span a report covers ENDS when the PIC powered us up, because that is the
// instant the PIC froze the batch it is about to send (doc 03 section C.3) -
// not when we happen to publish. We recover that instant from the cloud clock
// minus our own uptime at the moment of the sync:
//     spanEndLocal = (cloud local epoch at sync) - (millis at sync)/1000
// Anything captured while this session is running belongs to the NEXT report.
static uint32_t g_spanEndLocal    = 0;    // local epoch of the PIC wake that started this session
static bool     g_spanEndValid    = false;
static uint32_t g_timeSyncLocal   = 0;    // local epoch we actually handed the PIC
static uint32_t g_timeSyncMs      = 0;    // millis() at that moment
static uint32_t g_cloudReadyMs    = 0;    // millis() when the cloud clock first became valid
static bool     g_picSeriesOk     = false;// a batch was received this session (or a clean zero)
static bool     g_cloudPublishOk  = false;// the report reached the cloud

// ---- V064 P-1/P-3: deferred continuity verdict -----------------------------
// V063 decided "the series is broken" inside the failure branch of a single
// REQ_DATA and acted on it immediately. These three record what happened instead,
// so the decision can be taken once, at session end, with the whole session in
// view. g_picAttemptFailed is retracted by any good reply (P-2 counts n=0 as
// good), which is the entire point: a transaction failure that the session later
// recovers from must leave no trace on the stored data.
//
// All three are plain statics, deliberately. They describe THIS session only and
// must not survive the power cut - a fresh session starts with no accusation
// against the link.
static bool     g_picAttemptFailed = false;   // at least one REQ_DATA failed, not yet retracted
static uint8_t  g_picFailCount     = 0;       // how many failed (for the session summary)
static int      g_picLastErr       = 0;       // last PIC_ERR_* seen (for the session summary)

// ---- Appendix H.13.3 cloud-fault injection + H.15/H.16/H.20 accounting ------
// A retained session counter so CLOUD_FAIL_EVERY_N can pick which sessions fail
// across the power cuts between them. The kind of the injected failure this
// session (connect vs publish) is decided once at session start.
enum CloudFailKind { CLOUD_FAIL_NONE = 0, CLOUD_FAIL_CONNECT = 1, CLOUD_FAIL_PUBLISH = 2 };
retained uint32_t g_sessionCounter   = 0;      // increments once per session (H.13.3 / H.20 #)
static CloudFailKind g_injectFail    = CLOUD_FAIL_NONE;   // this session's injected failure, if any
// Session-level "are we actually online this session". Differs from the build's
// cloud axis (g_cfg.cloudEnabled) only when H.13.3 injects a CONNECT failure into
// a cloud-enabled build: that one session runs offline exactly like a real outage.
static bool     g_cloudOnlineThisSession = false;

// H.15/H.16/H.20: per-session network + publish accounting, reset each session.
static uint32_t g_netConnectMs    = 0;    // ms the cloud connect took (0 = not measured)
static uint32_t g_netPublishMs    = 0;    // cumulative ms spent publishing this session
static uint16_t g_pubAttempted    = 0;    // publish events attempted
static uint16_t g_pubOk           = 0;    // publish events acked
static uint16_t g_pubFailed       = 0;    // publish events failed
static uint32_t g_pubBytes        = 0;    // total payload bytes attempted
static uint16_t g_sessionWarnings = 0;    // WARN lines counted this session (H.20)

// V065 (req 4.병): what this session actually RECEIVED AND DURABLY STORED.
//
// The [SUM] data line used to report g_hourly.samplesUsed, which counts samples
// that reached a BUCKET. Those are different quantities, and whenever bucket
// placement does not happen -- no cloud time, the entire condition of the five
// CLOUD_FAIL sessions -- the second is 0 while the first is 34. The summary
// therefore read "n=0" in the same sessions whose event log read
// "BATCH_STORED_ACKED: n=34".
//
// Not a functional defect: the data was received, stored and ACKed correctly
// every time. But the summary block is what a verifier reads to judge a session,
// and read alone it said the opposite of what happened. Under CLOUD_FAST, where
// cloud-less and cloud-bearing sessions are deliberately mixed, that ambiguity
// is worse than useless.
//
// So both numbers are now reported, and named for what they mean: rx = samples
// stored and ACKed, placed = samples that landed in a bucket. In a healthy
// CLOUD_FAST session they converge; a gap between them is exactly the diagnostic
// signal the checklist wants (items 2, 4, 6).
static uint32_t g_sessionSamplesRx  = 0;  // samples durably stored + ACKed this session
static uint16_t g_sessionBatchesRx  = 0;  // batches (incl. replayed blocks) behind that count
static bool     g_prevTimeValid   = false;// Time.isValid() seen on the previous poll (H.15 0->1 edge)

// Grid values the PIC reports back, for the log and the published echo.
static PicGridParams g_gridFromPic = {0u, 0u};
static bool          g_gridFromPicValid = false;

// The bucket series produced this session, and the running publish accounting.
static HourlyResult  g_hourly;
static bool          g_hourlyValid   = false;
static uint32_t      g_hourlyMakeable = 0;   // MM: buckets the run could have produced
static uint32_t      g_hourlySent     = 0;   // NN: buckets actually published

// ---- Appendix H.3: time acquisition, validity carried WITH the value --------
// The pre-V063 primitives returned a bare uint32_t. Without a cloud sync the
// Particle clock is NOT 0 - it sits at its unsynced 2000-01-01 default
// (UTC 946684800), a value that LOOKS like a plausible epoch and so slipped
// past every downstream check. Appendix F.1-B caught one call site; Appendix
// H.2 found the same class again on the recovery path; H.3 removes the class
// itself. The rule now: an invalid time cannot be expressed AS a value. Every
// reader takes the bool, so validity can never be silently ignored, and no
// accessor ever returns a fabricated epoch - it returns false and writes 0.
//
// clockNowUtc() is the SINGLE place Time.now is read anywhere in the codebase
// (enforced by the check.sh "single clock gate" structural guard). When the
// clock is not valid it yields false / out=0, so 946684800 can never enter the
// data path. "No time" is the default; "time present" is the added branch.
static inline bool clockNowUtc(uint32_t &outUtc) {
  if (!Time.isValid()) { outUtc = 0u; return false; }
  outUtc = (uint32_t)Time.now();
  return true;
}

// Current LOCAL epoch, WITH validity. This is the ONLY place the tz offset is
// applied. Returns false / out=0 when there is no real clock.
static inline bool localNow(uint32_t &outLocal) {
  uint32_t utc;
  if (!clockNowUtc(utc)) { outLocal = 0u; return false; }
  outLocal = (uint32_t)((int64_t)utc + (int64_t)g_tzOffsetSec);
  return true;
}

// Best estimate of "local epoch, right now" once a REAL sync has happened,
// tracked forward with the monotonic millisecond counter so a long publish
// burst cannot drag the numbers around. Validity is carried: a session that
// never synced (g_timeSyncLocal == 0) returns false / out=0 - it does NOT fall
// back to a bare Time.now read. That silent fallback was exactly the Appendix
// H.2 leak on replayBufferedReports(). The "time present" state is the ADDED
// path; "no time" is the default (Appendix H.3.2, point 4).
static inline bool localNowTracked(uint32_t &outLocal) {
  if (g_timeSyncLocal != 0u) {
    outLocal = g_timeSyncLocal + (uint32_t)((millis() - g_timeSyncMs) / 1000u);
    return true;
  }
  return localNow(outLocal);   // no session sync yet -> valid only if the clock itself is
}
static PicSample picBuf[PIC_MAX_SAMPLES];        // ~4 KB scratch
                                                //   A reusable buffer to hold decoded PIC samples (up to 1000).
static PicValve  lastValve = {0, 0, 0, 0, 0};    // last RSP_VALVE seen (for publish)
                                                //   Remember the most recent valve status so we can report it.
static bool      haveValve = false;              // true once we have successfully read valve status at least once.

// Power-gating session phases. The PIC powers us only when a report is due; we
// connect, report once, then ask the PIC to cut our power (PKT_PHOTON_OFF_REQ).
//   STARTUP/CONNECTING : bring up the cloud (bounded by TIMEOUT_CANNOT_FIND_CLOUD_MS)
//   MONITORING         : connected -> pull PIC data, publish, do other business
//   SESSION_DONE       : 0x07 sent; idle until the PIC removes our power
enum SystemState { STATE_STARTUP, STATE_CONNECTING, STATE_INITIAL_HOLD, STATE_MONITORING, STATE_SESSION_DONE };
SystemState currentState = STATE_STARTUP;        // We begin in the STARTUP phase.
bool initialHold = false;                        // True if the PIC reported its initial cold-boot power-hold at boot.

bool          lastButtonState = false;           // The button's state on the previous loop (to detect a new press).
unsigned long lastPressTime   = 0;               // When the button was last accepted as pressed (for debounce).
volatile bool resetShutoff    = false;           // Set by a timer to request auto-clearing the shutoff ("volatile" = set in a callback).
volatile bool triggerPublish  = false;           // Set anywhere to request a cloud publish soon.

unsigned long sleepStart    = 0;                 // When the current awake window started.
unsigned long lastWakeTime  = 0;                 // When we last woke from sleep.
int           sleepCycleCount = 0;               // How many sleep cycles we have done (used by the awake-window logic).

// Persistence file paths (LittleFS, /usr/ is user space on P2/Gen3)
static const char *GMETER_PATH = "/usr/gmeter.dat";   // File where we save the gMeter logger buffer.
static const char *BENCHTIME_PATH = "/usr/benchtime.dat";   // FAST_BENCH_TEST: persisted virtual clock.
static const char *MODEL_PATH  = "/usr/leakmodel.dat";// File where we save the leak model.
static const uint32_t MODEL_FILE_MAGIC = 0x4C4D444CUL; // "LMDL"
                                                //   Tag at the start of the model file so we know it's valid/ours.

// =============================================================== Prototypes
// A "prototype" announces a function's name + arguments before it is defined,
// so functions can call each other regardless of the order they appear below.
void changeState(SystemState s);                 // Switch the device's current phase.
void calibrateGyroscope();                       // Measure the gyro's resting bias (IMU only).
int  imuInit();                                  // Initialize the IMU (or a no-op stub if disabled).
int  imuGet();                                   // Read the latest IMU values.
void imuPrint();                                 // Log a one-line IMU/status summary.
void imuPublish();                               // Build and publish the main "sensorData" JSON to the cloud.

void loadFlowCal();   void saveFlowCal();        // Load / save the flow calibration to EEPROM.
void loadConfig();    void saveConfig();         // Load / save the host config to EEPROM.
void loadPicParams(); void savePicParams();      // Load / save the cached PIC params to EEPROM.
bool saveBlob(const char *path, const void *data, size_t len);   // Save raw bytes to a file.
bool loadBlob(const char *path, void *data, size_t len);         // Load raw bytes from a file.
void persistAll();    void restorePersisted();   // Save / load the big RAM buffers (gMeter + model) to flash.
void syncBackupRam();                            // Flush retained RAM to flash (P2 needs this explicitly).

static float freqToGpm(float freq);              // Convert a flow-sensor frequency (Hz) into gallons-per-minute.
static bool ingestReport(const PicSample *s, uint32_t n, const PicReportInfo &info,
                         uint32_t endLocal, bool haveAbsTime, const char *tag,
                         bool reconstructed = false);   // Place one report on the time axis.
void serviceMeterFromPic(bool picInitiated);     // Pull and process data from the PIC.
void picKeepalivePump();                         // Send KEEPALIVE if the PIC link has been idle too long.
void appendIntervalSample(float gpm);            // Store one reading in the interval logger.
bool senseLeak(uint32_t tsUtc, float gpm);       // Decide whether the current reading indicates a leak.
void onLeakDetected();                           // React to a detected leak (count, maybe shut off, maybe alert).

int  shutoffSwitch(String cmd);                  // Cloud function: control the local valve (close/open/off).
int  leakingSwitch(String cmd);                  // Cloud function: manually set/clear the leaking flag.
int  setFlowCal(String cmd);  int getFlowCal(String cmd);   // Cloud functions: set/get flow calibration.
int  setConfig(String cmd);   int getConfig(String cmd);    // Cloud functions: set/get host config.
int  setLeakParams(String cmd); int getLeakParams(String cmd);  // Cloud functions: set/get PIC leak params.
int  getValve(String cmd);    int unlockValve(String cmd);  // Cloud functions: read valve / clear valve lock.
int  picReset(String cmd);                       // Cloud function: reset the PIC.
int  setTzOffset(String cmd);                    // Cloud function: change the local time offset (restarts).
int  setGrid(String cmd);                        // Cloud function: change anchor/interval (restarts).
int  getGrid(String cmd);                        // Cloud function: log the grid + offset in use.
int  syncPic(String cmd);                        // Cloud function: force-push cached PIC params now.
bool pushPicParams();                            // Send the cached PIC params to the PIC; return true on ACK.
void readValveStatus();                          // Read and remember the PIC valve status.
void publishIntervalDataChunks();                // Publish the interval logger to the cloud in chunks.
void restartSleepTimer(const char *reason);      // Reset the awake window (called whenever activity happens).
void runSleep();                                 // No-op under power-gating (kept for reference/compat).
void endSession();                               // Finish the session: go idle and let the PIC cut power.
#if USE_WIFI                                     // Only on Wi-Fi boards...
int  setWiFi(String cmd);  int clearWiFi(String cmd);   // Cloud functions: set/clear Wi-Fi credentials.
#endif                                           // End Wi-Fi-only prototypes.

// Interrupt handler: counts one pulse from the local flow sensor (if enabled).
void countPulse() {
#if USE_LOCAL_METER                              // Only do anything if the local meter is enabled...
  retainedPulseCount++;                          // ...add one to the pulse counter (runs in an interrupt context).
#endif
}

// =============================================================== Helpers
// Change which phase the state machine is in.
void changeState(SystemState s) { currentState = s; }   // Just store the new state.

// On P2, retained RAM is only written to flash when we explicitly ask. Do that here.
void syncBackupRam() {
#if PLATFORM_ID == PLATFORM_P2                   // Only the P2 board needs this manual flush...
  System.backupRamSync();   // flush retained RAM to flash (P2 needs this)
#endif                                           // (Gen3 boards persist retained memory automatically.)
}

// Convert a flow-sensor frequency (in Hz) into a water-flow rate (gallons per
// minute). The polynomial itself now lives in flow_cal.h so the host tests can
// exercise it (Appendix G.6.3); the maths is identical to before. See Appendix
// G.1.1 for the known high-frequency collapse this preserves.
static float freqToGpm(float freq) {
  return flowFreqToGpm(freq, flowCalScale);
}

// The calibration's usable range, computed once from the live coefficients and
// scale (Appendix G.3.1). Cached so the ~13000-step scan runs at most once.
static FlowValidRange g_flowRange       = {0.0f, 0.0f, 0.0f, false};
static bool           g_flowRangeValid  = false;
static float          g_flowRangeScale  = 0.0f;
static const FlowValidRange &flowRange() {
  if (!g_flowRangeValid || g_flowRangeScale != flowCalScale) {
    g_flowRange      = flowComputeValidRange(flowCalScale);
    g_flowRangeScale = flowCalScale;
    g_flowRangeValid = true;
  }
  return g_flowRange;
}

// Appendix H.10.5 (1) - INTERIM measure, coefficients UNCHANGED (Appendix H.10).
// Above the usable (monotonic) peak the polynomial first falls and then collapses
// to 0, so a real over-peak flow reads as 0 GPM - "max flow -> 0 gallons", which
// no interpretation can defend. Until the bucket-fill test (H.10.4) settles the
// curve, clamp an over-peak result to the peak GPM instead of letting it fall to
// 0: that converts a silent LOSS into an explicit LOWER BOUND. This touches only
// the application wrapper; flow_cal.h's pure polynomial (and the G.6.3 collapse
// tests) are untouched. Flip FLOW_CLAMP_ABOVE_PEAK to 0 to restore the raw curve
// once the client confirms the intended domain. 'clamped' reports whether the
// interim clamp fired, so the caller can emit the required interim WARN once.
static float freqToGpmUsable(float freq, bool *clamped) {
  if (clamped) *clamped = false;
  float gpm = freqToGpm(freq);
#if FLOW_CLAMP_ABOVE_PEAK
  const FlowValidRange &r = flowRange();
  if (r.peakFreqHz > 0.0f && freq > r.peakFreqHz) {
    if (clamped) *clamped = true;
    return r.peakGpm;               // lower bound; the true flow is higher
  }
#endif
  return gpm;
}

// Frequency (Hz) a sample represents at a given equalized step, for the
// out-of-calibration checks in the verification log (Appendix G.3.3).
static inline float sampleFreqHz(uint16_t pulses, float dtSec) {
  return (dtSec > 0.0f) ? ((float)pulses / dtSec) : 0.0f;
}

// ---- Sample -> gallons ------------------------------------------------------
// Handed to the binning engine so the calibration lives in exactly one place.
// The rate is computed from the sample's own EQUALIZED duration, which is what
// removes the PIC's count-quantization error from the flow figure.
//
// NOTE (carried over from V056, open question A - still unresolved): FLOW_C4
// divides the VOLUME but not the RATE returned by freqToGpm(). If freqToGpm()
// truly returns gallons per minute, then every gallon figure is 27 % below the
// published GPM series. Deliberately left as-is: changing it moves every
// historical total, so it must be settled by a measured bucket-fill test first.
static float sampleToGallons(uint16_t pulses, float dtSec) {
  if (dtSec <= 0.0f) return 0.0f;
  float freq = (float)pulses / dtSec;
  // Appendix H.10.5 (1): use the usable-clamped rate so an over-peak flow becomes
  // a lower-bound gallon figure instead of a silent 0 (coefficients unchanged).
  float gpm  = freqToGpmUsable(freq, nullptr);
  return gpm * (dtSec / 60.0f) / FLOW_C4;
}

// Legacy 24-slot rolling view, kept for the existing dashboard schema. It is a
// DERIVED artefact now: the authoritative output is the variable-length bucket
// list in g_hourly. Feeding it from the finished buckets means the two can never
// disagree about a total.
static void legacyRollingApply(const HourlyResult &h) {
  for (uint16_t i = 0; i < h.count; i++) {
    uint32_t binStart = h.baseLocal + (uint32_t)i * h.bucketSec;
    uint32_t dayLen   = (uint32_t)BUCKET_COUNT * h.bucketSec;
    uint32_t day      = binStart / dayLen;
    int      idx      = (int)((binStart / h.bucketSec) % BUCKET_COUNT);

    if (lastBucketIngested < 0) {                 // very first bucket we ever saw
      lastBucketIngested = idx;
      lastBucketDay      = day;
    } else if (day != lastBucketDay) {            // a whole day of buckets has passed
      Log.info("[DAT] DAY_ROLLOVER: daily=%.2f gal (bucket=%us, day %lu -> %lu); "
               "hourlyData[24] is NOT zeroed (req 5: roll, do not reset)",
               dailyGallons, (unsigned)h.bucketSec,
               (unsigned long)lastBucketDay, (unsigned long)day);
      dailyGallons       = 0.0f;
      lastBucketIngested = idx;
      lastBucketDay      = day;
    } else if (idx != lastBucketIngested) {
      lastBucketIngested = idx;
    }
    hourlyData[idx] += h.gal[i];
    dailyGallons    += h.gal[i];
  }
}

// Round to a chosen number of decimals. The publish path sets the JSON writer's
// float places as well; this keeps the LOGGED value identical to the PUBLISHED
// one, so a capture and a cloud record can be compared digit for digit.
static float roundDecimals(float v, uint8_t places) {
  float m = 1.0f;
  for (uint8_t i = 0; i < places; i++) m *= 10.0f;
  return floorf(v * m + 0.5f) / m;
}

// ---- V068: ALERT1 / ALERT2 / shutoff over the publish window ---------------
//
// Every branch below corresponds to a state seen on the bench, not a
// hypothetical. The purpose of all of them is one distinction: "no leak
// happened" (0) must never be published where the truth is "we could not count"
// (-1). Those are different facts and a dashboard that draws them the same way
// is wrong in the direction that matters.
static void leakEventsUpdate(uint32_t picTempLocks, bool timeValid, uint32_t nowUtc) {
  if (g_a1LockPrev == LEAK_EVT_INVALID) {
    // No baseline: this is the first read after a flash, or after retained RAM
    // was lost. The PIC's counter is a LIFETIME total, so counting it now would
    // report its entire history as having happened in this window.
    g_a1LockPrev       = picTempLocks;
    g_a1EventsWindow   = 0;
    g_a1WindowStartUtc = timeValid ? nowUtc : 0u;
    g_a1WindowValid    = timeValid ? 1u : 0u;
    return;
  }

  uint32_t delta;
  if (picTempLocks >= g_a1LockPrev) {
    delta = picTempLocks - g_a1LockPrev;
  } else {
    // The PIC rebooted and its counter restarted. Whatever it counted between
    // our last read and its reset is unrecoverable - it was never sent.
    delta = picTempLocks;
    g_a1WindowValid = 0u;
    Log.warn("[LEAK] PIC ALERT1 counter went backwards (%lu -> %lu): the PIC rebooted. "
             "Events between our last read and that reset are unrecoverable, so this "
             "window publishes a1Events=-1 rather than an undercount.",
             (unsigned long)g_a1LockPrev, (unsigned long)picTempLocks);
  }
  g_a1LockPrev      = picTempLocks;
  g_a1EventsWindow += delta;

  if (!timeValid) {
    // We can still count the events, but not state the period they cover, and a
    // count without its window is exactly the ambiguity a1WindowSec exists to
    // remove. Observed on a real 104 s no-cloud session.
    g_a1WindowValid = 0u;
    return;
  }
  if (g_a1WindowStartUtc == 0u) {
    // The window was never opened (previous session had no clock). Anything
    // accumulated belongs to an unmeasurable period; drop it rather than credit
    // it to a window it did not occur in.
    g_a1WindowStartUtc = nowUtc;
    g_a1EventsWindow   = 0;
    g_a1WindowValid    = 1u;
  }
}

// Close the window and open the next one. Called once, straight after the
// publish, so the numbers published and the numbers reset are the same numbers.
static void leakEventsAfterPublish(bool timeValid, uint32_t nowUtc) {
  g_a1EventsWindow   = 0;
  g_shutoffSeen      = 0u;
  g_a1WindowStartUtc = timeValid ? nowUtc : 0u;
  g_a1WindowValid    = timeValid ? 1u : 0u;
}

void picKeepalivePump() {
  if ((uint32_t)(millis() - picLink.lastTxMillis()) >= PIC_BUSY_KEEPALIVE_MS) {
    picLink.sendKeepalive();
  }
}

// =============================================================== Persistence
// Save 'len' bytes from 'data' into the file at 'path'. Returns true on success.
bool saveBlob(const char *path, const void *data, size_t len) {
  int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);   // Open for writing; create if missing; empty it first.
  if (fd < 0) return false;                      // A negative file descriptor means the open failed.
  int w = write(fd, data, len);                  // Write the bytes; 'w' = how many were actually written.
  close(fd);                                     // Always close the file when done.
  return (w == (int)len);                        // Success only if we wrote exactly the requested number of bytes.
}

// Load 'len' bytes from the file at 'path' into 'data'. Returns true on success.
bool loadBlob(const char *path, void *data, size_t len) {
  int fd = open(path, O_RDONLY);                 // Open the file for reading only.
  if (fd < 0) return false;                      // Open failed (e.g. file does not exist).
  int r = read(fd, data, len);                   // Read up to 'len' bytes; 'r' = how many were read.
  close(fd);                                     // Close the file.
  return (r == (int)len);                        // Success only if we read exactly 'len' bytes.
}

// Save the big in-RAM buffers (gMeter + leak model) to flash so they survive a reset.
void persistAll() {
  saveBlob(GMETER_PATH, &gMeter, sizeof(gMeter));   // Write the whole gMeter struct to its file.
  // Model file: 4-byte magic header + payload, for validity checking.
  int fd = open(MODEL_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0666);   // Open/create/empty the model file.
  if (fd >= 0) {                                  // If it opened successfully...
    write(fd, &MODEL_FILE_MAGIC, sizeof(MODEL_FILE_MAGIC));   // First write the 4-byte "this is a valid model" tag.
    write(fd, &model, sizeof(model));             // Then write the whole model struct.
    close(fd);                                    // Close the file.
  }
  syncBackupRam();                                // Also flush retained RAM to flash (P2).
  Log.info("PERSIST: gMeter(%u) + model saved", (unsigned)gMeter.count);   // Log how many samples were saved.
}

// Load gMeter + the leak model back from flash at boot (or start fresh if missing/invalid).
void restorePersisted() {
  uint32_t       now    = 0u;                     // The clock is NOT synced yet at this point in a cloud boot.
  const bool     timeOk = clockNowUtc(now);       // H.3: the one checked gate; now stays 0 when invalid.

  // IMPORTANT: the "not in the future" test may only run when the clock is
  // actually valid. restorePersisted() executes before the cloud time sync, so
  // V055 compared a real stored epoch against an unsynced (near-zero) clock,
  // decided the file was corrupt, and wiped the interval logger on EVERY boot -
  // i.e. on every power-gated session.
  // ==== V064 P-6: a persisted time anchor from the forbidden band ============
  // The bench capture shows "day0Utc":946684805 in a published payload - the
  // 2000-01-01 default plus 5 s, matching intervalSec=5. Where it comes from
  // matters more than the symptom: clockNowUtc() is the single clock gate and
  // check.sh enforces that the clock is read nowhere else, so NO path in this
  // build can write that value. What can, is an OLDER build - and this blob
  // outlives a firmware update.
  //
  // The door is right here. restorePersisted() runs BEFORE the cloud sync, so
  // timeOk is false and the "not in the future" test below is skipped by design
  // (that skip fixed a V055 bug where every boot wiped the logger). A poisoned
  // anchor therefore passes validation on every boot and is never overwritten
  // unless gMeter.count > 0 at the next publish. So it persists indefinitely.
  //
  // Reject it explicitly here rather than sanitising the value at the point it is
  // printed: covering the output would remove the symptom and leave the cause in
  // place, still feeding anything that later reads the anchor. Same shape as the
  // Appendix H.2-C one-time seam wipe.
  bool day0Poisoned = false;

  bool gOk = loadBlob(GMETER_PATH, &gMeter, sizeof(gMeter)) &&   // Try to load gMeter, AND check it looks valid:
             gMeter.count <= METER_LOG_SLOTS &&                   //   count must be within range,
             gMeter.day0_utc_midnight != 0 &&                     //   the window start must be set,
             !(day0Poisoned = epochInForbiddenBand(gMeter.day0_utc_midnight)) &&
                                                                  //   V064 P-6: and not a fake 2000-01-01 epoch,
             (!timeOk || gMeter.day0_utc_midnight <= now);        //   and (if we can tell) not be in the future.
  if (day0Poisoned) {
    Log.error("[DAT] persisted day0_utc_midnight=%lu is inside the forbidden "
              "2000-01-01 band - written by an older build, never a real "
              "measurement time. Discarding the interval-logger window.",
              (unsigned long)gMeter.day0_utc_midnight);
  }
  if (!gOk) {                                     // If loading failed or the data looked wrong...
    // 1 is a deliberate "window started, clock was unset" sentinel - distinct
    // from 0 ("no window") and outside the forbidden band, so it can never be
    // mistaken for an absolute epoch by the check above.
    gMeter.day0_utc_midnight = timeOk ? now : 1u;
    gMeter.start_slot = 0;                        // Reset the dropped-sample counter.
    gMeter.count = 0;                             // Reset the sample count to empty.
    Log.info("PERSIST: gMeter fresh (timeValid=%d)", (int)timeOk);
  } else {
    Log.info("PERSIST: gMeter restored (%u samples)", (unsigned)gMeter.count);   // Note how many we recovered.
  }

  int fd = open(MODEL_PATH, O_RDONLY);            // Open the leak-model file for reading.
  bool mOk = false;                               // Assume failure until proven otherwise.
  if (fd >= 0) {                                  // If the file opened...
    uint32_t magic = 0;                           // Place to read the leading magic tag.
    if (read(fd, &magic, sizeof(magic)) == (int)sizeof(magic) &&   // Read the tag, and if it came through fully,
        magic == MODEL_FILE_MAGIC &&                                // it matches OUR tag,
        read(fd, &model, sizeof(model)) == (int)sizeof(model)) {    // and the full model loaded too...
      mOk = true;                                 // ...then the model is valid.
    }
    close(fd);                                    // Close the file either way.
  }
  if (!mOk) {                                     // If the model was missing or invalid...
    memset(&model, 0, sizeof(model));             // ...zero out the whole model (learning restarts from scratch).
    Log.info("PERSIST: leak model fresh (1-week learning restarts)");   // Note the fresh start.
  } else {
    Log.info("PERSIST: leak model restored");     // Note that we recovered the learned model.
  }
}

// =============================================================== EEPROM stores
// Small wrapper structs that pair a magic tag with the data, so we can verify on load.
struct FlowCalStore { uint32_t magic; float scale; };       // For the flow calibration value.
struct CfgStore     { uint32_t magic; AppConfig cfg; };     // For the host config block.

// Load the flow calibration from EEPROM, or use the default if none/invalid is stored.
void loadFlowCal() {
  FlowCalStore s = {0, 0};                        // Start with an empty store (magic 0, scale 0).
  EEPROM.get(FLOW_CAL_EEPROM_ADDR, s);            // Read whatever is stored at the flow-cal address into 's'.
  if (s.magic == FLOW_CAL_MAGIC && s.scale == s.scale &&   // Valid only if the tag matches AND scale is not NaN
      s.scale >= FLOW_CAL_MIN && s.scale <= FLOW_CAL_MAX) { // (s.scale==s.scale is false for NaN) AND in range.
    flowCalScale = s.scale;                       // Use the stored calibration value.
  } else {
    flowCalScale = FLOW_CAL_DFLT;                 // Otherwise fall back to the default.
    saveFlowCal();                                // And write that default back so it's there next time.
  }
  Log.info("FLOW_CAL: %.4f", flowCalScale);       // Log the calibration in use.
}
// Save the current flow calibration value (with its magic tag) to EEPROM.
void saveFlowCal() {
  FlowCalStore s = {FLOW_CAL_MAGIC, flowCalScale};   // Bundle the tag + current scale.
  EEPROM.put(FLOW_CAL_EEPROM_ADDR, s);               // Write the bundle to EEPROM.
}

// Load the host config from EEPROM, or use defaults if none is stored.
void loadConfig() {
  CfgStore s;                                     // Place to read the stored config into.
  EEPROM.get(CFG_EEPROM_ADDR, s);                 // Read whatever is at the config address.
  if (s.magic == CFG_MAGIC) {                     // If the tag matches, the data is ours/valid...
    appConfig = s.cfg;                            // ...so use it.
  } else {
    appConfig = {CFG_LEAK_GPM_DFLT, CFG_SHUTOFF_DFLT,    // Otherwise initialize the config from defaults...
                 CFG_AUTOSHUT_DFLT, CFG_ALERTMODE_DFLT};
    saveConfig();                                 // ...and save those defaults for next boot.
  }
  Log.info("CFG: leak=%.2f shutoffVol=%.1f auto=%u alert=%u",   // Log the active config values.
           appConfig.leakThreshGpm, appConfig.shutoffVolGal,
           appConfig.autoShutoff, appConfig.alertMode);
}
// Save the current host config (with its magic tag) to EEPROM.
void saveConfig() {
  CfgStore s = {CFG_MAGIC, appConfig};            // Bundle the tag + current config.
  EEPROM.put(CFG_EEPROM_ADDR, s);                 // Write it to EEPROM.
}

struct PicpStore { uint32_t magic; PicParams p; };   // Wrapper pairing a tag with the cached PIC params.

// Load the cached PIC leak parameters from EEPROM, or use defaults if none stored.
void loadPicParams() {
  PicpStore s;                                    // Place to read the stored params into.
  EEPROM.get(PICP_EEPROM_ADDR, s);                // Read whatever is at the PIC-params address.
  if (s.magic == PICP_MAGIC) {                    // If the tag matches...
    picParams = s.p;                              // ...use the stored params.
  } else {
    picParams = {PIC_LEAK1_COUNTS_DFLT, PIC_LEAK1_WINDOW_DFLT,   // Otherwise initialize from defaults...
                 PIC_LEAK2_COUNTS_DFLT, PIC_LEAK2_WINDOW_DFLT};
    savePicParams();                              // ...and save them.
  }
  Log.info("PICP: l1=%u/%us l2=%u/%us",           // Log the active PIC params.
           picParams.leak1_counts, picParams.leak1_window_s,
           picParams.leak2_counts, picParams.leak2_window_s);
}
// Save the current cached PIC params (with tag) to EEPROM.
void savePicParams() {
  PicpStore s = {PICP_MAGIC, picParams};          // Bundle tag + current params.
  EEPROM.put(PICP_EEPROM_ADDR, s);                // Write to EEPROM.
}

// =============================================================== Leak model
// Given a UTC timestamp, compute which of the 2016 weekly 5-minute slots it falls in.
static inline int slotOfWeekFromUtc(uint32_t tsUtc) {   // "inline" hints the compiler to insert this directly (fast).
  uint32_t days = tsUtc / 86400UL;                // How many whole days since the Unix epoch.
  int dow = (int)((days + 4UL) % 7UL);            // Sun=0
                                                  //   Day-of-week (the +4 aligns the epoch so Sunday=0).
  int slotInDay = (int)((tsUtc % 86400UL) / LEAK_MODEL_INTERVAL_SEC);   // 0..287
                                                  //   Which 5-minute slot within the day (seconds-into-day / 300).
  return dow * SlotModel::SLOTS_PER_DAY + slotInDay;   // 0..2015
                                                  //   Combine into a single weekly slot index.
}

// Update the learned average (mu) and spread (var) for slot 's' with a new value 'x'.
static inline void updateBaseline(int s, float x) {
  if (s < 0 || s >= SlotModel::SLOTS_PER_WEEK) return;   // Safety: ignore an out-of-range slot index.
  if (!model.init[s]) {                           // If this slot has never been learned before...
    model.mu[s] = x; model.var[s] = 0.05f * 0.05f; model.init[s] = 1; return;   // ...seed it with x and a small variance.
  }
  float mu = model.mu[s];                         // Current learned average for this slot.
  float err = x - mu;                             // How far the new value is from that average.
  mu += ALPHA * err;                              // Nudge the average toward the new value (EWMA update).
  float var = (1.0f - BETA) * model.var[s] + BETA * (err * err);   // Update the variance the same smoothed way.
  float minVar = EPS * EPS;                       // A tiny floor for variance so it never reaches zero.
  if (var < minVar) var = minVar;                 // Apply that floor.
  model.mu[s] = mu; model.var[s] = var;           // Store the updated average and variance back.
}

// Return the standard deviation (sigma) for slot 's' = square root of its variance.
static inline float sigmaOf(int s) {
  float v = model.var[s];                         // The slot's variance.
  if (v < EPS * EPS) v = EPS * EPS;               // Keep it at least the tiny floor.
  return sqrtf(v);                                // sigma = sqrt(variance).
}

// Convert a flow rate (GPM) into gallons used during one 5-minute model slot.
static inline float gallonsIn5min(float gpm) {
  return gpm * (LEAK_MODEL_INTERVAL_SEC / 60.0f);   // GPM * (300 s / 60 s-per-min) = GPM * 5 minutes.
}

// Appendix H.19: the last leak-model evaluation, captured so a per-report [LEAK]
// line can show what the Photon's GPM-based detector is doing (to compare with
// the PIC's pulse-based verdict). Updated on every senseLeak() call.
static float    g_leakLastGpm   = 0.0f;
static float    g_leakLastMu    = 0.0f;
static float    g_leakLastSigma = 0.0f;
static float    g_leakLastZ     = 0.0f;
static float    g_leakLastThr   = 0.0f;
static bool     g_leakLastHigh  = false;
static bool     g_leakLastVerdict = false;

// Decide whether the reading 'gpm' at time 'tsUtc' should be treated as a leak.
bool senseLeak(uint32_t tsUtc, float gpm) {
  int s = slotOfWeekFromUtc(tsUtc);               // Which weekly slot this reading belongs to.

  if (!model.init[s]) { updateBaseline(s, gpm); g_leakLastGpm = gpm; g_leakLastMu = gpm;
                        g_leakLastSigma = 0.0f; g_leakLastZ = 0.0f; g_leakLastHigh = false;
                        g_leakLastVerdict = false; return false; }   // Slot not learned yet.

  float mu = model.mu[s];                         // Learned average flow for this slot.
  float sig = sigmaOf(s);                         // Learned spread (sigma) for this slot.

  // Condition A: persistent high flow (EWMA) OR absolute cloud threshold.
  float thresh = mu + SIGMA_MULT * sig;           // Statistical alarm level = average + 4*sigma.
  if (thresh < ABS_GPM_MIN) thresh = ABS_GPM_MIN; // Never let the threshold fall below the absolute floor.
  bool highNow = (gpm > thresh) || (gpm > appConfig.leakThreshGpm); // cloud var (1)
                                                  //   "High" if above the statistical OR the user-set threshold.
  leakRunLen = highNow ? (leakRunLen + 1) : 0;    // Count consecutive highs; reset to 0 the moment it drops.

  // Condition B: 30-min volume over the cloud-set threshold (cloud var 2).
  volWin[volIdx] = gpm;                           // Store this reading in the sliding volume window.
  volIdx = (volIdx + 1) % VOL_WIN;                // Advance the write position, wrapping around the window.
  if (volCount < VOL_WIN) volCount++;             // Track how many slots of the window are filled (up to VOL_WIN).
  float gal30 = 0.0f;                             // Accumulator for total gallons over the window.
  for (int i = 0; i < volCount; i++) gal30 += gallonsIn5min(volWin[i]);   // Sum gallons across all filled slots.
  bool volTrip = (volCount == VOL_WIN) && (gal30 > appConfig.shutoffVolGal);   // Trip only when full AND over the limit.

  bool leakNow = (leakRunLen >= N_CONSEC) || volTrip;   // It's a leak if enough consecutive highs OR the volume tripped.
  if (!leakNow) updateBaseline(s, gpm);           // only learn when not leaking

  // Appendix H.19: snapshot this evaluation for the per-report [LEAK] line.
  g_leakLastGpm = gpm; g_leakLastMu = mu; g_leakLastSigma = sig;
  g_leakLastZ = (sig > 1e-6f) ? ((gpm - mu) / sig) : 0.0f;
  g_leakLastThr = thresh; g_leakLastHigh = highNow; g_leakLastVerdict = leakNow;
  return leakNow;                                 // Report the leak decision.
}

// Appendix H.19: one [LEAK] block per report (level 3) exposing the Photon's
// statistical model, so its GPM-based verdict can be compared against the PIC's
// pulse-based one. Nothing here changes the detector - it only reports it.
static void logLeakModel() {
  if (g_cfg.debugHourly < 3u) return;
  Log.info("[LEAK] model: mean=%.4f sigma=%.4f z=%.2f (alpha=%.2f beta=%.2f)",
           (double)g_leakLastMu, (double)g_leakLastSigma, (double)g_leakLastZ,
           (double)ALPHA, (double)BETA);
  Log.info("[LEAK] model: flowing=%d lastFlowTs=%lu runLen=%d",
           (int)triggerState, (unsigned long)lastTriggerTime, (int)leakRunLen);
  Log.info("[LEAK] model: verdict=%s (threshGpm=%.4f, cfgLeakGpm=%.2f, lastGpm=%.4f)",
           g_leakLastVerdict ? "leak" : "none", (double)g_leakLastThr,
           (double)appConfig.leakThreshGpm, (double)g_leakLastGpm);
}

// =============================================================== PIC ingest
// ---- Appendix E.4 verification-log helpers ---------------------------------
// The goal of these logs is that a bench run can be VERIFIED from the capture
// alone: the numbers printed on one line cross-check against another, without
// re-running the arithmetic or reading the source.

// (a) Received-data summary: sum, min/max, and the first/last few samples.
// The sum must equal the PIC's total_impulses when the ring did not overrun;
// when it does not, the difference is exactly the missed-fill amount (e).
static void logSampleSummary(const PicSample *s, uint32_t n) {
  if (n == 0u) { Log.info("[DAT] samples n=0 (empty batch)"); return; }
  uint32_t sum = 0u;
  uint16_t mn  = 0xFFFFu, mx = 0u;
  for (uint32_t k = 0; k < n; k++) {
    uint16_t v = s[k].pulses;
    sum += v;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  // First up-to-4 and last up-to-3, without overlapping when the batch is tiny.
  char fbuf[48]; int fp = 0;
  uint32_t fShow = (n < 4u) ? n : 4u;
  for (uint32_t k = 0; k < fShow; k++)
    fp += snprintf(fbuf + fp, sizeof(fbuf) - fp, (k == 0u) ? "%u" : " %u", (unsigned)s[k].pulses);
  char lbuf[40]; int lp = 0;
  uint32_t lShow = (n < 3u) ? n : 3u;
  uint32_t lStart = (n > lShow) ? (n - lShow) : 0u;
  for (uint32_t k = lStart; k < n; k++)
    lp += snprintf(lbuf + lp, sizeof(lbuf) - lp, (k == lStart) ? "%u" : " %u", (unsigned)s[k].pulses);
  Log.info("[DAT] samples n=%lu sum=%lu min=%u max=%u first=[%s] last=[%s]",
           (unsigned long)n, (unsigned long)sum, (unsigned)mn, (unsigned)mx, fbuf, lbuf);
}

// (b) Equalization result, in the PIC's own millisecond units so perStep can be
// checked against nominal by eye. Prints whether the measured step was accepted
// or replaced by the nominal interval (the 0.25x..4x trust gate in hourly.cpp).
//
// Appendix H.4: for a reconstructed no-time span the perStep IS the nominal
// (millisecond) interval and the honest span is n x nominal, so both are printed
// from the interval and the interval source is named (block header sampleIntervalMs
// / RSP_PHOTON_CFG captureIntervalMs / integer-second constant).
static void logEqualize(const char *tag, uint32_t n, uint32_t spanStartLocal,
                        uint32_t spanEndLocal, float perSampleSec,
                        uint32_t nominalMs, bool equalized,
                        bool reconstructed = false, const char *stepSource = nullptr) {
  double   perStepMs = (double)perSampleSec * 1000.0;
  uint32_t spanMs;
  if (reconstructed) {
    // The whole-second epoch endpoints are a rounded echo; report the true span.
    spanMs = (uint32_t)(perStepMs * (double)n + 0.5);
  } else {
    spanMs = (spanEndLocal > spanStartLocal)
           ? (spanEndLocal - spanStartLocal) * 1000u : 0u;
  }
  if (reconstructed && stepSource) {
    Log.info("[HRLY] %s equalize n=%lu span=%lums perStep=%.1fms (source=%s) nominal=%lums (%s)",
             tag, (unsigned long)n, (unsigned long)spanMs, perStepMs, stepSource,
             (unsigned long)nominalMs, "reconstructed -> interval used");
  } else {
    Log.info("[HRLY] %s equalize n=%lu span=%lums perStep=%.1fms nominal=%lums (%s)",
             tag, (unsigned long)n, (unsigned long)spanMs, perStepMs,
             (unsigned long)nominalMs,
             equalized ? "in-range" : "out-of-range -> nominal used");
  }
}

// (e) missed-fill reconciliation: totalImpulses vs the sum of the samples we
// actually placed. Zero means the series carried every pulse; a positive value
// is water the ring could not keep (ring overrun) OR that a 0xFFFF-clamped
// sample could not represent (the OVFLOW bench phase, Appendix E supplement B).
// 'placed' is false on the no-absolute-time path, where there are no buckets to
// restore into, so the discrepancy is reported but explicitly not restored.
static void logMissedFill(const char *tag, uint32_t totalImpulses,
                          uint32_t sumSamples, uint32_t buckets, uint32_t spanSec,
                          uint8_t missedFillMode, float restoredGal, bool placed) {
  if (totalImpulses == sumSamples) {
    Log.info("[HRLY] %s missed = totalImpulses(%lu) - sum(samples)(%lu) = 0 "
             "-> nothing to restore", tag,
             (unsigned long)totalImpulses, (unsigned long)sumSamples);
    return;
  }
  if (totalImpulses < sumSamples) {
    // Appendix F.4: the sample sum should never exceed the PIC's own impulse
    // total. When it does, the report's total_impulses (taken at REQ_DATA time)
    // and the samples (up to the last capture boundary) are measured on different
    // bases - some impulses of this report land as samples in the NEXT one. The
    // hourly.cpp guard skips missed-fill safely, but Appendix E.4 (a) requires
    // "sum must equal imp", so this must be shown, not passed over silently.
    Log.warn("[HRLY] %s sample sum (%lu) EXCEEDS totalImpulses (%lu) by %lu "
             "- header basis mismatch, missed-fill skipped", tag,
             (unsigned long)sumSamples, (unsigned long)totalImpulses,
             (unsigned long)(sumSamples - totalImpulses));
    return;
  }
  uint32_t missed = totalImpulses - sumSamples;
  if (!placed) {
    Log.info("[HRLY] %s missed = %lu - %lu = %lu pulses "
             "(buckets not placed this session -> not restored)", tag,
             (unsigned long)totalImpulses, (unsigned long)sumSamples,
             (unsigned long)missed);
    return;
  }
  if (missedFillMode == PIC_MISSED_FILL_AVERAGE) {
    Log.info("[HRLY] %s missed = %lu - %lu = %lu pulses restored as +%.4f gal "
             "across %lu bucket(s) covering %lus", tag,
             (unsigned long)totalImpulses, (unsigned long)sumSamples,
             (unsigned long)missed, restoredGal,
             (unsigned long)buckets, (unsigned long)spanSec);
  } else {
    Log.info("[HRLY] %s missed = %lu - %lu = %lu pulses NOT restored "
             "(missed-fill mode ZERO)", tag,
             (unsigned long)totalImpulses, (unsigned long)sumSamples,
             (unsigned long)missed);
  }
}

// (c)+(d)+(f) Per-bucket basis, carry seam, and MM-of-NN accounting for one
// report. bucketSamples[i] is this batch's sample count in completed bucket i;
// carryBefore/carryAfter are the seam state before and after this report so the
// "carry out = next carry in" invariant can be checked across reports.
static void logBucketBreakdown(const char *tag, const HourlyResult &h,
                               const uint16_t *bucketSamples,
                               const HourlyCarry &carryBefore,
                               const HourlyCarry &carryAfter,
                               const float *bucketGalSamples,
                               const float *bucketGalMissed,
                               uint8_t debugHourly) {
  // (d) carry in: what the previous report handed this one (0 if no seam).
  Log.info("[HRLY] %s carry in  = %.4f gal / %lu pulses at %lu%s", tag,
           carryBefore.valid ? carryBefore.gallons : 0.0f,
           (unsigned long)(carryBefore.valid ? carryBefore.pulses : 0u),
           (unsigned long)(carryBefore.valid ? carryBefore.binStartLocal : 0u),
           carryBefore.valid ? "" : " (no prior seam)");

  // Which window bucket, if any, was seeded with the carry (Appendix G.3.5:
  // carryIn is non-zero only in that one bucket).
  bool     haveSeed  = h.carryMerged;
  uint32_t seededIdx = 0u;
  if (haveSeed && carryBefore.valid && carryBefore.binStartLocal >= h.baseLocal &&
      h.bucketSec > 0u) {
    seededIdx = (carryBefore.binStartLocal - h.baseLocal) / h.bucketSec;
  }

  // (c) per-bucket basis. With <=12 buckets, verbose config, or detailed level,
  // every bucket is printed; otherwise the first 3 and last 3 plus a count.
  uint32_t cnt = h.count;
  bool verbose = (APP_DEBUG_VERBOSE_HOURLY != 0) || (debugHourly >= 2u);
  bool showAll = verbose || (cnt <= 12u);
  bool split   = (debugHourly >= 1u) && (bucketGalSamples != nullptr);
  for (uint32_t i = 0; i < cnt; i++) {
    if (!showAll && i == 3u) {
      Log.info("[HRLY] %s bucket ... %lu more ...", tag, (unsigned long)(cnt - 6u));
    }
    if (!showAll && i >= 3u && i < cnt - 3u) continue;
    uint32_t t0  = h.baseLocal + i * h.bucketSec;
    uint32_t t1  = t0 + h.bucketSec;
    uint32_t sod = t0 % 86400u;                    // seconds-of-day for a readable label
    uint32_t eod = t1 % 86400u;
    unsigned smp = (bucketSamples ? bucketSamples[i] : 0u);
    if (split) {
      // Appendix G.3.5: pulses are the sample-derived count only; the gallon
      // figure is broken into its three sources so the two can be cross-checked.
      float gSamp = bucketGalSamples[i];
      float gMiss = bucketGalMissed ? bucketGalMissed[i] : 0.0f;
      float gCarry = (haveSeed && i == seededIdx) ? carryBefore.gallons : 0.0f;
      Log.info("[HRLY] %s bucket[%lu] %02lu:%02lu:%02lu-%02lu:%02lu:%02lu pulses=%lu samples=%u "
               "gal: samples=%.4f + missed=%.4f + carryIn=%.4f = %.4f (complete)",
               tag, (unsigned long)i,
               (unsigned long)(sod / 3600u), (unsigned long)((sod % 3600u) / 60u), (unsigned long)(sod % 60u),
               (unsigned long)(eod / 3600u), (unsigned long)((eod % 3600u) / 60u), (unsigned long)(eod % 60u),
               (unsigned long)h.pulses[i], smp, gSamp, gMiss, gCarry, h.gal[i]);
    } else {
      Log.info("[HRLY] %s bucket %02lu:%02lu:%02lu pulses=%lu gal=%.4f samples=%u (complete)",
               tag, (unsigned long)(sod / 3600u), (unsigned long)((sod % 3600u) / 60u),
               (unsigned long)(sod % 60u),
               (unsigned long)h.pulses[i], h.gal[i], smp);
    }
  }

  // (d) carry out: the still-open bucket handed to the next report. This value
  // must reappear as the next report's "carry in".
  if (carryAfter.valid && (carryAfter.pulses > 0u || carryAfter.gallons > 0.0f)) {
    unsigned partialSmp = (bucketSamples && h.count < HOURLY_MAX_BUCKETS)
                        ? bucketSamples[h.count] : 0u;
    Log.info("[HRLY] %s carry out = %.4f gal / %lu pulses at %lu (partial, %u samples "
             "-> seeds next report)", tag,
             carryAfter.gallons, (unsigned long)carryAfter.pulses,
             (unsigned long)carryAfter.binStartLocal, partialSmp);
  } else {
    Log.info("[HRLY] %s carry out = 0.0000 gal (span ended on a bucket boundary)", tag);
  }

  // (f) MM-of-NN accounting: how many buckets this report COULD make (MM) vs how
  // many it completed and will publish (NN), against the 250-bucket array cap.
  int carryPartial = (carryAfter.valid &&
                      (carryAfter.pulses > 0u || carryAfter.gallons > 0.0f)) ? 1 : 0;
  Log.info("[HRLY] %s produced %lu bucket(s), published %u (cap %u), carry %d partial", tag,
           (unsigned long)h.totalMakeable, (unsigned)h.count,
           (unsigned)HOURLY_MAX_BUCKETS, carryPartial);
}

// ---- Appendix H.15/H.18/H.20 logging helpers -------------------------------
// A single log-level axis for the Appendix H detail lines. Reuses debugHourly so
// the bench can raise it: 0=off, 1=summary(+all WARN), 2=detailed(+H.15-H.18),
// 3=full. WARN always prints regardless (H.6). CLOUD_FAST recommends level 3.
static inline bool hLogAtLeast(uint8_t lvl) { return g_cfg.debugHourly >= lvl; }

// Appendix H.15: the Time.isValid() false->true transition and its surrounding
// values - the decisive observation point for the whole H.2/H.3 family. Called
// every loop from dbgUartService()'s sibling; here it is driven from the session
// path. Prints once, on the edge.
static void logCloudTimeEdge() {
  bool nowValid = Time.isValid();
  if (nowValid && !g_prevTimeValid) {
    uint32_t utc = 0u; (void)clockNowUtc(utc);
    Log.info("[NET] cloud time received: utc=%lu local=%lu (Time.isValid 0 -> 1)",
             (unsigned long)utc, (unsigned long)((int64_t)utc + (int64_t)g_tzOffsetSec));
  }
  g_prevTimeValid = nowValid;
}

// Appendix H.18: when and on what basis prevReportEnd is (or is not) updated, so
// a regression of H.2 is caught by this one line. Called from ingestReport at the
// seam decision.
static void logSeamUpdate(const char *tag, uint32_t newEnd, bool updated,
                          const char *source) {
  if (!hLogAtLeast(2u)) return;
  if (updated)
    Log.info("[TIME] %s seam: prevReportEnd updated to %lu (valid=1, source=%s)",
             tag, (unsigned long)newEnd, source);
  else
    Log.info("[TIME] %s seam: prevReportEnd NOT updated (source=%s)", tag, source);
}

// Appendix H.20: the one-block per-session health summary. Everything needed to
// judge a session at a glance; the warnings line is non-zero only when something
// asked for attention this session.
static void logSessionSummary(bool hadTime) {
  const char *mode =
      !g_cfg.cloudEnabled ? (BENCH_VIRTUAL_CLOCK ? "FAST_BENCH" : "CLOUD_FAIL")
                          : (g_cfg.cadenceFast ? "CLOUD_FAST" : "PRODUCTION");
  Log.info("[SUM] ===== session #%lu summary =====", (unsigned long)g_sessionCounter);
  Log.info("[SUM] mode=%s  cloud=%s  timeValid=%d  uptime=%.1f s",
           mode, g_cloudOnlineThisSession ? (g_cloudPublishOk ? "OK" : "FAIL") : "OFFLINE",
           (int)hadTime, (double)(millis() / 1000.0f));
  // V065 (req 4.병): rx = stored + ACKed this session (what BATCH_STORED_ACKED
  // reports); placed = of those, how many reached a bucket. They differ whenever
  // there is no time axis, and the old single "n=" reported only the second.
  Log.info("[SUM] data: rx=%lu in %u batch(es)  placed=%lu  picSeries=%s hourly=%s",
           (unsigned long)g_sessionSamplesRx, (unsigned)g_sessionBatchesRx,
           (unsigned long)(g_hourlyValid ? g_hourly.samplesUsed : 0ul),
           g_picSeriesOk ? "OK" : "none",
           g_hourlyValid ? "placed" : "not placed");
  if (g_sessionSamplesRx > 0u && !g_hourlyValid) {
    Log.warn("[SUM] %lu sample(s) received but NOT placed - held raw in flash for "
             "replay once a cloud time arrives (this is the expected no-time path, "
             "not a loss)", (unsigned long)g_sessionSamplesRx);
  }
  Log.info("[SUM] hourly: %u bucket(s) placed, carry %.4f gal",
           (unsigned)(g_hourlyValid ? g_hourly.count : 0u),
           (double)(hourlyCarry.valid ? hourlyCarry.gallons : 0.0f));
#if FLASH_BUFFER_BLOCKS > 0
  Log.info("[SUM] flash: held %u/%u", (unsigned)flashBufferCount(), (unsigned)FLASH_BUFFER_BLOCKS);
#endif
  Log.info("[SUM] cloud: %u event(s), %u OK, %u failed, %lu bytes, connect %.1f s / publish %.1f s",
           (unsigned)g_pubAttempted, (unsigned)g_pubOk, (unsigned)g_pubFailed,
           (unsigned long)g_pubBytes, (double)(g_netConnectMs / 1000.0f),
           (double)(g_netPublishMs / 1000.0f));
  Log.info("[SUM] warnings this session: %u", (unsigned)g_sessionWarnings);
  Log.info("[SUM] ================================");
}

// Appendix G.3.1: the calibration coefficients and the usable frequency range,
// once per session. Printed at debugHourly>=1 so a reviewer does not have to
// recompute Appendix G.1.1 by hand to know where the polynomial collapses.
static void logFlowCalBanner() {
  if (g_cfg.debugHourly < 1u) return;
  const FlowValidRange &r = flowRange();
  Log.info("[HRLY] flowcal C0=%.4f C1=%.4f C2=%.4f C3=%.4f C4=%.4f C5=%.4f C6=%.4f",
           (double)FLOW_C0, (double)FLOW_C1, (double)FLOW_C2, (double)FLOW_C3,
           (double)FLOW_C4, (double)FLOW_C5, (double)FLOW_C6);
  if (r.collapses) {
    Log.info("[HRLY] flowcal scale=%.4f -> valid freq range 0..%.1f Hz "
             "(max %.3f GPM at %.1f Hz), above this freqToGpm() returns 0",
             flowCalScale, r.validMaxHz, r.peakGpm, r.peakFreqHz);
  } else {
    Log.info("[HRLY] flowcal scale=%.4f -> monotone over the scanned range "
             "(max %.3f GPM at %.1f Hz, no collapse observed)",
             flowCalScale, r.peakGpm, r.peakFreqHz);
  }

  // Appendix H.10.5 (2): the usable (monotonic) band is 0..peakFreqHz; above the
  // peak the polynomial is non-monotonic (two frequencies map to one GPM) and it
  // returns 0 above validMaxHz. The old "valid freq range 0..108.8 Hz" wording
  // read as a usable ceiling, which it is not. Split the two values explicitly.
  Log.info("[HRLY] flowcal scale=%.4f -> usable (monotonic) 0..%.1f Hz, peak %.3f GPM; "
           "non-monotonic %.1f..%.1f Hz; returns 0 above %.1f Hz",
           flowCalScale, r.peakFreqHz, r.peakGpm,
           r.peakFreqHz, r.validMaxHz, r.validMaxHz);

  // Appendix H.5.3: one NOTE per session making the two calibration series' 27.3%
  // divergence explicit, so a reviewer cross-checking a published GPM against a
  // published gallon figure is not confused. FLOW_C4 divides the VOLUME only; the
  // RATE returned by freqToGpm() is not divided by it. Coefficients are FROZEN
  // (Appendix H.10) - this is observation, not a change. Open question A pends the
  // bucket-fill test (H.10.4). The divergence relative to the honest integral is
  // (C4-1)/C4 = 27.3%; integrating the published GPM overstates the gallon by
  // (C4-1) = 37.6%.
  Log.info("[HRLY] flowcal NOTE: C4=%.4f divides VOLUME only; the published GPM series "
           "and the gallon series differ by %.1f%% by construction "
           "(open question A, pending bucket test)",
           (double)FLOW_C4, (double)((FLOW_C4 - 1.0f) / FLOW_C4 * 100.0f));

  // Appendix H.10.5 (3): a leak threshold above the calibration's own maximum can
  // never be reached by the gallon path - flag it once at session start.
  if (appConfig.leakThreshGpm > r.peakGpm) {
    Log.warn("[CFG] leak threshold %.2f GPM exceeds the calibration's maximum %.3f GPM "
             "- this threshold can never be reached by the gallon path",
             (double)appConfig.leakThreshGpm, (double)r.peakGpm);
  }
}

// Appendix G.3.2: how the bucket grid for this report was chosen, and whether the
// carry seam actually merged. carryMerged=no with a non-zero carry-in is the
// Appendix G.1.7 loss, made visible immediately.
static void logGridDecision(const char *tag, const HourlyResult &h,
                            const HourlyCarry &carryBefore) {
  // Appendix H.6 / G.1.7: the carry-LOST WARN must fire at ANY level - a silently
  // dropped carry is exactly what this verification output exists to catch. It is
  // evaluated ABOVE the debugHourly gate; only the info detail lines below are gated.
  if (carryBefore.valid && !h.carryMerged &&
      (carryBefore.gallons > 0.0f || carryBefore.pulses > 0u)) {
    Log.warn("[HRLY] %s carry LOST at seam: %.4f gal / %lu pulses at %lu were not "
             "merged (carryMerged=no) - volume dropped at the report boundary", tag,
             carryBefore.gallons, (unsigned long)carryBefore.pulses,
             (unsigned long)carryBefore.binStartLocal);
  }

  if (g_cfg.debugHourly < 1u) return;
  uint32_t phase = (h.bucketSec > 0u) ? (h.baseLocal % h.bucketSec) : 0u;
  const char *align =
#if BUCKET_ALIGN_MODE == BUCKET_ALIGN_FROM_NOW
    "from_now";
#else
    "clock";
#endif
  Log.info("[HRLY] %s grid phase=%lu bucketSec=%lu align=%s", tag,
           (unsigned long)phase, (unsigned long)h.bucketSec, align);
  uint32_t dropped = (h.totalMakeable > h.count) ? (h.totalMakeable - h.count) : 0u;
  Log.info("[HRLY] %s grid firstBin=%lu nBins=%lu published=%u (dropped %lu) carryMerged=%s",
           tag, (unsigned long)h.baseLocal, (unsigned long)h.totalMakeable,
           (unsigned)h.count, (unsigned long)dropped,
           h.carryMerged ? "yes" : "no");
}

// Appendix G.3.4: the missed-fill (ring-overrun / 0xFFFF-clamp) reconstruction
// intermediates, so a 0.0000 gal restoration can be told apart from a collapse.
// clampedSamples comes from the report header (info.overflowFfff).
static void logMissedTrace(const char *tag, const HourlyResult &h,
                           uint32_t totalImpulses, uint32_t sumSamples,
                           uint32_t totalCaptures, unsigned clampedSamples) {
  const MissedFillTrace &m = h.missed;

  // Appendix H.6 / G.1.1: real missed pulses converting to 0 gallons is a lost
  // volume, not a no-op; this WARN fires at ANY level, above the debugHourly gate.
  if (m.applied && m.collapsed) {
    float freq = (h.perSampleSec > 0.0f)
               ? ((m.clamped ? 65535.0f : m.perCapturePulses) / h.perSampleSec) : 0.0f;
    Log.warn("[HRLY] %s missed calc produced 0 gal from %lu pulses - perCapture "
             "freq %.1fHz exceeds the calibration range (%.1fHz). The restored "
             "volume is LOST, not zero.", tag,
             (unsigned long)m.missedPulses, (double)freq, (double)flowRange().validMaxHz);
  }

  if (g_cfg.debugHourly < 1u) return;
  if (!m.applied) return;                 // nothing was reconstructed (F.4 case handled elsewhere)

  Log.info("[HRLY] %s missed inputs: totalImpulses=%lu sentPulses=%lu missed=%lu "
           "totalCaptures=%lu n=%lu missedCaptures=%lu clampedSamples=%u", tag,
           (unsigned long)totalImpulses, (unsigned long)sumSamples,
           (unsigned long)m.missedPulses, (unsigned long)totalCaptures,
           (unsigned long)h.samplesUsed, (unsigned long)m.missedCaptures, clampedSamples);
  Log.info("[HRLY] %s missed calc: spanCaps=%lu (source=%s) perCapturePulses=%.1f "
           "clamped=%s -> toGallons(%lu, %.3fs) x%lu = %.4f gal", tag,
           (unsigned long)m.spanCaps, m.spanCapsFromMissed ? "missedCaptures" : "n",
           (double)m.perCapturePulses, m.clamped ? "yes" : "no",
           (unsigned long)(m.clamped ? 65535u : (uint32_t)(m.perCapturePulses + 0.5f)),
           (double)h.perSampleSec, (unsigned long)m.spanCaps, m.missedGallons);
  Log.info("[HRLY] %s missed spread: %.4f gal / %lu bucket(s) = %.4f gal each", tag,
           m.missedGallons, (unsigned long)m.windowBins, m.perBucketGallons);
}

// Appendix G.3.6 + G.3.7: the two self-checks that decide whether the whole
// calculation is right. Gallons and pulses must each be conserved across the
// report: sum(completed) + carryOut - carryIn == source. A mismatch is a WARN at
// any level - it means volume was created or lost inside hourlyProcess().
static void logSelfChecks(const char *tag, const HourlyResult &h,
                          const HourlyCarry &carryAfter) {
  const bool verbose = (g_cfg.debugHourly >= 1u);

  // --- gallons (G.3.6) ---
  // Appendix H.6: the conservation verdict is computed and WARNed at ANY level -
  // a silent mismatch is exactly what Appendix G exists to prevent. Only the
  // info "CHECK ..." working lines are gated by debugHourly.
  float sumBucketGal = 0.0f;
  for (uint16_t i = 0; i < h.count; i++) sumBucketGal += h.gal[i];
  float carryOutGal = (carryAfter.valid ? carryAfter.gallons : 0.0f);
  float lhsGal      = sumBucketGal + carryOutGal - h.carryInGallons;
  float rhsGal      = h.sampleGallons + h.missedGallons;
  float dGal        = lhsGal - rhsGal;
  float tolGal      = 1e-4f;
  { float scaled = (rhsGal > 0.0f ? rhsGal : -rhsGal) * 1e-5f; if (scaled > tolGal) tolGal = scaled; }
  bool  galOk       = ((dGal < 0.0f ? -dGal : dGal) <= tolGal);
  if (verbose) {
    Log.info("[HRLY] %s CHECK sum(bucket gal)=%.4f + carryOut=%.4f - carryIn=%.4f = %.4f",
             tag, sumBucketGal, carryOutGal, h.carryInGallons, lhsGal);
  }
  if (galOk) {
    if (verbose) {
      Log.info("[HRLY] %s CHECK expected sample gal=%.4f + missed gal=%.4f = %.4f -> MATCH (delta %.7f)",
               tag, h.sampleGallons, h.missedGallons, rhsGal, (double)dGal);
    }
  } else {
    Log.warn("[HRLY] %s CHECK MISMATCH: got %.4f expected %.4f (delta %.7f) "
             "- volume created or lost inside hourlyProcess()",
             tag, lhsGal, rhsGal, (double)dGal);
  }

  // --- pulses (G.3.7). missed is NOT added to pulses by design (Appendix G.1.4). ---
  uint32_t sumBucketPul = 0u;
  for (uint16_t i = 0; i < h.count; i++) sumBucketPul += h.pulses[i];
  uint32_t carryOutPul = (carryAfter.valid ? carryAfter.pulses : 0u);
  uint32_t lhsPul = sumBucketPul + carryOutPul;                 // completed + carry out
  uint32_t rhsPul = h.carryInPulses + h.placedPulses;           // carry in + placed samples
  if (verbose) {
    Log.info("[HRLY] %s CHECK sum(bucket pulses)=%lu + carryOut=%lu = %lu ; "
             "expected carryIn=%lu + placed samples=%lu = %lu (missed 0 NOT added to pulses by design) -> %s",
             tag, (unsigned long)sumBucketPul, (unsigned long)carryOutPul, (unsigned long)lhsPul,
             (unsigned long)h.carryInPulses, (unsigned long)h.placedPulses, (unsigned long)rhsPul,
             (lhsPul == rhsPul) ? "MATCH" : "MISMATCH");
  }
  if (lhsPul != rhsPul) {
    Log.warn("[HRLY] %s CHECK PULSE MISMATCH: got %lu expected %lu - pulses created "
             "or lost inside hourlyProcess()", tag,
             (unsigned long)lhsPul, (unsigned long)rhsPul);
  }
}

// =============================================================== PIC ingest
// Place one report on the time axis and turn it into completed buckets.
//
// The PIC supplies the AMOUNT (pulses) and its own idea of the span; we supply
// the TIME. The rules, in order:
//   - span end   = the PIC wake that started this session (g_spanEndLocal),
//                  not the moment of publishing;
//   - span start = the PIC's start_time when valid, else back-computed, and in
//                  either case snapped to the previous seam when they agree;
//   - equalize   = spread the samples evenly over the real elapsed span;
//   - bin        = credit each sample to the bucket containing its end instant;
//   - carry      = hand the unfinished last bucket to the next report.
// Pulse counts are never modified anywhere in this path.
//
// Appends its buckets to g_hourly so several reports (a flash-buffer recovery)
// can be merged into one continuous series.
//
// haveAbsTime is false in BUILD_MODE_CLOUD_FAIL, where no cloud time was ever
// obtained (Appendix E.5). The batch is then summarised, reconciled and stored
// raw, but NOT placed on a time axis - there is no absolute time to place it on -
// and the reason is logged rather than binning onto a meaningless 1970 grid.
static bool ingestReport(const PicSample *s, uint32_t n, const PicReportInfo &info,
                         uint32_t endLocal, bool haveAbsTime, const char *tag,
                         bool reconstructed) {
  if (n == 0u) return false;

  // The PIC's own intended interval is the sanity reference for equalization and
  // the only step available when there is no measured span. Trim-independent, so
  // it is computed once here and used by both branches below.
  float nominalSec = (float)info.sampleIntervalMs / 1000.0f;
  if (nominalSec < 0.001f) nominalSec = g_cfg.captureIntervalSecF;

  // ==== Appendix F.1-A: NO ABSOLUTE TIME IS HANDLED FIRST ===================
  // With no cloud time there is no axis to place buckets on and therefore no seam
  // to defend: hourlyResolveSpanStart() and hourlyOverlapSkip() must not run at
  // all. The pre-V062 code ran them first and, because a stale retained
  // prevReportEnd compared "after" a meaningless local uptime, the overlap guard
  // returned the whole batch as already-placed and this branch never executed -
  // so Appendix E.4/E.5's verification log was silent every session. Deciding the
  // no-time case at the very top of the flow is the fix: if there is no clock, we
  // never touch the time-axis functions.
  if (!haveAbsTime) {
    uint32_t sumSamples = 0u;
    for (uint32_t k = 0; k < n; k++) sumSamples += s[k].pulses;

    uint32_t nomMs = info.sampleIntervalMs ? info.sampleIntervalMs
                                           : (uint32_t)(nominalSec * 1000.0f + 0.5f);
    logEqualize(tag, n, 0u, 0u, nominalSec, nomMs, false);
    logMissedFill(tag, info.totalImpulses, sumSamples, 0u, 0u,
                  g_cfg.missedFillMode, 0.0f, false);   // no buckets -> not restored
    Log.info("[HRLY] %s no absolute time -> buckets not placed; batch stored raw "
             "(n=%lu sum=%lu)", tag, (unsigned long)n, (unsigned long)sumSamples);

    // Per-sample bookkeeping still runs: the leak model and interval logger work
    // on relative spacing and do not need an absolute epoch. Appendix F.1-B: the
    // caller now passes endLocal=0 (no fabricated epoch), so timestamps here use
    // an explicitly RELATIVE base derived from the monotonic millisecond counter.
    // These are NOT absolute times and the log says so.
    uint32_t relBase = (uint32_t)(millis() / 1000u);
    bool     flowSeen   = false;
    uint32_t lastFlowTs = 0;
    for (uint32_t k = 0; k < n; k++) {
      uint32_t tsRel = relBase + (uint32_t)((float)k * nominalSec + 0.5f);   // forward, relative
      float    freq  = (float)s[k].pulses / nominalSec;
      float    gpm   = freqToGpm(freq);
      // Appendix G.1.1 collapse is a WARN at any level - a 0 GPM that came from an
      // out-of-range frequency must not look like a genuine no-flow reading.
      if (s[k].pulses > 0u && gpm <= 0.0f && freq > flowRange().validMaxHz) {
        Log.warn("[DAT] %s sample[%lu/%lu]: pulses=%u freq=%.1fHz OUT OF CALIBRATION "
                 "RANGE -> 0.000 GPM (polynomial collapsed, not saturated)", tag,
                 (unsigned long)(k + 1u), (unsigned long)n, s[k].pulses, (double)freq);
      }
      if (g_cfg.debugHourly >= 2u) {
        Log.info("[DAT] %s sample[%lu/%lu]: tRel=%lus(relative,no absolute time) pulses=%u "
                 "freq=%.3fHz -> %.3f GPM, %.4f gal", tag,
                 (unsigned long)(k + 1u), (unsigned long)n, (unsigned long)tsRel,
                 s[k].pulses, (double)freq, gpm, sampleToGallons(s[k].pulses, nominalSec));
      }
      if (gpm >= FLOW_ACTIVE_GPM) { flowSeen = true; lastFlowTs = tsRel; }
      if (senseLeak(tsRel, gpm)) onLeakDetected();
      appendIntervalSample(gpm);
    }
    if (flowSeen) { triggerState = true; lastTriggerTime = lastFlowTs; }

    // prevReportEnd is deliberately NOT advanced and NOT validated: nothing was
    // placed on the axis, so there is no seam. The credited totals ARE recorded so
    // a lost-0x0B retransmit still reconciles in the overlap path.
    prevCreditedImpulses = info.totalImpulses;
    prevCreditedCaptures = info.totalCaptures;

    Log.info("[DAT] %s ingested %lu samples (raw, no buckets)%s",
             tag, (unsigned long)n, flowSeen ? " [flow active]" : "");
    return true;
  }

  // ==== Absolute-time path ==================================================
  // Everything below runs ONLY when a real cloud-time axis exists.
  //
  // Appendix F.1-C: consult prevReportEnd as a seam only when it was written by a
  // genuine absolute-time placement. A device that ran the no-cloud build can hold
  // a stale prevReportEnd from an older firmware; treating that as an epoch is
  // what produced the false overlap-skip. When invalid, pass 0 so both
  // hourlyResolveSpanStart() and hourlyOverlapSkip() see "no seam recorded yet".
  uint32_t prevEnd = prevReportEndValid ? prevReportEnd : 0u;

  const char *startSrc = "";
  uint32_t startLocal = hourlyResolveSpanStart(info, endLocal, prevEnd,
                                               SPAN_CONTINUITY_TOL_SEC, &startSrc);

  // ---- V059: drop the part of this batch that is already on the axis -------
  // Two paths deliver the same samples twice, and neither is exotic:
  //   - a lost PKT_DATA_RECEIVED (0x0B): the PIC never advanced its mark, so it
  //     retransmits the whole span on the next session;
  //   - replayBufferedReports(): the flash ring is only cleared on a SUCCESSFUL
  //     cloud publish, so one failed publish replays blocks that were already
  //     ingested and already acked.
  // Nothing below this point is idempotent: hourlyProcess() bins every sample it
  // is handed, and legacyRollingApply() accumulates into the retained
  // hourlyData[] / dailyGallons with "+=". Snapping the span start is not enough
  // either - it only compresses the same pulses into a shorter span. The samples
  // themselves have to go.
  PicReportInfo eff  = info;                      // header totals, on the same basis
  uint32_t      skip = hourlyOverlapSkip(n, startLocal, endLocal, prevEnd);

  if (skip >= n) {
    // An exact retransmission of a report we already placed. Returning true is
    // deliberate: the caller must still send 0x0B, or the PIC keeps this batch
    // forever and resends it every session.
    Log.warn("[DAT] %s: all %lu sample(s) precede prevReportEnd=%lu - already "
             "placed, batch skipped (acking so the PIC can release it)",
             tag, (unsigned long)n, (unsigned long)prevEnd);
    return true;
  }

  if (skip > 0u) {
    // Trimming the samples is only half of it. The header totals are cumulative
    // from the PIC's mark, so they still describe the water we already credited;
    // left alone, missed-fill would see totalImpulses > sum(kept samples) and
    // restore the overlap a SECOND time as "water with no samples".
    //
    // Subtracting the pulses of the dropped SAMPLES is not the right correction
    // either - when the ring overran, the samples covering the overlap no longer
    // exist, so their pulses understate what was already credited. What we
    // credited is exactly the totals of the earlier report, which is why they
    // are retained.
    if (info.totalImpulses >= prevCreditedImpulses &&
        info.totalCaptures >= prevCreditedCaptures) {
      eff.totalImpulses = info.totalImpulses - prevCreditedImpulses;
      eff.totalCaptures = info.totalCaptures - prevCreditedCaptures;
    } else {
      // The totals went backwards, so they are NOT measured from the same mark
      // (a PIC cold boot, a counter reset). Fall back to the sample-level
      // correction, which is conservative rather than exact.
      uint32_t droppedPulses = 0u;
      for (uint32_t k = 0; k < skip; k++) droppedPulses += s[k].pulses;
      eff.totalImpulses = (info.totalImpulses > droppedPulses)
                        ? (info.totalImpulses - droppedPulses) : 0u;
      eff.totalCaptures = (info.totalCaptures > skip) ? (info.totalCaptures - skip) : 0u;
      Log.warn("[DAT] %s: header totals are below what we already credited - "
               "the PIC mark moved; falling back to a sample-level trim", tag);
    }
    eff.sampleCount    = n - skip;
    eff.startTime      = prevEnd;
    eff.startTimeValid = 1u;

    Log.warn("[DAT] %s: %lu of %lu sample(s) overlap prevReportEnd=%lu - dropped; "
             "totals %lu/%lu -> %lu/%lu (imp/caps); retransmit or replay",
             tag, (unsigned long)skip, (unsigned long)n, (unsigned long)prevEnd,
             (unsigned long)info.totalImpulses, (unsigned long)info.totalCaptures,
             (unsigned long)eff.totalImpulses, (unsigned long)eff.totalCaptures);
    // Appendix E.4 (h): make the double-count defence self-checking in the log -
    // header impulses minus what we already credited is the effective new water.
    Log.info("[DAT] %s overlap skip=%lu samples (prevReportEnd=%lu); "
             "prevCredited imp=%lu caps=%lu -> effective imp=%lu caps=%lu",
             tag, (unsigned long)skip, (unsigned long)prevEnd,
             (unsigned long)prevCreditedImpulses, (unsigned long)prevCreditedCaptures,
             (unsigned long)eff.totalImpulses, (unsigned long)eff.totalCaptures);

    s         += skip;
    n         -= skip;
    startLocal = prevEnd;
    startSrc   = "overlap_trimmed";
  }

  // Sum of the (post-trim) samples we are about to place. Cross-checks against
  // the PIC's totalImpulses in the missed-fill line (Appendix E.4 e).
  uint32_t sumSamples = 0u;
  for (uint32_t k = 0; k < n; k++) sumSamples += s[k].pulses;

  // ---- Appendix E.4: full placement with verification logging ---------------
  HourlyCarry carryBefore = hourlyCarry;            // (d) seam state entering this report
  // File-scope (not stack): 250 * (u16 + 2*float) is ~2.5 kB. ingestReport() is
  // never reentered, so a single shared set of scratch arrays is safe.
  static uint16_t bucketSamples[HOURLY_MAX_BUCKETS];    // (c) per-bucket sample counts
  static float    bucketGalSamples[HOURLY_MAX_BUCKETS]; // (G.3.5) sample-derived gallons per bucket
  static float    bucketGalMissed[HOURLY_MAX_BUCKETS];  // (G.3.5) missed-fill gallons per bucket

  HourlyResult h;
  if (!hourlyProcess(s, n, eff, startLocal, endLocal, g_bucketSec,
                     nominalSec, sampleToGallons, g_cfg.missedFillMode,
                     hourlyCarry, h, bucketSamples, HOURLY_MAX_BUCKETS,
                     bucketGalSamples, bucketGalMissed, reconstructed)) {
    Log.warn("[DAT] %s: cannot place batch (n=%lu span=%lu..%lu)",
             tag, (unsigned long)n, (unsigned long)startLocal, (unsigned long)endLocal);
    return false;
  }

  // (b) equalize, (c/d/f) bucket breakdown + carry seam + MM-of-NN, (e) missed,
  // plus the Appendix G verification lines: grid decision (G.3.2), missed-fill
  // intermediates (G.3.4), split bucket lines (G.3.5) and the two self-checks
  // (G.3.6/G.3.7). The G lines are gated by g_cfg.debugHourly; the WARN lines
  // inside them fire at any level.
  {
    uint32_t nomMs   = eff.sampleIntervalMs ? eff.sampleIntervalMs
                                            : (uint32_t)(nominalSec * 1000.0f + 0.5f);
    uint32_t spanSec = (h.spanEndLocal > h.spanStartLocal)
                     ? (h.spanEndLocal - h.spanStartLocal) : 0u;
    // Appendix H.4: name the interval source used for a reconstructed span.
    const char *stepSrc = eff.sampleIntervalMs ? "block header sampleIntervalMs"
                                               : "RSP_PHOTON_CFG captureIntervalMs";
    logEqualize(tag, n, h.spanStartLocal, h.spanEndLocal, h.perSampleSec, nomMs, h.equalized,
                reconstructed, stepSrc);
    Log.info("[HRLY] %s bins base=%lu bucketSec=%lu completed=%u makeable=%lu src=%s",
             tag, (unsigned long)h.baseLocal, (unsigned long)h.bucketSec,
             (unsigned)h.count, (unsigned long)h.totalMakeable, startSrc);
    logGridDecision(tag, h, carryBefore);                                     // G.3.2
    logBucketBreakdown(tag, h, bucketSamples, carryBefore, hourlyCarry,
                       bucketGalSamples, bucketGalMissed, g_cfg.debugHourly);  // (c/d/f) + G.3.5
    logMissedFill(tag, eff.totalImpulses, sumSamples, h.count, spanSec,
                  g_cfg.missedFillMode, h.missedGallons, true);   // (e) placed on the axis
    logMissedTrace(tag, h, eff.totalImpulses, sumSamples,
                   eff.totalCaptures, (unsigned)eff.overflowFfff);            // G.3.4
    logSelfChecks(tag, h, hourlyCarry);                                       // G.3.6 + G.3.7
  }

  // Append this report's buckets to the session series. Contiguity is guaranteed
  // by the shared carry, so the only join to make is index arithmetic.
  if (!g_hourlyValid) {
    g_hourly       = h;
    g_hourlyValid  = true;
    g_hourlyMakeable = h.totalMakeable;
  } else {
    for (uint16_t i = 0; i < h.count; i++) {
      uint32_t binStart = h.baseLocal + (uint32_t)i * h.bucketSec;
      if (binStart < g_hourly.baseLocal) continue;                 // older than the window
      uint32_t idx = (binStart - g_hourly.baseLocal) / h.bucketSec;
      if (idx >= HOURLY_MAX_BUCKETS) {
        // The merged series outgrew the array. Drop the OLDEST buckets so the
        // newest data survives; the loss stays visible as makeable > count.
        uint32_t shift = idx - HOURLY_MAX_BUCKETS + 1u;
        if (shift >= g_hourly.count) { g_hourly.count = 0; g_hourly.baseLocal = binStart; idx = 0; }
        else {
          for (uint32_t k = shift; k < g_hourly.count; k++) {
            g_hourly.gal[k - shift]    = g_hourly.gal[k];
            g_hourly.pulses[k - shift] = g_hourly.pulses[k];
          }
          g_hourly.count     = (uint16_t)(g_hourly.count - shift);
          g_hourly.baseLocal = g_hourly.baseLocal + shift * h.bucketSec;
          idx -= shift;
        }
      }
      for (uint32_t k = g_hourly.count; k < idx; k++) { g_hourly.gal[k] = 0.0f; g_hourly.pulses[k] = 0u; }
      if (idx >= g_hourly.count) { g_hourly.gal[idx] = 0.0f; g_hourly.pulses[idx] = 0u; }
      g_hourly.gal[idx]    += h.gal[i];
      g_hourly.pulses[idx] += h.pulses[i];
      if (idx + 1u > g_hourly.count) g_hourly.count = (uint16_t)(idx + 1u);
    }
    g_hourly.spanEndLocal  = h.spanEndLocal;
    g_hourly.samplesUsed  += h.samplesUsed;
    g_hourly.missedGallons += h.missedGallons;
    g_hourlyMakeable      += h.totalMakeable;
  }

  // Everything below is per-sample bookkeeping the rest of the firmware needs:
  // the leak model, the interval logger and the flow-active latch. It uses the
  // same equalized placement as the buckets so the two views agree.
  bool     flowSeen   = false;
  uint32_t lastFlowTs = 0;
  uint32_t windowFirstBin = h.baseLocal;
  for (uint32_t k = 0; k < n; k++) {
    uint32_t back  = (uint32_t)((float)(n - 1u - k) * h.perSampleSec + 0.5f);
    uint32_t tsEnd = (endLocal > back) ? (endLocal - back) : startLocal;
    float    freq  = (float)s[k].pulses / h.perSampleSec;
    bool     clamped = false;
    float    gpm   = freqToGpmUsable(freq, &clamped);   // H.10.5 (1): usable clamp

    if (clamped) {
      // Appendix H.10.5 (1): explicit INTERIM notice. The flow is past the usable
      // peak, so the reported GPM is a LOWER BOUND, not the true (higher) flow.
      Log.warn("[HRLY] freq %.1fHz is beyond the calibration's usable range (%.1fHz). "
               "Clamped to peak %.3f GPM - this is a LOWER BOUND, the true flow is higher. "
               "INTERIM measure pending the bucket-fill test; coefficients unchanged.",
               (double)freq, (double)flowRange().peakFreqHz, (double)flowRange().peakGpm);
    } else if (s[k].pulses > 0u && gpm <= 0.0f && freq > flowRange().validMaxHz) {
      // Appendix G.1.1 collapse (only reachable with FLOW_CLAMP_ABOVE_PEAK=0):
      // a 0 GPM produced by an out-of-calibration frequency, not a genuine no-flow.
      Log.warn("[DAT] %s sample[%lu/%lu]: pulses=%u freq=%.1fHz OUT OF CALIBRATION "
               "RANGE -> 0.000 GPM (polynomial collapsed, not saturated)", tag,
               (unsigned long)(k + 1u), (unsigned long)n, s[k].pulses, (double)freq);
    }
    // ---- V068 requirement 3: the contract's 4-column diagnostic table -------
    // Timestamp / sample_ID / sample_count / GPM, one line per sample.
    // Deliberately NOT behind debugHourly: the agreement asks for this to be
    // readable "using Photon serial without the cloud", which a debug-gated line
    // does not satisfy. tsEnd is the LOCAL epoch at which the sample ends (the
    // same axis every other epoch in this firmware uses).
    if (DIAG_TABLE_ENABLE) {
      Log.info("[DIAG] %lu\t%lu\t%u\t%.3f",
               (unsigned long)tsEnd, (unsigned long)(k + 1u), s[k].pulses, (double)gpm);
    }
    if (DIAG_VERBOSE_ENABLE && g_cfg.debugHourly >= 2u) {
      // Appendix G.3.3: the conversion intermediates and the bucket this sample
      // landed in, so binning and the polynomial can both be hand-checked.
      // Kept alongside the 4-column table because the table cannot express dt -
      // the field that exposed the 2x Timer0 period error in the 2026-08
      // campaign - nor bin, freq/g0/poly or the gallon figure the conservation
      // check needs.
      float f   = freq / (1.0f + (FLOW_C5 * freq + FLOW_C6));
      float g0  = FLOW_C0 * f;
      float pol = g0 - (FLOW_C1 * g0 * g0 + FLOW_C2 * g0 + FLOW_C3);
      uint32_t bin = (h.bucketSec > 0u && tsEnd >= windowFirstBin)
                   ? ((tsEnd - 1u - windowFirstBin) / h.bucketSec) : 0u;
      Log.info("[DAT] %s sample[%lu/%lu]: t=%lu bin=%lu pulses=%u dt=%.3fs "
               "freq=%.3fHz f=%.3f g0=%.4f poly=%.4f -> %.4f GPM -> %.4f gal", tag,
               (unsigned long)(k + 1u), (unsigned long)n, (unsigned long)tsEnd,
               (unsigned long)bin, s[k].pulses, (double)h.perSampleSec,
               (double)freq, (double)f, (double)g0, (double)pol, gpm,
               sampleToGallons(s[k].pulses, h.perSampleSec));
    }
    if (gpm >= FLOW_ACTIVE_GPM) { flowSeen = true; lastFlowTs = tsEnd; }
    if (senseLeak(tsEnd, gpm)) onLeakDetected();
    appendIntervalSample(gpm);
  }
  logLeakModel();                                 // Appendix H.19: per-report [LEAK] state (level 3)

  legacyRollingApply(h);                          // keep the 24-slot dashboard view in step
  roll48Apply(h);                                 // V068: and the contract's 48-slot sliding window
                                                  //   (same finished buckets -> the three views cannot
                                                  //    disagree about a total)

  // Appendix H.2-B: prevReportEnd may only be advanced/validated from a MEASURED
  // cloud-time placement. A reconstructed span (sec 6.7 recovery) is anchored at
  // "now" but its endpoint is a back-computed estimate, not an observed seam;
  // treating it as a real epoch is what let H.2 poison the seam. The next report
  // placed on a measured clock sets the seam instead. H.2-A already prevents
  // reconstruction without a real clock; this is the double defence H.2-B asks for.
  if (!reconstructed) {
    prevReportEnd      = endLocal;                // this seam continues the next report
    prevReportEndValid = true;                    // Appendix F.1-C: a real absolute placement
    logSeamUpdate(tag, endLocal, true, "real cloud time");   // Appendix H.18
  } else {
    Log.info("[DAT] %s: prevReportEnd NOT advanced from a reconstructed span "
             "(Appendix H.2-B); awaiting a measured-clock placement", tag);
    logSeamUpdate(tag, prevReportEnd, false, "reconstructed span (H.2-B)");   // Appendix H.18
  }

  // V059: remember what we credited against the PIC's current mark. If the 0x0B
  // ack is lost, the next delivery carries totals from the SAME mark and these
  // are what make the overlap correction exact. Always the report's own totals,
  // never a running sum: they are already cumulative from the mark.
  prevCreditedImpulses = info.totalImpulses;
  prevCreditedCaptures = info.totalCaptures;

  if (flowSeen) { triggerState = true; lastTriggerTime = lastFlowTs; }

  imu_data.dailyGallons = dailyGallons;
  Log.info("[DAT] %s ingested %lu samples -> %u buckets, daily=%.2f gal%s",
           tag, (unsigned long)n, (unsigned)h.count, dailyGallons,
           flowSeen ? " [flow active]" : "");

  if (g_cfg.debugDataseries) {                    // compact raw series, bounded length
    const int kMaxShown = 80;
    char pbuf[560];
    int  ppos = 0;
    uint32_t shown = (n < (uint32_t)kMaxShown) ? n : (uint32_t)kMaxShown;
    for (uint32_t k = 0; k < shown; k++) {
      ppos += snprintf(pbuf + ppos, sizeof(pbuf) - ppos,
                       (k == 0u) ? "%u" : ",%u", (unsigned)s[k].pulses);
    }
    if (n > shown) snprintf(pbuf + ppos, sizeof(pbuf) - ppos,
                            ",...(+%lu more)", (unsigned long)(n - shown));
    Log.info("[DAT] %s samples: [%s]", tag, pbuf);
  }
  return true;
}

// One logger entry per PIC sample (intervals are aligned 1:1).
// V056: when the buffer is full the OLDEST sample is dropped instead of silently
// discarding every new one. A session then always publishes the most recent
// history. gMeter.start_slot counts how many samples have been dropped, and is
// published as "droppedOldest" so the dashboard can see the truncation.
void appendIntervalSample(float gpm) {
  uint16_t val = (uint16_t)(gpm * 100.0f + 0.5f); // 0.01 GPM units
                                                  //   Store GPM as an integer in hundredths (e.g. 1.23 GPM -> 123). +0.5 rounds.
  if (val > 999u) val = 999u;                     // Cap at 999 (9.99 GPM) so each value fits a 3-digit slot.

  if (gMeter.count >= METER_LOG_SLOTS) {          // Buffer full -> make room by dropping the oldest.
    memmove(&gMeter.raw[0], &gMeter.raw[1],
            (size_t)(METER_LOG_SLOTS - 1) * sizeof(gMeter.raw[0]));
    gMeter.count      = METER_LOG_SLOTS - 1;
    gMeter.start_slot = (uint16_t)(gMeter.start_slot + 1u);   // cumulative dropped count
    static bool warned = false;
    if (!warned) {                                // Say it once per boot, not once per sample.
      warned = true;
      Log.warn("[DAT] interval logger full (%u slots) -> dropping oldest samples",
               (unsigned)METER_LOG_SLOTS);
    }
  }
  gMeter.raw[gMeter.count++] = val;               // Store the value and advance the count by one.
}

// React to a detected leak: bump counters, optionally shut off the valve, optionally alert.
void onLeakDetected() {
  if (!imu_data.overflow) overflowEventCount++;   // First time seeing overflow this cycle -> count it.
  imu_data.overflow = true;                       // Mark overflow active.

  if (!imu_data.leaking) leakingEventCount++;     // First time seeing a leak this cycle -> count it.
  imu_data.leaking = true;                        // Mark leaking active.

  if (appConfig.autoShutoff) {                    // cloud var (3)
    shutoffSwitch("close");                       //   If auto-shutoff is enabled, close the local valve.
  }
  if (appConfig.alertMode >= 1) {                 // cloud var (4)
    triggerPublish = true;                        //   If alerts are enabled, request a cloud publish.
  }
}

#if USE_LOCAL_METER                               // The following local-sensor code only exists when enabled.
// Optional local hall-sensor path (disabled by default). Kept for fallback.
void serviceLocalMeter() {
  static unsigned long lastCalc = 0;              // Remember the last time we computed gallons (persists between calls).
  unsigned long nowMs = millis();                 // Current time since boot, in milliseconds.
  if (lastMeterWakeTime == 0) { lastMeterWakeTime = nowMs; return; }   // First call: just record the time and exit.
  if (nowMs - lastMeterWakeTime < 1000) return;   // Only recompute about once per second.

  float freq = (retainedPulseCount * 1000.0f) / (nowMs - lastMeterWakeTime);   // Pulses per second = frequency in Hz.
  retainedPulseCount = 0;                         // Reset the pulse counter for the next window.
  lastMeterWakeTime  = nowMs;                     // Record this as the new window start.

  float gpm = freqToGpm(freq);                    // Convert the frequency to gallons-per-minute.
  uint32_t now = 0u;
  if (gpm > 0 && localNow(now)) {                 // Only proceed if there is real flow AND a valid clock (H.3).
    float gallons = gpm * ((nowMs - lastCalc) / 60000.0f) / FLOW_C4;   // Gallons over the elapsed minutes (ms/60000).
    // The optional local sensor is a fallback path with no report structure, so
    // it cannot go through the bucket engine (which works per report span). It
    // feeds the legacy rolling view directly.
    int idx = (int)((now / g_bucketSec) % BUCKET_COUNT);
    hourlyData[idx] += gallons;
    dailyGallons    += gallons;                   // Add to the daily total.
    if (senseLeak(now, gpm)) onLeakDetected();    // Run the leak detector and react if needed.
  }
  lastCalc = nowMs;                               // Remember this time for the next gallons calculation.
}
#endif                                            // End of local-meter code.

// =============================================================== IMU
#if USE_IMU                                       // Real IMU code (only compiled when the sensor is enabled).
// Measure the gyroscope's resting bias by averaging many still readings.
void calibrateGyroscope() {
  sensors_event_t g; float sx = 0, sy = 0, sz = 0;   // 'g' holds one reading; sx/sy/sz accumulate sums.
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {    // Take GYRO_CAL_SAMPLES (200) readings...
    lsm_gyro->getEvent(&g); sx += g.gyro.x; sy += g.gyro.y; sz += g.gyro.z;   // ...summing each axis.
  }
  gyroBiasX = sx / GYRO_CAL_SAMPLES;              // Average X = the resting bias on X.
  gyroBiasY = sy / GYRO_CAL_SAMPLES;              // Average Y bias.
  gyroBiasZ = sz / GYRO_CAL_SAMPLES;              // Average Z bias.
}

// Find and start the IMU on one of its two possible I2C addresses. Returns 1 ok / 0 fail.
int imuInit() {
  if      (lsm6ds.begin_I2C(LSM6DS3_ADDR_A)) imu_data.sensor = LSM6DS3_ADDR_A;   // Try address A first.
  else if (lsm6ds.begin_I2C(LSM6DS3_ADDR_B)) imu_data.sensor = LSM6DS3_ADDR_B;   // Else try address B.
  else { Log.error("LSM6DS not found"); return 0; }   // 0 = failure
                                                  //   Neither address worked -> report the sensor is missing.
  lsm_temp  = lsm6ds.getTemperatureSensor();      // Get a handle to the temperature sub-sensor.
  lsm_accel = lsm6ds.getAccelerometerSensor();    // Get a handle to the accelerometer.
  lsm_gyro  = lsm6ds.getGyroSensor();             // Get a handle to the gyroscope.
  lsm6ds.enableWakeup(true);                      // Enable motion-based wake on the IMU.
  Log.info("IMU at 0x%02X", imu_data.sensor);     // Log which address the IMU answered on.
  return 1;                                        // 1 = success
}

// Read the latest temperature/accel/gyro values into imu_data. Returns 0 ok / -1 if no IMU.
int imuGet() {
  if (imu_data.sensor == 0) return -1;            // No IMU present -> nothing to read.
  sensors_event_t a, g, t;                        // Holders for accel (a), gyro (g), temperature (t).
  lsm_temp->getEvent(&t); lsm_accel->getEvent(&a); lsm_gyro->getEvent(&g);   // Read all three sub-sensors.
  imu_data.temperature = t.temperature;           // Store the temperature.
  imu_data.accelX = a.acceleration.x; imu_data.accelY = a.acceleration.y; imu_data.accelZ = a.acceleration.z;   // Store accel X/Y/Z.
  imu_data.gyroX  = g.gyro.x - gyroBiasX;         // Store gyro X minus its bias.
  imu_data.gyroY  = g.gyro.y - gyroBiasY;         // Store gyro Y minus its bias.
  imu_data.gyroZ  = g.gyro.z - gyroBiasZ;         // Store gyro Z minus its bias.
  return 0;                                        // Success.
}
#else  // USE_IMU == 0 : no physical IMU, keep the rest of the app working
// These are harmless stand-ins so the rest of the program still compiles/runs without an IMU.
void calibrateGyroscope() {}                      // Do nothing (no gyro to calibrate).
int  imuInit() { imu_data.sensor = 0; return 0; } // Mark "no IMU" and report success of the stub.
int  imuGet()  { return -1; }                     // Always report "no IMU data".
#endif                                            // End of IMU vs no-IMU selection.

// Log a short one-line IMU/status summary (or note it's unavailable).
void imuPrint() {
  if (imuGet() != 0) { Log.info("IMU not available"); return; }   // If reading failed, say so and stop.
  Log.info("IMU 0x%02X Leak:%s Shutoff:%s Temp:%.1fC", imu_data.sensor,   // Otherwise print address, flags, temp.
           imu_data.leaking ? "Y" : "N", imu_data.shutoff ? "Y" : "N",
           imu_data.temperature);
}

// Round a value to one decimal place (e.g. 1.27 -> 1.3). Used to shrink published numbers.
static inline float roundTenth(float v) { return floorf(v * 10.0f + 0.5f) / 10.0f; }   // *10, round, /10.

// Echo the hourly-flow summary over USB CDC: the 24 hour buckets + the daily total, using
// the SAME rounding the cloud uses (roundTenth, one decimal) so the USB line matches the
// published "hourlyGallons" array exactly. Call this AFTER cloud connect + ingest so the
// values reflect freshly ingested, time-valid data rather than stale retained (flash)
// values. Used in both builds: the cloud build's imuPublish() also sends these up; this
// line just makes them directly visible on the serial monitor for the test.
void printHourlyFlow() {
  if (!g_hourlyValid || g_hourly.count == 0u) {
    Log.info("[DAT] hourlyGallons: none produced this session "
             "(carry=%.4f gal at %lu, dailyGal=%.1f)",
             hourlyCarry.gallons, (unsigned long)hourlyCarry.binStartLocal,
             roundTenth(dailyGallons));
    return;
  }

  // The bucket count is whatever the span produced - print it, never assume it.
  Log.info("[DAT] hourlyGallons: %u completed bucket(s) of %lu s, base=%lu local, "
           "makeable=%lu, span=%lu..%lu, samples=%lu",
           (unsigned)g_hourly.count, (unsigned long)g_hourly.bucketSec,
           (unsigned long)g_hourly.baseLocal, (unsigned long)g_hourlyMakeable,
           (unsigned long)g_hourly.spanStartLocal, (unsigned long)g_hourly.spanEndLocal,
           (unsigned long)g_hourly.samplesUsed);

  // Printed in bounded lines so a 250-bucket recovery does not produce one
  // unreadable kilobyte-long log entry.
  const uint16_t perLine = 24;
  for (uint16_t off = 0; off < g_hourly.count; off += perLine) {
    char buf[220];
    int  pos = 0;
    uint16_t end = (uint16_t)((off + perLine < g_hourly.count) ? (off + perLine) : g_hourly.count);
    for (uint16_t i = off; i < end && pos < (int)sizeof(buf) - 12; i++) {
      pos += snprintf(buf + pos, sizeof(buf) - pos,
                      (i == off) ? "%.1f" : ",%.1f", roundTenth(g_hourly.gal[i]));
    }
    Log.info("[DAT]   bucket[%u..%u] t0=%lu : [%s]",
             (unsigned)off, (unsigned)(end - 1u),
             (unsigned long)(g_hourly.baseLocal + (uint32_t)off * g_hourly.bucketSec), buf);
  }
  Log.info("[DAT] carry forward: %lu pulses / %.4f gal in the open bucket starting %lu",
           (unsigned long)hourlyCarry.pulses, hourlyCarry.gallons,
           (unsigned long)hourlyCarry.binStartLocal);
  Log.info("[DAT] dailyGal=%.1f bucketSec=%lu (48-slot window is hourlyGallons)",
           roundTenth(dailyGallons), (unsigned long)g_bucketSec);
}

// =============================================================== Publishing
// Publish the interval logger to the cloud in chunks of up to 120 samples each.
// Emit a cloud event. In the cloud build this actually publishes AND logs it.
// In FAST_BENCH_TEST there is no cloud, so we DO NOT transmit -- we only log the
// exact payload over USB-CDC, so the bench can see every byte that would have
// gone to the cloud without any network. This is how "all cloud-bound data is
// mirrored to USB-CDC" while nothing is actually connected.
// Returns whether the payload actually reached the cloud. That answer decides
// whether the raw report may be dropped or must stay in the flash ring, so it is
// tracked rather than assumed. On the bench there is no cloud, and a simulated
// emit counts as success so the bench exercises the same code path.
static bool cloudEmit(const char *event, const char *payload) {
  picKeepalivePump();          // a publish burst must not look like a dead Photon to the PIC
  size_t bytes = payload ? strlen(payload) : 0u;
  g_pubAttempted++;
  g_pubBytes += (uint32_t)bytes;

  // Not online this session (no-cloud build, or an injected CONNECT failure -
  // Appendix H.13.1/H.13.3): log the exact payload over USB so the bench sees
  // every byte that would have gone out, and count it as delivered so the same
  // code path runs.
  if (!g_cloudOnlineThisSession) {
    Log.info("[CLD] SIM  %s: %s", event, payload);
    g_pubOk++;
    return true;
  }

  // Appendix H.13.3: inject a publish failure this session if selected. The
  // connect-failure kind is handled earlier (the session never gets here online);
  // this arm models "connected but publish refused" - a different code path.
  if (g_injectFail == CLOUD_FAIL_PUBLISH) {
    g_pubFailed++;
    g_sessionWarnings++;
    g_cloudPublishOk = false;
    Log.warn("[CLD] publish FAILED %s: injected fault (CLOUD_FAIL_EVERY_N) "
             "-> g_cloudPublishOk=false (payload kept in the flash ring)", event);
    return false;
  }

  uint32_t t0 = millis();
  bool ok = Particle.publish(event, payload);     // real publish to the Particle cloud
  uint32_t dt = millis() - t0;
  g_netPublishMs += dt;
  if (ok) {
    g_pubOk++;
    // Appendix H.16: record the ack timing and byte size, not just the payload.
    Log.info("[CLD] publish OK  %s (ack in %lu ms, %u bytes)",
             event, (unsigned long)dt, (unsigned)bytes);
    Log.info("[CLD] PUB  %s: %s", event, payload);
  } else {
    g_pubFailed++;
    g_sessionWarnings++;
    g_cloudPublishOk = false;              // one failed chunk fails the report
    Log.warn("[CLD] publish FAILED %s: rejected -> g_cloudPublishOk=false "
             "(payload kept in the flash ring)", event);
  }
  return ok;
}

// ---- Bucket publish (doc 05 section 5.5) -----------------------------------
// The bucket count is variable, so the payload is variable too. Two rules the
// dashboard side depends on:
//   - only the VALID buckets are sent, never a padded fixed-length array;
//   - every message carries "how many could be made" (MM) alongside "how many
//     are actually being sent" (NN), so a truncated recovery is visible rather
//     than looking like a short but complete report.
// A Particle event carries at most 1 kB, so long series are split into chunks
// that each repeat the metadata and can therefore be interpreted alone.
static void publishHourlyBuckets() {
  if (!g_hourlyValid || g_hourly.count == 0u) {
    Log.info("[CLD] no completed buckets to publish this session");
    g_hourlySent = 0u;
    return;
  }

  const uint16_t per        = HOURLY_PUBLISH_PER_CHUNK;
  const uint16_t sendCount  = g_hourly.count;                       // NN
  const uint32_t makeable   = (g_hourlyMakeable > sendCount) ? g_hourlyMakeable : sendCount;  // MM
  const uint16_t totalChunk = (uint16_t)((sendCount + per - 1u) / per);

  // Bucket i covers [base + i*w, base + (i+1)*w), so the last one ends here.
  const uint32_t finalLocal = g_hourly.baseLocal + (uint32_t)sendCount * g_hourly.bucketSec;

  Log.info("[CLD] publishing %u of %lu makeable bucket(s) in %u chunk(s): %lu..%lu local",
           (unsigned)sendCount, (unsigned long)makeable, (unsigned)totalChunk,
           (unsigned long)g_hourly.baseLocal, (unsigned long)finalLocal);

  for (uint16_t c = 0; c < totalChunk; c++) {
    uint16_t start = (uint16_t)(c * per);
    uint16_t cnt   = (uint16_t)(sendCount - start);
    if (cnt > per) cnt = per;

    JsonWriterStatic<1024> jw;
    // V068: the AUDIT array keeps three decimals. The bench runs at ~0.3 gal/h,
    // where one decimal is a 7 % error and the conservation self-check cannot
    // close. Note this also fixes an existing defect: with no float-places set,
    // the library formats every float with "%f" - six decimals - so a bucket of
    // 12.4 was published as "12.400000". At 96 buckets per chunk that alone
    // overran the 1024-byte buffer and the payload was truncated silently.
    jw.setFloatPlaces(HOURLY_BUCKET_DECIMALS);
    {
      JsonWriterAutoObject obj(&jw);
      jw.insertKeyValue("platform",       PLATFORM_STR);
      jw.insertKeyValue("bucketSec",      (int)g_hourly.bucketSec);
      // Local epoch of the first bucket in the WHOLE series, and of the end of
      // the last one. Together with bucketSec they place every value exactly.
      jw.insertKeyValue("hourlyBaseLocal", (int)g_hourly.baseLocal);
      jw.insertKeyValue("hourlyFinalLocal",(int)finalLocal);
      // V068 section 4: the same two instants expressed as UTC. Every epoch in
      // this firmware is LOCAL (the offset is applied once, in TIME_SYNC), so a
      // UTC field must be CONVERTED, never merely renamed - renaming would put
      // the series 8 hours out of place with nothing in the payload to reveal it.
      // Publishing both makes them check each other:
      //   Utc - Local == -tzOffsetSec, always.
      jw.insertKeyValue("hourlyBaseUtc",  (int)((int32_t)g_hourly.baseLocal - g_tzOffsetSec));
      jw.insertKeyValue("hourlyFinalUtc", (int)((int32_t)finalLocal        - g_tzOffsetSec));
      jw.insertKeyValue("tzOffsetSec",    (int)g_tzOffsetSec);
      jw.insertKeyValue("spanStart",      (int)g_hourly.spanStartLocal);
      jw.insertKeyValue("spanEnd",        (int)g_hourly.spanEndLocal);
      jw.insertKeyValue("samplesUsed",    (int)g_hourly.samplesUsed);   // K
      jw.insertKeyValue("bucketsMakeable",(int)makeable);               // MM
      jw.insertKeyValue("bucketsSent",    (int)sendCount);              // NN
      jw.insertKeyValue("chunk",          (int)(c + 1));
      jw.insertKeyValue("totalChunks",    (int)totalChunk);
      jw.insertKeyValue("indexStart",     (int)start);
      jw.insertKeyValue("count",          (int)cnt);
      jw.insertKeyValue("equalized",      (int)(g_hourly.equalized ? 1 : 0));
      // V068: the array keeps its shape but moves to the "hourlyBuckets" event.
      // The name "hourlyGallons" now belongs to the contract's fixed 48-slot
      // window (publishHourlyRolling48). Same name + different shape is what
      // breaks a dashboard, so the two were separated rather than merged.
      jw.insertKeyArray("hourlyBuckets");
      for (uint16_t i = 0; i < cnt; i++)
        jw.insertArrayValue(roundDecimals(g_hourly.gal[start + i], HOURLY_BUCKET_DECIMALS));
      jw.finishObjectOrArray();
    }
    if (jw.isTruncated()) {
      Log.error("[CLD] hourlyBuckets chunk %u/%u was TRUNCATED at %u bytes - the "
                "payload is incomplete. Lower HOURLY_PUBLISH_PER_CHUNK.",
                (unsigned)(c + 1), (unsigned)totalChunk,
                (unsigned)jw.getBufferLen());
      g_sessionWarnings++;
    }
    if (!cloudEmit("hourlyBuckets", jw.getBuffer())) {
      Log.warn("[CLD] bucket chunk %u/%u not delivered - the whole report stays buffered",
               (unsigned)(c + 1), (unsigned)totalChunk);
    }
    if (g_cfg.cloudEnabled) delay(1100);            // cloud rate limit is about one event per second
    picKeepalivePump();                           // and the PIC must not see us go quiet for 20 s
  }
  g_hourlySent = sendCount;
}

// ---- V068: the report grid, as the PIC is ACTUALLY RUNNING it ---------------
// g_grid is what we ASKED for; under PIC_USE_OWN_TIMING the PIC ACKs SET_GRID
// and then keeps its own value, and says so in its log ("SET_GRID ... ->
// IGNORED"). Publishing the request instead of the observation is how V064
// reported gridIntervalSec=1800 while reports actually arrived every 180 s - a
// 10x error that stood because nothing cross-checked it. Every contract field
// derived from the grid goes through this one accessor so the same mistake
// cannot be made once per field.
static inline void contractGrid(uint32_t &intervalSec, uint32_t &anchorSec) {
  intervalSec = g_gridFromPicValid ? g_gridFromPic.intervalSec : g_grid.intervalSec;
  anchorSec   = g_gridFromPicValid ? g_gridFromPic.anchorSec   : g_grid.anchorSec;
}

// ---- V068: the contract's fixed 48-slot "hourlyGallons" event --------------
//
// Req 5: local-time bins [0-47], up to 48 completed hours. Slot 47 is the most
// recently CLOSED hour; the still-open hour is HourlyrReset, not [47]. A new
// completed hour rolls existing values toward slot 0 (no midnight wipe). Hours
// older than 48 fall off. There is no [0-23] publish.
//
// One message, one meaning: THIS IS THE STATE OF THE LAST 48 BUCKETS. It is
// deliberately not chunked - splitting it would destroy exactly that property,
// and there is no way to read half a sliding window. If it ever grows too large
// the answer is fewer decimals (ROLL48_DECIMALS), not more messages.
//
// slotsFilled and slotsGapFilled ride along because a 0 in the array has three
// possible meanings and the array alone cannot tell them apart:
//   a newly installed device      - no data yet
//   a device that missed 40 h     - data lost
//   a house that used no water    - a real zero
// With the two counters, the reader can tell which. Without them a dashboard
// draws "we don't know" and "no water was used" as the same picture.
static void publishHourlyRolling48() {
  const uint32_t bucketSec = (g_roll48BucketSec > 0u) ? g_roll48BucketSec : g_bucketSec;

  // Bucket boundaries of the window: slot 47 starts at g_roll48NewestLocal, so
  // slot 0 starts 47 buckets earlier and the window ends one bucket after 47.
  uint32_t baseLocal  = 0u;
  uint32_t finalLocal = 0u;
  if (g_roll48NewestLocal > 0u) {
    const uint32_t span = (uint32_t)(ROLL48_COUNT - 1) * bucketSec;
    baseLocal  = (g_roll48NewestLocal > span) ? (g_roll48NewestLocal - span) : 0u;
    finalLocal = g_roll48NewestLocal + bucketSec;
  }

  uint32_t iv = 0u, an = 0u;
  contractGrid(iv, an);

  // The open (incomplete) bucket. The agreement calls this "current incompleted
  // hour bin gallon"; it is the value this firmware already computes and logs as
  // the carry, so nothing new is measured here - it is only published.
  const float carryGal = (hourlyCarry.magic == HOURLY_CARRY_MAGIC && hourlyCarry.valid)
                       ? hourlyCarry.gallons : 0.0f;

  JsonWriterStatic<1024> jw;
  jw.setFloatPlaces(ROLL48_DECIMALS);
  {
    JsonWriterAutoObject obj(&jw);
    jw.insertKeyValue("pf",              PLATFORM_STR);
    jw.insertKeyValue("bucketSec",       (int)bucketSec);
    // Section 4: UTC is a CONVERSION of our local axis, never a rename of it,
    // and both are published so that Utc - Local == -tzOffsetSec can be checked.
    jw.insertKeyValue("hourlyBaseUtc",   (int)((int32_t)baseLocal  - g_tzOffsetSec));
    jw.insertKeyValue("hourlyFinalUtc",  (int)((int32_t)finalLocal - g_tzOffsetSec));
    jw.insertKeyValue("hourlyBaseLocal", (int)baseLocal);
    jw.insertKeyValue("hourlyFinalLocal",(int)finalLocal);
    jw.insertKeyValue("tzOffsetSec",     (int)g_tzOffsetSec);
    // Contract spelling. "HourlyrReset" carries a stray 'r' and looks like a
    // typo, but the client's dashboard may already read that exact key, so it
    // stays until they confirm otherwise (client question 6).
    jw.insertKeyValue("HourlyrReset",    (double)roundDecimals(carryGal, ROLL48_DECIMALS));
    jw.insertKeyValue("hourlyDayUtc",    (int)(an / 3600u));
    jw.insertKeyValue("reportIntervalHr",(int)(iv / 3600u));
    // Integer division makes any sub-hour interval read as 0, and 0 is also how
    // "not configured" reads - so the bench (180 s) would publish a value that
    // is both wrong and ambiguous. The seconds figure alongside keeps the truth.
    jw.insertKeyValue("reportIntervalSec",(int)iv);
    jw.insertKeyValue("slotsFilled",     (int)g_roll48Filled);
    jw.insertKeyValue("slotsGapFilled",  (int)g_roll48Gaps);
    jw.insertKeyArray("hourlyGallons");
    for (uint8_t i = 0; i < ROLL48_COUNT; i++)
      jw.insertArrayValue(roundDecimals(g_roll48Gal[i], ROLL48_DECIMALS));
    jw.finishObjectOrArray();
  }

  const size_t bytes = jw.getOffset();
  if (jw.isTruncated()) {
    Log.error("[CLD] hourlyGallons was TRUNCATED - the 48-slot window did not fit "
              "the %u-byte buffer. Reduce ROLL48_DECIMALS.",
              (unsigned)PARTICLE_EVENT_MAX_BYTES);
    g_sessionWarnings++;
  } else if (bytes > CONTRACT_EVENT_WARN_BYTES) {
    Log.warn("[CLD] hourlyGallons is %u bytes, past the %u-byte warning line "
             "(Particle limit %u). Reduce ROLL48_DECIMALS before it truncates.",
             (unsigned)bytes, (unsigned)CONTRACT_EVENT_WARN_BYTES,
             (unsigned)PARTICLE_EVENT_MAX_BYTES);
    g_sessionWarnings++;
  }

  Log.info("[R48] window %lu..%lu local (bucket=%lus) filled=%u gap=%lu carry=%.3f gal, %u bytes",
           (unsigned long)baseLocal, (unsigned long)finalLocal, (unsigned long)bucketSec,
           (unsigned)g_roll48Filled, (unsigned long)g_roll48Gaps, (double)carryGal,
           (unsigned)bytes);

  cloudEmit("hourlyGallons", jw.getBuffer());
  if (g_cfg.cloudEnabled) delay(1100);            // cloud rate limit is about one event per second
  picKeepalivePump();
}

void publishIntervalDataChunks() {
  const uint16_t per = 120;                       // How many samples go in each chunk message.
  if (gMeter.count == 0) return;                  // Nothing logged yet -> nothing to publish.
  uint16_t total = (gMeter.count + per - 1) / per;   // Total number of chunks (round up the division).

  for (uint16_t c = 0; c < total; c++) {          // For each chunk index c...
    uint16_t start = c * per;                     // The first sample index in this chunk.
    uint16_t cnt = gMeter.count - start;          // How many samples remain from here.
    if (cnt > per) cnt = per;                     // Cap this chunk at 'per' samples.

    char packed[(120 * 3) + 1];                   // Text buffer: up to 120 samples * 3 chars + 1 for the end marker.
    size_t pos = 0;                               // Current write position in 'packed'.
    for (uint16_t i = 0; i < cnt; i++) {          // For each sample in this chunk...
      pos += snprintf(&packed[pos], sizeof(packed) - pos, "%03u",   // ...append it as a 3-digit number ("007", "123").
                      (unsigned)gMeter.raw[start + i]);
      if (pos >= sizeof(packed) - 1) break;       // Safety: stop if the text buffer is full.
    }
    packed[pos] = '\0';                           // Terminate the string with a null character.

    // V068: 512 -> 640. This payload is all integers plus a packed string, but
    // that string alone is 120 samples x 3 characters = 360 bytes, and the
    // metadata around it runs to about 130 - roughly 500 against a 512-byte
    // buffer, with no margin and no warning if it were exceeded. The float
    // precision is set even though nothing here is a float, so that the rule
    // "every publish writer states its precision" has no exceptions to
    // remember; check.sh enforces it.
    JsonWriterStatic<640> jw;
    jw.setFloatPlaces(HOURLY_BUCKET_DECIMALS);
    {                                             // Inner scope so the JSON object auto-closes at the brace.
      JsonWriterAutoObject obj(&jw);              // Begin a JSON object { ... } (auto-finished when 'obj' ends).
      jw.insertKeyValue("platform", PLATFORM_STR);          // "platform": board name.
      jw.insertKeyValue("day0Utc", (int)gMeter.day0_utc_midnight);   // "day0Utc": window start, UTC seconds.
      jw.insertKeyValue("intervalSec",                               // RUNTIME capture interval (from the PIC),
                        (int)(g_cfg.captureIntervalSecF + 0.5f));    //   NOT the compile-time guess.
      jw.insertKeyValue("droppedOldest", (int)gMeter.start_slot);    // samples dropped by the ring (0 = none).
      jw.insertKeyValue("chunk", (int)(c + 1));             // "chunk": this chunk's number (1-based).
      jw.insertKeyValue("totalChunks", (int)total);         // "totalChunks": how many chunks total.
      jw.insertKeyValue("sampleStart", (int)start);         // "sampleStart": index of the first sample here.
      jw.insertKeyValue("sampleCount", (int)cnt);           // "sampleCount": how many samples in this chunk.
      jw.insertKeyValue("data", packed);                    // "data": the packed 3-digit sample string.
    }
    if (jw.isTruncated()) {
      Log.error("[CLD] meterIntervals chunk %u/%u was TRUNCATED - the sample string "
                "did not fit and the JSON is incomplete.",
                (unsigned)(c + 1), (unsigned)total);
      g_sessionWarnings++;
    }
    cloudEmit("meterIntervals", jw.getBuffer());  // Cloud: publish; bench: log the exact chunk payload.
    if (g_cfg.cloudEnabled) delay(1100);            // (cloud) Wait 1.1 s between publishes (cloud rate-limits ~1/sec).
    picKeepalivePump();                           // ...and keep the PIC's 20 s idle backstop from firing.
  }
}

// Build and publish the main "sensorData" status message to the cloud.
void imuPublish() {
  imuGet();                                       // Refresh IMU values first (no-op if no IMU).

  if (g_cfg.cloudEnabled) {                          // (cloud) ensure the radio is up before publishing; bench skips.
#if USE_WIFI
    if (!WiFi.ready()) { WiFi.connect(); if (!waitFor(WiFi.ready, WIFI_CONNECT_TIMEOUT_MS)) { Log.error("WiFi fail"); return; } }
#elif USE_CELLULAR
    if (!Cellular.ready()) { Cellular.connect(); if (!waitFor(Cellular.ready, CELL_CONNECT_TIMEOUT_MS)) { Log.error("Cell fail"); return; } }
#endif
  }

  // V068: 512 -> 1024, and an explicit float precision.
  //
  // With no float-places set, JsonWriter formats floats with "%f" (six decimals).
  // Combined with the old 24-slot hourlyRolling24 that overran 512 bytes and the
  // library silently dropped the closing brace. The 24-slot array is no longer
  // published (req 5). Three places keeps flowCal (1.255) exact. The warning
  // below exists so a future field cannot silently truncate again.
  JsonWriterStatic<1024> jw;
  jw.setFloatPlaces(HOURLY_BUCKET_DECIMALS);
  {                                               // Inner scope so the JSON object auto-finishes.
    JsonWriterAutoObject obj(&jw);                // Begin the JSON object.
    jw.insertKeyValue("platform", PLATFORM_STR);  // Board name.
    jw.insertKeyValue("sensor", imu_data.sensor); // IMU address (0 if none).
    jw.insertKeyValue("leaking", (int)leakingEventCount);    // Number of leak events this cycle.
    jw.insertKeyValue("shutoff", (int)shutoffEventCount);    // Number of shutoff events this cycle.
    jw.insertKeyValue("overflow", (int)overflowEventCount);  // Number of overflow events this cycle.
    jw.insertKeyValue("temp", imu_data.temperature);         // Current temperature.
    jw.insertKeyValue("flowCal", flowCalScale);              // Current flow calibration scale.
    // Echo the active host config back so the dashboard can confirm it.
    jw.insertKeyValue("cfgLeakGpm", appConfig.leakThreshGpm);        // Host leak threshold (GPM).
    jw.insertKeyValue("cfgShutoffVol", appConfig.shutoffVolGal);     // Host 30-min shutoff volume.
    jw.insertKeyValue("cfgAutoShutoff", (int)appConfig.autoShutoff); // Host auto-shutoff on/off.
    jw.insertKeyValue("cfgAlertMode", (int)appConfig.alertMode);     // Host alert mode.
    // PIC leak parameters (REQ_GET/SET_PARAM) + delivery state.
    jw.insertKeyValue("picLeak1Counts", (int)picParams.leak1_counts);   // PIC alert-1 counts.
    jw.insertKeyValue("picLeak1WinS",  (int)picParams.leak1_window_s);  // PIC alert-1 window seconds.
    jw.insertKeyValue("picLeak2Counts", (int)picParams.leak2_counts);   // PIC alert-2 counts.
    jw.insertKeyValue("picLeak2WinS",  (int)picParams.leak2_window_s);  // PIC alert-2 window seconds.
    jw.insertKeyValue("picParamsDirty", (int)picParamsDirty);   // 1 = a write to the PIC is still pending.
    // PIC valve subsystem status (REQ_GET_VALVE), if we have read it.
    if (haveValve) {                              // Only include valve fields if we've read them at least once.
      jw.insertKeyValue("valveMotion",   (int)lastValve.motion);         // Valve motion state (0..6).
      jw.insertKeyValue("valveLockFlags",(int)lastValve.lock_flags);     // Which locks are active.
      jw.insertKeyValue("valvePwr",      (int)lastValve.pwr_pin);        // Valve power pin level.
      jw.insertKeyValue("valveCtrl",     (int)lastValve.ctrl_pin);       // Valve control pin level.
      jw.insertKeyValue("valveTempLocks",(int)lastValve.temp_lock_count);// Cumulative temp-lock count.
    }
    jw.insertKeyValue("bucketSec", (int)g_bucketSec);   // Width of one bucket (3600 = true hourly).
    // The variable-length series is "hourlyBuckets". The contract 48-slot window
    // is "hourlyGallons". Counts here describe the audit series for this session.
    jw.insertKeyValue("bucketsMakeable", (int)g_hourlyMakeable);
    jw.insertKeyValue("bucketsSent",     (int)g_hourlySent);
    jw.insertKeyValue("tzOffsetSec",     (int)g_tzOffsetSec);
    // V065 (req 6): publish the grid the PIC is ACTUALLY RUNNING, not the one we
    // asked for. Under PIC_USE_OWN_TIMING the PIC ACKs SET_GRID and then keeps
    // its own value - by design, and the PIC log says so ("SET_GRID ... ->
    // IGNORED"). We already read the real value back (g_gridFromPic, captured in
    // syncGridWithPic) and then published the request anyway, so a bench run
    // reported gridIntervalSec=1800 while reports were actually arriving every
    // 180 s. Nothing downstream in THIS firmware consumed that field - the bucket
    // engine is driven entirely by bucketSec and the per-block span timestamps,
    // never by the grid interval - so no bucket was ever misplaced by it. But it
    // is the number a cloud-side consumer would reach for first, and it was off
    // by 10x. Publish the observed value when we have one; fall back to our
    // request only when the PIC has not told us.
    const uint32_t pubGridAnchor   = g_gridFromPicValid ? g_gridFromPic.anchorSec
                                                        : g_grid.anchorSec;
    const uint32_t pubGridInterval = g_gridFromPicValid ? g_gridFromPic.intervalSec
                                                        : g_grid.intervalSec;
    jw.insertKeyValue("gridAnchorSec",   (int)pubGridAnchor);
    jw.insertKeyValue("gridIntervalSec", (int)pubGridInterval);
    jw.insertKeyValue("gridFromPic",     (int)(g_gridFromPicValid ? 1 : 0));
    jw.insertKeyValue("carryPulses",     (int)hourlyCarry.pulses);

    // ==== V068 contract fields (Project Agreement 2026-07-18 sec 4) =========
    // The contract names several fields this firmware already publishes under
    // different names. BOTH names are sent. The existing dashboard may be
    // reading the old ones, and each duplicate costs about 20 bytes, so the
    // cheap option and the safe option are the same option; dropping a name a
    // live consumer reads is the only expensive mistake available here.
    jw.insertKeyValue("pf",      PLATFORM_STR);                       // == platform
    jw.insertKeyValue("Cal",     flowCalScale);                       // == flowCal
    jw.insertKeyValue("a1Count", (int)picParams.leak1_counts);        // == picLeak1Counts
    jw.insertKeyValue("a1Win",   (int)picParams.leak1_window_s);      // == picLeak1WinS
    jw.insertKeyValue("a2Count", (int)picParams.leak2_counts);        // == picLeak2Counts
    jw.insertKeyValue("a2Win",   (int)picParams.leak2_window_s);      // == picLeak2WinS

    // Grid-derived contract fields. iv/an come from the PIC's OBSERVED grid, not
    // the one we requested (see contractGrid).
    uint32_t ivC = 0u, anC = 0u;
    contractGrid(ivC, anC);
    jw.insertKeyValue("reportIntervalHr",  (int)(ivC / 3600u));
    jw.insertKeyValue("reportIntervalSec", (int)ivC);   // sub-hour intervals floor to 0 in the line above
    jw.insertKeyValue("hourlyDayUtc",      (int)(anC / 3600u));

    // Next scheduled wake, as an absolute epoch. The agreement's text says
    // "seconds" but its field name says Epoch; an epoch is published because a
    // remaining-seconds count is only true at the instant it is read, and this
    // record may be queried long afterwards. Documented for client confirmation.
    uint32_t nUtc = 0u;
    if (clockNowUtc(nUtc) && ivC > 0u) {
      const int32_t  loc  = (int32_t)nUtc + g_tzOffsetSec;
      const uint32_t rel  = (uint32_t)(loc - (int32_t)anC);
      const uint32_t next = (uint32_t)loc + (ivC - (rel % ivC));
      nextPublishEpoch = (unsigned long)((int32_t)next - g_tzOffsetSec);
      jw.insertKeyValue("nextPublishEpoch", (int)nextPublishEpoch);
    } else {
      jw.insertKeyValue("nextPublishEpoch", (int)0);    // no clock -> no schedule to state
    }

    // Leak events over the window they actually cover. -1 is NOT 0: it means
    // "could not be counted" (first session, PIC reboot, or no clock), and a
    // dashboard must not draw it as "no leak occurred".
    const bool a1Ok = (g_a1WindowValid != 0u);
    jw.insertKeyValue("a1Events", a1Ok ? (int)g_a1EventsWindow : (int)-1);
    uint32_t winSec = 0u;
    if (a1Ok && g_a1WindowStartUtc != 0u) {
      uint32_t nowW = 0u;
      if (clockNowUtc(nowW) && nowW > g_a1WindowStartUtc) winSec = nowW - g_a1WindowStartUtc;
    }
    jw.insertKeyValue("a1WindowSec", (int)winSec);
    // ALERT2 is a permanent lock: the PIC keeps a flag, not a counter, and that
    // is the right shape - a permanent lock cannot re-trigger until a human
    // clears it with unlockValve, so "how many times today" has no meaning.
    jw.insertKeyValue("a2Events",
        (int)((haveValve && (lastValve.lock_flags & VALVE_LOCK_PERM)) ? 1 : 0));
    // Triggered T/F since the last publish. A boolean needs no clock, so this
    // field is correct even in a session that never reached the cloud.
    jw.insertKeyValue("Shutoffs", (int)g_shutoffSeen);

    // Lifetime total. See the declaration for why this is the power-cycle-
    // surviving reading rather than the since-power-up one.
    jw.insertKeyValue("lifetimeGal", (double)g_lifetimeGal);
    // Req 5: do not publish [0-23]. The contract array is hourlyGallons[48]
    // (+ HourlyrReset) in publishHourlyRolling48().
    if (!g_cfg.cloudEnabled) {
      jw.insertKeyValue("rssi", (int)0);          // Bench: no radio; placeholder so the payload shape matches.
    } else {
#if USE_WIFI
      jw.insertKeyValue("rssi", (int)WiFi.RSSI().getStrength());        // Wi-Fi signal strength.
#elif USE_CELLULAR
      jw.insertKeyValue("rssi", (int)Cellular.RSSI().getStrength());    // Cellular signal strength.
#else
      jw.insertKeyValue("rssi", (int)0);
#endif
    }
#if HAS_FUEL_GAUGE
    FuelGauge fuel;                               // On Gen3 boards, create a fuel-gauge reader...
    jw.insertKeyValue("battery", fuel.getSoC());                     // Gen3 fuel gauge (% SoC)
                                                  //   ...and publish battery state-of-charge as a percentage.
#else
    jw.insertKeyValue("battery", analogRead(BATTERY_PIN) / 819.2f);  // P2 ADC divider
                                                  //   On P2, read the analog pin and scale it to a voltage.
#endif
    jw.insertKeyValue("freeMem", (int)System.freeMemory());   // Free RAM (helps debug memory issues).
    jw.insertKeyValue("uptime", (int)System.uptime());        // Seconds since boot.
  }

  // V068: a truncated status payload used to be invisible. It is now stated.
  if (jw.isTruncated()) {
    Log.error("[CLD] sensorData was TRUNCATED - the payload did not fit its buffer "
              "and the JSON is incomplete. Fields must be dropped or shortened.");
    g_sessionWarnings++;
  } else if (jw.getOffset() > PARTICLE_EVENT_MAX_BYTES) {
    Log.error("[CLD] sensorData is %u bytes, past the %u-byte Particle event limit.",
              (unsigned)jw.getOffset(), (unsigned)PARTICLE_EVENT_MAX_BYTES);
    g_sessionWarnings++;
  }

  cloudEmit("sensorData", jw.getBuffer());        // Cloud: publish + log; bench: log the exact payload (no transmit).
  if (g_cfg.cloudEnabled) delay(1100);            // cloud rate limit is about one event per second
  publishHourlyRolling48();                       // V068: the contract's fixed 48-slot window (one event).
  publishHourlyBuckets();                         // Then the variable-length audit series (MM / NN metadata).
  publishIntervalDataChunks();                    // Then publish/log the detailed interval logger in chunks.

  // V068: close the leak-event window on the SAME values that were just
  // published, so a count is never reported twice and never dropped between the
  // publish and the reset.
  {
    uint32_t nowU = 0u;
    const bool tOk = clockNowUtc(nowU);
    leakEventsAfterPublish(tOk, nowU);
  }

  // Roll the interval logger AFTER it has been emitted. Everything in the buffer
  // was just published (or, on the bench, fully logged), so keeping it would make
  // the next session re-send the same samples - V055 re-published the entire
  // buffer every session, duplicating rows in the cloud. The new window starts at
  // the current time.
  if (gMeter.count > 0) {
    // Appendix H.3: the old code read a bare Time.now here. In a no-time
    // session that returns the 2000-01-01 default (946684800) and stamped it
    // into gMeter.day0_utc_midnight - one of the forbidden-band leaks H.3.3
    // bans. Take the value only from the checked gate; when there is no clock,
    // the window start stays 0 (an explicit "no absolute time"), never a fake.
    uint32_t now = 0u;
    (void)clockNowUtc(now);
    gMeter.day0_utc_midnight = now;               // window start = this publish (0 if no clock)
    gMeter.start_slot = 0;                        // dropped-sample counter restarts
    gMeter.count = 0;                             // buffer emptied for the next window
    Log.info("[DAT] interval logger rolled (window start=%lu%s)",
             (unsigned long)now, now ? "" : " - no absolute time, relative window");
  }
  persistAll();                                   // Save everything to flash now that we've published.

  leakingEventCount = shutoffEventCount = overflowEventCount = 0;   // Clear the per-cycle event counters.
}

// =============================================================== Shutoff valve
// Timer callback: when the 10 s valve-power window ends, ask the main loop to reset.
void shutoffTimerCb() { resetShutoff = true; triggerPublish = true; }   // Set flags for loop() to act on.
Timer shutoffTimer(SHUTOFF_TIMER_MS, shutoffTimerCb, true);   // A one-shot 10 s timer that calls shutoffTimerCb.

// Cloud function: control the LOCAL valve. cmd = "close" | "open" | "off".
int shutoffSwitch(String cmd) {
  restartSleepTimer("shutoffSwitch");             // Any cloud action resets the awake window so we don't sleep mid-task.
  cmd.trim(); cmd.toLowerCase();                  // Clean up the command: remove spaces, make lowercase.
  if (cmd == "close") {                           // "close" = drive the valve shut.
    digitalWrite(LED1_PIN, HIGH); digitalWrite(SHUTOFF_SWITCH_PIN, HIGH); digitalWrite(SHUTOFF_SSR_PIN, HIGH);   // LED on, direction=close, power on.
    if (!imu_data.shutoff) shutoffEventCount++;   // Count a new shutoff event (only on the transition).
    imu_data.shutoff = true; triggerPublish = true; shutoffTimer.start(); return 2;   // Mark shut, publish, start 10 s timer, return 2.
  } else if (cmd == "open") {                     // "open" = drive the valve open.
    digitalWrite(LED1_PIN, HIGH); digitalWrite(SHUTOFF_SWITCH_PIN, LOW); digitalWrite(SHUTOFF_SSR_PIN, HIGH);    // LED on, direction=open, power on.
    imu_data.shutoff = false; triggerPublish = true; shutoffTimer.start(); return 1;   // Mark open, publish, start timer, return 1.
  } else if (cmd == "off") {                      // "off" = remove power from the valve (idle/safe).
    digitalWrite(LED1_PIN, LOW); digitalWrite(SHUTOFF_SWITCH_PIN, LOW); digitalWrite(SHUTOFF_SSR_PIN, LOW);      // LED off, direction low, power off.
    shutoffTimer.stop(); return 0;                // Stop the timer, return 0.
  }
  return -1;                                       // Unknown command -> error.
}

// Cloud function: manually set or clear the host "leaking" flag. cmd = "on" | "off" | "reset".
int leakingSwitch(String cmd) {
  restartSleepTimer("leakingSwitch");             // Reset the awake window.
  cmd.trim(); cmd.toLowerCase();                  // Normalize the command text.
  if (cmd == "on")  { if (!imu_data.leaking) leakingEventCount++; imu_data.leaking = true;  triggerPublish = true; return 1; }   // Force "leaking" on.
  if (cmd == "off" || cmd == "reset") { imu_data.leaking = false; leakRunLen = 0; triggerPublish = true; return 0; }            // Clear "leaking" and the run counter.
  return -1;                                       // Unknown command -> error.
}

// =============================================================== Cloud config
// Cloud function: set the flow calibration scale. cmd = a number like "1.25".
int setFlowCal(String cmd) {
  restartSleepTimer("setFlowCal");                // Reset the awake window.
  cmd.trim();                                     // Trim surrounding spaces.
  float v = atof(cmd.c_str());                    // Convert the text to a floating-point number.
  if (v != v || v < FLOW_CAL_MIN || v > FLOW_CAL_MAX) return -1;   // Reject NaN (v!=v) or out-of-range values.
  flowCalScale = v; saveFlowCal(); syncBackupRam(); triggerPublish = true;   // Accept it, save it, flush RAM, publish.
  Log.info("FLOW_CAL set %.4f", flowCalScale);    // Log the new value.
  return 1;                                        // Success.
}
// Cloud function: report the current flow calibration as an integer (scale * 1000).
int getFlowCal(String cmd) { (void)cmd; return (int)(flowCalScale * 1000.0f + 0.5f); }   // (void)cmd = "argument unused".

// setConfig: "leakGpm,shutoffVol,autoShutoff,alertMode"
// HOST-SIDE analytics + local SSR valve only. In V040 these no longer travel to
// the PIC (the old 0xC0 frame is gone). To configure the PIC's own leak logic
// use setLeakParams (REQ_SET_LEAK). Kept for dashboard backward-compat.
int setConfig(String cmd) {
  restartSleepTimer("setConfig");                 // Reset the awake window.
  cmd.trim();                                     // Trim spaces.
  int c1 = cmd.indexOf(','), c2 = cmd.indexOf(',', c1 + 1), c3 = cmd.indexOf(',', c2 + 1);   // Find the 3 commas.
  if (c1 < 0 || c2 < 0 || c3 < 0) { Log.warn("setConfig: need 4 CSV values"); return -1; }   // Need exactly 4 values.

  float   leak = atof(cmd.substring(0, c1).c_str());        // Value 1: leak threshold (text before comma 1) -> float.
  float   vol  = atof(cmd.substring(c1 + 1, c2).c_str());   // Value 2: shutoff volume.
  int     aut  = atoi(cmd.substring(c2 + 1, c3).c_str());   // Value 3: auto-shutoff (0/1) -> integer.
  int     alt  = atoi(cmd.substring(c3 + 1).c_str());       // Value 4: alert mode (0/1/2).

  if (leak < CFG_LEAK_GPM_MIN || leak > CFG_LEAK_GPM_MAX) return -2;   // Range-check the leak threshold.
  if (vol  < CFG_SHUTOFF_MIN  || vol  > CFG_SHUTOFF_MAX)  return -3;   // Range-check the volume.
  if (aut < 0 || aut > 1)  return -4;             // auto-shutoff must be 0 or 1.
  if (alt < 0 || alt > 2)  return -5;             // alert mode must be 0, 1, or 2.

  appConfig.leakThreshGpm = leak;                 // Store the accepted leak threshold.
  appConfig.shutoffVolGal = vol;                  // Store the volume threshold.
  appConfig.autoShutoff   = (uint8_t)aut;         // Store auto-shutoff.
  appConfig.alertMode     = (uint8_t)alt;         // Store alert mode.
  saveConfig(); syncBackupRam();                  // Save to EEPROM and flush retained RAM.
  triggerPublish = true;                          // Publish the updated config.
  Log.info("CFG set leak=%.2f vol=%.1f auto=%d alert=%d", leak, vol, aut, alt);   // Log the new config.
  return 1;                                        // Success.
}

// Cloud function: just trigger a publish of the current config (no input used).
int getConfig(String cmd) { (void)cmd; triggerPublish = true; return 1; }

// ---- PIC leak parameters (REQ_GET_PARAM / REQ_SET_LEAK) --------------------
// Push the cached leak parameters to the PIC. Returns true on ACK (spec 5.4).
// Appendix E: this is the leak-only write (0x03, len 8); the report grid is a
// separate packet (0x0F) handled by syncGridWithPic().
bool pushPicParams() {
  if (picLink.setLeak(picParams)) {               // Try to write the cached leak params to the PIC.
    picParamsDirty = false;                        // On ACK, clear the "still owe a write" flag.
    Log.info("[PARM] SET_LEAK delivered (ACK) a1=%u/%us a2=%u/%us",
             picParams.leak1_counts, picParams.leak1_window_s,
             picParams.leak2_counts, picParams.leak2_window_s);
    return true;                                   // Report success.
  }
  Log.warn("[PARM] SET_LEAK failed (nak=0x%02X)", picLink.lastNak());   // Log the failure + NAK reason.
  return false;                                    // Report failure (stays dirty for a later retry).
}

// setLeakParams: "leak1_counts,leak1_window_s,leak2_counts,leak2_window_s".
// Spec mandates read-modify-write of all four. We refresh from the PIC first
// (if reachable) so we never clobber a field we weren't asked to change, then
// apply the new values and SET all four.
int setLeakParams(String cmd) {
  restartSleepTimer("setLeakParams");             // Reset the awake window.
  cmd.trim();                                     // Trim spaces.
  int c1 = cmd.indexOf(','), c2 = cmd.indexOf(',', c1 + 1), c3 = cmd.indexOf(',', c2 + 1);   // Find the 3 commas.
  if (c1 < 0 || c2 < 0 || c3 < 0) { Log.warn("setLeakParams: need 4 CSV values"); return -1; }   // Need 4 values.

  long l1c = atol(cmd.substring(0, c1).c_str());        // Value 1: leak1 counts (text -> long integer).
  long l1w = atol(cmd.substring(c1 + 1, c2).c_str());   // Value 2: leak1 window seconds.
  long l2c = atol(cmd.substring(c2 + 1, c3).c_str());   // Value 3: leak2 counts.
  long l2w = atol(cmd.substring(c3 + 1).c_str());       // Value 4: leak2 window seconds.

  if (l1c < PIC_COUNTS_MIN || l1c > PIC_COUNTS_MAX) return -2;   // Range-check leak1 counts.
  if (l1w < PIC_WINDOW_MIN || l1w > PIC_WINDOW_MAX) return -3;   // Range-check leak1 window.
  if (l2c < PIC_COUNTS_MIN || l2c > PIC_COUNTS_MAX) return -4;   // Range-check leak2 counts.
  if (l2w < PIC_WINDOW_MIN || l2w > PIC_WINDOW_MAX) return -5;   // Range-check leak2 window.

  // Read-modify-write: pull the live set first if the PIC answers.
  PicParams live;                                 // A place to hold the PIC's current values.
  if (picLink.getParams(live)) picParams = live;  // If the PIC responds, start from its live values.

  picParams.leak1_counts   = (uint16_t)l1c;       // Apply the new leak1 counts.
  picParams.leak1_window_s = (uint16_t)l1w;       // Apply the new leak1 window.
  picParams.leak2_counts   = (uint16_t)l2c;       // Apply the new leak2 counts.
  picParams.leak2_window_s = (uint16_t)l2w;       // Apply the new leak2 window.
  savePicParams(); syncBackupRam();               // Save the updated params to EEPROM and flush RAM.
  picParamsDirty = true;                          // Mark that the PIC needs this new write.
  triggerPublish = true;                          // Request a publish so the dashboard sees the change.
  Log.info("PICP set l1=%ld/%lds l2=%ld/%lds", l1c, l1w, l2c, l2w);   // Log the new params.

  pushPicParams();   // try now; if it fails it stays dirty for next contact
                                                  //   Attempt delivery immediately; on failure it remains dirty.
  readValveStatus(); // confirm any lock the new threshold may have tripped
                                                  //   Re-read valve state in case the new thresholds locked it.
  return picParamsDirty ? 0 : 1;   // 1 = delivered, 0 = queued
                                                  //   Tell the caller whether the write went through (1) or is queued (0).
}

// getLeakParams: refresh the cached params straight from the PIC, then publish.
int getLeakParams(String cmd) {
  (void)cmd;                                      // Argument unused.
  restartSleepTimer("getLeakParams");             // Reset the awake window.
  PicParams live;                                 // Holder for the PIC's current values.
  if (picLink.getParams(live)) {                  // If the PIC responds...
    picParams = live; savePicParams(); syncBackupRam();   // ...update + save the cache, flush RAM.
    triggerPublish = true;                        // Publish the refreshed values.
    return 1;                                      // Success.
  }
  triggerPublish = true;   // publish the cached copy at least
                                                  //   PIC didn't answer: still publish the cached copy.
  return -1;                                       // Report that the live read failed.
}

// ---- Valve (REQ_GET_VALVE / REQ_VALVE_UNLOCK) ------------------------------
// Read the PIC valve status and remember it for later publishing/logging.
void readValveStatus() {
  PicValve v;                                     // Holder for the valve status.
  if (picLink.getValve(v)) {                      // If the PIC returns valve status...
    lastValve = v; haveValve = true;              // ...store it and note that we now have valid valve data.
    Log.info("VALVE pwr=%u ctrl=%u motion=%u lock=0x%02X tempLocks=%lu",   // Log the valve state.
             v.pwr_pin, v.ctrl_pin, v.motion, v.lock_flags,
             (unsigned long)v.temp_lock_count);

    // V068: temp_lock_count is the ONLY leak-event source the PIC sends, and it
    // is a cumulative total with no timestamps. Differencing it here - at every
    // valve read, not only at publish - is what turns it into "events since the
    // last publish". No PIC change is involved; the source was already on the
    // wire.
    uint32_t nowU = 0u;
    const bool tOk = clockNowUtc(nowU);
    leakEventsUpdate(v.temp_lock_count, tOk, nowU);

    // Shutoffs is Triggered T/F since the last publish, so a single observation
    // of either lock latches it. No clock is needed for a boolean, which is why
    // this field stays correct in a session that never reaches the cloud.
    if (v.lock_flags & (VALVE_LOCK_TEMP | VALVE_LOCK_PERM)) g_shutoffSeen = 1u;
  }
}

// Cloud function: read valve status now and publish it.
int getValve(String cmd) {
  (void)cmd;                                      // Argument unused.
  restartSleepTimer("getValve");                  // Reset the awake window.
  readValveStatus();                              // Read the valve status from the PIC.
  triggerPublish = true;                          // Request a publish.
  return haveValve ? 1 : -1;                      // 1 if we have valve data, else -1.
}

// unlockValve: "temp" | "perm" | "both" (or 1/2/3). Clears the PIC valve lock.
int unlockValve(String cmd) {
  restartSleepTimer("unlockValve");               // Reset the awake window.
  cmd.trim(); cmd.toLowerCase();                  // Normalize the command text.
  uint8_t flags;                                  // Which lock(s) to clear.
  if (cmd == "temp" || cmd == "1")      flags = VALVE_LOCK_TEMP;   // Clear the temporary lock.
  else if (cmd == "perm" || cmd == "2") flags = VALVE_LOCK_PERM;   // Clear the permanent lock.
  else if (cmd == "both" || cmd == "3") flags = VALVE_LOCK_BOTH;   // Clear both locks.
  else return -1;                                  // Unrecognized command -> error.

  bool ok = picLink.unlockValve(flags);           // Ask the PIC to clear the chosen lock(s).
  if (ok) readValveStatus();                      // On success, re-read the valve to confirm.
  triggerPublish = true;                          // Request a publish.
  Log.info("VALVE unlock 0x%02X -> %s (nak=0x%02X)",   // Log the outcome.
           flags, ok ? "ACK" : "fail", picLink.lastNak());
  return ok ? 1 : -1;                             // 1 on success, -1 on failure.
}

// ---- Timebase changes from the cloud (addendum A.2 / A.3) ------------------
//
// WHY A RESTART, RATHER THAN JUST WRITING THE NEW VALUE
// -----------------------------------------------------
// The PIC judges its own oscillator by comparing "the time I expected to wake"
// against "the time the cloud says it is". Normally those differ by the report
// interval and the difference is genuine oscillator error. But moving the local
// offset (or the anchor, or the interval) MOVES THE TIMEBASE ITSELF: the next
// time we hand the PIC a local epoch, it can be hours away from what it expected
// through no fault of its oscillator. The PIC would read that jump as "I am 12
// hours fast" and poison its calibration factor for many reports afterwards.
//
// Rather than teach the PIC to recognise and special-case such jumps, a timebase
// change restarts the whole system. Cold boot is a path that already exists and
// is already correct: HFINTOSC pre-calibration, then a fresh cloud time, then a
// first grid alignment from scratch. The jump is absorbed by initialisation
// instead of being handled as an exception.
//
// bucketSec is NOT in this list: it is computed entirely on the Photon and moves
// no clock, so it can change in place.
static void requestTimebaseRestart(const char *why) {
#if ALLOW_PARAM_CHANGE
  g_pendingRestart = true;
  g_gridDirty      = true;
  picParamsDirty   = true;
  syncBackupRam();                                // the new values must survive the reboot
  persistAll();
  Log.info("[EVT] TIMEBASE_CHANGE (%s) -> SYS_RESET to the PIC; it will power-cycle us "
           "for %lu ms and we both come back cold", why, 3000UL);
  // The PIC resets, and its boot code holds our supply off for about 3 s before
  // restoring it (App_Config.h PHOTON_COLDBOOT_OFF_MS). That full discharge is
  // what guarantees WE cold-boot too, so the two sides restart in step.
  picLink.sysReset();
#else
  Log.warn("[EVT] TIMEBASE_CHANGE (%s) stored but not applied: ALLOW_PARAM_CHANGE is 0", why);
#endif
}

// setTzOffset: local offset in SECONDS (for example -28800 for UTC-8), or in
// hours when the value is small enough to be unambiguous. Triggers a restart.
int setTzOffset(String cmd) {
  restartSleepTimer("setTzOffset");
  long v = atol(cmd.c_str());
  // A bare "-8" is far more likely to mean hours than 8 seconds, so accept both
  // spellings rather than silently applying an 8-second offset.
  int32_t off = (v > -24 && v < 24) ? (int32_t)(v * 3600L) : (int32_t)v;
  if (off < TZ_OFFSET_SEC_MIN || off > TZ_OFFSET_SEC_MAX) {
    Log.warn("[EVT] setTzOffset rejected: %ld s is outside %ld..%ld",
             (long)off, (long)TZ_OFFSET_SEC_MIN, (long)TZ_OFFSET_SEC_MAX);
    return -1;
  }
  if (off == g_tzOffsetSec) { Log.info("[EVT] setTzOffset: unchanged (%ld s)", (long)off); return 0; }

  Log.info("[EVT] setTzOffset: %ld -> %ld s", (long)g_tzOffsetSec, (long)off);
  g_tzOffsetSec = off;
  requestTimebaseRestart("local offset changed");
  return 1;
}

// setGrid: "anchorSec,intervalSec" - the PIC's report grid, both in seconds.
// anchor is seconds-of-day (local midnight = 0); interval is the gap between
// report times. The two are independent of each other and of bucketSec.
int setGrid(String cmd) {
  restartSleepTimer("setGrid");
  const char *p = cmd.c_str();
  const char *comma = strchr(p, ',');
  if (comma == nullptr) return -1;

  long anchor   = atol(p);
  long interval = atol(comma + 1);
  if (anchor < 0 || anchor >= 86400L) {
    Log.warn("[EVT] setGrid rejected: anchor %ld is not a second-of-day", anchor);
    return -1;
  }
  if (interval < (long)GRID_INTERVAL_SEC_MIN || interval > (long)GRID_INTERVAL_SEC_MAX) {
    Log.warn("[EVT] setGrid rejected: interval %ld is outside %lu..%lu s",
             interval, (unsigned long)GRID_INTERVAL_SEC_MIN,
             (unsigned long)GRID_INTERVAL_SEC_MAX);
    return -1;
  }
  if ((uint32_t)anchor == g_grid.anchorSec && (uint32_t)interval == g_grid.intervalSec) {
    Log.info("[EVT] setGrid: unchanged");
    return 0;
  }

  Log.info("[EVT] setGrid: anchor %lu -> %ld, interval %lu -> %ld",
           (unsigned long)g_grid.anchorSec, anchor,
           (unsigned long)g_grid.intervalSec, interval);
  g_grid.anchorSec   = (uint32_t)anchor;
  g_grid.intervalSec = (uint32_t)interval;
  requestTimebaseRestart("report grid changed");
  return 1;
}

// getGrid: log the stored grid, the PIC's view of it, and the offset, in raw
// seconds AND in a readable form - the two together are what makes a bench
// capture interpretable.
int getGrid(String cmd) {
  (void)cmd;
  restartSleepTimer("getGrid");
  Log.info("[DAT] grid (stored): anchor=%lu (%02lu:%02lu:%02lu) interval=%lu s (%.2f h) "
           "tzOffset=%ld s (%+.1f h) bucketSec=%lu",
           (unsigned long)g_grid.anchorSec,
           (unsigned long)(g_grid.anchorSec / 3600u),
           (unsigned long)((g_grid.anchorSec % 3600u) / 60u),
           (unsigned long)(g_grid.anchorSec % 60u),
           (unsigned long)g_grid.intervalSec, (double)g_grid.intervalSec / 3600.0,
           (long)g_tzOffsetSec, (double)g_tzOffsetSec / 3600.0,
           (unsigned long)g_bucketSec);
  if (g_gridFromPicValid) {
    Log.info("[DAT] grid (in the PIC): anchor=%lu interval=%lu s%s",
             (unsigned long)g_gridFromPic.anchorSec,
             (unsigned long)g_gridFromPic.intervalSec,
             (g_gridFromPic.anchorSec == g_grid.anchorSec &&
              g_gridFromPic.intervalSec == g_grid.intervalSec)
               ? "" : "   <-- DIFFERS from the stored values");
  }
  triggerPublish = true;
  return 1;
}

// picReset: PKT_SYS_RESET. No reply; also clears the PIC's permanent valve lock.
int picReset(String cmd) {
  (void)cmd;                                      // Argument unused.
  restartSleepTimer("picReset");                  // Reset the awake window.
  picLink.sysReset();                             // Send the reset command to the PIC.
  Log.info("PIC: SYS_RESET sent");                // Log that we sent it.
  return 1;                                        // Success (we don't wait for a reply).
}

// Force-push the cached PIC leak parameters now (REQ_SET_LEAK).
int syncPic(String cmd) {
  restartSleepTimer("syncPic");                   // Reset the awake window.
  (void)cmd;                                      // Argument unused.
  return pushPicParams() ? 1 : -1;                // Try to deliver the cached params; 1 if ACKed, else -1.
}

#if USE_WIFI                                       // Wi-Fi-only cloud functions below.
// Cloud function: set Wi-Fi credentials. cmd = "ssid,password".
int setWiFi(String cmd) {
  restartSleepTimer("setWiFi");                   // Reset the awake window.
  cmd.trim(); int comma = cmd.indexOf(',');       // Trim and find the comma separating ssid/password.
  if (comma <= 0) return -1;                      // Must have a comma with text before it.
  String ssid = cmd.substring(0, comma); String pass = cmd.substring(comma + 1);   // Split into ssid and password.
  ssid.trim(); pass.trim();                       // Trim each part.
  if (ssid.length() == 0 || !WiFi.setCredentials(ssid, pass)) return -1;   // Reject empty ssid or a failed save.
  WiFi.connect(); return 1;                       // Connect with the new credentials; success.
}
// Cloud function: forget all Wi-Fi credentials and disconnect.
int clearWiFi(String cmd) { restartSleepTimer("clearWiFi"); (void)cmd; WiFi.clearCredentials(); WiFi.disconnect(); return 1; }
#endif                                            // End Wi-Fi-only functions.

// === PIC session drain ===
// Under power-gating there is no WAKE line to poll; a session pulls REQ_DATA until
// the PIC reports an empty batch (COUNT=0), which sets wakeSessionDrained. The old
// D10/rate-limit polling state is gone (there is no "poll forever" risk anymore).
static bool     wakeSessionDrained = false;       // true once the PIC says "no more data" this session.

// Pull and process any data the PIC has. 'picInitiated' is now advisory only:
// under power-gating we are powered *because* the PIC wants a session, so there
// is no D10 gate to check -- we always proceed to REQ_DATA. (Removing that gate
// is what fixes the "meter count 0" bug: REQ_DATA now actually gets sent.)
void serviceMeterFromPic(bool picInitiated) {
  (void)picInitiated;                             // Kept for call-site compatibility; no longer gates anything.

  PicReportInfo info;
  memset(&info, 0, sizeof(info));
  int n = picLink.requestData(picBuf, PIC_MAX_SAMPLES, &info);   // REQ_DATA -> RSP_DATA.

  if (n < 0) {
    // ==== V064 P-1: A FAILED TRANSACTION IS NOT A BROKEN SERIES =============
    //
    // V063 treated these as the same thing and paid for it. It set
    // g_picSeriesOk=false, called flashBufferClear() and wiped the carry right
    // here - and the bench log shows what that cost: three accumulated blocks
    // destroyed, then a perfectly good RSP_DATA arriving 121 ms later. It
    // happened four times in one capture, and what it destroyed each time was
    // precisely the CLOUD_FAIL backlog the CLOUD_FAST replay test needs. With
    // that code the 4/4 accumulation could not be reached even in principle.
    //
    // err -2 is PIC_ERR_TIMEOUT: transact() exhausted its retries. That is a
    // statement about ONE transaction, not about the session. The PIC may have
    // answered the very next request - and per the paired PIC-side log, it had
    // in fact received and answered all five.
    //
    // So record the failure and decide later, when the session has actually
    // finished and there is evidence to decide ON. A single valid RSP_DATA
    // anywhere in this session retracts it entirely.
    g_picAttemptFailed = true;
    g_picFailCount++;
    g_picLastErr = n;
    Log.warn("[DAT] REQ_DATA transaction failed (err %d = %s) - continuity decision "
             "DEFERRED to session end (attempt %u; flash %u block(s) and carry "
             "PRESERVED)", n, PicLink::picErrName(n), (unsigned)g_picFailCount,
             (unsigned)flashBufferCount());
    // V059, still true: prevReportEnd is deliberately NOT cleared. It is a claim
    // about how far we have already placed data, and that stays true no matter
    // what the PIC does next. Zeroing it used to disarm the overlap trim in
    // ingestReport() at exactly the moment the PIC is about to retransmit.
    return;
  }

  // ==== V064 P-2: ANY ANSWERED REQ_DATA IS A HEALTHY LINK ===================
  // Including a zero-sample one. requestData() returns (int)f_count, so n == 0
  // is a SUCCESS return, never a negative error - the branch above cannot see
  // it. That distinction becomes load-bearing under PIC V023, which answers
  // n=0 caps=0 imp=0 as its normal reply to an extra REQ_DATA after the batch
  // has been ACKed in the same session. n=0 means "nothing to report", not
  // "could not reach the PIC"; counting it as a failure would let routine V023
  // behaviour trigger the destruction P-1 just removed.
  g_picSeriesOk      = true;
  g_picAttemptFailed = false;   // a good reply retracts any earlier failure this session

  Log.info("[DAT] RSP_DATA hdr: start=%lu valid=%u n=%lu caps=%lu imp=%lu ovf=%u interval=%lums",
           (unsigned long)info.startTime, (unsigned)info.startTimeValid,
           (unsigned long)info.sampleCount, (unsigned long)info.totalCaptures,
           (unsigned long)info.totalImpulses, (unsigned)info.overflowFfff,
           (unsigned long)info.sampleIntervalMs);

  // Appendix E.4 (a): content summary, not just the header. sum must equal the
  // PIC's imp= when the ring did not overrun; the gap is the missed-fill amount.
  if (n > 0) logSampleSummary(picBuf, (uint32_t)n);

  if (info.totalCaptures > info.sampleCount) {
    // The ring overran: the series carries only part of the period, but the
    // hardware totals still describe all of it, so no volume is unaccounted for.
    Log.warn("[DAT] FIFO overrun: %lu captures in the period, %lu samples carried "
             "(%lu captures not represented; totals preserved)",
             (unsigned long)info.totalCaptures, (unsigned long)info.sampleCount,
             (unsigned long)(info.totalCaptures - info.sampleCount));
  }
  if (info.overflowFfff > 0u) {
    Log.warn("[DAT] %u sample(s) clamped at 0xFFFF - per-sample rate understated there, "
             "total corrected from totalImpulses", (unsigned)info.overflowFfff);
  }

  if (n == 0) {                                   // PIC has nothing new -> session drained.
    wakeSessionDrained = true;
    // V064 P-2: this is a SUCCESSFUL exchange, and under PIC V023 a routine one.
    // Say so explicitly, so a reader of the capture cannot mistake "no samples"
    // for "no contact" - that conflation is what P-1 and P-2 exist to prevent.
    Log.info("[DAT] n=0: PIC has nothing to report - link healthy, continuity intact, "
             "flash %u block(s) and carry untouched", (unsigned)flashBufferCount());
    if (initialHold) Log.info("[DAT] zero samples during the initial hold - expected "
                              "(the PIC discards cold-boot captures)");
    return;
  }

  // ==== V064 P-5: is this a batch we have already stored? ====================
  // Runs AFTER the n==0 early return (a duplicate always carries samples) and
  // BEFORE any ingest, because ingestReport() is not idempotent: hourlyProcess()
  // bins everything handed to it and the rolling accumulators use "+=".
  //
  // The re-ACK is not optional. The PIC keeps an un-ACKed batch forever and
  // prefers it on every subsequent REQ_DATA, so a silent skip would wedge the
  // link on this batch permanently. Skip the STORE, repeat the ACK.
  if (info.batchSeqValid && g_lastStoredBatchSeqValid &&
      info.batchSeq == g_lastStoredBatchSeq) {
    Log.warn("[DAT] duplicate batch_seq=%u - storage skipped, ACK resent "
             "(already durably stored; PIC never saw our 0x0B)",
             (unsigned)info.batchSeq);
    picLink.sendDataReceived(info.batchSeq, true);
    return;
  }

  // The span this batch covers ends at the PIC wake, not now.
  //
  // Appendix F.1-B: when there is NO cloud time we pass 0, never a fabricated
  // epoch. The old code used localNowTracked(), which without a sync returns the
  // Particle default (946684800 + tzOffset + uptime) - a value that LOOKS like a
  // plausible epoch and so slipped past every downstream check, seeding the
  // 2000-grid binning and the false overlap-skip this release fixes. A 0 is
  // unmistakably not-a-time: anything that tried to use it as an epoch fails
  // immediately instead of silently. ingestReport()'s no-time branch does not use
  // endLocal as a timestamp base (it derives a clearly-relative one), so 0 is safe.
  bool     haveAbsTime = g_spanEndValid;
  uint32_t endLocal    = haveAbsTime ? g_spanEndLocal : 0u;
  if (!haveAbsTime) {
    Log.warn("[DAT] no synced span end (no cloud time) - buckets will not be placed "
             "this session; batch stored raw (Appendix E.5)");
  }

  if (!ingestReport(picBuf, (uint32_t)n, info, endLocal, haveAbsTime, "RPT")) {
    Log.warn("[DAT] batch could not be placed - not ACKing, the PIC will resend");
    return;                                       // no 0x0B: consume-on-ACK keeps it safe
  }

  // Keep the raw block so a cloud failure later in this session is recoverable.
  // Stored BEFORE the ACK, because after the ACK the PIC no longer holds it.
  {
    FlashBlockHeader bh;
    memset(&bh, 0, sizeof(bh));
    bh.startLocal       = info.startTimeValid ? info.startTime : 0u;
    bh.startValid       = info.startTimeValid;
    bh.endLocal         = haveAbsTime ? endLocal : 0u;   // Appendix F.3: no fake end when no cloud time
    bh.endValid         = haveAbsTime ? 1u : 0u;         // symmetric with startValid; drives sec 6.7 replay
    bh.sampleCount      = (uint32_t)n;
    bh.totalCaptures    = info.totalCaptures;
    bh.totalImpulses    = info.totalImpulses;
    bh.overflowFfff     = info.overflowFfff;
    bh.sampleIntervalMs = info.sampleIntervalMs;
    flashBufferStore(bh, picBuf, (uint32_t)n);    // no-op when FLASH_BUFFER_BLOCKS == 0
  }

  // V064 P-5: record WHICH batch is now durably ours, before the ACK and inside
  // the same persistAll() that makes the block durable. If power is cut between
  // the store and the ACK, the next session sees the retransmission, matches this
  // seq, and re-ACKs instead of storing it a second time.
  if (info.batchSeqValid) {
    g_lastStoredBatchSeq      = info.batchSeq;
    g_lastStoredBatchSeqValid = true;
  }

  persistAll();                                   // durable before we tell the PIC it may forget
  // V065 (req 4.병): this is the moment the batch is ours - stored durably and
  // about to be ACKed. Counted HERE and not at the hourly stage, because that is
  // the claim the summary needs to make: "this session received N samples",
  // which stays true whether or not a time axis existed to place them on. The
  // duplicate-batch path above deliberately does NOT count: it re-ACKs a batch
  // that was already counted in the session that first stored it.
  g_sessionSamplesRx += (uint32_t)n;
  g_sessionBatchesRx++;
  picLink.sendDataReceived(info.batchSeq, info.batchSeqValid != 0u);   // 0x0B: consume-on-ACK
  if (info.batchSeqValid) {
    Log.info("[EVT] BATCH_STORED_ACKED: n=%d batch_seq=%u (sent DATA_RECEIVED 0x0B)",
             n, (unsigned)info.batchSeq);
  } else {
    Log.info("[EVT] BATCH_STORED_ACKED: n=%d (sent DATA_RECEIVED 0x0B)", n);
  }

  if (picParamsDirty) pushPicParams();            // If we owe the PIC a params write, try it while connected.
}

// ---- V064 P-3/P-4: the session-end continuity verdict ----------------------
//
// The ONLY place a discontinuity may be declared. Called once, after the drain
// loop, when the whole session's evidence is in.
//
// PRESERVATION IS THE DEFAULT, and it is the default in the strong sense: doing
// nothing at all preserves. That property is what makes this safe, because most
// sessions do NOT end here. The PIC owns our power - main.c's photon_power_off()
// drives RC4 HIGH and the supply vanishes - so a session can be cut at any
// instant, including before this function is reached. Two tempting designs both
// fail on that fact:
//
//   - running the verdict periodically from loop() instead: that is exactly the
//     premature judgement P-1 removed, reintroduced under a different name;
//   - hanging it on a shutdown hook: the hook does not run when the power is
//     simply removed, so the rule quietly becomes a dead letter.
//
// Hence: no path anywhere may delete buffered data without a positive verdict
// reached here. A missing verdict must never mean deletion.
//
// And when the verdict IS "broken", the answer is a marker, not a delete (P-4).
// What we actually need to prevent is joining two unrelated runs into one
// plausible-looking series. A marker prevents that join exactly as well as
// deletion does, and keeps the data. The costs are not symmetric: a run kept in
// error can be discarded later, a run deleted in error is gone.
static void finishSessionContinuityVerdict() {
  if (!g_picAttemptFailed) {
    // Either every transaction succeeded, or a later one retracted an earlier
    // failure. Nothing to decide.
    return;
  }

  if (g_picSeriesOk) {
    // Belt and braces: a good reply already cleared g_picAttemptFailed, so this
    // should be unreachable. If it ever fires, the recovery reading is the safe
    // one and the state is inconsistent - say so rather than acting on it.
    Log.warn("[DAT] continuity verdict: %u failed attempt(s) but the series WAS "
             "received - recovered, nothing invalidated (state was inconsistent)",
             (unsigned)g_picFailCount);
    return;
  }

  // A real verdict: this session asked and never once got an answer.
  Log.warn("[EVT] CONTINUITY_BROKEN: %u REQ_DATA attempt(s) failed (last err %d = %s), "
           "no RSP_DATA at all this session -> recording a GAP MARKER",
           (unsigned)g_picFailCount, g_picLastErr, PicLink::picErrName(g_picLastErr));

  flashBufferMarkGap("no RSP_DATA received in the whole session");

  // The carry IS genuinely dead: it is a claim about one unfinished bucket, and
  // a hole in the axis means we can no longer say what completes it. Cutting it
  // here is what stops the bad join across the marker.
  hourlyCarryClear(hourlyCarry);

  // prevReportEnd survives, per V059. It records how far we have already placed
  // data, which stays true across a gap, and the overlap trim needs it most when
  // the PIC is about to retransmit. hourlyResolveSpanStart() only snaps a seam
  // within SPAN_CONTINUITY_TOL_SEC, so a real gap will not be bridged by it.
  syncBackupRam();                                // make the cut durable now
}

// ---- Session time sync (addendum A.1 / doc 05 section 3) -------------------
// Hands the PIC a LOCAL epoch and, from the same moment, derives the instant the
// PIC powered us up. That instant - not the publish time - is where this
// report's span ends, because the PIC froze its batch when it turned us on.
//
//     spanEnd = (local epoch at sync) - (our uptime at sync)
//
// Our uptime is measured from power-on, and the PIC powered us on, so the two
// are the same event. The PIC independently computes the same instant from its
// own side; neither subtracts the other's delay, so nothing is double-counted.
static void runSessionTimeSync() {
  // Re-read the clock right here through the one checked gate (H.3): a value
  // cached at cloud-connect could be minutes old by now, and the PIC would
  // silently inherit that error. timeOk carries the validity; utcNow is 0 when
  // there is no clock, never a fabricated epoch.
  uint32_t utcNow    = 0u;
  bool     timeOk    = clockNowUtc(utcNow);
  logCloudTimeEdge();                              // Appendix H.15: log the isValid 0->1 transition
  uint32_t localNowE = timeOk ? (uint32_t)((int64_t)utcNow + (int64_t)g_tzOffsetSec) : 0u;
  uint32_t atMs      = millis();

  Log.info("[EVT] SESSION_START: timeValid=%d utc=%lu offset=%lds local=%lu uptime=%lus",
           (int)timeOk, (unsigned long)utcNow, (long)g_tzOffsetSec,
           (unsigned long)localNowE, (unsigned long)(atMs / 1000u));

  bool tsOk = picLink.sendTimeSync(timeOk, localNowE);
  Log.info("[EVT] TIME_SYNC %s (PIC %s)", tsOk ? "OK" : "NO-ACK",
           tsOk ? "acked 0x0D" : "did not ack - it will treat this session as time-less");

  if (timeOk) {
    g_timeSyncLocal = localNowE;
    g_timeSyncMs    = atMs;
    uint32_t upSec  = atMs / 1000u;
    g_spanEndLocal  = (localNowE > upSec) ? (localNowE - upSec) : localNowE;
    g_spanEndValid  = true;
    Log.info("[DAT] span end (PIC wake) = %lu local  [= %lu - %lus uptime]; "
             "cloud clock became valid %lums after boot",
             (unsigned long)g_spanEndLocal, (unsigned long)localNowE, (unsigned long)upSec,
             (unsigned long)g_cloudReadyMs);
  } else {
    g_spanEndValid = false;
    Log.warn("[DAT] no cloud time this session -> the PIC is told time_valid=0 and "
             "sample placement falls back to its nominal interval");
  }
}

// ---- LOCO observation (doc 05 section 8) -----------------------------------
// REQ_GET_LOCO -> RSP_LOCO. Every factor is Q7, so both the raw value and the
// decoded multiplier are printed: 126 reads as 0.984.
static void logLocoStatus(const char *when) {
  PicLocoStatus st;
  if (!picLink.getLocoStatus(st)) {
    Log.warn("[DAT] LOCO (%s): no RSP_LOCO from the PIC", when);
    return;
  }
  Log.info("[DAT] LOCO (%s) applied_k=%u (%.4f) hf_precision=%u (%.4f) hf_cal=%u "
           "hf_in_range=%u cloud_cal=%u cloud_precision=%u (%.4f) cloud_in_range=%u",
           when,
           st.appliedK,       (double)st.appliedK       / PIC_LOCO_Q7_ONE,
           st.hfPrecision,    (double)st.hfPrecision    / PIC_LOCO_Q7_ONE,
           st.hfCalibrated, st.hfInRange, st.cloudCalibrated,
           st.cloudPrecision, (double)st.cloudPrecision / PIC_LOCO_Q7_ONE,
           st.cloudInRange);
}

// ---- Report grid: read it back, and write it after a PIC cold boot ---------
// The PIC keeps anchor/interval in RAM only, so a cold boot resets them to its
// compiled defaults. We hold the authoritative copy and push it back during the
// initial hold, which is the one moment the PIC is guaranteed to be listening
// and has not yet aligned its first grid point.
static void syncGridWithPic(bool picColdBoot) {
  PicParams     leak;
  PicGridParams grid;
  bool          gridValid = false;

  if (picLink.getParams(leak, &grid, &gridValid)) {
    picParams = leak;                             // cache what the PIC actually holds
    if (gridValid) {
      g_gridFromPic      = grid;
      g_gridFromPicValid = true;
      Log.info("[DAT] PIC grid: anchor=%lu (%02lu:%02lu:%02lu) interval=%lu s (%.2f h)",
               (unsigned long)grid.anchorSec,
               (unsigned long)(grid.anchorSec / 3600u),
               (unsigned long)((grid.anchorSec % 3600u) / 60u),
               (unsigned long)(grid.anchorSec % 60u),
               (unsigned long)grid.intervalSec,
               (double)grid.intervalSec / 3600.0);
    } else {
      Log.info("[DAT] PIC answered the 8-byte RSP_PARAM: this build has no grid fields");
    }
  } else {
    Log.warn("[DAT] REQ_GET_PARAM failed - PIC grid unknown this session");
  }

#if ALLOW_PARAM_CHANGE
  // Only write on a cold boot or when a change is pending. Writing the grid
  // mid-run would move the report times under the PIC's feet, which is exactly
  // the timebase jump the reboot path exists to avoid (addendum A.3).
  bool needWrite = picColdBoot || g_gridDirty;
  if (gridValid && grid.anchorSec == g_grid.anchorSec &&
      grid.intervalSec == g_grid.intervalSec && !g_gridDirty) {
    needWrite = false;                            // already correct
  }
  if (needWrite) {
    // Appendix E: two independent fixed-length writes, each confirmed on its own.
    // A failure of one does not invalidate the other, so they are tracked apart.

    // (1) REQ_SET_LEAK (0x03, len 8). Land the leak parameters.
    bool leakAck = picLink.setLeak(picParams);
    Log.info("[PARM] SET_LEAK a1=%u/%us a2=%u/%us -> %s",
             picParams.leak1_counts, picParams.leak1_window_s,
             picParams.leak2_counts, picParams.leak2_window_s,
             leakAck ? "ACK" : "NAK/FAILED");
    if (leakAck) picParamsDirty = false;
    else Log.warn("[PARM] SET_LEAK not ACKed (nak=0x%02X) - leak params stay pending",
                  picLink.lastNak());

    // (2) REQ_SET_GRID (0x0F, len 8). Land the report grid. An ACK proves the
    // frame parsed, not that the grid was applied (a PIC_USE_OWN_TIMING build
    // ACKs but keeps its own grid), so confirm by reading it back.
    bool gridAck     = picLink.setGrid(g_grid);
    bool gridWritten = false;
    Log.info("[PARM] SET_GRID anchor=%lu interval=%lu -> %s",
             (unsigned long)g_grid.anchorSec, (unsigned long)g_grid.intervalSec,
             gridAck ? "ACK" : "NAK/FAILED");
    if (!gridAck) {
      Log.warn("[PARM] SET_GRID not ACKed (nak=0x%02X) - grid stays pending",
               picLink.lastNak());
    }
#if PIC_VERIFY_SET_GRID
    else {
      PicParams     rbLeak;                          // read-back scratch (leak part unused here)
      PicGridParams rbGrid   = {0, 0};
      bool          rbValid  = false;
      if (!picLink.getParams(rbLeak, &rbGrid, &rbValid)) {
        Log.warn("[PARM] SET_GRID ACKed but read-back (REQ_GET_PARAM) failed "
                 "-> grid UNCONFIRMED, treating it as not written");
      } else if (!rbValid) {
        Log.warn("[PARM] SET_GRID ACKed but the PIC answers the 8-byte RSP_PARAM "
                 "-> no grid to confirm against, treating it as not written");
      } else if (rbGrid.anchorSec != g_grid.anchorSec ||
                 rbGrid.intervalSec != g_grid.intervalSec) {
        Log.warn("[PARM] SET_GRID ACKed but NOT applied: PIC holds anchor=%lu "
                 "interval=%lu, we asked anchor=%lu interval=%lu "
                 "(PIC_USE_OWN_TIMING on the PIC keeps its own grid)",
                 (unsigned long)rbGrid.anchorSec,  (unsigned long)rbGrid.intervalSec,
                 (unsigned long)g_grid.anchorSec,  (unsigned long)g_grid.intervalSec);
      } else {
        gridWritten = true;
        Log.info("[PARM] SET_GRID confirmed by read-back: anchor=%lu interval=%lu APPLIED",
                 (unsigned long)rbGrid.anchorSec, (unsigned long)rbGrid.intervalSec);
      }
    }
#else
    else gridWritten = gridAck;                      // trust the ACK when read-back is disabled
#endif
    if (gridWritten) g_gridDirty = false;            // clear ONLY on a confirmed apply

    Log.info("[EVT] GRID_WRITE leak=%s grid=%s (grid %s)",
             leakAck ? "ACK" : "FAILED", gridAck ? "ACK" : "FAILED",
             gridWritten ? "written" : "NOT written");
  }
#else
  (void)picColdBoot;
  Log.info("[EVT] GRID_WRITE skipped (ALLOW_PARAM_CHANGE is 0)");
#endif
}

// Replay everything the flash ring is holding, oldest first, into the same
// bucket series as the live report. Because each block carries its own span and
// they share one carry, the result is a single continuous timeline across the
// whole outage - which is the entire point of buffering the raw data rather than
// pre-computed buckets.
static void replayBufferedReports() {
#if FLASH_BUFFER_BLOCKS > 0
  uint16_t held = flashBufferCount();
  if (held == 0u) return;

  // Appendix H.2-A: the sec 6.7 back-computation anchors the reconstructed chain
  // at "now", so it is only valid when THIS session actually has a real cloud
  // time. localNowTracked() carries that validity (H.3): a session that never
  // synced returns false. Without a real 'now' we must NOT run the back-
  // computation - doing so is exactly what published a 2000-01-01 grid and
  // poisoned prevReportEnd (H.2). Hold every block, untouched, and place them in
  // a later session that has a real clock. This defers endValid=1 blocks too:
  // with no time axis the safe, simple choice is to postpone the whole replay
  // (H.2.5). The flash ring is cleared only on a SUCCESSFUL delivery, so nothing
  // is lost by waiting.
  uint32_t nowAbs = 0u;
  if (!localNowTracked(nowAbs)) {
    Log.info("[DAT] RECOVERY: %u block(s) held, still no cloud time -> not replayed "
             "(sec 6.7 needs a real 'now'; Appendix H.2-A)", (unsigned)held);
    return;   // blocks preserved; batched in a session that has a real clock
  }

  Log.info("[DAT] RECOVERY: replaying %u buffered report(s) before the current one", (unsigned)held);

  // Appendix H.17: summarise the replay set - how many carry their own absolute
  // end (endValid=1) vs must be reconstructed from the interval (endValid=0).
  {
    uint16_t withEnd = 0u;
    for (uint16_t i = 0; i < held; i++) {
      FlashBlockHeader bh;
      if (flashBufferLoad(i, &bh, picBuf, PIC_MAX_SAMPLES) <= 0) continue;
      if (bh.endValid) withEnd++;
    }
    Log.info("[FLASH] replay: %u block(s) eligible, %u with endValid=1, %u reconstructed",
             (unsigned)held, (unsigned)withEnd, (unsigned)(held - withEnd));
  }

  // Appendix F.3 (sec 6.7): blocks stored with no absolute end (endValid=0) were
  // captured while the device had no cloud time. Their endLocal is not an epoch
  // and MUST NOT be binned onto a 2000/1970 grid. Recovery runs only after cloud
  // time is back (gated above), so we reconstruct their placement from NOW: lay
  // the no-time blocks end-to-end, oldest first, so the newest ends at the
  // current instant, each block spanning sampleCount * sampleInterval. A first
  // pass measures the trailing no-time run so the chain can be anchored at "now".
  //
  // Appendix H.4: the span is accumulated in MILLISECONDS from the block header's
  // sampleIntervalMs, not truncated to integer seconds per block. The old
  // intervalSec = sampleIntervalMs / 1000u dropped 5207 ms to 5 s, and the error
  // compounded across every block of a long outage. Only the final anchor is
  // rounded to a whole-second epoch (epochs are second-resolution).
  uint64_t noTimeSpanMs = 0u;
  for (uint16_t i = 0; i < held; i++) {
    FlashBlockHeader bh;
    if (flashBufferLoad(i, &bh, picBuf, PIC_MAX_SAMPLES) <= 0) continue;
    if (!bh.endValid) {
      uint32_t stepMs = bh.sampleIntervalMs ? bh.sampleIntervalMs : 1000u;
      noTimeSpanMs += (uint64_t)bh.sampleCount * stepMs;
    }
  }
  uint32_t noTimeSpan = (uint32_t)((noTimeSpanMs + 500u) / 1000u);   // ms -> whole seconds, rounded
  // The oldest no-time block starts this far back from now; the cursor advances
  // as each no-time block is consumed so consecutive blocks abut without overlap.
  // Kept in milliseconds so per-block rounding does not drift the chain.
  uint64_t noTimeCursorMs = (uint64_t)((nowAbs > noTimeSpan) ? (nowAbs - noTimeSpan) : 0u) * 1000u;

  for (uint16_t i = 0; i < held; i++) {
    FlashBlockHeader bh;
    int n = flashBufferLoad(i, &bh, picBuf, PIC_MAX_SAMPLES);
    if (n <= 0) { Log.warn("[DAT] RECOVERY: block %u unreadable (%d) - skipped", (unsigned)i, n); continue; }

    PicReportInfo info;
    memset(&info, 0, sizeof(info));
    info.sampleCount      = bh.sampleCount;
    info.totalCaptures    = bh.totalCaptures;
    info.totalImpulses    = bh.totalImpulses;
    info.overflowFfff     = bh.overflowFfff;
    info.sampleIntervalMs = bh.sampleIntervalMs;

    // V064 P-4: this block does not continue the one before it. Cut the carry
    // here so no unfinished bucket is carried across the hole - that bad join,
    // not the mere existence of older data, is the thing the old
    // flashBufferClear() was really trying to prevent. Both sides of the marker
    // stay valid in their own right and are replayed normally.
    if (bh.gapBefore) {
      Log.warn("[DAT] RECOVERY: GAP MARKER before block %u (seq=%lu) - carry cut, "
               "the preceding run is NOT joined to this one",
               (unsigned)i, (unsigned long)bh.seq);
      hourlyCarryClear(hourlyCarry);
    }

    uint32_t endLocal;
    if (bh.endValid) {
      // A block that already had a real absolute end: place it as before.
      info.startTime      = bh.startLocal;
      info.startTimeValid = bh.startValid;
      endLocal            = bh.endLocal;
    } else {
      // Appendix F.3 / sec 6.7: no trustworthy times. Back-compute the end from the
      // chain cursor and let hourlyResolveSpanStart() derive the start from the
      // interval. startTimeValid=0 forces the back-computation.
      //
      // Appendix H.4: advance the cursor in MILLISECONDS using the block header's
      // sampleIntervalMs (the most accurate source - it can differ per block),
      // then round to a whole-second epoch only for the axis endpoint. The
      // interval source is recorded so a capture shows which value drove the
      // reconstruction (header vs CFG vs constant).
      uint32_t stepMs      = bh.sampleIntervalMs ? bh.sampleIntervalMs : 1000u;
      const char *stepSrc  = bh.sampleIntervalMs ? "block header sampleIntervalMs"
                                                 : "fallback 1000ms constant";
      uint64_t blockSpanMs = (uint64_t)bh.sampleCount * stepMs;
      uint64_t startMs     = noTimeCursorMs;
      noTimeCursorMs      += blockSpanMs;           // this block ends here (ms)
      endLocal             = (uint32_t)((noTimeCursorMs + 500u) / 1000u);
      info.startTime       = 0u;
      info.startTimeValid  = 0u;
      Log.info("[DAT] RECOVERY: block %u had no absolute end (endValid=0) -> "
               "reconstructed span %lu..%lu (%llums, step=%lums source=%s) from interval (sec 6.7)",
               (unsigned)i, (unsigned long)((startMs + 500u) / 1000u),
               (unsigned long)endLocal, (unsigned long long)blockSpanMs,
               (unsigned long)stepMs, stepSrc);
    }

    // Recovery is gated on real cloud time now (H.2-A), so the block IS placed on
    // the axis. Blocks whose span was reconstructed from the interval (endValid=0)
    // pass reconstructed=true so equalize uses the exact ms step (H.4).
    ingestReport(picBuf, (uint32_t)n, info, endLocal, true, "RECOVER", !bh.endValid);
  }
#endif
}

// =============================================================== Sleep mgmt
// Reset the awake window so the device stays up a bit longer (called on any activity).
void restartSleepTimer(const char *reason) {
  sleepCycleCount = 0; lastWakeTime = millis(); sleepStart = millis();   // Restart the timers from "now".
  Log.info("SLEEP: timer restarted (%s)", reason);   // Log why the timer was restarted.
}

// runSleep() is intentionally a NO-OP under the power-gating model (PIC V048).
//
// The Photon no longer owns its sleep/wake: the PIC switches our SUPPLY through a
// P-MOS on RC4, so "the PIC turning the power off IS the sleep." There is also no
// GPIO wake source anymore (the old .gpio(PIC_WAKE_PIN, RISING) is gone -- D10
// carries no signal). Instead of sleeping, a session simply finishes its work and
// asks the PIC to cut power with PKT_PHOTON_OFF_REQ (func 0x07); see
// handleMonitoring() / endSession(). This stub is kept only so the name still
// exists for reference and any stray caller compiles harmlessly.
void runSleep() {
  return;   // Power-gating: never self-sleep. The PIC removes our power to end a session.
}

// =============================================================== State machine
// End the session: 0x07 has (just) been sent, so let the last bytes flush and go
// idle. The PIC will drive RC4 HIGH and remove our power shortly; we never sleep
// ourselves. Nothing else should run after this until power is cut.
void endSession() {
  Particle.process();                             // Let the outgoing 0x07 frame / final publish flush.
  changeState(STATE_SESSION_DONE);                // Enter the idle phase.
  Log.info("SESSION ended -> idle; awaiting power-off from PIC");   // Announce we're done.
}

// In the CONNECTING phase: once the network + cloud are ready, move to the report
// (MONITORING) phase. If the cloud is NOT reachable within the 80 s budget, tell
// the PIC why with OFF_REASON_CLOUD_FAIL *before* its 90 s cutoff, then go idle.
void handleConnecting() {
  // Keep the PIC from powering us off while we wait for the cloud. During this
  // phase the Photon sends no requests to the PIC, so its 20 s ACTIVE idle
  // backstop would otherwise cut power ~31 s after boot -- before we connect. A
  // zero-payload keepalive every KEEPALIVE_INTERVAL_MS (< the PIC's 20 s) resets
  // that timer, holding power for a live-but-connecting Photon. If we truly die,
  // the keepalives stop and the PIC's backstop fires normally (safety net kept).
  // Once connected we leave this function (below), so the keepalives auto-stop.
  static uint32_t lastKeepalive = 0;
  if (millis() - lastKeepalive >= KEEPALIVE_INTERVAL_MS) {
    lastKeepalive = millis();
    picLink.sendKeepalive();                      // AA 55 0A 00 00 <crc>; fire-and-forget, no reply.
  }

  bool netUp =
#if USE_WIFI
      WiFi.ready();                               // Wi-Fi interface up?
#elif USE_CELLULAR
      Cellular.ready();                           // Cellular interface up?
#else
      true;
#endif
  if (netUp && Particle.connected()) {            // Network + cloud both up...
    if (g_cloudReadyMs == 0u) g_cloudReadyMs = millis();   // how long the connect took
    Log.info("[CLD] UP   cloud connected (t=%lus) -> %s",
             (unsigned long)(millis() / 1000),
             initialHold ? "INITIAL_HOLD" : "MONITORING");
    if (initialHold) {                            // PIC put us in its cold-boot hold...
      changeState(STATE_INITIAL_HOLD);            // ...stay powered and stream the meter live (no 0x07).
    } else {                                      // Normal operation...
      changeState(STATE_MONITORING);              // ...pull data + publish, then ask the PIC to cut power.
      triggerPublish = true;                      // This session should publish once.
    }
    return;
  }

  // Diagnostic heartbeat (~every 5 s) so a capture shows WHERE bring-up stalls:
  //   netUp=0            -> still no IP (Wi-Fi link may be up but DHCP hasn't finished)
  //   netUp=1 ip=0.0.0.0 -> ready flag set but no address (shouldn't happen; DHCP issue)
  //   netUp=1 cloud=0    -> IP is fine; the CLOUD (DTLS to device server, UDP 5684) is
  //                         not reachable -> firewall/port, DNS, or unclaimed/bad keys.
  static uint32_t lastDiag = 0;
  if (millis() - lastDiag >= 5000) {
    lastDiag = millis();
#if USE_WIFI
    IPAddress ip = WiFi.localIP();                // 0.0.0.0 until DHCP hands us an address.
    Log.info("[CLD] WAIT t=%lus netUp=%d ip=%u.%u.%u.%u cloud=%d",
             (unsigned long)(millis() / 1000), (int)netUp,
             ip[0], ip[1], ip[2], ip[3], (int)Particle.connected());
#else
    Log.info("[CLD] WAIT t=%lus netUp=%d cloud=%d",
             (unsigned long)(millis() / 1000), (int)netUp, (int)Particle.connected());
#endif
  }

  // Cloud-fail path applies ONLY to a NORMAL session. 80 s is the Photon's own
  // self-imposed budget (TIMEOUT_CANNOT_FIND_CLOUD_MS). The PIC's timing is:
  //   - PWR_WAIT_FIRST: 90 s from power-on WITHOUT any packet (hard cutoff).
  //   - After the first packet: 20 s since the LAST valid packet (KEEPALIVE counts).
  // Because we send KEEPALIVE every ~5 s during this phase, the PIC's 20 s backstop
  // is continuously reset and the PIC will NOT cut power before we send 0x07. We
  // therefore have enough time to collect PIC data before terminating. In the INITIAL
  // hold the PIC keeps us powered for the full ~10 min and we must NEVER send 0x07 --
  // so we simply keep trying to connect until the cloud comes up (-> INITIAL_HOLD) or
  // the PIC removes power. millis() is time since power-on, bounding NORMAL to 80 s.
  if (!initialHold && millis() >= TIMEOUT_CANNOT_FIND_CLOUD_MS) { // NORMAL ran out of time...
    Log.warn("CLOUD: unreachable within %lu ms -> collecting PIC data, then "
             "PHOTON_OFF(CLOUD_FAIL)",
             (unsigned long)TIMEOUT_CANNOT_FIND_CLOUD_MS);
    // Mark offline so handleMonitoring() skips the cloud-time guard and goes straight
    // to the PIC exchange. Data will be stored raw in flash for replay once cloud time
    // is available (the existing no-time / endValid=0 path). [SUM] is then emitted
    // normally. handleMonitoring() sends PHOTON_OFF(CLOUD_FAIL) and ends the session.
    g_cloudOnlineThisSession = false;
    changeState(STATE_MONITORING);
  }
}

// The MONITORING phase (power-gating): a one-shot report. Pull the PIC's data,
// publish it, do any other pending business, then send PKT_PHOTON_OFF_REQ (0x07)
// so the PIC cuts our power. Runs once per session, not as a repeating poll.
// In FAST_BENCH_TEST the cloud/publish steps are compiled out: we only do the
// PIC UART exchange, log it over USB, and still end with func 0x07.
void handleMonitoring() {
  if (g_cloudOnlineThisSession) {
    // (cloud) We only reach here once the cloud is connected. Publishing and
    // interval scheduling need a valid clock; wait for the cloud time sync, but
    // still inside the 80 s budget (KEEPALIVE keeps the PIC's 20 s last-packet
    // backstop reset, so this is safe).
    if (!Time.isValid()) {                          // Clock not synced from the cloud yet...
      if (millis() >= TIMEOUT_CANNOT_FIND_CLOUD_MS) {   // ...and we are out of time...
        Log.warn("CLOUD: connected but no time sync in %lu ms -> PHOTON_OFF(CLOUD_FAIL)",
                 (unsigned long)TIMEOUT_CANNOT_FIND_CLOUD_MS);
        Log.info("[EVT] CLOUD_FAIL -> PHOTON_OFF(CLOUD_FAIL) (no cloud time)");
        picLink.sendPhotonOff(OFF_REASON_CLOUD_FAIL);   // Report the failure so the PIC shuts us down cleanly.
        endSession();
      } else {
        picKeepalivePump();                         // still waiting -> keep our power alive
      }
      return;                                       // Otherwise keep waiting for time.
    }

    // First-pass housekeeping: align the (optional) local sampler to the next boundary.
    uint32_t now = 0u; (void)clockNowUtc(now);                         // Current UTC (H.3 gate; valid here - guarded above).
    uint32_t stepSec = (uint32_t)(g_cfg.captureIntervalSecF + 0.5f);   // RUNTIME interval (from the PIC).
    if (stepSec == 0u) stepSec = METER_INTERVAL_SEC;                   // paranoia: never divide by zero
    if (nextSampleAtUtc == 0 || nextSampleAtUtc <= now)                // If the next-sample time is unset or past...
      nextSampleAtUtc = ((now / stepSec) + 1) * stepSec;               // ...set it to the next boundary.
  }

  // 1) Hand the PIC the time, then pull its batch.
  //
  //    TIME_SYNC carries a LOCAL epoch (addendum A.1, design A). The PIC does not
  //    know or care that it is local; it just compares epoch % 86400 against its
  //    anchor, so giving it local time is exactly what makes its report grid land
  //    on local midnight. The offset is applied HERE and nowhere else.
  //
  //    Delay handling: the epoch is re-read immediately before the frame is built,
  //    so it is current at the instant of transmission. The PIC then subtracts its
  //    OWN elapsed time since it powered us (main.c: wake_abs = t_now - delay) to
  //    recover the wake instant. We must not subtract that delay as well or it
  //    would be counted twice - see docs/CHANGES_V056_to_V057.md, finding P2.
  runSessionTimeSync();

  //    Read the grid the PIC is actually running on. A mismatch against our
  //    stored copy is worth seeing in the log: it means the PIC cold-booted back
  //    to its compiled defaults and the report times are not where we think.
  syncGridWithPic(false);

  //    Everything the PIC is about to send belongs to the span that ENDED when it
  //    powered us up; captures taken while this session runs go in the next report.
  wakeSessionDrained = false;                     // Start a fresh drain for this session.
  // V064 P-1: a new session starts with no accusation against the link. These are
  // per-session evidence, never carried across a power cut.
  g_picAttemptFailed = false; g_picFailCount = 0; g_picLastErr = 0;
  g_hourlyValid      = false;                     // Fresh bucket series for this session.
  g_hourlyMakeable   = 0;
  g_hourlySent       = 0;
  // Appendix H.20: reset the per-session publish/warning accounting (the network
  // connect time was already recorded in handleConnecting; keep it).
  g_pubAttempted = 0; g_pubOk = 0; g_pubFailed = 0; g_pubBytes = 0; g_sessionWarnings = 0;
  // V065 (req 4.병): per-session receive accounting, reset with the rest.
  g_sessionSamplesRx = 0; g_sessionBatchesRx = 0;

  //    Appendix G.3.1: log the calibration coefficients and the usable frequency
  //    range once per session, so the calculation-verification output can be read
  //    without recomputing the collapse point by hand.
  logFlowCalBanner();

  //    Anything the flash ring is still holding from a previous cloud outage is
  //    replayed FIRST, so the buffered reports and this one form one continuous
  //    timeline instead of two disjoint ones.
  replayBufferedReports();

  for (uint8_t i = 0; i < PIC_DATA_MAX_REQUESTS && !wakeSessionDrained; i++) {
    serviceMeterFromPic(true);                    // REQ_DATA -> ingest samples (COUNT=0 is normal, not an error).
  }

  // V064 P-1/P-3: the drain is over, so the evidence is complete. This is the one
  // and only point at which a discontinuity may be declared - and it declares one
  // only if EVERY attempt this session failed.
  finishSessionContinuityVerdict();

  //    Read the PIC's internal calibration state for the log. The PIC has no log
  //    path of its own, so this is the only way to watch applied_k converge.
  logLocoStatus("post-sync");

#if USE_LOCAL_METER
  serviceLocalMeter();                            // If the local sensor is enabled, service it too.
#endif

  // Shutoff auto-reset (if a valve command ran during this session).
  if (resetShutoff) {                             // If the 10 s valve timer asked us to reset...
    shutoffSwitch("off"); imu_data.leaking = false; imu_data.shutoff = false;   // ...power off the valve and clear flags.
    triggerPublish = true; resetShutoff = false;  // ...request a publish and clear the request flag.
  }

  // 2) Refresh PIC-side state over UART (no cloud needed for either of these).
  if (picParamsDirty) pushPicParams();            // Push queued leak params to the PIC.
  readValveStatus();                              // Read fresh valve status (logged over USB in bench mode).

  // 3) Emit the two data lines requested for the cloud test, then publish. The raw
  //    per-batch PIC pulse series was already logged during ingest (step 1, above);
  //    the hourly-flow summary is logged here. Both run AFTER cloud connect + ingest,
  //    so they reflect real, time-valid data instead of stale retained values. The
  //    publish block below is CLOUD-BUILD ONLY (compiled out in fast mode), but the
  //    same two USB lines print in either build.
  imuPrint();                                     // Log a one-line status summary (USB).
  printHourlyFlow();                              // The real bucket series + the carry (both builds, post-ingest).

  // Appendix H.13: delivery verdict is now RUNTIME, not compile-time, so the same
  // binary handles all four modes and injected faults. A session that had no cloud
  // time (CLOUD_FAIL, or an injected CONNECT failure) can never have delivered its
  // report, so its raw block MUST stay in the flash ring for the next session (the
  // 4-sector ring cycle, Appendix F.2). A session that HAD time starts optimistic;
  // cloudEmit() clears the flag if any real (or injected-PUBLISH) event fails.
  bool hadTimeThisSession = Time.isValid();
  g_cloudPublishOk = hadTimeThisSession;          // no time -> not delivered; time -> optimistic
  imuPublish();                                   // Cloud: publish. Bench/offline: BUILD+LOG payload over USB.

  // Failure kind 1 (doc 05 section 6.5): the PIC series arrived but the cloud
  // did not take it. The raw blocks stay in the ring and are replayed next
  // session, so the timeline is continued rather than restarted. Only a fully
  // delivered report lets us drop them.
  if (g_cloudPublishOk && g_picSeriesOk) {
    flashBufferClear("report delivered to the cloud");
  } else if (!g_cloudPublishOk) {
    Log.warn("[CLD] report NOT delivered -> %u raw block(s) retained for the next session",
             (unsigned)flashBufferCount());
  }

  persistAll();                                   // Flush RAM buffers to flash before power is cut.
  triggerPublish = false;                         // Report delivered.

  logSessionSummary(hadTimeThisSession);          // Appendix H.20: one-block session health line.

  // 4) Done with all business -> ask the PIC to cut our power. Sending 0x07 here
  //    (rather than waiting for the PIC's 20 s idle backstop) is the normal, clean
  //    end of a session: the PIC drives RC4 HIGH -> P-MOS off -> Photon unpowered.
  //
  // Appendix H.13: the off-reason follows whether this session had cloud time. A
  // no-time session (CLOUD_FAIL / injected connect-fail) reports CLOUD_FAIL - the
  // code the PIC would see from a real field outage; a session with time (cloud,
  // CLOUD_FAST, or the FAST_BENCH virtual clock) reports DONE.
  if (!hadTimeThisSession) {
    Log.info("[EVT] SESSION_DONE -> PHOTON_OFF(CLOUD_FAIL) (no cloud time this session)");
    picLink.sendPhotonOff(OFF_REASON_CLOUD_FAIL);   // AA 55 07 00 01 01 00 C1
  } else {
    Log.info("[EVT] SESSION_DONE -> PHOTON_OFF(DONE) (asking PIC to cut power)");
    if (!g_cloudOnlineThisSession)
      Log.info("BENCH: PIC exchange done -> PHOTON_OFF(DONE)");
    else
      Log.info("SESSION report published -> PHOTON_OFF(DONE)");
    picLink.sendPhotonOff(OFF_REASON_DONE);         // AA 55 07 00 01 00 C0 00
  }
  endSession();                                   // Idle until the PIC removes power.
}

// The INITIAL cold-boot hold (power-state = 0): the PIC keeps us FULLY POWERED for its
// ~10-min window and CUTS OUR POWER when it ends. Hard rules from the spec: never send
// func 0x07, never self-sleep, never run our own timer. We just poll the meter every
// ~INITIAL_HOLD_POLL_MS and log each batch over USB (raw per-capture pulse series +
// hourly/daily summary -- the same logging a NORMAL session does), so the flow can be
// watched live. The session ends only when the PIC removes power; there is nothing here
// to send and nothing to time.
void handleInitialHold() {
  if (!Time.isValid()) return;                    // Wait for cloud time: the PIC's whole first grid
                                                  //   alignment depends on the time we are about to give it.

  // The cold-boot handshake happens exactly once per hold. This is the moment
  // the PIC is guaranteed to be listening and has not yet chosen its first grid
  // point, so it is the only correct place to (re)write the grid it must use.
  static bool coldBootDone = false;
  if (!coldBootDone) {
    coldBootDone = true;

    if (g_pendingRestart) {
      // We are back up because WE asked for this restart (a timebase change).
      Log.info("[PWR] cold boot after a requested restart: applying the stored "
               "offset=%lds anchor=%lu interval=%lu",
               (long)g_tzOffsetSec, (unsigned long)g_grid.anchorSec,
               (unsigned long)g_grid.intervalSec);
      g_pendingRestart = false;
      // A timebase change makes every carried-over seam meaningless: the old
      // partial bucket belongs to a local axis that no longer exists.
      hourlyCarryClear(hourlyCarry);
      prevReportEnd        = 0;
      prevReportEndValid   = false;  // Appendix F.1-C: the seam belongs to the old axis
      prevCreditedImpulses = 0;      // the seam is gone, so nothing is "already credited"
      prevCreditedCaptures = 0;
      flashBufferClear("timebase changed - old blocks belong to a different axis");
      syncBackupRam();
    }

    syncGridWithPic(true);                        // write anchor/interval for this cold boot
    runSessionTimeSync();                         // then hand it the local time it aligns from
    logLocoStatus("initial-hold");                // HFINTOSC pass should read ~126 here
    g_hourlyValid    = false;
    g_hourlyMakeable = 0;
  }

  static uint32_t lastPoll = 0;                   // Rate-limit polling to the ~1-3 s window.
  if (millis() - lastPoll < INITIAL_HOLD_POLL_MS) return;
  lastPoll = millis();

  // The PIC answers every REQ_DATA during the hold with ZERO samples and wipes
  // whatever it captured, deliberately - cold-boot captures have no trustworthy
  // time behind them. Seeing 0 here, repeatedly and even on retransmits, is the
  // correct result, not a fault.
  wakeSessionDrained = false;
  for (uint8_t i = 0; i < PIC_DATA_MAX_REQUESTS && !wakeSessionDrained; i++) {
    serviceMeterFromPic(true);
  }
  // V064 P-3: NO continuity verdict here, deliberately. The hold is not a session
  // with an end we control - the PIC cuts power when the window closes - and the
  // PIC answers every REQ_DATA during it with zero samples by design. There is
  // nothing to judge, and under P-3 taking no action is precisely what preserves
  // the buffered data.
  imuPrint();                                     // One-line status summary (USB).
  printHourlyFlow();                              // Bucket view (USB).
  // Deliberately NO imuPublish(), NO sendPhotonOff(), NO sleep, NO timeout: the PIC owns
  // the window and ends the session by removing power.
}

// =============================================================== Arduino entry
// Request our timing + debug config from the PIC (RSP_PHOTON_CFG, 0x09). This is
// the FIRST PIC exchange each boot. transact() already retries on timeout/CRC;
// we add a few more attempts so a slow first byte after power-up cannot make us
// fall back to defaults unnecessarily. Rules:
//   - PIC answers provided=1  -> adopt its values.
//   - PIC answers provided=0  -> it deliberately provides none: keep our defaults
//                                (this is a SUCCESS, we do not retry).
//   - PIC never answers (old PIC / broken link) -> after the retries, keep our
//                                defaults so we still run.
static void requestPicConfig() {
  const int   MAX_TRIES   = 5;
  const uint32_t RETRY_MS = 200;
  PicPhotonCfg c;
  Log.info("CFG: requesting from PIC (0x09)...");
  for (int attempt = 1; attempt <= MAX_TRIES; attempt++) {
    int r = picLink.getPhotonConfig(&c);
    if (r == PIC_OK) {
      if (c.provided) {
        g_cfg.fromPic            = true;
        g_cfg.captureIntervalSecF = (float)c.captureIntervalMs / 1000.0f;
        g_cfg.samplesPerReport   = c.samplesPerReport;
        // Appendix H.13.1: the PIC's fastBench bit is a CADENCE bit only - it says
        // how fast the PIC reports, not whether WE use the cloud. Apply it to the
        // cadence axis. The cloud axis stays whatever the build fixed it to
        // (CLOUD_ENABLED); the PIC does NOT get to enable or disable the cloud, so
        // a bench PIC reporting fastBench=0 can never pull a no-cloud build online.
        g_cfg.cadenceFast        = c.fastBench;
        // g_cfg.cloudEnabled left as its compile-time (CLOUD_ENABLED) value.
        g_cfg.debugDataseries    = c.debugDataseries;
        g_cfg.missedFillMode     = c.missedFillMode;
        g_cfg.serialDelayMs      = c.serialDelayMs;
        Log.info("CFG: from PIC v%u (capture=%.3fs, samples=%u, cadenceFast=%d, cloudEnabled=%d, dataseries=%d, missedFill=%d, serialDelay=%ums) [try %d]",
                 c.version, g_cfg.captureIntervalSecF, g_cfg.samplesPerReport,
                 (int)g_cfg.cadenceFast, (int)g_cfg.cloudEnabled, (int)g_cfg.debugDataseries,
                 (int)g_cfg.missedFillMode, g_cfg.serialDelayMs, attempt);
      } else {
        Log.info("CFG: PIC provides none -> using Photon defaults (capture=%.3fs, samples=%u, cadenceFast=%d, cloudEnabled=%d) [try %d]",
                 g_cfg.captureIntervalSecF, g_cfg.samplesPerReport,
                 (int)g_cfg.cadenceFast, (int)g_cfg.cloudEnabled, attempt);
      }
      return;                                       // answered (provided or not) -> done
    }
    // Appendix H.7.3: decode the failure so it can be matched against the PIC's
    // txdrops= counter next round. rx 0 bytes on a timeout points at line/power/
    // timing; a CRC/wrong-func with rx N bytes points at signal quality.
    if (r == PIC_ERR_TIMEOUT) {
      Log.warn("CFG: no valid reply (err %d = %s, waited %lums, rx %u bytes), retry %d/%d",
               r, PicLink::picErrName(r), (unsigned long)PHOTON_TIMEOUT_READ_MS,
               (unsigned)picLink.lastRxBytes(), attempt, MAX_TRIES);
    } else if (r == PIC_ERR_BAD_CRC || r == PIC_ERR_WRONG_RSP || r == PIC_ERR_BAD_FRAME) {
      Log.warn("CFG: no valid reply (err %d = %s, rx %u bytes, func=0x%02X len=%u), retry %d/%d",
               r, PicLink::picErrName(r), (unsigned)picLink.lastRxBytes(),
               (unsigned)picLink.lastRxFunc(), (unsigned)picLink.lastRxLen(), attempt, MAX_TRIES);
    } else {
      Log.warn("CFG: no valid reply (err %d = %s), retry %d/%d",
               r, PicLink::picErrName(r), attempt, MAX_TRIES);
    }
    delay(RETRY_MS);                                // pumps the system; short gap before retry
  }
  Log.warn("CFG: PIC unreachable after %d tries -> Photon defaults (capture=%.3fs, samples=%u, cadenceFast=%d)",
           MAX_TRIES, g_cfg.captureIntervalSecF, g_cfg.samplesPerReport, (int)g_cfg.cadenceFast);
}


// setup() runs ONCE at power-on/reset. It prepares everything the program needs.
void setup() {
  Serial.begin(115200);                           // Start the USB serial monitor at 115200 baud.
  dbgUartBegin();                                 // ADD a second log sink on the debug UART (Serial3 D15).
                                                  //   USB handler stays registered -> logs go out BOTH.
  // NOTE: no USB-CDC monitor wait anymore. The debug UART mirror (dbgUartBegin
  // above, Serial3) already captures every boot line from power-up without a
  // host having to open the USB port, so blocking on Serial.isConnected() would
  // only waste time (and, in bench mode, delay the first PIC exchange). This is
  // exactly why the separate debug UART exists.
  // Display only. All arithmetic in this firmware uses explicit local epochs
  // (UTC + g_tzOffsetSec); nothing depends on Time.zone().
  Time.zone((int)(g_tzOffsetSec / 3600));
  Log.info("[SYS] BOOT LeakSense %s on %s", FW_VERSION_STR, PLATFORM_STR);   // Announce boot: firmware version + board.

  // Appendix H.2-C: clear a seam left poisoned by V062's sec 6.7 reconstruction,
  // exactly once, on the first boot after the update. Without this a stale
  // 946656003/946656017 in retained RAM would keep skipping every batch as
  // "precedes prevReportEnd". Guarded by a schema marker so it runs only once.
  if (seamSchemaMagic != SEAM_SCHEMA_V063) {
    Log.warn("[SYS] seam schema %08lx != %08lx -> clearing prevReportEnd=%lu "
             "(Appendix H.2-C one-time wipe)",
             (unsigned long)seamSchemaMagic, (unsigned long)SEAM_SCHEMA_V063,
             (unsigned long)prevReportEnd);
    prevReportEnd      = 0u;
    prevReportEndValid = false;
    seamSchemaMagic    = SEAM_SCHEMA_V063;
    syncBackupRam();                                // persist the cleared seam immediately
  }

  loadFlowCal();                                  // Load the saved flow calibration from EEPROM.
  loadConfig();                                   // Load the saved host config from EEPROM.
  loadPicParams();                                // Load the saved PIC params from EEPROM.

  pinMode(LED1_PIN, OUTPUT);                       // Configure the status LED pin as an output.
  pinMode(SHUTOFF_SWITCH_PIN, OUTPUT);             // Configure the valve direction pin as an output.
  pinMode(SHUTOFF_SSR_PIN, OUTPUT);                // Configure the valve power (SSR) pin as an output.
  pinMode(MODE_PIN, INPUT_PULLUP);                 // Configure the button pin as an input with a pull-up resistor.
#if !HAS_FUEL_GAUGE
  pinMode(BATTERY_PIN, INPUT);                     // On boards without a fuel gauge, set the battery ADC pin as input.
#endif
#if USE_LOCAL_METER
  pinMode(METER_PIN, INPUT_PULLUP);                // If using the local sensor, set its pin as a pulled-up input.
#endif
  shutoffSwitch("off");          // known-safe valve power state
                                 //   Start with the valve power OFF (a safe, defined state).

  picLink.begin(PIC_UART_BAUD); // Serial1 (+ D10 held pulled-up; unused under power-gating)
                                // V065: was a bare 9600 literal. The constants that depend on
                                // the baud (PHOTON_RX_QUIESCE_MAX_MS, PHOTON_TIMEOUT_DATA_MS)
                                // are static_asserted against PIC_UART_BAUD in app_config.h,
                                // so a future baud change fails to COMPILE rather than
                                // producing a build that only misbehaves on large frames.
                                 //   Initialize the UART used to talk to the PIC. D10 no longer carries a signal.

  // Register all the cloud functions so the dashboard/CLI can call them by name.
  Particle.function("shutoffSwitch", shutoffSwitch);   // Control the local valve.
  Particle.function("leakingSwitch", leakingSwitch);   // Set/clear the host leak flag.
  Particle.function("setFlowCal", setFlowCal);         // Set flow calibration.
  Particle.function("getFlowCal", getFlowCal);         // Get flow calibration.
  Particle.function("setConfig", setConfig);   // host analytics (4 vars)   // Set the 4 host settings.
  Particle.function("getConfig", getConfig);                                // Publish current config.
  Particle.function("setLeakParams", setLeakParams); // PIC 4 leak params   // Set the PIC's 4 leak params.
  Particle.function("getLeakParams", getLeakParams);                        // Refresh PIC params from the PIC.
  Particle.function("getValve", getValve);      // PIC valve status         // Read the PIC valve status.
  Particle.function("unlockValve", unlockValve);// clear PIC valve lock     // Clear a PIC valve lock.
  Particle.function("picReset", picReset);      // PKT_SYS_RESET            // Reset the PIC.
  Particle.function("syncPic", syncPic);                                    // Force-push cached PIC params.
  Particle.function("setTzOffset", setTzOffset); // local offset (restarts the system)
  Particle.function("setGrid", setGrid);         // anchor + interval (restarts the system)
  Particle.function("getGrid", getGrid);         // report the grid + offset currently in use
#if USE_WIFI
  Particle.function("setWiFi", setWiFi);          // (Wi-Fi only) set credentials.
  Particle.function("clearWiFi", clearWiFi);      // (Wi-Fi only) clear credentials.
#endif

#if USE_IMU
  if (!imuInit()) Log.error("IMU init failed");   // Initialize the real IMU; log if it fails.
  calibrateGyroscope();                           // Calibrate the gyro's resting bias.
#else
  imuInit();   // no-op stub: marks IMU absent, telemetry fields stay 0
                                                  //   With no IMU, just run the harmless stub.
#endif

  // Restore persisted buffers from local flash first -- this needs no cloud, so
  // do it before we start (and possibly fail) the network connection.
  restorePersisted();            // load gMeter + leak model from LittleFS
                                 //   Restore the logger buffer and leak model from flash.
  flashBufferBegin();            // recover the raw-report ring bookkeeping

  // Retained memory can come back scrambled after a brownout or a firmware
  // change, and a bad carry would silently corrupt the first bucket of the next
  // report. Validate it rather than trusting it.
  if (hourlyCarry.magic != HOURLY_CARRY_MAGIC) {
    hourlyCarryClear(hourlyCarry);
    Log.warn("[DAT] carry record invalid -> cleared (the first bucket after this "
             "boot starts fresh)");
  }
  if (g_tzOffsetSec < TZ_OFFSET_SEC_MIN || g_tzOffsetSec > TZ_OFFSET_SEC_MAX) {
    Log.warn("[DAT] retained tz offset %ld out of range -> default %ld",
             (long)g_tzOffsetSec, (long)TZ_OFFSET_SEC_DFLT);
    g_tzOffsetSec = TZ_OFFSET_SEC_DFLT;
  }
  if (g_grid.intervalSec < GRID_INTERVAL_SEC_MIN || g_grid.intervalSec > GRID_INTERVAL_SEC_MAX ||
      g_grid.anchorSec >= 86400u) {
    Log.warn("[DAT] retained grid invalid -> defaults");
    g_grid.anchorSec   = GRID_ANCHOR_SEC_DFLT;
    g_grid.intervalSec = GRID_INTERVAL_SEC_PROD;
    g_gridDirty        = true;
  }

#if USE_LOCAL_METER
  attachInterrupt(METER_PIN, countPulse, CHANGE); // If local metering, call countPulse() on every pin change.
#endif

// V066 을: 전원 인가 직후 과도현상이 가라앉을 때까지 PIC와 아무것도 주고받지 않는다.
  // 분압저항 추가(하드웨어 수정)와 함께 ESD 경로 안정화를 돕는다.
  // PIC는 첫 패킷을 90 s까지 기다리므로(TIMEOUT_NO_MSG_PHOTON2PIC_MS) 3 s 침묵은
  // 그 한계의 3.3%에 불과하다 — PIC 측에 부작용 없음.
  // flushRx()로 전원 천이가 RX FIFO에 밀어넣은 쓰레기를 제거한 뒤 첫 교환을 시작한다.
  // (기존 serial-settle 블록과 달리, 조용히 있는 것이 목적이므로 keepalive를 보내지 않는다.)
  Log.info("[SYS] PIC quiet window: %lu ms before the first exchange",
           (unsigned long)PIC_QUIET_MS);
  delay(PIC_QUIET_MS);    // Particle의 delay()는 내부에서 Particle.process()를 돌린다
  picLink.flushRx();      // 전원 천이가 밀어넣은 쓰레기 제거

// FIRST PIC exchange: pull our timing + debug config from the PIC (0x09), with
  // retries. Runs in BOTH bench and cloud modes so the settings (including
  // fastBench) are known BEFORE we branch on them.
  requestPicConfig();

  // The bench/production decision comes from the PIC (RSP_PHOTON_CFG.fastBench),
  // so the bucket width must be chosen HERE, not at compile time - otherwise a
  // TEST-compiled Photon told "fastBench=0" by the PIC would publish real cloud
  // sessions with 5-minute buckets under the key "hourlyGallons".
  g_bucketSec = g_cfg.cadenceFast ? BUCKET_SEC_TEST : BUCKET_SEC_PROD;

  // V064 P-9: source and effect on ONE line, adjacent and unseparable.
  //
  // These two facts were previously several lines apart with unrelated output
  // between them, and that gap hid a live inversion for a whole campaign:
  //
  //     CFG received OK  -> cadenceFast=0, bucket 3600 s (production)  <- wrong test condition
  //     CFG receive FAIL -> cadenceFast=1, bucket 60 s   (fast)        <- accidentally right
  //
  // PIC V023 corrects the values themselves. This line is the separate, durable
  // half of the fix: whatever produced the settings and whatever they turned into
  // are now printed together, so the NEXT inversion of this kind is visible the
  // moment it happens instead of surviving a full test run unnoticed.
  Log.info("CFG: source=%s cadenceFast=%d bucket=%lus cloudEnabled=%d",
           g_cfg.fromPic ? "PIC" : "DEFAULT",
           (int)g_cfg.cadenceFast, (unsigned long)g_bucketSec,
           (int)g_cfg.cloudEnabled);

  // A change of bucket width invalidates the carry: a partial 300 s bucket
  // cannot be completed as part of a 3600 s one.
  if (hourlyCarry.valid && hourlyCarry.bucketSec != 0u && hourlyCarry.bucketSec != g_bucketSec) {
    Log.warn("[DAT] bucket width changed %lu -> %lu s: carry dropped (%.4f gal)",
             (unsigned long)hourlyCarry.bucketSec, (unsigned long)g_bucketSec,
             hourlyCarry.gallons);
    hourlyCarryClear(hourlyCarry);
  }

  // The bench build reports every 60 s rather than every 48 hours. This is
  // the SAME algorithm with different numbers - the whole point of the fast test
  // is that nothing else changes.
  if (g_cfg.cadenceFast && g_grid.intervalSec == GRID_INTERVAL_SEC_PROD) {
    g_grid.intervalSec = GRID_INTERVAL_SEC_TEST;
    g_gridDirty        = true;
    Log.info("[SYS] bench build -> report interval set to %lu s",
             (unsigned long)g_grid.intervalSec);
  }

  // V066 갑: serialDelayMs 블록 삭제.
  // 이 블록의 목적은 검정자의 USB serial monitor가 COM 포트를 다시 잡을 때까지
  // 기다리는 것이었다. 그러나 Serial3 debug UART 미러가 도입된 이후
  // (leaksense.cpp:3475 NOTE 참조) USB를 기다릴 이유가 사라졌다.
  // Serial3는 부팅 첫 줄부터 잡아주므로 USB 대기 코드 자체가 이미 제거된 상태이고,
  // serialDelayMs 블록만 홀로 남아 있었다.
  //
  // serialDelayMs 프로토콜 필드는 하위호환을 위해 RSP_PHOTON_CFG에 그대로 존재하며,
  // Photon 측에서는 무시한다. PIC 쪽은 변경 불필요.
  //
  // (V064 P-8 이전의 원 코멘트와 블록 전체 제거. 2026-08 req 112 갑.)
  Log.info("[SYS] bucket width=%lu s (%s), align=%s, max buckets=%u, "
           "tzOffset=%ld s (%+.1f h), grid anchor=%lu interval=%lu s",
           (unsigned long)g_bucketSec,
           g_cfg.cadenceFast ? "fast" : "production",
           (BUCKET_ALIGN_MODE == BUCKET_ALIGN_CLOCK) ? "clock" : "from-now",
           (unsigned)HOURLY_MAX_BUCKETS,
           (long)g_tzOffsetSec, (double)g_tzOffsetSec / 3600.0,
           (unsigned long)g_grid.anchorSec, (unsigned long)g_grid.intervalSec);

  // Power-state handshake: ask whether the PIC is in its initial ~10-min cold-boot
  // hold (0=INITIAL / 1=NORMAL). Retried at ~1 Hz until it succeeds. Done in both
  // modes so the initial-hold info is always exchanged (per design).
  uint8_t ps    = POWER_STATE_NORMAL;
  bool    psOk  = false;
  for (uint8_t t = 1; t <= POWER_STATE_MAX_TRIES; t++) {
    if (picLink.getPowerState(&ps) == PIC_OK) { psOk = true; break; }
    Log.warn("[PWR] power-state handshake failed (try %u/%u); retry in 1 s",
             (unsigned)t, (unsigned)POWER_STATE_MAX_TRIES);
    picLink.sendKeepalive();   // keep the PIC's idle backstop quiet while we retry
    delay(1000);
  }
  if (!psOk) {                 // Bounded: V055 looped here forever with no keepalives.
    ps = POWER_STATE_NORMAL;
    Log.error("[PWR] no RSP_POWER_STATE after %u tries -> assuming NORMAL",
              (unsigned)POWER_STATE_MAX_TRIES);
  }
  initialHold = (ps == POWER_STATE_INITIAL);
  // V064 P-7: report the state, not a guessed duration. The old string said
  // "INITIAL (10-min cold-boot hold)"; the bench debug configuration actually
  // holds for ~60 s ([BOOT] initial hold done -> [PWR ] Photon OFF t=60000).
  // The hold length is a PIC-side setting the Photon is never told, so printing
  // a constant for it states as fact something we do not know. Harmless to
  // function, corrosive to verification: once the log stops describing the
  // device, a tester has no way to tell which readings to trust.
  Log.info("[PWR] PIC power-state = %s", initialHold ? "INITIAL" : "NORMAL");

  // Appendix H.13.3: decide this session's injected cloud failure (if any) BEFORE
  // choosing the connect path. Only meaningful in a cloud-enabled build. The
  // counter is retained so the N-session pattern holds across the power cuts.
  g_sessionCounter++;
  g_injectFail = CLOUD_FAIL_NONE;
#if CLOUD_FAIL_EVERY_N > 0
  if (g_cfg.cloudEnabled && (g_sessionCounter % (uint32_t)CLOUD_FAIL_EVERY_N) == 0u) {
    // Alternate CONNECT and PUBLISH failures so both code paths are exercised
    // (H.13.3: "가능하면 실패의 종류도 나누어 달라").
    g_injectFail = ((g_sessionCounter / (uint32_t)CLOUD_FAIL_EVERY_N) & 1u)
                 ? CLOUD_FAIL_PUBLISH : CLOUD_FAIL_CONNECT;
    Log.warn("[NET] CLOUD_FAIL_EVERY_N=%d: session #%lu injected failure = %s",
             CLOUD_FAIL_EVERY_N, (unsigned long)g_sessionCounter,
             g_injectFail == CLOUD_FAIL_CONNECT ? "CONNECT (wifi not attempted)"
                                                : "PUBLISH (connected, publish refused)");
  }
#endif
  // Online this session iff the build enables the cloud AND we are not injecting a
  // connect failure. A publish-fault session is still "online" - it connects and
  // gets real time, only the publish is refused.
  g_cloudOnlineThisSession = g_cfg.cloudEnabled && (g_injectFail != CLOUD_FAIL_CONNECT);

  if (!g_cloudOnlineThisSession) {
    // No cloud this session: either a no-cloud build, or an injected CONNECT
    // failure in a cloud build. We do NOT attempt Wi-Fi and go straight to the
    // PIC exchange. Whether a virtual clock is seeded is a separate axis
    // (BENCH_VIRTUAL_CLOCK): FAST_BENCH seeds one so Time.isValid() is true;
    // CLOUD_FAIL and an injected connect-fail run with NO clock, so the no-time
    // path (Appendix E.5 / H.2 / H.3) runs exactly as on a real outage.
#if BENCH_VIRTUAL_CLOCK
    // Seed a LOCAL evolving virtual clock (spreads bench data across hours and
    // drifts the per-session sample count a little). Virtual period uses PIC timing.
    uint32_t reportPeriodSec = (uint32_t)((float)g_cfg.samplesPerReport * g_cfg.captureIntervalSecF);
    uint32_t jitterSec       = (uint32_t)(2.0f * g_cfg.captureIntervalSecF);
    uint32_t prevVirt = 0;
    uint32_t virtNow;
    if (loadBlob(BENCHTIME_PATH, &prevVirt, sizeof(prevVirt)) && prevVirt >= FAST_BENCH_TEST_EPOCH) {
      randomSeed(prevVirt ^ millis());
      int32_t jitter = (int32_t)random(-(int32_t)jitterSec, (int32_t)jitterSec + 1);
      virtNow = prevVirt + reportPeriodSec + (uint32_t)jitter;
      Log.info("BENCH virtual clock: %lu = prev %lu + period %lu + jitter %ld",
               (unsigned long)virtNow, (unsigned long)prevVirt,
               (unsigned long)reportPeriodSec, (long)jitter);
    } else {
      virtNow = FAST_BENCH_TEST_EPOCH;
      Log.info("BENCH virtual clock: %lu (first boot, base epoch)", (unsigned long)virtNow);
    }
    Time.setTime(virtNow);
    saveBlob(BENCHTIME_PATH, &virtNow, sizeof(virtNow));
    Log.info("Setup complete -> FAST_BENCH (no cloud; evolving local clock; PIC UART + USB log)");
#else
    // No cloud AND no virtual clock: Time.isValid() stays false all run.
    if (g_injectFail == CLOUD_FAIL_CONNECT)
      Log.info("Setup complete -> injected CONNECT failure this session "
               "(no cloud, no time; PIC UART + USB log)");
    else
      Log.info("Setup complete -> CLOUD_FAIL (no cloud, no time, immediate; "
               "PIC UART + debug-UART/USB log)");
#endif
    sleepStart = millis(); lastWakeTime = millis();
    changeState(STATE_MONITORING);
  } else {
    // Power-gating (cloud): kick off the connection WITHOUT blocking here --
    // handleConnecting() in loop() finishes it within the budget or CLOUD_FAILs,
    // then handleMonitoring() does one report and ends with 0x07.
    g_netConnectMs = 0; g_netPublishMs = 0;
    Log.info("[NET] wifi connecting... (session #%lu)", (unsigned long)g_sessionCounter);
#if USE_WIFI
    WiFi.on();
    Log.info("[NET] wifi credentials stored=%d", (int)WiFi.hasCredentials());
    WiFi.connect(WIFI_CONNECT_SKIP_LISTEN);
#elif USE_CELLULAR
    Cellular.on(); Cellular.connect();
#endif
    Particle.connect();
    triggerPublish = true;
    sleepStart = millis(); lastWakeTime = millis();
    changeState(STATE_CONNECTING);
    Log.info("Setup complete -> connecting (%s session%s)",
             initialHold ? "INITIAL-hold" : "normal power-gating",
             g_injectFail == CLOUD_FAIL_PUBLISH ? ", injected PUBLISH failure pending" : "");
  }
}

// loop() runs over and over after setup(). Under power-gating it drives ONE
// session (connect -> report -> ask the PIC to cut power), then idles until the
// PIC removes power. The Photon never sleeps itself anymore.
void loop() {
  Particle.process();   // Service the cloud link every pass (required in SYSTEM_MODE(MANUAL)).
  dbgUartService();     // 1-minute cloud-time log; runs in EVERY state. Non-blocking.

  switch (currentState) {                         // Act based on the current session phase.
    case STATE_STARTUP:                           // (STARTUP falls through to CONNECTING.)
    case STATE_CONNECTING:  handleConnecting();  break;   // Bring up the cloud (bounded) or CLOUD_FAIL out.
    case STATE_INITIAL_HOLD: handleInitialHold(); break;  // Cold-boot hold: stream the meter; the PIC cuts power.
    case STATE_MONITORING:  handleMonitoring();  break;   // Connected: pull PIC data, publish, then 0x07.
    case STATE_SESSION_DONE:                              // 0x07 already sent...
      // Session finished. The PIC will drive RC4 HIGH and remove our power very
      // shortly (it acts on our 0x07, or its 20 s idle / 90 s no-packet backstop).
      // There is nothing to do but idle -- the PIC owns power now.
      break;
  }
  delay(5);                                       // Small pause so the loop doesn't spin too hard.
}