/*
 * LeakSense P2 firmware
 * Integrates:
 *   - LeakSense flow/leak/publish pipeline (doc 1)
 *   - PIC18F06Q40 UART meter source (doc 2)  -> pic_link.*
 *
 * Kevin's requirements:
 *   #1 Ingest PIC data and align it to hourly buckets + the interval logger.
 *   #2 Push 4 cloud-set variables (alert + shutoff settings) down to the PIC.
 *   #3 Boron build kept compatible via PLATFORM guards (Kevin verifies HW).
 *   #4 Connect to the cloud only every 24 h (PIC/button wakes do NOT connect).
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
#include "pic_link.h"                 // The PicLink class for talking to the PIC chip.
#include "JsonParserGeneratorRK.h"    // A helper library to build JSON text we publish to the cloud.
#if USE_IMU                            // Only if the optional motion sensor is enabled (it is OFF by default)...
#include "Adafruit_LSM6DS33.h"         // ...include the IMU driver library.
#endif                                 // End of the optional include.

#include <fcntl.h>      // open() flags (O_WRONLY, O_CREAT, ...) for reading/writing files.
#include <unistd.h>     // read(), write(), close() for files.
#include <sys/stat.h>   // File status/permission helpers.
#include <string.h>     // memset(), strlen(), and similar C string/memory tools.
#include <math.h>       // sqrtf(), floorf(), and other math functions.

PRODUCT_VERSION(2);                            // Tag this firmware as product version 2 (for the Particle cloud).
SYSTEM_MODE(MANUAL);                           // We control Wi-Fi/cloud connection ourselves (not automatic).
SerialLogHandler logHandler(LOG_LEVEL_INFO);   // Send Log.info()/warn()/error() messages to the USB serial port.

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
  uint32_t day0_utc_midnight;                   // UTC time (seconds) of midnight at the start of this logging day.
  uint16_t start_slot;                          // Which slot index the buffer starts at.
  uint16_t count;                               // How many readings are currently stored.
  uint16_t raw[MAX_SAMPLES];                    // each 0..999 (0.01 GPM units)
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
retained float        hourlyData[24]     = {0.0f}; // Gallons used in each of the 24 hours of the current day.
retained int          lastHourIngested   = -1;     // The hour of the last sample we processed (-1 = none yet).
retained int          lastDayIngested    = -1;     // The day of the last sample we processed (-1 = none yet).
retained uint32_t     leakingEventCount  = 0;      // How many leak events happened since the last publish.
retained uint32_t     shutoffEventCount  = 0;      // How many valve-shutoff events since the last publish.
retained uint32_t     overflowEventCount = 0;      // How many overflow events since the last publish.
retained unsigned long nextPublishEpoch  = 0;      // UTC time (seconds) of the next scheduled 24 h cloud upload.
retained uint32_t     nextSampleAtUtc    = 0;      // UTC time of the next expected sampling boundary.
retained bool         triggerState       = false;  // true while water is actively flowing (keeps us awake).
retained unsigned long lastTriggerTime   = 0;      // UTC time of the most recent flowing sample.

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
enum SystemState { STATE_STARTUP, STATE_CONNECTING, STATE_MONITORING, STATE_SESSION_DONE };
SystemState currentState = STATE_STARTUP;        // We begin in the STARTUP phase.

bool          lastButtonState = false;           // The button's state on the previous loop (to detect a new press).
unsigned long lastPressTime   = 0;               // When the button was last accepted as pressed (for debounce).
volatile bool resetShutoff    = false;           // Set by a timer to request auto-clearing the shutoff ("volatile" = set in a callback).
volatile bool triggerPublish  = false;           // Set anywhere to request a cloud publish soon.

unsigned long sleepStart    = 0;                 // When the current awake window started.
unsigned long lastWakeTime  = 0;                 // When we last woke from sleep.
int           sleepCycleCount = 0;               // How many sleep cycles we have done (used by the awake-window logic).

// Persistence file paths (LittleFS, /usr/ is user space on P2/Gen3)
static const char *GMETER_PATH = "/usr/gmeter.dat";   // File where we save the gMeter logger buffer.
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
void ingestPicBatch(const PicSample *s, int n);  // Process a batch of n decoded PIC samples.
void serviceMeterFromPic(bool picInitiated);     // Pull and process data from the PIC.
void accumulateHourly(int hr, int day, float gallons);   // Add gallons into the right hourly bucket.
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

// Convert a flow-sensor frequency (in Hz) into a water-flow rate (gallons per minute).
static float freqToGpm(float freq) {
  if (freq <= 0.0f) return 0.0f;                 // No frequency means no flow -> 0 GPM.
  float f  = freq / (1.0f + (FLOW_C5 * freq + FLOW_C6));   // freq correction
                                                //   Apply a correction so high frequencies aren't over-counted.
  float g0 = FLOW_C0 * f;                                  // raw GPM
                                                //   First rough conversion to gallons-per-minute.
  float g  = g0 - (FLOW_C1 * g0 * g0 + FLOW_C2 * g0 + FLOW_C3);   // Apply the calibration polynomial to refine it.
  g *= flowCalScale;                             // Multiply by the user's calibration scale.
  return (g < 0.0f) ? 0.0f : g;                  // Never return a negative flow; clamp to 0.
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
  uint32_t now = Time.now();                      // Current UTC time in seconds.

  bool gOk = loadBlob(GMETER_PATH, &gMeter, sizeof(gMeter)) &&   // Try to load gMeter, AND check it looks valid:
             gMeter.count <= MAX_SAMPLES &&                       //   count must be within range,
             gMeter.day0_utc_midnight != 0 &&                     //   the start-of-day must be set,
             gMeter.day0_utc_midnight <= now;                     //   and not be in the future.
  if (!gOk) {                                     // If loading failed or the data looked wrong...
    gMeter.day0_utc_midnight = (now / 86400UL) * 86400UL;   // ...start a fresh day at today's UTC midnight.
    gMeter.start_slot = 0;                        // Reset the start slot.
    gMeter.count = 0;                             // Reset the sample count to empty.
    Log.info("PERSIST: gMeter fresh");            // Note that we started a fresh logger.
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

// Decide whether the reading 'gpm' at time 'tsUtc' should be treated as a leak.
bool senseLeak(uint32_t tsUtc, float gpm) {
  int s = slotOfWeekFromUtc(tsUtc);               // Which weekly slot this reading belongs to.

  if (!model.init[s]) { updateBaseline(s, gpm); return false; }   // Slot not learned yet: just learn, not a leak.

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
                                                  //   (Avoid teaching the model that leak-level flow is "normal".)
  return leakNow;                                 // Report the leak decision.
}

// =============================================================== PIC ingest
// ASSUMPTION (documented in README): the batch is a contiguous run of samples,
// each spanning PIC_SAMPLE_INTERVAL_SEC, the last one ending at the most recent
// interval boundary <= now. If the PIC stamps real times, replace this mapping.
void ingestPicBatch(const PicSample *s, int n) {
  if (n <= 0) return;                             // Nothing to do if the batch is empty.
  if (!Time.isValid()) { Log.warn("PIC: time invalid, dropping batch"); return; }   // Need a valid clock to place samples.

  uint32_t now = Time.now();                      // Current UTC time in seconds.
  uint32_t endBoundary = (now / PIC_SAMPLE_INTERVAL_SEC) * PIC_SAMPLE_INTERVAL_SEC;   // Most recent 60-s boundary <= now.

  bool     flowSeen   = false;                    // any sample in this batch shows water moving
  uint32_t lastFlowTs = 0;                        // epoch of the most recent flowing sample

  for (int k = 0; k < n; k++) {                   // Loop over every sample in the batch.
    uint32_t tsEnd = endBoundary - (uint32_t)(n - 1 - k) * PIC_SAMPLE_INTERVAL_SEC;
                                                  //   Compute each sample's end-time: the last sample ends at
                                                  //   endBoundary, earlier samples step back 60 s each.

    float freq    = (float)s[k].pulses / (float)PIC_SAMPLE_INTERVAL_SEC; // Hz
                                                  //   Frequency = pulses counted / seconds in the interval.
    float gpm     = freqToGpm(freq);              // Convert that frequency to gallons-per-minute.
    float gallons = gpm * (PIC_SAMPLE_INTERVAL_SEC / 60.0f) / FLOW_C4;   // Gallons used in this interval (with trickle fix).

    // Show each received measurement so the raw data is visible in the log
    // (not just the batch count and the running daily total).
    Log.info("PIC sample[%d/%d]: idx=%u pulses=%u -> %.3f GPM, %.4f gal",   // Print this sample's details to the log.
             k + 1, n, s[k].index, s[k].pulses, gpm, gallons);

    if (gpm >= FLOW_ACTIVE_GPM) { flowSeen = true; lastFlowTs = tsEnd; }   // If flowing, remember it and the time.

    int hr  = Time.hour(tsEnd);                   // Which hour-of-day this sample ends in.
    int day = Time.day(tsEnd);                    // Which day-of-month this sample ends in.
    accumulateHourly(hr, day, gallons);           // Add the gallons into the correct hourly bucket.
    dailyGallons += gallons;                      // Add to the running daily total.

    if (senseLeak(tsEnd, gpm)) onLeakDetected();  // Run the leak detector; react if it says "leak".

    appendIntervalSample(gpm);                    // Store this reading in the interval logger.
  }

  // Flow-in-progress latch. This is the ONLY place per-sample flow is known,
  // so triggerState is set here; runSleep() releases it after the idle timeout.
  if (flowSeen) {                                 // If any sample in the batch showed flow...
    triggerState    = true;                       // ...mark "flow in progress" (keeps the device awake).
    lastTriggerTime = lastFlowTs;                 // Remember when that flow last happened.
  }

  imu_data.dailyGallons = dailyGallons;           // Mirror the daily total into the status struct (for publishing).
  Log.info("PIC: ingested %d samples, daily=%.2f gal%s",   // Summarize the batch in the log.
           n, dailyGallons, flowSeen ? " [flow active]" : "");

  // Compact one-line dump of THIS batch's raw per-capture pulse counts, so the whole
  // series is visible at a glance in addition to the per-sample lines above. Bounded:
  // a batch can be up to PIC_MAX_SAMPLES, so we cap the line length and note any
  // remainder (the per-sample "PIC sample[i]:" lines above are the full record).
  {
    const int kMaxShown = 80;                     // Keep the log line to a sane length.
    char pbuf[560];                               // 80 * up-to-6 chars ("65535,") fits with margin.
    int  ppos = 0;
    int  shown = (n < kMaxShown) ? n : kMaxShown; // How many pulse counts we actually print.
    for (int k = 0; k < shown; k++) {             // Append each pulse count, comma-separated.
      ppos += snprintf(pbuf + ppos, sizeof(pbuf) - ppos,
                       (k == 0) ? "%u" : ",%u", (unsigned)s[k].pulses);   // k=0 has no leading comma.
    }
    if (n > shown)                                // If the batch was longer than we printed...
      snprintf(pbuf + ppos, sizeof(pbuf) - ppos, ",...(+%d more)", n - shown);   // ...note the remainder.
    Log.info("PIC samples: [%s]", pbuf);          // Raw per-capture pulses for this batch.
  }
}

// Add 'gallons' into the bucket for hour 'hr'; also handle hour/day rollovers.
void accumulateHourly(int hr, int day, float gallons) {
  if (lastHourIngested == -1) {                   // first ever sample
    lastHourIngested = hr;                        //   Remember this hour as the starting point.
    lastDayIngested  = day;                       //   ...and this day.
  } else if (hr != lastHourIngested) {            // If we've moved into a new hour...
    Log.info("Hour %d done: %.2f gal -> hour %d",  //   ...log how much the finished hour used.
             lastHourIngested, hourlyData[lastHourIngested], hr);
    if (hr == 0 && day != lastDayIngested) {      // midnight rollover
                                                  //   New hour is 0 AND the day changed = a new calendar day began.
      Log.info("Midnight: daily=%.2f gal reset", dailyGallons);   // Log the day's final total before clearing.
      for (int i = 0; i < 24; i++) hourlyData[i] = 0.0f;   // Clear all 24 hourly buckets for the new day.
      dailyGallons = 0.0f;                         // ONLY reset here (doc 1 bug fixed)
                                                  //   Reset the daily total (this is the single correct place to do it).
    }
    lastHourIngested = hr;                        // Update the "current hour" marker.
    lastDayIngested  = day;                       // Update the "current day" marker.
  }
  hourlyData[hr] += gallons;                      // Finally, add the gallons into this hour's bucket.
}

// One logger entry per PIC sample (intervals are aligned 1:1).
void appendIntervalSample(float gpm) {
  if (gMeter.count >= MAX_SAMPLES) return;        // hold until daily publish/roll
                                                  //   If the day's buffer is full, stop adding (it rolls over at publish).
  uint16_t val = (uint16_t)(gpm * 100.0f + 0.5f); // 0.01 GPM units
                                                  //   Store GPM as an integer in hundredths (e.g. 1.23 GPM -> 123). +0.5 rounds.
  if (val > 999u) val = 999u;                     // Cap at 999 so each value fits the logger's 3-digit slot.
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
  if (gpm > 0 && Time.isValid()) {                // Only proceed if there is real flow and a valid clock.
    uint32_t now = Time.now();                    // Current UTC time.
    float gallons = gpm * ((nowMs - lastCalc) / 60000.0f) / FLOW_C4;   // Gallons over the elapsed minutes (ms/60000).
    accumulateHourly(Time.hour(now), Time.day(now), gallons);   // Add into the right hourly bucket.
    dailyGallons += gallons;                      // Add to the daily total.
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
  char buf[200];                                  // Holds up to 24 comma-separated "%.1f" values.
  int  pos = 0;
  for (int i = 0; i < 24 && pos < (int)sizeof(buf) - 12; i++) {   // Leave margin for one more field + null.
    pos += snprintf(buf + pos, sizeof(buf) - pos,
                    (i == 0) ? "%.1f" : ",%.1f", roundTenth(hourlyData[i]));   // hh=0 has no leading comma.
  }
  Log.info("hourlyGallons[24]=[%s] dailyGal=%.1f",   // Same numbers the cloud publishes.
           buf, roundTenth(dailyGallons));
}

// =============================================================== Publishing
// Publish the interval logger to the cloud in chunks of up to 120 samples each.
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

    JsonWriterStatic<512> jw;                     // A 512-byte JSON builder for this chunk's message.
    {                                             // Inner scope so the JSON object auto-closes at the brace.
      JsonWriterAutoObject obj(&jw);              // Begin a JSON object { ... } (auto-finished when 'obj' ends).
      jw.insertKeyValue("platform", PLATFORM_STR);          // "platform": board name.
      jw.insertKeyValue("day0Utc", (int)gMeter.day0_utc_midnight);   // "day0Utc": start-of-day UTC seconds.
      jw.insertKeyValue("intervalSec", (int)METER_INTERVAL_SEC);     // "intervalSec": seconds per sample (60).
      jw.insertKeyValue("chunk", (int)(c + 1));             // "chunk": this chunk's number (1-based).
      jw.insertKeyValue("totalChunks", (int)total);         // "totalChunks": how many chunks total.
      jw.insertKeyValue("sampleStart", (int)start);         // "sampleStart": index of the first sample here.
      jw.insertKeyValue("sampleCount", (int)cnt);           // "sampleCount": how many samples in this chunk.
      jw.insertKeyValue("data", packed);                    // "data": the packed 3-digit sample string.
    }
    Particle.publish("meterIntervals", jw.getBuffer());     // Send this chunk to the cloud as a "meterIntervals" event.
    delay(1100);                                  // Wait 1.1 s between publishes (the cloud rate-limits to ~1/sec).
  }
}

// Build and publish the main "sensorData" status message to the cloud.
void imuPublish() {
  imuGet();                                       // Refresh IMU values first (no-op if no IMU).

#if USE_WIFI                                       // On Wi-Fi boards, make sure Wi-Fi is up before publishing...
  if (!WiFi.ready()) { WiFi.connect(); if (!waitFor(WiFi.ready, WIFI_CONNECT_TIMEOUT_MS)) { Log.error("WiFi fail"); return; } }
                                                  //   If not ready, connect and wait; give up (return) if it times out.
#elif USE_CELLULAR                                 // On cellular boards, do the same with the modem...
  if (!Cellular.ready()) { Cellular.connect(); if (!waitFor(Cellular.ready, CELL_CONNECT_TIMEOUT_MS)) { Log.error("Cell fail"); return; } }
#endif

  JsonWriterStatic<512> jw;                       // A 512-byte JSON builder for the status message.
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
    jw.insertKeyArray("hourlyGallons");           // Begin an array "hourlyGallons": [ ... ].
    for (int i = 0; i < 24; i++) jw.insertArrayValue(roundTenth(hourlyData[i]));   // Add each hour's gallons (1 decimal).
    jw.finishObjectOrArray();                     // Close the hourlyGallons array.
#if USE_WIFI
    jw.insertKeyValue("rssi", (int)WiFi.RSSI().getStrength());          // Wi-Fi signal strength.
#elif USE_CELLULAR
    jw.insertKeyValue("rssi", (int)Cellular.RSSI().getStrength());      // Cellular signal strength.
#endif
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

  Particle.publish("sensorData", jw.getBuffer()); // Send the finished JSON as a "sensorData" cloud event.
  Log.info("Published: %s", jw.getBuffer());      // Also log exactly what we published.
  publishIntervalDataChunks();                    // Then publish the detailed interval logger in chunks.

  // Roll to a fresh UTC-day window after a full-day buffer was sent.
  if (gMeter.count >= MAX_SAMPLES) {              // If the logger filled a whole day...
    uint32_t now = Time.now();                    //   get the current time...
    gMeter.day0_utc_midnight = (now / 86400UL) * 86400UL;   //   set the new day's midnight,
    gMeter.start_slot = 0;                        //   reset the start slot,
    gMeter.count = 0;                             //   and empty the buffer for a fresh day.
    Log.info("gMeter rolled");                    //   Log the rollover.
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
// use setLeakParams (REQ_SET_PARAM). Kept for dashboard backward-compat.
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

// ---- PIC leak parameters (REQ_GET/SET_PARAM) -------------------------------
// Push the cached picParams to the PIC. Returns true on ACK (spec 5.4).
bool pushPicParams() {
  if (picLink.setParams(picParams)) {             // Try to write the cached params to the PIC.
    picParamsDirty = false;                        // On ACK, clear the "still owe a write" flag.
    Log.info("PIC: params delivered (ACK)");       // Log success.
    return true;                                   // Report success.
  }
  Log.warn("PIC: SET_PARAM failed (nak=0x%02X)", picLink.lastNak());   // Log the failure + NAK reason.
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

// picReset: PKT_SYS_RESET. No reply; also clears the PIC's permanent valve lock.
int picReset(String cmd) {
  (void)cmd;                                      // Argument unused.
  restartSleepTimer("picReset");                  // Reset the awake window.
  picLink.sysReset();                             // Send the reset command to the PIC.
  Log.info("PIC: SYS_RESET sent");                // Log that we sent it.
  return 1;                                        // Success (we don't wait for a reply).
}

// Force-push the cached PIC leak parameters now (REQ_SET_PARAM).
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

  int n = picLink.requestData(picBuf, PIC_MAX_SAMPLES);   // Ask the PIC for data; n = sample count or an error.
  if (n > 0) {                                    // Got real samples...
    ingestPicBatch(picBuf, n);                    // ...process them...
    persistAll();                                 // ...and save to flash.
  } else if (n == 0) {                            // PIC has nothing new...
    wakeSessionDrained = true;          // PIC has nothing new -> stop this session
  } else {                                        // n < 0 means an error occurred.
    Log.warn("PIC REQ_DATA err %d", n);           // Log the error code.
  }

  if (picParamsDirty) pushPicParams();            // If we owe the PIC a params write, try it while connected.
  // readValveStatus() removed from here (Fix 2): valve is read at publish time
  // and right after a setParams()/unlockValve(), not on every data request.
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
  bool netUp =
#if USE_WIFI
      WiFi.ready();                               // Wi-Fi interface up?
#elif USE_CELLULAR
      Cellular.ready();                           // Cellular interface up?
#else
      true;
#endif
  if (netUp && Particle.connected()) {            // Network + cloud both up...
    changeState(STATE_MONITORING);                // ...move on to pull data + publish.
    triggerPublish = true;                        // This session should publish once.
    return;
  }

  // Cloud-fail path: the PIC powers us off unconditionally at 90 s if it has
  // heard nothing, so we must send our reason first. millis() is time since
  // power-on, so this bounds the whole boot+connect attempt to 80 s.
  if (millis() >= TIMEOUT_CANNOT_FIND_CLOUD_MS) { // Ran out of time to reach the cloud...
    Log.warn("CLOUD: unreachable within %lu ms -> PHOTON_OFF(CLOUD_FAIL)",
             (unsigned long)TIMEOUT_CANNOT_FIND_CLOUD_MS);
    picLink.sendPhotonOff(OFF_REASON_CLOUD_FAIL); // AA 55 07 00 01 01 00 C1
    endSession();                                 // Idle; the PIC removes power before its 90 s mark.
  }
}

// The MONITORING phase (power-gating): a one-shot report. Pull the PIC's data,
// publish it, do any other pending business, then send PKT_PHOTON_OFF_REQ (0x07)
// so the PIC cuts our power. Runs once per session, not as a repeating poll.
// In FAST_BENCH_TEST the cloud/publish steps are compiled out: we only do the
// PIC UART exchange, log it over USB, and still end with func 0x07.
void handleMonitoring() {
#ifndef FAST_BENCH_TEST
  // (cloud build) We only reach here once the cloud is connected. Publishing and
  // interval scheduling need a valid clock; wait for the cloud time sync, but
  // still inside the 80 s budget so a stuck sync doesn't miss the PIC's 90 s cutoff.
  if (!Time.isValid()) {                          // Clock not synced from the cloud yet...
    if (millis() >= TIMEOUT_CANNOT_FIND_CLOUD_MS) {   // ...and we're out of time...
      Log.warn("CLOUD: connected but no time sync in %lu ms -> PHOTON_OFF(CLOUD_FAIL)",
               (unsigned long)TIMEOUT_CANNOT_FIND_CLOUD_MS);
      picLink.sendPhotonOff(OFF_REASON_CLOUD_FAIL);   // Report the failure so the PIC shuts us down cleanly.
      endSession();
    }
    return;                                       // Otherwise keep waiting for time.
  }

  // First-pass housekeeping: align the (optional) local sampler to the next boundary.
  uint32_t now = Time.now();                      // Current UTC time in seconds.
  if (nextSampleAtUtc == 0 || nextSampleAtUtc <= now)   // If the next-sample time is unset or past...
    nextSampleAtUtc = ((now / METER_INTERVAL_SEC) + 1) * METER_INTERVAL_SEC;   // ...set it to the next 60 s boundary.
#endif // !FAST_BENCH_TEST

  // 1) Pull the meter batch from the PIC (REQ_DATA -> RSP_DATA). No D10 gate --
  //    we are powered *because* the PIC wants this session, so we just ask. The
  //    PIC may have more than one chunk queued, so re-request until it reports an
  //    empty batch (COUNT=0 -> wakeSessionDrained) or we hit the request cap.
  //    This is the exact exchange FAST_BENCH_TEST exists to watch over USB.
  wakeSessionDrained = false;                     // Start a fresh drain for this session.
  for (uint8_t i = 0; i < PIC_DATA_MAX_REQUESTS && !wakeSessionDrained; i++) {
    serviceMeterFromPic(true);                    // REQ_DATA -> ingest samples (COUNT=0 is normal, not an error).
  }

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
  printHourlyFlow();                              // hourlyGallons[24]=[...] dailyGal=... (both builds, post-ingest).
#ifndef FAST_BENCH_TEST
  imuPublish();                                   // Publish "sensorData" + interval chunks (cloud build).
  persistAll();                                   // Flush RAM buffers to flash before power is cut.
  triggerPublish = false;                         // Report delivered.
#endif

  // 4) Done with all business -> ask the PIC to cut our power. Sending 0x07 here
  //    (rather than waiting for the PIC's 20 s idle backstop) is the normal, clean
  //    end of a session: the PIC drives RC4 HIGH -> P-MOS off -> Photon unpowered.
  //    Identical in both builds -- fast mode keeps production timing, minus cloud.
#ifdef FAST_BENCH_TEST
  Log.info("FAST_BENCH_TEST: PIC exchange done -> PHOTON_OFF(DONE)");
#else
  Log.info("SESSION report published -> PHOTON_OFF(DONE)");
#endif
  picLink.sendPhotonOff(OFF_REASON_DONE);         // AA 55 07 00 01 00 C0 00
  endSession();                                   // Idle until the PIC removes power.
}

// =============================================================== Arduino entry
// setup() runs ONCE at power-on/reset. It prepares everything the program needs.
void setup() {
  Serial.begin(115200);                           // Start the USB serial monitor at 115200 baud.
  waitFor(Serial.isConnected, SERIAL_CONNECT_MS); // step 2: OS enumerated the CDC device
#ifdef FAST_BENCH_TEST
  delay(POST_USB_OPEN_MS);                        // step 3 margin: let the monitor re-open the port
#endif
  Time.zone(-8);                 // PST display; logger math stays in UTC
                                 //   Show local time as UTC-8 (Pacific); internal math still uses UTC.
  Log.info("LeakSense P2 boot on %s", PLATFORM_STR);   // Announce boot and which board we are.

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

  picLink.begin(38400);          // Serial1 (+ D10 held pulled-up; unused under power-gating)
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

#if USE_LOCAL_METER
  attachInterrupt(METER_PIN, countPulse, CHANGE); // If local metering, call countPulse() on every pin change.
#endif

#ifdef FAST_BENCH_TEST
  // Fast bench-test: NO cloud. Skip every connect -- loop() -> handleMonitoring()
  // will do the PIC UART exchange, log it over USB, and end with func 0x07. The
  // production connect/publish path is compiled out entirely (see app_config.h).
  sleepStart = millis(); lastWakeTime = millis(); // Keep timers sane for helpers that read them.
  changeState(STATE_MONITORING);                  // Go straight to the PIC exchange (no CONNECTING phase).
  Log.info("Setup complete -> FAST_BENCH_TEST (no cloud; PIC UART + USB log only)");
#else
  // Power-gating (PIC V048): the PIC applied our power because a report is due.
  // Kick off the cloud connection WITHOUT blocking here -- handleConnecting() in
  // loop() finishes it within TIMEOUT_CANNOT_FIND_CLOUD_MS (measured from power-on)
  // or sends OFF_REASON_CLOUD_FAIL, then handleMonitoring() does one report and
  // ends the session with PKT_PHOTON_OFF_REQ (0x07). We never self-sleep now.
#if USE_WIFI
  WiFi.on(); WiFi.connect();                      // Start Wi-Fi (non-blocking).
#elif USE_CELLULAR
  Cellular.on(); Cellular.connect();              // Start cellular (non-blocking).
#endif
  Particle.connect();                             // Start connecting to the Particle cloud (non-blocking).

  triggerPublish = true;                          // This session should publish once.
  sleepStart = millis(); lastWakeTime = millis(); // Keep timers sane for helpers that read them.
  changeState(STATE_CONNECTING);                  // Let loop() drive: connect -> report -> 0x07.
  Log.info("Setup complete -> connecting (power-gating session)");   // Announce setup finished.
#endif
}

// loop() runs over and over after setup(). Under power-gating it drives ONE
// session (connect -> report -> ask the PIC to cut power), then idles until the
// PIC removes power. The Photon never sleeps itself anymore.
void loop() {
  Particle.process();   // Service the cloud link every pass (required in SYSTEM_MODE(MANUAL)).

  switch (currentState) {                         // Act based on the current session phase.
    case STATE_STARTUP:                           // (STARTUP falls through to CONNECTING.)
    case STATE_CONNECTING:  handleConnecting();  break;   // Bring up the cloud (bounded) or CLOUD_FAIL out.
    case STATE_MONITORING:  handleMonitoring();  break;   // Connected: pull PIC data, publish, then 0x07.
    case STATE_SESSION_DONE:                              // 0x07 already sent...
      // Session finished. The PIC will drive RC4 HIGH and remove our power very
      // shortly (it acts on our 0x07, or its 20 s idle / 90 s no-packet backstop).
      // There is nothing to do but idle -- the PIC owns power now.
      break;
  }
  delay(5);                                       // Small pause so the loop doesn't spin too hard.
}