#ifndef CLOUD_UPDATE_H
#define CLOUD_UPDATE_H

#include "Particle.h"

/* ====================================================================
 *  NETWORK TRANSPORT SELECT -- Wiring_WiFi/Wiring_Cellular are set by
 *  Device OS per PLATFORM_ID (1 on exactly one of the two), so USE_WIFI/
 *  USE_CELLULAR fall out automatically for whichever target is compiled
 *  (p2/photon2 -> WiFi, boron -> Cellular). No per-build edits needed.
 * ==================================================================== */
#if Wiring_WiFi
  #define USE_WIFI     1
  #define USE_CELLULAR 0
  #define PLATFORM_STR "P2"
#elif Wiring_Cellular
  #define USE_WIFI     0
  #define USE_CELLULAR 1
  #define PLATFORM_STR "Boron"
#else
  #error "cloudUpdate: platform has neither WiFi nor Cellular -- add USE_WIFI/USE_CELLULAR support"
#endif
/* Named PLATFORM_STR, not PLATFORM_NAME: Device OS's own build already passes
 * -DPLATFORM_NAME=p2 (bareword, not a string) as a compiler flag, so reusing
 * that name here redefines it with different, incompatible replacement text
 * ("P2" the string vs p2 the bareword) -- a real warning, not a false one.
 * PLATFORM_STR also happens to match leaksense.cpp's app_config.h convention
 * (same values per platform), so it's a benign, identical redefinition in
 * any translation unit that includes both headers. */

/* ====================================================================
 *  DEVICE CONFIG -- fetched from the backend via the "device/check_config"
 *  webhook, persisted in EEPROM so it survives sleep/reset.
 * ==================================================================== */
static const uint32_t    CONFIG_MAGIC = 0xA55A1234;
static const char *const FW_VERSION   = "1.0.0";

/* leaksense.cpp already owns EEPROM 0 (flow-cal), 32 (app config), 64 (PIC
 * params) -- see FLOW_CAL_EEPROM_ADDR/CFG_EEPROM_ADDR/PICP_EEPROM_ADDR there.
 * This module links into the same firmware image, so it must NOT reuse any
 * of those addresses (that would silently corrupt whichever struct loses
 * the race). 128 starts well clear of PICP's ~16-byte footprint at 64. */
static const int CLOUD_UPDATE_EEPROM_ADDR = 128;

struct DeviceConfig {
    uint32_t magic             = CONFIG_MAGIC;
    uint32_t rev               = 0;
    uint32_t uploadIntervalSec = 1800;   /* 30 min default */
    float    leakGpmThreshold  = 0.5f;
    uint32_t pulseFactor       = 450;
    bool     valveEnabled      = true;
    /* Real PIC-side tuning params
     * NOTE: this bench firmware has no live PIC UART command to relay these
     * onward -- they're received, applied here, and persisted, but actually
     * forwarding them to a physical PIC18F04 needs a new command added to
     * the UART protocol (and PIC-side firmware this repo doesn't have). */
    float    cal               = 1.0f;
    uint32_t a1Count           = 100;
    uint32_t a1Win             = 480;
    uint32_t a2Count           = 400;
    uint32_t a2Win             = 180;
    /* WiFi credentials -- one-way, no confirmation tracked. Empty ssid
     * means "none set yet". */
    char     ssid[64]          = {0};
    char     password[64]      = {0};
};

extern DeviceConfig     cfg;
extern volatile bool    g_configReceived;

/* Named loadCloudUpdateConfig()/saveCloudUpdateConfig() (not loadConfig()/
 * saveConfig()) because leaksense.cpp already has its own global functions
 * with those exact names (for a different struct, AppConfig) -- both files
 * link into the same firmware image, and identically-named globals is a
 * link-time "multiple definition" error, not just a same-file clash. */
void loadCloudUpdateConfig(void);
void saveCloudUpdateConfig(void);

/* Hook-response handler for "device/check_config" -- parses the config
 * payload, applies it to cfg AND (for cal/a1Count/a1Win/a2Count/a2Win) to
 * the real flowCalScale/picParams via setCal()/setA1Count()/etc. below,
 * persists, and sets g_configReceived. */
void configHandler(const char *event, const char *data);

/* Server-side parameter updates -- one Particle.function() per value. These
 * delegate straight to leaksense.cpp's real setFlowCal()/setLeakParams(), so
 * they drive the actual flowCalScale/picParams the meter and PIC use (not a
 * local copy) -- same validation, EEPROM persist, and PIC push those already
 * do. Also called by configHandler() above when the server pushes a value.
 * Call registerCloudUpdateFunctions() once from setup() to expose all five
 * as Particle.function() RPCs. */
int  setCal(String cmd);        // Cal:      -> setFlowCal().
int  setA1Count(String cmd);    // a1Count:  -> setLeakParams(), other 3 fields held at their cached value.
int  setA1Win(String cmd);      // a1Win:    -> setLeakParams(), other 3 fields held at their cached value.
int  setA2Count(String cmd);    // a2Count:  -> setLeakParams(), other 3 fields held at their cached value.
int  setA2Win(String cmd);      // a2Win:    -> setLeakParams(), other 3 fields held at their cached value.
void registerCloudUpdateFunctions(void);

/* Join the network -- WiFi on USE_WIFI builds, Cellular on USE_CELLULAR
 * builds -- using stored credentials/SIM. Returns true once ready. */
bool connectNetwork(uint32_t timeoutMs);

/* Connect to the Particle Cloud. Returns true once Particle.connected(). */
bool connectParticleCloud(uint32_t timeoutMs);

/* Boot-time server config pull: connects (bounded), publishes
 * "device/check_config", and waits (bounded) for the hook-response, which
 * configHandler() applies -- so the server can push a new Cal/a1Count/
 * a1Win/a2Count/a2Win that lands even though this device is normally only
 * online for a few seconds per report period (too narrow a window to
 * reliably catch a live setCal()/setLeakParams() RPC call). Call once from
 * setup(). Safe to call even if the cloud/webhook never answers -- always
 * returns (never hangs), and the device just keeps its last-known values. */
void fetchServerConfig(void);

/* Current radio signal strength (dBm), from whichever transport is active. */
int getSignalStrength(void);

/* Publish this device's current config/status to the "sensorData" event. */
void publishSensorData(void);

#endif /* CLOUD_UPDATE_H */
