#include "cloudUpdate.h"
#include "pic_link.h"   // PicParams -- read-only here, to snapshot the 3 leak fields a single-value setter doesn't touch.

DeviceConfig  cfg;
volatile bool g_configReceived = false;

/* Load cfg from EEPROM, falling back to defaults if no valid config
 * (bad magic) has been persisted yet. */
void loadCloudUpdateConfig(void) {
    EEPROM.get(CLOUD_UPDATE_EEPROM_ADDR, cfg);
    if (cfg.magic != CONFIG_MAGIC) {
        cfg = DeviceConfig();
        EEPROM.put(CLOUD_UPDATE_EEPROM_ADDR, cfg);
        Serial.printf("Config: no valid EEPROM config, using defaults\r\n");
    } else {
        Serial.printf("Config: loaded rev=%lu interval=%lus leak=%.2f pulse=%lu valve=%d\r\n",
                      (unsigned long)cfg.rev, (unsigned long)cfg.uploadIntervalSec,
                      cfg.leakGpmThreshold, (unsigned long)cfg.pulseFactor, (int)cfg.valveEnabled);
    }
}

/* Persist the current cfg to EEPROM. */
void saveCloudUpdateConfig(void) {
    EEPROM.put(CLOUD_UPDATE_EEPROM_ADDR, cfg);
}

/* ====================================================================
 *  SERVER-SIDE PARAMETER UPDATES -- Cal / a1Count / a1Win / a2Count / a2Win
 * ------------------------------------------------------------------
 *  One Particle.function() per value. These delegate straight to
 *  leaksense.cpp's real setFlowCal()/setLeakParams() -- i.e. the actual
 *  flowCalScale/picParams the meter and PIC use -- rather than keeping a
 *  local copy: writing only to this file's own DeviceConfig.cal/a1Count/...
 *  would leave the real state (and the "sensorData" event) untouched.
 *  setFlowCal()/setLeakParams() already do validation, EEPROM persistence,
 *  and (for leak params) the PIC push -- reusing them means this file owns
 *  no separate bounds/state for these five values. setLeakParams() takes
 *  all 4 leak fields at once (read-modify-write on the PIC), so the 4
 *  single-value setters below patch just their one field into a snapshot
 *  of the Photon's cached picParams for the other 3. */
extern int setFlowCal(String cmd);
extern int setLeakParams(String cmd);   // "l1c,l1w,l2c,l2w"
extern PicParams picParams;             // Photon's cached copy of the PIC's 4 leak params.

static constexpr uint32_t kCountsMin = 1, kCountsMax = 65000;   // matches PIC_COUNTS_MIN/MAX
static constexpr uint32_t kWindowMin = 1, kWindowMax = 65000;   // matches PIC_WINDOW_MIN/MAX

int setCal(String cmd) {
    return setFlowCal(cmd);   // identical operation; setFlowCal already validates+persists+publishes.
}

int setA1Count(String cmd) {
    cmd.trim();
    long v = atol(cmd.c_str());
    if (v < (long)kCountsMin || v > (long)kCountsMax) return -1;
    char params[48];
    snprintf(params, sizeof(params), "%ld,%u,%u,%u", v,
             (unsigned)picParams.leak1_window_s, (unsigned)picParams.leak2_counts, (unsigned)picParams.leak2_window_s);
    return setLeakParams(String(params));
}

int setA1Win(String cmd) {
    cmd.trim();
    long v = atol(cmd.c_str());
    if (v < (long)kWindowMin || v > (long)kWindowMax) return -1;
    char params[48];
    snprintf(params, sizeof(params), "%u,%ld,%u,%u",
             (unsigned)picParams.leak1_counts, v, (unsigned)picParams.leak2_counts, (unsigned)picParams.leak2_window_s);
    return setLeakParams(String(params));
}

int setA2Count(String cmd) {
    cmd.trim();
    long v = atol(cmd.c_str());
    if (v < (long)kCountsMin || v > (long)kCountsMax) return -1;
    char params[48];
    snprintf(params, sizeof(params), "%u,%u,%ld,%u",
             (unsigned)picParams.leak1_counts, (unsigned)picParams.leak1_window_s, v, (unsigned)picParams.leak2_window_s);
    return setLeakParams(String(params));
}

int setA2Win(String cmd) {
    cmd.trim();
    long v = atol(cmd.c_str());
    if (v < (long)kWindowMin || v > (long)kWindowMax) return -1;
    char params[48];
    snprintf(params, sizeof(params), "%u,%u,%u,%ld",
             (unsigned)picParams.leak1_counts, (unsigned)picParams.leak1_window_s, (unsigned)picParams.leak2_counts, v);
    return setLeakParams(String(params));
}

void registerCloudUpdateFunctions(void) {
    Particle.function("setCal",     setCal);
    Particle.function("setA1Count", setA1Count);
    Particle.function("setA1Win",   setA1Win);
    Particle.function("setA2Count", setA2Count);
    Particle.function("setA2Win",   setA2Win);
}

/* Extract a quoted string value for "key":"value" into out (size n). */
static void extractStringField(const char *data, const char *key, char *out, size_t n) {
    const char *p = strstr(data, key);
    if (!p) return;
    p += strlen(key);
    if (*p != '"') return;
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i < n - 1) out[i++] = *p++;
    out[i] = '\0';
}

/* Minimal hand-rolled field extraction (no JSON library dependency in this
 * single-file build) for the fixed "device/check_config" hook-response schema:
 *   {"update":true,"config_rev":18,"upload_interval_sec":1800,
 *    "leak_gpm_threshold":0.45,"pulse_factor":450,"valve_enabled":true,
 *    "cal":1.0,"a1_count":100,"a1_win":480,"a2_count":400,"a2_win":180,
 *    "ssid":"...","password":"..."}
 * or {"update":false} when the device is already current. */
void configHandler(const char *event, const char *data) {
    (void)event;
    if (!data || !strstr(data, "\"update\":true")) {
        Serial.printf("Config: server says up to date (rev=%lu)\r\n", (unsigned long)cfg.rev);
        g_configReceived = true;
        return;
    }

    const char *p;
    if ((p = strstr(data, "\"config_rev\":")))
        cfg.rev = (uint32_t)atol(p + strlen("\"config_rev\":"));
    if ((p = strstr(data, "\"upload_interval_sec\":")))
        cfg.uploadIntervalSec = (uint32_t)atol(p + strlen("\"upload_interval_sec\":"));
    if ((p = strstr(data, "\"leak_gpm_threshold\":")))
        cfg.leakGpmThreshold = (float)atof(p + strlen("\"leak_gpm_threshold\":"));
    if ((p = strstr(data, "\"pulse_factor\":")))
        cfg.pulseFactor = (uint32_t)atol(p + strlen("\"pulse_factor\":"));
    if (strstr(data, "\"valve_enabled\":true"))
        cfg.valveEnabled = true;
    else if (strstr(data, "\"valve_enabled\":false"))
        cfg.valveEnabled = false;

    /* Real PIC-side tuning params -- applied via the real setCal()/
     * setA1Count()/setA1Win()/setA2Count()/setA2Win() (delegating to
     * setFlowCal()/setLeakParams()), so a value the server pushes here
     * actually takes effect: validated, persisted to the real EEPROM
     * addresses, and (for leak params) pushed to the PIC -- not just
     * cached in this file's local cfg mirror. cfg.cal/a1Count/... are kept
     * in sync too, as a record of "what the server last told us". */
    char buf[16];
    if ((p = strstr(data, "\"cal\":"))) {
        cfg.cal = (float)atof(p + strlen("\"cal\":"));
        snprintf(buf, sizeof(buf), "%.4f", cfg.cal);
        setCal(String(buf));
    }
    if ((p = strstr(data, "\"a1_count\":"))) {
        cfg.a1Count = (uint32_t)atol(p + strlen("\"a1_count\":"));
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)cfg.a1Count);
        setA1Count(String(buf));
    }
    if ((p = strstr(data, "\"a1_win\":"))) {
        cfg.a1Win = (uint32_t)atol(p + strlen("\"a1_win\":"));
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)cfg.a1Win);
        setA1Win(String(buf));
    }
    if ((p = strstr(data, "\"a2_count\":"))) {
        cfg.a2Count = (uint32_t)atol(p + strlen("\"a2_count\":"));
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)cfg.a2Count);
        setA2Count(String(buf));
    }
    if ((p = strstr(data, "\"a2_win\":"))) {
        cfg.a2Win = (uint32_t)atol(p + strlen("\"a2_win\":"));
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)cfg.a2Win);
        setA2Win(String(buf));
    }

#if USE_WIFI
    /* WiFi credentials -- one-way, applied immediately, no confirmation.
     * Not applicable on USE_CELLULAR builds (no SSID/password to set). */
    char newSsid[64] = {0};
    extractStringField(data, "\"ssid\":", newSsid, sizeof(newSsid));
    if (newSsid[0] != '\0' && strcmp(newSsid, cfg.ssid) != 0) {
        char newPass[64] = {0};
        extractStringField(data, "\"password\":", newPass, sizeof(newPass));
        strncpy(cfg.ssid, newSsid, sizeof(cfg.ssid) - 1);
        strncpy(cfg.password, newPass, sizeof(cfg.password) - 1);
        WiFi.setCredentials(cfg.ssid, cfg.password);
        Serial.printf("Config: new WiFi credentials saved for \"%s\"\r\n", cfg.ssid);
    }
#endif

    saveCloudUpdateConfig();
    Serial.printf("Config: updated rev=%lu interval=%lus leak=%.2f pulse=%lu valve=%d "
                  "cal=%.3f a1=%lu/%lu a2=%lu/%lu\r\n",
                  (unsigned long)cfg.rev, (unsigned long)cfg.uploadIntervalSec,
                  cfg.leakGpmThreshold, (unsigned long)cfg.pulseFactor, (int)cfg.valveEnabled,
                  cfg.cal, (unsigned long)cfg.a1Count, (unsigned long)cfg.a1Win,
                  (unsigned long)cfg.a2Count, (unsigned long)cfg.a2Win);
    g_configReceived = true;
}

/* Join the network transport for this build (WiFi or Cellular), returning
 * once ready or once timeoutMs elapses. */
#if USE_WIFI
bool connectNetwork(uint32_t timeoutMs) {
    if (WiFi.ready()) return true;
    WiFi.on();
    WiFi.connect();
    return waitFor(WiFi.ready, timeoutMs);
}
#elif USE_CELLULAR
bool connectNetwork(uint32_t timeoutMs) {
    if (Cellular.ready()) return true;
    Cellular.on();
    Cellular.connect();
    return waitFor(Cellular.ready, timeoutMs);
}
#endif

/* Current radio signal strength (dBm). CellularSignal implicitly converts
 * to int (RSSI) for backwards compatibility with the WiFi.RSSI() shape. */
int getSignalStrength(void) {
#if USE_WIFI
    return WiFi.RSSI();
#elif USE_CELLULAR
    return Cellular.RSSI();
#endif
}

/* ====================================================================
 *  PARTICLE CLOUD
 * ==================================================================== */
/* Connect to the Particle Cloud, returning once connected or once
 * timeoutMs elapses. Network transport must already be up. */
bool connectParticleCloud(uint32_t timeoutMs) {
    if (Particle.connected()) return true;
    Particle.connect();
    return waitFor(Particle.connected, timeoutMs);
}

/* Payload keys match real device schema exactly (pf/a1Events/
 * a2Events/shutoffs/Cal/a1Count/a1Win/a2Count/a2Win/picParamsDirty/rssi/
 * uptime) so the dashboard's confirmation logic -- which parses these
 * exact key names out of payload_json -- can match against real
 * device-reported values and flip Pending -> Confirmed for real.
 * Cal/a1Count/a1Win/a2Count/a2Win are genuinely accurate: they're this
 * device's own currently-applied config, not fabricated. a1Events/
 * a2Events/shutoffs/picParamsDirty are honestly 0 -- there's no PIC
 * attached to this bench unit generating real events or needing a param
 * write, so 0 is the true state. Deliberately omits hourlyGallons/
 * battery/totalGallons -- no meter/battery hardware wired up here, so
 * those would have to be faked outright. */
void publishSensorData(void) {
    char payload[512];
    snprintf(payload, sizeof(payload),
             "{\"pf\":\"" PLATFORM_STR "\",\"a1Events\":0,\"a2Events\":0,\"shutoffs\":0,"
             "\"Cal\":%.6f,\"a1Count\":%lu,\"a1Win\":%lu,\"a2Count\":%lu,\"a2Win\":%lu,"
             "\"picParamsDirty\":0,\"rssi\":%d,\"uptime\":%lu}",
             cfg.cal, (unsigned long)cfg.a1Count, (unsigned long)cfg.a1Win,
             (unsigned long)cfg.a2Count, (unsigned long)cfg.a2Win,
             getSignalStrength(),
             (unsigned long)System.uptime());

    /* Particle.publish() is async (queued on the system thread) and, without
     * WITH_ACK, the returned Future resolves as soon as the event is handed
     * off locally -- NOT once the cloud actually receives it. On cellular's
     * higher-latency, lossier link that let events silently vanish in transit
     * while isSucceeded() still reported true (no error, nothing in Console).
     * WITH_ACK makes success mean the cloud genuinely acknowledged it. */
    particle::Future<bool> pub = Particle.publish("sensorData", payload, PRIVATE, WITH_ACK);
    if (!pub.wait(10000) || !pub.isSucceeded()) {
        Serial.printf("Publish: sensorData failed or timed out\r\n");
    }
}

/* ====================================================================
 *  BOOT-TIME SERVER CONFIG PULL
 * ------------------------------------------------------------------
 *  This device is power-gated: it's normally off, and only online for a
 *  few seconds once per report period. A live setCal()/setLeakParams()
 *  Particle.function() call from the dashboard can easily miss that
 *  window entirely. fetchServerConfig() flips the direction: the DEVICE
 *  asks the server (via the "device/check_config" webhook) on every boot,
 *  so a change queued at any time is picked up on the very next session
 *  instead of depending on a live RPC landing in a narrow window.
 *  configHandler() (the hook-response handler, subscribed here) does the
 *  actual apply: it calls setCal()/setA1Count()/setA1Win()/setA2Count()/
 *  setA2Win() for whatever fields the response contains, which validates,
 *  persists to the real EEPROM addresses, and (for leak params) pushes to
 *  the PIC -- same effect as if the dashboard had called them directly.
 *  Every step here is bounded: a PIC-less bench unit, a dead network, or a
 *  webhook that never answers all just fall through and setup() continues
 *  with whatever Cal/leak-params were already persisted from last time. */
 
static const uint32_t SERVER_CFG_NET_TIMEOUT_MS  = USE_CELLULAR ? 120000 : 10000; //= 10000UL;
static const uint32_t SERVER_CFG_CLOUD_TIMEOUT_MS = USE_CELLULAR ? 30000 : 10000; // = 10000UL;
static const uint32_t SERVER_CFG_REPLY_TIMEOUT_MS = USE_CELLULAR ? 30000 : 5000;  // = 5000UL;

void fetchServerConfig(void) {
    Particle.subscribe("hook-response/device/check_config", configHandler, MY_DEVICES);

    if (!connectNetwork(SERVER_CFG_NET_TIMEOUT_MS)) {
        Serial.printf("ServerConfig: network unreachable in %lus -- skipping\r\n",
                      (unsigned long)(SERVER_CFG_NET_TIMEOUT_MS / 1000));
        return;
    }
    if (!connectParticleCloud(SERVER_CFG_CLOUD_TIMEOUT_MS)) {
        Serial.printf("ServerConfig: cloud unreachable in %lus -- skipping\r\n",
                      (unsigned long)(SERVER_CFG_CLOUD_TIMEOUT_MS / 1000));
        return;
    }

    g_configReceived = false;
    particle::Future<bool> pub = Particle.publish("device/check_config", "{}", PRIVATE, WITH_ACK);
    if (!pub.wait(5000) || !pub.isSucceeded()) {
        Serial.printf("ServerConfig: request publish failed/timed out\r\n");
        return;
    }

    uint32_t t0 = millis();
    while (!g_configReceived && (millis() - t0) < SERVER_CFG_REPLY_TIMEOUT_MS) {
        Particle.process();
    }
    if (!g_configReceived) {
        Serial.printf("ServerConfig: no hook-response within %lus -- keeping last-known values\r\n",
                      (unsigned long)(SERVER_CFG_REPLY_TIMEOUT_MS / 1000));
    }
}
