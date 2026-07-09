#define CODE1
// #define CODE2

#ifdef CODE1
#include "Particle.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

/* ====================================================================
 *  THINGSBOARD
 * ==================================================================== */
const char*    TB_HOST  = "thingsboard.cloud";
const uint16_t TB_PORT  = 80;
const char*    TB_TOKEN = "uYxLHvrbkFEjEi92DFWo"; // replace with your actual token


/* ====================================================================
 *  DEVICE CONFIG -- fetched from ThingsBoard shared attributes,
 *  persisted in EEPROM so it survives power cycles.
 * ==================================================================== */
static const uint32_t CONFIG_MAGIC = 0xA55A1234;

struct DeviceConfig {
    uint32_t magic             = CONFIG_MAGIC;
    uint32_t uploadIntervalSec = 1800;
    float    leakGpmThreshold  = 0.5f;
    uint32_t pulseFactor       = 450;
    bool     valveEnabled      = true;
};

DeviceConfig cfg;

void loadConfig(void) {
    EEPROM.get(0, cfg);
    if (cfg.magic != CONFIG_MAGIC) {
        cfg = DeviceConfig();
        EEPROM.put(0, cfg);
        Serial.printf("Config: no EEPROM config, using defaults\r\n");
    } else {
        Serial.printf("Config: loaded interval=%lus leak=%.2f pulse=%lu valve=%d\r\n",
                      (unsigned long)cfg.uploadIntervalSec, cfg.leakGpmThreshold,
                      (unsigned long)cfg.pulseFactor, (int)cfg.valveEnabled);
    }
}

void saveConfig(void) {
    EEPROM.put(0, cfg);
}

/* ====================================================================
 *  WIFI
 * ==================================================================== */
bool connectWiFi(uint32_t timeoutMs) {
    if (WiFi.ready()) return true;
    WiFi.on();
    WiFi.connect();
    return waitFor(WiFi.ready, timeoutMs);
}

/* ====================================================================
 *  THINGSBOARD HTTP
 * ==================================================================== */

/* POST a JSON telemetry body to ThingsBoard. */
bool tbPostTelemetry(const char *json) {
    TCPClient client;
    if (!client.connect(TB_HOST, TB_PORT)) {
        Serial.printf("TB: TCP connect failed\r\n");
        return false;
    }
    char req[512];
    int bodyLen = strlen(json);
    int reqLen = snprintf(req, sizeof(req),
        "POST /api/v1/%s/telemetry HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "\r\n"
        "%s",
        TB_TOKEN, TB_HOST, bodyLen, json);
    client.write((const uint8_t *)req, reqLen);
    uint32_t start = millis();
    while (client.connected() && millis() - start < 5000) {
        while (client.available()) Serial.write(client.read());
    }
    client.stop();
    return true;
}

/* GET shared attributes from ThingsBoard; copies HTTP response body into bodyBuf.
 * Returns true if any body bytes were received. */
bool tbGetAttributes(char *bodyBuf, uint16_t bufSize) {
    TCPClient client;
    if (!client.connect(TB_HOST, TB_PORT)) {
        Serial.printf("TB: TCP connect failed\r\n");
        return false;
    }
    char req[256];
    snprintf(req, sizeof(req),
        "GET /api/v1/%s/attributes?sharedKeys=uploadIntervalSec,leakGpmThreshold,pulseFactor,valveEnabled HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Connection: close\r\n"
        "\r\n",
        TB_TOKEN, TB_HOST);
    client.write((const uint8_t *)req, strlen(req));

    uint32_t start = millis();
    uint16_t idx = 0;
    bool inBody = false;
    char prev[4] = {0};
    while (client.connected() && millis() - start < 5000) {
        while (client.available() && idx < (uint16_t)(bufSize - 1)) {
            char c = (char)client.read();
            if (!inBody) {
                prev[0] = prev[1]; prev[1] = prev[2]; prev[2] = prev[3]; prev[3] = c;
                if (prev[0]=='\r' && prev[1]=='\n' && prev[2]=='\r' && prev[3]=='\n')
                    inBody = true;
            } else {
                bodyBuf[idx++] = c;
            }
        }
    }
    bodyBuf[idx] = '\0';
    client.stop();
    return idx > 0;
}

/* Parse TB shared-attribute response and update + save config.
 * Expected shape: {"shared":{"uploadIntervalSec":900,"valveEnabled":true,...}} */
void applyAttributes(const char *body) {
    const char *p;
    bool changed = false;
    if ((p = strstr(body, "\"uploadIntervalSec\":"))) {
        cfg.uploadIntervalSec = (uint32_t)atol(p + strlen("\"uploadIntervalSec\":"));
        changed = true;
    }
    if ((p = strstr(body, "\"leakGpmThreshold\":"))) {
        cfg.leakGpmThreshold = (float)atof(p + strlen("\"leakGpmThreshold\":"));
        changed = true;
    }
    if ((p = strstr(body, "\"pulseFactor\":"))) {
        cfg.pulseFactor = (uint32_t)atol(p + strlen("\"pulseFactor\":"));
        changed = true;
    }
    if (strstr(body, "\"valveEnabled\":true")) {
        cfg.valveEnabled = true; changed = true;
    } else if (strstr(body, "\"valveEnabled\":false")) {
        cfg.valveEnabled = false; changed = true;
    }
    if (changed) {
        saveConfig();
        Serial.printf("Config: saved interval=%lus leak=%.2f pulse=%lu valve=%d\r\n",
                      (unsigned long)cfg.uploadIntervalSec, cfg.leakGpmThreshold,
                      (unsigned long)cfg.pulseFactor, (int)cfg.valveEnabled);
    }
}

void setup(void) {
    Serial.begin();
    waitFor(Serial.isConnected, 300);
    loadConfig();

    for (;;) {
        if (connectWiFi(30000)) {
            char attrBody[512];
            if (tbGetAttributes(attrBody, sizeof(attrBody))) {
                Serial.printf("TB attrs: %s\r\n", attrBody);
                applyAttributes(attrBody);
            } else {
                Serial.printf("TB: no attribute response\r\n");
            }

            char tbJson[128];
            snprintf(tbJson, sizeof(tbJson),
                     "{\"interval\":%lu,\"leak\":%.2f,\"pulse\":%lu,\"valve\":%d}",
                     (unsigned long)cfg.uploadIntervalSec, cfg.leakGpmThreshold,
                     (unsigned long)cfg.pulseFactor, (int)cfg.valveEnabled);
            tbPostTelemetry(tbJson);

            WiFi.off();
        } else {
            Serial.printf("WiFi: connect timed out\r\n");
        }

        delay((unsigned long)cfg.uploadIntervalSec * 1000UL);
    }
}

void loop(void) {}

#endif
#ifdef CODE2
/*
 * Project myProject
 * Author: Your Name
 * Date:
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);



// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

const pin_t powPin = D3;
const pin_t pulsePin = D6;
const unsigned long pulseOnMs = 30UL * 1000UL;  // 30 seconds
const unsigned long pulseOffMs = 5UL * 60UL * 1000UL; // 5 minutes


// Button state tracking for edge detection + debounce
int btnState = HIGH;
int lastBtnState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

bool pulseOn = true;
unsigned long pulsePhaseStartMs = 0;

// setup() runs once, when the device is first turned on
void setup() {
  pinMode(powPin, OUTPUT);
  digitalWrite(powPin, HIGH);
  pinMode(pulsePin, OUTPUT);
  digitalWrite(pulsePin, HIGH);
  pulsePhaseStartMs = millis();

  // The on-board `BUTTON` is already configured by Device OS; don't reconfigure it here.
  // Initialize button state tracking from the actual pin state.
  lastBtnState = System.buttonPushed() > 0 ? LOW : HIGH; // active-low button
  btnState = lastBtnState;
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  unsigned long now = millis();

  if (pulseOn && (now - pulsePhaseStartMs >= pulseOnMs)) {
    pulseOn = false;
    pulsePhaseStartMs = now;
    digitalWrite(pulsePin, LOW);
    Log.info("D6 LOW for 20 minutes");
  } else if (!pulseOn && (now - pulsePhaseStartMs >= pulseOffMs)) {
    pulseOn = true;
    pulsePhaseStartMs = now;
    digitalWrite(pulsePin, HIGH);
    Log.info("D6 HIGH for 5 seconds (periodic)");
  }

  int reading = System.buttonPushed() > 0 ? LOW : HIGH; // active-low button

  // If the button reading changed, reset the debounce timer
  if (reading != lastBtnState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the button state has changed (and is stable), take action
    if (reading != btnState) {
      btnState = reading;

      // Button is active-low (INPUT_PULLUP): press -> LOW
      if (btnState == LOW) {
        pulseOn = true;
        pulsePhaseStartMs = now;
        digitalWrite(pulsePin, HIGH);
        Log.info("D6 HIGH for 5 seconds (button)");
      }
    }
  }

  lastBtnState = reading;
}
#endif
