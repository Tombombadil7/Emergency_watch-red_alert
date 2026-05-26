// =============================================================================
// Emergency Alert Watch — ESP32-S3 + SIM7080G
// WiFi REMOVED. All network I/O goes through the cellular modem via AT commands.
// =============================================================================
//task to complete: add ssl for updates ,create animation + vibration pattern and button shutoff for emergancy alert, battary low event.
// think and change gps failed event, does it need to trriger alert??
#include <Arduino.h>
#include <esp_sleep.h>
#include "LittleFS.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/ecp.h"
#include "mbedtls/md.h"
#include <TFT_eSPI.h>
// -----------------------------------------------------------------------------
// Pin Definitions
// -----------------------------------------------------------------------------
#define MODEM_WAKEUP_PIN    12   // SIM7080G RI (Ring Indicator) pin → EXT1 wakeup
#define BATTERY_WAKEUP_PIN  14   // Battery gauge alert pin          → EXT1 wakeup
#define MODEM_RX_PIN        16   // ESP32 RX ← SIM7080G TX
#define MODEM_TX_PIN        17   // ESP32 TX → SIM7080G RX
#define MODEM_PWRKEY_PIN    4    // SIM7080G PWRKEY — pulse LOW 1.5s to toggle power
#define BACKLIGHT_PIN 5
//TFT library defied pins (not in code, just here for documentation): 
//#define TFT_MOSI 23
//#define TFT_SCLK 18
//#define TFT_CS   15  // Chip select control pin
//#define TFT_DC    2  // Data Command control pin
//#define TFT_RST   4 
// -----------------------------------------------------------------------------
// Modem Timing Constants
// -----------------------------------------------------------------------------
#define MODEM_BAUD          115200
#define AT_SHORT_DELAY      150    // ms — for simple AT commands
#define AT_MEDIUM_DELAY     3000   // ms — for network operations
#define AT_LONG_DELAY       8000   // ms — for HTTP connections
#define SMS_READ_TIMEOUT    5000   // ms — window to read SMS from modem UART
#define GPS_FIX_TIMEOUT     6000   // ms — A-GPS assisted fix should be <1-2s

// -----------------------------------------------------------------------------
// Server / APN Configuration  ← adjust to your SIM plan
// -----------------------------------------------------------------------------
#define MODEM_APN           "iot.1nce.net"  // Replace with your SIM's APN

// -----------------------------------------------------------------------------
// ECDSA Public Key (33 bytes, compressed P-256 point)
// Replace the placeholder bytes with your actual server-generated public key.
// -----------------------------------------------------------------------------
const uint8_t PUBLIC_KEY[] = { 0x03, 0x5a, /* ... replace with real key bytes ... */ };
const size_t  PUBLIC_KEY_LEN = sizeof(PUBLIC_KEY);

// -----------------------------------------------------------------------------
// Hardware Serial — UART1 dedicated to modem
// -----------------------------------------------------------------------------
HardwareSerial SerialModem(1);
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite canvas = TFT_eSprite(&tft);
// -----------------------------------------------------------------------------
// Geo Data Structures
// -----------------------------------------------------------------------------
struct Point {
    float lat;
    float lon;
};

struct PolygonData {
    uint32_t id;
    String   name;
    uint16_t num_points;
    Point*   points;
};

struct GeoObject {
    uint32_t     id;
    String       label;
    uint8_t      num_polygons;
    PolygonData* polygons;
};

// 12-byte packed index record for fast binary search in geoindex.bin
struct __attribute__((__packed__)) IndexRecord {
    uint32_t id;
    uint32_t offset;
    uint32_t length;
};



String   sendAT(const String& cmd, uint32_t timeoutMs = 1000);
bool     waitForResponse(const String& target, uint32_t timeoutMs);
void     modemPowerOn();

String   decodeBase85(const String& input);
String   extractSmsBody(const String& rawAtResponse);
bool     verify_emergency(const uint8_t* data, size_t data_len, const uint8_t* raw_sig);

bool     modemHttpGetToFile(const String& url, const String& savePath);
bool     activatePDP();
void     deactivatePDP();
void     triggerGeoUpdate(const String& indexUrl, const String& dataUrl);

bool     getGPSFix(float& lat, float& lon, uint32_t timeoutMs = GPS_FIX_TIMEOUT);
void     triggerEmergencyHardware();
void     downloadAndSaveFile(const String& savePath, uint32_t expectedBytes);


// =============================================================================
//  SETUP  —  This is the ONLY entry point after every deep-sleep wakeup
// =============================================================================
void setup() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    // 1. KEEP THE SCREEN OFF IMMEDIATELY
    pinMode(BACKLIGHT_PIN, OUTPUT);
    digitalWrite(BACKLIGHT_PIN, LOW); 

  // 2. Initialize the screen invisibly
    tft.init();
    canvas.createSprite(240, 240);
    canvas.setTextDatum(MC_DATUM);
    // -------------------------------------------------------------------------
    // EMERGENCY FAST-TRACK: woken by an external pin (modem RI or battery alert)
    // -------------------------------------------------------------------------
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();

        if (wakeup_pin_mask & (1ULL << MODEM_WAKEUP_PIN)) {
            handleModemWakeup();   // Read SMS → verify → geo-check → alert
        } else if (wakeup_pin_mask & (1ULL << BATTERY_WAKEUP_PIN)) {
            handleBatteryWakeup(); // Low-battery tactile warning
        }
    }

    // -------------------------------------------------------------------------
    // COLD BOOT: first power-on after battery insertion
    // -------------------------------------------------------------------------
    else if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
        Serial.begin(115200);
        delay(200);
        Serial.println("[BOOT] Cold boot — running diagnostics and modem setup.");
        runOneTimeDiagnostics();
        configureModemOnFirstBoot();

    }

    // -------------------------------------------------------------------------
    // SLEEP SETUP: arm both wakeup pins, prepare screen for sleep and enter deep sleep
    // -------------------------------------------------------------------------
    uint64_t pin_mask = (1ULL << MODEM_WAKEUP_PIN) | (1ULL << BATTERY_WAKEUP_PIN);
    digitalWrite(BACKLIGHT_PIN, LOW); // Turn off backlight
    tft.writecommand(0x10); // Send GC9A01 to low-power Sleep Mode
    esp_sleep_enable_ext1_wakeup(pin_mask, ESP_EXT1_WAKEUP_ALL_LOW);
    esp_deep_sleep_start();
}

// loop() is never reached — device always deep-sleeps from setup()
void loop() {}


// =============================================================================
//  COLD BOOT — One-time modem initialisation
// =============================================================================
void configureModemOnFirstBoot() {
    Serial.println("[MODEM] Initialising SIM7080G...");

    SerialModem.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
    modemPowerOn();   // Pulse PWRKEY if modem is off

    sendAT("AT",           1500);  // Echo test — confirms UART link
    sendAT("ATE0",         500);   // Disable echo (critical for clean AT parsing)
    sendAT("AT+CMEE=2",    500);   // Verbose error codes (easier debugging)

    // --- SIM & Network ---
    String simStatus = sendAT("AT+CPIN?", 3000);
    if (simStatus.indexOf("READY") == -1) {
        Serial.println("[ERROR] SIM not ready — check SIM tray.");
    }
    sendAT("AT+CMNB=3",            1000);  // LTE-M + NB-IoT (auto-select)
    sendAT("AT+CNMP=38",           1000);  // LTE only (no 2G fallback — saves power)
    sendAT("AT+CNCFGAPN=\"" + String(MODEM_APN) + "\"", 1000); // APN

    // Wait for network registration (up to 30s)
    bool registered = false;
    for (int i = 0; i < 30; i++) {
        String reg = sendAT("AT+CEREG?", 1000);
        if (reg.indexOf(",1") != -1 || reg.indexOf(",5") != -1) {
            registered = true;
            Serial.println("[MODEM] Registered on network.");
            break;
        }
        delay(1000);
    }
    if (!registered) Serial.println("[WARN] Network registration timeout.");

    // --- SMS Configuration ---
    sendAT("AT+CMGF=1",            500);   // Text mode SMS (works with Base85 ASCII payload)
    sendAT("AT+CNMI=2,1,0,0,0",    500);   // New SMS triggers RI pin pulse immediately
    sendAT("AT+CMGD=1,4",          1000);  // Clear any leftover SMS from flash

    // --- Cell Broadcast (Path 1 — passive CMAS/EU-Alert listener) ---
    // Channel 4370 = Israel Home Front Command (Pikud HaOref)
    // Channels 4352-4354 = CMAS Presidential / Extreme / Severe
    sendAT("AT+CSCB=0,\"4352,4353,4354,4370\"", 1000);

    // --- Ring Indicator pin behaviour ---
    // Ensures RI goes LOW on new SMS (wakes ESP32 via EXT1)
    sendAT("AT+CSCLK=1",           500);   // Enable slow-clock / RI wakeup mode

    // --- LittleFS ---
    if (!LittleFS.begin(true)) {
        Serial.println("[ERROR] LittleFS mount failed.");
    } else {
        Serial.println("[FS] LittleFS ready.");
    }

    Serial.println("[BOOT] Setup complete. Entering deep sleep.");
}


// =============================================================================
//  MODEM WAKEUP — Called when SIM7080G RI pin fires (new SMS received)
// =============================================================================
void handleModemWakeup() {
    SerialModem.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
    delay(AT_SHORT_DELAY);

    // --- 1. Read unread SMS ---
    sendAT("AT+CMGF=1", 300);
    sendAT("AT+CMGL=\"REC UNREAD\"", 200);  // Prime the command

    String rawSmsResponse = "";
    uint32_t deadline = millis() + SMS_READ_TIMEOUT;
    while (millis() < deadline) {
        while (SerialModem.available()) {
            rawSmsResponse += (char)SerialModem.read();
        }
    }

    // --- 2. Extract Base85 payload from AT response ---
    String base85Payload = extractSmsBody(rawSmsResponse);
    if (base85Payload.length() == 0) {
        // No valid SMS body — delete junk and sleep
        sendAT("AT+CMGD=1,4", 500);
        return;
    }

    // --- 3. Decode Base85 → raw binary string ---
    String decodedSms = decodeBase85(base85Payload);
    size_t decodedSmsLen = decodedSms.length();

    // Minimum valid message: 3 bytes prefix + 64 bytes ECDSA signature = 67 bytes
    if (decodedSmsLen < 67) {
        sendAT("AT+CMGD=1,4", 500);
        return;
    }

    // --- 4. Extract prefix and payload sections ---
    // Layout: [0..2] = prefix (EMG / GEO / OTA)
    //         [3..66] = 64-byte ECDSA signature (raw r||s)
    //         [67..]  = action data
    String prefix     = decodedSms.substring(0, 3);
    String actionData = decodedSms.substring(67);   // data after signature

    const uint8_t* msgBytes = (const uint8_t*)decodedSms.c_str();
    const uint8_t* sigBytes  = msgBytes + 3;   // 64-byte signature at offset 3
    const uint8_t* dataBytes = msgBytes + 67;  // actual payload after signature
    size_t dataLen = decodedSmsLen - 67;

    // --- 5. Verify ECDSA P-256 signature before acting ---
    if (!verify_emergency(dataBytes, dataLen, sigBytes)) {
        // Signature mismatch — discard silently (could be spoofed)
        sendAT("AT+CMGD=1,4", 500);
        return;
    }

    // --- 6. Route to correct action based on prefix ---
    if (prefix == "EMG") {
        // Emergency alert — get GPS fix, check polygon, vibrate if inside zone
        float userLat = 0.0f, userLon = 0.0f;
        bool hasFix = getGPSFix(userLat, userLon, GPS_FIX_TIMEOUT);
        if (!hasFix) {
            // GPS failed → fail-safe: alert anyway (safety > false positives)
            triggerEmergencyHardware();
        } else {
            // TODO: load polygon from LittleFS and run point-in-polygon check
            // For now, trigger alert unconditionally until geo-check is implemented
            triggerEmergencyHardware();
        }

    } else if (prefix == "GEO") {
        // Geo-file update — actionData contains index URL and data URL, pipe-separated
        // Expected format: "https://example.com/geoindex.bin|https://example.com/geodata.bin"
        int sep = actionData.indexOf('|');
        if (sep != -1) {
            String indexUrl = actionData.substring(0, sep);
            String dataUrl  = actionData.substring(sep + 1);
            triggerGeoUpdate(indexUrl, dataUrl);
        }

    } else if (prefix == "OTA") {
        // OTA firmware update — actionData contains the binary URL
        // performOTAUpdate(actionData);  // ← implement in next sprint
    }

    // --- 7. Delete all SMS from modem flash (keep it clean) ---
    sendAT("AT+CMGD=1,4", 500);
}


// =============================================================================
//  GEO UPDATE — Download geoindex.bin + geodata.bin over cellular modem
// =============================================================================
void triggerGeoUpdate(const String& indexUrl, const String& dataUrl) {
    if (!activatePDP()) return;  // Bring up data bearer

    Serial.println("[GEO] Downloading geoindex.bin...");
    modemHttpGetToFile(indexUrl, "/geoindex.bin");

    Serial.println("[GEO] Downloading geodata.bin...");
    modemHttpGetToFile(dataUrl, "/geodata.bin");

    deactivatePDP();  // Release data bearer — critical for power saving
}


// =============================================================================
//  MODEM HTTP GET → LittleFS
//  Downloads a file at 'url' and saves it to LittleFS at 'savePath'.
//  Uses SIM7080G AT+SHTTPCREATE/CON/REQ/READ/DIS command set.
//  Returns true on success.
// =============================================================================
bool modemHttpGetToFile(const String& url, const String& savePath) {
    // Parse URL: strip "https://" and split host from path
    String workUrl = url;
    bool   useHttps = workUrl.startsWith("https://");
    workUrl.replace("https://", "");
    workUrl.replace("http://",  "");

    int slashIdx = workUrl.indexOf('/');
    String host    = (slashIdx != -1) ? workUrl.substring(0, slashIdx) : workUrl;
    String reqPath = (slashIdx != -1) ? workUrl.substring(slashIdx)    : "/";

    // --- 1. Create HTTP(S) session ---
    String createCmd = useHttps ? "AT+SHTTPCREATE=\"https\"" : "AT+SHTTPCREATE=\"http\"";
    String createResp = sendAT(createCmd, 2000);
    // Response: +SHTTPCREATE: <session_id>
    int sessionId = 0;
    int idx = createResp.indexOf("+SHTTPCREATE:");
    if (idx != -1) {
        sessionId = createResp.substring(idx + 13).toInt();
    }

    // --- 2. Set host ---
    sendAT("AT+SHTTPCFG=" + String(sessionId) + ",\"" + host + "\",443", 1000);

    // --- 3. Connect ---
    String conResp = sendAT("AT+SHTTPCON=" + String(sessionId), AT_LONG_DELAY);
    if (conResp.indexOf("OK") == -1) {
        Serial.println("[HTTP] Connection failed.");
        return false;
    }

    // --- 4. Send GET request ---
    String reqCmd = "AT+SHTTPREQ=" + String(sessionId) + ",\"GET\",\"" + reqPath + "\"";
    String reqResp = sendAT(reqCmd, AT_LONG_DELAY);
    // Response includes +SHTTPREQ: <session>,<status_code>,<data_len>
    int dataLen = 0;
    int rIdx = reqResp.indexOf("+SHTTPREQ:");
    if (rIdx != -1) {
        // Extract data length (3rd comma-separated field)
        String fields = reqResp.substring(rIdx + 10);
        int c1 = fields.indexOf(',');
        int c2 = (c1 != -1) ? fields.indexOf(',', c1 + 1) : -1;
        if (c2 != -1) {
            dataLen = fields.substring(c2 + 1).toInt();
        }
    }

    if (dataLen <= 0) {
        Serial.println("[HTTP] No data in response.");
        sendAT("AT+SHTTPDIS=" + String(sessionId), 1000);
        return false;
    }

    Serial.printf("[HTTP] Receiving %d bytes → %s\n", dataLen, savePath.c_str());

    // --- 5. Open LittleFS file for writing ---
    if (!LittleFS.begin(false)) {
        Serial.println("[FS] LittleFS not mounted.");
        sendAT("AT+SHTTPDIS=" + String(sessionId), 1000);
        return false;
    }
    File f = LittleFS.open(savePath, FILE_WRITE);
    if (!f) {
        Serial.println("[FS] Could not open file for writing.");
        sendAT("AT+SHTTPDIS=" + String(sessionId), 1000);
        return false;
    }

    // --- 6. Read response body in 1024-byte chunks ---
    const int CHUNK = 1024;
    int bytesRead = 0;
    while (bytesRead < dataLen) {
        int toRead = min(CHUNK, dataLen - bytesRead);
        String readCmd = "AT+SHTTPREAD=" + String(sessionId) + "," +
                         String(bytesRead) + "," + String(toRead);
        SerialModem.println(readCmd);

        // Collect raw bytes from UART (skip the "+SHTTPREAD:" header line)
        String chunk = "";
        uint32_t dl = millis() + 3000;
        bool headerPassed = false;
        while (millis() < dl) {
            while (SerialModem.available()) {
                char c = (char)SerialModem.read();
                if (!headerPassed) {
                    chunk += c;
                    if (chunk.endsWith("\n") && chunk.indexOf("+SHTTPREAD:") != -1) {
                        headerPassed = true;
                        chunk = "";  // Discard header, start collecting data
                    }
                } else {
                    f.write((uint8_t)c);
                    bytesRead++;
                    if (bytesRead >= dataLen) break;
                }
            }
            if (bytesRead >= dataLen) break;
        }
    }

    f.close();
    Serial.printf("[HTTP] Saved %d bytes to %s\n", bytesRead, savePath.c_str());

    // --- 7. Disconnect session ---
    sendAT("AT+SHTTPDIS=" + String(sessionId), 1000);

    return (bytesRead >= dataLen);
}


// =============================================================================
//  PDP CONTEXT — Activate / Deactivate cellular data bearer
// =============================================================================
bool activatePDP() {
    String resp = sendAT("AT+CNACT=0,1", AT_MEDIUM_DELAY);
    if (resp.indexOf("OK") == -1 && resp.indexOf("+APP PDP: 0,ACTIVE") == -1) {
        // Try to check if already active
        String check = sendAT("AT+CNACT?", 1000);
        if (check.indexOf("ACTIVE") != -1) return true;
        Serial.println("[PDP] Failed to activate data bearer.");
        return false;
    }
    delay(1000);  // Give bearer a moment to stabilise
    return true;
}

void deactivatePDP() {
    sendAT("AT+CNACT=0,0", AT_MEDIUM_DELAY);  // Deactivate bearer — saves power
}


// =============================================================================
//  GPS FIX — Use SIM7080G embedded GNSS engine for A-GPS fix
//  With A-GPS and a clear view, fix should arrive in < 2 seconds.
//  lat and lon are filled on success. Returns false on timeout.
// =============================================================================
bool getGPSFix(float& lat, float& lon, uint32_t timeoutMs) {
    sendAT("AT+CGNSSPWR=1", 1000);   // Power on GNSS engine
    delay(500);

    bool fixFound = false;
    uint32_t deadline = millis() + timeoutMs;

    while (millis() < deadline) {
        String info = sendAT("AT+CGNSSINFO", 1000);
        // Response: +CGNSSINFO: <mode>,<GPS satellites used>,<GNSS satellites used>,
        //           <GLONASS satellites used>,<date>,<UTC time>,<lat>,<lat dir>,
        //           <lon>,<lon dir>,<alt>,<speed>,<course>,<PDOP>,<HDOP>,<VDOP>
        int infoIdx = info.indexOf("+CGNSSINFO:");
        if (infoIdx != -1) {
            // A fix is valid when field 6 (lat) is non-empty
            String fields = info.substring(infoIdx + 12);
            // Split CSV
            String parts[16];
            int partIdx = 0;
            int start = 0;
            for (int i = 0; i < (int)fields.length() && partIdx < 16; i++) {
                if (fields[i] == ',') {
                    parts[partIdx++] = fields.substring(start, i);
                    start = i + 1;
                }
            }
            // parts[6] = latitude, parts[8] = longitude (if non-empty, fix acquired)
            if (parts[6].length() > 0 && parts[6] != "0") {
                lat = parts[6].toFloat();
                lon = parts[8].toFloat();
                // Handle Southern / Western hemispheres
                if (parts[7] == "S") lat = -lat;
                if (parts[9] == "W") lon = -lon;
                fixFound = true;
                break;
            }
        }
        delay(200);
    }

    sendAT("AT+CGNSSPWR=0", 500);  // Power off GNSS immediately — saves ~30mA
    return fixFound;
}


// =============================================================================
//  EMERGENCY HARDWARE TRIGGER — Vibration motor + any future alert output
// =============================================================================
void triggerEmergencyHardware() {
    // TODO: drive MOSFET gate pin controlling the LRA/coin vibration motor
    // e.g.: digitalWrite(VIBRATION_MOSFET_PIN, HIGH); delay(500); ...
    // Placeholder:
    Serial.println("[ALERT] EMERGENCY TRIGGERED — vibration motor should fire here.");
}


// =============================================================================
//  BATTERY WAKEUP — Low battery alert via tactile vibration
// =============================================================================
void handleBatteryWakeup() {
    // TODO: read battery gauge over I2C (e.g. MAX17048) to confirm level
    // For now, short vibration pulse to warn user
    Serial.println("[BATTERY] Low battery warning.");
    triggerEmergencyHardware();  // Reuse vibration — distinguish by pattern later
}


// =============================================================================
//  DIAGNOSTICS — Cold boot self-test
// =============================================================================
void runOneTimeDiagnostics() {
    Serial.println("[DIAG] Running self-test...");

    // LittleFS
    if (LittleFS.begin(true)) {
        Serial.printf("[DIAG] LittleFS OK — total: %u bytes, used: %u bytes\n",
                      LittleFS.totalBytes(), LittleFS.usedBytes());
    } else {
        Serial.println("[DIAG] LittleFS FAIL");
    }

    // Modem UART echo test
    SerialModem.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
    modemPowerOn();
    String atResp = sendAT("AT", 1500);
    if (atResp.indexOf("OK") != -1) {
        Serial.println("[DIAG] Modem UART OK");
    } else {
        Serial.println("[DIAG] Modem UART — no response. Check wiring/power.");
    }

    // Report chip info
    Serial.printf("[DIAG] ESP32 chip: %s, cores: %d, freq: %dMHz\n",
                  ESP.getChipModel(), ESP.getChipCores(), ESP.getCpuFreqMHz());
    Serial.printf("[DIAG] Free heap: %u bytes\n", ESP.getFreeHeap());
}


// =============================================================================
//  MODEM POWER ON — Pulse PWRKEY to wake SIM7080G if it is off
// =============================================================================
void modemPowerOn() {
    // Check if modem already responds
    String resp = sendAT("AT", 800);
    if (resp.indexOf("OK") != -1) return;  // Already on

    // Pulse PWRKEY LOW for 1.5s to toggle power on
    pinMode(MODEM_PWRKEY_PIN, OUTPUT);
    digitalWrite(MODEM_PWRKEY_PIN, LOW);
    delay(1500);
    digitalWrite(MODEM_PWRKEY_PIN, HIGH);
    delay(3000);  // Wait for modem to boot and print its startup message
    Serial.println("[MODEM] Power-on pulse sent.");
}


// =============================================================================
//  AT COMMAND HELPER — Send command, collect response until timeout
// =============================================================================
String sendAT(const String& cmd, uint32_t timeoutMs) {
    while (SerialModem.available()) SerialModem.read();  // Flush RX buffer
    SerialModem.println(cmd);

    String response = "";
    uint32_t deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        while (SerialModem.available()) {
            response += (char)SerialModem.read();
        }
        // Early exit on final response codes
        if (response.indexOf("OK\r")    != -1) break;
        if (response.indexOf("ERROR")   != -1) break;
        if (response.indexOf("+CME")    != -1) break;
    }
    return response;
}


// =============================================================================
//  BASE85 DECODER  (ASCII85 / RFC 1924 variant)
// =============================================================================
String decodeBase85(const String& input) {
    String   output = "";
    uint32_t tuple  = 0;
    int      count  = 0;

    for (size_t i = 0; i < input.length(); i++) {
        char c = input[i];

        if (c >= '!' && c <= 'u') {
            tuple = tuple * 85 + (c - '!');
            count++;
            if (count == 5) {
                output += (char)(tuple >> 24);
                output += (char)(tuple >> 16);
                output += (char)(tuple >> 8);
                output += (char)(tuple);
                tuple = 0;
                count = 0;
            }
        } else if (c == 'z') {
            // 'z' shorthand for four zero bytes
            output += (char)0; output += (char)0;
            output += (char)0; output += (char)0;
        }
        // Skip whitespace and other non-data characters silently
    }

    // Handle remaining partial group
    if (count > 0) {
        for (int i = count; i < 5; i++) tuple = tuple * 85 + 84;
        for (int i = 0; i < count - 1; i++) {
            output += (char)(tuple >> (24 - (i * 8)));
        }
    }
    return output;
}


// =============================================================================
//  SMS BODY EXTRACTOR  — Parses raw AT+CMGL response, returns Base85 body
// =============================================================================
String extractSmsBody(const String& rawAtResponse) {
    int headerIndex = rawAtResponse.indexOf("+CMGL:");
    if (headerIndex == -1) return "";

    // Skip past the +CMGL: header line (ends at the \n after the timestamp field)
    int bodyStartIndex = rawAtResponse.indexOf('\n', headerIndex);
    if (bodyStartIndex == -1) return "";
    bodyStartIndex++;  // Character after the \n

    // Find end of the message body (\r or \n, whichever comes first)
    int bodyEndIndex = rawAtResponse.indexOf('\r', bodyStartIndex);
    if (bodyEndIndex == -1) {
        bodyEndIndex = rawAtResponse.indexOf('\n', bodyStartIndex);
    }

    String payload = (bodyEndIndex != -1)
        ? rawAtResponse.substring(bodyStartIndex, bodyEndIndex)
        : rawAtResponse.substring(bodyStartIndex);

    payload.trim();
    return payload;
}


// =============================================================================
//  ECDSA P-256 SIGNATURE VERIFICATION  (mbedTLS hardware-accelerated on ESP32)
//  data     — the message bytes to verify (payload after the signature)
//  data_len — length of data
//  raw_sig  — 64 raw bytes: first 32 = r, last 32 = s
// =============================================================================
bool verify_emergency(const uint8_t* data, size_t data_len, const uint8_t* raw_sig) {
    mbedtls_ecp_group    group;
    mbedtls_ecp_point    Q;
    mbedtls_mpi          r, s;

    mbedtls_ecp_group_init(&group);
    mbedtls_ecp_point_init(&Q);
    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);

    int ret = mbedtls_ecp_group_load(&group, MBEDTLS_ECP_DP_SECP256R1);
    if (ret == 0) ret = mbedtls_ecp_point_read_binary(&group, &Q, PUBLIC_KEY, PUBLIC_KEY_LEN);
    if (ret == 0) ret = mbedtls_mpi_read_binary(&r, raw_sig,      32);
    if (ret == 0) ret = mbedtls_mpi_read_binary(&s, raw_sig + 32, 32);

    if (ret != 0) goto cleanup;

    {
        // SHA-256 computed in hardware on ESP32 via mbedTLS
        uint8_t              hash[32];
        mbedtls_md_context_t md_ctx;
        mbedtls_md_init(&md_ctx);
        mbedtls_md_setup(&md_ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 0);
        mbedtls_md_starts(&md_ctx);
        mbedtls_md_update(&md_ctx, data, data_len);
        mbedtls_md_finish(&md_ctx, hash);
        mbedtls_md_free(&md_ctx);

        ret = mbedtls_ecdsa_verify(&group, hash, sizeof(hash), &Q, &r, &s);
    }

cleanup:
    mbedtls_ecp_group_free(&group);
    mbedtls_ecp_point_free(&Q);
    mbedtls_mpi_free(&r);
    mbedtls_mpi_free(&s);

    return (ret == 0);
}




void drawEventOne() {
  canvas.fillSprite(TFT_BLACK);
  canvas.setTextColor(TFT_GREEN);
  canvas.drawString("TIMER WAKEUP", 120, 120, 4);
  canvas.pushSprite(0, 0);
}
