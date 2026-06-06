#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <atomic>
#include <Arduino.h>
#include <esp_sleep.h>
#include "LittleFS.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/ecp.h"
#include "mbedtls/md.h"
#include <TFT_eSPI.h>

// --- ממשקי FreeRTOS ---
QueueHandle_t emergencyQueue;
QueueHandle_t backgroundQueue;
SemaphoreHandle_t uartMutex;           // שים לב: זה יהיה מנעול רקורסיבי
SemaphoreHandle_t modemInterruptSem;   // סמפור להערת משימת ההאזנה כשהמודם מאותת

std::atomic<bool> isCriticalSection(false); // דגל שמסמן שאין לקטוע פעולה רגישה
std::atomic<bool> keepAwake(true);          // דגל שיקבע מתי אפשר לחזור ל-Deep Sleep
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite canvas = TFT_eSprite(&tft);
std::atomic<bool> smsPendingInterrupt{false};
HardwareSerial SerialModem(1); // UART1

// =============================================================================
//  all global strctures and definitions
// =============================================================================

// -----------------------------------------------------------------------------
// Physical Pin Definitions
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
#define TFT_WIDTH 240   // GC9A01A width
#define TFT_HEIGHT 240  // GC9A01A height
#define TFT_ROTATION 0
#define MODEM_APN           "iot.1nce.net"  // Replace with your SIM's APN
// Message prefix constants
#define MSG_EMERGENCY   '1'
#define MSG_PRE_EMERGENCY '2'
#define MSG_GEO_UPDATE  '3'
#define MSG_OTA_UPDATE  '4'


// -----------------------------------------------------------------------------
// Global Data Structures
// -----------------------------------------------------------------------------


// 12-byte packed index record for fast binary search in geoindex.bin
struct __attribute__((__packed__)) IndexRecord {
    uint32_t id;
    uint32_t offset;
    uint32_t length;
};

// DownloadResult enum
enum class DownloadResult {
    OK,
    INTERRUPTED,
    ERROR
};


// =============================================================================
//  MESSAGE PAYLOAD STRUCTS
//  Rules:
//  - Every struct must be __attribute__((packed))
//  - Every struct must be ≤ 75 bytes (SMS budget: 140 - 1 prefix - 64 sig)
//  - Add new struct here, then add its prefix constant and a case in MainLogicTask
// =============================================================================


// Type "1"/"2" — Emergency alert
// Layout: [id_count (1 byte)] [id_0 (2 bytes)] [id_1 (2 bytes)] ... [id_N (2 bytes)]
// Max 37 IDs (1 + 37*2 = 75 bytes)
// areaid values from Pikud HaOref are small integers — uint16_t covers all current
// and likely future values safely.
struct __attribute__((packed)) EmergencyPayload {
    uint8_t  id_count;
    uint16_t ids[37];

    // How many bytes of this struct are actually used (for validation)
    size_t usedSize() const { return sizeof(id_count) + id_count * sizeof(uint16_t); }
};

// Type "3" — Geo file update or Type "4" — OTA update (same format, different handling)
// Layout: two null-terminated strings packed back to back
// e.g. "https://raw.githubusercontent.com/.../index.bin\0https://.../data.bin\0"
struct __attribute__((packed)) UrlPayload {
    char url[75];
};



// =============================================================================
//  MASTER PACKET — do not modify, just add structs above and cases in MainLogicTask
// =============================================================================
struct __attribute__((packed)) MessagePacket {
    uint8_t prefix;
    union {
        EmergencyPayload emergency;   // prefix '1'
        GeoPayload       geo;         // prefix '3'
        // NewTypePayload newtype;    // ← add future types here
        uint8_t raw[75];              // hard ceiling — union is always exactly 75 bytes
    } payload;
};

static_assert(sizeof(MessagePacket) == 76, "MessagePacket exceeds SMS budget!");



const uint8_t PUBLIC_KEY[] = { 0x03, 0x5a, /* ... replace with real key bytes ... */ };
const size_t  PUBLIC_KEY_LEN = sizeof(PUBLIC_KEY);
String sendAT(const String& cmd, uint32_t timeoutMs = 1000);
void     modemPowerOn();
String   extractSmsBody(const String& rawAtResponse);
bool     verify_emergency(const uint8_t* data, size_t data_len, const uint8_t* raw_sig);
DownloadResult modemHttpGetToFile(const String& url, const String& savePath);
bool     activatePDP();
void     deactivatePDP();
void     triggerGeoUpdate(const String& baseUrl);
void     runOneTimeDiagnostics();
void     configureModemOnFirstBoot();
bool     getGPSFix(float& lat, float& lon, uint32_t timeoutMs = GPS_FIX_TIMEOUT);
void     downloadAndSaveFile(const String& savePath, uint32_t expectedBytes);


// פונקציית פסיקת חומרה (ISR) - מופעלת כשהמכשיר *ער* ופין ה-RI יורד ל-LOW
void IRAM_ATTR modemRiInterrupt() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(modemInterruptSem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void setup() {
    // 1. שמירה על מסך כבוי מיד בהתחלה
    pinMode(BACKLIGHT_PIN, OUTPUT);
    digitalWrite(BACKLIGHT_PIN, LOW); 
    tft.init();
    canvas.createSprite(240, 240);
    canvas.setTextDatum(MC_DATUM);

    Serial.begin(115200);

    // 2. יצירת אובייקטי FreeRTOS
    emergencyQueue = xQueueCreate(5, sizeof(MessagePacket));
    backgroundQueue = xQueueCreate(5, sizeof(MessagePacket)); 
    uartMutex = xSemaphoreCreateRecursiveMutex(); 
    modemInterruptSem = xSemaphoreCreateBinary();

    // 3. הקצאת משימות לליבות שונות
    // משימת ההאזנה (עדיפות גבוהה) על ליבה 0
    xTaskCreatePinnedToCore(SmsListenerTask, "SMS_Task", 8192, NULL, 3, NULL, 0);
    // המשימה הראשית (לוגיקה) על ליבה 1
    xTaskCreatePinnedToCore(MainLogicTask, "Main_Task", 16384, NULL, 1, NULL, 1);

    // ה-Scheduler מתחיל לעבוד אוטומטית, פונקציית setup מסתיימת.
}

void loop() {
    vTaskDelete(NULL); // ✅ נכון - אבל עדיף גם להוסיף לפניו:
    vTaskSuspend(NULL); // כ-fallback במידה ו-FreeRTOS לא ימחק מיד
}
// =============================================================================
//  Dual-task definitions: MainLogicTask + SmsListenerTask
// =============================================================================

void MainLogicTask(void *pvParameters) {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    // --- א. טיפול בהתעוררות משינה עמוקה ---
    if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
        // Cold Boot
        Serial.println("[BOOT] Cold boot...");
        runOneTimeDiagnostics();
        configureModemOnFirstBoot();
    } 
    else if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
        
        if (wakeup_pin_mask & (1ULL << MODEM_WAKEUP_PIN)) {
            // התעוררנו בגלל SMS! הפעימה של ה-RI כבר קרתה, ולכן הפסיקה לא תופעל.
            // נשחרר ידנית את הסמפור כדי שמשימת ההאזנה תדע שיש הודעה שמחכה לה.
            xSemaphoreGive(modemInterruptSem);
        } 
        else if (wakeup_pin_mask & (1ULL << BATTERY_WAKEUP_PIN)) {
            handleBatteryWakeup(); 
        }
    }

    // הגדרת פסיקה חיה: אם נקבל SMS מעכשיו והלאה כשאנחנו ערים
    pinMode(MODEM_WAKEUP_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MODEM_WAKEUP_PIN), modemRiInterrupt, FALLING);

    // --- ב. לולאת זמן ריצה (Runtime Loop) ---
    uint32_t idleStartTime = millis();

    while (keepAwake) {
        MessagePacket receivedMsg;
        receivedMsg.prefix = (uint8_t)basePayload[0];
        
        const uint8_t* dataBytes = (const uint8_t*)basePayload.c_str() + DATA_OFFSET;
        size_t dataLen = basePayload.length() - DATA_OFFSET;
        
        if (xQueueReceive(emergencyQueue, &receivedMsg, 0) == pdPASS) {
            switch (receivedMsg.prefix) {
                case MSG_EMERGENCY: {
                    EmergencyPayload& e = receivedMsg.payload.emergency;
                    triggerEmergencyalert(e.ids[],receivedMsg.prefix);
                    // e.id_count and e.ids[] available for use
                    break;
                }
            }
            idleStartTime = millis();
        }
        else if (!isCriticalSection) {
            if (xQueueReceive(backgroundQueue, &receivedMsg, 0) == pdPASS) {
                switch (receivedMsg.prefix) {
                    case MSG_GEO_UPDATE: {
                        UrlPayload& u = receivedMsg.payload.url;
                        triggerGeoUpdate(String(u.url));
                        break;
                    }
                    case MSG_OTA_UPDATE: {
                        UrlPayload& u = receivedMsg.payload.url;
                        triggerOtaUpdate(String(u.url));
                        break;
                    }
                }
                idleStartTime = millis();
            }
        }

    }

        // אם עבר זמן מסוים ללא פעילות - אפשר לחזור לישון
        if (millis() - idleStartTime > 15000) { 
            keepAwake = false; // שובר את הלולאה ומוביל להרדמת המכשיר
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // --- ג. חזרה ל-Deep Sleep ---
    detachInterrupt(digitalPinToInterrupt(MODEM_WAKEUP_PIN));
    prepareForSleep(); // פונקציה שמרכזת את כיבוי המסך (0x10), הגדרת פינים ל-Wakeup, והפעלת esp_deep_sleep_start()
}

void SmsListenerTask(void *pvParameters) {
    while (true) {
        // המשימה ממתינה פה (ללא צריכת מעבד) עד שהסמפור משוחרר ע"י הפסיקה או ה-Main
        if (xSemaphoreTake(modemInterruptSem, portMAX_DELAY) == pdTRUE) {
            
            // מחכים לקבל בלעדיות על קו ה-UART מול המודם
            if (xSemaphoreTakeRecursive(uartMutex, portMAX_DELAY) == pdTRUE) {
                
                // *** חובה לאפס את דגל הביטול ברגע שקיבלנו שליטה, כדי שפעולות הבאות לא יתבטלו! ***
                smsPendingInterrupt = false; 
                
                // קריאת ה-SMS
                sendAT("AT+CMGF=1", 300);
                String rawSmsResponse = sendAT("AT+CMGL=\"REC UNREAD\"", SMS_READ_TIMEOUT);
                String basePayload = extractSmsBody(rawSmsResponse);
                
                if (basePayload.length() > 0) {
                    size_t basePayloadLen = basePayload.length();
                    
                    // 2. חילוץ המצביעים לצורך אימות (כעת משתמשים ב-basePayload הקיים)
                    const uint8_t* msgBytes = (const uint8_t*)basePayload.c_str();
                    const uint8_t* sigBytes  = msgBytes + 1;   // 64-byte signature at offset 1
                    const uint8_t* dataBytes = msgBytes + 65;  // actual payload after signature
                    size_t dataLen = basePayloadLen - 65;
                    
                    // 3. אימות חתימה
                    if (!verify_emergency(dataBytes, dataLen, sigBytes)) {
                        // Signature mismatch — discard silently
                        sendAT("AT+CMGD=1,4", 500);
                        xSemaphoreGiveRecursive(uartMutex);
                        continue; // חוזר לתחילת הלולאה להמתין לסמפור הבא
                    }
                    
                    // 4. חילוץ ואריזת הנתונים ל-MessagePacket
                    String prefix = basePayload.substring(0, 1);
                    String payload = basePayload.substring(66); // שאר הנתונים (לינקים)
                    
                    MessagePacket msg;
                    msg.prefix = (uint8_t)basePayload[0];

                    const uint8_t* dataBytes = (const uint8_t*)basePayload.c_str() + DATA_OFFSET;
                    size_t dataLen = basePayload.length() - DATA_OFFSET;

                    if (msg.prefix == MSG_EMERGENCY || msg.prefix == MSG_PRE_EMERGENCY) {
                        memcpy(&msg.payload.emergency, dataBytes,
                            min(dataLen, sizeof(EmergencyPayload)));
                        xQueueSend(emergencyQueue, &msg, portMAX_DELAY);
                    }
                    else if (msg.prefix == MSG_GEO_UPDATE) {
                        memcpy(&msg.payload.geo, dataBytes,
                            min(dataLen, sizeof(GeoPayload)));
                        xQueueSend(backgroundQueue, &msg, portMAX_DELAY);
                    }
                }
                
                sendAT("AT+CMGD=1,4", 500); // ניקוי הזיכרון במודם
                
                // משחררים את קו ה-UART למשימות אחרות
                xSemaphoreGiveRecursive(uartMutex);
            }
        }
    }
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
    sendAT("AT+CMGF=1",            500);   // Text mode SMS 
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

// =============================================================================
//  GEO UPDATE — Download geoindex.bin + geodata.bin over cellular modem
// =============================================================================
void triggerGeoUpdate(const String& baseUrl) {
    if (!activatePDP()) return;

    if (smsPendingInterrupt) {
        Serial.println("[GEO] Emergency pending — skipping.");
        deactivatePDP();
        return;
    }

    // Download both into temp files first — old files untouched until both succeed
    Serial.println("[GEO] Downloading geoindex.bin...");
    DownloadResult r1 = modemHttpGetToFile(baseUrl + "geoindex.bin", "/geoindex.tmp");

    if (r1 != DownloadResult::OK) {
        Serial.println("[GEO] Index download failed/interrupted — aborting, old files intact.");
        LittleFS.remove("/geoindex.tmp");  // clean up partial if any
        deactivatePDP();
        return;
    }

    if (smsPendingInterrupt) {
        Serial.println("[GEO] Interrupted before data file — reverting.");
        LittleFS.remove("/geoindex.tmp");
        deactivatePDP();
        return;
    }

    Serial.println("[GEO] Downloading geodata.bin...");
    DownloadResult r2 = modemHttpGetToFile(baseUrl + "geodata.bin", "/geodata.tmp");

    if (r2 != DownloadResult::OK) {
        Serial.println("[GEO] Data download failed/interrupted — reverting, old files intact.");
        LittleFS.remove("/geoindex.tmp");
        LittleFS.remove("/geodata.tmp");
        deactivatePDP();
        return;
    }

    // Both succeeded — now atomically swap in the new files
    LittleFS.remove("/geoindex.bin");
    LittleFS.rename("/geoindex.tmp", "/geoindex.bin");
    LittleFS.remove("/geodata.bin");
    LittleFS.rename("/geodata.tmp", "/geodata.bin");

    Serial.println("[GEO] Geo files updated successfully.");
    deactivatePDP();
}

// =============================================================================
//  MODEM HTTP GET → LittleFS
//  Downloads a file at 'url' and saves it to LittleFS at 'savePath'.
//  Uses SIM7080G AT+SHTTPCREATE/CON/REQ/READ/DIS command set.
//  Returns true on success.
// =============================================================================
DownloadResult modemHttpGetToFile(const String& url, const String& savePath) {
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
        sendAT("AT+SHTTPDIS=" + String(sessionId), 1000);
        return DownloadResult::ERROR;
    }

    // --- 4. Send GET request ---
    String reqCmd = "AT+SHTTPREQ=" + String(sessionId) + ",\"GET\",\"" + reqPath + "\"";
    String reqResp = sendAT(reqCmd, AT_LONG_DELAY);
    int dataLen = 0;
    int rIdx = reqResp.indexOf("+SHTTPREQ:");
    if (rIdx != -1) {
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
        return DownloadResult::ERROR;
    }

    Serial.printf("[HTTP] Receiving %d bytes → %s\n", dataLen, savePath.c_str());

    // --- 5. Open LittleFS file for writing ---
    File f = LittleFS.open(savePath, FILE_WRITE);
    if (!f) {
        Serial.println("[FS] Could not open file for writing.");
        sendAT("AT+SHTTPDIS=" + String(sessionId), 1000);
        return DownloadResult::ERROR;
    }

    // Helper: safe abort — closes + deletes partial file, disconnects session
    auto safeAbort = [&](DownloadResult reason) -> DownloadResult {
        f.close();
        LittleFS.remove(savePath);  // partial file is worse than no file
        sendAT("AT+SHTTPDIS=" + String(sessionId), 1000);
        Serial.printf("[HTTP] Aborted %s\n", savePath.c_str());
        return reason;
    };

    // --- 6. Read response body in 1024-byte chunks ---
    const int CHUNK = 1024;
    int bytesRead = 0;

    while (bytesRead < dataLen) {

        // *** Emergency interrupt check — once per chunk ***
        if (smsPendingInterrupt) {
            Serial.println("[HTTP] Emergency interrupt — bailing download safely.");
            return safeAbort(DownloadResult::INTERRUPTED);
        }

        int toRead = min(CHUNK, dataLen - bytesRead);
        char readCmd[64];
        snprintf(readCmd, sizeof(readCmd), "AT+SHTTPREAD=%d,%d,%d", sessionId, bytesRead, toRead);
        SerialModem.println(readCmd);

        String chunkHeader = "";
        uint32_t dl = millis() + 3000;
        bool headerPassed = false;

        while (millis() < dl) {
            while (SerialModem.available()) {
                char c = (char)SerialModem.read();
                if (!headerPassed) {
                    chunkHeader += c;
                    if (chunkHeader.endsWith("\n") && chunkHeader.indexOf("+SHTTPREAD:") != -1) {
                        headerPassed = true;
                        chunkHeader = "";
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

    // --- 7. Close file and disconnect ---
    f.close();
    Serial.printf("[HTTP] Saved %d bytes to %s\n", bytesRead, savePath.c_str());
    sendAT("AT+SHTTPDIS=" + String(sessionId), 1000);

    return (bytesRead >= dataLen) ? DownloadResult::OK : DownloadResult::ERROR;
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
     // Reuse vibration — distinguish by pattern later
}

String sendAT(const String& cmd, uint32_t timeoutMs) {
    if (xSemaphoreTakeRecursive(uartMutex, portMAX_DELAY) != pdTRUE) return "";

    SerialModem.println(cmd);

    String response = "";
    uint32_t deadline = millis() + timeoutMs;

    while (millis() < deadline) {
        while (SerialModem.available()) {
            response += (char)SerialModem.read();
        }
        if (response.indexOf("OK") != -1 || response.indexOf("ERROR") != -1) break;
    }

    xSemaphoreGiveRecursive(uartMutex);
    return response;
}