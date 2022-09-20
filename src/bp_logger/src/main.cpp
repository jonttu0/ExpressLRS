#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#if WIFI_MANAGER
#include <WiFiManager.h>
#endif /* WIFI_MANAGER */
#include <ESP8266HTTPUpdateServer.h>
#if USE_LITTLE_FS // Must be included here to make 6.1x pio happy
#include <LittleFS.h>
#endif

#include "storage.h"
#include "main.h"
#if !CONFIG_HDZERO
#include "stm32_ota.h"
#include "stm32Updater.h"
#endif
#include "common_defs.h"
#include "html_default.h"
#include "led.h"
#include "comm_espnow.h"
#include "expresslrs_msp.h"
#include "hdzero_msp.h"
#if CONFIG_HDZERO
#include "hdzero_webpage.h"
#endif
#include "buzzer.h"


#ifndef SERIAL_INVERTED
#define SERIAL_INVERTED 0
#endif

#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "ExpressLRS AP"
#endif
#ifndef WIFI_AP_PSK
#define WIFI_AP_PSK "expresslrs"
#endif
#ifndef WIFI_AP_HIDDEN
#define WIFI_AP_HIDDEN 0
#endif
#ifndef WIFI_AP_MAX_CONN
#define WIFI_AP_MAX_CONN 2
#endif

#if CONFIG_HANDSET
#define WIFI_AP_SUFFIX " HANDSET"
#elif CONFIG_HDZERO
#define WIFI_AP_SUFFIX " HDZero"
#else
#define WIFI_AP_SUFFIX " MODULE"
#endif

#ifndef WIFI_TIMEOUT
#define WIFI_TIMEOUT 60 // default to 1min
#endif

#define WIFI_DBG 0


ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
ESP8266HTTPUpdateServer httpUpdater;

#ifndef LOGGER_HOST_NAME
#define LOGGER_HOST_NAME "elrs_logger"
#endif
static const char hostname[] = LOGGER_HOST_NAME;
static const char target_name[] = STR(TARGET_NAME);

#if WIFI_DBG
String wifi_log = "";
#endif
String boot_log = "";

enum {
    WSMSGID_ESPNOW_ADDRS = WSMSGID_BASE_ESPNOW,

    // STM32 control messages
    WSMSGID_STM32_RESET = WSMSGID_BASE_STM32,
};


/*************************************************************************/

class CtrlSerialPrivate: public CtrlSerial
{
public:
    size_t available(void) {
        return Serial.available();
    }
    uint8_t read(void) {
        return Serial.read();
    }

    void write(uint8_t * buffer, size_t size) {
        Serial.write(buffer, size);
    }
};

CtrlSerialPrivate my_ctrl_serial;
CtrlSerial& ctrl_serial = my_ctrl_serial;

#if CONFIG_HDZERO
HDZeroMsp msp_handler(&my_ctrl_serial);
#else
ExpresslrsMsp msp_handler(&my_ctrl_serial);
#endif

/*************************************************************************/

int get_reset_reason(void)
{
    rst_info *resetInfo;
    resetInfo = ESP.getResetInfoPtr();
    int reset_reason = resetInfo->reason;
    return reset_reason;
}


/*************************************************************************/

int esp_now_msp_rcvd(mspPacket_t &msp_pkt)
{
    return msp_handler.handle_received_msp(msp_pkt);
}


/*************************************************************************/

void websocket_send(char const * data, int num)
{
    if (0 <= num)
        webSocket.sendTXT(num, data);
    else
        webSocket.broadcastTXT(data);
}

void websocket_send(String & data, int num)
{
    if (!data.length())
        return;
    websocket_send(data.c_str(), num);
}

void websocket_send(uint8_t const * data, uint8_t const len, int const num)
{
    websocket_send_bin(data, len, num);
}

void websocket_send_bin(uint8_t const * data, uint8_t const len, int const num)
{
    if (!len || !data)
        return;
    if (0 <= num)
        webSocket.sendBIN(num, data, len);
    else
        webSocket.broadcastBIN(data, len);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
        case WStype_DISCONNECTED:
            break;

        case WStype_CONNECTED:
        {
            //IPAddress ip = webSocket.remoteIP(num);
            //Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\r\n", num, ip[0], ip[1], ip[2], ip[3], payload);
            //socketNumber = num;

            IPAddress my_ip;
            my_ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP() : WiFi.softAPIP();
            String info_str = "NA";
            info_str = "My IP address = ";
            info_str += my_ip.toString();

            websocket_send(info_str, num);
            websocket_send(espnow_get_info(), num);

            websocket_send(target_name, num);
#if defined(LATEST_COMMIT)
            info_str = "Current version (SHA): ";
            uint8_t commit_sha[] = {LATEST_COMMIT};
            for (uint8_t iter = 0; iter < sizeof(commit_sha); iter++) {
                info_str += String(commit_sha[iter], HEX);
            }
#if LATEST_COMMIT_DIRTY
            info_str += "-dirty";
#endif
            websocket_send(info_str, num);
#endif // LATEST_COMMIT
#if WIFI_DBG
            websocket_send(wifi_log, num);
#endif
            if (boot_log)
                websocket_send(boot_log, num);
#if ESP_NOW
            if (eeprom_storage.espnow_initialized == LOGGER_ESPNOW_INIT_KEY) {
                uint8_t const size = eeprom_storage.espnow_clients_count * 6;
                uint8_t buffer[size + 2];
                buffer[0] = (uint8_t)(WSMSGID_ESPNOW_ADDRS >> 8);
                buffer[1] = (uint8_t)(WSMSGID_ESPNOW_ADDRS);
                memcpy(&buffer[2], eeprom_storage.espnow_clients, size);
                websocket_send(buffer, sizeof(buffer), num);
            }
#endif
            msp_handler.syncSettings(num);
            break;
        }

        case WStype_TEXT: {
            msp_handler.parse_command((char*)payload, length, num);
            break;
        }

        case WStype_BIN: {
            //Serial.printf("[%u] get binary length: %u\r\n", num, length);
            //hexdump(payload, length);
            // echo data back to browser
            //webSocket.sendBIN(num, payload, length);
            websoc_bin_hdr_t const * const header = (websoc_bin_hdr_t*)payload;
            length -= sizeof(header->msg_id);

            // ====================== ESP-NOW COMMANDS ==============
            if (WSMSGID_ESPNOW_ADDRS == header->msg_id) {
                // ESP-Now client list
                espnow_update_clients(header->payload, length);
                websocket_send(espnow_get_info(), num);

            // ====================== STM COMMANDS ==================
#if CONFIG_STM_UPDATER
            } else if (WSMSGID_STM32_RESET == header->msg_id) {
                // STM reset
                reset_stm32_to_app_mode();
#endif // CONFIG_STM_UPDATER

            // ====================== DEFAULT =======================
            } else if (msp_handler.parse_command(header, length, num) < 0) {
                String error = "Invalid message: 0x";
                error += String(header->msg_id, HEX);
                websocket_send(error, num);
            }
            break;
        }

        default:
            break;
    }
}


/***********************************************************************************/

void sendReturn()
{
    server.send_P(200, "text/html", GO_BACK);
}

void handle_recover()
{
#if CONFIG_HDZERO
    server.send_P(200, "text/html", HDZ_INDEX_HTML);
#else
    server.send_P(200, "text/html", INDEX_HTML);
#endif
}

void handleMacAddress()
{
    String message = "WiFi STA MAC: ";
    message += WiFi.macAddress();
    message += "\n  - channel in use: ";
    message += wifi_get_channel();
    message += "\n  - mode: ";
    message += (uint8_t)WiFi.getMode();
    message += "\n\nWiFi SoftAP MAC: ";
    message += WiFi.softAPmacAddress();
    message += "\n  - IP: ";
    message += WiFi.softAPIP().toString();
    message += "\n";
    server.send(200, "text/plain", message);
}

void handle_fs(void)
{
    FSInfo fs_info;
    FILESYSTEM.info(fs_info);
    String message = "FS ino: used ";
    message += fs_info.usedBytes;
    message += "/";
    message += fs_info.totalBytes;
    message += "\n**** FS files ****\n";

    Dir dir = FILESYSTEM.openDir("/");
    while (dir.next()) {
        message += dir.fileName();
        if (dir.fileSize()) {
            File f = dir.openFile("r");
            message += " - ";
            message += f.size();
            message += "B";
        }
        message += "\n";
    }

    server.send(200, "text/plain", message);
}

String getContentType(String filename)
{
    if(filename.endsWith(".html"))
        return "text/html";
    else if(filename.endsWith(".css"))
        return "text/css";
    else if(filename.endsWith(".js"))
        return "application/javascript";
    else if(filename.endsWith(".ico"))
        return "image/x-icon";
    else if(filename.endsWith(".gz"))
        return "application/x-gzip";
    return "text/plain";
}

bool handleFileRead(String path)
{
    if (path.endsWith("/"))
        // Send the index file if a folder is requested
        path += "index.html";
    // Get the MIME type
    String contentType = getContentType(path);
    uint8_t pathWithGz = FILESYSTEM.exists(path + ".gz");
    if (pathWithGz || FILESYSTEM.exists(path)) {
        if (pathWithGz)
            path += ".gz";
        File file = FILESYSTEM.open(path, "r");
        server.streamFile(file, contentType);
        file.close();
        return true;
    }
    return false;
}


/*************************************************/

enum {
    WIFI_STATE_NA = 0,
    WIFI_STATE_STA,
    WIFI_STATE_AP,
};

static WiFiEventHandler stationConnectedHandler;
static WiFiEventHandler stationDisconnectedHandler;
static WiFiEventHandler stationGotIpAddress;
static WiFiEventHandler stationDhcpTimeout;
static WiFiEventHandler AP_ConnectedHandler;
static WiFiEventHandler AP_DisconnectedHandler;
static uint32_t wifi_connect_started;
static uint8_t wifi_connection_state;


void onStationConnected(const WiFiEventStationModeConnected& evt) {
#if WIFI_DBG
    wifi_log += "Wifi_STA_connect() ";
    wifi_log += "RSSI=";
    wifi_log += WiFi.RSSI();
    wifi_log += "\n";
#endif
}

void onStationDisconnected(const WiFiEventStationModeDisconnected& evt) {
#if WIFI_DBG
    wifi_log += "Wifi_STA_diconnect();\n";
#endif
}

void onStationGotIP(const WiFiEventStationModeGotIP& evt) {
#if WIFI_DBG
    wifi_log += "Wifi_STA_GotIP();\n";
#endif
    wifi_connection_state = WIFI_STATE_STA;

    MDNS.end();
    MDNS.setHostname(hostname);
    if (MDNS.begin(hostname, WiFi.localIP()))
    {
        String instance = String(hostname) + "_" + WiFi.macAddress();
        instance.replace(":", "");
        MDNS.setInstanceName(hostname);
        MDNSResponder::hMDNSService service = MDNS.addService(instance.c_str(), "http", "tcp", 80);
        MDNS.addServiceTxt(service, "vendor", "elrs");
        MDNS.addServiceTxt(service, "target", target_name);
        MDNS.addServiceTxt(service, "version", LATEST_COMMIT_STR);
        MDNS.addServiceTxt(service, "type", "rx");

        MDNS.addService(instance.c_str(), "ws", "tcp", 81);

        // If the probe result fails because there is another device on the network with the same name
        // use our unique instance name as the hostname. A better way to do this would be to use
        // MDNSResponder::indexDomain and change wifi_hostname as well.
        MDNS.setHostProbeResultCallback([instance](const char* p_pcDomainName, bool p_bProbeResult) {
            if (!p_bProbeResult) {
                WiFi.hostname(instance);
                MDNS.setInstanceName(instance);
            }
        });
    }
    led_set(LED_WIFI_STA);
    /*buzzer_beep(440, 30);
    delay(200);
    buzzer_beep(440, 30);*/
}

void onStationDhcpTimeout(void)
{
#if WIFI_DBG
    wifi_log += "Wifi_STA_DhcpTimeout();\n";
#endif
    wifi_connection_state = WIFI_STATE_NA;
    MDNS.end();
}

void onSoftAPModeStationConnected(const WiFiEventSoftAPModeStationConnected& evt) {
    /* Client connected */
#if WIFI_DBG
    wifi_log += "Wifi_AP_connect();\n";
#endif
}

void onSoftAPModeStationDisconnected(const WiFiEventSoftAPModeStationDisconnected& evt) {
    /* Client disconnected */
#if WIFI_DBG
    wifi_log += "Wifi_AP_diconnect();\n";
#endif
}

static void wifi_config_ap(void)
{
    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);

    // WiFi not connected, Start access point
    WiFi.setAutoReconnect(true);
    WiFi.disconnect(true);
    WiFi.forceSleepWake();
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(WIFI_AP_SSID WIFI_AP_SUFFIX, WIFI_AP_PSK, ESP_NOW_CHANNEL,
                WIFI_AP_HIDDEN, WIFI_AP_MAX_CONN);
    wifi_connection_state = WIFI_STATE_AP;
#if WIFI_DBG
    wifi_log += "WifiAP started\n";
#endif
}

static void wifi_config(void)
{
    wifi_station_set_hostname(hostname);

    wifi_connection_state = WIFI_STATE_NA;
#if WIFI_DBG
    wifi_log = "";
#endif
    boot_log = "Reset reason: ";
    switch (get_reset_reason()) {
        case REASON_WDT_RST:
            /* 1 = hardware watch dog reset */
            boot_log += "HW WD reset";
            break;
        case REASON_EXCEPTION_RST:
            /* 2 = exception reset, GPIO status won’t change */
            boot_log += "Exception";
            break;
        case REASON_SOFT_WDT_RST:
            /* 3 = software watch dog reset, GPIO status won’t change */
            boot_log += "SW WD reset";
            break;
        case REASON_SOFT_RESTART:
            /* 4 = software restart ,system_restart , GPIO status won’t change */
            boot_log += "SW restart";
            break;
        case REASON_DEEP_SLEEP_AWAKE:
            /* 5 = wake up from deep-sleep */
            boot_log += "deep sleep wakeup";
            break;
        case REASON_EXT_SYS_RST:
            /* 6 = external system reset */
            boot_log += "External reset";
            break;
        case REASON_DEFAULT_RST:
        default:
            /* 0 = normal startup by power on */
            break;
    }

#if defined(WIFI_SSID) && defined(WIFI_PSK)
    /* Force WIFI off until it is realy needed */
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();
    delay(10);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
    WiFi.disconnect(true);

    /* STA mode callbacks */
    stationConnectedHandler = WiFi.onStationModeConnected(&onStationConnected);
    stationDisconnectedHandler = WiFi.onStationModeDisconnected(&onStationDisconnected);
    stationGotIpAddress = WiFi.onStationModeGotIP(&onStationGotIP);
    stationDhcpTimeout = WiFi.onStationModeDHCPTimeout(&onStationDhcpTimeout);
    /* AP mode callbacks */
    AP_ConnectedHandler = WiFi.onSoftAPModeStationConnected(&onSoftAPModeStationConnected);
    AP_DisconnectedHandler = WiFi.onSoftAPModeStationDisconnected(&onSoftAPModeStationDisconnected);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PSK, WIFI_CHANNEL);
    wifi_connect_started = millis();

#elif WIFI_MANAGER
    #warning "WiFi manager not supported anymore!"
    WiFiManager wifiManager;
    wifiManager.setConfigPortalTimeout(WIFI_TIMEOUT);
    if (wifiManager.autoConnect(WIFI_AP_SSID WIFI_AP_SUFFIX)) {
        // AP found, connected
    }
#else
    wifi_config_ap();
#endif /* WIFI_MANAGER */

    ESP.wdtFeed();
    espnow_init(ESP_NOW_CHANNEL, esp_now_msp_rcvd, &ctrl_serial);
}

static void wifi_check(void)
{
    #define WIFI_LOOP_TIMEOUT (WIFI_TIMEOUT * 1000)
    static uint32_t wifi_last_check, led_state;

    uint32_t const now = millis();
    if (WIFI_LOOP_TIMEOUT < (now - wifi_connect_started)) {
        wifi_config_ap();
        led_set(LED_WIFI_AP);
        buzzer_beep(400, 20);
    } else if (500 <= (now - wifi_last_check)) {
        wifi_last_check = now;
        /* Blink led */
        led_set(led_state ? LED_WIFI_AP : LED_OFF);
        led_state ^= 1;
    }
}

static void wifi_config_server(void)
{
    server.on("/fs", handle_fs);
    server.on("/return", sendReturn);
    server.on("/mac", handleMacAddress);
#if CONFIG_STM_UPDATER
    server.on("/upload", HTTP_POST, // STM32 OTA upgrade
        stm32_ota_handleFileUploadEnd, stm32_ota_handleFileUpload);
#endif
    server.onNotFound([]() {
        if (!handleFileRead(server.uri())) {
            // No matching file, respond with a 404 (Not Found) error
            //server.send(404, "text/plain", "404: Not Found");
            handle_recover();
        }
    });

    httpUpdater.setup(&server);
    server.begin();

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}


void setup()
{
    //ESP.eraseConfig();

    eeprom_storage.setup();

    msp_handler.init();

    buzzer_init();

    //Serial.setRxBufferSize(256);
    Serial.begin(SERIAL_BAUD, SERIAL_8N1, SERIAL_FULL, 1, SERIAL_INVERTED);

    led_init();
    led_set(LED_INIT);

#if CONFIG_STM_UPDATER && (BOOT0_PIN == 2 || BOOT0_PIN == 0)
    reset_stm32_to_app_mode();
#endif

    FILESYSTEM.begin();
    //FILESYSTEM.format();

    wifi_config();
    wifi_config_server();

    // wsnum does not matter because settings_valid == false
    msp_handler.syncSettings(-1);
}


int serialEvent(String &log_string)
{
    int temp;
    uint8_t inChar;
    while (Serial.available()) {
        temp = Serial.read();
        if (temp < 0)
            break;

        inChar = (uint8_t)temp;

        if (msp_handler.parse_data(inChar) < 0) {
            if (inChar == '\r')
                continue;
            else if (inChar == '\n')
                return 0;
            // add if printable char
            if (isprint(inChar))
                log_string += (char)inChar;
            // for safety... send in pieces
            if (70 <= log_string.length())
                return 0;
        }
    }
    return -1;
}


void loop()
{
    static String input_log_string = "";

    if (wifi_connection_state == WIFI_STATE_NA)
        wifi_check();

    ESP.wdtFeed();

    if (0 <= serialEvent(input_log_string)) {
        websocket_send(input_log_string);
        input_log_string = "";
    }
    ESP.wdtFeed();

    server.handleClient();
    webSocket.loop();
    MDNS.update();

    msp_handler.loop();

    eeprom_storage.update();
}
