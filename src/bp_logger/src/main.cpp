#include <Arduino.h>
#include <WebSocketsServer.h>
#ifdef ARDUINO_ARCH_ESP32
#include <WebSocketsServer.h>
#include <WebServer.h>
#include <HTTPUpdate.h>
#include <esp_wifi.h>
#include <ESPmDNS.h>
#else
#include <Hash.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#endif
#if USE_LITTLE_FS // Must be included here to make 6.1x pio happy
#include <LittleFS.h>
#else
#include <SPIFFS.h>
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
#elif WIFI_AP_MAX_CONN < 1
#error "WIFI_AP_MAX_CONN min is 1"
#elif 8 < WIFI_AP_MAX_CONN
#error "WIFI_AP_MAX_CONN max is 8"
#endif

#if CONFIG_HANDSET
#define WIFI_AP_SUFFIX " HANDSET"
#elif CONFIG_HDZERO
#define WIFI_AP_SUFFIX " HDZero"
#else
#define WIFI_AP_SUFFIX " MODULE"
#endif

#ifndef WIFI_SEARCH_RSSI_MIN
#define WIFI_SEARCH_RSSI_MIN -100
#endif

#ifndef WIFI_TIMEOUT
#define WIFI_TIMEOUT 60 // default to 1min
#endif

#ifndef UART_DEBUG_EN
#define UART_DEBUG_EN 0
#endif
#define WIFI_DBG (0 || UART_DEBUG_EN)



#if (LED_BUILTIN != BOOT0_PIN) && (LED_BUILTIN != RESET_PIN) && (LED_BUILTIN != BUZZER_PIN) && (LED_BUILTIN != WS2812_PIN)
#define BUILTIN_LED_INIT()      pinMode((LED_BUILTIN), OUTPUT)
#define BUILTIN_LED_SET(_state) digitalWrite((LED_BUILTIN), !(_state))
#else
#define BUILTIN_LED_INIT()
#define BUILTIN_LED_SET(_state)
#endif


#ifdef ARDUINO_ARCH_ESP32
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
#else
ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
ESP8266HTTPUpdateServer httpUpdater;
#endif

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

uint8_t char_to_dec(uint8_t const chr)
{
    if ('0' <= chr && chr <= '9')
        return chr - '0';
    return 0;
}

uint8_t hex_char_to_dec(uint8_t const chr)
{
    if ('A' <= chr && chr <= 'F')
        return (10 + (chr - 'A'));
    if ('a' <= chr && chr <= 'f')
        return (10 + (chr - 'a'));
    return char_to_dec(chr);
}

uint8_t wifi_ap_channel_get(void)
{
    // Calculate WiFi channel (1...12) according to UID
    uint8_t const my_uid[] = {MY_UID};
    return ((my_uid[3] + my_uid[4] + my_uid[5]) % 12) + 1;
}

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

#ifdef ARDUINO_ARCH_ESP32
#include <rom/rtc.h>
String get_reset_reason(RESET_REASON const reason)
{
    switch (reason) {
        case  1 : return "POWERON_RESET";         /**<1, Vbat power on reset*/
        case  3 : return "SW_RESET";              /**<3, Software reset digital core*/
        case  4 : return "OWDT_RESET";            /**<4, Legacy watch dog reset digital core*/
        case  5 : return "DEEPSLEEP_RESET";       /**<5, Deep Sleep reset digital core*/
        case  6 : return "SDIO_RESET";            /**<6, Reset by SLC module, reset digital core*/
        case  7 : return "TG0WDT_SYS_RESET";      /**<7, Timer Group0 Watch dog reset digital core*/
        case  8 : return "TG1WDT_SYS_RESET";      /**<8, Timer Group1 Watch dog reset digital core*/
        case  9 : return "RTCWDT_SYS_RESET";      /**<9, RTC Watch dog Reset digital core*/
        case 10 : return "INTRUSION_RESET";       /**<10, Instrusion tested to reset CPU*/
        case 11 : return "TGWDT_CPU_RESET";       /**<11, Time Group reset CPU*/
        case 12 : return "SW_CPU_RESET";          /**<12, Software reset CPU*/
        case 13 : return "RTCWDT_CPU_RESET";      /**<13, RTC Watch dog Reset CPU*/
        case 14 : return "EXT_CPU_RESET";         /**<14, for APP CPU, reseted by PRO CPU*/
        case 15 : return "RTCWDT_BROWN_OUT_RESET";/**<15, Reset when the vdd voltage is not stable*/
        case 16 : return "RTCWDT_RTC_RESET";      /**<16, RTC Watch dog reset digital core and rtc module*/
        default : return "NO_MEAN";
    }
}
void print_reset_reason(void)
{
    boot_log += "CPU0: ";
    boot_log += get_reset_reason(rtc_get_reset_reason(0));
    boot_log += "CPU1: ";
    boot_log += get_reset_reason(rtc_get_reset_reason(1));
}
#else
void print_reset_reason(void)
{
    rst_info *resetInfo;
    resetInfo = ESP.getResetInfoPtr();

    switch (resetInfo->reason) {
        case REASON_WDT_RST:
            /* 1 = hardware watch dog reset */
            boot_log += "Reset: HW WD";
            break;
        case REASON_EXCEPTION_RST:
            /* 2 = exception reset, GPIO status won’t change */
            boot_log += "Reset: Exception";
            break;
        case REASON_SOFT_WDT_RST:
            /* 3 = software watch dog reset, GPIO status won’t change */
            boot_log += "Reset: SW WD";
            break;
        case REASON_SOFT_RESTART:
            /* 4 = software restart ,system_restart , GPIO status won’t change */
            boot_log += "Reset: SW restart";
            break;
        case REASON_DEEP_SLEEP_AWAKE:
            /* 5 = wake up from deep-sleep */
            boot_log += "Reset: deep sleep wakeup";
            break;
        case REASON_EXT_SYS_RST:
            /* 6 = external system reset */
            boot_log += "Reset: External";
            break;
        case REASON_DEFAULT_RST:
        default:
            /* 0 = normal startup by power on */
            break;
    }
}
#endif


void wifi_networks_report(int wsnum)
{
    if (eeprom_storage.wifi_is_valid()) {
        // CMD_WIFINETS</\/\IDXMACSSID>
        String info_str = "CMD_WIFINETS";
        for (size_t index = 0; index < ARRAY_SIZE(eeprom_storage.wifi_nets); index++) {
            wifi_networks_t const * const wifi_ptr = &eeprom_storage.wifi_nets[index];
            if (wifi_is_psk_valid(wifi_ptr) && (wifi_is_ssid_valid(wifi_ptr) || wifi_is_mac_valid(wifi_ptr))) {
                info_str += "/\\/\\";
                if (index < 10) info_str += '0';
                info_str += index;
                if (wifi_is_mac_valid(wifi_ptr)) {
                    info_str += mac_addr_print(wifi_ptr->mac);
                } else {
                    info_str += "00:00:00:00:00:00";
                }
                if (wifi_is_ssid_valid(wifi_ptr)) {
                    info_str += wifi_ptr->ssid;
                }
            }
        }
        websocket_send(info_str, wsnum);
    }
}

/*************************************************************************/

int esp_now_msp_rcvd(mspPacket_t &msp_pkt)
{
    if (msp_handler.handle_received_msp(msp_pkt) < 0) {
        // Not handler internally, pass to serial
        MSP::sendPacket(&msp_pkt, &ctrl_serial);
    }
    return 0;
}


/*************************************************************************/
String mac_addr_print(uint8_t const * const mac_addr)
{
    char macStr[18] = {0};
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac_addr[0], mac_addr[1], mac_addr[2],
            mac_addr[3], mac_addr[4], mac_addr[5]);
    return String(macStr);
}


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
            info_str += " ms: ";
            info_str += millis();
            info_str += " rssi: ";
            info_str += WiFi.RSSI();

            websocket_send(info_str, num);
            websocket_send(espnow_get_info(), num);
#if ESP_NOW
            info_str = "ESP-NOW channel: ";
            info_str += espnow_channel();
            websocket_send(info_str, num);
#endif
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
            if (wifi_log) {
                websocket_send(wifi_log, num);
                wifi_log = "";
            }
#endif
            if (boot_log)
                websocket_send(boot_log, num);
#if ESP_NOW
            if (eeprom_storage.espnow_initialized == LOGGER_ESPNOW_INIT_KEY) {
                uint8_t const size = eeprom_storage.espnow_clients_count * 6;
                if (size) {
                    uint8_t buffer[size + 2];
                    buffer[0] = (uint8_t)(WSMSGID_ESPNOW_ADDRS >> 8);
                    buffer[1] = (uint8_t)(WSMSGID_ESPNOW_ADDRS);
                    memcpy(&buffer[2], eeprom_storage.espnow_clients, size);
                    websocket_send(buffer, sizeof(buffer), num);
                }
            }
#endif
            wifi_networks_report(num);

            msp_handler.syncSettings(num);
            break;
        }

        case WStype_TEXT: {
            size_t index;
            char const * temp = strstr((char*)payload, "WIFIADD/");
            if (temp) { // WiFi network add
                temp += 8;
                wifi_networks_t * wifi_ptr = NULL;
                for (index = 0; index < ARRAY_SIZE(eeprom_storage.wifi_nets); index++, wifi_ptr = NULL) {
                    wifi_ptr = &eeprom_storage.wifi_nets[index];
                    if (!wifi_is_ssid_valid(wifi_ptr) && !wifi_is_psk_valid(wifi_ptr) && !wifi_is_mac_valid(wifi_ptr)) {
                        // Free slot found
                        break;
                    }
                }
                if (wifi_ptr) {
                    size_t len;
                    // Parse SSID
                    len  = (10 * char_to_dec(*temp++));
                    len += char_to_dec(*temp++);
                    temp += 1; // Skip '/'
                    memcpy(wifi_ptr->ssid, temp, len);
                    wifi_ptr->ssid[len] = 0;
                    temp += len + 1; // Skip SSID + '/'
                    // Parse PSK
                    len  = (10 * char_to_dec(*temp++));
                    len += char_to_dec(*temp++);
                    temp += 1; // Skip '/'
                    memcpy(wifi_ptr->psk, temp, len);
                    wifi_ptr->psk[len] = 0;
                    temp += len; // Skip PSK
                    // Parse MAC if included
                    if ((((uintptr_t)temp - (uintptr_t)payload) + 1 + 12) <= length) {
                        temp += 1; // Skip '/'
                        wifi_ptr->mac[0] = (char_to_dec(temp[0]) << 4) + char_to_dec(temp[1]);
                        wifi_ptr->mac[1] = (char_to_dec(temp[2]) << 4) + char_to_dec(temp[3]);
                        wifi_ptr->mac[2] = (char_to_dec(temp[4]) << 4) + char_to_dec(temp[5]);
                        wifi_ptr->mac[3] = (char_to_dec(temp[6]) << 4) + char_to_dec(temp[7]);
                        wifi_ptr->mac[4] = (char_to_dec(temp[8]) << 4) + char_to_dec(temp[9]);
                        wifi_ptr->mac[5] = (char_to_dec(temp[10]) << 4) + char_to_dec(temp[11]);
                    }
                    eeprom_storage.markDirty();
                    wifi_networks_report(num);
                } else {
                    websocket_send("WIFIADD: No free slots!", num);
                }
                break;
            }
            temp = strstr((char*)payload, "WIFIDEL/");
            if (temp) { // WiFi network remove
                temp += 8;
                if ((((uintptr_t)temp - (uintptr_t)payload) + 2) <= length) {
                    // Parse index
                    index = (10 * char_to_dec(*temp++));
                    index += char_to_dec(*temp++);
                    if (index < ARRAY_SIZE(eeprom_storage.wifi_nets)) {
                        wifi_networks_t * const wifi_ptr = &eeprom_storage.wifi_nets[index];
                        memset(wifi_ptr, 0, sizeof(*wifi_ptr));
                        eeprom_storage.markDirty();
                        wifi_networks_report(num);
                    } else {
                        websocket_send("WIFIDEL: Invalid index!", num);
                    }
                } else {
                    websocket_send("WIFIDEL: Invalid msg len!", num);
                }
                break;
            }

            msp_handler.parse_command((char*)payload, length, num);
            break;
        }

        case WStype_BIN: {
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

void handleMacAddress(void)
{
    uint8_t channel;
#ifdef ARDUINO_ARCH_ESP32
    wifi_second_chan_t secondChan;
    esp_wifi_get_channel(&channel, &secondChan);
    (void)secondChan;
#else
    channel = wifi_get_channel();
#endif

    String message = "WiFi STA MAC: ";
    message += WiFi.macAddress();
    message += "\n  - channel in use: ";
    message += channel;
    message += "\n  - mode: ";
    message += (uint8_t)WiFi.getMode();
    message += "\n\nWiFi SoftAP MAC: ";
    message += WiFi.softAPmacAddress();
    message += "\n  - IP: ";
    message += WiFi.softAPIP().toString();
    message += "\n";
    server.send(200, "text/plain", message);
}

void handleApInfo(void)
{
    String message = "== WiFi AP ==\n  - MAC: ";
    message += WiFi.softAPmacAddress();
    message += "\n  - IP: 192.168.4.1 / 24";
    message += "\n  - AP channel: ";
    message += wifi_ap_channel_get();
    message += "\n  - SSID: ";
    message += WIFI_AP_SSID;
    message +=  WIFI_AP_SUFFIX;
    message += "\n  - PSK: ";
    message += WIFI_AP_PSK;
    message += "\n  - Hidden: ";
    message += !!WIFI_AP_HIDDEN;
    message += "\n  - Conn max: ";
    message += WIFI_AP_MAX_CONN;
    server.send(200, "text/plain", message);
}

void handle_fs(void)
{
    String message = "FS info: used ";
#ifdef ARDUINO_ARCH_ESP32
    message += " [ESP32 not implemented!]";
#else
    FSInfo fs_info;
    FILESYSTEM.info(fs_info);
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
#endif
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

#ifdef ARDUINO_ARCH_ESP8266
static WiFiEventHandler stationConnectedHandler;
static WiFiEventHandler stationDisconnectedHandler;
static WiFiEventHandler stationGotIpAddress;
static WiFiEventHandler stationDhcpTimeout;
#endif
static uint32_t wifi_connect_started;
static uint8_t wifi_connection_state;


#ifdef ARDUINO_ARCH_ESP32
void onStationConnected(WiFiEvent_t event, WiFiEventInfo_t info)
#else
void onStationConnected(const WiFiEventStationModeConnected& evt)
#endif
{
    // Don't let AP to be triggered while still connecting to STA
    wifi_connect_started = millis();
#if WIFI_DBG
#if UART_DEBUG_EN
    wifi_log = "";
#endif
    wifi_log += "Wifi_STA_connect() ";
    wifi_log += "RSSI=";
    wifi_log += WiFi.RSSI();
#if !UART_DEBUG_EN
    wifi_log += "\n";
#endif
#if UART_DEBUG_EN
    Serial.println(wifi_log);
#endif
#endif
}

#ifdef ARDUINO_ARCH_ESP32
void onStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
#else
void onStationDisconnected(const WiFiEventStationModeDisconnected& evt)
#endif
{
#if WIFI_DBG
#if UART_DEBUG_EN
    wifi_log = "";
#endif
    wifi_log += "Wifi_STA_diconnect() ";
    wifi_log += "reason:";
    wifi_log += evt.reason;
#if !UART_DEBUG_EN
    wifi_log += "\n";
#endif
#if UART_DEBUG_EN
    Serial.println(wifi_log);
#endif
#endif
    if (WIFI_STATE_NA != wifi_connection_state) {
#if UART_DEBUG_EN
        Serial.println("  CONNECTION LOST!");
#endif
        /* Start check again only after connection drop */
        wifi_connect_started = millis();
        wifi_connection_state = WIFI_STATE_NA;
    }
    MDNS.end();
    WiFi.reconnect(); // Force reconnect

    BUILTIN_LED_SET(0);
}

#ifdef ARDUINO_ARCH_ESP32
void onStationGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
#else
void onStationGotIP(const WiFiEventStationModeGotIP& evt)
#endif
{
#if UART_DEBUG_EN
    Serial.println("WiFi STA got IP");
#endif
#if WIFI_DBG
    wifi_log += "Wifi_STA_GotIP();\n";
#endif
    wifi_connection_state = WIFI_STATE_STA;

    String instance = String(hostname) + "_" + WiFi.macAddress();
    instance.replace(":", "");

    MDNS.end();
#ifdef ARDUINO_ARCH_ESP32
    if (MDNS.begin(hostname)) {
        MDNS.setInstanceName(instance);
        MDNS.addService("http", "tcp", 80);
        MDNS.addServiceTxt("http", "tcp", "vendor", "elrs");
        MDNS.addServiceTxt("http", "tcp", "target", (const char *)target_name);
        MDNS.addServiceTxt("http", "tcp", "version", LATEST_COMMIT_STR);
        MDNS.addServiceTxt("http", "tcp", "type", "bp_logger");
    }
#else
    MDNS.setHostname(hostname);
    if (MDNS.begin(hostname, WiFi.localIP()))
    {
        MDNS.setInstanceName(hostname);
        MDNSResponder::hMDNSService service = MDNS.addService(instance.c_str(), "http", "tcp", 80);
        MDNS.addServiceTxt(service, "vendor", "elrs");
        MDNS.addServiceTxt(service, "target", target_name);
        MDNS.addServiceTxt(service, "version", LATEST_COMMIT_STR);
        MDNS.addServiceTxt(service, "type", "bp_logger");

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
#endif

    led_set(LED_WIFI_STA);
    BUILTIN_LED_SET(1);
    /*buzzer_beep(440, 30);
    delay(200);
    buzzer_beep(440, 30);*/

    // Configure ESP-NOW
    uint8_t channel;
#ifdef ARDUINO_ARCH_ESP32
    wifi_second_chan_t secondChan;
    esp_wifi_get_channel(&channel, &secondChan);
    (void)secondChan;
#else
    channel = wifi_get_channel();
#endif
    espnow_init(channel, esp_now_msp_rcvd);
}

void onStationDhcpTimeout(void)
{
#if WIFI_DBG
    wifi_log += "Wifi_STA_DhcpTimeout();\n";
#endif
    wifi_connect_started = millis();
    wifi_connection_state = WIFI_STATE_NA;
    MDNS.end();
#if UART_DEBUG_EN
    Serial.println("WiFi STA DHCP timeout");
#endif
}



static void wifi_config_ap(void)
{
    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    uint8_t const channel = wifi_ap_channel_get();

    //esp_wifi_set_channel(chan,WIFI_SECOND_CHAN_NONE);

    // WiFi not connected, Start access point
    WiFi.disconnect(true);
#ifdef ARDUINO_ARCH_ESP8266
    WiFi.forceSleepWake();
#endif
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    if (WiFi.softAP(WIFI_AP_SSID WIFI_AP_SUFFIX, WIFI_AP_PSK, channel,
                    !!WIFI_AP_HIDDEN, WIFI_AP_MAX_CONN)) {
        wifi_connection_state = WIFI_STATE_AP;
#if WIFI_DBG
        wifi_log += "WifiAP started\n";
#endif
        led_set(LED_WIFI_AP);
        buzzer_beep(400, 20);
        BUILTIN_LED_SET(1);
#if UART_DEBUG_EN
        Serial.println("WiFi AP ok");
#endif
    } else {
        //wifi_connect_started = millis() - 100;
        WiFi.mode(WIFI_OFF);
#if UART_DEBUG_EN
        Serial.println("WiFi AP start fail");
#endif
    }
    // Configure ESP-NOW
    espnow_init(channel, esp_now_msp_rcvd);
}


typedef struct {
    uint8_t network_index;
    uint8_t channel;
} wifi_info_t;

#if PLATFORM_ESP32
#define ENC_TYPE_NONE WIFI_AUTH_OPEN
#endif

static bool wifi_search_networks(wifi_info_t &output)
{
    output.network_index = 0xff;
    int rssi_best = WIFI_SEARCH_RSSI_MIN;
#if UART_DEBUG_EN
    Serial.println("-------------------------");
    Serial.println(" Available WiFi networks");
    Serial.println("-------------------------");
#endif
    int const numberOfNetworks = WiFi.scanNetworks(false, true); // Show hidden
    for (int iter = 0; iter < numberOfNetworks; iter++) {
        String ssid;
        uint8_t encryptionType;
        int32_t rssi;
        uint8_t* mac;
        int32_t channel;
        bool hidden = false;
#ifdef ARDUINO_ARCH_ESP32
        WiFi.getNetworkInfo(iter, ssid, encryptionType, rssi, mac, channel);
#else
        WiFi.getNetworkInfo(iter, ssid, encryptionType, rssi, mac, channel, hidden);
#endif

#if UART_DEBUG_EN
        Serial.printf("  %d: '%s', Ch:%d (%ddBm) BSSID:%s %s %s\n",
                      (iter + 1), ssid.c_str(), channel, rssi,
                      WiFi.BSSIDstr(iter).c_str(),
                      encryptionType == ENC_TYPE_NONE ? "open" : "",
                      hidden ? "hidden": "");
#endif
        (void)hidden;

        if (rssi < WIFI_SEARCH_RSSI_MIN) continue;  // Ignore very bad networks

        for (size_t jter = 0; jter < ARRAY_SIZE(eeprom_storage.wifi_nets); jter++) {
            wifi_networks_t const * const wifi_ptr = &eeprom_storage.wifi_nets[jter];
            if ((wifi_is_mac_valid(wifi_ptr) && memcmp(wifi_ptr->mac, mac, sizeof(wifi_ptr->mac))) ||
                    (wifi_is_ssid_valid(wifi_ptr) && strncmp(wifi_ptr->ssid, ssid.c_str(), sizeof(wifi_ptr->ssid)) == 0)) {
#if UART_DEBUG_EN
                Serial.print("    ** Configured network found! ");
#endif
#if WIFI_DBG
                wifi_log += "Configured network found! ";
                wifi_log += jter;
                wifi_log += "\n";
#endif
                // Select based on the best RSSI
                if (rssi_best < rssi) {
                    rssi_best = rssi;
                    output.network_index = jter; // selected
                    output.channel = channel;
#if UART_DEBUG_EN
                    Serial.print(" << selected!");
#endif
                }
#if UART_DEBUG_EN
                Serial.println();
#endif
                break;
            }
        }
    }
    WiFi.scanDelete();  // Cleanup scan results
    return (output.network_index != 0xff);
}

static void wifi_config(void)
{
#ifdef ARDUINO_ARCH_ESP32
    WiFi.setHostname(hostname);
#else
    wifi_station_set_hostname(hostname);
#endif

    wifi_connection_state = WIFI_STATE_NA;
#if WIFI_DBG
    wifi_log = "";
#endif

    /* Set AP MAC to UID for ESP-NOW messaging */
    uint8_t ap_mac[] = {MY_UID};
    if (ap_mac[0] & 0x1) ap_mac[0] &= ~0x1;

#ifdef ARDUINO_ARCH_ESP32
    esp_wifi_set_mac(WIFI_IF_AP, &ap_mac[0]);
#else
    wifi_set_macaddr(SOFTAP_IF, &ap_mac[0]);
#endif

    /* Force WIFI off until it is realy needed */
    WiFi.mode(WIFI_OFF);
#ifdef ARDUINO_ARCH_ESP8266
    WiFi.forceSleepBegin();
#endif
    delay(10);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
    WiFi.disconnect(true);

    /* STA mode callbacks */
#ifdef ARDUINO_ARCH_ESP32
    WiFi.onEvent(onStationConnected, SYSTEM_EVENT_STA_CONNECTED);
    WiFi.onEvent(onStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);
    WiFi.onEvent(onStationGotIP, SYSTEM_EVENT_STA_GOT_IP);
    //WiFi.onEvent(onStationDhcpTimeout, SYSTEM_EVENT_STA_LOST_IP);
#else
    stationConnectedHandler = WiFi.onStationModeConnected(&onStationConnected);
    stationDisconnectedHandler = WiFi.onStationModeDisconnected(&onStationDisconnected);
    stationGotIpAddress = WiFi.onStationModeGotIP(&onStationGotIP);
    stationDhcpTimeout = WiFi.onStationModeDHCPTimeout(&onStationDhcpTimeout);
#endif

    WiFi.mode(WIFI_AP_STA);  // WIFI_AP_STA

    // Search for configure networks:
    wifi_info_t scan_result;
    if (wifi_search_networks(scan_result)) {
        wifi_networks_t const * const wifi_ptr =
            &eeprom_storage.wifi_nets[scan_result.network_index];
#if UART_DEBUG_EN
        Serial.print("WiFi connecting: '");
        Serial.print(wifi_ptr->ssid);
        Serial.println("'");
#endif
        WiFi.begin(wifi_ptr->ssid, wifi_ptr->psk, scan_result.channel);
        wifi_connect_started = millis();

        // Initial ESP-NOW configuration
        espnow_init(scan_result.channel, esp_now_msp_rcvd);
    } else {
        wifi_config_ap();
    }
}


static void wifi_check(void)
{
    #define WIFI_LOOP_TIMEOUT (WIFI_TIMEOUT * 1000)
    static uint32_t wifi_last_check, led_state;

    uint32_t const now = millis();
    if (WIFI_LOOP_TIMEOUT < (now - wifi_connect_started)) {
#if UART_DEBUG_EN
        Serial.println("WIFI config timeout... start AP");
#endif
        wifi_config_ap();
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
    server.on("/ap", handleApInfo);
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

#ifdef ARDUINO_ARCH_ESP32
#else
    httpUpdater.setup(&server);
#endif
    server.begin();

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}


void setup()
{
    //ESP.eraseConfig();

    boot_log = "";
    print_reset_reason();

#if CONFIG_HDZERO
    //delay(500);  // delay boot a bit
#endif
    Serial.setRxBufferSize(512);
#ifdef ARDUINO_ARCH_ESP32
    Serial.begin(SERIAL_BAUD, SERIAL_8N1, -1, -1, SERIAL_INVERTED);
#else
    Serial.begin(SERIAL_BAUD, SERIAL_8N1, SERIAL_FULL, 1, SERIAL_INVERTED);
#endif

    BUILTIN_LED_INIT();
    BUILTIN_LED_SET(0);

    eeprom_storage.setup();

    msp_handler.init();

    buzzer_init();

    led_init();
    led_set(LED_INIT);

#if CONFIG_STM_UPDATER && (BOOT0_PIN == 2 || BOOT0_PIN == 0)
    reset_stm32_to_app_mode();
#endif

    FILESYSTEM.begin();
    //FILESYSTEM.format();

#if UART_DEBUG_EN
#warning "Serial debugging enabled!"
    Serial.println("ELRS Logger started... starting up the wifi next");
#endif

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

#ifdef ARDUINO_ARCH_ESP8266
    ESP.wdtFeed();
#endif

    if (0 <= serialEvent(input_log_string)) {
#if CONFIG_HDZERO
        if (!strstr(input_log_string.c_str(), "Resource\\_000.bmp") && !strstr(input_log_string.c_str(), "fc_variant:  ...")) // IGNORE dummy dbg print
#endif
            websocket_send(input_log_string);
        input_log_string = "";
    }
#ifdef ARDUINO_ARCH_ESP8266
    ESP.wdtFeed();
#endif

    server.handleClient();
    webSocket.loop();
#ifdef ARDUINO_ARCH_ESP8266
    ESP.wdtFeed();
#endif

    msp_handler.loop();

    eeprom_storage.update();

#if WIFI_DBG
    if (512 <= wifi_log.length())
        wifi_log = "F*CK, overflow!\n";
#endif
}
