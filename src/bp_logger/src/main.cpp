#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
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
                    char macStr[18] = { 0 };
                    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
                            wifi_ptr->mac[0], wifi_ptr->mac[1],
                            wifi_ptr->mac[2], wifi_ptr->mac[3],
                            wifi_ptr->mac[4], wifi_ptr->mac[5]);
                    info_str += macStr;
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
                uint8_t buffer[size + 2];
                buffer[0] = (uint8_t)(WSMSGID_ESPNOW_ADDRS >> 8);
                buffer[1] = (uint8_t)(WSMSGID_ESPNOW_ADDRS);
                memcpy(&buffer[2], eeprom_storage.espnow_clients, size);
                websocket_send(buffer, sizeof(buffer), num);
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

void handleMacAddress(void)
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

void handleApInfo(void)
{
    String message = "== WiFi AP ==\n  - MAC: ";
    message += WiFi.softAPmacAddress();
    message += "\n  - IP: 192.168.4.1 / 24";
    message += "\n  - AP channel: ";
    message += ESP_NOW_CHANNEL;
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

void onStationDisconnected(const WiFiEventStationModeDisconnected& evt) {
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

void onStationGotIP(const WiFiEventStationModeGotIP& evt) {
#if UART_DEBUG_EN
    Serial.println("WiFi STA got IP");
#endif
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
    BUILTIN_LED_SET(1);
    /*buzzer_beep(440, 30);
    delay(200);
    buzzer_beep(440, 30);*/

    // Configure ESP-NOW
    espnow_init(wifi_get_channel(), esp_now_msp_rcvd, &ctrl_serial);
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

void onSoftAPModeStationConnected(const WiFiEventSoftAPModeStationConnected& evt) {
    /* Client connected */
#if WIFI_DBG
    wifi_log += "Wifi_AP_connect();\n";
#endif
#if UART_DEBUG_EN
    Serial.println("WiFi AP connected");
#endif
}

void onSoftAPModeStationDisconnected(const WiFiEventSoftAPModeStationDisconnected& evt) {
    /* Client disconnected */
#if WIFI_DBG
    wifi_log += "Wifi_AP_diconnect();\n";
#endif
#if UART_DEBUG_EN
    Serial.println("WiFi AP disconnected");
#endif
}


static void wifi_config_ap(void)
{
    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);

    // WiFi not connected, Start access point
    WiFi.disconnect(true);
    WiFi.forceSleepWake();
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    if (WiFi.softAP(WIFI_AP_SSID WIFI_AP_SUFFIX, WIFI_AP_PSK, ESP_NOW_CHANNEL,
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
    espnow_init(ESP_NOW_CHANNEL, esp_now_msp_rcvd, &ctrl_serial);
}


static int wifi_search_networks(void)
{
    int selected = -1;
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
        bool hidden;
        WiFi.getNetworkInfo(iter, ssid, encryptionType, rssi, mac, channel, hidden);

#if UART_DEBUG_EN
        Serial.printf("  %d: '%s', Ch:%d (%ddBm) BSSID:%s %s %s\n",
                      (iter + 1), ssid.c_str(), channel, rssi,
                      WiFi.BSSIDstr(iter).c_str(),
                      encryptionType == ENC_TYPE_NONE ? "open" : "",
                      hidden ? "hidden" : "");
#endif
        if (rssi < WIFI_SEARCH_RSSI_MIN) continue;  // Ignore very bad networks

        for (size_t jter = 0; jter < ARRAY_SIZE(eeprom_storage.wifi_nets); jter++) {
            wifi_networks_t const * const wifi_ptr = &eeprom_storage.wifi_nets[jter];
            if ((hidden && wifi_is_mac_valid(wifi_ptr) && memcmp(wifi_ptr->mac, mac, sizeof(wifi_ptr->mac))) ||
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
                    selected = jter; // selected
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
    return selected;
}


static void wifi_config(void)
{
    wifi_station_set_hostname(hostname);

    wifi_connection_state = WIFI_STATE_NA;
#if WIFI_DBG
    wifi_log = "";
#endif

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

    // Search for configure networks:
    int const network_idx = wifi_search_networks();

    if (network_idx < 0) {
        wifi_config_ap();
    } else {
        wifi_networks_t const * const wifi_ptr = &eeprom_storage.wifi_nets[network_idx];
#if UART_DEBUG_EN
        Serial.print("WiFi connecting: '");
        Serial.print(wifi_ptr->ssid);
        Serial.println("'");
#endif
        WiFi.begin(wifi_ptr->ssid, wifi_ptr->psk, WIFI_CHANNEL);
        wifi_connect_started = millis();

        // Initial ESP-NOW configuration
        espnow_init(ESP_NOW_CHANNEL, esp_now_msp_rcvd, &ctrl_serial);
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

    httpUpdater.setup(&server);
    server.begin();

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}


void setup()
{
    //ESP.eraseConfig();

    boot_log = "";
    switch (get_reset_reason()) {
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

    BUILTIN_LED_INIT();
    BUILTIN_LED_SET(0);

    eeprom_storage.setup();

    msp_handler.init();

    buzzer_init();

    Serial.setRxBufferSize(512);
    Serial.begin(SERIAL_BAUD, SERIAL_8N1, SERIAL_FULL, 1, SERIAL_INVERTED);

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

#if WIFI_DBG
    if (512 <= wifi_log.length())
        wifi_log = "F*CK, overflow!\n";
#endif
}
