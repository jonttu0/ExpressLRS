#include "comm_espnow.h"
#include "main.h"
#include "storage.h"
#include "platform.h"
#include <ESP8266WiFi.h>
#include <espnow.h>


#define MAC_ADDR_LEN        6
#define DEBUG_TX_CALLBACK   0
#define DEBUG_LOG           0

static String espnow_init_info = "";
#if DEBUG_LOG
  #define LOG_SET(...) espnow_init_info  = __VA_ARGS__
  #define LOG_ADD(...) espnow_init_info += __VA_ARGS__
#else
  #define LOG_SET(...)
  #define LOG_ADD(...)
#endif

#if ESP_NOW

static uint8_t msp_tx_buffer[128];
//static uint8_t broadcast_addr[] = {0xff,0xff,0xff,0xff,0xff,0xff};
static uint32_t esp_now_channel;
static esp_now_msp_rcvd_cb_t msp_handler;
static bool esp_now_initialized = false;


static void esp_now_recv_cb(uint8_t *mac_addr, uint8_t *data, uint8_t const data_len)
{
    static MSP esp_now_msp_rx;
    uint8_t iter;

    if (!data_len)
        return;
#if 0
    String temp = "ESPNOW received: ";
    temp += mac_addr_print(mac_addr);
    websocket_send(temp);
#endif
    esp_now_msp_rx.markPacketFree();

    for (iter = 0; iter < data_len; iter++) {
        if (esp_now_msp_rx.processReceivedByte(data[iter])) {
            //  MSP received, check content
            mspPacket_t &packet = esp_now_msp_rx.getPacket();
            if (esp_now_is_peer_exist(mac_addr)) {
                if (msp_handler)
                    msp_handler(packet);
            } else {
                /* Handle broadcast messages */
                if (packet.type == MSP_PACKET_V2_COMMAND /*||
                    packet.type == MSP_PACKET_V2_RESPONSE*/) {
                }
            }
            /* Done, clear the packet and proceed */
            esp_now_msp_rx.markPacketFree();
        }
    }
}


#if DEBUG_TX_CALLBACK
static void esp_now_send_cb(uint8_t *mac_addr, u8 status) {
    String temp = "ESPNOW Sent: ";
    temp += mac_addr_print(mac_addr);
    temp += (status ? " FAIL" : " SUCCESS");
    websocket_send(temp);
}
#endif


static void add_peer(uint8_t const * const mac_addr, uint32_t const channel)
{
    LOG_ADD("[PEER ");
    LOG_ADD(mac_addr_print(mac_addr));
    if (esp_now_add_peer((u8 *)mac_addr, ESP_NOW_ROLE_COMBO, channel, NULL, 0) != 0) {
        LOG_ADD(" !FAILED!");
    }
    LOG_ADD("] ");
    ESP.wdtFeed();
}
#endif // ESP_NOW


void espnow_init(uint32_t const channel, esp_now_msp_rcvd_cb_t const cb)
{
    ESP.wdtFeed();
#if ESP_NOW

    uint8_t my_uid[MAC_ADDR_LEN];
    uint8_t iter;

    WiFi.softAPmacAddress(&my_uid[0]);

    if (esp_now_initialized) {
        if (channel != esp_now_channel) {
            // Already initialized, just update wifi channel
            esp_now_set_peer_channel(my_uid, channel);
            iter = eeprom_storage.espnow_clients_count;
            while (iter--) {
                esp_now_set_peer_channel(eeprom_storage.espnow_clients[iter].mac_addr, channel);
            }
        }
        esp_now_channel = channel;
        return;
    }

    esp_now_initialized = true;
    msp_handler = cb;
    esp_now_channel = channel;

    LOG_SET("ESP NOW init: ");

    if (eeprom_storage.espnow_initialized != LOGGER_ESPNOW_INIT_KEY) {
        eeprom_storage.espnow_clients_count = 0;
        eeprom_storage.espnow_initialized = LOGGER_ESPNOW_INIT_KEY;
        // init storage
        memset(eeprom_storage.espnow_clients, 0, sizeof(eeprom_storage.espnow_clients));
        eeprom_storage.markDirty();
    }

    if (esp_now_init() != 0) {
        LOG_ADD("esp_now_init() failed!");
        return;
    }
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
#if DEBUG_TX_CALLBACK
    esp_now_register_send_cb(esp_now_send_cb);
#endif
    esp_now_register_recv_cb(esp_now_recv_cb);

    add_peer(my_uid, channel);
    for (iter = 0; iter < eeprom_storage.espnow_clients_count; iter++) {
        add_peer(eeprom_storage.espnow_clients[iter].mac_addr, channel);
    }

    LOG_ADD(" - DONE!");
#else
    (void)channel;
    (void)cb;
#endif // ESP_NOW
}


uint8_t espnow_channel(void)
{
#if ESP_NOW
    return esp_now_channel;
#else
    return 0;
#endif
}


void espnow_update_clients(uint8_t const * const data, uint8_t const len, int const wsnum)
{
#if ESP_NOW
    espnow_clients_t const * const client = (espnow_clients_t*)data;
    uint8_t const count_new = (len / MAC_ADDR_LEN);
    uint8_t iter;

    LOG_SET("ESP NOW update: ");

    if (!data) {
        // No valid data or not initilized yet
        LOG_ADD(" - FAILED!");
        return;
    }

    // Remove existing peers
    iter = eeprom_storage.espnow_clients_count;
    while (iter--) {
        esp_now_del_peer(eeprom_storage.espnow_clients[iter].mac_addr);
    }
    // Add new peers
    eeprom_storage.espnow_initialized = LOGGER_ESPNOW_INIT_KEY;
    eeprom_storage.espnow_clients_count = count_new;
    for (iter = 0; iter < count_new; iter++) {
        memcpy(eeprom_storage.espnow_clients[iter].mac_addr,
               client[iter].mac_addr, MAC_ADDR_LEN);
        add_peer(client[iter].mac_addr, esp_now_channel);
    }
    LOG_ADD(" - DONE!");
    eeprom_storage.markDirty();
#else
    espnow_init_info = "ESP NOW is not configured!";
    (void)data;
    (void)len;
    (void)wsnum;
#endif // ESP_NOW
}


String & espnow_get_info(void)
{
    return espnow_init_info;
}


void espnow_send_msp(mspPacket_t &msp)
{
#if ESP_NOW
    size_t const len = MSP::bufferPacket(msp_tx_buffer, &msp);
    if (len) {
        esp_now_send(NULL, msp_tx_buffer, len);
    }
#else
    (void)msp;
#endif // ESP_NOW
}
