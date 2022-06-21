#include "comm_espnow.h"
#include "main.h"
#include "storage.h"
#include "platform.h"
#include <ESP8266WiFi.h>
#include <espnow.h>


#define ESP_NOW_ETH_ALEN    6
#define DEBUG_TX_CALLBACK   0

String espnow_init_info = "ESP NOW is not configured!";


#if ESP_NOW

class CtrlSerialEspNow: public CtrlSerial
{
public:
    size_t available(void) {
        return 0; // not used
    }
    uint8_t read(void) {
        return 0; // not used
    }
    void write(uint8_t * buffer, size_t size) {
        esp_now_send(NULL, buffer, size);
        //websocket_send("MSP sent!");
    }
};

static CtrlSerialEspNow esp_now_sender;
static MSP esp_now_msp_rx;
static esp_now_msp_rcvd_cb_t msp_handler;
static CtrlSerial * ctrl_serial_ptr;
static uint32_t esp_now_channel;


static void esp_now_recv_cb(uint8_t *mac_addr, uint8_t *data, uint8_t const data_len)
{
    uint8_t iter;
    /* No data or peer is unknown => ignore */
    if (!data_len || !esp_now_is_peer_exist(mac_addr))
        return;

    //websocket_send("ESP NOW message received!");

    esp_now_msp_rx.markPacketFree();

    for (iter = 0; iter < data_len; iter++) {
        if (esp_now_msp_rx.processReceivedByte(data[iter])) {
            //  MSP received, check content
            mspPacket_t &packet = esp_now_msp_rx.getPacket();

            if (!msp_handler || 0 > msp_handler(packet)) {
                // Not handler internally, pass to serial
                MSP::sendPacket(&packet, ctrl_serial_ptr);
                //Serial.write((uint8_t*)packet.payload, packet.payloadSize);
            }
            /* Done, clear the packet and proceed */
            esp_now_msp_rx.markPacketFree();
        }
    }
}


#if DEBUG_TX_CALLBACK
static void esp_now_send_cb(uint8_t *mac_addr, u8 status) {
    String temp = "ESPNOW Sent: ";
    for (uint8_t iter = 0; iter < 6; iter++) {
        if (iter) temp += ":";
        temp += String(mac_addr[iter], HEX);
    }
    temp += (status ? " FAIL" : " SUCCESS");
    websocket_send(temp);
}
#endif


static void add_peer(espnow_clients_t const * const mac_addr, uint32_t const channel)
{
    char macStr[18] = { 0 };
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac_addr->mac_addr[0], mac_addr->mac_addr[1], mac_addr->mac_addr[2],
            mac_addr->mac_addr[3], mac_addr->mac_addr[4], mac_addr->mac_addr[5]);
    espnow_init_info += "[PEER ";
    espnow_init_info += String(macStr);
    if (esp_now_add_peer((u8 *)mac_addr->mac_addr,
                ESP_NOW_ROLE_COMBO, channel, NULL, 0) != 0) {
        espnow_init_info += " !FAILED!";
    }
    espnow_init_info += "] ";
    ESP.wdtFeed();
}
#endif // ESP_NOW


void espnow_init(uint32_t channel, esp_now_msp_rcvd_cb_t cb, CtrlSerial *serial_ptr)
{
#if ESP_NOW
    uint8_t iter;

    msp_handler = cb;
    ctrl_serial_ptr = serial_ptr;
    esp_now_channel = channel;

    espnow_init_info = "ESP NOW init: ";

    if (eeprom_storage.espnow_initialized != LOGGER_ESPNOW_INIT_KEY) {
        eeprom_storage.espnow_clients_count = 0;
        eeprom_storage.espnow_initialized = LOGGER_ESPNOW_INIT_KEY;
        // init storage
        memset(eeprom_storage.espnow_clients, 0, sizeof(eeprom_storage.espnow_clients));
#ifdef ESP_NOW_PEERS
        uint8_t peers[][ESP_NOW_ETH_ALEN] = ESP_NOW_PEERS;
        uint8_t const num_peers = sizeof(peers) / ESP_NOW_ETH_ALEN;
        for (iter = 0; iter < num_peers; iter++) {
            memcpy(eeprom_storage.espnow_clients[iter].mac_addr, peers[iter], ESP_NOW_ETH_ALEN);
        }
        eeprom_storage.espnow_clients_count = num_peers;
#endif
        eeprom_storage.markDirty();
    }

    if (esp_now_init() != 0) {
        espnow_init_info += "esp_now_init() failed!";
        return;
    }
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
#if DEBUG_TX_CALLBACK
    esp_now_register_send_cb(esp_now_send_cb);
#endif
    esp_now_register_recv_cb(esp_now_recv_cb);

    for (iter = 0; iter < eeprom_storage.espnow_clients_count; iter++) {
        add_peer(&eeprom_storage.espnow_clients[iter], channel);
    }

    espnow_init_info += " - DONE!";
#else
    (void)channel;
    (void)cb;
    (void)serial_ptr;
#endif // ESP_NOW
}


void espnow_update_clients(uint8_t const * const data, uint8_t const len, int const wsnum)
{
#if ESP_NOW
    espnow_clients_t const * const client = (espnow_clients_t*)data;
    uint8_t const count_new = (len / ESP_NOW_ETH_ALEN);
    uint8_t iter;

    espnow_init_info = "ESP NOW update: ";

    if (!count_new || !data || !ctrl_serial_ptr)
        // No valid data or not initilized yet
        return;

    // Remove all existing peers
    iter = eeprom_storage.espnow_clients_count;
    while (iter--) {
        esp_now_del_peer(eeprom_storage.espnow_clients[iter].mac_addr);
    }
    // Add new peers
    eeprom_storage.espnow_initialized = LOGGER_ESPNOW_INIT_KEY;
    eeprom_storage.espnow_clients_count = count_new;
    for (iter = 0; iter < count_new; iter++) {
        memcpy(eeprom_storage.espnow_clients[iter].mac_addr,
               client[iter].mac_addr, ESP_NOW_ETH_ALEN);
        add_peer(&client[iter], esp_now_channel);
    }
    espnow_init_info += " - DONE!";
    eeprom_storage.markDirty();
#else
    (void)data;
    (void)len;
    (void)wsnum;
#endif // ESP_NOW
}


String & espnow_get_info()
{
    return espnow_init_info;
}


void espnow_send_msp(mspPacket_t &msp)
{
#if ESP_NOW
    MSP::sendPacket(&msp, &esp_now_sender);
#else
    (void)msp;
#endif // ESP_NOW
}
