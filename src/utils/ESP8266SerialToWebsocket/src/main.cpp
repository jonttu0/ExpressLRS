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

#include "storage.h"
#include "handset.h"
#include "main.h"
#include "stm32_ota.h"
#include "stm32Updater.h"
#include "stk500.h"
#include "msp.h"
#include "common_defs.h"
#include "rc_channels.h"
#include "html_default.h"
#include "led.h"
#include "comm_espnow.h"


#define STRINGIFY(s) #s
#define STRINGIFY_TMP(A) STRINGIFY(A)
#define CONCAT(A, B) A##B

//#define INVERTED_SERIAL // Comment this out for non-inverted serial

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
#else
#define WIFI_AP_SUFFIX " MODULE"
#endif

#ifndef WIFI_TIMEOUT
#define WIFI_TIMEOUT 60 // default to 1min
#endif

#define WIFI_DBG 0

MDNSResponder mdns;

ESP8266WebServer server(80);

WebSocketsServer webSocket = WebSocketsServer(81);
ESP8266HTTPUpdateServer httpUpdater;

String inputString = "";
#if WIFI_DBG
String wifi_log = "";
#endif

#if CONFIG_HANDSET
/* Handset specific data */
struct gimbal_limit gimbals[TX_NUM_ANALOGS];
struct mixer mixer[TX_NUM_MIXER];
static uint8_t handset_num_switches, handset_num_aux;
static uint8_t handset_mixer_ok = 0, handset_adjust_ok = 0;
#endif

#define MSP_SAVE_DELAY_MS   100
static volatile uint32_t msp_send_save_requested_ms;

/*************************************************************************/

int get_reset_reason(void)
{
  rst_info *resetInfo;
  resetInfo = ESP.getResetInfoPtr();
  int reset_reason = resetInfo->reason;
  return reset_reason;
}

/*************************************************************************/

void beep(int note, int duration, int wait=1)
{
#ifdef BUZZER_PIN
#if BUZZER_PASSIVE
  tone(BUZZER_PIN, note, duration);
  if (wait)
    delay(duration);
#else // BUZZER_ACTIVE
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
#endif /* BUZZER_PASSIVE */
#else /* !BUZZER_PIN */
  (void)note, (void)duration, (void)wait;
#endif /* BUZZER_PIN */
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

static uint8_t settings_rate;
static uint8_t settings_power, settings_power_max;
static uint8_t settings_tlm;
static uint8_t settings_region;
static uint8_t settings_valid;

MSP msp_handler;
mspPacket_t msp_out;

void SettingsWrite(uint8_t * buff, uint8_t len)
{
  // Fill MSP packet
  msp_out.type = MSP_PACKET_V1_ELRS;
  msp_out.flags = MSP_ELRS_INT;
  msp_out.payloadSize = len;
  msp_out.function = ELRS_INT_MSP_PARAMS;
  memcpy((void*)msp_out.payload, buff, len);
  // Send packet
  MSP::sendPacket(&msp_out, &my_ctrl_serial);
}

void handleSettingRate(const char * input, int num = -1)
{
  String settings_out = "[INTERNAL ERROR] something went wrong";
  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_rates_input=";
    settings_out += settings_rate;
  } else if (*input == '=') {
    input++;
    settings_out = "Setting rate: ";
    settings_out += input;
    // Write to ELRS
    char val = *input;
    uint8_t buff[] = {1, (uint8_t)(val == 'R' ? 0xff : (val - '0'))};
    SettingsWrite(buff, sizeof(buff));
  }
  websocket_send(settings_out, num);
}

void handleSettingPower(const char * input, int num = -1)
{
  String settings_out = "[INTERNAL ERROR] something went wrong";
  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_power_input=";
    settings_out += settings_power;
    settings_out += ",";
    settings_out += settings_power_max;
  } else if (*input == '=') {
    input++;
    settings_out = "Setting power: ";
    settings_out += input;
    // Write to ELRS
    char val = *input;
    uint8_t buff[] = {3, (uint8_t)(val == 'R' ? 0xff : (val - '0'))};
    SettingsWrite(buff, sizeof(buff));
  }
  websocket_send(settings_out, num);
}

void handleSettingTlm(const char * input, int num = -1)
{
  String settings_out = "[INTERNAL ERROR] something went wrong";
  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_tlm_input=";
    settings_out += settings_tlm;
  } else if (*input == '=') {
    input++;
    settings_out = "Setting telemetry: ";
    settings_out += input;
    // Write to ELRS
    char val = *input;
    uint8_t buff[] = {2, (uint8_t)(val == 'R' ? 0xff : (val - '0'))};
    SettingsWrite(buff, sizeof(buff));
  }
  websocket_send(settings_out, num);
}

void handleSettingRfPwr(const char * input, int num = -1)
{
  String settings_out = "[INTERNAL ERROR] something went wrong";
  if (input == NULL || *input == '?') {
    return;
  } else if (*input == '=') {
    input++;
    settings_out = "Setting RF PWR: ";
    settings_out += input;
    // Write to ELRS
    char val = *input;
    if ('A' <= val)
      val = 10 + (val - 'A');
    else
      val = (val - '0');
    uint8_t buff[] = {6, (uint8_t)val};
    SettingsWrite(buff, sizeof(buff));
  }
  websocket_send(settings_out, num);
}

void handleSettingRfModule(const char * input, int num = -1)
{
  String settings_out = "[INTERNAL ERROR] something went wrong";
  if (input == NULL || *input == '?') {
    return;
  } else if (*input == '=') {
    input++;
    settings_out = "Setting RF module: ";
    settings_out += input;
    // Write to ELRS
    char val = *input;
    if ('A' <= val)
      val = 10 + (val - 'A');
    else
      val = (val - '0');
    uint8_t buff[] = {4, (uint8_t)val};
    SettingsWrite(buff, sizeof(buff));
  }
  websocket_send(settings_out, num);
}

void handleSettingDomain(const char * input, int num = -1)
{
  String settings_out = "[ERROR] Domain set is not supported!";
  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_region_domain=";
    settings_out += settings_region;
  }
  websocket_send(settings_out, num);
}

void MspVtxWriteToElrs(uint16_t const freq)
{
  if (freq == 0)
    return;

  eeprom_storage.vtx_freq = freq;
  eeprom_storage.markDirty();

  // Fill MSP packet
  msp_out.reset();
  msp_out.type = MSP_PACKET_V1_CMD;
  msp_out.flags = MSP_VERSION | MSP_STARTFLAG;
  msp_out.function = MSP_VTX_SET_CONFIG;
  msp_out.payloadSize = 2; // 4 => 2, power and pitmode can be ignored
  msp_out.payload[0] = (freq & 0xff);
  msp_out.payload[1] = (freq >> 8);
  //msp_out.payload[2] = power;
  //msp_out.payload[3] = (power == 0); // pit mode
  // Send packet to ELRS
  MSP::sendPacket(&msp_out, &my_ctrl_serial);

  // Request save to make set permanent
  msp_send_save_requested_ms = millis();
}

void MspVtxWrite(const char * input, int num)
{
  String settings_out = "[ERROR] invalid command";
  uint16_t freq;
  //uint8_t power = 1;

  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_vtx_freq=";
    settings_out += eeprom_storage.vtx_freq;
  } else if (input[0] == '=') {
    settings_out = "Setting vtxfreq to: ";

    freq = (input[1] - '0');
    freq = freq*10 + (input[2] - '0');
    freq = freq*10 + (input[3] - '0');
    freq = freq*10 + (input[4] - '0');

    if (freq == 0)
      return;

    settings_out += freq;
    settings_out += "MHz";

    MspVtxWriteToElrs(freq);

    // Send to other esp-now clients
    msp_out.type = MSP_PACKET_V2_COMMAND;
    msp_out.flags = 0;
    msp_out.function = MSP_VTX_SET_CONFIG;
    msp_out.payloadSize = 2; // 4 => 2, power and pitmode can be ignored
    msp_out.payload[0] = (freq & 0xff);
    msp_out.payload[1] = (freq >> 8);
    espnow_send_msp(msp_out);
  }
  websocket_send(settings_out, num);
}

void SettingsGet(uint8_t wsnum)
{
  if (!settings_valid) {
    /* Unknown... reguest update */
    uint8_t buff[] = {0, 0};
    SettingsWrite(buff, sizeof(buff));
  } else {
    /* Valid, just send those to client */
    delay(5);
    handleSettingDomain(NULL, wsnum);
    delay(5);
    handleSettingRate(NULL, wsnum);
    delay(5);
    handleSettingPower(NULL, wsnum);
    delay(5);
    handleSettingTlm(NULL, wsnum);
    delay(5);
  }
  MspVtxWrite(NULL, wsnum);
}

#if CONFIG_HANDSET
uint8_t char_to_dec(uint8_t const chr)
{
  if ('0' <= chr && chr <= '9') {
    return chr - '0';
  }
  return 0;
}

uint8_t char_to_hex(uint8_t chr)
{
  if ('A' <= chr && chr <= 'F')
    return (10 + (chr - 'A'));
  if ('a' <= chr && chr <= 'f')
    return (10 + (chr - 'a'));
  return char_to_dec(chr);
}

uint8_t char_u8_to_hex(const char * chr)
{
  uint8_t val = char_to_hex(*chr++);
  val <<= 4;
  val += char_to_hex(*chr++);
  return val;
}

uint8_t char_u8_to_dec(const char * chr)
{
  uint8_t val = char_to_dec(*chr++) * 10;
  val += char_to_dec(*chr++);
  return val;
}

uint16_t char_u12_to_hex(const char * chr)
{
  uint16_t val = char_to_hex(*chr++);
  val <<= 4;
  val += char_to_hex(*chr++);
  val <<= 4;
  val += char_to_hex(*chr++);
  return val;
}

void handleHandsetCalibrate(const char * input)
{
  uint8_t value = 0;
  // Fill MSP packet
  msp_out.reset();

  value = char_to_hex(input[3]);
  if (value == 1)
    value = GIMBAL_CALIB_LOW;
  else if (value == 2)
    value = GIMBAL_CALIB_MID;
  else if (value == 3)
    value = GIMBAL_CALIB_HIGH;

  if (!strncmp(input, "L1_", 3)) {
    value |= (GIMBAL_CALIB_L_V);
  } else if (!strncmp(input, "L2_", 3)) {
    value |= (GIMBAL_CALIB_L_H);
  } else if (!strncmp(input, "R1_", 3)) {
    value |= (GIMBAL_CALIB_R_V);
  } else if (!strncmp(input, "R2_", 3)) {
    value |= (GIMBAL_CALIB_R_H);
  }

  if (!value)
    return;

  msp_out.type = MSP_PACKET_V1_ELRS;
  msp_out.flags = MSP_ELRS_INT;
  msp_out.function = ELRS_HANDSET_CALIBRATE;
  msp_out.addByte(value);
  msp_out.addByte(1); // Start
  msp_out.setIteratorToSize();
  // Send packet
  MSP::sendPacket(&msp_out, &my_ctrl_serial);
}

void handleHandsetCalibrateResp(uint8_t * data, int num = -1)
{
  String out = "ELRS_handset_calibrate=ERROR!";
  websocket_send(out, num);
}


void handleHandsetMixer(const char * input, size_t length)
{
  uint32_t iter, index, scale;
  const char * read = input;
  // Fill MSP packet
  msp_out.reset();
  msp_out.type = MSP_PACKET_V1_ELRS;
  msp_out.flags = MSP_ELRS_INT;
  msp_out.function = ELRS_HANDSET_MIXER;
  for (iter = 0; iter < ARRAY_SIZE(mixer) && ((size_t)(read - input) < length); iter++) {
    // channel index
    index = char_to_hex(*read++);
    msp_out.addByte(index);
    // channel out
    msp_out.addByte(char_to_hex(*read++));
    // inverted
    msp_out.addByte(char_to_hex(*read++));
    // Scale
    if (index < 4) {
      scale = char_u8_to_dec(read);
      msp_out.addByte(scale);

      String temp = "Gimbal: ";
      temp += index;
      temp += " Scale: ";
      temp += scale;
      websocket_send(temp);

      read += 2;
    }
  }
  msp_out.setIteratorToSize();
  // Send packet
  MSP::sendPacket(&msp_out, &my_ctrl_serial);
}

void handleHandsetMixerResp(uint8_t * data, int num = -1)
{
  String out = "ELRS_handset_mixer=";
  uint8_t iter;
  if (data) {
    for (iter = 0; iter < ARRAY_SIZE(mixer); iter++) {
      if (data[0] < ARRAY_SIZE(mixer)) {
        mixer[data[0]] = (struct mixer){
          .index=data[1], .inv=data[2], .scale=data[3]};
      }
      data += 4;
    }
    handset_num_switches = *data++;
    handset_num_aux = *data++;
  }

  out += handset_num_aux;
  out += ";";
  out += handset_num_switches;
  out += ";";

  for (iter = 0; iter < ARRAY_SIZE(mixer); iter++) {
    if (iter)
      out += ',';
    out += iter;
    out += ":";
    out += mixer[iter].index;
    out += ":";
    out += mixer[iter].inv;
    out += ":";
    out += mixer[iter].scale;
  }

  websocket_send(out, num);
}


void handleHandsetAdjust(const char * input)
{
  const char * temp;
  uint16_t value;
  // Fill MSP packet
  msp_out.reset();
  msp_out.type = MSP_PACKET_V1_ELRS;
  msp_out.flags = MSP_ELRS_INT;

  if (strncmp(&input[3], "min", 3) == 0)
    msp_out.function = ELRS_HANDSET_ADJUST_MIN;
  else if (strncmp(&input[3], "mid", 3) == 0)
    msp_out.function = ELRS_HANDSET_ADJUST_MID;
  else if (strncmp(&input[3], "max", 3) == 0)
    msp_out.function = ELRS_HANDSET_ADJUST_MAX;
  else
    /* not valid cmd */
    return;

  if (!strncmp(input, "L1_", 3))
    msp_out.payload[0] = GIMBAL_IDX_L1;
  else if (!strncmp(input, "L2_", 3))
    msp_out.payload[0] = GIMBAL_IDX_L2;
  else if (!strncmp(input, "R1_", 3))
    msp_out.payload[0] = GIMBAL_IDX_R1;
  else if (!strncmp(input, "R2_", 3))
    msp_out.payload[0] = GIMBAL_IDX_R2;
  else
    /* not valid cmd */
    return;

  temp = strstr((char*)input, "=");
  value = char_u12_to_hex(&temp[1]);
  msp_out.payload[1] = (uint8_t)(value >> 8);
  msp_out.payload[2] = (uint8_t)value;
  // Send packet
  msp_out.payloadSize = 3;
  MSP::sendPacket(&msp_out, &my_ctrl_serial);
}

void handleHandsetAdjustResp(uint8_t * data, int num = -1)
{
  String out = "ELRS_handset_adjust=";
  uint8_t iter;
  if (data) {
    memcpy(gimbals, data, sizeof(gimbals));
  }

  for (iter = 0; iter < ARRAY_SIZE(gimbals); iter++) {
    struct gimbal_limit * limits = &gimbals[iter];
    out += limits->low;
    out += ":";
    out += limits->mid;
    out += ":";
    out += limits->high;
    out += ";";
  }

  websocket_send(out, num);
}

void HandsetConfigGet(uint8_t wsnum, uint8_t force=0)
{
  if (!handset_mixer_ok || !handset_adjust_ok || force) {
    // Fill MSP packet
    msp_out.reset();
    msp_out.type = MSP_PACKET_V1_ELRS;
    msp_out.flags = MSP_ELRS_INT;
    msp_out.function = ELRS_HANDSET_CONFIGS_LOAD;
    msp_out.payload[0] = 1;
    msp_out.payload[1] = 1;
    msp_out.payloadSize = 2;
    // Send packet
    MSP::sendPacket(&msp_out, &my_ctrl_serial);
    return;
  }

  delay(5);
  handleHandsetMixerResp(NULL, wsnum);
  delay(5);
  handleHandsetAdjustResp(NULL, wsnum);
}

void HandsetConfigSave(uint8_t wsnum)
{
  // Fill MSP packet
  msp_out.reset();
  msp_out.type = MSP_PACKET_V1_ELRS;
  msp_out.flags = MSP_ELRS_INT;
  msp_out.function = ELRS_HANDSET_CONFIGS_SAVE;
  msp_out.payload[0] = 1;
  msp_out.payload[1] = 1;
  msp_out.payloadSize = 2;
  // Send packet
  MSP::sendPacket(&msp_out, &my_ctrl_serial);
}

void handleHandsetTlmLnkStats(uint8_t * data)
{
  String out = "ELRS_tlm_uldl=";
  LinkStatsLink_t * stats = (LinkStatsLink_t*)data;
  // Uplink
  out += "ULQ:"; out += stats->uplink_Link_quality;
  out += ",UR1:"; out += (int8_t)stats->uplink_RSSI_1;
  out += ",UR2:"; out += (int8_t)stats->uplink_RSSI_2;
  out += ",USN:"; out += (int8_t)stats->uplink_SNR;
  //out += ",PWR:"; out += stats->uplink_TX_Power;
  //out += ",MO:"; out += stats->rf_Mode;
  // Downlink
  out += ",DLQ:"; out += stats->downlink_Link_quality;
  out += ",DR1:"; out += (int8_t)(stats->downlink_RSSI - 120);
  out += ",DSN:"; out += (int8_t)stats->downlink_SNR;
  websocket_send(out);
}

void handleHandsetTlmBattery(uint8_t * data)
{
  String out = "ELRS_tlm_batt=";
  LinkStatsBatt_t * stats = (LinkStatsBatt_t*)data;
  out += "V:"; out += BYTE_SWAP_U16(stats->voltage);
  out += ",A:"; out += BYTE_SWAP_U16(stats->current);
  out += ",C:"; out += BYTE_SWAP_U32((uint32_t)stats->capacity << 8);
  out += ",R:"; out += stats->remaining;
  websocket_send(out);
}

void handleHandsetTlmGps(uint8_t * data)
{
  String out = "ELRS_tlm_gps=";
  GpsOta_t * stats = (GpsOta_t*)data;
  out += "lat:"; out += BYTE_SWAP_U32(stats->latitude);
  out += ",lon:"; out += BYTE_SWAP_U32(stats->longitude);
  out += ",spe:"; out += BYTE_SWAP_U16(stats->speed);
  out += ",hea:"; out += BYTE_SWAP_U16(stats->heading) / 10; // convert to degrees
  out += ",alt:"; out += (int)BYTE_SWAP_U16(stats->altitude) - 1000; // 1000m offset
  out += ",sat:"; out += stats->satellites;
  websocket_send(out);
}


//#define ADC_VOLT(X) (((X) * ADC_R2) / (ADC_R1 + ADC_R2))
#define ADC_SCALE (ADC_R1 / ADC_R2)

#define ADC_REF_mV  (1075U * ADC_SCALE)
#define ADC_MAX     1023U
#define ADC_VOLT(X) (((uint32_t)(X) * ADC_REF_mV) / ADC_MAX)

static uint32_t batt_voltage;
static uint32_t batt_voltage_warning_limit;
static uint32_t batt_voltage_dead_limit;

static uint32_t batt_voltage_meas_last_ms;
static uint32_t batt_voltage_warning_last_ms;
static uint32_t batt_voltage_warning_timeout = 5000;
#ifdef WS2812_PIN
static uint8_t batt_voltage_last_bright;
#endif

void battery_voltage_report(int num = -1)
{
  String out = "ELRS_handset_battery=";
  out += batt_voltage;
  out += ",";
  out += eeprom_storage.batt_voltage_scale;
  out += ",";
  out += eeprom_storage.batt_voltage_warning;

  websocket_send(out, num);
}

void battery_voltage_parse(const char * input, int num = -1)
{
  const char * temp = strstr(input, ",");
  uint32_t scale = char_u8_to_hex(input);
  if (50 <= scale && scale <= 150) {
    eeprom_storage.batt_voltage_scale = scale;
    eeprom_storage.markDirty();
  }
  if (temp) {
    scale = char_u8_to_hex(&temp[1]);
    if (10 <= scale && scale <= 100) {
      eeprom_storage.batt_voltage_warning = scale;
      eeprom_storage.markDirty();

      batt_voltage_warning_limit = ((scale * BATT_NOMINAL_mV) / 100);
    }
  }
}

void batt_voltage_init(void)
{
  batt_voltage_warning_limit =
    ((eeprom_storage.batt_voltage_warning * BATT_NOMINAL_mV) / 100);
  batt_voltage_dead_limit =
    ((BATT_DEAD_DEFAULT * BATT_NOMINAL_mV) / 100);
}

void batt_voltage_measure(void)
{
  uint32_t ms = millis();
  if (eeprom_storage.batt_voltage_interval <= (uint32_t)(ms - batt_voltage_meas_last_ms)) {
    int adc = analogRead(A0);
    batt_voltage = ADC_VOLT(adc);
    batt_voltage = (batt_voltage * eeprom_storage.batt_voltage_scale) / 100;

    battery_voltage_report();

    uint32_t brightness = (batt_voltage * 255) / BATT_NOMINAL_mV;
    if (255 < brightness) {
      batt_voltage_warning_timeout = 5000;
      brightness = 255;
    } else if (batt_voltage <= batt_voltage_dead_limit) {
      batt_voltage_warning_timeout = 500;
      brightness = 0;
    } else if (batt_voltage <= batt_voltage_warning_limit) {
      batt_voltage_warning_timeout = 500 +
        5 * (int32_t)(batt_voltage - batt_voltage_warning_limit);
      if ((int32_t)batt_voltage_warning_timeout < 500) {
        batt_voltage_warning_timeout = 500;
      }
    }

    led_brightness_set(brightness);

    batt_voltage_meas_last_ms = ms;
  }

  if (batt_voltage && (batt_voltage <= batt_voltage_warning_limit) &&
      (batt_voltage_warning_timeout <= (ms - batt_voltage_warning_last_ms))) {
#ifdef WS2812_PIN
    if (!batt_voltage_last_bright) {
      batt_voltage_last_bright = led_brightness_get();
      led_brightness_set(0, 1);
    } else {
      led_brightness_set(batt_voltage_last_bright);
      led_set();
      batt_voltage_last_bright = 0;
    }
#endif
    // 400Hz, 20ms
    beep(400, 20, 0);

    batt_voltage_warning_last_ms = ms;
  }
}

#else
#define batt_voltage_init()
#define batt_voltage_measure()
#endif

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

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  char * temp;

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

    // Send settings
    SettingsGet(num);
#if CONFIG_HANDSET
    HandsetConfigGet(num);
    delay(5);
    battery_voltage_report(num);
#endif
    break;
  }
  case WStype_TEXT:

    temp = strstr((char*)payload, "stm32_cmd=");
    if (temp) {
      // Command STM32
      if (strstr((char*)&temp[10], "reset")) {
        // Reset STM32
        reset_stm32_to_app_mode();
      }
    } else {

#if CONFIG_HANDSET
      /* Handset specific */
      temp = strstr((char*)payload, "handset_");
      if (temp) {

        // handset_mixer=
        temp = strstr((char*)payload, "mixer=");
        if (temp) {
          temp = &temp[6];
          handleHandsetMixer(temp, (length - ((uintptr_t)temp - (uintptr_t)payload)));
          break;
        }

        // handset_calibrate=
        temp = strstr((char*)payload, "calibrate=");
        if (temp) {
          handleHandsetCalibrate(&temp[10]);
          break;
        }

        // handset_adjust_[axe]_[min|max]=val
        temp = strstr((char*)payload, "_adjust_");
        if (temp) {
          handleHandsetAdjust(&temp[8]);
          break;
        }

        temp = strstr((char*)payload, "_refresh");
        if (temp) {
          HandsetConfigGet(num, 1);
          break;
        }

        temp = strstr((char*)payload, "_save");
        if (temp) {
          HandsetConfigSave(num);
          break;
        }

        temp = strstr((char*)payload, "_battery_config=");
        if (temp) {
          battery_voltage_parse(&temp[16], num);
          break;
        }
      } else
#endif // CONFIG_HANDSET

      {
        // ExLRS setting commands
        temp = strstr((char*)payload, "S_rate");
        if (temp) {
          handleSettingRate(&temp[6], num);
          break;
        }
        temp = strstr((char*)payload, "S_power");
        if (temp) {
          handleSettingPower(&temp[7], num);
          break;
        }
        temp = strstr((char*)payload, "S_telemetry");
        if (temp) {
          handleSettingTlm(&temp[11], num);
          break;
        }
        temp = strstr((char*)payload, "S_vtx_freq");
        if (temp) {
          MspVtxWrite(&temp[10], num);
          break;
        }
        temp = strstr((char*)payload, "S_rf_pwr");
        if (temp) {
          handleSettingRfPwr(&temp[8], num);
          break;
        }
        temp = strstr((char*)payload, "S_rf_module");
        if (temp) {
          handleSettingRfModule(&temp[11], num);
          break;
        }
      }
    }
    break;
  case WStype_BIN:
    //Serial.printf("[%u] get binary length: %u\r\n", num, length);
    hexdump(payload, length);

    // echo data back to browser
    webSocket.sendBIN(num, payload, length);
    break;
  default:
    break;
  }
}


/***********************************************************************************/
/*************                    ESP OTA UPGRADE                      *************/
#define LOCAL_OTA 0
#if LOCAL_OTA
void handle_upgrade_error(void)
{
  server.sendHeader("Location", "/return");          // Redirect the client to the success page
  server.send(303);
    websocket_send(
      (Update.hasError()) ? "Update Failure!" : "Update Successful!");
  server.send(Update.hasError() ? 200 : 400);
}

void handleEspUpgrade(void)
{
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    //Serial.setDebugOutput(true);
    WiFiUDP::stopAll();
    //Serial.printf("Update: %s\n", upload.filename.c_str());
    websocket_send("*** ESP OTA ***\n  file: " + upload.filename);

    /*if (upload.name != "backpack_fw") {
      websocket_send("Invalid upload type!");
      return;
    } else*/ if (!upload.filename.startsWith("backpack.bin")) {
      websocket_send("Invalid file! Update aborted...");
      handle_upgrade_error();
      return;
    }

    uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    if (!Update.begin(maxSketchSpace, U_FLASH)) { //start with max available size
      //Update.printError(Serial);
      websocket_send("File is too big!");
      handle_upgrade_error();
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      //Update.printError(Serial);
      websocket_send("Junk write failed!");
      handle_upgrade_error();
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) { //true to set the size to the current progress
      //Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      websocket_send("Update Success! Rebooting...");
    } else {
      //Update.printError(Serial);
      websocket_send("Update failed!");
    }
    //Serial.setDebugOutput(false);
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.end();
  }
  delay(0); //same as yield();
}

void handleEspUpgradeFinalize(void)
{
  server.sendHeader("Location", "/return");          // Redirect the client to the success page
  server.send(303);
  websocket_send(
    (Update.hasError()) ? "Update Failure!" : "Update Successful!");
  server.send(Update.hasError() ? 200 : 400);
  //server.sendHeader("Connection", "close");
  //server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
  ESP.restart();
}
#endif // LOCAL_OTA

/***********************************************************************************/

void sendReturn()
{
  server.send_P(200, "text/html", GO_BACK);
}

void handle_recover()
{
  server.send_P(200, "text/html", INDEX_HTML);
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
      if(dir.fileSize()) {
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

WiFiEventHandler stationConnectedHandler;
WiFiEventHandler stationDisconnectedHandler;
WiFiEventHandler stationGotIpAddress;
WiFiEventHandler stationDhcpTimeout;
WiFiEventHandler AP_ConnectedHandler;
WiFiEventHandler AP_DisconnectedHandler;

static uint32_t wifi_connect_started;
static uint8_t wifi_sta_connected;

void onStationConnected(const WiFiEventStationModeConnected& evt) {
#if WIFI_DBG
  wifi_log += "Wifi_STA_connect();\n";
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
  wifi_sta_connected = WIFI_STATE_STA;
  mdns.setHostname("elrs_tx");
  if (mdns.begin("elrs_tx", WiFi.localIP())) {
    mdns.addService("http", "tcp", 80);
    mdns.addService("ws", "tcp", 81);
  }
  led_set(LED_WIFI_STA);
  /*beep(440, 30);
  delay(200);
  beep(440, 30);*/
}

void onStationDhcpTimeout(void)
{
#if WIFI_DBG
  wifi_log += "Wifi_STA_DhcpTimeout();\n";
#endif
  wifi_sta_connected = WIFI_STATE_NA;
  mdns.end();
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

void wifi_config(void)
{
  wifi_station_set_hostname("elrs_tx");

  wifi_sta_connected = WIFI_STATE_NA;
#if WIFI_DBG
  wifi_log = "";
#endif

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
#endif /* WIFI_MANAGER */

  ESP.wdtFeed();
  espnow_init(ESP_NOW_CHANNEL);
}

#define WIFI_LOOP_TIMEOUT (WIFI_TIMEOUT * 1000)
static uint32_t wifi_last_check, led_state;

void wifi_check(void)
{
  uint32_t const now = millis();
  if (WIFI_LOOP_TIMEOUT < (now - wifi_connect_started)) {
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
    wifi_sta_connected = WIFI_STATE_AP;
#if WIFI_DBG
    wifi_log += "WifiAP started\n";
#endif
    led_set(LED_WIFI_AP);
    beep(400, 20);
  } else if (500 <= (now - wifi_last_check)) {
    wifi_last_check = now;
    /* Blink led */
    led_set(led_state ? LED_WIFI_AP : LED_OFF);
    led_state ^= 1;
  }
}

void wifi_config_server(void)
{
  server.on("/fs", handle_fs);
  server.on("/return", sendReturn);
  server.on("/mac", handleMacAddress);
#if LOCAL_OTA
  server.on("/update_tst", HTTP_POST, // ESP OTA upgrade
    handleEspUpgradeFinalize, handleEspUpgrade);
#endif // LOCAL_OTA
  server.on("/upload", HTTP_POST, // STM32 OTA upgrade
    stm32_ota_handleFileUploadEnd, stm32_ota_handleFileUpload);
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

  msp_handler.markPacketFree();

  eeprom_storage.setup();

  /* Reset values */
  settings_rate = 1;
  settings_power = 4;
  settings_power_max = 8;
  settings_tlm = 7;
  settings_region = 255;
  settings_valid = 0;
#if CONFIG_HANDSET
  handset_num_switches = 6;
  handset_num_aux = 5;
  handset_mixer_ok = 0;
  handset_adjust_ok = 0;
  batt_voltage = 0;
  batt_voltage_warning_last_ms = millis();
#endif

#ifdef BUZZER_PIN
  pinMode(BUZZER_PIN, OUTPUT);
#endif

  //Serial.setRxBufferSize(256);
#ifdef INVERTED_SERIAL
  // inverted serial
  Serial.begin(SERIAL_BAUD, SERIAL_8N1, SERIAL_FULL, 1, true);
#else
  // non-inverted serial
  Serial.begin(SERIAL_BAUD);
#endif

  led_init();
  led_set(LED_INIT);
  batt_voltage_init();

#if (BOOT0_PIN == 2 || BOOT0_PIN == 0)
  reset_stm32_to_app_mode();
#endif

  FILESYSTEM.begin();
  //FILESYSTEM.format();

  wifi_config();
  wifi_config_server();

  // wsnum does not matter because settings_valid == false
  SettingsGet(0);
}


int serialEvent()
{
  int temp;
  uint8_t inChar;
  while (Serial.available()) {
    temp = Serial.read();
    if (temp < 0)
      break;

    inChar = (uint8_t)temp;
    if (msp_handler.processReceivedByte(inChar)) {
      uint8_t forward = true;
      String info = "MSP received: ";
      // msp fully received
      mspPacket_t &msp_in = msp_handler.getPacket();
      if (msp_in.type == MSP_PACKET_V1_ELRS) {
        uint8_t * payload = (uint8_t*)msp_in.payload;
        switch (msp_in.function) {
          case ELRS_INT_MSP_PARAMS: {
            info += "ELRS params";
            settings_rate = payload[0];
            settings_tlm = payload[1];
            settings_power = payload[2];
            settings_power_max = payload[3];
            settings_region = payload[4];
            settings_valid = 1;

#if 0
            if (settings_region != 255) {
              if (3 <= settings_region)
                led_set(LED_FREQ_2400);
              else
                led_set(LED_FREQ_900);
            }
#endif

            handleSettingDomain(NULL);
            handleSettingRate(NULL);
            handleSettingPower(NULL);
            handleSettingTlm(NULL);
            break;
          }
#if CONFIG_HANDSET
          case ELRS_HANDSET_CALIBRATE: {
            info += "CALIBRATE error";
            handleHandsetCalibrateResp(payload);
            break;
          }
          case ELRS_HANDSET_MIXER: {
            info += "MIXER config";
            handleHandsetMixerResp(payload);
            handset_mixer_ok = 1;
            break;
          }
          case ELRS_HANDSET_ADJUST: {
            info += "ADJUST config";
            handleHandsetAdjustResp(payload);
            handset_adjust_ok = 1;
            break;
          }
          case ELRS_HANDSET_TLM_LINK_STATS: {
            info = "";
            forward = false;
            handleHandsetTlmLnkStats(payload);
            break;
          }
          case ELRS_HANDSET_TLM_BATTERY: {
            info = "";
            forward = false;
            handleHandsetTlmBattery(payload);
            break;
          }
          case ELRS_HANDSET_TLM_GPS: {
            info = "";
            forward = false;
            handleHandsetTlmGps(payload);
            break;
          }
#endif /* CONFIG_HANDSET */
          default:
            info += "UNKNOWN";
            forward = false;
            break;
        };
      }

      if (info.length())
        websocket_send(info);
      if (forward)
        espnow_send_msp(msp_in);

      msp_handler.markPacketFree();
    } else if (!msp_handler.mspOngoing()) {
      if (inChar == '\r') {
        continue;
      } else if (inChar == '\n' || 64 <= inputString.length()) {
        return 0;
      }
      if (isprint(inChar))
        inputString += (char)inChar;
    }

    //if (msp_handler.error())
    //  msp_handler.markPacketFree();
  }
  return -1;
}


void loop()
{
  if (wifi_sta_connected == WIFI_STATE_NA)
    wifi_check();

  ESP.wdtFeed();

  if (0 <= serialEvent()) {
    websocket_send(inputString);
    inputString = "";
  }
  ESP.wdtFeed();

  server.handleClient();
  webSocket.loop();
  mdns.update();

  batt_voltage_measure();

  if (msp_send_save_requested_ms &&
      (MSP_SAVE_DELAY_MS <= (millis() - msp_send_save_requested_ms))) {
    /* Save send requested, send and clear the flag */
    msp_out.reset();
    msp_out.type = MSP_PACKET_V1_CMD;
    msp_out.flags = MSP_VERSION | MSP_STARTFLAG;
    msp_out.function = MSP_EEPROM_WRITE;
    msp_out.payloadSize = 0; // No params
    MSP::sendPacket(&msp_out, &my_ctrl_serial);

    msp_send_save_requested_ms = 0;
  }

  eeprom_storage.update();
}
