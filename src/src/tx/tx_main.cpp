#include <Arduino.h>
#include "utils.h"
#include "common.h"
#if RADIO_SX128x
#include "SX1280.h"
#else
#include "LoRa_SX127x.h"
#endif
#include "CRSF_TX.h"
#include "FHSS.h"
#include "targets.h"
#include "POWERMGNT.h"
#include "HwTimer.h"
#include "debug_elrs.h"
#include "rc_channels.h"
#include "LowPassFilter.h"
#include "msp.h"
#include <stdlib.h>

static uint8_t SetRFLinkRate(uint8_t rate, uint8_t init = 0);

//// CONSTANTS ////
#define RX_CONNECTION_LOST_TIMEOUT        1500U // After 1500ms of no TLM response consider that slave has lost connection
#define LQ_CALCULATE_INTERVAL             500u
#ifndef TLM_REPORT_INTERVAL
#define TLM_REPORT_INTERVAL               300u
#endif
#define TX_POWER_UPDATE_PERIOD            1500

///////////////////

/// define some libs to use ///
#if RADIO_SX128x
SX1280Driver Radio(RadioSpi, OTA_PACKET_SIZE);
#else
SX127xDriver Radio(RadioSpi, OTA_PACKET_SIZE);
#endif
CRSF_TX crsf(CrsfSerial);
POWERMGNT PowerMgmt(Radio);

volatile uint32_t DRAM_ATTR _rf_rxtx_counter;
static volatile uint8_t DMA_ATTR rx_buffer[OTA_PACKET_SIZE];
static volatile uint8_t DRAM_ATTR rx_buffer_handle;
static volatile uint8_t red_led_state;

static uint16_t DRAM_ATTR CRCCaesarCipher;

struct platform_config pl_config;

/////////// SYNC PACKET ////////
static uint32_t DRAM_ATTR SyncPacketNextSend;
static volatile uint32_t DRAM_ATTR sync_send_interval; // Default is send always

/////////// CONNECTION /////////
static uint32_t DRAM_ATTR LastPacketRecvMillis;
volatile connectionState_e DRAM_ATTR connectionState = STATE_disconnected;

//////////// TELEMETRY /////////
static volatile uint32_t DMA_ATTR expected_tlm_counter;
static uint32_t DMA_ATTR recv_tlm_counter;
static volatile uint32_t DRAM_ATTR tlm_check_ratio;
static volatile uint_fast8_t DRAM_ATTR TLMinterval;
static mspPacket_t msp_packet_tx;
static mspPacket_t msp_packet_rx;
static MSP msp_packet_parser;
static volatile uint_fast8_t DRAM_ATTR tlm_msp_send;
static uint32_t DRAM_ATTR TlmSentToRadioTime;
static LPF DRAM_ATTR LPF_dyn_tx_power(3);
static uint32_t DRAM_ATTR dyn_tx_updated;
//////////// LUA /////////

///////////////////////////////////////

void init_globals(void) {
    current_rate_config = RATE_DEFAULT;

    pl_config.key = 0;
    pl_config.mode = RATE_DEFAULT;
    pl_config.power = TX_POWER_DEFAULT;
    pl_config.tlm = TLM_RATIO_DEFAULT;

    connectionState = STATE_disconnected;
}

///////////////////////////////////////

static void ICACHE_RAM_ATTR ProcessTLMpacket(uint8_t *buff);
static void ICACHE_RAM_ATTR HandleTLM();

int8_t tx_tlm_change_interval(uint8_t value, uint8_t init = 0)
{
    uint32_t ratio = 0;
    if (value == TLM_RATIO_DEFAULT)
    {
        // Default requested
        value = ExpressLRS_currAirRate->TLMinterval;
    }
    else if (TLM_RATIO_MAX <= value)
    {
        DEBUG_PRINTLN("TLM: Invalid value! disable tlm");
        value = TLM_RATIO_NO_TLM;
    }

    if (value != TLMinterval || init)
    {
        if (TLM_RATIO_NO_TLM < value) {
            Radio.RXdoneCallback1 = ProcessTLMpacket;
            Radio.TXdoneCallback1 = HandleTLM;
            connectionState = STATE_disconnected;
            ratio = TLMratioEnumToValue(value);
            DEBUG_PRINT("TLM ratio ");
            DEBUG_PRINTLN(ratio);
            ratio -= 1;
        } else {
            Radio.RXdoneCallback1 = SXRadioDriver::rx_nullCallback;
            Radio.TXdoneCallback1 = SXRadioDriver::tx_nullCallback;
            // Set connected if telemetry is not used
            connectionState = STATE_connected;
            DEBUG_PRINTLN("TLM disabled");
        }
        TLMinterval = value;
        tlm_check_ratio = ratio;
        return 0;
    }
    return -1;
}

void tx_tlm_disable_enable(uint8_t enable)
{
    if (enable)
        tx_tlm_change_interval(TLM_RATIO_DEFAULT);
    else
        tx_tlm_change_interval(TLM_RATIO_NO_TLM);
}

int8_t tx_tlm_toggle(void)
{
    /* Toggle TLM between NO_TLM and DEFAULT */
    uint8_t tlm = (TLMinterval == TLM_RATIO_NO_TLM) ? TLM_RATIO_DEFAULT : TLM_RATIO_NO_TLM;
    tx_tlm_change_interval(tlm);
    return (TLMinterval != TLM_RATIO_NO_TLM);
}

///////////////////////////////////////

static void process_rx_buffer()
{
    uint32_t ms = millis();
    const uint16_t crc = CalcCRC16((uint8_t*)rx_buffer, OTA_PACKET_DATA, CRCCaesarCipher);
    const uint16_t crc_in = ((uint16_t)rx_buffer[OTA_PACKET_DATA] << 8) + rx_buffer[OTA_PACKET_DATA+1];
    uint8_t type = RcChannels_packetTypeGet(rx_buffer);

    if (crc_in != crc)
    {
        DEBUG_PRINT("!C");
        return;
    }

    //DEBUG_PRINT(" PROC_RX ");

    connectionState = STATE_connected;
    platform_connection_state(STATE_connected);
    platform_set_led(0);
    LastPacketRecvMillis = ms;
    recv_tlm_counter++;

    switch (type)
    {
        case DL_PACKET_TLM_MSP:
        {
            //DEBUG_PRINTLN("DL MSP junk");
            RcChannels_tlm_downlink_receive(rx_buffer, msp_packet_rx);
            break;
        }
        case DL_PACKET_TLM_LINK:
        {
            crsf.LinkStatisticsExtract(rx_buffer,
                                       Radio.LastPacketSNR,
                                       Radio.LastPacketRSSI);

            // Check RSSI and update TX power if needed
            int8_t rssi = LPF_dyn_tx_power.update((int8_t)crsf.LinkStatistics.uplink_RSSI_1);
            if (TX_POWER_UPDATE_PERIOD <= (ms - dyn_tx_updated)) {
                dyn_tx_updated = ms;
                if (-75 < rssi) {
                    PowerMgmt.decPower();
                } else if (-95 > rssi) {
                    PowerMgmt.incPower();
                }
            }
            break;
        }
        case DL_PACKET_GPS:
        {
            crsf.GpsStatsExtract(rx_buffer);
            break;
        }
        case DL_PACKET_FREE1:
        default:
            break;
    }
}

static void ICACHE_RAM_ATTR ProcessTLMpacket(uint8_t *buff)
{
    volatile_memcpy(rx_buffer, buff, sizeof(rx_buffer));
    rx_buffer_handle = 1;

    //DEBUG_PRINT(" R ");
}

static void ICACHE_RAM_ATTR HandleTLM()
{
    //DEBUG_PRINT("X ");
    if (tlm_check_ratio && (_rf_rxtx_counter & tlm_check_ratio) == 0)
    {
        // receive tlm package
        PowerMgmt.pa_off();
        Radio.RXnb(FHSSgetCurrFreq());
        expected_tlm_counter++;
        //DEBUG_PRINT(" RX ");
    }
}

///////////////////////////////////////

static void ICACHE_RAM_ATTR HandleFHSS_TX()
{
    // Called from HW ISR context
    if ((_rf_rxtx_counter % ExpressLRS_currAirRate->FHSShopInterval) == 0)
    {
        // it is time to hop
        FHSSincCurrIndex();
    }
    _rf_rxtx_counter++;
}

static void ICACHE_RAM_ATTR GenerateSyncPacketData(uint8_t *const output)
{
    ElrsSyncPacket_s * sync = (ElrsSyncPacket_s*)output;
    sync->fhssIndex = FHSSgetCurrIndex();
    sync->rxtx_counter = _rf_rxtx_counter;
#if RX_UPDATE_AIR_RATE
    sync->air_rate = current_rate_config;
#endif
    sync->tlm_interval = TLMinterval;
    sync->CRCCaesarCipher = CRCCaesarCipher;
    sync->pkt_type = UL_PACKET_SYNC;
}

static void ICACHE_RAM_ATTR SendRCdataToRF(uint32_t current_us)
{
    // Called by HW timer
    uint32_t freq;
    // esp requires word aligned buffer
    uint32_t __tx_buffer[(OTA_PACKET_SIZE + sizeof(uint32_t) - 1) / sizeof(uint32_t)];
    uint8_t *tx_buffer = (uint8_t *)__tx_buffer;
    uint8_t index = OTA_PACKET_DATA;

    crsf.UpdateOpenTxSyncOffset(current_us); // tells the crsf that we want to send data now - this allows opentx packet syncing

    // Check if telemetry RX ongoing
    if (tlm_check_ratio && (_rf_rxtx_counter & tlm_check_ratio) == 0)
    {
        // Skip TX because TLM RX is ongoing
        HandleFHSS_TX();
        return;
    }

    freq = FHSSgetCurrFreq();

    //only send sync when its time and only on channel 0;
    if ((FHSSgetCurrSequenceIndex() == 0) &&
        (sync_send_interval <= (uint32_t)(current_us - SyncPacketNextSend)))
    {
        GenerateSyncPacketData(tx_buffer);
        SyncPacketNextSend = current_us;
    }
    else if ((tlm_msp_send == 1) && (msp_packet_tx.type == MSP_PACKET_TLM_OTA))
    {
        /* send tlm packet if needed */
        if (RcChannels_tlm_ota_send(tx_buffer, msp_packet_tx) || msp_packet_tx.error) {
            DEBUG_PRINT("<< MSP DONE ");
            DEBUG_PRINTLN(msp_packet_tx.error);
            msp_packet_tx.reset();
            tlm_msp_send = 0;
        } else {
            DEBUG_PRINTLN("<< MSP junk sent");
        }
#if 0
        DEBUG_PRINT(" MSP: >");
        for (uint8_t iter = 0; iter < sizeof(__tx_buffer); iter++) {
            DEBUG_PRINT(" 0x");
            DEBUG_PRINT(tx_buffer[iter], HEX);
        }
        DEBUG_PRINTLN(" <");
#endif
    }
    else
    {
        RcChannels_get_packed_data(tx_buffer);
    }

#if OTA_PACKET_10B
    tx_buffer[index++] = FHSSgetCurrIndex();
    tx_buffer[index++] = _rf_rxtx_counter;
#endif // OTA_PACKET_10B

    // Calculate the CRC
    uint16_t crc = CalcCRC16(tx_buffer, index, CRCCaesarCipher);
    tx_buffer[index++] = (crc >> 8);
    tx_buffer[index++] = (crc & 0xFF);
    // Enable PA
    PowerMgmt.pa_on();
    // Debugging
    //delayMicroseconds(random(0, 400)); // 300 ok
    //if (random(0, 99) < 55) tx_buffer[1] = 0;
    // Send data to rf
    Radio.TXnb(tx_buffer, index, freq);
    // Increase TX counter
    HandleFHSS_TX();
    //DEBUG_PRINT(" T");
}

///////////////////////////////////////

static int8_t SettingsCommandHandle(uint8_t const cmd, uint8_t const len, uint8_t const *msg, uint8_t *out)
{
    uint8_t modified = 0;
    uint8_t value = msg[0];

    switch (cmd)
    {
        case 0: // send all params
            break;

        case 1:
            if (len != 1)
                return -1;

            // set air rate
            if (get_elrs_airRateMax() > value)
                modified |= SetRFLinkRate(value) ? (1 << 1) : 0;
            DEBUG_PRINT("Rate: ");
            //DEBUG_PRINTLN(get_elrs_airRateIndex((void*)ExpressLRS_currAirRate));
            //DEBUG_PRINTLN(current_rate_config);
            DEBUG_PRINTLN(ExpressLRS_currAirRate->rate);
            break;

        case 2:
            // set TLM interval
            if (len != 1)
                return -1;

            if (tx_tlm_change_interval(value) >= 0)
            {
                modified = (1 << 2);
            }
            break;

        case 3:
            // set TX power
            if (len != 1)
                return -1;
            modified = PowerMgmt.currPower();
            PowerMgmt.setPower((PowerLevels_e)value);
            DEBUG_PRINT("Power: ");
            DEBUG_PRINTLN(PowerMgmt.currPower());
            modified = (modified != PowerMgmt.currPower()) ? (1 << 3) : 0;
            break;

        case 4:
            // RFFreq
            break;

        case 5:
            // Start WiFi
            platform_wifi_start();
            break;

        default:
            return -1;
    }

    // Fill response
    if (out) {
        //out[0] = get_elrs_airRateIndex((void*)ExpressLRS_currAirRate);
        out[0] = (uint8_t)current_rate_config;
        out[1] = (uint8_t)TLMinterval;
        out[2] = (uint8_t)PowerMgmt.currPower();
        out[3] = (uint8_t)PowerMgmt.maxPowerGet();
#if defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_FCC_915)
        out[4] = 0;
#elif defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_EU_868_R9)
        out[4] = 1;
#elif defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
        out[4] = 2;
#elif defined(Regulatory_Domain_ISM_2400_800kHz)
        out[4] = 3;
#elif defined(Regulatory_Domain_ISM_2400)
        out[4] = 4;
#else
        out[4] = 0xff;
#endif
    }

    if (modified)
    {
        // Stop timer before save if not already done
        if ((modified & (1 << 1)) == 0) {
            TxTimer.stop();
            Radio.StopContRX();
        }

        // Save modified values
        pl_config.key = ELRS_EEPROM_KEY;
        pl_config.mode = current_rate_config;
        //pl_config.mode = get_elrs_airRateIndex((void*)ExpressLRS_currAirRate);
        pl_config.power = PowerMgmt.currPower();
        pl_config.tlm = TLMinterval;
        platform_config_save(pl_config);

        // and restart timer
        TxTimer.start();
    }

    return 0;
}

static void ParamWriteHandler(uint8_t const *msg, uint16_t len)
{
    // Called from UART handling loop (main loop)
    uint8_t resp[5];
    if (0 > SettingsCommandHandle(msg[0], (len - 1), &msg[1], resp))
        return;
    crsf.sendLUAresponseToRadio(resp, sizeof(resp));

#ifdef CTRL_SERIAL
    msp_packet_parser.sendPacket(
        &ctrl_serial, MSP_PACKET_V1_ELRS, ELRS_INT_MSP_PARAMS, MSP_ELRS_INT, sizeof(resp), resp);
#endif /* CTRL_SERIAL */
}

///////////////////////////////////////

static uint8_t SetRFLinkRate(uint8_t rate, uint8_t init) // Set speed of RF link (hz)
{
    const expresslrs_mod_settings_t *const config = get_elrs_airRateConfig(rate);
    if (config == NULL || config == ExpressLRS_currAirRate)
        return 0; // No need to modify, rate is same

    if (!init)
    {
        // Stop timer and put radio into sleep
        TxTimer.stop();
        Radio.StopContRX();
    }

    LPF_dyn_tx_power.init(-55);

    current_rate_config = rate;
    ExpressLRS_currAirRate = config;
    TxTimer.updateInterval(config->interval); // TODO: Make sure this is equiv to above commented lines

    FHSSsetCurrIndex(0);
    Radio.Config(config->bw, config->sf, config->cr, GetInitialFreq(),
                 config->PreambleLen, (OTA_PACKET_CRC == 0));
    crsf.setRcPacketRate(config->interval);
    crsf.LinkStatistics.rf_Mode = config->rate_osd_num;

    tx_tlm_change_interval(TLMinterval, init);

    sync_send_interval = config->syncInterval;

    platform_connection_state(connectionState);

    return 1;
}

static void hw_timer_init(void)
{
    red_led_state = 1;
    platform_set_led(1);
    TxTimer.init();
    TxTimer.start();
}
static void hw_timer_stop(void)
{
    red_led_state = 0;
    platform_set_led(0);
    TxTimer.stop();
}

static void rc_data_cb(crsf_channels_t const *const channels)
{
    RcChannels_processChannels(channels);
}

/* OpenTX sends v1 MSPs */
static void msp_data_cb(uint8_t const *const input)
{
     /* process MSP packet from radio
     *  [0] header: seq&0xF,
     *  [1] payload size
     *  [2] function
     *  [3...] payload + crc
     */
    mspHeaderV1_t *hdr = (mspHeaderV1_t *)input;
    uint8_t payloadSize = hdr->payloadSize + 1U; // include size

   DEBUG_PRINT("MSP from radio: ");

    if (tlm_msp_send) {
        DEBUG_PRINTLN(" msp packet reserved, ignore");
        return;
    } else if (sizeof(msp_packet_tx.payload) < payloadSize) {
        /* too big, ignore */
        DEBUG_PRINTLN(" too big, ignore!");
        return;
    }

    // BF: MSP from radio: size: 0 flags: 48 func: 88 Lsize: 0 Lflags: 48 Lfunc: 88 received

    DEBUG_PRINT("size: ");
    DEBUG_PRINT(hdr->payloadSize);
    DEBUG_PRINT(" flags: ");
    DEBUG_PRINT(hdr->flags);
    DEBUG_PRINT(" func: ");
    DEBUG_PRINT(hdr->function);
    DEBUG_PRINTLN(" >>");

    msp_packet_tx.reset(hdr);
    msp_packet_tx.type = MSP_PACKET_TLM_OTA;
    msp_packet_tx.payloadSize = payloadSize;

    volatile_memcpy(msp_packet_tx.payload,
                    (void*)&input[1], // skip flags
                    payloadSize);

    tlm_msp_send = 1; // rdy for sending
}

#ifdef CTRL_SERIAL
static void MspOtaCommandsSend(mspPacket_t &packet)
{
    uint8_t iter;

    DEBUG_PRINT("CTRL_SERIAL MSP: ");

    if (tlm_msp_send) {
        DEBUG_PRINTLN(" msp packet reserved, ignore");
        return;
    }

    DEBUG_PRINT("size: ");
    DEBUG_PRINT(packet.payloadSize);
    DEBUG_PRINT(" flags: ");
    DEBUG_PRINT(packet.flags, HEX);
    DEBUG_PRINT(" func: ");
    DEBUG_PRINT(packet.function);
    DEBUG_PRINTLN(" >>");

    msp_packet_tx.reset();
    msp_packet_tx.type = MSP_PACKET_TLM_OTA;
    msp_packet_tx.flags = packet.flags;
    msp_packet_tx.function = packet.function;
    // Convert to CRSF encapsulated MSP message
    msp_packet_tx.addByte(packet.payloadSize);
    msp_packet_tx.addByte(packet.function);
    for (iter = 0; iter < packet.payloadSize; iter++) {
        msp_packet_tx.addByte(packet.payload[iter]);
    }
    msp_packet_tx.addByte(msp_packet_tx.crc);
    msp_packet_tx.setIteratorToSize();

    tlm_msp_send = 1; // rdy for sending
}
#endif

void platform_radio_force_stop(void)
{
    TxTimer.stop();
    Radio.End();
}

void setup()
{
    PowerLevels_e power;

    init_globals();

    msp_packet_tx.reset();
    msp_packet_rx.reset();

    CRCCaesarCipher = CalcCRC16(UID, sizeof(UID), 0);

    CrsfSerial.Begin(CRSF_TX_BAUDRATE_FAST);

    platform_setup();
    DEBUG_PRINTLN("ExpressLRS TX Module...");
    platform_config_load(pl_config);
    current_rate_config = pl_config.mode % get_elrs_airRateMax();
    power = (PowerLevels_e)(pl_config.power % PWR_UNKNOWN);
    TLMinterval = pl_config.tlm;
    platform_mode_notify(get_elrs_airRateMax() - current_rate_config);

    crsf.connected = hw_timer_init; // it will auto init when it detects UART connection
    crsf.disconnected = hw_timer_stop;
    crsf.ParamWriteCallback = ParamWriteHandler;
    crsf.RCdataCallback1 = rc_data_cb;
    crsf.MspCallback = msp_data_cb;

    TxTimer.callbackTock = &SendRCdataToRF;

    //FHSSrandomiseFHSSsequence();

#if defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
    Radio.RFmodule = RFMOD_SX1278;
#elif !RADIO_SX128x
    Radio.RFmodule = RFMOD_SX1276;
#endif
    Radio.SetPins(GPIO_PIN_RST, GPIO_PIN_DIO0, GPIO_PIN_DIO1, GPIO_PIN_DIO2,
                  GPIO_PIN_BUSY, GPIO_PIN_TX_ENABLE, GPIO_PIN_RX_ENABLE);
    Radio.SetSyncWord(getSyncWord());
    Radio.Begin();

    PowerMgmt.Begin();
    PowerMgmt.setPower(power);

    SetRFLinkRate(current_rate_config, 1);

#ifdef CTRL_SERIAL
    uint8_t resp[5];
    if (0 == SettingsCommandHandle(0, 0, resp, resp))
        msp_packet_parser.sendPacket(
            &ctrl_serial, MSP_PACKET_V1_ELRS,
            ELRS_INT_MSP_PARAMS, MSP_ELRS_INT, sizeof(resp), resp);
#endif /* CTRL_SERIAL */

    crsf.Begin();
}

void loop()
{
    uint8_t can_send;

    if (rx_buffer_handle)
    {
        process_rx_buffer();
        rx_buffer_handle = 0;
        platform_wd_feed();
    }

    if (TLM_RATIO_NO_TLM < TLMinterval)
    {
        uint32_t current_ms = millis();

        if (STATE_lost < connectionState &&
            RX_CONNECTION_LOST_TIMEOUT < (uint32_t)(current_ms - LastPacketRecvMillis))
        {
            connectionState = STATE_disconnected;
            platform_connection_state(STATE_disconnected);
            platform_set_led(red_led_state);
        }
        else if (connectionState == STATE_connected &&
                 TLM_REPORT_INTERVAL <= (uint32_t)(current_ms - TlmSentToRadioTime))
        {
            TlmSentToRadioTime = current_ms;

            // Calc LQ based on good tlm packets and receptions done
            uint8_t rx_cnt = expected_tlm_counter;
            uint32_t tlm_cnt = recv_tlm_counter;
            expected_tlm_counter = recv_tlm_counter = 0; // Clear RX counter

            if (rx_cnt)
                crsf.LinkStatistics.downlink_Link_quality = (tlm_cnt * 100u) / rx_cnt;
            else
                // failure value??
                crsf.LinkStatistics.downlink_Link_quality = 0;

            crsf.LinkStatistics.uplink_TX_Power = PowerMgmt.power_to_radio_enum();
            crsf.LinkStatisticsSend();
            crsf.BatterySensorSend();
            crsf.GpsSensorSend();
        }
    }

    // Process CRSF packets from TX
    can_send = crsf.handleUartIn(rx_buffer_handle);

    if (!rx_buffer_handle) {
        // Send MSP resp if allowed and packet ready
        if (can_send && msp_packet_rx.iterated())
        {
            DEBUG_PRINTLN("DL MSP rcvd => radio");
            if (!msp_packet_rx.error)
                crsf.sendMspPacketToRadio(msp_packet_rx);
            msp_packet_rx.reset();
        }
#ifdef CTRL_SERIAL
        else if (ctrl_serial.available()) {
            platform_wd_feed();
            uint8_t in = ctrl_serial.read();
            if (msp_packet_parser.processReceivedByte(in)) {
                //  MSP received, check content
                mspPacket_t &packet = msp_packet_parser.getPacket();

                //DEBUG_PRINT("MSP rcvd, type:");
                //DEBUG_PRINTLN(packet.type);

                /* Check if packet is ELRS internal */
                if ((packet.type == MSP_PACKET_V1_ELRS) && (packet.flags & MSP_ELRS_INT)) {
                    switch (packet.function) {
                        case ELRS_INT_MSP_PARAMS: {
                            uint8_t * msg = (uint8_t*)packet.payload;
                            if (0 == SettingsCommandHandle(msg[0], (packet.payloadSize - 1), &msg[1], msg)) {
                                //packet.type = MSP_PACKET_V1_ELRS;
                                packet.payloadSize = 5;
                                msp_packet_parser.sendPacket(&packet, &ctrl_serial);
                            }
                            break;
                        }
                        default:
                            break;
                    };
                } else {
                    MspOtaCommandsSend(packet);
                }
                msp_packet_parser.markPacketFree();
            }
        }
#endif /* CTRL_SERIAL */
    }

    platform_loop(connectionState);
    platform_wd_feed();
}
