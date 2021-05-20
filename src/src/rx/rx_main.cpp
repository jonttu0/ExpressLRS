#include <Arduino.h>
#include "targets.h"
#include "utils.h"
#include "common.h"
#include "LowPassFilter.h"
#if RX_GHST_ENABLED
#include "GHST.h"
#define RX_CLASS GHST
#define RX_BAUDRATE GHST_RX_BAUDRATE
#else
#include "CRSF_RX.h"
#define RX_CLASS CRSF_RX
#define RX_BAUDRATE CRSF_RX_BAUDRATE
#endif
#include "FHSS.h"
#include "rx_LinkQuality.h"
#include "HwTimer.h"
#include "HwSerial.h"
#include "debug_elrs.h"
#include "helpers.h"
#include "rc_channels.h"
#include "rx_servo_out.h"

static void SetRFLinkRate(uint8_t rate);
void FAST_CODE_1 LostConnection();

//// CONSTANTS ////
#define SEND_LINK_STATS_TO_FC_INTERVAL 100
#define PRINT_FREQ_ERROR               0
//#define NUM_FAILS_TO_RESYNC            100
#define PRINT_RATE 1
#define PRINT_TIMING 0

#if PRINT_RATE && NO_DATA_TO_FC
uint32_t print_rate_cnt;
uint32_t print_rate_cnt_fail;
uint32_t print_Rate_cnt_time;
#endif
#if PRINT_TIMING
static uint32_t DRAM_ATTR print_rx_isr_end_time;
#endif

///////////////////

RX_CLASS DRAM_FORCE_ATTR crsf(CrsfSerial); //pass a serial port object to the class for it to use

connectionState_e DRAM_ATTR connectionState;
static volatile uint8_t DRAM_ATTR NonceRXlocal; // nonce that we THINK we are up to.
static uint8_t DRAM_ATTR TLMinterval;
static volatile uint32_t DRAM_ATTR tlm_check_ratio;
static volatile uint32_t DRAM_ATTR rx_last_valid_us; //Time the last valid packet was recv
static volatile int32_t DRAM_ATTR rx_freqerror;
#if NUM_FAILS_TO_RESYNC
static uint32_t rx_lost_packages;
#endif
static volatile int32_t DRAM_ATTR rx_hw_isr_running;

static uint16_t DRAM_ATTR CRCCaesarCipher;

#if SERVO_OUTPUTS_ENABLED
#if !SERVO_WRITE_FROM_ISR
static uint8_t DRAM_ATTR update_servos;
#endif
#endif
static rc_channels_rx_t DRAM_ATTR CrsfChannels;
static LinkStats_t DRAM_ATTR LinkStatistics;
static GpsOta_t DRAM_ATTR GpsTlm;

///////////////////////////////////////////////
////////////////  Filters  ////////////////////
static LPF DRAM_FORCE_ATTR LPF_FreqError(5);
static LPF DRAM_FORCE_ATTR LPF_UplinkRSSI(5);
static LPF DRAM_FORCE_ATTR LPF_UplinkSNR(5);

//////////////////////////////////////////////////////////////
////// Variables for Telemetry and Link Quality //////////////

static uint32_t DRAM_ATTR LastValidPacket; //Time the last valid packet was recv
#if !SERVO_OUTPUTS_ENABLED
static uint32_t DRAM_ATTR SendLinkStatstoFCintervalNextSend;
#endif
static mspPacket_t DRAM_FORCE_ATTR msp_packet_rx;
static uint32_t DRAM_ATTR msp_packet_rx_sent;
static mspPacket_t DRAM_FORCE_ATTR msp_packet_tx;
static uint8_t DRAM_ATTR tlm_msp_send, tlm_msp_rcvd;
static uint8_t DRAM_ATTR uplink_Link_quality;

///////////////////////////////////////////////////////////////
///////////// Variables for Sync Behaviour ////////////////////
static uint32_t DRAM_ATTR RFmodeNextCycle;
static uint8_t DRAM_ATTR scanIndex;
static uint8_t DRAM_ATTR tentative_cnt;
static uint8_t DRAM_ATTR no_sync_armed;

///////////////////////////////////////
#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
struct gpio_out dbg_pin_tmr;
#endif
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
struct gpio_out dbg_pin_rx;
#endif

///////////////////////////////////////

static bool DRAM_ATTR ledState;
inline void led_set_state(bool state)
{
    ledState = state;
    platform_set_led(state);
}

inline void led_toggle(void)
{
    led_set_state(!ledState);
}

static void FAST_CODE_1 handle_tlm_ratio(uint8_t interval)
{
    if (TLMinterval == interval) return;

    if ((TLM_RATIO_NO_TLM < interval) && (TLM_RATIO_MAX > interval))
    {
        tlm_check_ratio = TLMratioEnumToValue(interval) - 1;
    }
    else
    {
        tlm_check_ratio = 0;
    }
    TLMinterval = interval;
    DEBUG_PRINTF("TLM: %u\n", interval);
}
///////////////////////////////////////

void FAST_CODE_1 FillLinkStats()
{
    int32_t rssiDBM = Radio->LastPacketRSSI;
    rssiDBM = LPF_UplinkRSSI.update(rssiDBM);
    // our rssiDBM is currently in the range -128 to 98, but BF wants a value in the range
    // 0 to 255 that maps to -1 * the negative part of the rssiDBM, so cap at 0.
    if (0 < rssiDBM) rssiDBM = 0;
    else if (rssiDBM < INT8_MIN) rssiDBM = INT8_MIN;
    //CrsfChannels.ch15 = UINT10_to_CRSF(MAP(rssiDBM, -100, -50, 0, 1023));
    //CrsfChannels.ch14 = UINT10_to_CRSF(MAP_U16(LinkStatistics.link.uplink_Link_quality, 0, 100, 0, 1023));
    LinkStatistics.link.uplink_RSSI_1 = -1 * rssiDBM; // to match BF
    LinkStatistics.link.uplink_SNR = LPF_UplinkSNR.update(Radio->LastPacketSNR * 10);
}

uint8_t FAST_CODE_1 RadioFreqErrorCorr(void)
{
    // Do freq correction before FHSS
    /* Freq tunin ~84us */
    int32_t freqerror = rx_freqerror;
    uint8_t retval = 0;
    rx_freqerror = 0;
    if (!freqerror)
        return 0;

#if PRINT_FREQ_ERROR
    DEBUG_PRINTF(" > freqerror:%u", freqerror);
#endif

#define MAX_ERROR 0 //3000
#if MAX_ERROR
    if (freqerror < -MAX_ERROR)
        freqerror = -MAX_ERROR;
    else if (freqerror > MAX_ERROR)
        freqerror = MAX_ERROR;
#endif
    freqerror = LPF_FreqError.update(freqerror);

#if PRINT_FREQ_ERROR
    DEBUG_PRINTF(" smooth:%u", freqerror);
#endif

    if (abs(freqerror) > 100) // 120
    {
        FHSSfreqCorrectionSet(freqerror);
        Radio->setPPMoffsetReg(freqerror, 0);
        retval = 1;
    }
#if PRINT_FREQ_ERROR
    //extern int_fast32_t FreqCorrection;
    //DEBUG_PRINTF(" local:%u", FreqCorrection);
#endif

    return retval;
}

uint8_t FAST_CODE_1 HandleFHSS(uint_fast8_t & nonce)
{
    uint8_t fhss = ((nonce % ExpressLRS_currAirRate->FHSShopInterval) == 0);
    if (fhss) {
        FHSSincCurrIndex();
    }
    ++nonce;
    return fhss;
}

void FAST_CODE_1 HandleSendTelemetryResponse(uint_fast8_t lq) // total ~79us
{
    DEBUG_PRINTF(" X");
    // esp requires word aligned buffer
    uint32_t __tx_buffer[(OTA_PACKET_SIZE + sizeof(uint32_t) - 1) / sizeof(uint32_t)];
    uint8_t *tx_buffer = (uint8_t *)__tx_buffer;
    uint8_t index = OTA_PACKET_DATA;

    if ((tlm_msp_send == 1) && (msp_packet_tx.type == MSP_PACKET_TLM_OTA))
    {
        if (RcChannels_tlm_ota_send(tx_buffer, msp_packet_tx, 0) || msp_packet_tx.error) {
            msp_packet_tx.reset();
            tlm_msp_send = 0;
        }
    }
    else if (RcChannels_gps_pack(tx_buffer, GpsTlm))
    {
        RcChannels_packetTypeSet(tx_buffer, DL_PACKET_GPS);
    }
    else
    {
        RcChannels_link_stas_pack(tx_buffer, LinkStatistics, lq);
        RcChannels_packetTypeSet(tx_buffer, DL_PACKET_TLM_LINK);
    }

    uint16_t crc = CalcCRC16(tx_buffer, index, CRCCaesarCipher);
    tx_buffer[index++] = (crc >> 8);
    tx_buffer[index++] = (crc & 0xFF);
    Radio->TXnb(tx_buffer, index, FHSSgetCurrFreq());

    // Adds packet to LQ otherwise an artificial drop in LQ is seen due to sending TLM.
    LQ_packetAck();
}

void tx_done_cb(void)
{
    // Configure RX only next is not hopping time
    //if (((NonceRXlocal + 1) % ExpressLRS_currAirRate->FHSShopInterval) != 0)
    //    Radio->RXnb(FHSSgetCurrFreq());
}

void FAST_CODE_1 HWtimerCallback(uint32_t const us)
{
    //DEBUG_PRINTF("H");
#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_tmr, 1);
#endif

    rx_hw_isr_running = 1;

    int32_t diff_us = 0;
    const uint32_t last_rx_us = rx_last_valid_us;
    const uint_fast8_t tlm_ratio = tlm_check_ratio;
    uint_fast8_t fhss_config_rx = 0;
    uint_fast8_t nonce = NonceRXlocal;
    rx_last_valid_us = 0;

#if PRINT_TIMING
    uint32_t freq_now = FHSSgetCurrFreq();
#endif

    /* do adjustment */
    if (last_rx_us != 0)
    {
#if 1 //!USE_TIMER_KICK
        diff_us = (int32_t)((uint32_t)(us - last_rx_us));

#if 0
        int32_t const interval = ExpressLRS_currAirRate->interval;
        if (diff_us > (interval >> 1))
        {
            /* the timer was called too early */
            diff_us %= interval;
            diff_us -= interval;
        }
#else
        if (diff_us < -TIMER_OFFSET) diff_us = -TIMER_OFFSET;
        else if (diff_us > TIMER_OFFSET) diff_us = TIMER_OFFSET;
#endif
        /* Adjust the timer */
        //if ((TIMER_OFFSET_LIMIT < diff_us) || (diff_us < 0))
            TxTimer.reset(diff_us - TIMER_OFFSET);
#else // USE_TIMER_KICK
        TxTimer.reset(-TIMER_OFFSET_KICK);
#endif // USE_TIMER_KICK
#if PRINT_RATE && NO_DATA_TO_FC
        print_rate_cnt++;
#endif
    }
    else
    {
#if PRINT_RATE && NO_DATA_TO_FC
        print_rate_cnt_fail++;
#endif
#if !USE_TIMER_KICK
        TxTimer.setTime(); // Reset timer interval
#else
        TxTimer.reset(0); // Reset timer interval
#endif
    }

    fhss_config_rx |= RadioFreqErrorCorr();
    fhss_config_rx |= HandleFHSS(nonce);

    uint_fast8_t lq = LQ_getlinkQuality();
    uplink_Link_quality = lq;
    LQ_nextPacket();

    if ((0 < tlm_ratio) && ((nonce & tlm_ratio) == 0))
    {
#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
        gpio_out_write(dbg_pin_tmr, 0);
#endif
        HandleSendTelemetryResponse(lq);
#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
        gpio_out_write(dbg_pin_tmr, 1);
#endif
        goto hw_tmr_isr_exit;
    }
#if NUM_FAILS_TO_RESYNC
    else if (!last_rx_us)
    {
        if (NUM_FAILS_TO_RESYNC < (++rx_lost_packages)) {
            // consecutive losts => trigger connection lost to resync
            LostConnection();
            DEBUG_PRINTF("RESYNC!");
            goto hw_tmr_isr_exit;
        }
    }
#endif

    /* Configure next reception if needed */
    if (fhss_config_rx) {
        Radio->RXnb(FHSSgetCurrFreq());
    }

hw_tmr_isr_exit:
    NonceRXlocal = nonce;

#if (PRINT_TIMING)
    uint32_t now = micros();
    DEBUG_PRINTF("RX:%u (t:%u) HW:%u diff:%d (t:%u) [f %u]\n",
                 last_rx_us, (print_rx_isr_end_time - last_rx_us),
                 us, diff_us, (uint32_t)(now - us), freq_now);
#endif

    rx_hw_isr_running = 0;

#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_tmr, 0);
#endif
    return;
}

void FAST_CODE_1 LostConnection()
{
    if (connectionState <= STATE_lost)
    {
        return; // Already disconnected
    }

#if SERVO_OUTPUTS_ENABLED
    servo_out_fail_safe();
#endif
    no_sync_armed = 0;

    uint32_t irq = _SAVE_IRQ();
    TxTimer.stop(); // Stop sync timer
    _RESTORE_IRQ(irq);

    // Reset FHSS
    FHSSfreqCorrectionReset();
    FHSSsetCurrIndex(0);
    LPF_FreqError.init(0);

    connectionState = STATE_lost; //set lost connection

    led_set_state(0);             // turn off led
    Radio->RXnb(GetInitialFreq()); // in conn lost state we always want to listen on freq index 0
    DEBUG_PRINTF("lost conn\n");

    platform_connection_state(connectionState);
}

void FAST_CODE_1 TentativeConnection(int32_t freqerror)
{
    /* Do initial freq correction */
    FHSSfreqCorrectionSet(freqerror);
    Radio->setPPMoffsetReg(freqerror, 0);
    LPF_FreqError.init(freqerror);
    rx_last_valid_us = 0;

    tentative_cnt = 0;
    connectionState = STATE_tentative;
    DEBUG_PRINTF("tentative\n");
    TxTimer.start();     // Start local sync timer
#if !USE_TIMER_KICK
    TxTimer.setTime(80); // Trigger isr right after reception
#endif
    led_set_state(1); // turn on led
}

void FAST_CODE_1 GotConnection()
{
    connectionState = STATE_connected; //we got a packet, therefore no lost connection

    led_set_state(1); // turn on led
    DEBUG_PRINTF("connected\n");

    platform_connection_state(connectionState);
}

void FAST_CODE_1 ProcessRFPacketCallback(uint8_t *rx_buffer, const uint32_t current_us)
{
    /* Processing takes:
        R9MM: ~160us
    */
    if (rx_hw_isr_running)
        // Skip if hw isr is triggered already (e.g. TX has some weird latency)
        return;

    /* Error in reception (CRC etc), kick hw timer */
    if (!rx_buffer) {
        DEBUG_PRINTF("_");
        return;
    }

#if (DBG_PIN_RX_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_rx, 1);
#endif

    //DEBUG_PRINTF("I");
    int32_t freq_err;
    const connectionState_e _conn_state = connectionState;
    const uint16_t crc = CalcCRC16(rx_buffer, OTA_PACKET_PAYLOAD, CRCCaesarCipher);
    const uint16_t crc_in = ((uint16_t)rx_buffer[OTA_PACKET_PAYLOAD] << 8) + rx_buffer[OTA_PACKET_PAYLOAD+1];
    const uint8_t type = RcChannels_packetTypeGet(rx_buffer);

    if (crc_in != crc)
    {
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
        gpio_out_write(dbg_pin_rx, 0);
#endif
        DEBUG_PRINTF("!");
        return;
    }

    freq_err = Radio->GetFrequencyError();
    //DEBUG_PRINTF("E%d ", freq_err);

    rx_last_valid_us = current_us;
    LastValidPacket = millis();

    switch (type)
    {
        case UL_PACKET_SYNC:
        {
            //DEBUG_PRINTF(" S");
            ElrsSyncPacket_s const * const sync = (ElrsSyncPacket_s*)rx_buffer;

            if ((sync->CRCCaesarCipher == CRCCaesarCipher) &&
                (sync->sync_key == SYNC_KEY) &&
                (sync->arm_aux == AUX_CHANNEL_ARM))
            {
                if ((_conn_state == STATE_disconnected) || (_conn_state == STATE_lost))
                {
                    TentativeConnection(freq_err);
                    freq_err = 0;
                }
                else if (_conn_state == STATE_tentative)
                {
                    if (NonceRXlocal == sync->rxtx_counter &&
                        FHSSgetCurrIndex() == sync->fhssIndex)
                    {
                        no_sync_armed = sync->no_sync_armed;
                        GotConnection();
                    }
                    else if (2 < (tentative_cnt++))
                    {
                        LostConnection();
                        return;
                    }
                } else if (no_sync_armed && ((CRSF_CHANNEL_OUT_VALUE_MIN + 100) < CrsfChannels.ch4)) {
                    /* Sync should not be received, ignore it */
                    rx_last_valid_us = 0;
                    freq_err = 0;
                    return;
                }

                //DEBUG_PRINTF("nonce: %u <= %u\n", NonceRXlocal, sync->rxtx_counter);

                handle_tlm_ratio(sync->tlm_interval);
                FHSSsetCurrIndex(sync->fhssIndex);
                NonceRXlocal = sync->rxtx_counter;
            } else {
                /* Not a valid packet, ignore it */
                rx_last_valid_us = 0;
                freq_err = 0;
                return;
            }
            break;
        }
        case UL_PACKET_RC_DATA: //Standard RC Data Packet
            //DEBUG_PRINTF(" R");
            if (STATE_lost < _conn_state)
            {
#if CRC16_POLY_TESTING
                if (memcmp(rx_buffer, CRC16_POLY_PKT, sizeof(CRC16_POLY_PKT))) {
                    // Bad pkt content
                    DEBUG_PRINTF(" #");
                    return;
                }
#else // !CRC16_POLY_TESTING
                RcChannels_channels_extract(rx_buffer, CrsfChannels);
#if SERVO_OUTPUTS_ENABLED
#if SERVO_WRITE_FROM_ISR
                servo_out_write(&CrsfChannels);
#else
                update_servos = 1;
#endif
#else // !SERVO_OUTPUTS_ENABLED
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
                gpio_out_write(dbg_pin_rx, 0);
#endif
                crsf.sendRCFrameToFC(&CrsfChannels);
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
                gpio_out_write(dbg_pin_rx, 1);
#endif
#endif // SERVO_OUTPUTS_ENABLED
#endif // CRC16_POLY_TESTING
            }
            break;

        case UL_PACKET_MSP: {
            if (no_sync_armed && ((CRSF_CHANNEL_OUT_VALUE_MIN + 100) > CrsfChannels.ch4)) {
                /* Not a valid packet, ignore it */
                rx_last_valid_us = 0;
                freq_err = 0;
                return;
            }
#if !SERVO_OUTPUTS_ENABLED
            //DEBUG_PRINTF(" M");
            if (RcChannels_tlm_ota_receive(rx_buffer, msp_packet_rx)) {
                tlm_msp_rcvd = 1;
            }
#endif
            break;
        }

        case UL_PACKET_UNKNOWN:
        default:
            //DEBUG_PRINTF(" _");
            /* Not a valid packet, ignore it */
            rx_last_valid_us = 0;
            freq_err = 0;
            return;
            //break;
    }

    rx_freqerror = freq_err;

#if NUM_FAILS_TO_RESYNC
    rx_lost_packages = 0;
#endif
    LQ_packetAck();
    FillLinkStats();

#if PRINT_TIMING
    print_rx_isr_end_time = micros();
#endif
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_rx, 0);
#endif

#if USE_TIMER_KICK
    TxTimer.triggerSoon(); // Trigger FHSS ISR
#endif
}

void forced_stop(void)
{
    uint32_t irq = _SAVE_IRQ();
    TxTimer.stop();
    Radio->StopContRX();
    _RESTORE_IRQ(irq);
}

static void SetRFLinkRate(uint8_t rate) // Set speed of RF link (hz)
{
    const expresslrs_mod_settings_t *const config = get_elrs_airRateConfig(rate);
    if (config == NULL || config == ExpressLRS_currAirRate)
        return; // No need to modify, rate is same

    // Stop ongoing actions before configuring next rate
    forced_stop();

    ExpressLRS_currAirRate = config;
    current_rate_config = rate;

    DEBUG_PRINTF("Set RF rate: %u\n", config->rate);

    // Init CRC aka LQ array
    LQ_reset();
    // Reset FHSS
    FHSSfreqCorrectionReset();
    FHSSsetCurrIndex(0);

    handle_tlm_ratio(TLM_RATIO_NO_TLM);

    Radio->SetCaesarCipher(CRCCaesarCipher);
    Radio->Config(config->bw, config->sf, config->cr, GetInitialFreq(),
                  config->PreambleLen, (OTA_PACKET_CRC == 0),
                  config->pkt_type);

    // Measure RF noise
#if 0 && defined(DEBUG_SERIAL) // TODO: Enable this when noize floor is used!
    int RFnoiseFloor = Radio->MeasureNoiseFloor(10, GetInitialFreq());
    DEBUG_PRINTF("RF noise floor: %d dBm\n", RFnoiseFloor);
    (void)RFnoiseFloor;
#endif

    TxTimer.updateInterval(config->interval);
    LPF_FreqError.init(0);
    LPF_UplinkRSSI.init(0);
    LPF_UplinkSNR.init(0);
#if !SERVO_OUTPUTS_ENABLED
    LinkStatistics.link.uplink_RSSI_2 = 0;
    LinkStatistics.link.rf_Mode = config->rate_osd_num;
#endif
    //TxTimer.setTime();
    Radio->RXnb();
}

/* FC sends v1 MSPs */
void msp_data_cb(uint8_t const *const input)
{
#if RX_GHST_ENABLED
    (void)input;
#else

    if ((read_u8(&tlm_msp_send) != 0) || (tlm_check_ratio == 0))
        return;

    /* process MSP packet from flight controller
     *  [0]         header: seq&0xF,
     *  [1]         payload size
     *  [2]         function
     *  [3...57]    payload + crc
     */
    mspHeaderV1_TX_t *hdr = (mspHeaderV1_TX_t *)input;
    uint16_t iter;

    if (hdr->hdr.flags & MSP_STARTFLAG) {
        msp_packet_tx.reset();
        msp_packet_tx.type = MSP_PACKET_TLM_OTA;
        msp_packet_tx.payloadSize = hdr->hdr.payloadSize + 1; // incl crc
        msp_packet_tx.function = hdr->hdr.function;
        msp_packet_tx.flags = hdr->hdr.flags;
    }
    for (iter = 0; (iter < sizeof(hdr->hdr.payload)) && !msp_packet_tx.iterated(); iter++) {
        msp_packet_tx.addByte(hdr->hdr.payload[iter]);
    }
    if (iter <= sizeof(hdr->hdr.payload)) {
        if (!msp_packet_tx.error) {
            msp_packet_tx.setIteratorToSize();
            write_u8(&tlm_msp_send, 1); // rdy for sending
        } else {
            msp_packet_tx.reset();
        }
    }
#endif
}

void battery_info_cb(LinkStatsBatt_t * batt)
{
    LinkStatistics.batt.voltage = batt->voltage;
    LinkStatistics.batt.current = batt->current;
    LinkStatistics.batt.capacity = batt->capacity;
    LinkStatistics.batt.remaining = batt->remaining;
}

void gps_info_cb(GpsOta_t * gps)
{
    GpsTlm.latitude = gps->latitude;
    GpsTlm.longitude = gps->longitude;
    GpsTlm.speed = gps->speed;
    GpsTlm.heading = gps->heading;
    GpsTlm.altitude = gps->altitude;
    GpsTlm.satellites = gps->satellites;
    GpsTlm.pkt_cnt = 3;
}

void radio_prepare(uint8_t type)
{
    // Prepare radio
    Radio = common_config_radio(type);
    Radio->RXdoneCallback1 = ProcessRFPacketCallback;
    Radio->TXdoneCallback1 = tx_done_cb;
    Radio->SetOutputPower(0b1111); // default RX to max power for tlm
}

void setup()
{
    uint8_t UID[6] = {MY_UID};
    uint8_t radio_type;
#if RADIO_SX127x
    radio_type = RADIO_TYPE_127x;
#elif RADIO_SX128x
#if RADIO_SX128x_FLRC
    radio_type = RADIO_TYPE_128x_FLRC;
#else
    radio_type = RADIO_TYPE_128x;
#endif
#endif

#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
    dbg_pin_tmr = gpio_out_setup(DBG_PIN_TMR_ISR, 0);
#endif
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
    dbg_pin_rx = gpio_out_setup(DBG_PIN_RX_ISR, 0);
#endif

    ExpressLRS_currAirRate = NULL;
    connectionState = STATE_disconnected;
    tentative_cnt = 0;
    CRCCaesarCipher = CalcCRC16(UID, sizeof(UID), 0);

#if !SERVO_OUTPUTS_ENABLED
    CrsfSerial.Begin(RX_BAUDRATE);
#endif

    msp_packet_tx.reset();
    msp_packet_rx.reset();

    DEBUG_PRINTF("ExpressLRS RX Module...\n");
    platform_setup();

    // Prepare radio
    radio_prepare(radio_type);

#if (GPIO_PIN_ANTENNA_SELECT != UNDEF_PIN)
    gpio_out_setup(GPIO_PIN_ANTENNA_SELECT, 0);
#endif

    // Set call back for timer ISR
    TxTimer.callbackTock = &HWtimerCallback;
    TxTimer.init();
    // Init first scan index
    scanIndex = RATE_DEFAULT;
#if SERVO_OUTPUTS_ENABLED
    servo_out_init();
#else
    // Initialize CRSF protocol handler
    crsf.MspCallback = msp_data_cb;
    crsf.BattInfoCallback = battery_info_cb;
    crsf.GpsCallback = gps_info_cb;
    crsf.Begin();
#if PROTOCOL_CRSF_V3_TO_FC
    crsf.negotiate_baud(CRSF_RX_BAUDRATE_V3);
#endif
#endif
}

static uint32_t led_toggle_ms = 0;
void loop()
{
    uint32_t now = millis();

    const connectionState_e _conn_state = (connectionState_e)read_u32(&connectionState);

#if !SERVO_OUTPUTS_ENABLED && PROTOCOL_CRSF_V3_TO_FC
    if (crsf.negotiate_accepted()) {
        CrsfSerial.end();
        CrsfSerial.Begin(CRSF_RX_BAUDRATE_V3);
        crsf.Begin();
    }
#endif

    if (STATE_lost < _conn_state) {
        // check if connection is lost or in very bad shape
        if (ExpressLRS_currAirRate->connectionLostTimeout <= (int32_t)(now - read_u32(&LastValidPacket))
            /*|| read_u8(&uplink_Link_quality) <= 10*/) {
            LostConnection();
        } else if (_conn_state == STATE_connected) {
#if SERVO_OUTPUTS_ENABLED
#if !SERVO_WRITE_FROM_ISR
            if (read_u8(&update_servos))
                servo_out_write(&CrsfChannels);
#endif
#else
            if (SEND_LINK_STATS_TO_FC_INTERVAL <= (uint32_t)(now - SendLinkStatstoFCintervalNextSend)) {
                LinkStatistics.link.uplink_Link_quality = read_u8(&uplink_Link_quality);
                crsf.LinkStatisticsSend(LinkStatistics.link);
                SendLinkStatstoFCintervalNextSend = now;
            }
#endif
        }
    } else if (_conn_state == STATE_disconnected) {
        /* Cycle only if initial connection search */
        if ((!ExpressLRS_currAirRate) ||
            (ExpressLRS_currAirRate->syncSearchTimeout < (uint32_t)(now - RFmodeNextCycle))) {
            uint8_t max_rate = get_elrs_airRateMax();
#if RADIO_SX128x_FLRC
            if (max_rate <= scanIndex) {
                radio_prepare((get_elrs_current_radio_type() == RADIO_TYPE_128x_FLRC) ? RADIO_TYPE_128x : RADIO_TYPE_128x_FLRC);
                scanIndex = 0;
            }
#endif
            SetRFLinkRate((scanIndex % max_rate)); // switch between rates
            scanIndex++;
#if RADIO_SX128x_FLRC
            if (get_elrs_current_radio_type() == RADIO_TYPE_128x)
#endif
                if (max_rate <= scanIndex)
                    platform_connection_state(STATE_search_iteration_done);

            RFmodeNextCycle = now;
        } else if (150 <= (uint32_t)(now - led_toggle_ms)) {
            led_toggle();
            led_toggle_ms = now;
        }
    } else if (_conn_state == STATE_lost) {
        /* Just blink a led if connections is lost */
        if (300 <= (uint32_t)(now - led_toggle_ms)) {
            led_toggle();
            led_toggle_ms = now;
        }
    }

#if !SERVO_OUTPUTS_ENABLED
    crsf.handleUartIn();
#endif

    platform_loop(_conn_state);

    platform_wd_feed();

    /* Send MSP in junks to FC */
    if (read_u8(&tlm_msp_rcvd) && (10 <= (now - msp_packet_rx_sent))) {
        crsf.sendMSPFrameToFC(msp_packet_rx);
        if (msp_packet_rx.iterated() || msp_packet_rx.error) {
            msp_packet_rx.reset();
            write_u8(&tlm_msp_rcvd, 0);
        }
        msp_packet_rx_sent = now;
    }

#if PRINT_RATE && NO_DATA_TO_FC
    if ((1000U <= (uint32_t)(now - print_Rate_cnt_time)) &&
        (_conn_state == STATE_connected)) {
        DEBUG_PRINTF(" Rate: -%u +%u LQ:%u RSSI:%d SNR:%d\n",
            read_u32(&print_rate_cnt_fail),
            read_u32(&print_rate_cnt),
            read_u8(&uplink_Link_quality),
            LPF_UplinkRSSI.value(),
            LPF_UplinkSNR.value());
        write_u32(&print_rate_cnt, 0);
        write_u32(&print_rate_cnt_fail, 0);
        print_Rate_cnt_time = now;
    }
#endif
}
