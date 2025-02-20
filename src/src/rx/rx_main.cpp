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
#define LINK_STATS_SEND_INTERVAL_MS 100U
#define LINK_STATS_SEND_INTERVAL_US (LINK_STATS_SEND_INTERVAL_MS * 1000)

/* Debug variables */
#define PRINT_FREQ_ERROR    0
#ifndef PRINT_RATE
#define PRINT_RATE          1
#endif
#define PRINT_TIMING        0
#define PRINT_TIMING_IN_CRSF_FRAME   0

#if PRINT_RATE && NO_DATA_TO_FC
uint32_t print_rate_cnt;
uint32_t print_rate_cnt_fail;
uint32_t print_tlm_cnt;
uint32_t print_Rate_cnt_time;
#endif
#if PRINT_TIMING && NO_DATA_TO_FC
static uint32_t DRAM_ATTR print_rx_isr_end_time;
#endif

///////////////////

static lq_data_t DRAM_ATTR rx_lq;

RX_CLASS DRAM_FORCE_ATTR crsf(CrsfSerial); //pass a serial port object to the class for it to use

int8_t DRAM_ATTR connectionState;
static volatile uint8_t DRAM_ATTR NonceRXlocal; // nonce that we THINK we are up to.
static uint8_t DRAM_ATTR TLMinterval;
static uint32_t DRAM_ATTR tlm_check_ratio;
static uint32_t DRAM_ATTR rx_last_valid_us; // Time the last valid packet was recv
static int32_t DRAM_ATTR rx_freqerror;
static volatile int32_t DRAM_ATTR rx_hw_isr_running;

static uint16_t DRAM_ATTR CRCCaesarCipher;
static uint32_t DRAM_ATTR SyncCipher;

#if SERVO_OUTPUTS_ENABLED
static uint8_t DRAM_ATTR update_servos;
#endif
static rc_channels_rx_t DRAM_ATTR rcChannelsData;
static LinkStats_t DRAM_ATTR LinkStatistics;
static GpsOta_t DRAM_ATTR GpsTlm;
static DeviceInfo_t DRAM_ATTR DevInfo;
static uint8_t DRAM_ATTR rcDataRcvdCnt;
static uint32_t DRAM_ATTR rcDataTxCountMask;

///////////////////////////////////////////////
////////////////  Filters  ////////////////////
static LPF DRAM_FORCE_ATTR LPF_UplinkRSSI(5);
static LPF DRAM_FORCE_ATTR LPF_UplinkSNR(5);

//////////////////////////////////////////////////////////////
////// Variables for Telemetry and Link Quality //////////////

static uint32_t DRAM_ATTR LastValidPacket_ms; //Time the last valid packet was recv
static mspPacket_t DRAM_FORCE_ATTR msp_packet_rx;
//static uint32_t DRAM_ATTR msp_packet_rx_sent;
static uint32_t DRAM_ATTR msp_packet_received_ms;
static mspPacket_t DRAM_FORCE_ATTR msp_packet_tx;
static uint8_t DRAM_ATTR tlm_msp_send;
static uint8_t DRAM_ATTR tlm_msp_rcvd;
static uint8_t DRAM_ATTR uplink_Link_quality;

///////////////////////////////////////////////////////////////
///////////// Variables for Sync Behaviour ////////////////////
static uint32_t DRAM_ATTR RfModeCycled_ms;
static uint8_t DRAM_ATTR scanIndex;
static uint8_t DRAM_ATTR tentative_cnt;
static uint8_t DRAM_ATTR no_sync_armed;
static int8_t DRAM_ATTR rcvd_rate_index;
static int8_t DRAM_ATTR rcvd_pkt_type;

///////////////////////////////////////
#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
struct gpio_out dbg_pin_tmr;
#endif
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
struct gpio_out dbg_pin_rx;
#endif

#if AUX_CHANNEL_ARM == 0
    #define IS_ARMED_CHECK() (SWITCH_IS_SET(rcChannelsData.ch4))
#elif AUX_CHANNEL_ARM == 1
    #define IS_ARMED_CHECK() (SWITCH_IS_SET(rcChannelsData.ch5))
#elif AUX_CHANNEL_ARM == 2
    #define IS_ARMED_CHECK() (SWITCH_IS_SET(rcChannelsData.ch6))
#elif AUX_CHANNEL_ARM == 3
    #define IS_ARMED_CHECK() (SWITCH_IS_SET(rcChannelsData.ch7))
#elif AUX_CHANNEL_ARM == 4
    #define IS_ARMED_CHECK() (SWITCH_IS_SET(rcChannelsData.ch8))
#elif AUX_CHANNEL_ARM == 5
    #define IS_ARMED_CHECK() (SWITCH_IS_SET(rcChannelsData.ch9))
#elif AUX_CHANNEL_ARM == 6
    #define IS_ARMED_CHECK() (SWITCH_IS_SET(rcChannelsData.ch10))
#elif AUX_CHANNEL_ARM == 7
    #define IS_ARMED_CHECK() (SWITCH_IS_SET(rcChannelsData.ch11))
#elif AUX_CHANNEL_ARM == 8
    #define IS_ARMED_CHECK() (SWITCH_IS_SET(rcChannelsData.ch12))
#elif AUX_CHANNEL_ARM == 9
    #define IS_ARMED_CHECK() (SWITCH_IS_SET(rcChannelsData.ch13))
#elif AUX_CHANNEL_ARM == 10
    #define IS_ARMED_CHECK() (SWITCH_IS_SET(rcChannelsData.ch14))
#elif AUX_CHANNEL_ARM == 11
    #define IS_ARMED_CHECK() (SWITCH_IS_SET(rcChannelsData.ch15))
#else
    #define IS_ARMED_CHECK() 1
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
        tlm_check_ratio = TlmEnumToMask(interval);
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
    int32_t rssiDBM = (int16_t)read_u16(&Radio->LastPacketRSSI);
    rssiDBM = LPF_UplinkRSSI.update(rssiDBM);
    // our rssiDBM is currently in the range -128 to 98, but BF wants a value in the range
    // 0 to 255 that maps to -1 * the negative part of the rssiDBM, so cap at 0.
    if (0 < rssiDBM) rssiDBM = 0;
    else if (rssiDBM < INT8_MIN) rssiDBM = INT8_MIN;

    LinkStatistics.link.uplink_RSSI_1 = -1 * rssiDBM; // to match BF
    LinkStatistics.link.uplink_SNR = LPF_UplinkSNR.update((int8_t)read_u8(&Radio->LastPacketSNR) * 10);
}

void FAST_CODE_1 SendLinkStats(uint32_t const us, uint8_t const lq)
{
    LinkStatistics.link.uplink_Link_quality = uplink_Link_quality;
    crsf.LinkStatisticsSend(LinkStatistics.link, us);
}

uint8_t FAST_CODE_1 RadioFreqErrorCorr(void)
{
    // Do freq correction before FHSS
    /* Freq tunin ~84us */
    int32_t freqerror = rx_freqerror;
    rx_freqerror = 0;
    if (freqerror) {
#if PRINT_FREQ_ERROR && NO_DATA_TO_FC
        DEBUG_PRINTF(" > freqerror:%u", freqerror);
#endif
        freqerror = FHSSfreqCorrectionApply(freqerror);
        if (freqerror) {
#if PRINT_FREQ_ERROR && NO_DATA_TO_FC
            //DEBUG_PRINTF(" local:%u", freqerror);
#endif
            Radio->setPPMoffsetReg(freqerror);
            return 1;
        }
    }
    return 0;
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

void FAST_CODE_1 HandleSendTelemetryResponse(void) // total ~79us
{
    //DEBUG_PRINTF("X");
#if PRINT_RATE && NO_DATA_TO_FC
    //print_tlm_cnt++;
#endif
    // esp requires word aligned buffer
    uint32_t __tx_buffer[(OTA_PAYLOAD_MAX + sizeof(uint32_t) - 1) / sizeof(uint32_t)];
    uint8_t *tx_buffer = (uint8_t *)__tx_buffer;
    uint16_t crc_or_type;
    uint_fast8_t payloadSize = RcChannels_payloadSizeGet();

    if (RcChannels_dev_info_pack(tx_buffer, DevInfo)) {
        crc_or_type = DL_PACKET_DEV_INFO;
    } else if ((tlm_msp_send == 1) && (msp_packet_tx.type == MSP_PACKET_TLM_OTA)) {
        if (RcChannels_tlm_ota_send(tx_buffer, msp_packet_tx)) {
            tlm_msp_send = 0;
        }
        crc_or_type = DL_PACKET_TLM_MSP;
    } else if (RcChannels_gps_pack(tx_buffer, GpsTlm)) {
        crc_or_type = DL_PACKET_GPS;
    } else {
        RcChannels_link_stas_pack(tx_buffer, LinkStatistics, uplink_Link_quality);
        crc_or_type = DL_PACKET_TLM_LINK;
    }

    RcChannels_packetTypeSet(tx_buffer, payloadSize, crc_or_type);

    if (ExpressLRS_currAirRate->hwCrc == HWCRC_DIS) {
        crc_or_type = CalcCRC16(tx_buffer, payloadSize, CRCCaesarCipher);
        tx_buffer[payloadSize++] = (crc_or_type >> 8);
        tx_buffer[payloadSize++] = (crc_or_type & 0xFF);
    }
    Radio->TXnb(tx_buffer, payloadSize, FHSSgetCurrFreq());
}

void tx_done_cb(void)
{
    // Configure RX only next is not hopping time
    //if (((NonceRXlocal + 1) % ExpressLRS_currAirRate->FHSShopInterval) != 0)
    //    Radio->RXnb(FHSSgetCurrFreq());
#if PRINT_RATE && NO_DATA_TO_FC
    print_tlm_cnt++;
#endif
}


void SendDataToFcCallback(uint32_t const us); // prototype
static uint8_t DRAM_ATTR rcDataSend;
#define SEND_CB_OFFSET_US   400


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
    const int8_t conn_state = connectionState;
    uint_fast8_t fhss_config_rx = 0;
    uint_fast8_t nonce = NonceRXlocal;
    rx_last_valid_us = 0;

#if PRINT_TIMING && NO_DATA_TO_FC
    const uint32_t freq_now = FHSSgetCurrFreq();
#endif

    /* do the timing adjustment based on last reception */
    if (0 != last_rx_us) {
        diff_us = (int32_t)((uint32_t)(us - last_rx_us));

        if (diff_us < -TIMER_OFFSET) diff_us = -TIMER_OFFSET;
        else if (diff_us > TIMER_OFFSET) diff_us = TIMER_OFFSET;

        /* Adjust the timer offset */
        diff_us -= TIMER_OFFSET;

#if PRINT_RATE && NO_DATA_TO_FC
        print_rate_cnt++;
#endif
    } else {
#if PRINT_RATE && NO_DATA_TO_FC
        print_rate_cnt_fail++;
#endif
    }

    // adjust the timer
    TxTimer.callbackTick = &SendDataToFcCallback;
    TxTimer.setTime(SEND_CB_OFFSET_US - diff_us);

    if ((nonce & rcDataTxCountMask) == 0) {
        if (0 < rcDataRcvdCnt) {
            rcDataSend = true;
            rcDataRcvdCnt = 0;
            LQ_packetAck(&rx_lq);
        }
        uplink_Link_quality = LQ_getlinkQuality(&rx_lq);
        LQ_nextPacket(&rx_lq);
    }

    /*fhss_config_rx |=*/ RadioFreqErrorCorr();
    fhss_config_rx |= HandleFHSS(nonce);

    if (TlmFrameCheck(nonce, tlm_ratio)
            && (conn_state == STATE_connected)
            && !FHSScurrSequenceIndexIsSyncChannel()) {
        /* Send telemetry response */
#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
        gpio_out_write(dbg_pin_tmr, 0);
#endif
        HandleSendTelemetryResponse();
#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
        gpio_out_write(dbg_pin_tmr, 1);
#endif

    } else if (fhss_config_rx) {
        /* Configure next reception if needed */
        Radio->RXnb(FHSSgetCurrFreq());
    }

    NonceRXlocal = nonce;

#if PRINT_TIMING && NO_DATA_TO_FC
    uint32_t now = micros();
    DEBUG_PRINTF("RX:%u (t:%u) HW:%u diff:%d (t:%u) [f %u]\n",
                 last_rx_us, (print_rx_isr_end_time - last_rx_us),
                 us, diff_us, (uint32_t)(now - us), freq_now);
#endif
#if PRINT_TIMING_IN_CRSF_FRAME
    static uint32_t timer_last_call_us;
    //if (!last_rx_us)
    {
        rcChannelsData.ch2 = us - timer_last_call_us;
        //DEBUG_PRINTF("D%u ", (us - timer_last_call_us));
        rcDataSend = true;
    }
    timer_last_call_us = us;
#else
    if (!last_rx_us) {
        //DEBUG_PRINTF("-");  // RX and TX missed
    }
#endif

    rx_hw_isr_running = 0;

#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_tmr, 0);
#endif
    return;
}


/* This is used send UART data to flight controller.
 *  functionality is moved to here since it takes too much time on ESP based
 *  receivers (no DMA).
 */
void FAST_CODE_1 SendDataToFcCallback(uint32_t const us)
{
#if PLATFORM_ESP8266
    #define TIMER_ADJUST_EXTRA  7
#elif PLATFORM_STM32 || PLATFORM_AT32
    #define TIMER_ADJUST_EXTRA  6
#else
    #define TIMER_ADJUST_EXTRA  0
#endif

    // Change callback function
    TxTimer.callbackTick = &HWtimerCallback;
    // Reset the timer
    TxTimer.reset(SEND_CB_OFFSET_US + TIMER_ADJUST_EXTRA);

    // Send RC data
    if (rcDataSend) {
#if PRINT_TIMING_IN_CRSF_FRAME
        rcChannelsData.ch3 = connectionState;
#endif
        crsf.sendRCFrameToFC(&rcChannelsData);
        rcDataSend = false;
#if PRINT_TIMING_IN_CRSF_FRAME
        rcChannelsData.ch0 = 0;
        rcChannelsData.ch1 = 0;
        rcChannelsData.ch2 = 0;
#endif
    }

    /* Send stats to FC from here to avoid RC data blocking.
     * MSP messages are sent from main loop (blocked when armed)
     */
#if SERVO_OUTPUTS_ENABLED
    if (update_servos && STATE_lost < connectionState) {
        servo_out_write(&rcChannelsData, us);
    }
    update_servos = 0;
#else
    if (STATE_connected == connectionState && !PRINT_TIMING_IN_CRSF_FRAME) {
        SendLinkStats(us, uplink_Link_quality);
    }
#endif
}


void FAST_CODE_1 LostConnection()
{
    if (connectionState <= STATE_lost) {
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
    FHSSresetCurrIndex();

    connectionState = STATE_lost; //set lost connection

    Radio->RXnb(FHSSgetCurrFreq());
    DEBUG_PRINTF("lost conn\n");

    platform_connection_state(STATE_lost);
}

void FAST_CODE_1 TentativeConnection(int32_t const freqerror)
{
    /* Do initial freq correction */
    Radio->setPPMoffsetReg(FHSSfreqCorrectionApply(freqerror));

    tentative_cnt = 0;
    connectionState = STATE_tentative;
    DEBUG_PRINTF("tentative\n");
    TxTimer.callbackTick = &HWtimerCallback;
    TxTimer.start();     // Start local sync timer
}

void FAST_CODE_1 GotConnection(uint32_t const current_us)
{
    connectionState = STATE_connected;
    platform_connection_state(STATE_connected);
    DEBUG_PRINTF("connected in %d ms\n", (int32_t)(current_us - RfModeCycled_ms));
}

void FAST_CODE_1
ProcessRFPacketCallback(uint8_t *rx_buffer, uint32_t current_us, size_t payloadSize, int32_t freq_err)
{
    /* Skip if hw isr is triggered already (e.g. TX has some weird latency)
     * or  error in reception (CRC etc), kick hw timer
     */
    if (rx_hw_isr_running || !rx_buffer) {
        DEBUG_PRINTF("!");
#if PRINT_TIMING_IN_CRSF_FRAME
        rcChannelsData.ch1 = 1;
#endif
        return;
    }

#if (DBG_PIN_RX_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_rx, 1);
#endif

    //DEBUG_PRINTF("I");
    const int8_t _conn_state = connectionState;

    if (ExpressLRS_currAirRate->hwCrc == HWCRC_DIS) {
        payloadSize -= OTA_PACKET_CRC;

        const uint16_t crc = CalcCRC16(rx_buffer, payloadSize, CRCCaesarCipher);
        const uint16_t crc_in = ((uint16_t)rx_buffer[payloadSize] << 8) + rx_buffer[payloadSize + 1];

        if (crc_in != crc) {
    #if (DBG_PIN_RX_ISR != UNDEF_PIN)
            gpio_out_write(dbg_pin_rx, 0);
    #endif
            DEBUG_PRINTF("!");
            return;
        }
    }

    // TODO, FIXME: Disable IRQs?

    //DEBUG_PRINTF("E%d ", freq_err);

    switch (RcChannels_packetTypeGet(rx_buffer, payloadSize))
    {
        case UL_PACKET_SYNC:
        {
            //DEBUG_PRINTF(" S");
            ElrsSyncPacket_s const * const sync = (ElrsSyncPacket_s*)rx_buffer;
            const uint_fast8_t numOfTxPerRc = ExpressLRS_currAirRate->numOfTxPerRc;

            if (sync->cipher == SyncCipher) {
                if ((_conn_state == STATE_disconnected) || (_conn_state == STATE_lost)) {
                    /* Jump to tentative if the pkt mode and rate are correct */
                    if (sync->radio_mode == ExpressLRS_currAirRate->pkt_type &&
                            sync->rate_index == current_rate_config) {
                        TentativeConnection(freq_err);
                    } else {
                        rcvd_pkt_type = sync->radio_mode;
                        rcvd_rate_index = sync->rate_index;
                        /* Mark lost to stay on received config */
                        connectionState = STATE_lost;
                        goto exit_rx_isr;
                    }
                    freq_err = 0;
                } else if (_conn_state == STATE_tentative) {
                    if (NonceRXlocal == sync->rxtx_counter &&
                            FHSSgetCurrIndex() == sync->fhssIndex) {
                        no_sync_armed = sync->no_sync_armed;
                        GotConnection(current_us);
                    } else if (2 < (tentative_cnt++)) {
                        LostConnection();
                        goto exit_rx_isr;
                    }
                } else if (numOfTxPerRc == 1 && no_sync_armed && IS_ARMED_CHECK()) {
                    /* Sync should not be received, ignore it */
                    goto exit_rx_isr;
                }
                //DEBUG_PRINTF("nonce: %u <= %u\n", NonceRXlocal, sync->rxtx_counter);

                handle_tlm_ratio(sync->tlm_interval);
                FHSSsetCurrIndex(sync->fhssIndex);
                NonceRXlocal = sync->rxtx_counter;
            } else {
                /* Not a valid packet, ignore it */
                goto exit_rx_isr;
            }
            break;
        }
        case UL_PACKET_RC_DATA: //Standard RC Data Packet
            //DEBUG_PRINTF(" R");
            if (STATE_tentative <= _conn_state) {
#if CRC16_POLY_TESTING
                if (memcmp(rx_buffer, CRC16_POLY_PKT, sizeof(CRC16_POLY_PKT))) {
                    // Bad pkt content
                    DEBUG_PRINTF(" #");
                    return;
                }
#else // !CRC16_POLY_TESTING
                RcChannels_channels_extract(rx_buffer, rcChannelsData);
    #if SERVO_OUTPUTS_ENABLED
                update_servos = 1;
    #else // !SERVO_OUTPUTS_ENABLED
                ++rcDataRcvdCnt;
    #endif // SERVO_OUTPUTS_ENABLED
#endif // CRC16_POLY_TESTING
            }
            break;

        case UL_PACKET_MSP: {
            if (IS_ARMED_CHECK()) {
                /* Not a valid packet, ignore it */
                goto exit_rx_isr;
            }
#if !SERVO_OUTPUTS_ENABLED
            //DEBUG_PRINTF(" M");
            if (RcChannels_tlm_ota_receive(rx_buffer, msp_packet_rx)) {
                tlm_msp_rcvd = 1;
                msp_packet_received_ms = millis();
            }
#endif
            break;
        }

        case UL_PACKET_UNKNOWN:
        default:
            DEBUG_PRINTF("?");
            /* Not a valid packet, ignore it */
            goto exit_rx_isr;
    }

    LastValidPacket_ms = millis();
    rx_last_valid_us = current_us;
    rx_freqerror = freq_err;

    FillLinkStats();

#if PRINT_TIMING && NO_DATA_TO_FC
    print_rx_isr_end_time = micros();
#endif
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_rx, 0);
#endif
#if PRINT_TIMING_IN_CRSF_FRAME
    rcChannelsData.ch0 = 1;
    rcChannelsData.ch1 = 0;
#endif

    TxTimer.callbackTick = &HWtimerCallback;
    TxTimer.triggerSoon(); // Trigger FHSS ISR
    return;

exit_rx_isr:
    rx_last_valid_us = 0;
    rx_freqerror = 0;
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_rx, 0);
#endif
    DEBUG_PRINTF(".");
    return;
}

void forced_stop(void)
{
    uint32_t irq = _SAVE_IRQ();
    TxTimer.stop();
    if (Radio)
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

    LQ_reset(&rx_lq);
    FHSSfreqCorrectionReset();
    FHSSresetCurrIndex();

    handle_tlm_ratio(TLM_RATIO_NO_TLM);
    // Repeated TX params
    rcDataRcvdCnt = 0;
    rcDataTxCountMask = (0 < config->numOfTxPerRc) ? config->numOfTxPerRc - 1 : 0;

    DEBUG_PRINTF("Set RF rate: %u (sync ch: %u)\n", config->rate, FHSScurrSequenceIndex());

    RcChannels_initRcPacket(config->payloadSize);
    Radio->SetRxBufferSize(config->payloadSize + (config->hwCrc == HWCRC_DIS ? OTA_PACKET_CRC : 0));
    //Radio->SetPacketInterval(config->interval);
    Radio->SetCaesarCipher(CRCCaesarCipher);
    Radio->Config(config->bw, config->sf, config->cr, FHSSgetCurrFreq(),
                  config->PreambleLen, (config->hwCrc == HWCRC_EN),
                  config->pkt_type);

    // Measure RF noise
#if 0 && defined(DEBUG_SERIAL) // TODO: Enable this when noize floor is used!
    int RFnoiseFloor = Radio->MeasureNoiseFloor(10, FHSSgetCurrFreq());
    DEBUG_PRINTF("RF noise floor: %d dBm\n", RFnoiseFloor);
    (void)RFnoiseFloor;
#endif

    TxTimer.updateInterval(config->interval);
    LPF_UplinkRSSI.init(0);
    LPF_UplinkSNR.init(0);
#if !SERVO_OUTPUTS_ENABLED
    LinkStatistics.link.uplink_RSSI_2 = 0;
    LinkStatistics.link.rf_Mode = config->rate_osd_num;
#endif
    // Start the sync packet reception
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
     *    Start:
     *      [0] header: seq&0xF,
     *      [1] payload size
     *      [2] function
     *      [3...57] payload + crc
     *    Next:
     *      [0] header
     *      [1...57] payload + crc
     */
    mspHeaderV1_TX_t const * const p_msp = (mspHeaderV1_TX_t *)input;
    uint8_t const * p_src = p_msp->payload;
    size_t iter;

    if (p_msp->flags & MSP_ERRORFLAG) {
        // Handle error!
    }

    if (p_msp->flags & MSP_STARTFLAG) {
        msp_packet_tx.reset();
        msp_packet_tx.type = MSP_PACKET_TLM_OTA;
        msp_packet_tx.payloadSize = p_msp->hdr.payloadSize;
        msp_packet_tx.function = p_msp->hdr.function;
        p_src = p_msp->hdr.payload;
    }
    for (iter = 0; (iter < sizeof(p_msp->payload)) && !msp_packet_tx.iterated(); iter++) {
        msp_packet_tx.addByte(p_src[iter]);
    }

    if (msp_packet_tx.iterated() && !msp_packet_tx.error) {
        msp_packet_tx.setIteratorToSize();
        write_u8(&tlm_msp_send, 1); // rdy for sending
    } else {
        msp_packet_tx.reset();
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

void dev_info_cb(uint8_t const state)
{
    // FC connection ok, send info to transmitter
    DevInfo.state = state;
    DevInfo.transmit = true;
}

void radio_prepare(uint8_t const type)
{
    if (get_elrs_current_radio_type() == type)
        return;
    forced_stop();
    // Prepare radio
    Radio = common_config_radio(type);
    if (!Radio) {
        /* Infinite loop in case of failure */
        while(1) {
            led_set_state(true);
            delay(100);
            led_set_state(false);
            delay(100);
            led_set_state(true);
            delay(100);
            led_set_state(false);
            DEBUG_PRINTF("RADIO CONFIG ERROR!\n");
#if defined(DEBUG_SERIAL) && 0  // DEBUG TEST CODE!
            static uint8_t temp_buff[256];
            int len = DEBUG_SERIAL.available();
            uint8_t * outbuff = temp_buff;
            while (len--) {
                *outbuff++ = DEBUG_SERIAL.read();
            }
            len = (uintptr_t)outbuff - (uintptr_t)temp_buff;
            if (len)
                DEBUG_SERIAL.write(temp_buff, len);
#endif
            delay(1000);
        }
    }
    Radio->RXdoneCallback1 = ProcessRFPacketCallback;
    Radio->TXdoneCallback1 = tx_done_cb;
    Radio->SetOutputPower(RECEIVER_TRANSMIT_POWER);
}

void setup()
{
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
    CRCCaesarCipher = my_uid_crc16();
    SyncCipher = (CRCCaesarCipher ^ my_uid_to_u32()) & SYNC_CIPHER_MASK;

#if !SERVO_OUTPUTS_ENABLED
    CrsfSerial.Begin(RX_BAUDRATE);
#endif

    msp_packet_tx.reset();
    msp_packet_rx.reset();

    DEBUG_PRINTF("ExpressLRS RX Module...\n");
    my_uid_print();
    platform_setup();

    // Prepare radio
    radio_prepare(radio_type);

#if (GPIO_PIN_ANTENNA_SELECT != UNDEF_PIN)
    gpio_out_setup(GPIO_PIN_ANTENNA_SELECT, 0);
#endif

    // Set call back for timer ISR
    TxTimer.callbackTick = &HWtimerCallback;
    TxTimer.init();
    // Init first scan index
    scanIndex = RATE_DEFAULT;
    rcvd_pkt_type = rcvd_rate_index = -1;

#if SERVO_OUTPUTS_ENABLED
    servo_out_init();
#else
    // Initialize CRSF protocol handler
    crsf.MspCallback = msp_data_cb;
    crsf.BattInfoCallback = battery_info_cb;
    crsf.GpsCallback = gps_info_cb;
    crsf.DevInfoCallback = dev_info_cb;
    crsf.Begin();
#endif
}


int handle_received_ptk_type_and_rate_index(uint32_t const now_ms)
{
    int8_t ota_type = rcvd_pkt_type, rate_index = rcvd_rate_index;
    //write_u8((uint8_t*)&rcvd_pkt_type, (uint8_t)-1);
    //write_u8((uint8_t*)&rcvd_rate_index, (uint8_t)-1);
    /* Force mode if correct values are received from sync message */
    if (0 > ota_type || RADIO_FLRC < ota_type ||
        0 > rate_index || get_elrs_airRateMax() <= rate_index) {
        return -1;
    }
#if RADIO_SX128x_FLRC
    ota_type = (ota_type == RADIO_FLRC) ? RADIO_TYPE_128x_FLRC : RADIO_TYPE_128x;
    radio_prepare(ota_type);
#endif
    SetRFLinkRate(rate_index);
    RfModeCycled_ms = now_ms;
    rcvd_pkt_type = -1;
    rcvd_rate_index = -1;
    return 0;
}


static uint32_t DRAM_ATTR led_toggle_ms = 0;
void loop()
{
    uint32_t const now = millis();

    const int8_t _conn_state = (int8_t)read_u8(&connectionState);

    if (STATE_lost < _conn_state) {
        led_set_state(_conn_state == STATE_connected);
        // check if connection is lost or in very bad shape
        if (ExpressLRS_currAirRate->connectionLostTimeout <= (int32_t)(now - read_u32(&LastValidPacket_ms))
            /*|| read_u8(&uplink_Link_quality) <= 10*/) {
            LostConnection();
            RfModeCycled_ms = now;
        }
    } else if (_conn_state == STATE_disconnected) {
        /* Cycle only if initial connection search */
        if ((!ExpressLRS_currAirRate) ||
            (ExpressLRS_currAirRate->syncSearchTimeout < (uint32_t)(now - RfModeCycled_ms))) {
            uint8_t max_rate = get_elrs_airRateMax();
#if RADIO_SX128x_FLRC
            uint8_t current_radio_type = get_elrs_current_radio_type();
            if (max_rate <= scanIndex) {
                current_radio_type = (current_radio_type == RADIO_TYPE_128x_FLRC) ?
                        RADIO_TYPE_128x : RADIO_TYPE_128x_FLRC;
                radio_prepare(current_radio_type);
                scanIndex = 0;
                max_rate = get_elrs_airRateMax();
            }
#endif
            SetRFLinkRate((scanIndex % max_rate)); // switch between rates
            scanIndex++;
#if RADIO_SX128x_FLRC
            if (current_radio_type == RADIO_TYPE_128x)
#endif
                if (max_rate <= scanIndex)
                    platform_connection_state(STATE_search_iteration_done);

            RfModeCycled_ms = now;
        } else if (150 <= (uint32_t)(now - led_toggle_ms)) {
            led_toggle();
            led_toggle_ms = now;
        }
    } else if (_conn_state == STATE_lost) {
        handle_received_ptk_type_and_rate_index(now);
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
    if (read_u8(&tlm_msp_rcvd) /*&& (10 <= (now - msp_packet_rx_sent))*/) {
        if (!msp_packet_rx.error)
            crsf.sendMSPFrameToFC(msp_packet_rx);
        if (msp_packet_rx.iterated() || msp_packet_rx.error || (100 <= (now - read_u32(&msp_packet_received_ms)))) {
            msp_packet_rx.reset();
            write_u8(&tlm_msp_rcvd, 0);
            DEBUG_PRINTF("MSP done!\n");
        }
        //msp_packet_rx_sent = now;
    }

#if PRINT_RATE && NO_DATA_TO_FC
    if ((1000U <= (uint32_t)(now - print_Rate_cnt_time)) &&
        (_conn_state == STATE_connected)) {
#if 1
        uint32_t const bad = read_u32(&print_rate_cnt_fail);
        uint32_t const good = read_u32(&print_rate_cnt);
        uint32_t const tlmtx = read_u32(&print_tlm_cnt);
        write_u32(&print_rate_cnt_fail, 0);
        write_u32(&print_rate_cnt, 0);
        write_u32(&print_tlm_cnt, 0);
        DEBUG_PRINTF(" Rate: +%u !%u t%u LQ:%u RSSI:%d SNR:%d - RC: %u|%u|%u|%u|*|%u|%u|%u|%u|\n",
            good, (uint32_t)abs((long int)(bad - tlmtx)), tlmtx,
            uplink_Link_quality,
            LPF_UplinkRSSI.value(),
            LPF_UplinkSNR.value(),
            rcChannelsData.ch0, rcChannelsData.ch1, rcChannelsData.ch2, rcChannelsData.ch3,
            rcChannelsData.ch4, rcChannelsData.ch5, rcChannelsData.ch6, rcChannelsData.ch7
            );
#else
        DEBUG_PRINTF("\n");
#endif
        print_Rate_cnt_time = now;
    }
#endif
}
