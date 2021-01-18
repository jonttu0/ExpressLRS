#include <Arduino.h>
#include "utils.h"
#include "common.h"
#include "targets.h"
#include "POWERMGNT.h"
#include "debug_elrs.h"
#include "tx_common.h"
#include "gimbals.h"
#include <stdlib.h>


static uint32_t DRAM_ATTR TlmSentToRadioTime;

///////////////////////////////////////

void setup()
{
    tx_common_init_globals();
    platform_setup();
    DEBUG_PRINTF("ExpressLRS TX Module...\n");

    tx_common_init();
}

void loop()
{
    tx_common_handle_rx_buffer();

    if (0 <= tx_common_has_telemetry())
    {
        uint32_t current_ms = millis();
        if (0 <= tx_common_check_connection() &&
            connectionState == STATE_connected &&
            TLM_REPORT_INTERVAL <= (uint32_t)(current_ms - TlmSentToRadioTime))
        {
            TlmSentToRadioTime = current_ms;
            tx_common_update_link_stats();

            // TODO: Send TLM data to CTRL_SERIAL (MSP)
        }
    }

    // Send MSP resp if allowed and packet ready
    if (tlm_msp_rcvd)
    {
        DEBUG_PRINTF("DL MSP rcvd. func: %x, size: %u\n",
            msp_packet_rx.function, msp_packet_rx.payloadSize);

        // TODO: Send received MSP packet to CTRL_SERIAL (MSP)
        // msp_packet_rx;

        msp_packet_rx.reset();
        tlm_msp_rcvd = 0;
    } else {
        tx_common_handle_ctrl_serial();
    }

    platform_loop(connectionState);
    platform_wd_feed();
}
