#ifndef _RX_LINK_QUALITY_H__
#define _RX_LINK_QUALITY_H__

#include <string.h>

#define LQ_LEN  100

typedef struct {
    uint_fast8_t index;
    uint_fast8_t lq;
    uint8_t data[LQ_LEN];
} lq_data_t;


void FAST_CODE_1 LQ_nextPacket(lq_data_t * const lq)
{
    uint_fast8_t index = lq->index;
    // Next index
    index = (index + 1) % sizeof(lq->data);
    // Remove oldest value
    lq->lq -= lq->data[index];
    // Clear oldest value
    lq->data[index] = 0;
    lq->index = index;
}

void FORCED_INLINE LQ_packetAck(lq_data_t * const lq)
{
    lq->data[lq->index] = 1;
    lq->lq++;
}

uint_fast8_t FORCED_INLINE LQ_getlinkQuality(lq_data_t const * const lq)
{
    return lq->lq;
}

void FAST_CODE_1 LQ_reset(lq_data_t * const lq)
{
    // default to all set
    memset(lq->data, 1, sizeof(lq->data));
    lq->index = 0;
    lq->lq = sizeof(lq->data);
}

#endif // _RX_LINK_QUALITY_H__
