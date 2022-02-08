#include "rx_servo_out.h"
#include "LowPassFilter.h"
#include "platform.h"

#if SERVO_OUTPUTS_ENABLED

#include "platform.h"
#include "targets.h"
#include "Arduino.h"
#include <Servo.h>


#define SERVO_PIN_ENABLED(_P) ((_P != UNDEF_PIN) ? 1 : 0)
#define NUM_OF_SERVO_CHANNELS \
    ( \
        + SERVO_PIN_ENABLED(SERVO_PIN_CH1) \
        + SERVO_PIN_ENABLED(SERVO_PIN_CH2) \
        + SERVO_PIN_ENABLED(SERVO_PIN_CH3) \
        + SERVO_PIN_ENABLED(SERVO_PIN_CH4) \
        + SERVO_PIN_ENABLED(SERVO_PIN_CH5) \
        + SERVO_PIN_ENABLED(SERVO_PIN_CH6) \
        + SERVO_PIN_ENABLED(SERVO_PIN_CH7) \
        + SERVO_PIN_ENABLED(SERVO_PIN_CH8) \
    )

#define US_OUT_MIN               988U    //1000
#define US_OUT_MAX              2012U    //2000
#define US_OUT_MID              ((US_OUT_MIN + US_OUT_MAX) / 2)
#define SERVO_USE_LPF           1
#define LPF_SMOOTHING_FACTOR    3

#define SERVO_UPDATE_INTERVAL_MS (50U)  // 50ms
#define SERVO_UPDATE_INTERVAL_US (SERVO_UPDATE_INTERVAL_MS * 1000) //us

// Scale receiver output values
#define CRSF_IN_to_US(val) \
    MAP_U16((val), CRSF_CHANNEL_OUT_VALUE_MIN, \
            CRSF_CHANNEL_OUT_VALUE_MAX, US_OUT_MIN, US_OUT_MAX)

#ifndef SERVO_OUTPUTS_THROTTLE_CH
    #define SERVO_OUTPUTS_THROTTLE_CH 1
#endif
#if SERVO_OUTPUTS_THROTTLE_CH < 1 || NUM_OF_SERVO_CHANNELS < SERVO_OUTPUTS_THROTTLE_CH
    #error "Servo channel must be 1 ... 4"
#endif

#if PROTOCOL_ELRS_TO_FC || PROTOCOL_CRSF_V3_TO_FC
    #error "ERLS and CRSFv3 protocols are not supported"
#endif

struct servo_pins {
    int32_t pin;
    Servo * servo;
    LPF * lpf;
};


#if (SERVO_PIN_CH1 != UNDEF_PIN)
Servo ch1;
#if SERVO_USE_LPF
static LPF DRAM_FORCE_ATTR lpf_ch1(LPF_SMOOTHING_FACTOR);
#endif // SERVO_USE_LPF
#endif
#if (SERVO_PIN_CH2 != UNDEF_PIN)
Servo ch2;
#if SERVO_USE_LPF
static LPF DRAM_FORCE_ATTR lpf_ch2(LPF_SMOOTHING_FACTOR);
#endif // SERVO_USE_LPF
#endif
#if (SERVO_PIN_CH3 != UNDEF_PIN)
Servo ch3;
#if SERVO_USE_LPF
static LPF DRAM_FORCE_ATTR lpf_ch3(LPF_SMOOTHING_FACTOR);
#endif // SERVO_USE_LPF
#endif
#if (SERVO_PIN_CH4 != UNDEF_PIN)
Servo ch4;
#if SERVO_USE_LPF
static LPF DRAM_FORCE_ATTR lpf_ch4(LPF_SMOOTHING_FACTOR);
#endif // SERVO_USE_LPF
#endif
#if (SERVO_PIN_CH5 != UNDEF_PIN)
Servo ch5;
#if SERVO_USE_LPF
static LPF DRAM_FORCE_ATTR lpf_ch5(LPF_SMOOTHING_FACTOR);
#endif // SERVO_USE_LPF
#endif
#if (SERVO_PIN_CH6 != UNDEF_PIN)
Servo ch6;
#if SERVO_USE_LPF
static LPF DRAM_FORCE_ATTR lpf_ch6(LPF_SMOOTHING_FACTOR);
#endif // SERVO_USE_LPF
#endif
#if (SERVO_PIN_CH7 != UNDEF_PIN)
Servo ch7;
#if SERVO_USE_LPF
static LPF DRAM_FORCE_ATTR lpf_ch7(LPF_SMOOTHING_FACTOR);
#endif // SERVO_USE_LPF
#endif
#if (SERVO_PIN_CH8 != UNDEF_PIN)
Servo ch8;
#if SERVO_USE_LPF
static LPF DRAM_FORCE_ATTR lpf_ch8(LPF_SMOOTHING_FACTOR);
#endif // SERVO_USE_LPF
#endif


static struct servo_pins DRAM_ATTR servo_pins[] = {
#if (SERVO_PIN_CH1 != UNDEF_PIN)
    {SERVO_PIN_CH1, &ch1, &lpf_ch1},
#endif
#if (SERVO_PIN_CH2 != UNDEF_PIN)
    {SERVO_PIN_CH2, &ch2, &lpf_ch2},
#endif
#if (SERVO_PIN_CH3 != UNDEF_PIN)
    {SERVO_PIN_CH3, &ch3, &lpf_ch3},
#endif
#if (SERVO_PIN_CH4 != UNDEF_PIN)
    {SERVO_PIN_CH4, &ch4, &lpf_ch4},
#endif
#if (SERVO_PIN_CH5 != UNDEF_PIN)
    {SERVO_PIN_CH5, &ch5, &lpf_ch5},
#endif
#if (SERVO_PIN_CH6 != UNDEF_PIN)
    {SERVO_PIN_CH6, &ch6, &lpf_ch6},
#endif
#if (SERVO_PIN_CH7 != UNDEF_PIN)
    {SERVO_PIN_CH7, &ch7, &lpf_ch7},
#endif
#if (SERVO_PIN_CH8 != UNDEF_PIN)
    {SERVO_PIN_CH8, &ch8, &lpf_ch8},
#endif
};


void servo_out_init(void) {
    uint16_t value;
    for (uint8_t iter = 0; iter < ARRAY_SIZE(servo_pins); iter++) {
        /* Set throttle channel to low, others to middle */
        value = (iter == (SERVO_OUTPUTS_THROTTLE_CH-1)) ? US_OUT_MIN : US_OUT_MID;
        servo_pins[iter].servo->attach(
            servo_pins[iter].pin, US_OUT_MIN, US_OUT_MAX, value);
        servo_pins[iter].lpf->init(0);
    }
}


void FAST_CODE_2 servo_out_fail_safe(void)
{
    uint16_t value;
    for (uint8_t iter = 0; iter < ARRAY_SIZE(servo_pins); iter++) {
        /* Set throttle channel to low, others to middle */
        value = (iter == (SERVO_OUTPUTS_THROTTLE_CH-1)) ? US_OUT_MIN : US_OUT_MID;
        servo_pins[iter].servo->writeMicroseconds(value);
    }
}


void FAST_CODE_2 servo_out_write(rc_channels_rx_t const * const channels, uint32_t const now_us)
{
    static uint32_t DRAM_ATTR last_update_us;
    uint32_t const irq = _SAVE_IRQ();
    uint16_t channel_values[ARRAY_SIZE(servo_pins)] = {
        #if (SERVO_PIN_CH1 != UNDEF_PIN)
            (uint16_t)channels->ch0,
        #endif
        #if (SERVO_PIN_CH2 != UNDEF_PIN)
            (uint16_t)channels->ch1,
        #endif
        #if (SERVO_PIN_CH3 != UNDEF_PIN)
            (uint16_t)channels->ch2,
        #endif
        #if (SERVO_PIN_CH4 != UNDEF_PIN)
            (uint16_t)channels->ch3,
        #endif
        #if (SERVO_PIN_CH5 != UNDEF_PIN)
            (uint16_t)channels->ch4,
        #endif
        #if (SERVO_PIN_CH6 != UNDEF_PIN)
            (uint16_t)channels->ch5,
        #endif
        #if (SERVO_PIN_CH7 != UNDEF_PIN)
            (uint16_t)channels->ch6,
        #endif
        #if (SERVO_PIN_CH8 != UNDEF_PIN)
            (uint16_t)channels->ch7,
        #endif
    };
    _RESTORE_IRQ(irq);

#if SERVO_USE_LPF
    for (uint8_t iter = 0; iter < ARRAY_SIZE(servo_pins); iter++) {
        channel_values[iter] = servo_pins[iter].lpf->update(channel_values[iter]);
    }
#endif

    if (SERVO_UPDATE_INTERVAL_US < (now_us - last_update_us)) {
        for (uint8_t iter = 0; iter < ARRAY_SIZE(servo_pins); iter++) {
            servo_pins[iter].servo->writeMicroseconds(CRSF_IN_to_US(channel_values[iter]));
        }
        last_update_us = now_us;
    }
}

#endif /* SERVO_OUTPUTS_ENABLED */
