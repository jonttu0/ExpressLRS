#pragma once

#include "RadioInterface.h"
#include "DAC.h"

#ifndef TX_POWER_DEFAULT
/* Just in case, this should be defined in user_defines.txt file */
#define TX_POWER_DEFAULT PWR_50mW
#endif


typedef enum
{
    PWR_DYNAMIC = 0,
    PWR_10mW,
    PWR_25mW,
    PWR_50mW,
    PWR_100mW,
    PWR_250mW,
    PWR_500mW,
    PWR_1000mW,
    PWR_2000mW,
    PWR_UNKNOWN
} PowerLevels_e;

#if TX_POWER_DEFAULT > PWR_2000mW || TX_POWER_DEFAULT < PWR_10mW
#error "Default power is not valid!"
#endif

class POWERMGNT
{
private:
    RadioInterface *p_radio;
    PowerLevels_e p_current_power;
    PowerLevels_e p_max_power;
    uint_fast8_t p_dyn_power;
    R9DAC * p_dac;
    struct gpio_out fan_pin;

    uint8_t p_set_power(PowerLevels_e power);

public:
    POWERMGNT(int fan = -1);
    void Begin(RadioInterface *radio, R9DAC *dac = NULL);

    void set_fan(int pin) {
        fan_pin = gpio_out_setup(pin, 0);
    }

    // inc and decPower are used to control dynamic tx power
    PowerLevels_e incPower();
    PowerLevels_e decPower();
    // loop is used to control power by button on tx module
    PowerLevels_e loopPower();

    PowerLevels_e currPower() const
    {
        return p_dyn_power ? PWR_DYNAMIC : p_current_power;
    }
    PowerLevels_e maxPowerGet() const
    {
        return p_max_power;
    }
    uint8_t setPower(PowerLevels_e power);

    uint8_t power_to_radio_enum(PowerLevels_e power = PWR_UNKNOWN)
    {
        if (power == PWR_UNKNOWN || power == PWR_DYNAMIC)
            power = p_current_power;
        // OpenTX: ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW, 250mW )
        switch (power)
        {
            case PWR_10mW:
                return 1;
            case PWR_25mW:
                return 2;
            case PWR_50mW:
                return 2; // not speficied by otx, use 25mW
            case PWR_100mW:
                return 3;
            case PWR_250mW:
                return 7;
            case PWR_500mW:
                return 4;
            case PWR_1000mW:
                return 5;
            case PWR_2000mW:
                return 6;
            default:
                break;
        }
        return 0;
    }

    void pa_off(void) const;
    void pa_on(void) const;
};
