#include "POWERMGNT.h"
#include "debug_elrs.h"


/* Power arrays per module type */
typedef struct {
    int8_t power[PWR_UNKNOWN];
} PowerArray_t;

PowerArray_t power_array[MODULE_COUNT] = {
    // max pwr,  10mW, 25mW, 50mW, 100mW, 250mW, 500mW, 1000mW, 2000mW

    /********** SX127x MODULES **********/
#if defined(TARGET_PWR_MAX_SX127x) && defined(TARGET_PWR_LUT_SX127x)
    {TARGET_PWR_MAX_SX127x, TARGET_PWR_LUT_SX127x},
#elif defined(TARGET_MODULE_LORA1276F30)
    // MODULE_LORA1276F30. 0 = 40mW, 15 = 300mW
    {PWR_250mW,     0,    0,    1,     4,    11,    15,     15,     15},
#else
    // Bare SX127x
    {PWR_50mW,      8,   12,   15,    15,    15,    15,     15,     15},
#endif

    /********** SX128x MODULES **********/
#if defined(TARGET_PWR_MAX_SX128x) && defined(TARGET_PWR_LUT_SX128x)
    {TARGET_PWR_MAX_SX128x, TARGET_PWR_LUT_SX128x},
#elif defined(TARGET_MODULE_LORA1280F27)
    #error "Don't use LORA1280F27 module! It is not reliable!"
    {PWR_500mW,    -7,   -3,    0,     3,     8,    13,     13,     13},
#elif (TARGET_MODULE_E28_VER == 12) || defined(TARGET_MODULE_E28_2G4M12S)
    // E28_2G4M12S (18mW)
#warning "E28_2G4M12S needs to be verified"
    {PWR_10mW,      8,   12,   15,    15,    15,    15,     15,     15},
#elif (TARGET_MODULE_E28_VER == 20) || defined(TARGET_MODULE_E28_2G4M20S)
    // E28_2G4M20S (100mW)
#warning "E28_2G4M20S needs to be verified"
    {PWR_100mW,     8,   12,   15,    15,    15,    15,     15,     15},
#elif (TARGET_MODULE_E28_VER == 27) || defined(TARGET_MODULE_E28_2G4M27S)
    // E28_2G4M27S (500mW)
    {PWR_250mW,   -17,  -13,  -10,    -7,    -3,     0,      0,      0},
#else
    // Bare SX128x
    {PWR_25mW,      8,   13,   13,    13,    13,    13,     13,     13},
#endif
};

POWERMGNT::POWERMGNT(int fan)
{
    p_radio = NULL;
    p_dac = NULL;
    p_current_power = PWR_UNKNOWN;
    p_max_power = PWR_10mW;
    p_dyn_power = 0;
    set_fan(fan);
}

void POWERMGNT::Begin(RadioInterface *radio, R9DAC *dac)
{
    p_dac = dac;
    p_radio = radio;
    p_max_power = PWR_10mW;
    p_current_power = PWR_UNKNOWN;
    if (radio) {
        uint8_t type = radio->GetModuleType();
        if (type < MODULE_COUNT) {
            p_max_power = (PowerLevels_e)power_array[type].power[0];
            if (type == MODULE_SX127x && dac)
                radio->SetOutputPower(0b0000);
        }
    }
}

PowerLevels_e POWERMGNT::incPower()
{
    if (p_dyn_power && p_current_power < p_max_power) {
        p_set_power((PowerLevels_e)((uint8_t)p_current_power + 1));
    }
    return p_current_power;
}

PowerLevels_e POWERMGNT::decPower()
{
    if (p_dyn_power && p_current_power > PWR_10mW) {
        p_set_power((PowerLevels_e)((uint8_t)p_current_power - 1));
    }
    return p_current_power;
}

PowerLevels_e POWERMGNT::loopPower()
{
    PowerLevels_e next;
    if (p_dyn_power) {
        next = PWR_10mW;
    } else {
        next = (PowerLevels_e)((p_current_power + 1) % (p_max_power+1));
    }
    setPower(next);
    return next;
}

void POWERMGNT::setPower(PowerLevels_e power)
{
    if (power == PWR_DYNAMIC) {
        // enable dynamic power and reset current level to default
        p_dyn_power = 1;
        power = TX_POWER_DEFAULT;
    } else {
        if (power == PWR_UNKNOWN)
            power = TX_POWER_DEFAULT;
        p_dyn_power = 0;
    }

    p_set_power(power);
}

void FAST_CODE_1 POWERMGNT::pa_off(void) const
{
    if (p_dac && p_radio->GetModuleType() == MODULE_SX127x)
        p_dac->standby();
}

void FAST_CODE_1 POWERMGNT::pa_on(void) const
{
    if (p_dac && p_radio->GetModuleType() == MODULE_SX127x)
        p_dac->resume();
}

/************************** PRIVATE ******************************/

void POWERMGNT::p_set_power(PowerLevels_e power)
{
    if (power == p_current_power || power < PWR_10mW ||
        power > p_max_power || !p_radio)
        return;

    uint8_t type = p_radio->GetModuleType();

    DEBUG_PRINTF("MGMT set pwr:%u, type:%u\n", power, type);

    if (p_dac && type == MODULE_SX127x) {
        p_dac->setPower(power);
    } else {
        PowerArray_t * powers = &power_array[type];
        p_radio->SetOutputPower(powers->power[power]);
    }
    p_current_power = power;

    if (gpio_out_valid(fan_pin))
        gpio_out_write(fan_pin, (PWR_50mW < power));
}
