#include "internal.h"
#include "helpers.h"
#include "utils.h"
#include "platform_internal.h"
#include <stddef.h>

#define I2C_FUNCTION (GPIO_FUNCTION(4) | GPIO_OPEN_DRAIN)

struct i2c_info {
    i2c_type * handle;
    uint16_t function;
    uint8_t sda_pin, scl_pin;
};

static struct i2c_info i2c_bus[] = {
#ifdef I2C1_BASE
    {I2C1, I2C_FUNCTION, GPIO('B',  7), GPIO('B', 6)},
    //{I2C1, I2C_FUNCTION, GPIO('A', 10), GPIO('A', 9)},
#endif
#ifdef I2C2_BASE
    {I2C2, I2C_FUNCTION, GPIO('B', 11), GPIO('B', 10)},
#endif
#ifdef I2C3_BASE
    {I2C3, I2C_FUNCTION, GPIO('B',  4), GPIO('A', 7)},
#endif
};

void i2c_configure(i2c_type * handle, uint32_t rate, uint8_t own_addr)
{
    enable_pclock((uint32_t)handle);

    i2c_reset(handle);
    i2c_init(handle, I2C_FSMODE_DUTY_2_1, rate);
    i2c_own_address1_set(handle, I2C_ADDRESS_MODE_7BIT, own_addr);
    i2c_enable(handle, TRUE);
}

struct i2c_config i2c_setup(uint32_t rate, int scl, int sda, uint8_t addr)
{
    uint8_t bus;
    for (bus = 0; bus < ARRAY_SIZE(i2c_bus); bus++) {
        struct i2c_info const * const i2c = &i2c_bus[bus];
        if (i2c->sda_pin == sda && i2c->scl_pin == scl)
            break;
    }
    if (bus < ARRAY_SIZE(i2c_bus)) {
        // Enable I2C
        struct i2c_info const * const i2c = &i2c_bus[bus];
        if (!is_enabled_pclock((uint32_t)i2c->handle)) {
            gpio_peripheral(i2c->sda_pin, i2c->function, 0);
            gpio_peripheral(i2c->scl_pin, i2c->function, 0);
        }

        i2c_configure(i2c->handle, rate, addr);

        return (struct i2c_config){.i2c = i2c->handle, .addr = addr};
    }
    return {.i2c = NULL};
}

void i2c_transfer(struct i2c_config config, uint8_t receive_data, uint8_t len, uint8_t *data)
{
    i2c_type * const handle = (i2c_type *)config.i2c;
    if (!handle) return;
    i2c_own_address1_set(handle, I2C_ADDRESS_MODE_7BIT, config.addr);
    while (len--) {
        while (i2c_flag_get(handle, I2C_TDBE_FLAG) == RESET);
        handle->dt = *data;
        while (i2c_flag_get(handle, I2C_RDBF_FLAG) == RESET);
        uint8_t const rdata = handle->dt;
        if (receive_data)
            *data = rdata;
        data++;
    }
}
