#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include "platform_internal.h"
#include "common_defs.h"
#include <stdint.h>
#include <stddef.h>

#ifndef __section
#define __section(S)
#endif

#define FORCED_INLINE inline __attribute__((always_inline))
#ifndef noinline
#define noinline __attribute__((noinline))
#endif

#define SERVO_WRITE_FROM_ISR 0

#define barrier() __asm__ __volatile__("": : :"memory")

static inline void write_u32(void *addr, uint32_t val) {
    barrier();
    *(volatile uint32_t *)addr = val;
}
static inline void write_u16(void *addr, uint16_t val) {
    barrier();
    *(volatile uint16_t *)addr = val;
}
static inline void write_u8(void *addr, uint8_t val) {
    barrier();
    *(volatile uint8_t *)addr = val;
}
static inline uint32_t read_u32(const void *addr) {
    uint32_t val = *(volatile const uint32_t *)addr;
    barrier();
    return val;
}
static inline uint16_t read_u16(const void *addr) {
    uint16_t val = *(volatile const uint16_t *)addr;
    barrier();
    return val;
}
static inline uint8_t read_u8(const void *addr) {
    uint8_t val = *(volatile const uint8_t *)addr;
    barrier();
    return val;
}


/** EEPROM storage key
* v0 - initial
* v1 - tlm added
* v2 - power range changed (dynamic added)
*/
#define ELRS_EEPROM_KEY 0x454c5208 // ELR + version nbr

struct platform_config
{
    uint32_t key;
    uint32_t rf_mode;
    struct rf_s {
        uint32_t mode;
        uint32_t power;
        uint32_t tlm;
    } rf[3]; // rf_mode: 0 = SX127x, 1 = SX128x, 2 = SX128x_FLRC

    /* Handset specific data */
    struct gimbal_limit gimbals[TX_NUM_ANALOGS];
    struct mixer mixer[TX_NUM_MIXER];
};
extern struct platform_config pl_config;

int8_t platform_config_load(struct platform_config &config);
int8_t platform_config_save(struct platform_config &config);
void platform_setup(void);
void platform_mode_notify(uint8_t mode);
void platform_loop(int state);
void platform_connection_state(int state);
void platform_set_led(uint8_t state);
void platform_restart(void);
void platform_wd_feed(void);
void platform_wifi_start(void);
void platform_radio_force_stop(void);

class CtrlSerial
{
public:
    virtual size_t available(void) = 0;
    virtual uint8_t read(void) = 0;

    void write(uint8_t data) {
        write(&data, 1);
    }
    virtual void write(uint8_t * buffer, size_t size) = 0;

private:
};

// NOTE! tx_main uses CtrlSerial class for RX and TX
extern CtrlSerial& ctrl_serial;

#endif // _PLATFORM_H_
