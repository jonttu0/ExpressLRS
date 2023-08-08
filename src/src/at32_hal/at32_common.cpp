#include "platform.h"
#include "targets.h"
#include "debug_elrs.h"
#include "common.h"
#include "POWERMGNT.h"
#include "gpio.h"
#include "ws2812.h"
#if defined(RX_MODULE)
#if RX_GHST_ENABLED
#include "GHST.h"
#define RX_BAUDRATE GHST_RX_BAUDRATE
#else
#include "CRSF.h"
#define RX_BAUDRATE CRSF_RX_BAUDRATE
#endif
#endif // RX_MODULE
#include <at32_eeprom.h>

#if (GPIO_PIN_LED != UNDEF_PIN)
struct gpio_out led_red;
#define LED_STATE_RED(_x) gpio_out_write(led_red, ((!!(_x)) ^ GPIO_PIN_LED_RED_INV))
#else
#define LED_STATE_RED(_x) (void)(_x);
#endif // GPIO_PIN_LED
#if (GPIO_PIN_LED_GREEN != UNDEF_PIN)
struct gpio_out led_green;
#define LED_STATE_GREEN(_x) gpio_out_write(led_green, ((!!(_x)) ^ GPIO_PIN_LED_GREEN_INV))
#else
#define LED_STATE_GREEN(_x) (void)(_x);
#endif // GPIO_PIN_LED_GREEN
#if (GPIO_PIN_LED_RGB != UNDEF_PIN)
#undef LED_STATE_RED
#undef LED_STATE_GREEN
#define LED_STATE_RED(_x) ws2812_set_color(((_x)?0xff:0), 0x0, 0x0)
#define LED_STATE_GREEN(_x) ws2812_set_color(0x0, ((_x)?0xff:0), 0x0)
#endif

#if (GPIO_PIN_BUZZER != UNDEF_PIN)
struct gpio_out buzzer;
static inline void PLAY_SOUND(uint32_t wait = 244, uint32_t cnt = 50)
{
    for (uint32_t x = 0; x < cnt; x++)
    {
        // 1 / 2048Hz = 488uS, or 244uS high and 244uS low to create 50% duty cycle
        gpio_out_write(buzzer, HIGH);
        delayMicroseconds(wait);
        gpio_out_write(buzzer, LOW);
        delayMicroseconds(wait);
    }
}
#else
#define PLAY_SOUND(_x, _y)
#endif

#if defined(TX_MODULE)
extern POWERMGNT PowerMgmt;
static uint32_t tone_play_cnt = 0;
static uint32_t tone_last_played = 0;
void play_tone_loop(uint32_t ms)
{
    if (tone_play_cnt && 350 < (ms - tone_last_played))
    {
        PLAY_SOUND(244, 50);
        tone_play_cnt--;
        tone_last_played = ms;
    }
}
#endif /* TX_MODULE */


#if (GPIO_PIN_BUTTON != UNDEF_PIN)
#include "ClickButton.h"

/* Button is inverted */
ClickButton clickButton(GPIO_PIN_BUTTON, true, 40,  400,  600);

void button_handle(void)
{
    uint32_t ms = millis();
    clickButton.update(ms);
#if defined(TX_MODULE)
    if (clickButton.clicks == 1 && clickButton.lastClickLong) {
        tone_play_cnt = PowerMgmt.loopPower() + 1;
        clickButton.reset();
    } else if (clickButton.clicks == 2 && clickButton.lastClickLong) {
        extern int8_t tx_tlm_toggle(void);
        tone_play_cnt = tx_tlm_toggle() + 1;
        clickButton.reset();
    }
#elif defined(RX_MODULE)
    if (clickButton.clicks <= -(BUTTON_RESET_INTERVAL_RX / 600)) {
        platform_restart();
        clickButton.reset();
    }
#endif // RX_MODULE
}
#endif // GPIO_PIN_BUTTON

/******************* CONFIG *********************/
int8_t platform_config_load(struct platform_config &config)
{
#if STORE_TO_FLASH
    int8_t res = -1;
    struct platform_config temp;
    eeprom_read((uint8_t*)&temp, sizeof(temp));
    if (temp.key == ELRS_EEPROM_KEY) {
        /* load ok, copy values */
        memcpy(&config, &temp, sizeof(temp));
        res = 0;
    } else if ((temp.key & ELRS_EEPROM_KEY_MASK) == ELRS_EEPROM_KEY_BASE) {
        platform_config_migrate(&temp, config);
    }
#else
    int8_t res = 0;
    config.key = ELRS_EEPROM_KEY;
#endif
    return res;
}

int8_t platform_config_save(struct platform_config &config)
{
    if (config.key != ELRS_EEPROM_KEY)
        return -1;
#if STORE_TO_FLASH
    eeprom_write((uint8_t*)&config, sizeof(config));
#endif
    return 0;
}

/******************* SETUP *********************/
#if 0
void initVariant(void)
{
    led_red = gpio_out_setup(GPIO_PIN_LED, (0 ^ GPIO_PIN_LED_RED_INV));
    uint8_t state = 0;
    while (1) {
        LED_STATE_RED(state);
        state ^= 1;
        delay(500);
    }
}
#endif

void platform_setup(void)
{
#if defined(DEBUG_SERIAL) && defined(TX_MODULE) && !defined(TARGET_HANDSET_STM32F722)
    // Init debug serial if not done already
    if (((void *)&DEBUG_SERIAL != (void *)&CrsfSerial) &&
        ((void *)&DEBUG_SERIAL != (void *)&Serial1))
    {
        // init debug serial
#if (GPIO_PIN_DEBUG_TX != UNDEF_PIN)
        DEBUG_SERIAL.setTx(GPIO_PIN_DEBUG_TX);
#endif
#if (GPIO_PIN_DEBUG_RX != UNDEF_PIN)
        DEBUG_SERIAL.setRx(GPIO_PIN_DEBUG_RX);
#endif
        DEBUG_SERIAL.begin(400000);
    }
#endif /* DEBUG_SERIAL */

#if defined(CTRL_SERIAL)
#ifndef CTRL_SERIAL_BAUD
#define CTRL_SERIAL_BAUD 460800
#endif
    CTRL_SERIAL.begin(CTRL_SERIAL_BAUD);
    CTRL_SERIAL.setTimeout(5);
#endif // CTRL_SERIAL
#if defined(BT_SERIAL)
    BT_SERIAL.begin(BT_SERIAL_BAUD);
#endif // BT_SERIAL

    /*************** CONFIGURE LEDs *******************/
#if (GPIO_PIN_LED_RGB != UNDEF_PIN)
    ws2812_init(GPIO_PIN_LED_RGB);
#endif // GPIO_PIN_LED_RGB
#if (GPIO_PIN_LED != UNDEF_PIN)
    led_red = gpio_out_setup(GPIO_PIN_LED, (0 ^ GPIO_PIN_LED_RED_INV));
#endif
#if (GPIO_PIN_LED_GREEN != UNDEF_PIN)
    led_green = gpio_out_setup(GPIO_PIN_LED_GREEN, (0  ^ GPIO_PIN_LED_GREEN_INV));
#endif

#if defined(TX_MODULE)
    /*************** CONFIGURE TX *******************/

#if DAC_IN_USE
    // DAC is used to control ADC which sets PA output
    r9dac.init(GPIO_PIN_SDA, GPIO_PIN_SCL, DAC_I2C_ADDRESS,
               GPIO_PIN_RFswitch_CONTROL, GPIO_PIN_RFamp_APC1,
               GPIO_PIN_RFamp_APC2);
#endif // DAC_IN_USE
#if (GPIO_PIN_BUZZER != UNDEF_PIN)
    buzzer = gpio_out_setup(GPIO_PIN_BUZZER, LOW);
#endif // GPIO_PIN_BUZZER

#elif defined(RX_MODULE)
    /*************** CONFIGURE RX *******************/

#endif /* RX_MODULE */

#if defined(GPIO_SELECT_RFIO_HIGH) && defined(GPIO_SELECT_RFIO_LOW)
#if defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
    (void)gpio_out_setup(GPIO_SELECT_RFIO_HIGH, 0);
    (void)gpio_out_setup(GPIO_SELECT_RFIO_LOW, 1);
#else
    (void)gpio_out_setup(GPIO_SELECT_RFIO_LOW, 0);
    (void)gpio_out_setup(GPIO_SELECT_RFIO_HIGH, 1);
#endif /* RFIO HIGH / LOW */
#endif

    /* TODO: move ANT_CTRL_x to radio hal */
#if (GPIO_PIN_ANT_CTRL_1 != UNDEF_PIN)
    (void)gpio_out_setup(GPIO_PIN_ANT_CTRL_1, 1);
#endif
#if (GPIO_PIN_ANT_CTRL_2 != UNDEF_PIN)
    (void)gpio_out_setup(GPIO_PIN_ANT_CTRL_2, 0);
#endif
}

void platform_mode_notify(uint8_t mode)
{
#if (GPIO_PIN_BUZZER != UNDEF_PIN) || (GPIO_PIN_LED_GREEN != UNDEF_PIN)
    for (int i = 0; i < mode; i++)
    {
        delay(300);
        LED_STATE_GREEN(HIGH);
        PLAY_SOUND(244, 50);
        delay(50);
        LED_STATE_GREEN(LOW);
    }
#endif
}

void platform_loop(int state)
{
#if (GPIO_PIN_BUTTON != UNDEF_PIN)
#if defined(TX_MODULE)
    button_handle();
    play_tone_loop(millis());
#elif defined(RX_MODULE)
    if (state == STATE_disconnected)
        button_handle();
#endif /* RX_MODULE */
#endif // GPIO_PIN_BUTTON
    (void)state;
}

void platform_connection_state(int const state)
{
    uint8_t connected = (state == STATE_connected);
#if (GPIO_PIN_LED_RGB != UNDEF_PIN)
    ws2812_set_color_u32((connected ? 0xFF00 : 0x00FF));
#else
    LED_STATE_GREEN(connected);
#if defined(TX_MODULE)
    //platform_set_led(!connected);
#endif
#endif
}

void platform_set_led(uint8_t state)
{
#if (GPIO_PIN_LED_RGB != UNDEF_PIN)
    ws2812_set_color_u32((state ? 0xFF0000 /*blue*/ : 0xFF /*red*/));
#else
    LED_STATE_RED(state);
#endif
}

enum {
    BL_FLAG_KEY = 0x626C0000,
    /* 16bits */
    BL_FLAG_BAUDRATE = 1,
};

struct bootloader {
    uint32_t key;
    uint32_t reset_type;
    uint32_t flags;
    uint32_t baudrate;
};

void platform_restart(void)
{
    NVIC_SystemReset();
}

void platform_reboot_into_bootloader(const uint8_t * info)
{
#if defined(RX_MODULE)
    if (validate_bl_indentifier(info) < 0)
        return;
#if !USE_DEFAULT_LD_SCRIPT
    /* Fill reset info into RAM for bootloader */
    extern __IO uint32_t _bootloader_data;
    volatile struct bootloader * blinfo = ((struct bootloader*)&_bootloader_data) + 0;
    blinfo->key = 0x454c5253; // ELRS
    blinfo->reset_type = 0xACDC;
    blinfo->flags = BL_FLAG_KEY | BL_FLAG_BAUDRATE;
    blinfo->baudrate = RX_BAUDRATE;
#endif // USE_DEFAULT_LD_SCRIPT
#endif // RX_MODULE
    platform_restart();
}

void platform_wifi_start(void)
{
}

void platform_wd_feed(void)
{
}

/*************************************************************************/

class CtrlSerialPrivate: public CtrlSerial
{
public:
    size_t available(void);
    uint8_t read(void);

    void write(uint8_t * buffer, size_t size);
};

void CtrlSerialPrivate::write(uint8_t * data, size_t len)
{
#if defined(CTRL_SERIAL)
    CTRL_SERIAL.write(data, len);
#endif
}

size_t CtrlSerialPrivate::available(void)
{
#if defined(CTRL_SERIAL)
    return CTRL_SERIAL.available();
#else
    return 0;
#endif
}

uint8_t CtrlSerialPrivate::read(void)
{
#if defined(CTRL_SERIAL)
    return CTRL_SERIAL.read();
#else
    return 0;
#endif
}

CtrlSerialPrivate ctrl_serial_private;
CtrlSerial& ctrl_serial = ctrl_serial_private;
