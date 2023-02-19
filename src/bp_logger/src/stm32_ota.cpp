#include "stm32_ota.h"

#if CONFIG_STM_UPDATER
#include "main.h"
#include "stm32Updater.h"
#include "stk500.h"
#include <Arduino.h>
#include <ESPAsyncWebServer.h>

static String g_firmware_file;
static uint32_t g_flash_address;

static bool flash_stm32(uint32_t flash_addr, String & uploadedfilename)
{
    int8_t result = -1;

    websocket_send_txt("STM32 Firmware Flash Requested!");
    websocket_send_txt("  the firmware file: '" + uploadedfilename + "'");
    if (uploadedfilename.endsWith("firmware.elrs")) {
        result = stk500_write_file(uploadedfilename.c_str());
    } else if (uploadedfilename.endsWith("firmware.bin")) {
        result = esp8266_spifs_write_file(uploadedfilename.c_str(), flash_addr);
        if (result == 0)
            reset_stm32_to_app_mode(); // boot into app
    } else {
        websocket_send_txt("Invalid file!");
    }
    Serial.begin(SERIAL_BAUD);
    return (0 <= result);
}

bool stm32_ota_check_filename(String & filename)
{
    if (filename != "firmware.bin" && filename != "firmware.elrs") {
        g_firmware_file = "";
        return false;
    }
    g_firmware_file = "/" + filename;
    return true;
}

bool stm32_ota_parse_args(AsyncWebServerRequest * request)
{
    g_flash_address = BEGIN_ADDRESS;
    for (size_t i = 0; i < request->args(); i++) {
        String name = request->argName(i);
        String value = request->arg(i);
        if (name == "flash_address") {
            g_flash_address = strtol(&value.c_str()[2], NULL, 16);
        }
#if UART_DEBUG_EN
        Serial.printf(" arg: %s = %s\r\n", name.c_str(), value.c_str());
#endif
    }
    return true;
}

void stm32_ota_do_flash(void)
{
    if (FILESYSTEM.exists(g_firmware_file)) {
        bool const success = flash_stm32(g_flash_address, g_firmware_file);
        FILESYSTEM.remove(g_firmware_file);
        websocket_send_txt((success) ? "Update Successful!" : "Update Failure!");
#if UART_DEBUG_EN
        Serial.println("STM32 upgrade done!");
#endif
    } else {
        const char * error = "STM Upgrade: Requested file does not exist! Abort!";
        websocket_send_txt(error);
#if UART_DEBUG_EN
        Serial.println(error);
#endif
    }
}
#else

bool stm32_ota_check_filename(String & filename)
{
    return false;
}

bool stm32_ota_parse_args(AsyncWebServerRequest * request)
{
    return false;
}

void stm32_ota_do_flash(void)
{
}
#endif // CONFIG_STM_UPDATER
