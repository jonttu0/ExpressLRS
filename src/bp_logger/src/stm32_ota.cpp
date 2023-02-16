#if CONFIG_STM_UPDATER
#include "stm32_ota.h"
#include "main.h"
#include "stm32Updater.h"
#include "stk500.h"
#include <Arduino.h>
#include <ESPAsyncWebServer.h>


static bool flash_stm32(uint32_t flash_addr, String &uploadedfilename)
{
  int8_t result = -1;
  websocket_send("STM32 Firmware Flash Requested!");
  websocket_send("  the firmware file: '" + uploadedfilename + "'");
  if (uploadedfilename.endsWith("firmware.elrs")) {
    result = stk500_write_file(uploadedfilename.c_str());
  } else if (uploadedfilename.endsWith("firmware.bin")) {
    result = esp8266_spifs_write_file(uploadedfilename.c_str(), flash_addr);
    if (result == 0)
      reset_stm32_to_app_mode(); // boot into app
  } else {
    websocket_send("Invalid file!");
  }
  Serial.begin(SERIAL_BAUD);
  return (0 <= result);
}


bool stm32_ota_handleFileUploadEnd(AsyncWebServerRequest * request)
{
  String filename = "";
  uint32_t flash_base = BEGIN_ADDRESS;
  for (size_t i = 0; i < request->args(); i++) {
    String name = request->argName(i);
    String value = request->arg(i);
    if (name == "flash_address") {
      flash_base = strtol(&value.c_str()[2], NULL, 16);
    } else if (name == "firmware") {
      filename = "/" + name;
    }
#if UART_DEBUG_EN
    Serial.printf(" arg: %s = %s\r\n", name.c_str(), value.c_str());
#endif
  }

  bool const success = flash_stm32(flash_base, filename);

  if (filename.length() && FILESYSTEM.exists("/" + filename))
    FILESYSTEM.remove("/" + filename);

  websocket_send((success) ? "Update Successful!": "Update Failure!");
#if UART_DEBUG_EN
  Serial.println("STM32 upgrade done!");
#endif
  return success;
}

#endif // CONFIG_STM_UPDATER
