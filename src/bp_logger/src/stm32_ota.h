#pragma once
#include <WString.h>

class AsyncWebServerRequest;

bool stm32_ota_check_filename(String & filename);
bool stm32_ota_parse_args(AsyncWebServerRequest * request);
void stm32_ota_do_flash(void);
