#pragma once
#if CONFIG_STM_UPDATER
#include <WString.h>

class AsyncWebServerRequest;

bool stm32_ota_handleFileUploadEnd(AsyncWebServerRequest * request);
#endif // CONFIG_STM_UPDATER
