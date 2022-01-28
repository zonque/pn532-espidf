#pragma once

#include "esp_log.h"

#define TAG "PN532"

#ifdef CONFIG_PN532_DEBUG
#define PN532_DEBUG(format, ...)                        \
        do {                                            \
                ESP_LOGI(TAG, format, ##__VA_ARGS__);   \
        } while(0)
#define PN532_DEBUG_HEXDUMP(msg, buf, len)              \
        do {                                            \
                ESP_LOGI(TAG, msg);                     \
                esp_log_buffer_hex(TAG, buf, len);      \
        } while(0)
#else
#define PN532_DEBUG(format, args...)            do {} while(0)
#define PN532_DEBUG_HEXDUMP(msg, buf, len)      do {} while(0)
#endif
