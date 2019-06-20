/*
 * This file is subject to the terms of the Nanochip License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              ./LICENSE
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"


static const char *TAG = "VALVE_APP";

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}
