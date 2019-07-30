/*
 * This file is subject to the terms of the Nanochip License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              ./LICENSE
 */
#include "gfx.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"

#include "servo_control.h"

static const char *TAG = "ROBOT";

#define UART_TXD_PINNUM ()                     //(10)
#define UART_RXD_PINNUM ()                     //(9)
#define UART_RTS_PINNUM UART_PIN_NO_CHANGE     //(5)
#define UART_CTS_PINNUM UART_PIN_NO_CHANGE     //(18)

#define BUF_SIZE (1024)

static void uart_task(void *pv)
{

    uart_config_t uart_config = {.baud_rate = 115200,
                                 .data_bits = UART_DATA_8_BITS,
                                 .parity = UART_PARITY_DISABLE,
                                 .stop_bits = UART_STOP_BITS_1,
                                 .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART_TXD_PINNUM, UART_RXD_PINNUM, UART_RTS_PINNUM,
                 UART_CTS_PINNUM);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
    char *data = (char *)malloc(BUF_SIZE);
    uint32_t duty = 0;
    while (1) {
        // Read data from the UART
        int len =
            uart_read_bytes(UART_NUM_1, (uint8_t *)data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        if (len != 0) {
            duty = (uint32_t)atoi(data);

            if (duty < 500) {
                duty = 500;
            } else if (duty > 2500) {
                duty = 2500;
            }
            robot_set_cripper_width(duty);
            ESP_LOGI("UART_TAG", "duty ms add to servo %d", duty);
            uart_write_bytes(UART_NUM_1, (const char *)data, len);
        }
        vTaskDelay(19 / portTICK_RATE_MS);
    }
}

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

    servo_init();     // start timer and servo run task

    ESP_LOGI(TAG, "uart_task starting ...");
    xTaskCreate(uart_task, "UART-TASK", 4096, NULL, 5, NULL);
}
