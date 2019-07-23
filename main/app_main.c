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

#define UART_TXD_PINNUM (10)
#define UART_RXD_PINNUM (9)
#define UART_RTS_PINNUM (5)
#define UART_CTS_PINNUM (18)

#define BUF_SIZE (1024)

// static void servo_run_task(void *pv)
// {
//     servo_init();
//     while (1) {
//         if (servo_get_stt() == SERVO_STT_RUN) {
//             servo_run();
//         } else {
//             if (servo_get_request() == SERVO_RQST_RUN) {
//                 servo_step_cal();
//                 servo_run();
//                 ESP_LOGI("SERVO TASK", "request run OK");
//             }
//         }
//         servo_delay();
//     }
// }

// static void uart_task(void *pv)
// {
//     uart_config_t uart_config = {.baud_rate = 115200,
//                                  .data_bits = UART_DATA_8_BITS,
//                                  .parity = UART_PARITY_DISABLE,
//                                  .stop_bits = UART_STOP_BITS_1,
//                                  .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
//     uart_param_config(UART_NUM_1, &uart_config);
//     uart_set_pin(UART_NUM_1, UART_TXD_PINNUM, UART_RXD_PINNUM,
//     UART_RTS_PINNUM,
//                  UART_CTS_PINNUM);
//     uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
//     char *data = (char *)malloc(BUF_SIZE);
//     static int servo_count = 0;
//     static int servo_duty[6] = {0};
//     while (1) {
//         // Read data from the UART
//         int len = uart_read_bytes(UART_NUM_1, (uint8_t *)data, BUF_SIZE,
//                                   20 / portTICK_RATE_MS);
//         // Write data back to the UART
//         if (len != 0) {
//             servo_duty[servo_count] = (uint32_t)atoi(data);

//             if (servo_duty[servo_count] < 500) {
//                 servo_duty[servo_count] = 500;
//             } else if (servo_duty[servo_count] > 2500) {
//                 servo_duty[servo_count] = 2500;
//             }
//             ESP_LOGI("UART_TAG", "duty ms add to servo[%d]: %d", servo_count,
//                      servo_duty[servo_count]);
//             servo_count++;
//             if (servo_count == 6) {
//                 flag = 1;
//                 servo_count = 0;
//             }
//         }
//         vTaskDelay(19 / portTICK_RATE_MS);
//     }
// }

static void servo_run_task(void *arg)
{
    while (1) {
        ESP_LOGI("servo_run_task", "running");
        vTaskDelay(1000 / portTICK_RATE_MS);
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
    servo_init();
    xTaskCreate(servo_run_task, "SERVO-CONTROL-TASK", 4096, NULL, 5, NULL);
    // xTaskCreate(uart_task, "UART-TASK", 4096, NULL, 5, NULL);
}
