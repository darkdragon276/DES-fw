/*
 * This file is subject to the terms of the Nanochip License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              ./LICENSE
 */
// #include "gfx.h"

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

#define UART_TXD_PINNUM (10)     //(1)
#define UART_RXD_PINNUM (9)      //(3)
#define UART_RTS_PINNUM (5)      // UART_PIN_NO_CHANGE
#define UART_CTS_PINNUM (18)     // UART_PIN_NO_CHANGE
#define UART_NUM UART_NUM_1

#define BUF_SIZE (1024)

static void uart_task(void *pv)
{
    ESP_LOGI(TAG, "uart_task starting ...");
    uart_config_t uart_config = {.baud_rate = 115200,
                                 .data_bits = UART_DATA_8_BITS,
                                 .parity = UART_PARITY_DISABLE,
                                 .stop_bits = UART_STOP_BITS_1,
                                 .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TXD_PINNUM, UART_RXD_PINNUM, UART_RTS_PINNUM, UART_CTS_PINNUM);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    char *data = (char *)calloc(BUF_SIZE, sizeof(char));
    while (1) {
        memset(data, 0, BUF_SIZE);
        if (uart_read_bytes(UART_NUM, (uint8_t *)data, BUF_SIZE, 20 / portTICK_RATE_MS) != 0) {
            ESP_LOGI(TAG, "%s", data);

            if (strcmp(data, "AT POS") == 0) {
                memset(data, 0, BUF_SIZE);
                if (uart_read_bytes(UART_NUM, (uint8_t *)data, BUF_SIZE, 10000 / portTICK_RATE_MS) != 0) {
                    ESP_LOGI(TAG, "%s", data);

                    double x, y, z;
                    sscanf(data, "%lf %lf %lf", &x, &y, &z);
                    robot_set_position(x, y, z);

                    ESP_LOGI(TAG, "AT OK");
                } else {
                    ESP_LOGE(TAG, "AT POS: time out");
                }

            } else if (strcmp(data, "AT WIDTH") == 0) {
                memset(data, 0, BUF_SIZE);
                if (uart_read_bytes(UART_NUM, (uint8_t *)data, BUF_SIZE, 10000 / portTICK_RATE_MS) != 0) {
                    ESP_LOGI(TAG, "%s", data);

                    double width;
                    sscanf(data, "%lf", &width);
                    robot_set_cripper_width(width);

                    ESP_LOGI(TAG, "AT OK");
                } else {
                    ESP_LOGE(TAG, "AT WIDTH: time out");
                }
            } else if (strcmp(data, "AT DUTY") == 0) {
                memset(data, 0, BUF_SIZE);
                if (uart_read_bytes(UART_NUM, (uint8_t *)data, BUF_SIZE, 10000 / portTICK_RATE_MS) != 0) {
                    ESP_LOGI(TAG, "%s", data);

                    int duty, channel;
                    sscanf(data, "%d %d", &duty, &channel);
                    servo_duty_set_lspb_calc(duty, channel);

                    ESP_LOGI(TAG, "AT OK");
                } else {
                    ESP_LOGE(TAG, "AT DUTY: time out");
                }

            } else if (strcmp(data, "AT SAVE") == 0) {
                memset(data, 0, BUF_SIZE);
                if (uart_read_bytes(UART_NUM, (uint8_t *)data, BUF_SIZE, 10000 / portTICK_RATE_MS) != 0) {
                    ESP_LOGI(TAG, "%s", data);

                    int channel;
                    char comment[10];
                    sscanf(data, "%s %d", comment, &channel);
                    if (strcmp(comment, "UPPER") == 0) {
                        servo_nvs_save(OPTION_UPPER_LIMIT, channel);
                    } else if (strcmp(comment, "UNDER") == 0) {
                        servo_nvs_save(OPTION_UNDER_LIMIT, channel);
                    }
                    ESP_LOGI(TAG, "AT OK");
                } else {
                    ESP_LOGE(TAG, "AT SAVE: time out");
                }
            } else if (strcmp(data, "AT REST") == 0) {
                memset(data, 0, BUF_SIZE);
                if (uart_read_bytes(UART_NUM, (uint8_t *)data, BUF_SIZE, 10000 / portTICK_RATE_MS) != 0) {
                    ESP_LOGI(TAG, "%s", data);

                    int channel;
                    char comment[10];
                    sscanf(data, "%s %d", comment, &channel);
                    // if (strcmp(comment, "UPPER") == 0) {
                    //     servo_nvs_save(OPTION_UPPER_LIMIT, channel);
                    // } else if (strcmp(comment, "UNDER") == 0) {
                    //     servo_nvs_save(OPTION_UNDER_LIMIT, channel);
                    // }
                    servo_nvs_default();
                    ESP_LOGI(TAG, "AT OK");
                } else {
                    ESP_LOGE(TAG, "AT REST: time out");
                }
            } else {
                ESP_LOGI(TAG, "COMMENT IS NON-AVAILABLE");
            }

            vTaskDelay(10 / portTICK_RATE_MS);
        }
    }
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    servo_init();     // start timer and servo run task

    xTaskCreate(uart_task, "UART-TASK", 4096, NULL, 6, NULL);
}

// test case uart slip
// char data[4] = {0x30, 0x40, 0x7D, 0x7F};

// char *package = (char *)malloc(2 * sizeof(char));
// int pkg_len = msg_pack(data, 4, package);
// for (int i = 0; i < pkg_len; i++) {
//     ESP_LOGI("debug", "pkg[%d]: %X ", i, package[i]);
// }

// char *buff = (char *)malloc(2 * sizeof(char));
// int buff_len = msg_unpack(package, pkg_len, buff);
// for (int i = 0; i < buff_len; i++) {
//     ESP_LOGI("debug", "buff[%d]: %X ", i, buff[i]);
// }
