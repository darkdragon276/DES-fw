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

#define BUF_SIZE (200)

static char uart_buffer[BUF_SIZE] = {0};

typedef enum {
    IDLE = 0,
    SET_POS,
    SET_WID,
    SET_HOME,
    SET_DUTY,
    SAVE,
    REP,
} robot_mode_t;

void robot_response(int id_command, char *message);

static robot_mode_t mode = IDLE;
robot_mode_t robot_read_command(int *id_command, char *para)
{
    int data_len = uart_read_bytes(UART_NUM, (uint8_t *)uart_buffer, BUF_SIZE, 1 / portTICK_RATE_MS);

    if (data_len != 0) {
        char command[10] = {0};
        char buff[50] = {0};
        memmove(buff, uart_buffer, data_len);
        if (msg_unpack(buff, data_len) == 0) {
            robot_response((int)(INT16_MAX), "ERROR TRANSMIT");
            return IDLE;
        }
        sscanf(buff, "%d %s %20c", id_command, command, para);
        ESP_LOGI(TAG, "buff:%s, para:%s", uart_buffer, para);
        if (strcmp(command, "SETPOS") == 0) {
            return SET_POS;
        } else if (strcmp(command, "SETWID") == 0) {
            return SET_WID;
        } else if (strcmp(command, "SETHOME") == 0) {
            return SET_HOME;
        } else if (strcmp(command, "SETDUTY") == 0) {
            return SET_DUTY;
        } else if (strcmp(command, "SAVE") == 0) {
            return SAVE;
        } else {
            ESP_LOGE(TAG, "error command: %s", command);
            robot_response(*id_command, "ERROR COMMAND");
        }
    }
    return IDLE;
}

void robot_response(int id_command, char *message)
{
    char *buff = (char *)calloc(BUF_SIZE, sizeof(char));
    int buff_len = snprintf(buff, BUF_SIZE, "%d:%s", id_command, message);
    char *temp = (char *)calloc(100, sizeof(char));
    int temp_len = msg_pack(buff, buff_len, temp);
    uart_write_bytes(UART_NUM, temp, temp_len);
    free(temp);
    free(buff);
}

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
    int id_command = 0;
    char para[50];
    double x, y, z, width;
    int duty, channel;
    while (1) {
        switch (mode) {
        case IDLE:
            memset(para, 0, sizeof(para));
            mode = robot_read_command(&id_command, para);
            break;
        case SET_POS:
            sscanf(para, "%lf %lf %lf", &x, &y, &z);
            if (robot_set_position(x, y, z) == ESP_OK) {
                robot_response(id_command, "PROCESSING");
                mode = REP;
            } else {
                robot_response(id_command, "ERROR ARGUMENT");
                mode = IDLE;
            }
            break;
        case SET_WID:
            sscanf(para, "%lf", &width);
            if (robot_set_cripper_width(width) == ESP_OK) {
                robot_response(id_command, "PROCESSING");
                mode = REP;
            } else {
                robot_response(id_command, "ERROR ARGUMENT");
                mode = IDLE;
            }
            break;
        case SET_HOME:
            robot_set_home();
            robot_response(id_command, "PROCESSING");
            mode = REP;
            break;
        case SET_DUTY:
            sscanf(para, "%d %d", &duty, &channel);
            if (servo_duty_set_lspb_calc(duty, channel - 1) == ESP_OK) {
                robot_response(id_command, "PROCESSING");
                mode = REP;
            } else {
                robot_response(id_command, "ERROR ARGUMENT");
                mode = IDLE;
            }
            break;
        case SAVE:
            // robot_save();
            robot_response(id_command, "PROCESSING");
            mode = REP;
            break;
        case REP:
            if (robot_get_status() == SERVO_STATUS_IDLE) {
                robot_response(id_command, "DONE");
                mode = IDLE;
            } else if (robot_get_status() == SERVO_STATUS_ERROR) {
                robot_response(id_command, "ERROR");
            }
            break;
        default:
            break;
        }
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    servo_init();     // start timer and servo run task

    xTaskCreate(uart_task, "UART-TASK", 8 * 1024, NULL, 6, NULL);
}
