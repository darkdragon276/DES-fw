#ifndef _SERVO_CONTROL_H_
#define _SERVO_CONTROL_H_

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/uart.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_storage.h"

extern uint32_t servo_time;

void servo_init(void);

// time to change currennt duty to destination duty
void servo_set_all_duty_with_time(int *duty, uint32_t time);

esp_err_t robot_set_position(double x, double y, double z);
esp_err_t robot_set_cripper_width(double width);

esp_err_t robot_calib_manual_request(void);

// UART
int msg_unpack(char *pkg, int pkg_len, char *buffer);
int msg_pack(char *buff, int buff_len, char *package);

// NVS
esp_err_t nvs_load();
void nvs_save();
esp_err_t nvs_set_timeout(int timeout_ms);

#endif
