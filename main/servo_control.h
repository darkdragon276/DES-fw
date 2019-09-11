#ifndef _SERVO_CONTROL_H_
#define _SERVO_CONTROL_H_

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

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

#define OPTION_UPPER_LIMIT (1)
#define OPTION_UNDER_LIMIT (0)
extern uint32_t servo_time;

void servo_init(void);

// time to change currennt duty to destination duty
void servo_set_all_duty_with_time(int *duty, uint32_t time);

esp_err_t robot_set_position(double x, double y, double z);
esp_err_t robot_set_cripper_width(double width);
esp_err_t servo_duty_set_lspb_calc(int duty, int channel);
// UART
int msg_unpack(char *pkg, int pkg_len, char *buffer);
int msg_pack(char *buff, int buff_len, char *package);

// init and load data default if can't
// find its in flash
esp_err_t servo_nvs_load(void);
esp_err_t servo_nvs_save(bool option, int channel);
esp_err_t servo_nvs_restore(bool option, int channel);
esp_err_t servo_nvs_default(void);

#endif
