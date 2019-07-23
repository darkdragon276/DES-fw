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

typedef enum {
    SERVO_STT_STOP = 0,
    SERVO_STT_RUN,
} servo_stt_t;

typedef enum {
    SERVO_RQST_STOP = 0,
    SERVO_RQST_RUN,
} servo_rqst_t;

extern uint32_t servo_time;

void servo_init(void);

void servo_run();
void servo_step_cal(void);
void servo_delay(void);

servo_stt_t servo_get_stt(void);
void servo_set_request(servo_rqst_t rqst);
servo_rqst_t servo_get_request(void);
// time to change currennt duty to destination duty
void servo_set_time(uint32_t time);
void servo_set_duty(int duty, int channel);
void servo_set_all_duty_with_time(int *duty, uint32_t time);
