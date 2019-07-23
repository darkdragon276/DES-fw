#include "servo_control.h"

#define SERVO_MIN_PULSEWIDTH (500)  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH (2500) // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE (90)

#define SERVO_PINNUM_0 (15)
#define SERVO_PINNUM_1 (13)
#define SERVO_PINNUM_2 (12)
#define SERVO_PINNUM_3 (14)
#define SERVO_PINNUM_4 (27)
#define SERVO_PINNUM_5 (26)
// #define SERVO_PINNUM_6 (25)
// #define SERVO_PINNUM_7 (33)

#define SERVO_MAX_CHANNEL (6)
#define SERVO_TIME_STEP (100) // 100 ms is timer isr step to caculate

#define TIMER_DIVIDER (80)
#define TIMER_SCALE_SEC (1000000U)
#define TIMER_SCALE_MS (1000U)
#define TIMER_SCALE_US (1U)
#define TIMER_CLEAR (1)
#define TIMER_AUTO_RELOAD (1)

#define EVENT_ID_BASE (0x11)

/*
 *
 ****************ENUM DECLARE*******************
 *
 */
typedef enum {
    EVENT_GPIO = 0,
    EVENT_TIMER,
} event_type_t;

typedef enum {
    EVENT_ID_BTN_L = EVENT_ID_BASE,
    EVENT_ID_BTN_R,
    EVENT_ID_BTN_M,
    EVENT_ID_TIMER_SERVO,
} event_id_t;

typedef enum {
    SERVO_STATUS_ERROR = -1,
    SERVO_STATUS_IDLE,
    SERVO_STATUS_RUNNING,
} servo_status_t;

/*
 *
 ****************STRUCT DECLARE*******************
 *
 */
typedef struct {
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_io_signals_t io_signal;
    mcpwm_operator_t op;
    uint64_t pinnum;
} servo_config_t;

// event handle struct
typedef struct {
    event_type_t event_type;
    uint32_t event_id;
} event_handler_t;

typedef struct {
    int duty_current; // duty current
    int duty_target;  // duty target
    int step;

    servo_status_t status;
} servo_channel_ctrl_t;

typedef struct {
    servo_channel_ctrl_t channel[6];

    uint32_t time_fade; // time_step to caculate

    servo_rqst_t user_request;
    servo_status_t status;

} servo_t;
/*
 *
 ****************GLOBAL VARAIABLE DECLARE*******************
 *
 */
static xQueueHandle event_queue = NULL;
/*
 *
 ****************FUNCTION DECLARE*******************
 *
 */
servo_status_t _servo_check_status(servo_t *servo);
servo_status_t _servo_channel_check_status(servo_channel_ctrl_t *servo_channel);
void _servo_step_cal(servo_t *servo);
void _servo_channel_check_duty_error(servo_channel_ctrl_t *servo_channel);

static void _servo_run_task(void *arg);
static void _timer_init(bool auto_reload, double timer_interval, const int TIMER_SCALE);
void IRAM_ATTR _timer_group0_isr(void *para);

void _pwm_parameter_assign(servo_config_t *servo_config);
void _servo_mcpwm_out(servo_t *servo, servo_config_t *servo_config);
/*
 *
 ****************FUNCTION DECLARE*******************
 *
 */

// void servo_set_time(uint32_t time)
// {
//     const char *TAG = "servo_set_time";
//     if (time < 500) {
//         ESP_LOGE(TAG, "time input is short %d < 500ms", time);
//         return;
//     }
//     if (time > 5000) {
//         ESP_LOGE(TAG, "time input is long %d > 5000ms", time);
//         return;
//     }
//     servo_time = time;
//     ESP_LOGI(TAG, "servo set time: %d ms ", time);
// }

// void servo_set_duty(int duty, int channel)
// {
//     const char *TAG = "servo_set_duty";
//     if (duty < SERVO_MIN_PULSEWIDTH) {
//         ESP_LOGE(TAG, "duty input is short %d < 500ms", duty);
//         return;
//     } else if (duty > SERVO_MAX_PULSEWIDTH) {
//         ESP_LOGE(TAG, "duty input is long %d > 2500ms", duty);
//         return;
//     }

//     if (channel < 0 && channel > SERVO_MAX_CHANNEL) {
//         ESP_LOGE(TAG, "channel %d is not available", channel);
//         return;
//     }

//     servo_ch[channel].duty_dst = duty;
//     ESP_LOGI(TAG, "servo set duty: %d ms ", duty);
// }

// // void servo_set_all_duty_with_time(int *duty, uint32_t time)
// // {
// //     const char *TAG = "servo_set_all_duty_with_time";
// //     if (sizeof(duty) < SERVO_MAX_CHANNEL * sizeof(int)) {
// //         ESP_LOGE(TAG, "duty input is shortest: %d", sizeof(duty));
// //         return;
// //     }

// //     for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
// //         servo_set_duty(duty[i], servo_ch[i]);
// //     }

// //     servo_set_time(time);
// // }

// void servo_set_request(servo_rqst_t rqst)
// {
//     const char *TAG = "servo_set_request";
//     if (servo_get_stt() == SERVO_STT_STOP) {
//         servo_rqst = rqst;
//         ESP_LOGI(TAG, "servo request set OK");
//         return;
//     } else if (servo_get_stt() == SERVO_STT_RUN) {
//         if (rqst == SERVO_RQST_STOP) {
//             servo_rqst = rqst;
//             ESP_LOGI(TAG, "servo request set OK");
//             return;
//         }
//     }
//     ESP_LOGE(TAG, "servo is running, can't set orther position");
// }

// servo_rqst_t servo_get_request(void) { return servo_rqst; }

// servo_stt_t servo_get_stt(void)
// {
//     if (servo_stt_all == SERVO_ALL_IDLE) {
//         return SERVO_STT_STOP;
//     }
//     return SERVO_STT_RUN;
// }

// void _servo_set_run() { servo_stt_all = SERVO_ALL_RUNNING; }

/*
 *
 ****************SERVO STATUS CHECK*******************
 *
 */

servo_status_t _servo_channel_check_status(servo_channel_ctrl_t *servo_channel)
{
    const char *TAG = "file: servo_control.c , function: _servo_channel_check_status";
    if (servo_channel->duty_current == 0 || servo_channel->duty_target == 0) {
        ESP_LOGE(TAG, "argument is available. , duty current: %d , duty target: %d", (int)servo_channel->duty_current,
                 (int)servo_channel->duty_target);
        return SERVO_STATUS_ERROR;
    }
    if (abs(servo_channel->duty_current - servo_channel->duty_target) <= abs(servo_channel->step) ||
        servo_channel->step == 0) {
        servo_channel->status = SERVO_STATUS_IDLE;
        return SERVO_STATUS_IDLE;
    }
    return SERVO_STATUS_RUNNING;
}

servo_status_t _servo_check_status(servo_t *servo)
{
    for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
        if (_servo_channel_check_status(&servo->channel[i]) == SERVO_STATUS_IDLE) {
            continue;
        } else if (_servo_channel_check_status(&servo->channel[i]) == SERVO_STATUS_RUNNING) {
            servo->status = SERVO_STATUS_RUNNING;
            return SERVO_STATUS_RUNNING;
        } else {
            servo->status = SERVO_STATUS_ERROR;
            return SERVO_STATUS_ERROR;
        }
    }
    servo->status = SERVO_STATUS_IDLE;
    return SERVO_STATUS_IDLE;
}

void _servo_channel_check_duty_error(servo_channel_ctrl_t *servo_channel)
{
    const char *TAG = "file: servo_control.c , function: _servo_channel_check_duty_error";
    // check current duty
    if (servo_channel->duty_current < SERVO_MIN_PULSEWIDTH) {
        servo_channel->duty_current = SERVO_MIN_PULSEWIDTH;
        ESP_LOGE(TAG, "force current duty is %d ", (int)SERVO_MIN_PULSEWIDTH);
    } else if (servo_channel->duty_current > SERVO_MAX_PULSEWIDTH) {
        servo_channel->duty_current = SERVO_MAX_PULSEWIDTH;
        ESP_LOGE(TAG, "force current duty is %d ", (int)SERVO_MAX_PULSEWIDTH);
    }
    // check target duty
    if (servo_channel->duty_target < SERVO_MIN_PULSEWIDTH) {
        servo_channel->duty_target = SERVO_MIN_PULSEWIDTH;
        ESP_LOGE(TAG, "force current duty is %d ", (int)SERVO_MIN_PULSEWIDTH);
    } else if (servo_channel->duty_target > SERVO_MAX_PULSEWIDTH) {
        servo_channel->duty_target = SERVO_MAX_PULSEWIDTH;
        ESP_LOGE(TAG, "force current duty is %d ", (int)SERVO_MAX_PULSEWIDTH);
    }
}
/*
 *
 ****************CACULATE AND RUNNING*******************
 *
 */

void _servo_step_cal(servo_t *servo)
{
    const char *TAG = "file: servo_control.c , function: _servo_step_cal";
    if (_servo_check_status(servo) == SERVO_STATUS_IDLE) {
        for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
            servo->channel[i].step =
                (servo->channel[i].duty_target - servo->channel[i].duty_current) / (servo->time_fade / SERVO_TIME_STEP);
        }
    } else if (_servo_check_status(servo) == SERVO_STATUS_RUNNING) {
        ESP_LOGE(TAG, "servo is running");
    } else {
        ESP_LOGE(TAG, "error orcur");
    }
}

// set pwm out
void _servo_mcpwm_out(servo_t *servo, servo_config_t *servo_config)
{
    for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
        _servo_channel_check_duty_error(&servo->channel[i]);
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(servo_config[i].unit, servo_config[i].timer, servo_config[i].op,
                                             servo->channel[i].duty_current));
    }
}

// add step per cycle
void _servo_duty_add_step(servo_t *servo)
{
    for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
        servo->channel[i].duty_current += servo->channel[i].step;
        if (servo->channel[i].duty_current < SERVO_MIN_PULSEWIDTH) {
            servo->channel[i].duty_current = SERVO_MIN_PULSEWIDTH;
        }
        if (servo->channel[i].duty_current > SERVO_MAX_PULSEWIDTH) {
            servo->channel[i].duty_current = SERVO_MAX_PULSEWIDTH;
        }
    }
}

/*
 *
 ****************TIMER EVENT HANDLE*******************
 *
 */

void IRAM_ATTR _timer_group0_isr(void *para)
{
    TIMERG0.hw_timer[0].update = 1;
    TIMERG0.int_clr_timers.t0 = 1;
    event_handler_t event_handler = {
        .event_type = EVENT_TIMER,
        .event_id = EVENT_ID_TIMER_SERVO,
    };
    xQueueSendFromISR(event_queue, &event_handler, NULL);
    TIMERG0.hw_timer[0].config.alarm_en = TIMER_ALARM_EN;
}

static void _timer_init(bool auto_reload, double timer_interval, const int TIMER_SCALE)
{
    const char *TAG = "file: servo_control.c , function: _timer_init";
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .intr_type = TIMER_INTR_LEVEL,
        .auto_reload = auto_reload,
    };
    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &config));

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on
       alarm */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timer_interval * TIMER_SCALE);

    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    ESP_ERROR_CHECK(
        timer_isr_register(TIMER_GROUP_0, TIMER_0, _timer_group0_isr, (void *)TIMER_0, ESP_INTR_FLAG_IRAM, NULL));

    ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, TIMER_0));
    ESP_LOGI(TAG, "timer isr init %d ms: OK", (int)timer_interval);
}

/*
 *
 ****************SERVO INIT AND RUN *******************
 *
 */

// assign config parameter
void _pwm_parameter_assign(servo_config_t *servo_config)
{
    const char *TAG = "file: servo_control.c , function: _pwm_parameter_assign";
    memset(servo_config, 0, 6 * sizeof(servo_config_t));

    servo_config[0].unit = MCPWM_UNIT_0;
    servo_config[0].timer = MCPWM_TIMER_0;
    servo_config[0].io_signal = MCPWM0A;
    servo_config[0].op = MCPWM_OPR_A;
    servo_config[0].pinnum = SERVO_PINNUM_0;

    servo_config[1].unit = MCPWM_UNIT_0;
    servo_config[1].timer = MCPWM_TIMER_0;
    servo_config[1].io_signal = MCPWM0B;
    servo_config[1].op = MCPWM_OPR_B;
    servo_config[1].pinnum = SERVO_PINNUM_1;

    // timer 1, 2channel
    servo_config[2].unit = MCPWM_UNIT_0;
    servo_config[2].timer = MCPWM_TIMER_1;
    servo_config[2].io_signal = MCPWM1A;
    servo_config[2].op = MCPWM_OPR_A;
    servo_config[2].pinnum = SERVO_PINNUM_2;

    servo_config[3].unit = MCPWM_UNIT_0;
    servo_config[3].timer = MCPWM_TIMER_1;
    servo_config[3].io_signal = MCPWM1B;
    servo_config[3].op = MCPWM_OPR_B;
    servo_config[3].pinnum = SERVO_PINNUM_3;

    // timer 2, 2 channel
    servo_config[4].unit = MCPWM_UNIT_0;
    servo_config[4].timer = MCPWM_TIMER_2;
    servo_config[4].io_signal = MCPWM2A;
    servo_config[4].op = MCPWM_OPR_A;
    servo_config[4].pinnum = SERVO_PINNUM_4;

    servo_config[5].unit = MCPWM_UNIT_0;
    servo_config[5].timer = MCPWM_TIMER_2;
    servo_config[5].io_signal = MCPWM2B;
    servo_config[5].op = MCPWM_OPR_B;
    servo_config[5].pinnum = SERVO_PINNUM_5;
    ESP_LOGI(TAG, "servo 6 channels is assigned:  OK");
}

void _servo_parameter_assign(servo_t *servo)
{
    memset(servo, 0, sizeof(servo_t));
    for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
        servo->channel[i].duty_current = 1500;
        servo->channel[i].duty_target = 1500;
        servo->channel[i].status = SERVO_STATUS_IDLE;
        servo->channel[i].step = 0;
    }
    servo->status = SERVO_STATUS_IDLE;
    servo->time_fade = 1000; // 1000 ms
    servo->user_request = 0;
}

static void _servo_run_task(void *arg)
{
    const char *TAG = "file: servo_control.c , function: _servo_run_task";
    event_handler_t event_handler;

    servo_t *servo_handle = (servo_t *)malloc(sizeof(servo_t));
    _servo_parameter_assign(servo_handle);

    servo_config_t *servo_config = (servo_config_t *)malloc(6 * sizeof(servo_config_t));
    _pwm_parameter_assign(servo_config);

    ESP_LOGI(TAG, "servo_run_task starting ...");
    while (1) {
        if (xQueueReceive(event_queue, &event_handler, portMAX_DELAY)) {
            if (event_handler.event_type == EVENT_TIMER && event_handler.event_id == EVENT_ID_TIMER_SERVO) {

                for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
                    _servo_channel_check_duty_error(&servo_handle->channel[i]);
                }

                if (_servo_check_status(servo_handle) == SERVO_STATUS_IDLE) {
                    _servo_step_cal(servo_handle);
                }

                _servo_duty_add_step(servo_handle);
                _servo_mcpwm_out(servo_handle, servo_config);
            }
        }
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}

void servo_init(void)
{
    const char *TAG = "file: servo_control.c , function: servo_init";
    servo_config_t *servo_config = (servo_config_t *)malloc(6 * sizeof(servo_config_t));
    _pwm_parameter_assign(servo_config);
    // timer 0,  2 channel

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; // frequency = 50Hz, i.e. for every servo
    // time period should be 20ms
    pwm_config.cmpr_a = 0; // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0; // duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
        ESP_ERROR_CHECK(mcpwm_gpio_init(servo_config[i].unit, servo_config[i].io_signal, servo_config[i].pinnum));
        ESP_ERROR_CHECK(mcpwm_init(servo_config[i].unit, servo_config[i].timer, &pwm_config));
    }
    free(servo_config);
    ESP_LOGI(TAG, "servo 6 channels config:  OK");

    _timer_init(TIMER_AUTO_RELOAD, SERVO_TIME_STEP, TIMER_SCALE_MS);

    event_queue = xQueueCreate(20, sizeof(event_handler_t));

    xTaskCreate(_servo_run_task, "_SERVO_RUN_TASK", 4096, NULL, 5, NULL);
}
