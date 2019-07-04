// #include "servo_control.h"

// #define SERVO_MIN_PULSEWIDTH (500)  // Minimum pulse width in microsecond
// #define SERVO_MAX_PULSEWIDTH (2500) // Maximum pulse width in microsecond
// #define SERVO_MAX_DEGREE (90)

// #define SERVO_PINNUM_0 (15)
// #define SERVO_PINNUM_1 (13)
// #define SERVO_PINNUM_2 (12)
// #define SERVO_PINNUM_3 (14)
// #define SERVO_PINNUM_4 (27)
// #define SERVO_PINNUM_5 (26)
// #define SERVO_PINNUM_6 (25)
// #define SERVO_PINNUM_7 (33)

// #define SERVO_MAX_CHANNEL (6)
// #define SERVO_TIME_DELAY (5) // 5 ms

// #define SERVO_ALL_IDLE (0x3F)
// #define SERVO_ALL_RUNNING (0x00)

// typedef struct {
//     mcpwm_unit_t unit;
//     mcpwm_timer_t timer;
//     mcpwm_io_signals_t io_signal;

//     uint64_t pinnum;
//     mcpwm_operator_t op;
//     int duty_cur;
//     int duty_dst;
//     int step;

//     servo_stt_t stt;
// } servo_ctrl_t;

// static servo_ctrl_t servo_ch[6] = {0};
// static uint32_t servo_time = 1000;
// servo_rqst_t servo_rqst = SERVO_RQST_STOP;
// uint8_t servo_stt_all = 0;

// void servo_init(void)
// {
//     // timer 0,  2 channel
//     servo_ch[0].unit = MCPWM_UNIT_0;
//     servo_ch[0].timer = MCPWM_TIMER_0;
//     servo_ch[0].io_signal = MCPWM0A;
//     servo_ch[0].op = MCPWM_OPR_A;
//     servo_ch[0].pinnum = SERVO_PINNUM_0;

//     servo_ch[1].unit = MCPWM_UNIT_0;
//     servo_ch[1].timer = MCPWM_TIMER_0;
//     servo_ch[1].io_signal = MCPWM0B;
//     servo_ch[1].op = MCPWM_OPR_B;
//     servo_ch[1].pinnum = SERVO_PINNUM_1;

//     // timer 1, 2channel
//     servo_ch[2].unit = MCPWM_UNIT_0;
//     servo_ch[2].timer = MCPWM_TIMER_1;
//     servo_ch[2].io_signal = MCPWM1A;
//     servo_ch[2].op = MCPWM_OPR_A;
//     servo_ch[2].pinnum = SERVO_PINNUM_2;

//     servo_ch[3].unit = MCPWM_UNIT_0;
//     servo_ch[3].timer = MCPWM_TIMER_1;
//     servo_ch[3].io_signal = MCPWM1B;
//     servo_ch[3].op = MCPWM_OPR_B;
//     servo_ch[3].pinnum = SERVO_PINNUM_3;

//     // timer 2, 2 channel
//     servo_ch[4].unit = MCPWM_UNIT_0;
//     servo_ch[4].timer = MCPWM_TIMER_2;
//     servo_ch[4].io_signal = MCPWM2A;
//     servo_ch[4].op = MCPWM_OPR_A;
//     servo_ch[4].pinnum = SERVO_PINNUM_4;

//     servo_ch[5].unit = MCPWM_UNIT_0;
//     servo_ch[5].timer = MCPWM_TIMER_2;
//     servo_ch[5].io_signal = MCPWM2B;
//     servo_ch[5].op = MCPWM_OPR_B;
//     servo_ch[5].pinnum = SERVO_PINNUM_5;

//     mcpwm_config_t pwm_config;
//     pwm_config.frequency = 50; // frequency = 50Hz, i.e. for every servo
//     motor
//                                // time period should be 20ms
//     pwm_config.cmpr_a = 0;     // duty cycle of PWMxA = 0
//     pwm_config.cmpr_b = 0;     // duty cycle of PWMxb = 0
//     pwm_config.counter_mode = MCPWM_UP_COUNTER;
//     pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

//     for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
//         ESP_ERROR_CHECK(
//             mcpwm_gpio_init(servo->unit, servo->io_signal, servo->pinnum));
//         ESP_ERROR_CHECK(mcpwm_init(servo->unit, servo->timer, pwm_config));
//     }
//     ESP_LOGI("SERVO TASK/servo_init", "servo initial and config:  OK");
// }

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

// void servo_set_all_duty_with_time(int *duty, uint32_t time)
// {
//     const char *TAG = "servo_set_all_duty_with_time";
//     if (sizeof(duty) < SERVO_MAX_CHANNEL * sizeof(int)) {
//         ESP_LOGE(TAG, "duty input is shortest: %d", sizeof(duty));
//         return;
//     }

//     for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
//         servo_set_duty(duty[i], servo_ch[i]);
//     }

//     servo_set_time(time);
// }

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

// void servo_step_cal(void)
// {
//     const char *TAG = "servo_step_cal";
//     if (servo_get_stt() == SERVO_STT_STOP) {
//         for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
//             servo_ch[i].step = (servo_ch[i].duty_dst - servo_ch[i].duty_cur)
//             /
//                                (servo_time / SERVO_TIME_DELAY);
//         }
//         _servo_set_run();
//         ESP_LOGI(TAG, "servo start");
//         return;
//     }
//     ESP_LOGE(TAG, "servo is running");
// }

// void servo_delay() { vTaskDelay(SERVO_TIME_DELAY / portTICK_RATE_MS); }

// void servo_run() {}
