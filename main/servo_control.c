#include "servo_control.h"

#define SERVO_MIN_PULSEWIDTH (500)      // Minimum pulse width in us
#define SERVO_MAX_PULSEWIDTH (2500)     // Maximum pulse width in us
#define SERVO_MAX_DEGREE (90)

#define SERVO_PINNUM_0 (15)
#define SERVO_PINNUM_1 (13)
#define SERVO_PINNUM_2 (12)
#define SERVO_PINNUM_3 (14)
#define SERVO_PINNUM_4 (27)
// #define SERVO_PINNUM_5 (26)
// #define SERVO_PINNUM_6 (25)
#define SERVO_PINNUM_5 (33)

#define SERVO_CHANNEL_0 (0)
#define SERVO_CHANNEL_1 (1)
#define SERVO_CHANNEL_2 (2)
#define SERVO_CHANNEL_3 (3)
#define SERVO_CHANNEL_4 (4)
#define SERVO_CHANNEL_5 (5)

#define SERVO_MAX_CHANNEL (6)
#define SERVO_TIME_STEP (100)     // 100 ms is timer isr step to caculate

#define TIMER_DIVIDER (80)
#define TIMER_SCALE_SEC (1000000U)
#define TIMER_SCALE_MS (1000U)
#define TIMER_SCALE_US (1U)
#define TIMER_CLEAR (1)
#define TIMER_AUTO_RELOAD (1)

#define EVENT_ID_BASE (0x11)

#define mutex_lock(x) while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x) xSemaphoreGive(x)
#define mutex_create() xSemaphoreCreateMutex()
#define mutex_destroy(x) vQueueDelete(x)

#define PI acos(-1)

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
    int duty_current;     // duty current
    int duty_target;      // duty target
    int step;

    servo_status_t status;
} servo_channel_ctrl_t;

typedef struct {
    servo_channel_ctrl_t channel[6];

    uint32_t time_fade;     // time_step to caculate

    servo_rqst_t user_request;
    servo_status_t status;
    SemaphoreHandle_t lock;
} servo_t;
/*
 *
 ****************GLOBAL VARAIABLE DECLARE*******************
 *
 */
static xQueueHandle event_queue = NULL;
static servo_t servo_handler = {0};
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

void _servo_set_time(uint32_t time_fade);
void _servo_set_duty(int duty, int channel);
int Deg2Duty(double deg);
/*
 *
 ****************FUNCTION ACCESS DATA*******************
 *
 */

void _servo_set_time(uint32_t time_fade)
{
    const char *TAG = "file: servo_control.c , function: servo_set_time";
    if (time_fade < 500) {
        ESP_LOGE(TAG, "time input is short %d < 500ms", time_fade);
        return;
    }
    if (time_fade > 5000) {
        ESP_LOGE(TAG, "time input is long %d > 5000ms", time_fade);
        return;
    }

    mutex_lock(servo_handler.lock);
    servo_handler.time_fade = time_fade;
    mutex_unlock(servo_handler.lock);

    ESP_LOGI(TAG, "servo set time: %d ms ", time_fade);
}

// function set duty for a channel with non-locking
void _servo_set_duty(int duty, int channel)
{
    const char *TAG = "file: servo_control.c , function: servo_set_duty";
    if (duty < SERVO_MIN_PULSEWIDTH) {
        ESP_LOGE(TAG, "duty input is short %d < 500us", duty);
        return;
    } else if (duty > SERVO_MAX_PULSEWIDTH) {
        ESP_LOGE(TAG, "duty input is long %d > 2500us", duty);
        return;
    }

    if (channel < 0 && channel > SERVO_MAX_CHANNEL) {
        ESP_LOGE(TAG, "channel %d is not available", channel);
        return;
    }

    servo_handler.channel[channel].duty_target = duty;
    ESP_LOGI(TAG, "servo set duty: %d us ", duty);
}

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
        ESP_LOGE(TAG, "argument is available. , duty current: %d , duty target: %d",
                 (int)servo_channel->duty_current, (int)servo_channel->duty_target);
        return SERVO_STATUS_ERROR;
    }
    if (abs(servo_channel->duty_current - servo_channel->duty_target) <=
            abs(servo_channel->step) ||
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
        } else if (_servo_channel_check_status(&servo->channel[i]) ==
                   SERVO_STATUS_RUNNING) {
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
                (int)(servo->channel[i].duty_target - servo->channel[i].duty_current) /
                (int)(servo->time_fade / SERVO_TIME_STEP);
        }
        ESP_LOGD(TAG, "step[6] is : %d ,%d ,%d ,%d ,%d ,%d", servo->channel[0].step,
                 servo->channel[1].step, servo->channel[2].step, servo->channel[3].step,
                 servo->channel[4].step, servo->channel[5].step);
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
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(servo_config[i].unit, servo_config[i].timer,
                                             servo_config[i].op,
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
    ESP_ERROR_CHECK(timer_isr_register(TIMER_GROUP_0, TIMER_0, _timer_group0_isr,
                                       (void *)TIMER_0, ESP_INTR_FLAG_IRAM, NULL));

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
    servo->time_fade = 1000;     // 1000 ms
    servo->user_request = 0;
    servo->lock = mutex_create();
}

/*
 *
 ***************************************SERVO RUN TASK
 **************************************
 *
 */

static void _servo_run_task(void *arg)
{
    const char *TAG = "file: servo_control.c , function: _servo_run_task";

    servo_config_t *servo_config = (servo_config_t *)malloc(6 * sizeof(servo_config_t));
    _pwm_parameter_assign(servo_config);

    ESP_LOGI(TAG, "servo_run_task starting ...");
    while (1) {

        event_handler_t event_handler = {0};
        if (xQueueReceive(event_queue, &event_handler, portMAX_DELAY)) {
            if (event_handler.event_type == EVENT_TIMER &&
                event_handler.event_id == EVENT_ID_TIMER_SERVO) {

                mutex_lock(servo_handler.lock);

                for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
                    _servo_channel_check_duty_error(&servo_handler.channel[i]);
                }

                if (_servo_check_status(&servo_handler) == SERVO_STATUS_IDLE) {
                    _servo_step_cal(&servo_handler);
                }

                _servo_duty_add_step(&servo_handler);
                // ESP_LOGI(TAG, "pwm out");
                // export pwm
                _servo_mcpwm_out(&servo_handler, servo_config);
                ESP_LOGD(TAG, "pwm out");

                mutex_unlock(servo_handler.lock);
            }
        }

        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

/*
 *
 ***************************************SERVO INIT
 *******************************************
 *
 */

void servo_init(void)
{
    const char *TAG = "file: servo_control.c , function: servo_init";
    servo_config_t *servo_config = (servo_config_t *)malloc(6 * sizeof(servo_config_t));
    _pwm_parameter_assign(servo_config);
    // timer 0,  2 channel

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;     // frequency = 50Hz, i.e. for every servo
    // time period should be 20ms
    pwm_config.cmpr_a = 0;     // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;     // duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
        ESP_ERROR_CHECK(mcpwm_gpio_init(servo_config[i].unit, servo_config[i].io_signal,
                                        servo_config[i].pinnum));
        ESP_ERROR_CHECK(
            mcpwm_init(servo_config[i].unit, servo_config[i].timer, &pwm_config));
    }
    free(servo_config);
    ESP_LOGI(TAG, "servo 6 channels config:  OK");

    _timer_init(TIMER_AUTO_RELOAD, SERVO_TIME_STEP, TIMER_SCALE_MS);

    event_queue = xQueueCreate(20, sizeof(event_handler_t));

    _servo_parameter_assign(&servo_handler);

    xTaskCreate(_servo_run_task, "_SERVO_RUN_TASK", 4096, NULL, 5, NULL);
}

/*
 *
 ****************MATH FUNCTION *******************
 *
 */
double atan2d(double y, double x) { return atan2(y, x) * 180.0 / PI; }

double cosd(double x) { return cos(x * PI / 180.0); }

double sind(double x) { return sin(x * PI / 180.0); }

// convert deg to pulse with value
// [0:90] degree => [1000:2000] us
int Deg2Duty(double deg)
{
    double temp;
    temp = deg / 90;
    temp += 1;
    temp *= 1000;
    return (int)temp;
}

/********************************************************************************/
/***********************   Kinetic Calculate funciton   *************************/
// preprocess to put x y z position and theta3
// function return pointer of xyzther3 array
esp_err_t robot_set_position(double x, double y)
{
    const char *TAG = "file: servo_control.c , function: robot_set_position";
    mutex_lock(servo_handler.lock);
    //******************reprocessing argument************************//
    static double a = 1.0, d = 0.0, d1 = 8.7, a2 = 10.5, a3 = 10.0, d5 = 20.5;
    double z = 1.0, theta3 = 45;
    // double r = sqrt(x * x + y * y);
    // if( r <= 5)
    //   theta3 = 90;
    // else if( r < 40 )
    //   theta3 = floor( 90.0 - (r-5)*80.0/35.0 );
    // else
    // {
    // }

    //******************orient kinetic************************//

    double r = sqrt(x * x + y * y - d * d), phi = atan2d(y, x) + atan2d(d, r), s = d1 - z;
    r -= a;
    /*3 edge of triangle*/
    double m1 = sqrt(a2 * a2 + a3 * a3 + 2 * a2 * a3 * cosd(theta3)), m2 = d5,
           m3 = sqrt(r * r + s * s);
    /*sum of theta2 + theta3 + theta4*/
    double beta1 = atan2d(s, r), cbeta2 = -(m1 * m1 - m2 * m2 - m3 * m3) / (2 * m2 * m3),
           sbeta2 = sqrt(1 - cbeta2 * cbeta2), beta2 = atan2d(sbeta2, cbeta2),
           beta = 90.0 + beta1 + beta2;
    /*assign T matrix*/
    double cp = cosd(phi), sp = sind(phi), cb = cosd(beta), sb = sind(beta);

    /*
    // T =  |cb * cp    -sp     sb * cp    x|
    //      |cb * sp     cp     sb * sp    y|
    //      |-sb         0      cb         z|
    */
    double nx = cb * cp, ny = cb * sp;     //, nz = -sb;
    double ox = -sp, oy = cp;              //, oz = 0;
    double ax = sb * cp, ay = sb * sp, az = cb;
    double dx = x, dy = y, dz = z;

    // move cripper to d4
    dx -= d5 * ax;
    dy -= d5 * ay;
    dz -= d5 * az;

    //******************inverse kinetic************************//
    /*theta1 */
    // double r = sqrt(dx * dx + dy * dy - d * d);
    double theta1 = atan2d(dy, dx) + atan2d(d, r);
    double s1 = sind(theta1), c1 = cosd(theta1);
    /*theta3 and theta2*/
    r -= a;
    s = dz - d1;
    double c3 = (r * r + s * s - a2 * a2 - a3 * a3) / (2 * a2 * a3),
           s3 = -sqrt(1 - c3 * c3);

    theta3 = atan2d(s3, c3);
    double theta2 = atan2d(s, r) - atan2d(a3 * s3, a2 + a3 * c3);
    /*theta4*/
    double theta23 = theta2 + theta3, c23 = cosd(theta23), s23 = sind(theta23),
           s4 = az * s23 + ax * c23 * c1 + ay * c23 * s1,
           c4 = -(az * c23 - ax * s23 * c1 - ay * s23 * s1);

    double theta4 = atan2d(s4, c4);
    /*theta5*/
    double s5 = nx * s1 - ny * c1, c5 = ox * s1 - oy * c1;
    double theta5 = atan2d(s5, c5);

    //******************conver to duty************************//

    if (theta1 >= -45 && theta1 <= 45)
        theta1 += (45 + atan2d(-2.9, 13.5));     // bu` goc 1
    else {
        ESP_LOGE(TAG, "theta1 out of range[-45 45]: %lf", theta1);
        mutex_unlock(servo_handler.lock);
        return ESP_ERR_INVALID_ARG;
    }
    if (theta2 >= 0 && theta2 <= 90)
        theta2 = 90 - theta2;
    else {
        // error
        ESP_LOGE(TAG, "theta2 out of range[0 90]: %lf", theta2);
        mutex_unlock(servo_handler.lock);
        return ESP_ERR_INVALID_ARG;
    }
    if (theta3 >= -90 && theta3 <= 0)
        theta3 = 90 + theta3 - 8;
    else {
        // error
        ESP_LOGE(TAG, "theta3 out of range[-90 0]: %lf", theta3);
        mutex_unlock(servo_handler.lock);
        return ESP_ERR_INVALID_ARG;
    }
    if (theta4 >= -90 && theta4 <= 0)
        theta4 = 90 + theta4 - 5;
    else {
        // error
        ESP_LOGE(TAG, "theta4 out of range[-90 0]: %lf", theta4);
        mutex_unlock(servo_handler.lock);
        return ESP_ERR_INVALID_ARG;
    }
    theta5 = 45;

    // memcpy(debug, theta, sizeof(double) * 5);
    ESP_LOGI(TAG, "set duty of 5 channel in robot");
    _servo_set_duty(Deg2Duty(theta1), SERVO_CHANNEL_0);
    _servo_set_duty(Deg2Duty(theta2), SERVO_CHANNEL_1);
    _servo_set_duty(Deg2Duty(theta3), SERVO_CHANNEL_2);
    _servo_set_duty(Deg2Duty(theta4), SERVO_CHANNEL_3);
    _servo_set_duty(Deg2Duty(theta5), SERVO_CHANNEL_4);
    mutex_unlock(servo_handler.lock);
    return ESP_OK;
}

/****** CRIPPER WIDE WITH PULSE ******
 * PULSE (us)    1900    1800    1700    1600    1500    1400    1300    1200    1100
 *------------------------------------------------------------------------------------
 * WIDE  (cm)    0,5      1,3     2,5     3,5     4,4     5,1     5,5     5,9     6
 */

#define ROBOT_CRIPPER_MAX_WIDTH (6.0)
#define ROBOT_CRIPPER_MIN_WIDTH (1.0)

void robot_set_cripper_width(double width)
{
    // const char *TAG = "file: servo_control.c , function: robot_set_cripper_width";
    // if( width < ROBOT_CRIPPER_MIN_WIDTH || width > ROBOT_CRIPPER_MAX_WIDTH )
    // {
    //     ESP_LOGE(TAG, "width is: %lf out of range [1:6]", width);
    //     return;
    // }

    // int duty = 0;
    mutex_lock(servo_handler.lock);
    _servo_set_duty(width, SERVO_CHANNEL_5);
    mutex_unlock(servo_handler.lock);
}
