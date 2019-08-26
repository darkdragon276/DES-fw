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
#define servo_handle_tIME_STEP (100)     // 100 ms is timer isr step to caculate

#define TIMER_DIVIDER (80)
#define TIMER_SCALE_SEC (1000000U)
#define TIMER_SCALE_MS (1000U)
#define TIMER_SCALE_US (1U)
#define TIMER_CLEAR (1)
#define TIMER_AUTO_RELOAD (1)

#define EVENT_ID_BASE (0x11)

// semaphore macro
#define mutex_lock(x) while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x) xSemaphoreGive(x)
#define mutex_create() xSemaphoreCreateMutex()
#define mutex_destroy(x) vQueueDelete(x)

// event bit wait macro
#define event_clear(event_handler, bit) xEventGroupClearBits(event_handler, bit)
#define event_set(event_handler, bit) xEventGroupSetBits(event_handler, bit)
#define event_wait(event_handler, bit) while (!xEventGroupWaitBits(event_handler, bit, false, false, portMAX_DELAY))

/*
 *
 ****************ENUM DECLARE*******************
 *
 */

typedef enum {
    EVENT_TIMER_SERVO = EVENT_ID_BASE,
    EVENT_CALIB_MANUAL,
} event_type_t;

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

typedef struct {
    int duty_current;     // duty current
    int duty_target;      // duty target
    int step;
    servo_status_t status;
} servo_channel_ctrl_t;

typedef struct {
    double scale;
    double bias;
    double under_limit;
    double upper_libit;
} servo_channel_calib_t;

typedef struct {
    servo_channel_ctrl_t channel[6];
    servo_channel_calib_t calib[6];
    uint32_t time_fade;     // time_step to caculate
    servo_rqst_t user_request;
    servo_status_t status;
    SemaphoreHandle_t lock;
    EventGroupHandle_t event_wait;
} servo_handle_t;

/*
 *
 ****************GLOBAL VARAIABLE DECLARE*******************
 *
 */

static servo_handle_t servo_handler;
xQueueHandle event_queue;
static const int EVT_SERVO_RUN = BIT0;
static const int EVT_SERVO_CALIB = BIT1;

/*
 *
 ****************FUNCTION DECLARE*******************
 *
 */
servo_status_t _servo_check_status(servo_handle_t *servo);
servo_status_t _servo_channel_check_status(servo_channel_ctrl_t *servo_channel);
void _servo_step_cal(servo_handle_t *servo);
void _servo_channel_check_duty_error(servo_channel_ctrl_t *servo_channel);

static void _servo_run_task(void *arg);
static void _timer_init(bool auto_reload, double timer_interval, const int TIMER_SCALE);
void IRAM_ATTR _timer_group0_isr(void *para);

void _pwm_parameter_assign(servo_config_t *servo_config);
void _servo_mcpwm_out(servo_handle_t *servo, servo_config_t *servo_config);

void _servo_set_time(uint32_t time_fade);
void _servo_set_duty(int duty, int channel);

// math function
#define PI acos(-1)
double atan2d(double y, double x) { return atan2(y, x) * 180.0 / PI; }
double cosd(double x) { return cos(x * PI / 180.0); }
double sind(double x) { return sin(x * PI / 180.0); }
double acosd(double x) { return acos(x) * 180.0 / PI; }
int _math_deg2duty(double deg);
bool _math_in_circle(double x, double y, double x0, double y0, double R0);
bool _math_in_workspace(double d, double z, double theta, double r1, double r2, double r3);
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
    if (abs(servo_channel->duty_current - servo_channel->duty_target) <= abs(servo_channel->step) || servo_channel->step == 0) {
        servo_channel->status = SERVO_STATUS_IDLE;
        return SERVO_STATUS_IDLE;
    }
    return SERVO_STATUS_RUNNING;
}

servo_status_t _servo_check_status(servo_handle_t *servo)
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

void _servo_step_cal(servo_handle_t *servo)
{
    const char *TAG = "file: servo_control.c , function: _servo_step_cal";
    if (_servo_check_status(servo) == SERVO_STATUS_IDLE) {
        for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
            servo->channel[i].step = (int)(servo->channel[i].duty_target - servo->channel[i].duty_current) /
                                     (int)(servo->time_fade / servo_handle_tIME_STEP);
        }
        ESP_LOGD(TAG, "step[6] is : %d ,%d ,%d ,%d ,%d ,%d", servo->channel[0].step, servo->channel[1].step,
                 servo->channel[2].step, servo->channel[3].step, servo->channel[4].step, servo->channel[5].step);
    } else if (_servo_check_status(servo) == SERVO_STATUS_RUNNING) {
        ESP_LOGE(TAG, "servo is running");
    } else {
        ESP_LOGE(TAG, "error orcur");
    }
}

// set pwm out
void _servo_mcpwm_out(servo_handle_t *servo, servo_config_t *servo_config)
{
    for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
        _servo_channel_check_duty_error(&servo->channel[i]);
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(servo_config[i].unit, servo_config[i].timer, servo_config[i].op,
                                             servo->channel[i].duty_current));
    }
}

// add step per cycle
void _servo_duty_add_step(servo_handle_t *servo)
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
    event_type_t event_handler = EVENT_TIMER_SERVO;
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
    ESP_ERROR_CHECK(timer_isr_register(TIMER_GROUP_0, TIMER_0, _timer_group0_isr, (void *)TIMER_0, ESP_INTR_FLAG_IRAM, NULL));

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
    ESP_LOGI(TAG, "servo's 6 channels are assigned:  OK");
}

void _servo_parameter_assign(servo_handle_t *servo)
{
    int home[6] = {1500, 1000, 2000, 2000, 1500, 1500};
    memset(servo, 0, sizeof(servo_handle_t));
    for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
        servo->channel[i].duty_current = home[i];
        servo->channel[i].duty_target = home[i];
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
 *********************************SERVO RUN TASK************************************
 *
 */

static void _servo_run_task(void *arg)
{
    const char *TAG = "file: servo_control.c , function: _servo_run_task";

    servo_config_t *servo_config = (servo_config_t *)malloc(6 * sizeof(servo_config_t));
    _pwm_parameter_assign(servo_config);
    // load he so calib neu co
    ESP_LOGI(TAG, "servo_run_task starting ...");
    while (1) {

        event_type_t event_handler;
        if (xQueueReceive(event_queue, &event_handler, portMAX_DELAY)) {
            if (event_handler == EVENT_TIMER_SERVO) {
                mutex_lock(servo_handler.lock);
                for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
                    _servo_channel_check_duty_error(&servo_handler.channel[i]);
                }
                if (_servo_check_status(&servo_handler) == SERVO_STATUS_IDLE) {
                    _servo_step_cal(&servo_handler);
                }
                _servo_duty_add_step(&servo_handler);
                // export pwm
                ESP_LOGI(TAG, "pwm out");
                _servo_mcpwm_out(&servo_handler, servo_config);
                mutex_unlock(servo_handler.lock);
            } else if (event_handler == EVENT_CALIB_MANUAL) {
                timer_pause(TIMER_GROUP_0, TIMER_0);
                vTaskDelay(10000 / portTICK_RATE_MS);
                // load he so calib neu co
                timer_start(TIMER_GROUP_0, TIMER_0);
            }
        }
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}

esp_err_t robot_calib_manual_request(void)
{
    const char *TAG = "file: servo_control.c , function: robot_calib_manual_request";
    event_type_t event_handler = EVENT_CALIB_MANUAL;
    xQueueSendToFront(event_queue, &event_handler, 0);
    ESP_LOGI(TAG, "Request Calib manual sending ...");
    return ESP_OK;
}
/*
*********************************SERVO INIT**********************************
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
        ESP_ERROR_CHECK(mcpwm_gpio_init(servo_config[i].unit, servo_config[i].io_signal, servo_config[i].pinnum));
        ESP_ERROR_CHECK(mcpwm_init(servo_config[i].unit, servo_config[i].timer, &pwm_config));
    }
    free(servo_config);
    ESP_LOGI(TAG, "servo 6 channels config:  OK");

    _timer_init(TIMER_AUTO_RELOAD, servo_handle_tIME_STEP, TIMER_SCALE_MS);
    event_queue = xQueueCreate(20, sizeof(event_type_t));
    servo_handler.event_wait = xEventGroupCreate();
    _servo_parameter_assign(&servo_handler);
    xTaskCreate(_servo_run_task, "_SERVO_RUN_TASK", 4096, NULL, 5, NULL);
}

/*
 *
 ******************************* MATH FUNCTION ************************
 *
 */

// convert deg to pulse with value
// [0:90] degree => [1000:2000] us
int _math_deg2duty(double deg)
{
    double temp;
    temp = deg / 90;
    temp += 1;
    temp *= 1000;
    return (int)temp;
}
// check point(x, y) is in circle(x0, y0, RO) ?
bool _math_in_circle(double x, double y, double x0, double y0, double R0)
{
    if (((x - x0) * (x - x0) + (y - y0) * (y - y0)) <= R0 * R0) {
        return true;
    }
    return false;
}
// check point(d, z) is in workspace of 3link planar robot
// have first argument is theta, distance between link is r1, r2, r3
bool _math_in_workspace(double d, double z, double theta, double r1, double r2, double r3)
{
    // first circle , in the top. point in workspace is out of this circles
    double dtemp = (r1 + r2) * cosd(theta);
    double ztemp = (r1 + r2) * sind(theta);
    double Rtemp = r3;
    // = false if point is in workspace
    bool check1 = _math_in_circle(d, z, dtemp, ztemp, Rtemp);

    // second circle , in the bottom. point in workspace is in of this circles
    dtemp = r1 * cosd(theta) + r2 * sind(theta);
    ztemp = r1 * sind(theta) - r2 * cosd(theta);
    Rtemp = r3;
    // = true if point is in workspace
    bool check2 = _math_in_circle(d, z, dtemp, ztemp, Rtemp);

    // third circle , in the right. point in workspace is in of this circles
    dtemp = r1 * cosd(theta);
    ztemp = r1 * sind(theta);
    Rtemp = sqrt(r2 * r2 + r3 * r3 - 2 * r2 * r3 * cosd(135));
    // = true if point is in workspace
    bool check3 = _math_in_circle(d, z, dtemp, ztemp, Rtemp);

    // first circle , in the left. point in workspace is out of this circles
    dtemp = r1 * cosd(theta);
    ztemp = r1 * sind(theta);
    Rtemp = sqrt(r2 * r2 + r3 * r3 - 2 * r2 * r3 * cosd(45));
    // = false if point is in workspace
    bool check4 = _math_in_circle(d, z, dtemp, ztemp, Rtemp);

    if (~check1 & check2 & check3 & ~check4) {
        return true;
    }
    return false;
}

// calib function
double _math_scale_argument(double arg, double scale, double bias, double under_limit, double upper_limit)
{
    double temp = arg * scale + bias;
    if (temp > upper_limit) {
        return upper_limit;
    }
    if (temp < under_limit) {
        return under_limit;
    }
    return temp;
}
/********************************************************************************/
/***********************   Kinetic Calculate funciton   *************************/
// preprocess to put x y z position
// function return pointer of xyzther3 array
esp_err_t robot_set_position(double x, double y, double z)
{
    // static double a = 1.0, d = 0.0, d1 = 8.7, a2 = 10.5, a3 = 10.0, d5 = 20.5;
    const char *TAG = "file: servo_control.c , function: robot_set_position";
    mutex_lock(servo_handler.lock);
    double a2 = 10.5, a3 = 9.8, a4 = 20.0;
    double d = sqrt(x * x + y * y);     // z = 0;

    double theta1 = atan2d(y, x);     // + atan2d(d, r);
    double theta2 = 90;
    while (_math_in_workspace(d, z, theta2, a2, a3, a4) == false) {
        theta2--;
        if (theta2 == 0) {
            ESP_LOGE(TAG, "position is out of workspace");
            return ESP_ERR_INVALID_ARG;
        }
    }

    double z2 = a2 * sind(theta2);
    double d2 = a2 * cosd(theta2);
    double r24 = sqrt((z - z2) * (z - z2) + (d - d2) * (d - d2));
    double c4 = (r24 * r24 - a3 * a3 - a4 * a4) / (2 * a3 * a4);
    double s4 = sqrt(1 - c4 * c4);
    // theta4 < 0 => clockwise, -135 => -45
    double theta4 = atan2d(-s4, c4);
    // theta3 < 0 => clockwise
    double phi = acosd((r24 * r24 + a3 * a3 - a4 * a4) / (2 * r24 * a3));     // 0 => 180
    double alpha = atan2d(z - z2, d - d2);                                    // -90 => 90
    double theta3 = -(theta2 - phi - alpha);                                  // => theta3: -90 => 180
    // theta5
    double theta5 = 45;

    // calib
    ESP_LOGD(TAG, "argument caculate: theta1: %.2lf, theta2: %.2lf, theta3: %.2lf, theta4: %.2lf, theta5: %.2lf", theta1, theta2,
             theta3, theta4, theta5);
    theta1 = _math_scale_argument(theta1, 1, -45, 0, 90);     // real [1000:2000] us = [45:135] => 0: 90
    theta2 = _math_scale_argument(theta2, -1, 90, 0, 90);     // real [1000:2000] us = [90:0]   => 0: 90
    theta3 = _math_scale_argument(theta3, 1, 90, 0, 90);      // real [1000:2000] us = [-90:0]  => 0: 90
    theta4 = _math_scale_argument(theta4, 1, 135, 0, 90);     // real [1000:2000] us = [-135:-45] => 0: 90
    theta5 = _math_scale_argument(theta5, 1, 0, 0, 90);       // real [1000:2000] us = [0:90] => 0: 90
    ESP_LOGD(TAG, "argument after scale off: theta1: %.2lf, theta2: %.2lf, theta3: %.2lf, theta4: %.2lf, theta5: %.2lf", theta1,
             theta2, theta3, theta4, theta5);

    // convert to duty
    _servo_set_duty(_math_deg2duty(theta1), SERVO_CHANNEL_0);
    _servo_set_duty(_math_deg2duty(theta2), SERVO_CHANNEL_1);
    _servo_set_duty(_math_deg2duty(theta3), SERVO_CHANNEL_2);
    _servo_set_duty(_math_deg2duty(theta4), SERVO_CHANNEL_3);
    _servo_set_duty(_math_deg2duty(theta5), SERVO_CHANNEL_4);
    mutex_unlock(servo_handler.lock);
    return ESP_OK;
}

/****** CRIPPER WIDE WITH PULSE ******
 * PULSE (us)    1900    1800    1700    1600    1500    1400    1300    1200    1100
 *------------------------------------------------------------------------------------
 * WIDE  (cm)    0,5      1,3     2,5     3,5     4,4     5,1     5,5     5,9     6
 */

#define ROBOT_CRIPPER_MAX_WIDTH (6.0)
#define ROBOT_CRIPPER_MIN_WIDTH (2.0)

// error +-1cm , this function caculate base on reels data.
int _width2duty(double width)
{
    const char *TAG = "file: servo_control.c , function: _width2duty";
    if (width < ROBOT_CRIPPER_MIN_WIDTH || width > ROBOT_CRIPPER_MAX_WIDTH) {
        ESP_LOGE(TAG, "width is: %.1lf out of range [%.1lf:%.1lf]", width, ROBOT_CRIPPER_MIN_WIDTH, ROBOT_CRIPPER_MAX_WIDTH);
        return 0;
    }

    // parameter is caculated based y = ax + b;
    // link google sheets:
    // https://docs.google.com/spreadsheets/d/11esbf92rwcrUhVwUt
    // Z1Ag2tBRmxVL04o7HjGpEqcSPc/edit#gid=0
    double a1 = -100, b1 = 2010;
    double a2 = -167, b2 = 2290;
    int duty = 0;

    if (width < 3.0) {
        duty = (int)1800;
    } else if (width >= 3.0 && width <= 4.5) {
        duty = (int)(width * a1 + b1);
    } else if (width > 4.5 && width <= 5.5) {
        duty = (int)(width * a2 + b2);
    } else if (width > 5.5) {
        duty = (int)1200;
    }
    return duty;
}

esp_err_t robot_set_cripper_width(double width)
{
    const char *TAG = "file: servo_control.c , function: robot_set_cripper_width";
    int duty = _width2duty(width);
    if (duty == 0) {
        ESP_LOGE(TAG, "Invalid argument");
        return ESP_ERR_INVALID_ARG;
    }
    mutex_lock(servo_handler.lock);
    _servo_set_duty(duty, SERVO_CHANNEL_5);
    ESP_LOGI(TAG, "width set: %.1lf", width);
    mutex_unlock(servo_handler.lock);
    return ESP_OK;
}

/*
 *
 **************** UART pack and unpack function *******************
 *
 */

#define MSG_MIN_PKG_LEN (3)

// this function pack your buffer to package lead by pkg pointer
// this function can't check buffer_len vs sizeof(buff)
int msg_pack(char *buff, int buff_len, char *package)
{
    const char *TAG = "file: servo_control.c , function: msg_pack";
    if (buff == NULL) {
        ESP_LOGE(TAG, "buff pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (package == NULL) {
        ESP_LOGE(TAG, "input package is null pointer");
        return 0;
    }

    // mutex_lock(servo_handler.lock);
    // caculate package length to return
    int pkg_len = 0;
    for (int i = 0; i < buff_len; i++) {
        if (buff[i] == 0x7D || buff[i] == 0x7E || buff[i] == 0x7F) {
            pkg_len++;
        }
    }
    pkg_len += (buff_len + 2);
    char *pkg = (char *)malloc(pkg_len * sizeof(char));

    // packing
    pkg_len = 0;
    pkg[pkg_len++] = 0x7E;
    for (int i = 0; i < buff_len; i++) {

        if (buff[i] == 0x7D || buff[i] == 0x7E || buff[i] == 0x7F) {

            pkg[pkg_len++] = 0x7D;
            pkg[pkg_len++] = buff[i] ^ 0x20;

        } else {
            pkg[pkg_len++] = buff[i];
        }
    }
    pkg[pkg_len++] = 0x7F;

    memcpy(package, pkg, pkg_len);
    free(pkg);
    // mutex_unlock(servo_handler.lock);

    return pkg_len;
}

// this function unpack your package to buffer lead by buff pointer
// this function can't check pkg_len vs sizeof(pkg)
int msg_unpack(char *pkg, int pkg_len, char *buffer)
{
    const char *TAG = "file: servo_control.c , function: msg_unpack";
    // check lenght
    if (pkg_len < MSG_MIN_PKG_LEN) {
        ESP_LOGE(TAG, "package_len: %d < %d", pkg_len, MSG_MIN_PKG_LEN);
        return 0;
    }
    if (buffer == NULL) {
        ESP_LOGE(TAG, "input buffer is null pointer");
        return 0;
    }

    // check begin , end character
    if (pkg[0] != 0x7E || pkg[pkg_len - 1] != 0x7F) {
        ESP_LOGE(TAG, "begin char: %X != 0x7E or end char: %X != 0x7F", pkg[0], pkg[pkg_len - 1]);
        return 0;
    }

    // allocation buffer
    // mutex_lock(servo_handler.lock);
    int buff_len = pkg_len;
    for (int i = 0; i < pkg_len; i++) {
        if (pkg[i] == 0x7D || pkg[i] == 0x7E || pkg[i] == 0x7F) {
            buff_len--;
        }
    }
    char *buff = (char *)malloc(buff_len * sizeof(char));

    // unpacking
    buff_len = 0;
    for (uint32_t i = 1; i < pkg_len - 1; i++) {
        if (pkg[i] == 0x7E || pkg[i] == 0x7F) {
            ESP_LOGE(TAG, "pkg[%d] = %X", i, pkg[i]);
            return 0;
        } else if (pkg[i] == 0x7D) {
            buff[buff_len++] = pkg[++i] ^ 0x20;
        } else {
            buff[buff_len++] = pkg[i];
        }
    }

    memcpy(buffer, buff, buff_len);
    free(buff);
    // mutex_lock(servo_handler.unlock);

    return buff_len;
}
