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
#define SERVO_TIME_STEP (20)     // 20 ms is timer isr step to caculate
#define SERVO_NVS_MAGIC (0x27069700)
#define DEFAULT_UPPER_LIMIT (2000)
#define DEFAULT_UNDER_LIMIT (1000)

#define TIMER_DIVIDER (80)
#define TIMER_SCALE_SEC (1000000U)
#define TIMER_SCALE_MS (1000U)
#define TIMER_SCALE_US (1U)
#define TIMER_CLEAR (1)
#define TIMER_AUTO_RELOAD (1)

#define NVS_SAVE_TIME (3000)     //  60 second per save

#define EVENT_ID_BASE (0x11)

// semaphore macro
#define mutex_lock(x) while (xSemaphoreTake(x, portMAX_DELAY) != pdPASS)
#define mutex_unlock(x) xSemaphoreGive(x)
#define mutex_create() xSemaphoreCreateMutex()
#define mutex_destroy(x) vQueueDelete(x)

// // event bit wait macro
// #define event_clear(event_handler, bit) xEventGroupClearBits(event_handler, bit)
// #define event_set(event_handler, bit) xEventGroupSetBits(event_handler, bit)
// #define event_wait(event_handler, bit) while (!xEventGroupWaitBits(event_handler, bit, false, false, portMAX_DELAY))

/*
 *
 ****************ENUM DECLARE*******************
 *
 */

typedef enum {
    EVENT_TIMER_SERVO = EVENT_ID_BASE,
    EVENT_NVS_SAVE,
} event_type_t;

/*
 *
 ****************STRUCT DECLARE*******************
 *
 */
typedef struct {
    double a;
    double P0;
    double Pf;
    int tf;
    int tb;
} math_lspb_vector_t;

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
    math_lspb_vector_t lspb;
    servo_status_t status;
    int time_count;
} servo_channel_ctrl_t;

typedef struct {
    double scale;
    double bias;
    double under_limit;
    double upper_limit;
} servo_channel_calib_t;

typedef struct {
    servo_channel_ctrl_t channel[6];
    servo_channel_calib_t duty_calib[5];
    uint32_t time_full;     // time_step to caculate
    uint32_t time_balance;
    servo_status_t status;
    int nvs_magic;
    double cripper_len;
} servo_handle_t;

/*
 *
 ******************GLOBAL VARAIABLE DECLARE*******************
 *
 */

static SemaphoreHandle_t servo_lock;
static servo_handle_t servo_handler;
static servo_config_t servo_config_pv[6];
static esp_storage_handle_t storage_handle = NULL;
static int nvs_time_save = 0;
xQueueHandle event_queue;

/*
 *
 ****************FUNCTION DECLARE*******************
 *
 */
void _servo_set_duty(servo_handle_t *servo);
servo_status_t _servo_channel_check_status(servo_channel_ctrl_t *servo_channel);
void _servo_channel_check_duty_error(servo_channel_ctrl_t *servo_channel);

static void _servo_run_task(void *arg);
static void _timer_init(bool auto_reload, double timer_interval, const int TIMER_SCALE);
void IRAM_ATTR _timer_group0_isr(void *para);

void _pwm_config_default(servo_config_t *servo_config);
void _servo_param_set_default(servo_handle_t *servo);
void _servo_mcpwm_out(servo_handle_t *servo, servo_config_t *servo_config);
esp_err_t _servo_nvs_save_all(void);

// math function
#define PI acos(-1)
double atan2d(double y, double x) { return atan2(y, x) * 180.0 / PI; }
double cosd(double x) { return cos(x * PI / 180.0); }
double sind(double x) { return sin(x * PI / 180.0); }
double acosd(double x) { return acos(x) * 180.0 / PI; }
int _math_deg2duty(double deg, servo_channel_calib_t calib);
double _math_scale(double arg, double scale, double bias, double under_limit, double upper_limit);
bool _math_in_circle(double x, double y, double x0, double y0, double R0);
bool _math_in_workspace(double d, double z, double theta, double r1, double r2, double r3);

/*
 *
 ****************************************FUNCTION ACCESS DATA*******************************************
 *
 */

esp_err_t robot_set_time(int time_full)
{
    const char *TAG = "file: servo_control.c , function: robot_set_time";
    if (time_full < 500) {
        ESP_LOGE(TAG, "time input is short %d < 500ms", time_full);
        return ESP_ERR_INVALID_ARG;
    }
    if (time_full > 5000) {
        ESP_LOGE(TAG, "time input is long %d > 5000ms", time_full);
        return ESP_ERR_INVALID_ARG;
    }

    mutex_lock(servo_lock);
    servo_handler.time_full = time_full;
    servo_handler.time_balance = time_full * 30 / 100;
    mutex_unlock(servo_lock);

    ESP_LOGI(TAG, "servo set time: %d ms ", time_full);
    return ESP_OK;
}

// function for lspb calculator => path planning
void _math_lspb_vector_calc(int current_duty, int target_duty, int time_full, int time_balance,
                            math_lspb_vector_t *lspb_vector)
{
    const char *TAG = "file: servo_control.c , function: _math_lspb_vector_calc";
    ESP_LOGD(TAG, "path planning calculating");
    double tf_ = (double)time_full / SERVO_TIME_STEP;
    double tb_ = (double)time_balance / SERVO_TIME_STEP;
    double P0_ = (double)current_duty;
    double Pf_ = (double)target_duty;

    if (tf_ <= 0 || tb_ <= 0) {
        ESP_LOGE(TAG, "tf:%.1lf or tb:%.1lf input is under zero", tf_, tb_);
        return;
    }
    if (tf_ < tb_) {
        ESP_LOGE(TAG, "tf:%.1lf must be > tb:%.1lf", tf_, tb_);
    }

    // V <= 2 (pf - p0)/tf and V >= (pf - p0)/tf => chon 1.5
    lspb_vector->a = (Pf_ - P0_) / (tb_ * (tf_ - tb_));
    lspb_vector->P0 = P0_;
    lspb_vector->Pf = Pf_;
    lspb_vector->tf = (int)tf_;
    lspb_vector->tb = (int)tb_;
    ESP_LOGD(TAG, "a: %.1lf, P0: %.0lf, Pf: %.0lf, tf: %d, tb: %d", lspb_vector->a, lspb_vector->P0, lspb_vector->Pf,
             lspb_vector->tf, lspb_vector->tb);
}

// function path_planning
int _math_path_planning(double a, double P0, double Pf, int *time_count, int tf, int tb)
{
    const char *TAG = "file: servo_control.c , function: _math_path_planning";
    ESP_LOGD(TAG, "path planning caculate");
    if (a == 0) {
        ESP_LOGW(TAG, "warning input: a = %.2lf", a);
        return (int)P0;     // stop
    }

    double temp = 0;
    double T = (double)(*time_count);
    if (T < 0) {
        ESP_LOGE(TAG, "error T %.0lf", T);
        return (int)P0;     // stop
    } else if (T <= tb) {
        temp = P0 + 0.5 * a * T * T;
        ESP_LOGD(TAG, "velocity up");
    } else if (T <= (tf - tb)) {
        temp = P0 + 0.5 * a * (double)tb * (double)tb + a * (double)tb * (T - tb);
        ESP_LOGD(TAG, "velocity balance");
    } else if (T <= tf) {
        temp = Pf - 0.5 * a * (T - (double)tf) * (T - (double)tf);
        ESP_LOGD(TAG, "velocity down");
    } else if (T > tf) {
        return (int)Pf;
    }
    (*time_count)++;
    return (int)temp;
}

// function set duty for a channel with non-locking
esp_err_t servo_duty_set_lspb_calc(int duty, int channel)
{
    const char *TAG = "file: servo_control.c , function: servo_duty_set_lspb_calc";
    if (duty < SERVO_MIN_PULSEWIDTH) {
        ESP_LOGE(TAG, "duty input is short %d < 500us", duty);
        return ESP_ERR_INVALID_ARG;
    } else if (duty > SERVO_MAX_PULSEWIDTH) {
        ESP_LOGE(TAG, "duty input is long %d > 2500us", duty);
        return ESP_ERR_INVALID_ARG;
    }

    if (channel < 0 || channel > SERVO_MAX_CHANNEL) {
        ESP_LOGE(TAG, "channel %d is not available", channel);
        return ESP_ERR_INVALID_ARG;
    }
    servo_handler.channel[channel].duty_target = duty;
    ESP_LOGD(TAG, "servo %d set duty: %d us ", channel, duty);

    // step calculation
    _math_lspb_vector_calc(servo_handler.channel[channel].duty_current, duty, servo_handler.time_full,
                           servo_handler.time_balance, &servo_handler.channel[channel].lspb);
    servo_handler.channel[channel].time_count = 0;
    return ESP_OK;
}
/*
 *
 ********************************************SERVO STATUS CHECK*****************************************
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
    int sub_duty = abs(servo_channel->duty_current - servo_channel->duty_target);
    if (sub_duty == 0) {
        servo_channel->status = SERVO_STATUS_IDLE;
        ESP_LOGD(TAG, "CHANNEL SERVO IS IDLE");
        return SERVO_STATUS_IDLE;
    }
    return SERVO_STATUS_RUNNING;
}

void _servo_set_duty(servo_handle_t *servo)
{
    const char *TAG = "file: servo_control.c , function: _servo_set_duty";
    servo_status_t channel_status;
    int status = 0;
    for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
        channel_status = _servo_channel_check_status(&servo->channel[i]);
        if (channel_status == SERVO_STATUS_IDLE) {
        } else if (channel_status == SERVO_STATUS_RUNNING) {
            int temp = _math_path_planning(servo->channel[i].lspb.a, servo->channel[i].lspb.P0,
                                           servo->channel[i].lspb.Pf, &servo->channel[i].time_count,
                                           servo->channel[i].lspb.tf, servo->channel[i].lspb.tb);
            if (i == 2) {
                ESP_LOGD(TAG, "step: %d", temp - servo->channel[i].duty_current);
                ESP_LOGD(TAG, "time_count: %d", servo->channel[i].time_count);
            }
            servo->channel[i].duty_current = temp;

            if (servo->channel[i].duty_current < SERVO_MIN_PULSEWIDTH) {
                servo->channel[i].duty_current = SERVO_MIN_PULSEWIDTH;
            }
            if (servo->channel[i].duty_current > SERVO_MAX_PULSEWIDTH) {
                servo->channel[i].duty_current = SERVO_MAX_PULSEWIDTH;
            }
            servo->status = SERVO_STATUS_RUNNING;
            status++;
        } else {
            servo->status = SERVO_STATUS_ERROR;
            ESP_LOGE(TAG, "servo status ERROR");
            return;
        }
    }

    if (status > 0) {
        servo->status = SERVO_STATUS_RUNNING;
    } else {
        servo->status = SERVO_STATUS_IDLE;
    }
}

servo_status_t robot_get_status() { return servo_handler.status; }

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
 **************************************SET STEP AND EXPORT PWM****************************************
 *
 */

// set pwm out
void _servo_mcpwm_out(servo_handle_t *servo, servo_config_t *servo_config)
{
    const char *TAG = "file: servo_control.c , function: _servo_mcpwm_out";
    for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
        _servo_channel_check_duty_error(&servo->channel[i]);
        if(servo_config[i].unit != MCPWM_UNIT_0) {
            _pwm_config_default(servo_config);
        }
        esp_err_t error = mcpwm_set_duty_in_us(servo_config[i].unit, servo_config[i].timer, servo_config[i].op,
                                               servo->channel[i].duty_current);
        if( error != ESP_OK) {
            ESP_LOGE(TAG, "error unit: %d", servo_config[i].unit);
            break;
        }
    }
    ESP_LOGD(TAG, "%d     %d     %d     %d     %d", servo->channel[0].duty_current, servo->channel[1].duty_current,
             servo->channel[2].duty_current, servo->channel[3].duty_current, servo->channel[4].duty_current);
    ESP_LOGD(TAG, "%d     %d     %d     %d     %d", servo->channel[0].duty_target, servo->channel[1].duty_target,
             servo->channel[2].duty_target, servo->channel[3].duty_target, servo->channel[4].duty_target);
}

/*
 *
 *********************************************TIMER EVENT HANDLE**************************************************
 *
 */

void IRAM_ATTR _timer_group0_isr(void *para)
{
    TIMERG0.hw_timer[0].update = 1;
    TIMERG0.int_clr_timers.t0 = 1;
    event_type_t event_handler = EVENT_TIMER_SERVO;
    xQueueSendFromISR(event_queue, &event_handler, NULL);
    TIMERG0.hw_timer[0].config.alarm_en = TIMER_ALARM_EN;
    if (nvs_time_save-- == 0) {
        nvs_time_save = NVS_SAVE_TIME;
        event_handler = EVENT_NVS_SAVE;
        xQueueSendFromISR(event_queue, &event_handler, NULL);
    }
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
 *****************************************ASSIGN PARAMATER ****************************************
 *
 */

// assign config parameter
void _pwm_config_default(servo_config_t *servo_config)
{
    const char *TAG = "file: servo_control.c , function: _pwm_config_default";
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

// assign parameter of servo handle
void _servo_param_set_default(servo_handle_t *servo)
{
    memset(servo, 0, sizeof(servo_handle_t));
    int home[6] = {1500, 1050, 1980, 2100, 1500, 1900};
    int upper[5] = {1970, 2100, 1980, 2100, 2000};
    int under[5] = {950, 1050, 800, 1020, 1000};
    for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
        servo->channel[i].duty_current = home[i];
        servo->channel[i].duty_target = home[i];
        servo->channel[i].status = SERVO_STATUS_IDLE;
        servo->channel[i].lspb.a = 0;
        servo->channel[i].lspb.P0 = 0;
        servo->channel[i].lspb.Pf = 0;
        servo->channel[i].lspb.tb = 0;
        servo->channel[i].lspb.tf = 0;
        servo->channel[i].time_count = 0;
    }
    // non cripper
    for (int i = 0; i < SERVO_MAX_CHANNEL - 1; i++) {
        servo->duty_calib[i].scale = 1;
        servo->duty_calib[i].bias = 0;
        servo->duty_calib[i].under_limit = under[i];
        servo->duty_calib[i].upper_limit = upper[i];
    }
    servo->status = SERVO_STATUS_IDLE;
    servo->time_full = 2000;       // ms
    servo->time_balance = 800;     // ms
    servo->nvs_magic = SERVO_NVS_MAGIC;
    servo->cripper_len = 5.84;
}
/*
 *
 ****************************************SERVO RUN TASK********************************************
 *
 */
static void _servo_run_task(void *arg)
{
    const char *TAG = "file: servo_control.c , function: _SERVO_RUN_TASK";
    ESP_LOGI(TAG, "servo_run_task start ...");
    // check duty
    _servo_param_set_default(&servo_handler);
    // for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
    //     if (servo_handler.channel[i].duty_current != servo_handler.channel[i].duty_target) {
    //         _servo_param_set_default(&servo_handler);
    //         break;
    //     }
    // }
    while (1) {
        event_type_t event_handler;
        if (xQueueReceive(event_queue, &event_handler, portMAX_DELAY)) {
            if (event_handler == EVENT_TIMER_SERVO) {
                ESP_LOGD(TAG, "EVENT SERVO RUN");
                mutex_lock(servo_lock);
                for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
                    _servo_channel_check_duty_error(&servo_handler.channel[i]);
                }
                _servo_set_duty(&servo_handler);
                _servo_mcpwm_out(&servo_handler, servo_config_pv);
                mutex_unlock(servo_lock);
            } else if (event_handler == EVENT_NVS_SAVE) {
                // _servo_nvs_save_all();
            }
        }
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}
/*
*********************************SERVO INIT**********************************
*/

void servo_init(void)
{
    const char *TAG = "file: servo_control.c , function: servo_init";
    // servo_config_t *servo_config_pv = (servo_config_t *)calloc(6, sizeof(servo_config_t));
    _pwm_config_default(servo_config_pv);
    // timer 0,  2 channel

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;     // frequency = 50Hz, i.e. for every servo
    // time period should be 20ms
    pwm_config.cmpr_a = 0;     // duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;     // duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    for (int i = 0; i < SERVO_MAX_CHANNEL; i++) {
        ESP_ERROR_CHECK(
            mcpwm_gpio_init(servo_config_pv[i].unit, servo_config_pv[i].io_signal, servo_config_pv[i].pinnum));
        ESP_ERROR_CHECK(mcpwm_init(servo_config_pv[i].unit, servo_config_pv[i].timer, &pwm_config));
    }

    ESP_LOGI(TAG, "servo 6 channels config:  OK");

    nvs_time_save = NVS_SAVE_TIME;
    _timer_init(TIMER_AUTO_RELOAD, SERVO_TIME_STEP, TIMER_SCALE_MS);
    event_queue = xQueueCreate(20, sizeof(event_type_t));
    servo_lock = mutex_create();
    servo_nvs_load();
    // _servo_param_set_default(&servo_handler);
    xTaskCreate(_servo_run_task, "_SERVO_RUN_TASK", 8 * 1024, NULL, 5, NULL);
}

/*
 *
 *********************************************** MATH FUNCTION ************************************************
 *
 */

// convert deg to pulse with value
// [0:90] degree => [under_limit:upper_limit] us
int _math_deg2duty(double deg, servo_channel_calib_t calib)
{
    double top = calib.upper_limit, bot = calib.under_limit;
    double temp = deg / 90.0 * (top - bot) + bot;
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

// scale argument from math caculation to real argument of servo
double _math_scale(double arg, double scale, double bias, double under_limit, double upper_limit)
{
    double temp = arg * scale + bias;
    if (temp > upper_limit) {
        return -1;
    }
    if (temp < under_limit) {
        return -1;
    }
    return temp;
}

/*
 *
 * ***************************************Kinetic Calculate funciton**********************************************
 *
 */
// preprocess to put x y z position
// function return pointer of xyzther3 array
esp_err_t robot_set_position(double x, double y, double z)
{
    // static double a = 1.0, d = 0.0, d1 = 8.7, a2 = 10.5, a3 = 10.0, d5 = 20.5;
    const char *TAG = "file: servo_control.c , function: robot_set_position";
    ESP_LOGI(TAG, "position set: x: %.2lf, y: %.2lf, z: %.2lf", x, y, z);
    mutex_lock(servo_lock);
    z = z - 8.7;
    y = y + 7.94;
    double theta[5];
    double a1 = 0.915 ; // O0 to O1
    double a2 = 10.225, a3 = 9.7;
    double a4 = 14.6 + servo_handler.cripper_len;
    double d = sqrt(x * x + y * y) - a1;     // z = 0;

    theta[0] = atan2d(y, x);     // + atan2d(d, r);
    theta[1] = 90;
    while (_math_in_workspace(d, z, theta[1], a2, a3, a4) == false) {
        theta[1]--;
        if (theta[1] == 0) {
            ESP_LOGE(TAG, "position is out of workspace");
            mutex_unlock(servo_lock);
            return ESP_ERR_INVALID_ARG;
        }
    }

    double z2 = a2 * sind(theta[1]);
    double d2 = a2 * cosd(theta[1]);
    double r24 = sqrt((z - z2) * (z - z2) + (d - d2) * (d - d2));
    double c4 = (r24 * r24 - a3 * a3 - a4 * a4) / (2 * a3 * a4);
    double s4 = sqrt(1 - c4 * c4);
    // theta[3] < 0 => clockwise, -135 => -45
    theta[3] = atan2d(-s4, c4);
    // theta[2] < 0 => clockwise
    double phi = acosd((r24 * r24 + a3 * a3 - a4 * a4) / (2 * r24 * a3));     // 0 => 180
    double alpha = atan2d(z - z2, d - d2);                                    // -90 => 90
    theta[2] = -(theta[1] - phi - alpha);                                     // => theta[2]: -90 => 180
    // theta[4]
    theta[4] = 45;

    // convert arguments
    ESP_LOGI(TAG, "theta:  theta[0]: %.2lf, theta[1]: %.2lf, theta[2]: %.2lf, theta[3]: %.2lf, theta[4]: %.2lf",
             theta[0], theta[1], theta[2], theta[3], theta[4]);
    theta[0] = _math_scale(theta[0], 1, -45, 0, 90);     // real [1000:2000] us = [45:135] => 0: 90
    theta[1] = _math_scale(theta[1], -1, 90, 0, 90);     // real [1000:2000] us = [90:0]   => 0: 90
    theta[2] = _math_scale(theta[2], 1, 90, 0, 90);      // real [1000:2000] us = [-90:0]  => 0: 90
    theta[3] = _math_scale(theta[3], 1, 135, 0, 90);     // real [1000:2000] us = [-135:-45] => 0: 90
    theta[4] = _math_scale(theta[4], 1, 0, 0, 90);       // real [1000:2000] us = [0:90] => 0: 90
    ESP_LOGD(TAG, "scale off: theta[0]: %.2lf, theta[1]: %.2lf, theta[2]: %.2lf, theta[3]: %.2lf, theta[4]: %.2lf",
             theta[0], theta[1], theta[2], theta[3], theta[4]);
    // convert to duty
    int duty[5];
    for (int i = 0; i < SERVO_MAX_CHANNEL - 1; i++) {
        if (theta[i] == -1) {
            ESP_LOGE(TAG, "theta [%d] == -1", i);
            mutex_unlock(servo_lock);
            return ESP_ERR_INVALID_ARG;
        }
        duty[i] = _math_deg2duty(theta[i], servo_handler.duty_calib[i]);
    }
    ESP_LOGI(TAG, "duty : duty[0]: %d, duty[1]: %d, duty[2]: %d, duty[3]: %d, duty[4]: %d", duty[0],
             duty[1], duty[2], duty[3], duty[4]);

    // set duty to run servo
    // i = SERVO_CHANNEL_[I]
    for (int i = 0; i < SERVO_MAX_CHANNEL - 1; i++) {
        servo_duty_set_lspb_calc(duty[i], i);
    }
    // set time to zero
    mutex_unlock(servo_lock);
    return ESP_OK;
}

// preprocess to put x y z position and angle
// function return pointer of xyzther3 array
esp_err_t robot_set_position_with_angle(double x, double y, double z, double angle)
{
    const char *TAG = __func__;     //__func__
    ESP_LOGI(TAG, "position set: x: %.2lf, y: %.2lf, z: %.2lf", x, y, z);
    mutex_lock(servo_lock);
    z = z - 8.7;
    y = y + 7.94;
    double theta[5];
    double a1 = 0.915;     // O0 to O1
    double a2 = 10.225, a3 = 9.7;
    double a4 = 14.6 + servo_handler.cripper_len;

    double d = sqrt(x * x + y * y) - a1;     // z = 0;
    theta[0] = atan2d(y, x);
    // theta[1]
    double z_ = z + a4;
    double a23 = sqrt( z_*z_ + d*d );
    if( a23 > a2 + a3) {
        ESP_LOGE(TAG, "a23 > a2 + a3 ");
        mutex_unlock(servo_lock);
        return ESP_ERR_INVALID_ARG;
    }

    double alpha = atan2d(d, z_);
    double phi = acosd( (a2*a2 + a23*a23 - a3*a3) / (2*a2*a23));
    theta[1] = 90 - (alpha - phi);

    // theta[2]
    double beta = acosd( (a2*a2 + a3*a3 - a23*a23) / (2*a2*a3));
    if( beta < 90) {
        ESP_LOGE(TAG, "beta %lf  < 90", beta);
        mutex_unlock(servo_lock);
        return ESP_ERR_INVALID_ARG;
    }
    theta[2] = -(180.0 - beta);        // theta[2] [0:90]

    // theta[3]
    theta[3] = - (beta + phi - alpha);
    // theta[4]
    theta[4] = angle;

    // convert arguments
    ESP_LOGI(TAG, "theta:  theta[0]: %.2lf, theta[1]: %.2lf, theta[2]: %.2lf, theta[3]: %.2lf, theta[4]: %.2lf",
             theta[0], theta[1], theta[2], theta[3], theta[4]);
    theta[0] = _math_scale(theta[0], 1, -45, 0, 90);     // real [1000:2000] us = [45:135] => 0: 90
    theta[1] = _math_scale(theta[1], -1, 90, 0, 90);     // real [1000:2000] us = [90:0]   => 0: 90
    theta[2] = _math_scale(theta[2], 1, 90, 0, 90);      // real [1000:2000] us = [-90:0]  => 0: 90
    theta[3] = _math_scale(theta[3], 1, 135, 0, 90);     // real [1000:2000] us = [-135:-45] => 0: 90
    theta[4] = _math_scale(theta[4], 1, 0, 0, 90);       // real [1000:2000] us = [0:90] => 0: 90
    ESP_LOGD(TAG, "scale off: theta[0]: %.2lf, theta[1]: %.2lf, theta[2]: %.2lf, theta[3]: %.2lf, theta[4]: %.2lf",
             theta[0], theta[1], theta[2], theta[3], theta[4]);
    // convert to duty
    int duty[5];
    for (int i = 0; i < SERVO_MAX_CHANNEL - 1; i++) {
        if (theta[i] == -1) {
            ESP_LOGE(TAG, "theta [%d] == -1", i);
            mutex_unlock(servo_lock);
            return ESP_ERR_INVALID_ARG;
        }
        duty[i] = _math_deg2duty(theta[i], servo_handler.duty_calib[i]);
    }
    ESP_LOGI(TAG, "duty : duty[0]: %d, duty[1]: %d, duty[2]: %d, duty[3]: %d, duty[4]: %d", duty[0],
             duty[1], duty[2], duty[3], duty[4]);

    // set duty to run servo
    // i = SERVO_CHANNEL_[I]
    for (int i = 0; i < SERVO_MAX_CHANNEL - 1; i++) {
        servo_duty_set_lspb_calc(duty[i], i);
    }
    // set time to zero
    mutex_unlock(servo_lock);
    return ESP_OK;
}

esp_err_t robot_set_home()
{
    int home[5] = {1500, 1050, 1980, 2100, 1500};
    for (int i = 0; i < SERVO_MAX_CHANNEL - 1; i++) {
        servo_duty_set_lspb_calc(home[i], i);
    }
    return ESP_OK;
}

/************************************* CRIPPER WIDE WITH PULSE *********************************************
 * PULSE (us)    1900    1800    1700    1600    1500    1400    1300    1200    1100
 *------------------------------------------------------------------------------------
 * WIDE  (cm)    0.97    1.41    2.40    3.65    4.39    5.00    5.43    5.84    5.96
 *------------------------------------------------------------------------------------
 * ADD-LENG(cm)  5.84    5.76    5.54    5.19    4.80    4.38    3.87    3.35    3.01
 */

#define ROBOT_CRIPPER_MAX_WIDTH (6.0)
#define ROBOT_CRIPPER_MIN_WIDTH (2.0)

// error +-1cm , this function caculate base on reels data.
int _width2duty(double width)
{
    const char *TAG = "file: servo_control.c , function: _width2duty";
    if (width < ROBOT_CRIPPER_MIN_WIDTH || width > ROBOT_CRIPPER_MAX_WIDTH) {
        ESP_LOGE(TAG, "width is: %.1lf out of range [%.1lf:%.1lf]", width, ROBOT_CRIPPER_MIN_WIDTH,
                 ROBOT_CRIPPER_MAX_WIDTH);
        return 0;
    }
    double duty_ref[9] = {1900, 1800, 1700, 1600, 1500, 1400, 1300, 1200, 1100};
    double wid_ref[9] =  {0.97, 1.41, 2.40, 3.65, 4.39, 5.00, 5.43, 5.84, 5.96};
    double len_ref[9] =  {5.84, 5.76, 5.54, 5.19, 4.80, 4.38, 3.87, 3.35, 3.01};

    int duty = 0;
    for(int i = 0; i < 9 - 1; i++) {
        if(wid_ref[i] < width && width <= wid_ref[i +1]) {
            servo_handler.cripper_len = len_ref[i] + (width - wid_ref[i])*( len_ref[i+1] - len_ref[i])
                                        /(wid_ref[i+1] - wid_ref[i]);
            duty = (int) (duty_ref[i] + (width - wid_ref[i])*( duty_ref[i+1] - duty_ref[i])/(wid_ref[i+1] - wid_ref[i]));
            break;
        }
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
    mutex_lock(servo_lock);
    servo_duty_set_lspb_calc(duty, SERVO_CHANNEL_5);
    ESP_LOGI(TAG, "width set: %.1lf", width);
    mutex_unlock(servo_lock);
    return ESP_OK;
}
/*
 *
 **************************************** NVS FLASH LOAD AND SAVE PARAMETER ***************************************
 *
 */
static const char *SERVO_NVS = "servo_nvs";
// pack and unpack funtion
static int _pack_func(void *context, char *buffer, int max_buffer_size)
{
    memcpy(buffer, &servo_handler, sizeof(servo_handle_t));
    return sizeof(servo_handle_t);
}

static esp_err_t _unpack_func(void *context, char *buffer, int loaded_len)
{
    memcpy(&servo_handler, buffer, loaded_len);
    return ESP_OK;
}

// init and load data default if can't
// find its in flash
esp_err_t servo_nvs_load(void)
{
    const char *TAG = "file: servo_control.c , function: servo_nvs_load";
    ESP_LOGI(TAG, "Flash init ...");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    esp_storage_config_t storage_cfg = {
        .namespace = "servo_param",
        .buffer_size = 1024,
    };
    storage_handle = esp_storage_init(&storage_cfg);
    esp_storage_add(storage_handle, SERVO_NVS, _unpack_func, _pack_func, NULL);

    if (esp_storage_load(storage_handle, SERVO_NVS) != ESP_OK) {
        ESP_LOGW(TAG, "load flash fail, set param default");
        _servo_param_set_default(&servo_handler);
        return esp_storage_save(storage_handle, SERVO_NVS);
    }

    if (servo_handler.nvs_magic != SERVO_NVS_MAGIC) {
        ESP_LOGW(TAG, "magic = %x", servo_handler.nvs_magic);
        _servo_param_set_default(&servo_handler);
        return esp_storage_save(storage_handle, SERVO_NVS);
    }
    for (int i = 0; i < SERVO_MAX_CHANNEL - 1; i++) {
        ESP_LOGI(TAG, "servo limit: channel[%d].upper_limit: %lf", i, servo_handler.duty_calib[i].upper_limit);
        ESP_LOGI(TAG, "servo limit: channel[%d].under_limit: %lf", i, servo_handler.duty_calib[i].under_limit);
    }
    return ESP_OK;
}

// save param after timeout second
esp_err_t _servo_nvs_save_all(void)
{
    const char *TAG = "file: servo_control.c , function: _servo_nvs_save_all";
    if (storage_handle) {
        esp_storage_save(storage_handle, SERVO_NVS);
        ESP_LOGI(TAG, "saved ok");
        return ESP_OK;
    }
    ESP_LOGE(TAG, "save error");
    return ESP_ERR_FLASH_NOT_INITIALISED;
}
//
esp_err_t servo_nvs_save(bool option, int channel)
{
    const char *TAG = "file: servo_control.c , function: servo_nvs_save";
    if (option == OPTION_UPPER_LIMIT) {
        servo_handler.duty_calib[channel].upper_limit = (double)servo_handler.channel[channel].duty_target;
        ESP_LOGI(TAG, "upper limit channel[%d] change: %d", channel, servo_handler.channel[channel].duty_target);
    } else if (option == OPTION_UNDER_LIMIT) {
        servo_handler.duty_calib[channel].under_limit = (double)servo_handler.channel[channel].duty_target;
        ESP_LOGI(TAG, "under limit channel[%d] change: %d", channel, servo_handler.channel[channel].duty_target);
    }
    if (storage_handle) {
        esp_storage_save(storage_handle, SERVO_NVS);
        ESP_LOGI(TAG, "saved ok");
    }
    return ESP_OK;
}

esp_err_t servo_nvs_default(void)
{
    _servo_param_set_default(&servo_handler);
    _servo_nvs_save_all();
    return ESP_OK;
}

esp_err_t servo_nvs_restore(bool option, int channel)
{
    const char *TAG = "file: servo_control.c , function: servo_nvs_save";
    if (option == OPTION_UPPER_LIMIT) {
        servo_handler.duty_calib[channel].upper_limit = DEFAULT_UPPER_LIMIT;
        ESP_LOGI(TAG, "upper limit channel[%d] change: %d", channel, DEFAULT_UPPER_LIMIT);
    } else if (option == OPTION_UNDER_LIMIT) {
        servo_handler.duty_calib[channel].under_limit = DEFAULT_UNDER_LIMIT;
        ESP_LOGI(TAG, "under limit channel[%d] change: %d", channel, DEFAULT_UNDER_LIMIT);
    }
    if (storage_handle) {
        esp_storage_save(storage_handle, SERVO_NVS);
        ESP_LOGI(TAG, "saved ok");
    }
    return ESP_OK;
}
/*
 *
 ***************************************** UART pack and unpack function*******************************************
 *
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

    // mutex_lock(servo_lock);
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

    memmove(package, pkg, pkg_len);
    free(pkg);
    // mutex_unlock(servo_lock);

    return pkg_len;
}

// this function unpack your package to buffer lead by buff pointer
// this function can't check pkg_len vs sizeof(pkg)
int msg_unpack(char *pkg, int pkg_len)
{
    const char *TAG = "file: servo_control.c , function: msg_unpack";
    // check lenght
    if (pkg_len < MSG_MIN_PKG_LEN) {
        ESP_LOGE(TAG, "package_len: %d < %d", pkg_len, MSG_MIN_PKG_LEN);
        return 0;
    }

    // check begin , end character
    if (pkg[0] != 0x7E || pkg[pkg_len - 1] != 0x7F) {
        ESP_LOGE(TAG, "begin char: %X != 0x7E or end char: %X != 0x7F", pkg[0], pkg[pkg_len - 1]);
        return 0;
    }

    // allocation buffer
    // mutex_lock(servo_lock);
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
    memset(pkg, 0, pkg_len);
    memmove(pkg, buff, buff_len);
    free(buff);
    // mutex_lock(servo_handler.unlock);

    return buff_len;
}
