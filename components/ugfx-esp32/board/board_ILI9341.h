/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.org/license.html
 */

/**
 * @file    boards/addons/gdisp/board_ILI9341_spi.h
 * @brief   GDISP Graphic Driver subsystem board interface for the ILI9341 display.
 *
 * @note    This file contains a mix of hardware specific and operating system specific
 *          code. You will need to change it for your CPU and/or operating system.
 */

#ifndef _GDISP_LLD_BOARD_H
#define _GDISP_LLD_BOARD_H

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "rom/ets_sys.h"
#include "rom/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/spi_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_vfs_fat_vmt.h"

#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_14
#define PIN_NUM_DC   GPIO_NUM_27
#define PIN_NUM_RST  GPIO_NUM_33
#define PIN_NUM_BCKL GPIO_NUM_32

#define SPI_NUM  0x3

#define LCD_TYPE_ILI 1
#define LCD_TYPE_ST 0

#define GDISP_SCREEN_WIDTH   320
#define GDISP_SCREEN_HEIGHT   240

//----------
#define LCD_SEL_CMD()   gpio_set_level(PIN_NUM_DC, 0) // Low to send command
#define LCD_SEL_DATA()  gpio_set_level(PIN_NUM_DC, 1) // High to send data
#define LCD_RST_SET()   gpio_set_level(GPIO_NUM_33, 1)
#define LCD_RST_CLR()   gpio_set_level(GPIO_NUM_33, 0)
#define LCD_BKG_ON()    gpio_set_level(GPIO_NUM_32, 1) // Backlight ON
#define LCD_BKG_OFF()   gpio_set_level(GPIO_NUM_32, 0) //Backlight OFF
//----------
/*
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

spi_device_handle_t gspi;
#define BUF_SIZE (4092)
uint8_t lcd_buf[BUF_SIZE];
int write_idx = 0;

DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[] = {
    {0xEF, {0x03, 0x80, 0x02}, 3},
    {0xCF, {0x00, 0XC1, 0X30}, 3},
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    {0xE8, {0x85, 0x00, 0x78}, 3},
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    {0xF7, {0x20}, 1},
    {0xEA, {0x00, 0x00}, 2},
    {0xC0, {0x23}, 1},
    {0xC1, {0x10}, 1},
    {0xC5, {0x3e, 0x28}, 2},
    {0xC7, {0x86}, 1},
    // {0x36, {0x48}, 1},
    {0x36, {0x08}, 1},
    {0x3A, {0x55}, 1},
    // {0xB1, {0x00, 0x18}, 2},
    {0xB1, {0x00, 0x1B}, 2},
    {0xB6, {0x08, 0x82, 0x27}, 3},
    {0xF2, {0x00}, 1},
    {0x26, {0x01}, 1},
    {0xE0, {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00}, 15},
    {0XE1, {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F}, 15},

    // {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    // {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    // {0x2C, {0}, 0},
    // {0xB7, {0x07}, 1},
    {0x11, {0}, 0x80},
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};

static void lcd_set_brightness(int duty) {


#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (PIN_NUM_BCKL)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TEST_DUTY         (10)

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT, // resolution of PWM duty
        .freq_hz = 5000,              // frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,   // timer mode
        .timer_num = LEDC_HS_TIMER    // timer index
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel =
    {
        .channel    = LEDC_HS_CH0_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_HS_CH0_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .timer_sel  = LEDC_HS_TIMER
    };
    ledc_channel_config(&ledc_channel);

    // Initialize fade service.
    // ledc_fade_func_install(0);

    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, duty);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}


static GFXINLINE void spi_write_byte(const uint8_t data)
{

    while (READ_PERI_REG(SPI_CMD_REG(SPI_NUM))&SPI_USR);
    WRITE_PERI_REG((SPI_W0_REG(SPI_NUM)), data);
    SET_PERI_REG_MASK(SPI_CMD_REG(SPI_NUM), SPI_USR);
}

void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

/**
 * @brief   Initialise the board for the display.
 *
 * @param[in] g         The GDisplay structure
 *
 * @note    Set the g->board member to whatever is appropriate. For multiple
 *          displays this might be a pointer to the appropriate register set.
 *
 * @notapi
 */
static GFXINLINE void init_board(GDisplay *g) {

    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_device_interface_config_t devcfg = {
        // .clock_speed_hz=10*1000*1000,               //Clock out at 10 MHz
        .clock_speed_hz = 60 * 1000 * 1000,         //Clock out at 10 MHz
        .mode = 0,                              //SPI mode 0
        .spics_io_num = PIN_NUM_CS,             //CS pin
        .queue_size = 7,                        //We want to be able to queue 7 transactions at a time
        .pre_cb = lcd_spi_pre_transfer_callback, //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    if (espIsMounted() == FALSE) {
        ret = spi_bus_initialize(VSPI_HOST, &buscfg, 2);
        assert(ret==ESP_OK);
    }


    //Attach the LCD to the SPI bus
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &gspi);
    // assert(ret==ESP_OK);
    LCD_SEL_DATA();
}

void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 8;                   //Command is 8 bits
    t.tx_buffer = &cmd;             //The data is the cmd itself
    t.user = (void*)0;              //D/C needs to be set to 0
    ret = spi_device_transmit(spi, &t); //Transmit!
    assert(ret == ESP_OK);          //Should have had no issues.
}

//Send data to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0) return;           //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = len * 8;             //Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;             //Data
    t.user = (void*)1;              //D/C needs to be set to 1
    ret = spi_device_transmit(spi, &t); //Transmit!
    assert(ret == ESP_OK);          //Should have had no issues.
}
/**
 * @brief   After the initialisation.
 *
 * @param[in] g         The GDisplay structure
 *
 * @notapi
 */
static GFXINLINE void post_init_board(GDisplay *g) {
    (void) g;
    int cmd = 0;
    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    // gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    // gpio_set_direction(GPIO_NUM_25, GPIO_MODE_INPUT);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);
    while (ili_init_cmds[cmd].databytes != 0xff) {
        lcd_cmd(gspi, ili_init_cmds[cmd].cmd);
        lcd_data(gspi, ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes & 0x1F);
        if (ili_init_cmds[cmd].databytes & 0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }
    // SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(SPI_NUM), SPI_USR_MOSI_DBITLEN, 7, SPI_USR_MOSI_DBITLEN_S);
}

/**
 * @brief   Set or clear the lcd reset pin.
 *
 * @param[in] g         The GDisplay structure
 * @param[in] state     TRUE = lcd in reset, FALSE = normal operation
 *
 * @notapi
 */
static GFXINLINE void setpin_reset(GDisplay *g, bool_t state) {
    (void) g;
    if (state) {
        LCD_RST_SET();
    } else {
        LCD_RST_CLR();
    }
}

/**
 * @brief   Set the lcd back-light level.
 *
 * @param[in] g             The GDisplay structure
 * @param[in] percent       0 to 100%
 *
 * @notapi
 */
static GFXINLINE void set_backlight(GDisplay *g, uint8_t percent) {
    (void) g;
    (void) percent;
    lcd_set_brightness(percent * 50);
}

/**
 * @brief   Take exclusive control of the bus
 *
 * @param[in] g             The GDisplay structure
 *
 * @notapi
 */
static GFXINLINE void acquire_bus(GDisplay *g) {
    (void) g;
    // printf("=>");
    write_idx = 0;
}

/**
 * @brief   Release exclusive control of the bus
 *
 * @param[in] g             The GDisplay structure
 *
 * @notapi
 */
static GFXINLINE void release_bus(GDisplay *g) {
    (void) g;
    // printf("\n");
    if (write_idx > 0) {
        lcd_data(gspi, lcd_buf, write_idx);
        write_idx = 0;
    }
}


/**
 * @brief   Send data to the index register.
 *
 * @param[in] g             The GDisplay structure
 * @param[in] index         The index register to set
 *
 * @notapi
 */
static GFXINLINE void write_index(GDisplay *g, uint16_t index) {
    // LCD_SEL_CMD();
    // // printf("\n===%x===\n", index);
    // spi_write_byte(index);
    // LCD_SEL_DATA();
    // printf(":");
    if (write_idx > 0) {
        lcd_data(gspi, lcd_buf, write_idx);
        write_idx = 0;
    }
    lcd_cmd(gspi, index);
}

/**
 * @brief   Send data to the lcd with DC control.
 *
 * @param[in] g             The GDisplay structure
 * @param[in] data          The data to send
 *
 * @notapi
 */
static GFXINLINE void write_data(GDisplay *g, uint16_t data) {
    (void) g;
    // printf(".");
    // spi_write_byte(data);
    // lcd_data(gspi, &d, 1);
    lcd_buf[write_idx++] = data;
    if (write_idx >= BUF_SIZE) {
        lcd_data(gspi, lcd_buf, BUF_SIZE);
        write_idx = 0;
    }
}

/**
 * @brief   Set the bus in read mode
 *
 * @param[in] g             The GDisplay structure
 *
 * @notapi
 */
static GFXINLINE void setreadmode(GDisplay *g) {
    (void) g;
}

/**
 * @brief   Set the bus back into write mode
 *
 * @param[in] g             The GDisplay structure
 *
 * @notapi
 */
static GFXINLINE void setwritemode(GDisplay *g) {
    (void) g;
}

/**
 * @brief   Read data from the lcd.
 * @return  The data from the lcd
 *
 * @param[in] g             The GDisplay structure
 *
 * @notapi
 */
static GFXINLINE uint16_t read_data(GDisplay *g) {
    (void) g;
    return 0;
}

#if GDISP_NEED_CONTROL && GDISP_HARDWARE_CONTROL

#endif
#endif /* _GDISP_LLD_BOARD_H */

