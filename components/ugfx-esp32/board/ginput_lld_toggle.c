/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.org/license.html
 */

#include <sdkconfig.h>
#include "driver/gpio.h"
#include "gfx.h"

#if (GFX_USE_GINPUT && GINPUT_NEED_TOGGLE)
#include "../../../src/ginput/ginput_driver_toggle.h"

#include "ginput_lld_toggle_config.h"

GINPUT_TOGGLE_DECLARE_STRUCTURE();


void gpiobutton_handler(void *arg)
{
    // uint32_t gpio_num = (uint32_t) arg;

    // int new_state = gpio_get_level(gpio_num);
    // if (new_state != badge_gpiobutton_old_state[gpio_num])
    // {
    //     uint32_t button_id = badge_gpiobutton_conv[gpio_num];
    //     badge_input_add_event(button_id, new_state == 0 ? EVENT_BUTTON_PRESSED : EVENT_BUTTON_RELEASED, IN_ISR);
    // }
    // badge_gpiobutton_old_state[gpio_num] = new_state;
    ginputToggleWakeupI();
}
void ginput_lld_toggle_init(const GToggleConfig *ptc)
{
    printf("ginput_lld_toggle: init() %d\n", (int)ptc->id);
    // gpio_install_isr_service(0);
    // gpio_config_t cfg = {
    //     .pin_bit_mask  = (1ULL << (int)ptc->id),
    //     .mode = GPIO_MODE_INPUT,
    //     .pull_up_en = GPIO_PULLUP_ENABLE
    // };
    // gpio_config(&cfg);
    gpio_set_direction((gpio_num_t) ptc->id, GPIO_MODE_INPUT);
    gpio_pullup_en((gpio_num_t) ptc->id);
    // gpio_isr_handler_add((gpio_num_t) ptc->id, gpiobutton_handler, (void*) ptc->id);
    // gpio_intr_enable((gpio_num_t) ptc->id);

}

unsigned ginput_lld_toggle_getbits(const GToggleConfig *ptc)
{
    // printf("ginput_lld_toggle: getbits(%d)\n", (gpio_num_t) ptc->id);

    // drain input queue
    // while (badge_input_get_event(0) != 0);

    // pass on button state
    return gpio_get_level((gpio_num_t) ptc->id);
}

#endif /*  GFX_USE_GINPUT && GINPUT_NEED_TOGGLE */
