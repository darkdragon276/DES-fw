#ifndef GINPUT_LLD_TOGGLE_CONFIG
#define GINPUT_LLD_TOGGLE_CONFIG

#include "driver/gpio.h"
// re-export BADGE_BUTTON_* constants
#define GINPUT_TOGGLE_NUM_PORTS 3
#define GINPUT_TOGGLE_CONFIG_ENTRIES   3

#define M5BTN_LEFT 37
#define M5BTN_MID 38
#define M5BTN_RIGHT 39

// We are (well, will be) interrupt driven
// #define GINPUT_TOGGLE_POLL_PERIOD       GTIMER_FLG_PERIODIC

#define GINPUT_TOGGLE_DECLARE_STRUCTURE() \
    const GToggleConfig GInputToggleConfigTable[GINPUT_TOGGLE_CONFIG_ENTRIES] = { \
        { 37, 1, 1, GPIO_MODE_INPUT }, \
        { 38, 1, 1, GPIO_MODE_INPUT }, \
        { 39, 1, 1, GPIO_MODE_INPUT }, \
    }

#endif // GINPUT_LLD_TOGGLE_CONFIG
