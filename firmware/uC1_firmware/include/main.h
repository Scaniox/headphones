/*******************************************************************************
 *         File: headphones\firmware\uC1_firmware\include\main.h
 *       Author: Alexander Mills (Scaniox#7130)
 *      License: 
 *         Date: 16/06/23
 *  Description: uc1 firmware main header
 ******************************************************************************/

#include <Arduino.h>
#include <samd21\include\samd21g18a.h>
#include <AS3435.h>
#include <MAX17048.h>
#include <IS2020.h>
#include <Adafruit_NeoPixel.h>

// settings
#define VOL_MAX 100
#define POWER_ANIM_BLINK_TIME 300

// pin definitions
#define PIN_EAR_SENSE_L_EN      A3
#define PIN_EAR_SENSE_L_OUT     A4
#define PIN_BM83_PROG_EN        8u
#define PIN_BAT_V_SENSE         9u 

#define PIN_PWR_MFB             4u
#define PIN_RIGHT_SIDE_DATA     3u
#define PIN_BM83_UART_RX        1u
#define PIN_BM83_UART_TX        0u

#define PIN_LEDS_DATA_IN        2u
#define PIN_3V3_EN              5u
#define PIN_AUDIO_IN_DETECT     11u
#define PIN_RED_LED             PIN_LED
#define PIN_PWR_SW              10u
#define PIN_ANC_ON              12u
#define PIN_ANC_OFF             6u
#define PIN_ANC_PBO             7u

// operation modes
enum OPERATION_STATES {
    STATE_OFF,
    STATE_ON,
    STATE_PAIRING,
};

enum ANC_MODE {
    ANC,
    MONITOR,
    PBO,
};

// animations 
enum ANIMATIONS {
    ANIM_NONE,
    ANIM_POWER_ON,
    ANIM_POWER_OFF,
    ANIM_PAIRING,
};

// right side button mask
enum {
    VOL_UP,
    VOL_DOWN,
    PAUSE,
    BACK,
    FWRD,
};