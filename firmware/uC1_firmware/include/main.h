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
#define BAT_UVLO 3.6

// pin definitions
#define PIN_EAR_SENSE_L_EN      PIN_PA04 `
#define PIN_EAR_SENSE_L_OUT     PIN_PA05
#define PIN_BM83_PROG_EN        PIN_PA06
#define PIN_BAT_V_SENSE         PIN_PA07

#define PIN_PWR_MFB             PIN_PA08
#define PIN_RIGHT_SIDE_DATA     PIN_PA09
#define PIN_BM83_UART_RX        PIN_PA10
#define PIN_BM83_UART_TX        PIN_PA11

#define PIN_LEDS_DATA_IN        PIN_PA14
#define PIN_3V3_EN              PIN_PA15
#define PIN_AUDIO_IN_DETECT     PIN_PA16
#define PIN_RED_LED             PIN_PA17
#define PIN_PWR_SW              PIN_PA18
#define PIN_ANC_ON              PIN_PA19
#define PIN_ANC_OFF             PIN_PA20
#define PIN_ANC_PBO             PIN_PA21

// operation modes
enum OPERATION_STATES {
    OFF,
    BATTERY_EMPTY,
    ON,
    ONE_EAR,
    OFF_EAR,
    CHARGING,
};


enum ANC_MODE {
    ANC,
    MONITOR,
    PBO,
};