/*******************************************************************************
 *         File: headphones\firmware\uC1_firmware\include\main.h
 *       Author: Alexander Mills (Scaniox#7130)
 *      License: 
 *         Date: 16/06/23
 *  Description: uc1 firmware main header
 ******************************************************************************/

#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function
#include <samd21\include\samd21g18a.h>
#include <AS3435.h>
#include <MAX17048.h>
#include <IS2020.h>
#include <Adafruit_NeoPixel.h>

// settings
#define VOL_MAX 127
#define POWER_ANIM_BLINK_TIME 300
#define PAIRING_ANIM_BLINK_TIME 300

#define IDLE_BLINK_ON_TIME 100
#define IDLE_BLINK_OFF_TIME 900

// button timings
#define PAIRING_HOLD_TIME 2000
#define POWER_OFF_HOLD_TIME 500

// pin definitions
#define PIN_EAR_SENSE_L_EN      A3
#define PIN_EAR_SENSE_L_OUT     A4
#define PIN_BM83_PROG_EN        8u
#define PIN_BAT_V_SENSE         9u 

#define PIN_RIGHT_SIDE_DATA     3u
#define PIN_BM83_MFB            4u
#define PIN_BM83_UART_RX        1u
#define PIN_BM83_UART_TX        0u
#define PIN_BM83_RESET          A5

#define PIN_LEDS_DATA_IN        2u
#define PIN_3V3_EN              5u
#define PIN_AUDIO_IN_DETECT     11u
#define PIN_RED_LED             PIN_LED
#define PIN_PWR_SW              10u
#define PIN_ANC_ON              12u
#define PIN_ANC_OFF             6u
#define PIN_ANC_PBO             7u

#define PIN_CHRG_STAT           MOSI
#define PIN_BAT_ALRT            MISO

// operation modes
enum OPERATION_STATES {
    STATE_OFF,
    STATE_ON,
    STATE_PAIRING,
    STATE_BM83_PROG,
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


// colours
#define POWER_ON_BLUE           0x0000ffff
#define POWER_OFF_RED           0x00ff0000
#define DISCONECTED_ORANGE      0x00ff7300
#define DEVICE_ONE_GREEN        0x0000ff00
#define DEVICE_TWO_PURPLE       0x008000ff


// function declarations
/******************************************************************************/
// action functions
/******************************************************************************/
// set indicator LEDS
void set_ANC_indicator(uint32_t colour);
void set_power_indicator(uint32_t colour);

// animations
void start_animation(ANIMATIONS anim);
void update_animations();

// devices
void switch_device(uint8_t device);
bool is_connected(uint8_t device);
void start_pairing();
void stop_pairing();
void read_bm83_events();


// power
void power_off();
void power_on();

// debug usb
void dissable_debug_USB();
void enable_debug_USB();
void send_full_status();
void send_bt_status();
void debug_parse(String s);
void bm83_serial_bridge();

/******************************************************************************/
// interrupts
/******************************************************************************/
// power button isrs
void power_button_down_isr();
void power_button_up_isr();
void power_button_isr();

// serial isrs

// right side 
void right_button_press_handler(char right_side_data);
