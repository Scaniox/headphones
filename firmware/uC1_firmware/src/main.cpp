/*******************************************************************************
 *         File: headphones\firmware\uC1_firmware\src\main.cpp
 *       Author: Alexander Mills (Scaniox#7130)
 *      License: 
 *         Date: 
 *  Description: 
 ******************************************************************************/
#include "main.h"

// globals
Adafruit_NeoPixel indicator_LEDs;
AS3435 as3435_L;
AS3435 as3435_R;
IS2020 bm83;
MAX17048 fuel_guage;

OPERATION_STATES current_state = OFF;
OPERATION_STATES last_state = OFF;
bool just_switched_states = false;

ANC_MODE anc_mode = ANC;


uint32_t power_button_pressed_start_time = 0;

/******************************************************************************/
// helper functions
/******************************************************************************/
// set indicator LEDS
void set_power_indicator(uint32_t colour) {
    indicator_LEDs.setPixelColor(0, colour);
    indicator_LEDs.show();
}

void set_ANC_indicator(uint32_t colour) {
    indicator_LEDs.setPixelColor(1, colour);
    indicator_LEDs.show();
}

// switch operating state 
void switch_states(OPERATION_STATES new_state) {
    last_state = current_state;
    current_state = new_state;
    just_switched_states = true;
}

/******************************************************************************/
// interrupts
/******************************************************************************/
// power button isrs
void power_button_down_isr() {
    // debounce by ignoring very recent presses
    if (millis() - power_button_pressed_start_time < 5) {
        power_button_pressed_start_time = millis();
    }
}

void power_button_up_isr() {
    uint32_t press_duration = millis() - power_button_pressed_start_time;

    // debounce by discounting pulses that are too short
    if(press_duration < 5) {
        return;
    }

    

}



/******************************************************************************/
// main program
/******************************************************************************/
void setup() {
    // setup gpio
    pinMode(PIN_EAR_SENSE_L_EN, OUTPUT);
    pinMode(PIN_EAR_SENSE_L_OUT, INPUT);
    pinMode(PIN_BM83_PROG_EN, OUTPUT);
    pinMode(PIN_BAT_V_SENSE, INPUT);

    pinMode(PIN_PWR_MFB, OUTPUT);
    pinMode(PIN_RIGHT_SIDE_DATA, INPUT);
    pinMode(PIN_BM83_UART_RX, INPUT);
    pinMode(PIN_BM83_UART_TX, OUTPUT);

    pinMode(PIN_LEDS_DATA_IN, OUTPUT);
    pinMode(PIN_3V3_EN, OUTPUT);
    pinMode(PIN_AUDIO_IN_DETECT, INPUT);
    pinMode(PIN_RED_LED, OUTPUT);
    pinMode(PIN_PWR_SW, INPUT_PULLUP);
    pinMode(PIN_ANC_ON, INPUT_PULLUP);
    pinMode(PIN_ANC_OFF, INPUT_PULLUP);
    pinMode(PIN_ANC_PBO, INPUT_PULLUP);

    // set up indicator LEDs
    Adafruit_NeoPixel indicator_LEDs(2, PIN_LEDS_DATA_IN);
    indicator_LEDs.begin();
    indicator_LEDs.show();

    // set up i2c bus
    Wire.begin();
    Wire.setClock(400000);

    // set up fuel guage
    fuel_guage.reset();
    fuel_guage.quickStart();

    // set up AS3435
    as3435_L.begin(AS3435_I2CADDR_L);
    as3435_R.begin(AS3435_I2CADDR_R);

    // set up BM83
    Serial1.begin(115200);
    bm83.begin(Serial1);

}

void loop() {
    if (current_state == BATTERY_EMPTY) {
        if(/*charging detected*/) {
            switch_states(CHARGING);
        }
        return;
    }


    // Receive from uC2 :
    // • Send vol / media commands to BM83

    //   Read battery charge :
    // • Power off if too low
    //   • Low battery audible warning(every 15 mins)
    //   • BM83 updates connected device

    // power off it the battery is critically low
    float bat_votlage = analogRead(PIN_BAT_V_SENSE) * (2 / 1024.0) * 3.3;

    if (bat_votlage < BAT_UVLO) {
        // blink power indicator red thrice
        uint32_t red = indicator_LEDs.Color(255, 0, 0);
        for (int _ = 0; _ < 3; _++) {
            set_power_indicator(red);
            delay(200);
            set_power_indicator(0);
            delay(200);
        }

        // power off

    }

    //   Read on ear sensors :
    // • One ear : reduce volume
    //   • Both ears : pause

    //   Read power button press duration :
    // • Power on / off
    //   • Enter pairing mode
    //   • Report battery charge

    //   Read ANC slider
    //   • Update both AS3435

    //   detect wired audio in and enable AS3435 bypass

    //   Reset wakeup timer
    //   sleep
}