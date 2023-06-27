/*******************************************************************************
 *         File: headphones\firmware\uC1_firmware\src\main.cpp
 *       Author: Alexander Mills (Scaniox#7130)
 *      License: 
 *         Date: 
 *  Description: 
 ******************************************************************************/
#include "main.h"

// globals
Adafruit_NeoPixel indicator_LEDs(2, PIN_LEDS_DATA_IN);
AS3435 as3435_L;
AS3435 as3435_R;
IS2020 bm83(&Serial1);
MAX17048 fuel_guage;
Uart right_serial(&sercom2, PIN_RIGHT_SIDE_DATA, -1, SERCOM_RX_PAD_1, UART_TX_PAD_0);



OPERATION_STATES current_state = STATE_OFF;
ANC_MODE anc_mode = ANC;
bool charging = false;
bool devices_connected[] = {false, false};
int current_device = 0;
uint8_t volume = 100;

uint32_t power_button_pressed_start_time = 0;
uint32_t animation_start_time = 0;
ANIMATIONS current_animation = ANIM_NONE;
bool animations = true;

/******************************************************************************/
// helper functions
/******************************************************************************/
// set indicator LEDS

void set_ANC_indicator(uint32_t colour) {
    indicator_LEDs.setPixelColor(0, colour);
    delay(10);
    indicator_LEDs.show();
}

void set_power_indicator(uint32_t colour) {
    indicator_LEDs.setPixelColor(1, colour);
    delay(10);
    indicator_LEDs.show();
}

void power_off() {
    current_state = STATE_OFF;
    digitalWrite(PIN_3V3_EN, LOW);

    digitalWrite(PIN_LED, LOW);

    Serial.println("powering off");
}

void power_on() {
    current_state = STATE_ON;
    animation_start_time = millis();
    current_animation = ANIM_POWER_ON;
    digitalWrite(PIN_3V3_EN, HIGH);

    // set up AS3435
    as3435_L.begin(AS3435_I2CADDR_L);
    as3435_R.begin(AS3435_I2CADDR_R);

    // default leds
    set_ANC_indicator(0);
    set_power_indicator(indicator_LEDs.Color(0,255,255));

    digitalWrite(PIN_LED, HIGH);

    Serial.println("powering on");

}

/******************************************************************************/
// interrupts
/******************************************************************************/
// power button isrs
void power_button_down_isr() {
    // debounce by ignoring very recent presses
    if (millis() - power_button_pressed_start_time < 2) {
        //return;
    }

    power_button_pressed_start_time = millis();

    Serial.println("power pressed");
}

void power_button_up_isr() {
    uint32_t press_duration = millis() - power_button_pressed_start_time;

    // debounce by discounting pulses that are too short
    if(press_duration < 2) {
        //return;
    }

    Serial.println("power released");

    switch (current_state)
    {
    case STATE_ON:
    case STATE_PAIRING:
        if (500 < press_duration && press_duration < 2000) {
            current_animation = ANIM_POWER_OFF;
            animation_start_time = millis();
        }
        break;
    case STATE_OFF:
        if (press_duration < 2000) {
            power_on();
        }
        break;
    }

}

void power_button_isr() {
    delayMicroseconds(500);
    switch (digitalRead(PIN_PWR_SW)) {
    case LOW:
        power_button_down_isr();
        break;

    case HIGH:
        power_button_up_isr();
        break;
    }
}

// serial isrs

// right side 
// void SERCOM2_Handler() {
//     right_serial.IrqHandler();
// }



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

    pinMode(PIN_3V3_EN, OUTPUT);
    pinMode(PIN_AUDIO_IN_DETECT, INPUT);
    pinMode(PIN_RED_LED, OUTPUT);
    pinMode(PIN_PWR_SW, INPUT_PULLUP);
    pinMode(PIN_ANC_ON, INPUT_PULLUP);
    pinMode(PIN_ANC_OFF, INPUT_PULLUP);
    pinMode(PIN_ANC_PBO, INPUT_PULLUP);

    // set up i2c bus
    Wire.begin();
    Wire.setClock(400000);

    // set up fuel guage
    fuel_guage.reset();
    fuel_guage.quickStart();

    // usb serial
    Serial.begin(115200);

    // set up BM83
    Serial1.begin(115200);
    bm83.begin(-1);

    // set up right side serial
    right_serial.begin(9600);
    // pinPeripherial(PIN_RIGHT_SIDE_DATA, PIO_SERCOM);

    // assign interrupts
    attachInterrupt(digitalPinToInterrupt(PIN_PWR_SW), power_button_isr, CHANGE);

    // configures other stuff
    power_on();

    // set up indicator LEDs

    indicator_LEDs.begin();

    indicator_LEDs.fill(0x00ffffff);
    indicator_LEDs.show();

    delay(500);

}

void loop() {
    switch (current_state)
    {
    case STATE_OFF:
        
        break;                               

    case STATE_ON:
        // Receive from uC2 :
        // while (right_serial.available()) {
        //     char right_side_data = right_serial.read();

        //     if (right_side_data && 0x80) {
        //         if(devices_connected[0] || devices_connected[1]) {
        //             // button press
        //             if (right_side_data && (1 << VOL_UP)) {
        //                 // increase volume
        //                 volume = min((volume + 10), VOL_MAX);
        //                 bm83.avrcpSetAbsoluteVolume(current_device, volume);
        //             }

        //             if (right_side_data && (1 << VOL_DOWN)) {
        //                 // decrease volume
        //                 volume = max((volume - 10), 0);
        //                 bm83.avrcpSetAbsoluteVolume(current_device, volume);
        //             }

        //             if (right_side_data && (1 << PAUSE)) {
        //                 bm83.togglePlayPause(current_device);
        //             }

        //             if (right_side_data && (1 << BACK)) {
        //                 bm83.previousSong(current_device);
        //             }

        //             if (right_side_data && (1 << FWRD)) {
        //                 bm83.nextSong(current_device);
        //             }
        //         }
        //     }

        //     else {
        //         // ear sense data
        //         uint8_t ear_R_dark_reading = right_side_data;
        //         while(!right_serial.available());
        //         uint8_t ear_R_light_reading = right_serial.read();
        //     }

        // }

        //   Read battery charge :
        // float bat_SoC = fuel_guage.getSoC();
        // • Power off if too low
        //   • Low battery audible warning(every 15 mins)
        //   • BM83 updates connected device

        // power off it the battery is critically low

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

        // animate LEDs
        if (animations) {
            uint32_t animation_time = millis() - animation_start_time;
            switch (current_animation)
            {
            case ANIM_POWER_ON:
                if (animation_time < POWER_ANIM_BLINK_TIME * 2.5f){
                    if (animation_time % (POWER_ANIM_BLINK_TIME) < (POWER_ANIM_BLINK_TIME / 2)) {
                        set_power_indicator(0x0000ffff);
                    }
                    else {
                        set_power_indicator(0);
                    }  
                }

                break;
            case ANIM_POWER_OFF:
                if (animation_time < POWER_ANIM_BLINK_TIME * 2) {
                    if (animation_time % (POWER_ANIM_BLINK_TIME) < (POWER_ANIM_BLINK_TIME / 2)) {
                        set_power_indicator(0x00ff0000);
                    }
                    else {
                        set_power_indicator(0);
                    }
                }
                else {
                    power_off();
                }
                break;
            case ANIM_PAIRING:

                break;
            
            default:
                break;
            }
        }


        //   Reset wakeup timer
        //   sleep
        break;
    }

    // process debug serial
    String input = "";
    while (Serial.available())  
    {
        digitalWrite(PIN_RED_LED, HIGH);
        delay(100);
        digitalWrite(PIN_RED_LED, LOW);
        delay(100);
        input.concat((char)Serial.read());
        Serial.println(input);
    }

}