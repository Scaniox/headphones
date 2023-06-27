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
bool devices_connected[] = {true, false};
uint8_t current_device = 0;
volatile uint8_t volume = 100;

volatile uint32_t power_button_pressed_start_time = 0;  // used for button press timing
volatile bool power_button_pressed = false;          // used for hold and double presses

volatile uint32_t animation_start_time = 0;
volatile uint8_t animation_step = 0;
volatile ANIMATIONS current_animation = ANIM_NONE;
bool animations = true;


volatile uint8_t ear_R_dark_reading  = 0;
volatile uint8_t ear_R_light_reading = 0;


bool debug_serial = true;
String debug_input = "";

/******************************************************************************/
// action functions
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

// animations
void start_animation(ANIMATIONS anim) {
    current_animation = anim;
    animation_step = 0;
    animation_start_time = millis();
}

void update_animations() {
    uint32_t animation_time = millis() - animation_start_time;
    switch (current_animation) {
    case ANIM_POWER_ON: {
        uint32_t blink_time = animation_time % (POWER_ANIM_BLINK_TIME * 2);
        switch (animation_step % 2) {
        case 0:
            if (blink_time < POWER_ANIM_BLINK_TIME) {
                set_power_indicator(POWER_ON_BLUE);
                animation_step++;
            }
            break;
        case 1:
            if (blink_time > POWER_ANIM_BLINK_TIME) {
                set_power_indicator(0);
                animation_step++;
            }
            break;
        }
        // end of animation
        if (animation_step > 3) {
            start_animation(ANIM_NONE);
            set_power_indicator(DISCONECTED_ORANGE);
        }
        break;
    }

    case ANIM_POWER_OFF: {
        uint32_t blink_time = animation_time % (POWER_ANIM_BLINK_TIME * 2);
        switch (animation_step % 2) {
        case 0:
            if (blink_time < POWER_ANIM_BLINK_TIME) {
                set_power_indicator(POWER_OFF_RED);
                animation_step++;
            }
            break;
        case 1:
            if (blink_time > POWER_ANIM_BLINK_TIME) {
                set_power_indicator(0);
                animation_step++;
            }
            break;
        }
        // end of animation
        if (animation_step > 3) {
            start_animation(ANIM_NONE);
            power_off();
        }
        break;
    }

    case ANIM_PAIRING: {
        uint32_t blink_time = animation_time % (PAIRING_ANIM_BLINK_TIME * 2);
        switch (animation_step % 2) {
        case 0:
            if (blink_time < PAIRING_ANIM_BLINK_TIME) {
                set_power_indicator((current_device) ? DEVICE_ONE_GREEN : DEVICE_TWO_PURPLE);
                animation_step++;
            }
            break;
        case 1:
            if (blink_time > PAIRING_ANIM_BLINK_TIME) {
                set_power_indicator(DISCONECTED_ORANGE);
                animation_step++;
            }
            break;
        }
        break;
    }

    case ANIM_NONE: {
        switch (animation_step) {
        case 0:
            set_power_indicator((current_device) ? DEVICE_ONE_GREEN : DEVICE_TWO_PURPLE);
            animation_step++;
            break;
        
        default:
            break;
        }
        break;
    }


    default:
        break;
    }
}

// devices
void switch_device(uint8_t device) {
    current_device = device;
    start_animation(ANIM_NONE);
}


// power
void power_off() {
    current_state = STATE_OFF;
    digitalWrite(PIN_3V3_EN, LOW);

    digitalWrite(PIN_LED, LOW);
    Serial.println("powering off");
}

void power_on() {
    current_state = STATE_ON;
    start_animation(ANIM_POWER_ON);
    digitalWrite(PIN_3V3_EN, HIGH);

    // set up AS3435
    as3435_L.begin(AS3435_I2CADDR_L);
    as3435_R.begin(AS3435_I2CADDR_R);

    // default leds
    set_ANC_indicator(0);
    set_power_indicator(0);

    digitalWrite(PIN_LED, HIGH);
    Serial.println("powering on");

}

void start_pairing() {
    if (current_state == STATE_OFF) {
        power_on();
    }

    current_state = STATE_PAIRING;
    start_animation(ANIM_PAIRING);
}

// debug usb
void dissable_debug_USB() {
    USBDevice.detach();
}

void enable_debug_USB() {
    USBDevice.attach();
}

void send_full_status() {
    Serial.printf("=================CURRENT STATUS=================\n");
    Serial.printf("current_state: %i\n", current_state);
    Serial.printf("anc_mode: %i\n", anc_mode);
    Serial.printf("charging: %i\n", charging);
    Serial.printf("devices_connected: {%i, %i}\n", devices_connected[0], devices_connected[1]);
    Serial.printf("current_device: %i\n", current_device);
    Serial.printf("volume: %i\n", volume);

    Serial.printf("power_button_pressed_start_time: %i\n", power_button_pressed_start_time);
    Serial.printf("power_button_pressed: %i\n",power_button_pressed);

    Serial.printf("animation_start_time: %i\n", animation_start_time);
    Serial.printf("animation_step: %i\n", animation_step);
    Serial.printf("current_animation: %i\n", current_animation);
    Serial.printf("animations: %i\n", animations);
    Serial.printf("================================================\n\n");
}

void debug_parse(String s) {
    if (s == "stat") {
        send_full_status();
    }
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

    power_button_pressed = true;
    power_button_pressed_start_time = millis();
}

void power_button_up_isr() {
    uint32_t press_duration = millis() - power_button_pressed_start_time;

    // debounce by discounting pulses that are too short
    if(press_duration < 2) {
        //return;
    }

    power_button_pressed = false;

    switch (current_state) {
    case STATE_ON:
        if (press_duration <= POWER_OFF_HOLD_TIME) {
            switch_device((current_device + 1) % 2);
        }
        // deliberate fall through so power off works in both
    case STATE_PAIRING:
        if (POWER_OFF_HOLD_TIME < press_duration && press_duration <= PAIRING_HOLD_TIME) {
            start_animation(ANIM_POWER_OFF);
        }
        break;
    case STATE_OFF:
        if (press_duration < PAIRING_HOLD_TIME) {
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

void right_button_press_handler(char right_side_data) {
    if (devices_connected[0] || devices_connected[1]) {
        // button press
        if (right_side_data & (1 << VOL_UP)) {
            // increase volume
            volume = min((volume + 10), VOL_MAX);
            bm83.avrcpSetAbsoluteVolume(current_device, volume);
        }

        if (right_side_data & (1 << VOL_DOWN)) {
            // decrease volume
            volume = max((volume - 10), 0);
            bm83.avrcpSetAbsoluteVolume(current_device, volume);
        }

        if (right_side_data & (1 << PAUSE)) {
            bm83.togglePlayPause(current_device);
        }

        if (right_side_data & (1 << BACK)) {
            bm83.previousSong(current_device);
        }

        if (right_side_data & (1 << FWRD)) {
            bm83.nextSong(current_device);
        }
    }
}

void SERCOM2_Handler() {
    right_serial.IrqHandler();
    if (right_serial.available()) {
        // determine data type:
        // button presses
        if (right_serial.peek() && 0x80) {
            right_button_press_handler(right_serial.read());
        }

        // ear sense data ( must be 2 bytes)
        else if (right_serial.available() >= 2) {
            ear_R_dark_reading = right_serial.read();
            ear_R_light_reading = right_serial.read();
        }
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

    pinMode(PIN_3V3_EN, OUTPUT);
    pinMode(PIN_AUDIO_IN_DETECT, INPUT);
    pinMode(PIN_RED_LED, OUTPUT);
    pinMode(PIN_PWR_SW, INPUT_PULLUP);
    pinMode(PIN_ANC_ON, INPUT_PULLUP);
    pinMode(PIN_ANC_OFF, INPUT_PULLUP);
    pinMode(PIN_ANC_PBO, INPUT_PULLUP);

    // dissable debug usb if not needed
    if (!debug_serial) {
        dissable_debug_USB();
    }

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
        if (millis() - power_button_pressed_start_time > PAIRING_HOLD_TIME 
            && power_button_pressed) {
            start_pairing();
        }
        
        break;                               

    case STATE_ON:
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
        //   • Enter pairing mode
        if (millis() - power_button_pressed_start_time > PAIRING_HOLD_TIME 
            && power_button_pressed) {
            start_pairing();
        }

        //   Read ANC slider
        //   • Update both AS3435

        //   detect wired audio in and enable AS3435 bypass

        // animate LEDs
        if (animations) {
            update_animations();
        }

        //   Reset wakeup timer
        //   sleep
        break;

    case STATE_PAIRING:
        if (animations) {
            update_animations();
        }

        break;
    }

    // process debug serial
    if (debug_serial) {
        while (Serial.available()) {
            char c = Serial.read();
            if(c == '\n'){
                debug_parse(debug_input);
                debug_input = "";
            }
            else {
                debug_input.concat(c);
            }

        }
    }
}