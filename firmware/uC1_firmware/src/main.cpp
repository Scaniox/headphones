/*******************************************************************************
 *         File: headphones\firmware\uC1_firmware\src\main.cpp
 *       Author: Alexander Mills (Scaniox#7130)
 *      License: 
 *         Date: 
 *  Description: 
 ******************************************************************************/
#include "main.h"

const char bm83_prog_bytes[] = { 0x01, 0x20, 0xFC, 0x00 };
const String bm83_prog_str(bm83_prog_bytes);

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
int8_t current_device = 0;

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
    // don't disturb power of unless it is powering on
    if (current_animation == ANIM_POWER_OFF && anim != ANIM_POWER_ON){
        return;
    }

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
        if (animation_step == 0) {
            bm83.powerOff(current_device);
        }

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
                switch (current_device) {
                case 0: set_power_indicator(DEVICE_ONE_GREEN);  break;
                case 1: set_power_indicator(DEVICE_TWO_PURPLE); break;
                }
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
        // connected device - solid light of that colour
        if (is_connected(current_device))  {
            uint32_t blink_time = animation_time % (IDLE_BLINK_ON_TIME + IDLE_BLINK_OFF_TIME);
            switch (animation_step % 2) {
            case 0:
                if (blink_time < IDLE_BLINK_ON_TIME) {
                    switch (current_device) {
                    case 0: set_power_indicator(DEVICE_ONE_GREEN);  break;
                    case 1: set_power_indicator(DEVICE_TWO_PURPLE); break;
                    }
                    animation_step++;
                }
                break;
            case 1:
                if (blink_time > IDLE_BLINK_ON_TIME) {
                    set_power_indicator(0);
                    animation_step++;
                }
                break;
            }
        }

        // unconnected device - light of that colour blinks with yellow
        else {
            uint32_t blink_time = animation_time % (IDLE_BLINK_ON_TIME + IDLE_BLINK_OFF_TIME);
            switch (animation_step % 4) {
            case 0:
                if (blink_time < IDLE_BLINK_ON_TIME) {
                    switch (current_device) {
                    case 0: set_power_indicator(DEVICE_ONE_GREEN);  break;
                    case 1: set_power_indicator(DEVICE_TWO_PURPLE); break;
                    }
                    animation_step++;
                }
                break;
            case 2:
                if (blink_time < IDLE_BLINK_ON_TIME) {
                    set_power_indicator(DISCONECTED_ORANGE);
                    animation_step++;
                }
                break;

            case 1:
            case 3:
                if (blink_time > IDLE_BLINK_ON_TIME) {
                    set_power_indicator(0);
                    animation_step++;
                }
                break;
            }
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

bool is_connected(uint8_t device) {
    return (bm83.linkStatus[1 + device] & (1 << A2DP_profile_stream_channel_connected));
}

void start_pairing() {
    if (current_state == STATE_OFF) {
        power_on();
    }

    bm83.enterPairingMode(current_device);

    current_state = STATE_PAIRING;
    start_animation(ANIM_PAIRING);
}

void stop_pairing() {
    bm83.exitPairingMode(current_device);
    current_state = STATE_ON;
    start_animation(ANIM_NONE);   
}

void read_bm83_events() {
    // check for events from bm83
    while (bm83.btSerial->available() > 3) {
        bm83.getNextEventFromBt();
    }

    if (bm83.btmStatusChanged) {
        bm83.readLinkStatus();
        switch(bm83.btmState) {
        case BTM_STATE_A2DP_link_established:
        case BTM_STATE_A2DP_link_disconnected:
            // exit pairing
            if (current_state == STATE_PAIRING) {
                stop_pairing();
            }
            // deliberate fall through
        case BTM_STATE_pairing_fail:
        case BTM_STATE_pairing_successful:
            current_state = STATE_ON;

            // go to only device 0 connected
            if (is_connected(0) && !is_connected(1)) {
                switch_device(0);
                Serial.println("switch to device 0");
            }

            // go to only device 1 connected
            if (!is_connected(0) && is_connected(1)) {
                switch_device(1);
                Serial.println("switch to device 1");
            }
            break;
        default:
            break;
        }
        bm83.btmStatusChanged = false;
    }
}

// power
void power_off() {
    as3435_L.set_output_driver_en(0);
    delay(3);
    as3435_R.set_output_driver_en(0);

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
    as3435_L.pbo_mode();
    delay(3);
    as3435_R.begin(AS3435_I2CADDR_R);
    as3435_R.pbo_mode();


    bm83.resetModule();

    // bm83.btmUtilityFunction() isn't implemented
    // utility function 0x03, with data 0x00 dissables discoverable
    bm83.sendPacketArrayChar(3, CMD_BTM_Utility_Function, 0x03, 0x00);

    // bm83.enableAllSettingEvent();
    // delay(1000);
    // digitalWrite(PIN_BM83_MFB, HIGH);
    // delay(1000);
    delay(500);

    // default leds
    set_ANC_indicator(0);
    set_power_indicator(0);

    digitalWrite(PIN_LED, HIGH);
    Serial.println("powering on");
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

    Serial.printf("charging: %i\n", digitalRead(PIN_CHRG_STAT));
    bm83.report_Battery_Capacity(constrain( (int)fuel_guage.getSoC(), 0, 100));
    Serial.printf("bat soc: %f\n", fuel_guage.getSoC());
    Serial.printf("bm83 bat_level: %i, %i\n", bm83.btmBatteryStatus[0], bm83.btmBatteryStatus[1]);
    Serial.printf("bat voltage: %f\n", fuel_guage.getVCell());

    Serial.printf("power_button_pressed_start_time: %i\n", power_button_pressed_start_time);
    Serial.printf("power_button_pressed: %i\n",power_button_pressed);

    Serial.printf("animation_start_time: %i\n", animation_start_time);
    Serial.printf("animation_step: %i\n", animation_step);
    Serial.printf("current_animation: %i\n", current_animation);
    Serial.printf("animations: %i\n", animations);
    Serial.printf("================================================\n\n");
}

void send_bt_status() {
    Serial.printf("module state: %s\n", bm83.moduleState().c_str());
    Serial.printf("bt state: %s\n", bm83.btStatus().c_str());
    Serial.printf("current_device: %i\n", current_device);

    for (int dev_id = 0; dev_id < 2; dev_id++) {
        Serial.printf("\ndevice: %i\n", dev_id);
        Serial.printf("volume: %i/127\n", bm83.volume[dev_id]);
        Serial.printf("music status: %s\n", bm83.musicStatus(dev_id).c_str());
        Serial.printf("connection status: %s\n", bm83.connectionStatus(dev_id).c_str());
        Serial.printf("stream status: %s\n", bm83.streamStatus(dev_id).c_str());
        Serial.println();
    }
}

void debug_parse(String s) {
    if (s == "stat") {
        send_full_status();
    }

    if (s == "q") {
        bm83.readLocalDeviceName();
        Serial.println("NAME:");
        Serial.println(bm83.localDeviceName);
    }

    if (s == "bt stat") {
        send_bt_status();
    }
}

void bm83_serial_bridge() {
    current_state = STATE_BM83_PROG;
    indicator_LEDs.fill(0x00ff0000);
    indicator_LEDs.show();
    
    digitalWrite(PIN_BM83_RESET, LOW);
    digitalWrite(PIN_BM83_PROG_EN, LOW);
    delay(200);
    digitalWrite(PIN_BM83_RESET, HIGH);
    delay(200);


    // setup serial
    Serial1.begin(115200);
    Serial1.print(bm83_prog_str);
    // rolling 24 (incl null) char buffer
    char in_buff[24] = "test  test  test  test ";

    while (true) {
        // PC to BM83
        if (Serial.available()) {
            char c = Serial.read();
            // process input for "mcu_serial_bridge_exit"
            strcpy(in_buff, in_buff + 1);
            strncat(in_buff, &c, 1);

            if (strcmp(in_buff, "mcu_serial_bridge_exit\n") == 0) {
                Serial.printf("exiting serial bridge mode\n");
                break;
            }

            // copy from PC to BM83
            Serial1.write(c);
        }

        // bm83 to PC
        if (Serial1.available()) {
            Serial.write(Serial1.read());
        }
    }

    digitalWrite(PIN_BM83_PROG_EN, HIGH);
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
                // switch device
                switch_device((current_device + 1) % 2);
            }
            // deliberate fall through so power off works in both
        case STATE_PAIRING:
            if (press_duration < POWER_OFF_HOLD_TIME) {
                stop_pairing();
            }
            if (POWER_OFF_HOLD_TIME < press_duration && press_duration <= PAIRING_HOLD_TIME) {
                start_animation(ANIM_POWER_OFF);
            }
            break;
        case STATE_OFF:
            if (press_duration < PAIRING_HOLD_TIME) {
                power_on();
            }
            break;
        case STATE_BM83_PROG:
            // reset system to exit programming mode
            NVIC_SystemReset();
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
    if (is_connected(0) || is_connected(1)) {
        // button press
        if (right_side_data & (1 << VOL_UP)) {
            // increase volume
            uint8_t volume = min((bm83.volume[current_device] + 10), VOL_MAX);
            bm83.avrcpSetAbsoluteVolume(current_device, volume);
            Serial.println("VOL_UP");
        }

        if (right_side_data & (1 << VOL_DOWN)) {
            // decrease volume
            uint8_t volume = max((bm83.volume[current_device] - 10), 0);
            bm83.avrcpSetAbsoluteVolume(current_device, volume);
            Serial.println("VOL_DOWN");
        }

        if (right_side_data & (1 << PAUSE)) {
            bm83.togglePlayPause(current_device);
            Serial.println("pause");
        }

        if (right_side_data & (1 << BACK)) {
            bm83.previousSong(current_device);
            Serial.println("baack");
        }

        if (right_side_data & (1 << FWRD)) {
            bm83.nextSong(current_device);
            Serial.println("fwrd");
        }
    }
}

void SERCOM2_Handler() {
    right_serial.IrqHandler();
    if (right_serial.available()) {
        // Serial.printf("right side data: 0x%x\n", right_serial.read());
        // determine data type:
        // button presses
        if (right_serial.peek() & 0x80) {
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

    pinMode(PIN_BM83_MFB, OUTPUT);
    pinMode(PIN_RIGHT_SIDE_DATA, INPUT);
    pinMode(PIN_BM83_UART_RX, INPUT);
    pinMode(PIN_BM83_UART_TX, OUTPUT);
    pinMode(PIN_BM83_RESET, OUTPUT);

    pinMode(PIN_3V3_EN, OUTPUT);
    pinMode(PIN_AUDIO_IN_DETECT, INPUT);
    pinMode(PIN_RED_LED, OUTPUT);
    pinMode(PIN_PWR_SW, INPUT_PULLUP);
    pinMode(PIN_ANC_ON, INPUT_PULLUP);
    pinMode(PIN_ANC_OFF, INPUT_PULLUP);
    pinMode(PIN_ANC_PBO, INPUT_PULLUP);

    pinMode(PIN_CHRG_STAT, INPUT_PULLUP);
    pinMode(PIN_BAT_ALRT, INPUT_PULLUP);

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
    digitalWrite(PIN_BM83_PROG_EN, HIGH); // normal boot, not prog boot
    Serial1.begin(115200);
    bm83.begin(PIN_BM83_RESET);

    // set up right side serial
    right_serial.begin(9600);
    pinPeripheral(PIN_RIGHT_SIDE_DATA, PIO_SERCOM_ALT);

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
    switch (current_state) {
        case STATE_OFF:
            if (millis() - power_button_pressed_start_time > PAIRING_HOLD_TIME 
                && power_button_pressed) {
                start_pairing();
            }
            
            break;                               

        case STATE_ON:
            //   Read battery charge :
            if (fuel_guage.getSoC() < 5) {
                start_animation(ANIM_POWER_OFF);
            }
            // • Power off if too low
            //   • Low battery audible warning(every 15 mins)
            //   • BM83 updates connected device

            // check events from bm83
            read_bm83_events();

            if (bm83.btmStatusChanged) {

            }

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
            // exit pairing by the same way it was entered
            // if (millis() - power_button_pressed_start_time > PAIRING_HOLD_TIME
            //     && power_button_pressed) {
            //     stop_pairing();
            // }

            read_bm83_events();

            if (animations) {
                update_animations();
            }
            

            break;

        default:
            break;
    }

    // process debug serial
    if (debug_serial) {
        while (Serial.available()) {
            // digitalWrite(PIN_RED_LED, HIGH);
            // delay(100);
            // digitalWrite(PIN_RED_LED, LOW);
            // delay(100);

            char c = Serial.read();
            if(c == '\n'){
                debug_parse(debug_input);
                debug_input = "";
            }
            else {
                debug_input.concat(c);

                if (debug_input == bm83_prog_str){
                    debug_input = "";
                    bm83_serial_bridge();
                }
            }

        }
    }
}