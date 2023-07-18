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

Event_Timer battery_check_timer;
float batSOC = 0;

Event_Timer power_button_timer;

Event_Timer volume_button_timer;
volatile RIGHT_MASK volume_button_pressed = NONE;

volatile uint32_t animation_start_time = 0;
volatile uint8_t animation_step = 0;
volatile ANIMATIONS current_animation = ANIM_NONE;
bool animations = true;
Event_Timer animation_timer;

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
    animation_timer.start_countdown(0);
}

void update_animations() {

    switch (current_animation) {
    case ANIM_POWER_ON: {
        
        if (animation_timer.has_triggered()) {
            switch (animation_step % 2) {
            case 0:
                set_power_indicator(POWER_ON_BLUE);
                animation_step++;
                break;
            case 1:
                set_power_indicator(0);
                animation_step++;
                break;
            }

            // end of animation
            if (animation_step > 3) {
                start_animation(ANIM_NONE);
                set_power_indicator(DISCONECTED_ORANGE);
            }

            animation_timer.start_countdown(POWER_ANIM_BLINK_TIME);
        }
        break;
    }

    case ANIM_POWER_OFF: {

        if (animation_timer.has_triggered()) {

            if (animation_step == 0) {
                bm83.powerOff(current_device);
            }

            switch (animation_step % 2) {
            case 0:
                set_power_indicator(POWER_OFF_RED);
                animation_step++;
                break;
            case 1:
                set_power_indicator(0);
                animation_step++;
                break;
            }

            // end of animation
            if (animation_step > 5) {
                start_animation(ANIM_NONE);
                power_off();
            }

            animation_timer.start_countdown(POWER_ANIM_BLINK_TIME);
        }
        break;
    }

    case ANIM_PAIRING: {
        if (animation_timer.has_triggered()) {
            switch (animation_step % 2) {
            case 0:
                switch (current_device) {
                case 0: set_power_indicator(DEVICE_ONE_GREEN);  break;
                case 1: set_power_indicator(DEVICE_TWO_PURPLE); break;
                }
                animation_step++;
                break;
            case 1:
                set_power_indicator(DISCONECTED_ORANGE);
                animation_step++;
                break;
            }

            animation_timer.start_countdown(PAIRING_ANIM_BLINK_TIME);
        }
        break;
    }

    case ANIM_NONE: {
        // connected device - solid light of that colour
        // unconnected device - light of that colour blinks with yellow
        if (animation_timer.has_triggered()) {
            switch (animation_step % 2) {
            case 0:
                set_ANC_indicator(indicator_LEDs.ColorHSV(batSOC * 220));

                if (is_connected(current_device) || (animation_step % 4 == 0)) {
                    switch (current_device) {
                    case 0: set_power_indicator(DEVICE_ONE_GREEN);  break;
                    case 1: set_power_indicator(DEVICE_TWO_PURPLE); break;
                    }
                }
                else {
                    set_power_indicator(DISCONECTED_ORANGE);
                }

                animation_step++;
                animation_timer.start_countdown(IDLE_BLINK_ON_TIME);
                break;

            case 1:
                set_power_indicator(0);
                set_ANC_indicator(0);
                animation_step++;
                animation_timer.start_countdown(IDLE_BLINK_OFF_TIME);
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
    Serial.println("start_pairing");
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
    int x;
    while ((x = bm83.btSerial->available()) > 3) {
        bm83.getNextEventFromBt();
    }

    if (bm83.btmStatusChanged) {
        // bm83.readLinkStatus();
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

void check_battery() {
    if (battery_check_timer.has_triggered()) {
        battery_check_timer.start_countdown(BATTERY_CHECK_INTERVAL);

        batSOC = constrain(fuel_guage.getSoC(), 0, 100);
        bm83.report_Battery_Capacity(batSOC);

        if (batSOC < LOW_BAT_SHUTOFF_SOC) {
            start_animation(ANIM_POWER_OFF);
        }
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
    Serial.printf("bat soc: %f\n", batSOC);
    Serial.printf("bm83 bat_level: %i, %i\n", bm83.btmBatteryStatus[0], bm83.btmBatteryStatus[1]);
    Serial.printf("bat voltage: %f\n", fuel_guage.getVCell());

    Serial.printf("power_button_pressed_start_time: %i\n", power_button_timer.start_time);

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

    if (s == "timer stat") {
        Event_Timer::event_timers_stat();
    }

    if (s == "driver on") {
        as3435_L.set_output_driver_en(1);
        delay(10);
        as3435_R.set_output_driver_en(1);
    } 

    if (s == "driver off") {
        as3435_L.set_output_driver_en(0);
        delay(10);
        as3435_R.set_output_driver_en(0);
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
    if (power_button_timer.get_time_running() < 2) {
        //return;
    }

    power_button_timer.start_countdown(PAIRING_HOLD_TIME);
}

void power_button_up_isr() {
    uint32_t press_duration = power_button_timer.get_time_running();
    power_button_timer.stop();

    // debounce by discounting pulses that are too short
    if(press_duration < 2) {
        //return;
    }

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
    // need to ensure that the event timer sys time reading is corrected


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
    volume_button_timer.stop();

    if (is_connected(0) || is_connected(1)) {
        // button press
        volume_button_pressed = NONE;
        if (right_side_data & (1 << VOL_UP)) {
            // increase volume
            bm83.setOverallGain(current_device, OVERALL_GAIN_MASK_A2DP, OVERALL_GAIN_TYPE_VOL_UP);
            volume_button_pressed = VOL_UP;
            volume_button_timer.start_countdown(VOL_HOLD_START_TIME);
            Serial.println("VOL_UP");
        }

        if (right_side_data & (1 << VOL_DOWN)) {
            // decrease volume
            bm83.setOverallGain(current_device, OVERALL_GAIN_MASK_A2DP, OVERALL_GAIN_TYPE_VOL_DOWN);
            volume_button_pressed = VOL_DOWN;
            volume_button_timer.start_countdown(VOL_HOLD_START_TIME);
            Serial.println("VOL_DOWN");
        }

        if (right_side_data & (1 << PAUSE)) {
            bm83.togglePlayPause(current_device);
            Serial.println("pause");
        }

        if (right_side_data & (1 << BACK)) {
            bm83.previousSong(current_device);
            Serial.println("back");
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
    bm83.begin(PIN_BM83_RESET);
    bm83.enterPairingMode(0);
    bm83.resetLow();

    // set up right side serial
    right_serial.begin(9600);
    pinPeripheral(PIN_RIGHT_SIDE_DATA, PIO_SERCOM_ALT);

    // assign interrupts
    attachInterrupt(digitalPinToInterrupt(PIN_PWR_SW), power_button_isr, CHANGE);

    // Set the XOSC32K to run in standby
    SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;

    // Configure EIC to use GCLK1 which uses XOSC32K 
    // This has to be done after the first call to attachInterrupt()
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |
        GCLK_CLKCTRL_GEN_GCLK1 |
        GCLK_CLKCTRL_CLKEN;

    // configures other stuff
    power_on();

    // set up indicator LEDs
    indicator_LEDs.begin();

    indicator_LEDs.fill(0x00ffffff);
    indicator_LEDs.show();

    delay(5000);
    
    // start battery check timer
    battery_check_timer.start_countdown(BATTERY_CHECK_INTERVAL);
}

void loop() {
    switch (current_state) {
        case STATE_OFF:
            if (power_button_timer.has_triggered()) {
                start_pairing();
            }
            break;                               

        case STATE_ON:
            check_battery();
            read_bm83_events();

            //   Read on ear sensors :
            // • One ear : reduce volume
            //   • Both ears : pause

            //   Read power button press duration :
            //   • Enter pairing mode
            if (power_button_timer.has_triggered()) {
                start_pairing();
            }

            // volume button hold time
            if (volume_button_timer.has_triggered()) {
                if (volume_button_pressed == VOL_UP) {
                    bm83.setOverallGain(current_device, OVERALL_GAIN_MASK_A2DP, OVERALL_GAIN_TYPE_VOL_UP);
                }
                else if (volume_button_pressed == VOL_DOWN) {
                    bm83.setOverallGain(current_device, OVERALL_GAIN_MASK_A2DP, OVERALL_GAIN_TYPE_VOL_DOWN);
                }

                volume_button_timer.start_countdown(VOL_HOLD_REPEAT_TIME);
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
            check_battery();
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

    delay(10);

    // higher power usage in pairing mode, but it will be much easier to program
    if (current_state != STATE_PAIRING) {
        Event_Timer::sleep_until_next_trigger();
    }
}