/*******************************************************************************
 *         File: headphones\firmware\uC2_firmware\src\main.cpp
 *       Author: Alexander Mills (Scaniox#7130)
 *      License: 
 *         Date: 
 *  Description: 
 ******************************************************************************/

/* chip datasheet:
https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-8495-8-bit-AVR-Microcontrollers-ATtiny441-ATtiny841_Datasheet.pdf
*/

/* pinout:
 * PA0 - opto led enable
 * PA1 - opto output
 * PA2 - vol_up
 * PA3 - vol_down
 * PA4 - pause
 * PA5 - back
 * PA6 - Forward
 * PA7 - right_side_data
 * 
 * PB2 - opto_LDR enable
 * PB3 - reset
 */

#define P_VOL_UP    PIN_PA2
#define P_VOL_DOWN  PIN_PA3
#define P_PAUSE     PIN_PA4
#define P_BACK      PIN_PA5
#define P_FWRD      PIN_PA6

#define P_OPTO_LED_EN   PIN_PA0
#define P_OPTO_LDR_EN   PIN_PB2
#define P_OPTO_OUT      PIN_PA1



#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

void setup() {
    // set up pins
    pinMode(P_VOL_UP,   INPUT_PULLUP);
    pinMode(P_VOL_DOWN, INPUT_PULLUP);
    pinMode(P_PAUSE,    INPUT_PULLUP);
    pinMode(P_BACK,     INPUT_PULLUP);
    pinMode(P_FWRD,     INPUT_PULLUP);

    pinMode(P_OPTO_LDR_EN,  OUTPUT);
    pinMode(P_OPTO_LED_EN,  OUTPUT);
    pinMode(P_OPTO_OUT,     INPUT);

    // set up uart
    Serial.begin(9600);

    // set up WDT
    WDTCSR |= 1 << WDIE; // WDT interrupt
    attachInterrupt(WDT_vect_num, func, CHANGE);

    // start WDT
    wdt_enable(WDTO_120MS);

}

void loop() {
    // pause WDT
    sleep_disable();
    wdt_disable();
    
    // button measurements
    uint8_t buttons = 0;
    buttons |= digitalRead(P_VOL_UP)    << 0;
    buttons |= digitalRead(P_VOL_DOWN)  << 1;
    buttons |= digitalRead(P_PAUSE)     << 2;
    buttons |= digitalRead(P_BACK)      << 3;
    buttons |= digitalRead(P_FWRD)      << 4;

    // ear sense measurement
    digitalWrite(P_OPTO_LDR_EN, HIGH);
    uint8_t dark_ear_sense = analogRead(P_OPTO_OUT);
    digitalWrite(P_OPTO_LED_EN, HIGH);
    delay(2);
    uint8_t bright_ear_sense = analogRead(P_OPTO_OUT);
    digitalWrite(P_OPTO_LED_EN, LOW);
    digitalWrite(P_OPTO_LDR_EN, LOW);

    // transmit over uart
    Serial.print(buttons);
    Serial.print(dark_ear_sense);
    Serial.print(bright_ear_sense);

    // start / reset WDT
    wdt_enable(WDTO_120MS);

    // go to Power down mode
    sleep_enable();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_cpu();

}

