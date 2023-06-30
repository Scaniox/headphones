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

/* fuse setup
 * keep defaults
 *
 */

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

enum {
    VOL_UP,
    VOL_DOWN,
    PAUSE,
    BACK,
    FWRD,
};

#define DEBOUNCE_WAIT_TIME 50
#define DEBOUNCE_HOLDOFF_TIME 100

#define P_VOL_UP    PIN_PA2
#define P_VOL_DOWN  PIN_PA3
#define P_PAUSE     PIN_PA4
#define P_BACK      PIN_PA5
#define P_FWRD      PIN_PA6

#define P_OPTO_LED_EN   PIN_PA0
#define P_OPTO_LDR_EN   PIN_PB2
#define P_OPTO_OUT      PIN_PA1
#define P_DATA_OUT      PIN_PA7 // USART0 TX alternate pin

int last_press_time = 0;

// pin change interrupt
ISR(PCINT0_vect) {
    for (int _=0; _< DEBOUNCE_WAIT_TIME; _++) {
        delayMicroseconds(1000u);
    }

    uint8_t buttons = 0x80;
    buttons |= (!digitalRead(P_VOL_UP) & 0x01)   << VOL_UP;
    buttons |= (!digitalRead(P_VOL_DOWN) & 0x01) << VOL_DOWN;
    buttons |= (!digitalRead(P_PAUSE) & 0x01)    << PAUSE;
    buttons |= (!digitalRead(P_BACK) & 0x01)     << BACK;
    buttons |= (!digitalRead(P_FWRD) & 0x01)     << FWRD;
    Serial.write(buttons);

    // clear flag for this interrupt so it doesn't re-trigger immediately due to 
    // switch bounce
    GIFR |= _BV(PCIF0);
}

ISR(WDT_vect) { }

void setup() {
    cli();

    // set up pins
    pinMode(P_VOL_UP,   INPUT_PULLUP);
    pinMode(P_VOL_DOWN, INPUT_PULLUP);
    pinMode(P_PAUSE,    INPUT_PULLUP);
    pinMode(P_BACK,     INPUT_PULLUP);
    pinMode(P_FWRD,     INPUT_PULLUP);

    pinMode(P_OPTO_LDR_EN,  OUTPUT);
    pinMode(P_OPTO_LED_EN,  OUTPUT);
    pinMode(P_OPTO_OUT,     INPUT);

    // // setup button interrupts
    GIMSK |= _BV(PCIE0);
    PCMSK0 |= _BV(PCINT2) | _BV(PCINT3) |_BV(PCINT4) | _BV(PCINT5) |
              _BV(PCINT6); 

    // set up uart
    Serial.begin(9600);
    // dissable RX
    UCSR0B &= ~_BV(RXEN0); // disable RX
    // set to alternate pin (PA7)
    REMAP |= _BV(U0MAP);

    // // set up WDT
    // WDTCSR |= 1 << WDIE; // WDT interrupt

    // // start WDT
    // wdt_enable(WDTO_120MS);

    sei();
}

void loop() {
    cli();
    // pause WDT
    // sleep_disable();
    // wdt_disable();
    
    // ear sense measurement
    digitalWrite(P_OPTO_LDR_EN, HIGH);
    uint8_t dark_ear_sense = analogRead(P_OPTO_OUT) >> 1;
    digitalWrite(P_OPTO_LED_EN, HIGH);
    delay(100);
    uint8_t bright_ear_sense = analogRead(P_OPTO_OUT) >> 1;
    digitalWrite(P_OPTO_LED_EN, LOW);
    digitalWrite(P_OPTO_LDR_EN, LOW);

    // transmit over uart
    Serial.write(dark_ear_sense);
    Serial.write(bright_ear_sense);

    // start / reset WDT
    // wdt_enable(WDTO_120MS);

    sei();

    // // go to Power down mode
    // sleep_enable();
    // set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    // sleep_cpu();

    delay(500);
}

