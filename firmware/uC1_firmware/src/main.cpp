/*******************************************************************************
 *         File: headphones\firmware\uC1_firmware\src\main.cpp
 *       Author: Alexander Mills (Scaniox#7130)
 *      License: 
 *         Date: 
 *  Description: 
 ******************************************************************************/

#include <Arduino.h>
#include <AS3435.h>
#include <IS2020.h>
#include <Adafruit_NeoPixel.h>

void setup() {
  // setup 
}

void loop() {
  // Receive from uC2 :
  // • Send vol / media commands to BM83

  //   Read battery charge :
  // • Power off if too low
  //   • Low battery audible warning(every 15 mins)
  //   • BM83 updates connected device

  //   Read on ear sensors :
  // • One ear : reduce volume
  //   • Both ears : pause

  //   Read power button press duration :
  // • Power on / off
  //   • Enter pairing mode
  //   • Report battery charge

  //   Read ANC slider
  //   • Update both AS3435

  //   Reset wakeup timer
  //   sleep
}