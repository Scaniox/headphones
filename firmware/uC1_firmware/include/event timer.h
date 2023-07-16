/*******************************************************************************
 *         File: headphones\firmware\uC1_firmware\include\event timer.h
 *       Author: Alexander Mills (Scaniox#7130)
 *      License: 
 *         Date: 
 *  Description: declares a event timer object that can be used to time the 
 *               events that occur, such as animations or button presses
 ******************************************************************************/
#ifndef EVENT_TIMER
#define EVENT_TIMER

#include <Arduino.h>
#include "RTCZero.h" // specific fork of rtczero (https://github.com/amcmahon01/RTCZero)
                     // with more RTC modes (ms counter modes, as well as usual date time mode)
#include "etl/vector.h"

class Event_Timer {
    public:
        volatile bool triggered = true;
        volatile uint32_t trigger_time = 0;
        volatile uint32_t start_time = 0;

        Event_Timer();
        void start_countdown(uint32_t time); 
        void stop();
        uint32_t get_time_left();
        uint32_t get_time_running();
        bool has_elapsed();
        bool has_triggered();
        

        static void event_timers_stat();
        static uint32_t global_time_to_next_trigger();
        static void sleep_until_next_trigger();

        static RTCZero rtc;
        static bool rtc_initialised;
        static uint32_t get_sys_time_ms();
};

#endif