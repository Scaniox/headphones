/*******************************************************************************
 *         File: headphones\firmware\uC1_firmware\include\event timer.h
 *       Author: Alexander Mills (Scaniox#7130)
 *      License: 
 *         Date: 
 *  Description: declares a event timer object that can be used to time the 
 *               events that occur, such as animations or button presses
 ******************************************************************************/
#include <Arduino.h>
#include <Adafruit_SleepyDog.h> 
#include <etl/priority_queue.h> // https://www.etlcpp.com/priority_queue.html

#define TRIGGERS_PRIORITY_QUEUE_t etl::priority_queue<uint32_t, 10, etl::vector<uint32_t, 10>, etl::greater<uint32_t>>

class Event_Timer {
    public:
        uint32_t time = 0;
        bool running = false;
        bool triggered = false;

        Event_Timer() {}
        void start_countdown(uint32_t time); 
        void stop();
        uint32_t get_time_left();
        uint32_t get_time_running();
        bool has_elapsed();
        bool has_triggered();
        
        static uint32_t global_time_to_next_trigger();
        static void sleep_until_next_trigger();

        uint32_t trigger_time = 0;
        uint32_t start_time = 0;

        static uint32_t millis_offset;
        static TRIGGERS_PRIORITY_QUEUE_t trigger_times;
        static uint32_t get_sys_time_ms();
};