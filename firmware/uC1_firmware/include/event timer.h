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

class Event_Timer {
    public:
        uint32_t time = 0;
        bool triggered = false;

        void start_countdown(uint32_t time); 
        uint32_t get_time_left();
        bool has_triggered();
        
        static uint32_t global_time_to_next_trigger();
        static void sleep_until_next_trigger();

    private:
        uint32_t trigger_time = 0;

        static uint32_t millis_offset;
        static etl::priority_queue<uint32_t, 10, etl::vector<uint32_t, 10>, etl::greater<uint32_t>> trigger_times;
        static uint32_t get_sys_time_ms();
};