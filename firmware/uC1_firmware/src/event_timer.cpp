/*******************************************************************************
 *         File: headphones\firmware\uC1_firmware\src\event_timer.cpp
 *       Author: Alexander Mills (Scaniox#7130)
 *      License: 
 *         Date: 
 *  Description: 
 ******************************************************************************/
#include "event timer.h" 

uint32_t Event_Timer::millis_offset = 0;

// sets the timer to trigger in [time] ms
void Event_Timer::start_countdown(uint32_t time) {
    trigger_time = Event_Timer::get_sys_time_ms() + time;

    if (trigger_times.available()) {
        trigger_times.push(trigger_time);
    }
    else {
        Serial.printf("trigger times queue is full, failed to add timer, stalling likely\n");
    }
}

// returns (ms) how long before the timer triggers
uint32_t Event_Timer::get_time_left() {
    int32_t time_left = trigger_time - Event_Timer::get_sys_time_ms();
    return max(time_left, 0);
}

// returns if the timer has triggered yet
bool Event_Timer::has_triggered() {
    return get_time_left() == 0;
}

// returns (ms) how long until the next timer will trigger
uint32_t Event_Timer::global_time_to_next_trigger() {
    // remove old ones
    while (!trigger_times.empty() && trigger_times.top() < get_sys_time_ms()) {
        trigger_times.pop();
    }

    if (trigger_times.empty()) {
        return 0;
    }
    
    return trigger_times.top();
}

// sleeps until the next timer triggers
void Event_Timer::sleep_until_next_trigger() {
    millis_offset += Watchdog.sleep(global_time_to_next_trigger());
    USBDevice.attach();
}

// get the current system time (ms)
uint32_t Event_Timer::get_sys_time_ms() {
    return millis() + millis_offset;
}
