/*******************************************************************************
 *         File: headphones\firmware\uC1_firmware\src\event_timer.cpp
 *       Author: Alexander Mills (Scaniox#7130)
 *      License: 
 *         Date: 
 *  Description: 
 ******************************************************************************/
#include "event timer.h" 

// initialise statics
bool Event_Timer::rtc_initialised = false;
RTCZero Event_Timer::rtc = RTCZero();
TRIGGERS_PRIORITY_QUEUE_t Event_Timer::trigger_times = TRIGGERS_PRIORITY_QUEUE_t();


Event_Timer::Event_Timer() {
    // only initialise the RTC if it isn't already intialised
    if (!rtc_initialised) {
        rtc.begin(true, 0, false, RTCZero::Prescaler::MODE0_DIV1);

        rtc_initialised = true;
    }
}


// sets the timer to trigger in [time] ms
void Event_Timer::start_countdown(uint32_t time) {
    start_time = Event_Timer::get_sys_time_ms();
    trigger_time = start_time + time;
    triggered = false;

    // if (trigger_times.available()) {
    //     trigger_times.push(trigger_time);
    // }
    // else {
    //     Serial.printf("trigger times queue is full, failed to add timer, stalling likely\n");
    // }
}

// stops the timer, so it won't trigger
void Event_Timer::stop() {
    triggered = true;
}

// returns (ms) how long before the timer triggers
uint32_t Event_Timer::get_time_left() {
    int32_t time_left = trigger_time - Event_Timer::get_sys_time_ms();
    return max(time_left, 0);
}

uint32_t Event_Timer::get_time_running() {
    return Event_Timer::get_sys_time_ms() - start_time;
}

// returns true if it is after it's trigger time
bool Event_Timer::has_elapsed() {
    return get_time_left() == 0;
}

// returns true the first time it is called after the timer elapses 
// used to trigger the timer finishing actions
bool Event_Timer::has_triggered() {
    if (has_elapsed() && !triggered){
        triggered = true;
        return true;
    }
    else {
        return false;
    }
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
    // millis_offset += Watchdog.sleep(global_time_to_next_trigger());
    USBDevice.attach();
}

// get the current system time (ms)
uint32_t Event_Timer::get_sys_time_ms() {
    return rtc.getCount();
}
