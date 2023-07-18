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

etl::vector<Event_Timer*, MAX_TIMERS> all_timers = etl::vector<Event_Timer*, MAX_TIMERS>();

void count_done() {
    // Serial.println("rtc finished");
}


Event_Timer::Event_Timer() {
    // only initialise the RTC if it isn't already intialised
    if (!rtc_initialised) {
        rtc.begin(true, 0, false, RTCZero::Prescaler::MODE0_DIV1);
        rtc.attachInterrupt(count_done);

        rtc_initialised = true;
    }

    // add this timer to the vector of all timers
    all_timers.push_back(this);
}


// sets the timer to trigger in [time] ms
void Event_Timer::start_countdown(uint32_t time) {
    start_time = Event_Timer::get_sys_time_ms();
    trigger_time = start_time + time;
    triggered = false;
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
    // dissabling isrs here prevents the triggered changing mid-evaluation
    // actually only pcint ISRS need to be paused, but all is easier
    noInterrupts();
    bool triggering = has_elapsed() && !triggered;
    interrupts();

    if (triggering) {
        triggered = true;
        return true;
    }

    return false;
}

// print out the state of all timers:
void Event_Timer::event_timers_stat() {
    Serial.printf("==================TIMER STATUS==================\n");

    Serial.printf("there are: %i timers\n", all_timers.size());
    Serial.printf("index || start time ||  end time  || triggered ||\n");
    for (uint8_t timer_index = 0; timer_index < all_timers.size(); timer_index++) {
        Event_Timer* this_timer = all_timers[timer_index];
        Serial.printf("%5i || %10i || %10i || %9i ||\n", timer_index, this_timer->start_time, this_timer->trigger_time, this_timer->triggered);
    }

    Serial.printf("================================================\n\n");
}


// returns (ms [rtc ticks]) when the next trigger of any timer will happen
uint32_t Event_Timer::global_next_trigger_time() {
    uint32_t current_min = UINT32_MAX;
    for(uint8_t timer_index = 0; timer_index < all_timers.size(); timer_index++) {
        if(!(all_timers[timer_index]->triggered)) {
            current_min = min(current_min, all_timers[timer_index]->trigger_time);
        }
    }
    return current_min;
}

// sleeps until the next timer triggers
void Event_Timer::sleep_until_next_trigger() {
    // go to sleep
    rtc.enableCounter(global_next_trigger_time());
    // rtc.standbyMode();

    // restart usb
    // USBDevice.attach();
}

// get the current system time (ms)
uint32_t Event_Timer::get_sys_time_ms() {
    return rtc.getCount();
}
