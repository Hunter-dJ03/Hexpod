#include "rs_timed_loop.h"

// Constructor to set dealy till next time
RSTimedLoop::RSTimedLoop(int interval_ms) : interval(interval_ms) {
    next_time = chrono::high_resolution_clock::now() + interval;
}

// Delay thread until preset next time and set the next time for delay
void RSTimedLoop::realTimeDelay() {
    this_thread::sleep_until(next_time);
    next_time += interval;
}
