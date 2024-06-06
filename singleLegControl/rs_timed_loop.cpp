#include "rs_timed_loop.h"
#include "iostream"

using namespace std;

// Constructor to set dealy till next time
RSTimedLoop::RSTimedLoop(int interval_ms) : interval(interval_ms) {
    start_time = chrono::high_resolution_clock::now();
    next_time = chrono::high_resolution_clock::now() + interval;
}

// Delay thread until preset next time and set the next time for delay
void RSTimedLoop::realTimeDelay() {
    this_thread::sleep_until(next_time);
    next_time += interval;
    // cout << next_time.time_since_epoch().count() - start_time.time_since_epoch().count() <<endl;
}
