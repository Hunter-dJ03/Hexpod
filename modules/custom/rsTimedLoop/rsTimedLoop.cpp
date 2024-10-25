#include "rsTimedLoop.h"
#include "iostream"

using namespace std;

// Constructor to set dealy till next time
RSTimedLoop::RSTimedLoop(int interval_ms) : interval(interval_ms) {
    start_time = chrono::high_resolution_clock::now();
    updateTimeDelay();
}

// Delay thread until preset next time and set the next time for delay
void RSTimedLoop::realTimeDelay() {
    this_thread::sleep_until(next_time);
    // prev_time = next_time;
    next_time += interval;
    // cout << next_time.time_since_epoch().count() - prev_time.time_since_epoch().count() <<endl;
}

// Manually update the next time for real time delay
void RSTimedLoop::updateTimeDelay()
{
    next_time = chrono::high_resolution_clock::now() + interval;
}
