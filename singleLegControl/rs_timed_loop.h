#ifndef RS_TIMED_LOOP_H
#define RS_TIMED_LOOP_H

#include <chrono>
#include <thread>

using namespace std;

class RSTimedLoop {
public:
    RSTimedLoop(int interval_ms);
    void realTimeDelay();

private:
    chrono::milliseconds interval;  // Real Time step interval
    chrono::time_point<chrono::high_resolution_clock> next_time;    // Time to delay until
};

#endif // RS_TIMED_LOOP_H
