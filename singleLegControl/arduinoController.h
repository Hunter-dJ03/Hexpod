#ifndef ARDUINOCONTROLLER_H
#define ARDUINOCONTROLLER_H

#include <string>
#include <boost/asio.hpp>
#include <thread>
#include <atomic>

using namespace std;

class ArduinoController {
public:
    ArduinoController(const string& port, unsigned int baud_rate);
    ~ArduinoController();

    void sendCommand(const string& command);
    void closeConnection();
    string readData();

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
    thread read_thread;
    atomic<bool> running;
};

#endif // ARDUINOCONTROLLER_H
