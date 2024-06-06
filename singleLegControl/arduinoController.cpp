#include "arduinoController.h"
#include <iostream>
#include <boost/asio.hpp>
#include <thread>

ArduinoController::ArduinoController(const std::string& port, unsigned int baud_rate)
    : serial(io, port), running(true) {
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
}

ArduinoController::~ArduinoController() {
    closeConnection();
}

void ArduinoController::sendCommand(const std::string& command) {
    boost::asio::write(serial, boost::asio::buffer(command + "\n"));
}

std::string ArduinoController::readData() {
    char c;
    std::string result;
    boost::system::error_code ec;
    while (boost::asio::read(serial, boost::asio::buffer(&c, 1), ec)) {
        if (c == '\n') break;
        result += c;
    }
    if (ec && ec != boost::asio::error::eof) {
        std::cerr << "Error reading from Arduino: " << ec.message() << std::endl;
    }
    return result;
}

void ArduinoController::closeConnection() {
    running = false;
    if (serial.is_open()) {
        serial.close();
    }
}
