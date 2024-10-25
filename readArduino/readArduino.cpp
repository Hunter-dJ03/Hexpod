#include "../modules/custom/arduinoConnection/arduinoController.h"
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <iomanip>
#include <ctime>


using namespace std;

// Function to read data from Arduino and print it
void readAndPrintData(ArduinoController& arduino) {
    while (true) {
        string data = arduino.readData();
        cout << "Arduino: " << data << endl;
    }
}

int main() {
    auto now = chrono::system_clock::now();
    time_t currentTime = chrono::system_clock::to_time_t(now);
    tm* localTime = localtime(&currentTime);
    cout << "readArduino START: " << put_time(localTime, "%Y-%m-%d %H:%M:%S");
    try {
        ArduinoController arduino("/dev/ttyACM0", 921600);
        readAndPrintData(arduino);
    } catch (const std::exception& e) {
        cerr << "Error: " << e.what() << endl;
        return EXIT_FAILURE;
    }

    return 0;
}
