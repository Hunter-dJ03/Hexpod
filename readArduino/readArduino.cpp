#include "../modules/custom/arduinoConnection/arduinoController.h"
#include <iostream>
#include <cstdlib>

using namespace std;

// Function to read data from Arduino and print it
void readAndPrintData(ArduinoController& arduino) {
    while (true) {
        string data = arduino.readData();
        cout << "Arduino: " << data << endl;
    }
}

int main() {
    try {
        ArduinoController arduino("/dev/ttyACM0", 115200);
        readAndPrintData(arduino);
    } catch (const std::exception& e) {
        cerr << "Error: " << e.what() << endl;
        return EXIT_FAILURE;
    }

    return 0;
}
