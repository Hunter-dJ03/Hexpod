#include "arduinoController.h"
#include <iostream>
#include <cstdlib>

using namespace std;

int main() {
    ArduinoController arduino("/dev/ttyACM0", 9600);

    // Read data from Arduino and print it
    while (true) {
        string data = arduino.readData();
        cout << "Arduino: " << data << endl;
    }

    return 0;
}
