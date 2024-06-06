#include "arduinoController.h"
#include "hexapodLeg.h"
#include <iostream>

using namespace std;

ArduinoController arduino("/dev/ttyACM0", 115200);
HexapodLeg leg(1, arduino);

void parseCommand(string command);


int main() {
        string command;
    while (true) {
        cout << "Enter command: ";
        cin >> command;

        if (command == "exit") {
            arduino.~ArduinoController();
            break;
        };

        parseCommand(command);

        // arduino.sendCommand(command);
    }
    
    return 0;
}

void parseCommand(string command) {
    if (command == "zero") {
        leg.moveToZero();
    } else if (command == "basic") {
        leg.moveToBasic();
    } else if (command == "off") {
        leg.moveToOff();
    } else if (command == "ikTest") {
        leg.doIKTest();
    }
}