#include "arduinoController.h"
#include "rs_timed_loop.h"
#include "hexapodLeg.h"
#include <iostream>
#include <fstream>

using namespace std;

RSTimedLoop rsLoop(1);
// ArduinoController arduino("/dev/ttyACM0", 115200);



void parseCommand(string command, HexapodLeg &leg);


int main() {
    fstream arduinoPort("/dev/ttyACM0");

    ArduinoController* arduino; // Declare the variable
    bool simulationMode;

    if (arduinoPort) {
        arduino = new ArduinoController("/dev/ttyACM0", 9600);
        simulationMode = false;
        cout << "Arduino connected." << endl;
        
    } else {
        arduino = new ArduinoController();
        simulationMode = true;
        cout << "Arduino not connected. Running in simulation mode." << endl;
    }
    
    HexapodLeg leg(1, *arduino, rsLoop, simulationMode);

    string command;
    while (true) {
        cout << "Enter command: ";
        cin >> command;

        if (command == "exit") {
            arduino->~ArduinoController();
            break;
        };

        parseCommand(command, leg);

        // arduino.sendCommand(command);
    }
    
    return 0;
}

void parseCommand(string command, HexapodLeg &leg) {
    if (command == "zero") {
        leg.moveToZero();
    } else if (command == "basic") {
        leg.moveToBasic();
    } else if (command == "off") {
        leg.moveToOff();
    } else if (command == "ikTest") {
        leg.doIKTest();
    } else if (command == "jacTest") {
        leg.doJacobianTest(1);
    }
}