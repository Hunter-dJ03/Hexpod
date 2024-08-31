#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <cmath>
#include <thread>
#include <atomic>
#include <chrono>
#include <csignal>
#include "../modules/custom/utilities/utils.h"
#include <iomanip>
#include "raisim/RaisimServer.hpp"
#include "../modules/custom/arduinoConnection/arduinoController.h"
#include "../modules/custom/rsTimedLoop/rsTimedLoop.h"
#include "hexapod.h"
#include <fstream>
#include <memory>


using namespace std;

const bool raisimSimulator = true;
const float rsStep = 10; // Real Time Step (ms)

RSTimedLoop rsLoop(rsStep);

atomic<bool> running(true);
atomic<float> inputAxisX(0.0f);
atomic<float> inputAxisY(0.0f);

void signalHandler(int signum) {
    running = false; // Stop the loops
}

void readController() {
    const char *device = "/dev/input/event25";  // Using event20 as specified
    int fd = open(device, O_RDONLY);
    
    if (fd == -1) {
        cerr << "Failed to open input device." << endl;
        return;
    }

    struct input_event ev;

    while (running) {
        ssize_t n = read(fd, &ev, sizeof(ev));
        if (n == (ssize_t)-1) {
            cerr << "Failed to read input event." << endl;
            break;
        }

        if (ev.type == EV_ABS || ev.type == EV_KEY) {
            // Handle deadzone for Right Trigger (code 9) and Left Trigger (code 10)
            if ((ev.code == 9 || ev.code == 10) && abs(ev.value) < 2048) {
                continue; // Skip further processing if within the deadzone
            }

            switch (ev.code) {
                case 0: // Left Joystick X
                    // cout << "Left Joystick X: " << ev.value << " (" 
                    //           << (ev.value < 0 ? "Left" : "Right") << ")" << endl;

                    inputAxisX = ev.value;
                    break;
                case 1: // Left Joystick Y
                    // cout << "Left Joystick Y: " << ev.value << " (" 
                    //           << (ev.value < 0 ? "Up" : "Down") << ")" << endl;

                    inputAxisY = ev.value;
                    break;
                // case 2: // Right Joystick X
                //     // cout << "Right Joystick X: " << ev.value << " (" 
                //     //           << (ev.value < 0 ? "Left" : "Right") << ")" << endl;
                //     break;
                // case 5: // Right Joystick Y
                //     // cout << "Right Joystick Y: " << ev.value << " (" 
                //     //           << (ev.value < 0 ? "Up" : "Down") << ")" << endl;
                //     break;
                // case 16: // Dpad X
                //     // cout << "Dpad X: " << ev.value << " (" 
                //     //           << (ev.value < 0 ? "Left" : "Right") << ")" << endl;
                //     break;
                // case 17: // DPad Y
                //     // cout << "DPad Y: " << ev.value << " (" 
                //     //           << (ev.value > 0 ? "Down" : "Up") << ")" << endl;
                //     break;
                // case 9: // Right Trigger
                //     // cout << "Right Trigger: " << ev.value << endl;
                //     break;
                // case 10: // Left Trigger
                //     // cout << "Left Trigger: " << ev.value << endl;
                //     break;
                // case 317: // Left Joystick Click
                //     // cout << "Left Joystick Click: " << ev.value << " (" 
                //     //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                //     break;
                // case 318: // Right Joystick Click
                //     // cout << "Right Joystick Click: " << ev.value << " (" 
                //     //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                //     break;
                // case 304: // Button A
                //     // cout << "Button A: " << ev.value << " (" 
                //     //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                //     break;
                // case 307: // Button X
                //     // cout << "Button X: " << ev.value << " (" 
                //     //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                //     break;
                // case 305: // Button B
                //     // cout << "Button B: " << ev.value << " (" 
                //     //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                //     break;
                // case 308: // Button Y
                //     // cout << "Button Y: " << ev.value << " (" 
                //     //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                //     break;
                // case 311: // Right Bumper
                //     // cout << "Right Bumper: " << ev.value << " (" 
                //     //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                //     break;
                // case 310: // Left Bumper
                //     // cout << "Left Bumper: " << ev.value << " (" 
                //     //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                //     break;
                default:
                    // cout << "Unknown event. Code: " << ev.code << " Value: " << ev.value << endl;
                    break;
            }
        }
    }

    close(fd);
}

void runHexapod(HexapodControl& Hexapod) {
    // Simulation code goes here
    
    Hexapod.rsLoop.updateTimeDelay();

    while (running) {
        float moveVectorMag = Utils::constrain(sqrt(pow(inputAxisX, 2) + pow(inputAxisY, 2)), 0, 2047) / 2047 * 100;
        float moveVectorAng = atan2(-inputAxisY, inputAxisX);

        // Remap the angle from [-π, π] to [0, 2π]
        if (moveVectorAng < 0) {
            moveVectorAng += 2 * M_PI;
        }

        // Set precision and width for consistent output
        cout << "\rMove Vector: " 
                  << fixed << setprecision(2) << setw(6) << moveVectorMag << "\% at "
                  << fixed << setprecision(2) << setw(6) << moveVectorAng 
                  << flush;

        Hexapod.rsLoop.realTimeDelay();
        Hexapod.simulator->server->integrateWorldThreadSafe();
    }
}

int main(int argc, char* argv[]) {

    // Set the path for raisim simulator
    Path binaryPath = raisim::Path::setFromArgv(argv[0]);

    // Setup communication file stream for arduino connection
    fstream arduinoPort("/dev/ttyACM0");

    // Create arduino controller as blank unique pointer
    unique_ptr<ArduinoController> arduino;

    // Assume arduino Connected
    bool arduinoConnected = true;

    if (arduinoPort) {
        // Set the Arduino Controller up with port and abud rate
        arduino = make_unique<ArduinoController>("/dev/ttyACM0", 921600);

        cout << "Arduino connected." << endl;
        
    } else {
        // Set arduino as a blank controller
        arduino = make_unique<ArduinoController>();

        // Change sim mode
        arduinoConnected = false;

        cout << "Arduino not connected. Running in simulation mode." << endl;
    }

    // Create Hexapod 
    HexapodControl hexapod(1, move(arduino), rsLoop, arduinoConnected, raisimSimulator, rsStep, binaryPath);

    // Start the controller input reading in a separate thread
    thread input_thread(readController);

    // Register signal handler
    signal(SIGINT, signalHandler);

    // Run the simulation in the main thread
    runHexapod(hexapod);

    return 0;
}
