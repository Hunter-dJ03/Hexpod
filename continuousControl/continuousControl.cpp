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
atomic<bool> leftBumper(false);
atomic<bool> rightBumper(false);
atomic<bool> buttonA(false);
atomic<bool> buttonB(false);
atomic<bool> buttonY(false);
atomic<bool> buttonX(false);
atomic<int> dPadX(0);
atomic<int> dPadY(0);

atomic<float> moveVectorMag;
atomic<float> moveVectorAng;

void signalHandler(int signum) {
    running = false; // Stop the loops
}

void readController(HexapodControl& hexapod) {
    const char *device = "/dev/input/event20";  // Using event20 as specified
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
                case 16: // Dpad X
                    // cout << "Dpad X: " << ev.value << " (" 
                    //           << (ev.value < 0 ? "Left" : "Right") << ")" << endl;
                    dPadX = ev.value;
                    break;
                case 17: // DPad Y
                    // cout << "DPad Y: " << ev.value << " (" 
                    //           << (ev.value > 0 ? "Down" : "Up") << ")" << endl;
                    dPadY = ev.value;
                    break;
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
                case 318: // Right Joystick Click
                    // cout << "Right Joystick Click: " << ev.value << " (" 
                    //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                    running = false;
                    break;
                case 304: // Button A
                    // cout << "Button A: " << ev.value << " (" 
                    //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                    buttonA = ev.value;
                    break;
                case 307: // Button X
                    // cout << "Button X: " << ev.value << " (" 
                    //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                    buttonX = ev.value;
                    break;
                case 305: // Button B
                    static bool wasButtonBPressed = false; // Track the previous state of Button Y
                    // cout<<"ButtonB Pressed: " <<endl;
                    if (ev.value == 1 && !wasButtonBPressed) {
                        // Button B is pressed and was not pressed before (i.e., transition from unpressed to pressed)
                        cout<<"ButtonB Pressed: " <<endl;
                        
                        buttonB = true;
                        
                        hexapod.operationDuration = 0;
                        
                        wasButtonBPressed = true; // Update the state to pressed
                    } else {
                        // Button B is released, reset the state
                        buttonB = false;
                        wasButtonBPressed = false;
                    }
                    break;
                case 308: // Button Y
                    static bool wasButtonYPressed = false; // Track the previous state of Button Y
                    
                    if (ev.value == 1 && !wasButtonYPressed) {
                        // Button Y is pressed and was not pressed before (i.e., transition from unpressed to pressed)
                        buttonY = true;
                        
                        // Output the desired information
                        cout << "Current Angles: " << hexapod.currentAngles << endl;
                        cout << "Desired Angles: " << hexapod.desiredAngles << endl;
                        hexapod.printPos();
                        cout <<endl;
                        
                        wasButtonYPressed = true; // Update the state to pressed
                    } else {
                        // Button Y is released, reset the state
                        buttonY = false;
                        wasButtonYPressed = false;
                    }
                    break;
                case 311: // Right Bumper
                    // cout << "Right Bumper: " << ev.value << " (" 
                    //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                    rightBumper = ev.value;
                    break;
                case 310: // Left Bumper
                    // cout << "Left Bumper: " << ev.value << " (" 
                    //           << (ev.value == 1 ? "Pressed" : "Released") << ")" << endl;
                    leftBumper = ev.value;
                    break;
                default:
                    // cout << "Unknown event. Code: " << ev.code << " Value: " << ev.value << endl;
                    break;
            }
        }

        moveVectorMag = Utils::constrain(sqrt(pow(inputAxisX, 2) + pow(inputAxisY, 2)), 0, 2047)   / 2047 ;
        moveVectorAng = atan2(-inputAxisY, inputAxisX);

        if (moveVectorMag <0.2 && hexapod.directed) {
            hexapod.operationDuration = 0;
        } 

        hexapod.directed = (moveVectorMag >= 0.2);
    }

    close(fd);
}

void runHexapod(HexapodControl& hexapod) {
    
    // Record start time
    auto startTime = chrono::high_resolution_clock::now();
    hexapod.desiredAngles = hexapod.simulator->convertVecDynToEigen(hexapod.simulator->hexapodLegModel->getGeneralizedCoordinate());
    // cout<<hexapod.desiredAngles;
    hexapod.rsLoop.updateTimeDelay();

    while (running) {

        hexapod.currentAngles = hexapod.simulator->convertVecDynToEigen(hexapod.simulator->hexapodLegModel->getGeneralizedCoordinate());
        hexapod.currentAngularVelocities = hexapod.simulator->convertVecDynToEigen(hexapod.simulator->hexapodLegModel->getGeneralizedVelocity());

        hexapod.updatePos();
        
        // Preset holding position
        hexapod.simulator->setSimVelocity(hexapod.desiredAngles, Eigen::VectorXd::Zero(18));

        if (hexapod.active) {

            if (moveVectorMag >= 0.2) {
                
                hexapod.walk(moveVectorMag, moveVectorAng);

                cout << "\rMove Vector: " << moveVectorMag *100 << "\% at "<< moveVectorAng << endl;

                if (moveVectorMag < 0.2) {
                    hexapod.moveToStand(500);
                }  
            } 


            if (buttonX) {
                if (dPadX == -1) {
                    hexapod.jacobianTest(0);
                } else if (dPadX == 1) {
                    hexapod.jacobianTest(1);
                } else if (dPadY == 1) {
                    hexapod.jacobianTest(2);
                }
            }

        }
        
        // hexapod.jumpToOff();

        if (leftBumper && rightBumper) {

            if (!hexapod.active) {
                cout<<endl<<"standing" <<endl;
                hexapod.active = true;
                hexapod.stand();
                
            } else {
                cout<<endl<<"turning off" <<endl;
                hexapod.active = false;
                hexapod.moveToOff();
                
            }
           
        } 
        
        // if (buttonY) {
        //     cout<<"Current Angles: "<< hexapod.currentAngles<<endl;
        //     cout<<"Desired Angles: "<< hexapod.desiredAngles<<endl;
        //     hexapod.printPos();
        // }

        hexapod.rsLoop.realTimeDelay();
        hexapod.simulator->server->integrateWorldThreadSafe();
    }

    // Calculate the duration
    auto endTime = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed = endTime - startTime;

    hexapod.simulator->server->killServer();
    
    // Output the duration
    cout << "\nrunHexapod loop duration: " << elapsed.count() << " seconds." << endl;
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
    thread input_thread(readController, ref(hexapod));
    // Register signal handler
    signal(SIGINT, signalHandler);

    // Run the simulation in the main thread
    runHexapod(hexapod);

    return 0;
}
