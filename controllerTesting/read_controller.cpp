#include <iostream>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <cmath>

// Define the deadzone value
const int DEADZONE = 2048;

void read_controller_input() {
    const char *device = "/dev/input/event20";  // Using event20 as specified
    int fd = open(device, O_RDONLY);
    
    if (fd == -1) {
        std::cerr << "Failed to open input device." << std::endl;
        return;
    }

    struct input_event ev;

    while (true) {
        ssize_t n = read(fd, &ev, sizeof(ev));
        if (n == (ssize_t)-1) {
            std::cerr << "Failed to read input event." << std::endl;
            break;
        }

        if (ev.type == EV_ABS || ev.type == EV_KEY) {
            // Handle deadzone for Right Trigger (code 9) and Left Trigger (code 10)
            if ((ev.code == 9 || ev.code == 10) && std::abs(ev.value) < DEADZONE) {
                continue; // Skip further processing if within the deadzone
            }

            switch (ev.code) {
                case 0: // Left Joystick X
                    std::cout << "Left Joystick X: " << ev.value << " (" 
                              << (ev.value < 0 ? "Left" : "Right") << ")" << std::endl;
                    break;
                case 1: // Left Joystick Y
                    std::cout << "Left Joystick Y: " << ev.value << " (" 
                              << (ev.value < 0 ? "Up" : "Down") << ")" << std::endl;
                    break;
                case 2: // Right Joystick X
                    std::cout << "Right Joystick X: " << ev.value << " (" 
                              << (ev.value < 0 ? "Left" : "Right") << ")" << std::endl;
                    break;
                case 5: // Right Joystick Y
                    std::cout << "Right Joystick Y: " << ev.value << " (" 
                              << (ev.value < 0 ? "Up" : "Down") << ")" << std::endl;
                    break;
                case 16: // Dpad X
                    std::cout << "Dpad X: " << ev.value << " (" 
                              << (ev.value < 0 ? "Left" : "Right") << ")" << std::endl;
                    break;
                case 17: // DPad Y
                    std::cout << "DPad Y: " << ev.value << " (" 
                              << (ev.value > 0 ? "Down" : "Up") << ")" << std::endl;
                    break;
                case 9: // Right Trigger
                    std::cout << "Right Trigger: " << ev.value << std::endl;
                    break;
                case 10: // Left Trigger
                    std::cout << "Left Trigger: " << ev.value << std::endl;
                    break;
                case 317: // Left Joystick Click
                    std::cout << "Left Joystick Click: " << ev.value << " (" 
                              << (ev.value == 1 ? "Pressed" : "Released") << ")" << std::endl;
                    break;
                case 318: // Right Joystick Click
                    std::cout << "Right Joystick Click: " << ev.value << " (" 
                              << (ev.value == 1 ? "Pressed" : "Released") << ")" << std::endl;
                    break;
                case 304: // Button A
                    std::cout << "Button A: " << ev.value << " (" 
                              << (ev.value == 1 ? "Pressed" : "Released") << ")" << std::endl;
                    break;
                case 307: // Button X
                    std::cout << "Button X: " << ev.value << " (" 
                              << (ev.value == 1 ? "Pressed" : "Released") << ")" << std::endl;
                    break;
                case 305: // Button B
                    std::cout << "Button B: " << ev.value << " (" 
                              << (ev.value == 1 ? "Pressed" : "Released") << ")" << std::endl;
                    break;
                case 308: // Button Y
                    std::cout << "Button Y: " << ev.value << " (" 
                              << (ev.value == 1 ? "Pressed" : "Released") << ")" << std::endl;
                    break;
                case 311: // Right Bumper
                    std::cout << "Right Bumper: " << ev.value << " (" 
                              << (ev.value == 1 ? "Pressed" : "Released") << ")" << std::endl;
                    break;
                case 310: // Left Bumper
                    std::cout << "Left Bumper: " << ev.value << " (" 
                              << (ev.value == 1 ? "Pressed" : "Released") << ")" << std::endl;
                    break;
                default:
                    std::cout << "Unknown event. Code: " << ev.code << " Value: " << ev.value << std::endl;
                    break;
            }
        }
    }

    close(fd);
}

void run_simulation() {
    // Simulation code goes here
    // For demonstration, we'll just print a message periodically
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Running simulation..." << std::endl;
    }
}

int main() {
    // Start the controller input reading in a separate thread
    std::thread input_thread(read_controller_input);

    // Run the simulation in the main thread
    run_simulation();

    // Join the input thread (although this will not be reached in this infinite loop example)
    input_thread.join();

    return 0;
}
