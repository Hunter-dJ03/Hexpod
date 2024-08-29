#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <cmath>

// Define the deadzone value
const int DEADZONE = 2048;

int main() {
    const char *device = "/dev/input/event20";  // Using event20 as specified
    int fd = open(device, O_RDONLY);
    
    if (fd == -1) {
        std::cerr << "Failed to open input device." << std::endl;
        return 1;
    }

    struct input_event ev;

    while (true) {
        ssize_t n = read(fd, &ev, sizeof(ev));
        if (n == (ssize_t)-1) {
            std::cerr << "Failed to read input event." << std::endl;
            break;
        }

        if (ev.type == EV_ABS || ev.type == EV_KEY) {
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
                    if (std::abs(ev.value) < DEADZONE) {
                        continue;
                    }
                    std::cout << "Right Trigger: " << ev.value << std::endl;
                    break;
                case 10: // Left Trigger
                    if (std::abs(ev.value) < DEADZONE) {
                        continue;
                    }
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
    return 0;
}
