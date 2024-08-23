#include <SDL2/SDL.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>

std::atomic<bool> running(true);
std::atomic<float> inputAxisX(0.0f);
std::atomic<float> inputAxisY(0.0f);

void signalHandler(int signum) {
    running = false; // Stop the loops
}

void keyboardInputHandler() {
    // Initialize SDL2 and create a window
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl;
        return;
    }

    SDL_Window* window = SDL_CreateWindow("SDL2 Keyboard Input",
                                          SDL_WINDOWPOS_UNDEFINED,
                                          SDL_WINDOWPOS_UNDEFINED,
                                          640, 480,
                                          SDL_WINDOW_SHOWN);
    if (!window) {
        std::cerr << "Failed to create window: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return;
    }

    SDL_Event event;

    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
            if (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP) {
                float value = (event.type == SDL_KEYDOWN) ? 1.0f : 0.0f;

                switch (event.key.keysym.sym) {
                    case SDLK_w: // Forward
                        inputAxisY = inputAxisY-value;
                        break;
                    case SDLK_s: // Backward
                        inputAxisY = inputAxisY+value;
                        break;
                    case SDLK_a: // Left
                        inputAxisX = inputAxisX-value;
                        break;
                    case SDLK_d: // Right
                        inputAxisX = inputAxisX+value;
                        break;
                    case SDLK_ESCAPE: // Exit on Escape
                        running = false;
                        break;
                    default:
                        break;
                }

                // Print which key was pressed or released
                // std::cout << "Key event: " << SDL_GetKeyName(event.key.keysym.sym)
                //           << " | X: " << inputAxisX.load()
                //           << " | Y: " << inputAxisY.load() << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    SDL_DestroyWindow(window);
    SDL_Quit();
}

int main() {
    // Register signal handler
    signal(SIGINT, signalHandler);

    // Start the keyboard input handler thread
    std::thread keyboardThread(keyboardInputHandler);

    // Main loop: Print X and Y axis values
    while (running) {
        float axisX = inputAxisX.load();
        float axisY = inputAxisY.load();

        std::cout
                          << " | X: " << axisX
                          << " | Y: " << axisY << std::endl;


        // Sleep for 100 ms to reduce console spam
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Stop the keyboard input handler thread
    keyboardThread.join();

    return 0;
}
