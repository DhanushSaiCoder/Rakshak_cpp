#include <iostream>
#include <csignal>
#include "RakshakSystem.h"

std::unique_ptr<RakshakSystem> system_ptr;

void signal_handler(int signal) {
    std::cout << "\n[INFO] Termination signal received. Stopping system..." << std::endl;
    if (system_ptr) {
        system_ptr->stop();
    }
    exit(signal);
}

int main() {
    std::cout << "Starting Rakshak C++ Integrated System..." << std::endl;

    // Register signal handler for Ctrl+C
    std::signal(SIGINT, signal_handler);

    try {
        system_ptr = std::make_unique<RakshakSystem>();
        system_ptr->start();

        std::cout << "[SUCCESS] Rakshak System is running." << std::endl;
        std::cout << "Press Ctrl+C to stop." << std::endl;

        // Keep main thread alive
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

    } catch (const std::exception& e) {
        std::cerr << "[CRITICAL] System crash: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
