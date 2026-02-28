# Rakshak C++ Core File Descriptions

This document provides a map of the migrated C++ codebase to assist in future development and debugging.

## System Orchestration
*   **`src/main.cpp`**: The application entry point. It handles system signals (Ctrl+C), initializes the `RakshakSystem` orchestrator, and performs "Health Gating" to report if the system is running in a fully functional or degraded state.
*   **`include/RakshakSystem.h` & `src/RakshakSystem.cpp`**: The primary orchestrator. It manages the lifecycles of the Vision, Control, and Health threads. It handles the high-level business logic, such as switching between cameras if one is missing (Camera Fallback).

## State & Data Management
*   **`include/SharedState.h` & `src/SharedState.cpp`**: The "Single Source of Truth." Replaces Python's `multiprocessing.Manager`. Uses `std::atomic` for flags and `std::shared_mutex` for high-frequency detection data to ensure thread-safe access across the system.
*   **`include/Globals.h`**: A header-only file containing all system constants, reduction ratios, limits, and file paths.
*   **`include/OpMode.h`**: Defines the `OpMode` enum class (Manual, Auto, Semi-Auto, etc.).
*   **`include/DataManager.h` & `src/DataManager.cpp`**: Responsible for loading and parsing JSON datasets (FOV, PID gains, Depth-Height maps). It provides lookup methods for other subsystems.
*   **`include/MathUtils.h`**: Provides optimized mathematical helpers, specifically linear interpolation for depth estimation and motor command mapping.

## Vision & Inference
*   **`include/CameraReader.h` & `src/CameraReader.cpp`**: Manages high-speed frame capture using OpenCV in a dedicated thread. It utilizes a producer-consumer pattern to provide the latest frame with minimal latency.
*   **`include/YoloEngine.h` & `src/YoloEngine.cpp`**: Handles target detection using YOLOv11. It utilizes the OpenCV DNN module for ONNX portability, performing pre-processing (blob creation), forward pass, and NMS (Non-Maximum Suppression).
*   **`include/MJPEGStreamer.h` & `src/MJPEGStreamer.cpp`**: A multi-threaded HTTP server (via `cpp-httplib`) that streams live video feeds with bounding boxes to a browser.

## Control & Hardware
*   **`include/PIDController.h` & `src/PIDController.cpp`**: Implements the tracking "brain." It calculates error offsets and accumulates integrals to produce precise motor adjustment commands based on target pixel coordinates.
*   **`include/MotorController.h` & `src/MotorController.cpp`**: An abstraction layer for motor communication. Currently implements a mock interface, ready to be integrated with SocketCAN for Moteus controllers.
*   **`include/SerialPort.h` & `src/SerialPort.cpp`**: A low-level Linux serial wrapper using `termios`. Handles the raw byte streams for Arduino and VISCA communication.
*   **`include/TriggerController.h` & `src/TriggerController.cpp`**: Manages the hardware firing system (Laser and Solenoid) via serial commands.
*   **`include/ZoomController.h` & `src/ZoomController.cpp`**: Implements the VISCA protocol over serial to control the optical zoom levels of the cameras.

## Infrastructure
*   **`CMakeLists.txt`**: The build configuration file. Links OpenCV, CUDA, and Threads, and manages the compilation of all source files.
*   **`.gitignore`**: Ensures build artifacts, binaries, and large model files are not tracked by git.
