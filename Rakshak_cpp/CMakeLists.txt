cmake_minimum_required(VERSION 3.10)
project(Rakshak_cpp VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Dependencies
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# Include Directories
include_directories(include)

# Source Files
set(SOURCES
    src/SharedState.cpp
    src/DataManager.cpp
    src/SerialPort.cpp
    src/TriggerController.cpp
    src/ZoomController.cpp
    src/CameraReader.cpp
    src/MJPEGStreamer.cpp
    src/PIDController.cpp
    src/MotorController.cpp
    src/YoloEngine.cpp
    src/RakshakSystem.cpp
    src/main.cpp
)

# Executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link Libraries
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    Threads::Threads
)
