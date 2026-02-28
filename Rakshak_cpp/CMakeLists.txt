cmake_minimum_required(VERSION 3.10)
project(Rakshak_cpp VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Dependencies
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(CUDA REQUIRED)

# TensorRT (NVIDIA)
set(TENSORRT_INCLUDE_DIR /usr/include/x86_64-linux-gnu)
set(TENSORRT_LIB_DIR /usr/lib/x86_64-linux-gnu)
find_library(NVINFER_LIB nvinfer PATHS ${TENSORRT_LIB_DIR})
find_library(NVONNXPARSER_LIB nvonnxparser PATHS ${TENSORRT_LIB_DIR})

# Include Directories
include_directories(include ${CUDA_INCLUDE_DIRS} ${TENSORRT_INCLUDE_DIR})

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
    ${CUDA_LIBRARIES}
    ${NVINFER_LIB}
    ${NVONNXPARSER_LIB}
    Threads::Threads
)
