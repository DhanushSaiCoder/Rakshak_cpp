#include "CameraReader.h"
#include <iostream>

CameraReader::CameraReader(const std::string& video_source, const std::string& name)
    : source(video_source), camera_name(name) {}

CameraReader::~CameraReader() {
    stop();
}

void CameraReader::start() {
    if (running) return;
    
    // Open camera
    // If source is a number, convert to int
    try {
        int idx = std::stoi(source);
        cap.open(idx);
    } catch (...) {
        cap.open(source);
    }

    if (!cap.isOpened()) {
        std::cerr << "[ERROR] Failed to open camera: " << camera_name << " source: " << source << std::endl;
        return;
    }

    cap.set(cv::CAP_PROP_BUFFERSIZE, 1); // Minimize latency
    
    running = true;
    worker_thread = std::thread(&CameraReader::capture_loop, this);
}

void CameraReader::stop() {
    running = false;
    if (worker_thread.joinable()) {
        worker_thread.join();
    }
    if (cap.isOpened()) {
        cap.release();
    }
}

cv::Mat CameraReader::get_latest_frame() {
    std::lock_guard<std::mutex> lock(frame_mutex);
    return latest_frame.clone();
}

void CameraReader::capture_loop() {
    cv::Mat frame;
    while (running) {
        if (cap.read(frame)) {
            if (frame.empty()) continue;
            
            std::lock_guard<std::mutex> lock(frame_mutex);
            frame.copyTo(latest_frame);
        } else {
            std::cerr << "[WARN] Failed to grab frame from " << camera_name << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}
