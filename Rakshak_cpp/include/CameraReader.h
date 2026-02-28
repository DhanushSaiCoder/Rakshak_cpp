#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <atomic>
#include <thread>
#include <mutex>

class CameraReader {
public:
    CameraReader(const std::string& video_source, const std::string& name);
    ~CameraReader();

    void start();
    void stop();
    
    cv::Mat get_latest_frame();
    bool is_running() const { return running; }

private:
    void capture_loop();

    std::string source;
    std::string camera_name;
    cv::VideoCapture cap;
    
    cv::Mat latest_frame;
    std::mutex frame_mutex;
    
    std::atomic<bool> running{false};
    std::thread worker_thread;
};
