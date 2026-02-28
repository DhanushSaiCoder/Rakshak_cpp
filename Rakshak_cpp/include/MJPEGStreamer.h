#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <map>
#include "httplib.h"

class MJPEGStreamer {
public:
    MJPEGStreamer(int port = 8443);
    ~MJPEGStreamer();

    void start();
    void stop();
    
    void set_frame(const std::string& name, const cv::Mat& frame);

private:
    void server_loop();

    int port;
    httplib::Server svr;
    std::atomic<bool> running{false};
    std::thread server_thread;

    std::map<std::string, cv::Mat> frames;
    std::mutex frames_mutex;
};
