#include "MJPEGStreamer.h"
#include <iostream>
#include <vector>

MJPEGStreamer::MJPEGStreamer(int port) : port(port) {}

MJPEGStreamer::~MJPEGStreamer() {
    stop();
}

void MJPEGStreamer::start() {
    if (running) return;
    
    svr.Get("/:name", [&](const httplib::Request& req, httplib::Response& res) {
        std::string name = req.path_params.at("name");
        
        res.set_header("Content-Type", "multipart/x-mixed-replace; boundary=--jpgboundary");
        
        res.set_chunked_content_provider(
            "multipart/x-mixed-replace; boundary=--jpgboundary",
            [this, name](size_t offset, httplib::DataSink &sink) {
                if (!running) {
                    sink.done();
                    return false;
                }

                cv::Mat frame;
                {
                    std::lock_guard<std::mutex> lock(frames_mutex);
                    if (frames.count(name)) {
                        frame = frames[name].clone();
                    }
                }

                if (frame.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    return true;
                }

                std::vector<uchar> buf;
                cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 70});

                std::string header = "--jpgboundary\r\nContent-Type: image/jpeg\r\nContent-Length: " + 
                                     std::to_string(buf.size()) + "\r\n\r\n";
                
                sink.write(header.data(), header.size());
                sink.write(reinterpret_cast<const char*>(buf.data()), buf.size());
                sink.write("\r\n", 2);

                std::this_thread::sleep_for(std::chrono::milliseconds(30)); // Cap at ~30 FPS
                return true;
            }
        );
    });

    running = true;
    server_thread = std::thread(&MJPEGStreamer::server_loop, this);
}

void MJPEGStreamer::stop() {
    running = false;
    svr.stop();
    if (server_thread.joinable()) {
        server_thread.join();
    }
}

void MJPEGStreamer::set_frame(const std::string& name, const cv::Mat& frame) {
    if (frame.empty()) return;
    std::lock_guard<std::mutex> lock(frames_mutex);
    frames[name] = frame.clone();
}

void MJPEGStreamer::server_loop() {
    std::cout << "[INFO] MJPEG Streamer starting on port " << port << std::endl;
    if (!svr.listen("0.0.0.0", port)) {
        std::cerr << "[ERROR] MJPEG Streamer failed to listen on port " << port << std::endl;
    }
}
