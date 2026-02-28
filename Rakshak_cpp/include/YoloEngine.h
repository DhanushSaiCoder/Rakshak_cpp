#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>
#include "SharedState.h"

class YoloEngine {
public:
    YoloEngine(const std::string& model_path);
    ~YoloEngine() = default;

    bool load_model();
    std::vector<DetectedObject> detect(const cv::Mat& frame);

private:
    std::string model_path;
    cv::dnn::Net net;

    const int input_w = 640;
    const int input_h = 640;
    const float conf_threshold = 0.35f;
    const float nms_threshold = 0.45f;
};
