#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "SharedState.h"

class YoloEngine {
public:
    YoloEngine(const std::string& model_path);
    ~YoloEngine();

    bool load_model();
    std::vector<DetectedObject> detect(const cv::Mat& frame);

private:
    std::string model_path;
    // TensorRT specific members would go here:
    // nvinfer1::ICudaEngine* engine;
    // nvinfer1::IExecutionContext* context;
};
