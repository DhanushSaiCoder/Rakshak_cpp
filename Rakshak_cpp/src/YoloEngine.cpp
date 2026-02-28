#include "YoloEngine.h"
#include <fstream>
#include <iostream>

YoloEngine::YoloEngine(const std::string& model_path) : model_path(model_path) {}

bool YoloEngine::load_model() {
    try {
        net = cv::dnn::readNetFromONNX(model_path);
        
        // Attempt to use CUDA
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        
        std::cout << "[YOLO] Model loaded successfully: " << model_path << std::endl;
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "[YOLO][ERROR] Failed to load ONNX model: " << e.what() << std::endl;
        return false;
    }
}

std::vector<DetectedObject> YoloEngine::detect(const cv::Mat& frame) {
    if (frame.empty()) return {};

    // 1. Preprocess
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1.0/255.0, cv::Size(input_w, input_h), cv::Scalar(), true, false);
    net.setInput(blob);

    // 2. Forward pass
    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    // 3. Postprocess
    // Output shape is [1, 84, 8400]
    cv::Mat output = outputs[0];
    if (output.dims == 3) {
        // Reshape to [84, 8400]
        int sz[] = {output.size[1], output.size[2]};
        output = cv::Mat(2, sz, CV_32F, output.ptr<float>());
    }
    output = output.t(); // Transpose to [8400, 84]

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    float scale_w = (float)frame.cols / input_w;
    float scale_h = (float)frame.rows / input_h;

    for (int i = 0; i < output.rows; ++i) {
        float* row = output.ptr<float>(i);
        float* scores = row + 4;
        
        cv::Mat scores_mat(1, 80, CV_32F, scores); // Assuming 80 classes
        cv::Point class_id_point;
        double max_score;
        cv::minMaxLoc(scores_mat, nullptr, &max_score, nullptr, &class_id_point);

        if (max_score > conf_threshold) {
            float cx = row[0] * scale_w;
            float cy = row[1] * scale_h;
            float w = row[2] * scale_w;
            float h = row[3] * scale_h;

            int left = int(cx - w / 2);
            int top = int(cy - h / 2);

            class_ids.push_back(class_id_point.x);
            confidences.push_back((float)max_score);
            boxes.push_back(cv::Rect(left, top, int(w), int(h)));
        }
    }

    // 4. NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);

    std::vector<DetectedObject> detections;
    for (int idx : indices) {
        DetectedObject obj;
        obj.id = idx; // Placeholder for tracker
        obj.cls = class_ids[idx];
        obj.w = boxes[idx].width;
        obj.h = boxes[idx].height;
        obj.fx = boxes[idx].x + obj.w / 2.0;
        obj.fy = boxes[idx].y + obj.h / 2.0;
        obj.distance = 0.0;
        detections.push_back(obj);
    }

    return detections;
}
