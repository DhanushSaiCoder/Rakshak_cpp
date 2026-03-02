#pragma once

#include "SharedState.h"
#include "DataManager.h"
#include "CameraReader.h"
#include "MJPEGStreamer.h"
#include "TriggerController.h"
#include "ZoomController.h"
#include "MotorController.h"
#include "PIDController.h"
#include "YoloEngine.h"
#include "MessageBridge.h"
#include "SensorReader.h"

#include <thread>
#include <atomic>
#include <memory>

class RakshakSystem {
public:
    RakshakSystem();
    ~RakshakSystem();

    bool start();
    void stop();

private:
    void vision_thread_func();
    void control_thread_func();
    void health_thread_func();

    SharedState state;
    DataManager data_manager;
    
    std::unique_ptr<CameraReader> static_cam;
    std::unique_ptr<CameraReader> top_cam;
    std::unique_ptr<MJPEGStreamer> streamer;
    std::unique_ptr<TriggerController> trigger;
    std::unique_ptr<ZoomController> zoom_ctrl;
    std::unique_ptr<MotorController> motors;
    std::unique_ptr<PIDController> pid;
    std::unique_ptr<YoloEngine> yolo;
    std::unique_ptr<MessageBridge> mq_bridge;
    std::unique_ptr<SensorReader> sensors;

    std::thread vision_thread;
    std::thread control_thread;
    std::thread health_thread;
    
    std::atomic<bool> running{false};
};
