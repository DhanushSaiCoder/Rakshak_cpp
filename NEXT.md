Phase 3: Hardware & Messaging (Highest Priority)
   1. Real Moteus Integration:
       * Replace MotorController.cpp mock methods with actual SocketCAN frames or the libmoteus C++ library.
       * Implement the asynchronous position/velocity command loop seen in modular_main_nt.py.
   2. Messaging C2 (RabbitMQ):
       * Since C++ AMQP libraries are complex to set up, implement a ZMQ or UDP Proxy: A tiny Python script handles RabbitMQ and forwards commands
         to the C++ system via a local socket.
       * Migrate message_controller.py logic to handle operating_mode_control, zoom_control, and fire_action.
   3. Sensor Intake:
       * Migrate modular_sensor_reader.py to a C++ thread.
       * Parse the <ii10f binary struct from the Arduino Sensors port natively.


  Phase 4: Vision & Inference Refinement
   1. Tracking Integration (ByteTrack):
       * OpenCV DNN provides detections but not tracking IDs. Integrate a C++ implementation of ByteTrack to maintain target persistence (Target ID)
         across frames.
   2. GPS & Magnetometer Math:
       * Migrate gps_yaw/modular_reader.py logic.
       * Implement calculate_target_gps using C++ <cmath> (asin, atan2, radians) to get target coordinates from depth and bearing.
   3. GPU Pre-processing:
       * Optimize the preprocess function in YoloEngine.cpp using CUDA kernels to handle normalization and NCHW conversion faster than the current
         CPU loop.


  Phase 5: Control Loop & Logic
   1. Firing State Machine:
       * Implement the full firing logic from shared_state.py: Short-Burst, Burst, and Safety modes.
       * Add the "Hold Timer": Logic to only fire if the target has been centered for auto_target_hold_time.
   2. Advanced PID Gains:
       * Implement "Dynamic PID" Gain scheduling (look up gains based on current zoom level and target distance).


  Phase 6: Infrastructure & Logging
   1. State Persistence:
       * Implement SharedState::save() and load() to read/write runtime_state.json on startup/shutdown.
   2. Telemetry Module:
       * Migrate tegrastats_reader.py to C++ by parsing the output of the tegrastats command.
       * Implement the Telemetry thread to report CPU/GPU temps and system health.
   3. Structured Logging:
       * Migrate the high-frequency logger. Create a Logger class that writes motor and sensor data to CSV or binary format in a non-blocking
         background thread.


  Phase 7: Final Optimization
   1. TensorRT 10 Re-integration: Once on the Jetson, optimize the ONNX model back into a hardware-locked .engine for maximum FPS.
   2. Zero-Copy Frame Passing: If latency is still an issue, implement cv::cuda::GpuMat passing between the Camera thread and the Inference thread
      to avoid CPU-GPU memory transfers.

  ---


  Next Recommended Task:
  Implement State Persistence (saving/loading your settings) or the Telemetry Module (Jetson stats monitoring).