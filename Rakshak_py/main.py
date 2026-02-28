import cv2
import time
import numpy
import torch
from ultralytics import YOLO
import moteus # type: ignore

# If your input sizes are fixed-ish, let cuDNN pick fastest algos
torch.backends.cudnn.benchmark = True
from multiprocessing import Process
import threading
import warnings
warnings.filterwarnings("ignore", message="The value of the smallest subnormal*")
from stream_server import run_https_server
from ultralytics.utils import LOGGER
LOGGER.setLevel('WARNING') 
import asyncio
from share_frame import SharedFrame
import sys
import re
from MQ_Base import BaseMQ_New
from log_util import setup_combined_logger
import inspect
import traceback
import Globals
from modular_sensor_reader import SensorReader,SensorLogger
from gps_yaw.modular_reader import SensorReaderGPS
import json

# ==============
from shared_state import SharedState
from op_mode import OpMode
from message_controller import MessageController
from camera_reader import camera_reader, take_snapshot, video_recorder_process, yolo_inference_loop
from pathlib import Path
from telemetry_module import telemetry_process
# ==============

logger=setup_combined_logger(__name__)


if not Globals.SYSTEM_ID:
    raise ValueError("[BaseMQ Init] SYSTEM_ID not found in globals.py")

SYS = Globals.SYSTEM_ID.strip()
SYS = re.sub(r"[^a-zA-Z0-9_.-]", "", SYS)  # sanitize

if not SYS:
    raise ValueError("[BaseMQ Init] SYSTEM_ID is empty or invalid after sanitization")

SYS = SYS.lower()  # normalize for routing keys, queue names


MQ_PUB = BaseMQ_New(queue=f"{SYS}.pub.q", connection_name=f"{SYS}-pub")  # queue unused for pub but harmless

class FPSCounter:
    def __init__(self, name='Default'):
        self.name = name
        self.start_time = time.time()
        self.frame_count = 0
        self.latest_fps=0
 
    def update(self):
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        if elapsed >= 1.0:
            # print(f"[{self.name}] FPS: {self.frame_count:.2f}")
            self.latest_fps=self.frame_count
            self.frame_count = 0
            self.start_time = time.time()
    
calibration_mode = True
async def run_static_control(motor_controller,yaw_pos, pitch_pos, yaw_cmd, pitch_cmd, height, zoom):
    # print(f'Static cur pos : {yaw_pos},pitch pos: {pitch_pos,}')
    # return await motor_controller.StaticControl(yaw_pos, pitch_pos, yaw_cmd, pitch_cmd,height, zoom)
    if not calibration_mode and height is not None:
        return await motor_controller.StaticDynamicControl(yaw_pos, pitch_pos, yaw_cmd, pitch_cmd, height, zoom)
    else:
        return await motor_controller.StaticControl(yaw_pos, pitch_pos, yaw_cmd, pitch_cmd)

async def run_pid_control(motor_controller,YawCurrentPos,PitchCurrentPos,error_x,error_y,delta_t,PrevIx,PrevIy,zoom,height):
    # print(f'PID cur pos : {YawCurrentPos},pitch pos: {PitchCurrentPos,}')
    # return await motor_controller.PIDControl(YawCurrentPos,PitchCurrentPos,error_x,error_y,delta_t,PrevIx,PrevIy,zoom,height)
    if calibration_mode:
        return await motor_controller.PIDControl(YawCurrentPos,PitchCurrentPos,error_x,error_y,delta_t,PrevIx,PrevIy,zoom,height)
    else:
        return await motor_controller.PIDDynamicControl(YawCurrentPos,PitchCurrentPos,error_x,error_y,delta_t,PrevIx,PrevIy,zoom,height)

async def run_calibration(motor_controller,width,height):
    return await motor_controller.Calibrate(width,height)

async def run_velocity_control(motor_controller,yaw_velocity,pitch_velocity):
    return await motor_controller.velocity_control(yaw_velocity,pitch_velocity)

async def run_scan_mode(motor_controller,yaw_pos_cmd,pitch_pos_cmd,cam_pos_cmd):
    return await motor_controller.Scan_mode(yaw_pos_cmd,pitch_pos_cmd,cam_pos_cmd)

def compute_hit_accuracy_from_commanded(commanded_yaw, actual_yaw, commanded_pitch, actual_pitch):
    def safe_percentage_error(commanded, actual, axis=""):
        # NEW: Use small constant in denominator when commanded is near zero
        denominator = abs(commanded) if abs(commanded) >= 1e-2 else 1.0  # Use 1.0 to still compute % error from 0

        error = abs(commanded - actual) / denominator * 100

        # print(f"[DEBUG][{axis}] Cmd: {commanded:.2f}, Act: {actual:.2f}, Error: {error:.2f}%")
        return min(error, 100.0)

    yaw_error = safe_percentage_error(commanded_yaw, actual_yaw, axis="YAW")
    pitch_error = safe_percentage_error(commanded_pitch, actual_pitch, axis="PITCH")

    return yaw_error, pitch_error

async def pid_control_loop(shm_static, shm_top, shm_top_updated, shared_state):
    # ==============================================================
    # 1. READERS & MODELS (once)
    # ==============================================================
    static_reader = SharedFrame(name=shm_static)
    top_reader    = SharedFrame(name=shm_top)
    top_updated_writer    = SharedFrame(name=shm_top_updated)

    # === YOLO MODELS + WARMUP ===
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = Path(shared_state.ns.top_model)
    # print(f'model -> {model}')
    top_model  = YOLO(model, verbose=False).to(device)
    # person_pid_model= YOLO(Globals.PERSON_MODEL,verbose=False).to(device)
    pid_tracker="trackers/bytetrack.yaml"

    dummy = numpy.zeros((256, 256, 3), numpy.uint8)
    for model in (top_model,):
        with torch.inference_mode(), torch.amp.autocast(device, dtype=torch.float16):
            _ = model.track(source=dummy, persist=True, iou=0.8, max_det=1,
                            classes=[0,80,81], conf=0.35, device=device,
                            tracker="trackers/bytetrack.yaml")
    torch.cuda.synchronize()
    logger.info("[PID] Models warmed up")

    # ==============================================================
    # 2. READ STATIC FRAME ONCE → GET RESOLUTION
    # ==============================================================
    frame_static_sample = static_reader.read()
    frame_top_sample = top_reader.read()
    if frame_static_sample is None or frame_top_sample is None:
        raise RuntimeError("Failed to grab one of the camera frame")
    
    H_FULL, W_FULL = frame_static_sample.shape[:2]
    H_HALF = H_FULL // 2  
    W_HALF = W_FULL // 2    
    
    # ==============================================================
    # 3. FOV CACHE (static + pid)
    # ==============================================================
    def _load_fov(subkey: str):
        entry = shared_state.dataset_manager.get("fov_data", shared_state.zoom_level.value, subkey=subkey)
        return tuple(entry["horizontal"]), tuple(entry["vertical"])

    static_fov = _load_fov("static_fov")   # (HFOV, VFOV)
    pid_fov    = _load_fov("pid_fov")     # (HFOV_pid, VFOV_pid)
    
    # ==============================================================
    # 4. OTHER ONCE-ONLY INIT
    # ==============================================================
    mq_publisher = BaseMQ_New(queue=f"{SYS}_pid_loop.pub.q", connection_name=f"{SYS}_pid_loop-pub")
    fps_counter    = FPSCounter()

    await shared_state.motor_controller.probe_motors()
    # print(f' res ->{motors_status}')
    # Initial motor stop
    if shared_state.motors_ready.value == True:
        yaw_state, pitch_state = await shared_state.motor_controller.stop_motors()
        if yaw_state and pitch_state is not None:
            YawCurrentPos  = Globals.YAW_POS_CONST * yaw_state.values[moteus.Register.POSITION]
            PitchCurrentPos = Globals.PITCH_POS_CONST * pitch_state.values[moteus.Register.POSITION]
    else:
        shared_state.motor_controller.send_alert()

    p_time =time.time()
    
    tracking_start_time = time.time()
    seconds = 0
    firing = False
    zeroing = True
    
    last_mode = shared_state.mode.value
    held_for_mode_change = False

    logger.info(f'pid_control_loop proc started')
    while not shared_state.stop_flag.value:
        cur_mode = shared_state.mode.value
        try:
            # Frame reading
            frame_top=top_reader.read()
            # after each successful cap.read()
            if(shared_state.zoom_set.value):
                shared_state.zoom_set.value=False
                static_fov = _load_fov("static_fov")
                pid_fov    = _load_fov("pid_fov")
                
            elif(shared_state.calib_set.value):
                shared_state.calib_set.value=False
                if shared_state.motors_ready.value == True:
                    motor_task = asyncio.create_task(run_calibration(shared_state.motor_controller,W_FULL,H_FULL))
                    rval=await motor_task
                    # if rval == True:
                    #     mq_publisher.publish("event","calib_status",{"calib_status":rval})
                    shared_state.current_yaw_deg.value=0.0
                    shared_state.current_pitch_deg.value=0.0
                    print('Calibration Done')
            if shared_state.change_top_path.value == True:
                shared_state.change_top_path.value = False
                model = Path(shared_state.ns.top_model)
                top_model = YOLO(model,verbose=False).to("cuda")

            elif(shared_state.stop_motor_flag.value): # TBT
                if shared_state.motors_ready.value == True:
                    await shared_state.motor_controller.stop_motors()
                    # stop_motor_task = asyncio.create_task( shared_state.motor_controller.stop_motors())
                    # await stop_motor_task
                    print(f'Stop Called')
                    shared_state.stop_motor_flag.value=False
                    shared_state.target_id.value=-1
            elif(shared_state.pitch_set.value):
                shared_state.pitch_set.value=False
                # print(f'Pitch Received')
                YawCurrentPos = 420
                PitchCurrentPos = 420
                if shared_state.motors_ready.value == True:
                    await shared_state.motor_controller.set_pitch()
            elif(shared_state.yaw_set.value):
                shared_state.yaw_set.value=False
                # print(f'Yaw Received')
                YawCurrentPos = 420
                PitchCurrentPos = 420
                if shared_state.motors_ready.value == True:
                    await shared_state.motor_controller.set_yaw()

            if(frame_top is None):
                continue
            fps_counter.update()
            t1=time.time()
            p_time =time.time()
            if cur_mode != last_mode:
                # reset per-mode one-shot flags
                held_for_mode_change = False
                # semi_no_target_held = False

                # IMPORTANT: clear state that should not leak across modes
                with shared_state.lock:
                    shared_state.PID.value = False
                    shared_state.static_position_sent.value = False
                    shared_state.hold_start_time.value = 0.0
                    shared_state.solenoid_triggered.value = False
                    shared_state.target_id.value = -1  # optional; keeps transitions clean

                last_mode = cur_mode

            # ---------------- HOLD ONCE AFTER MODE CHANGE ----------------
            if (not held_for_mode_change) and shared_state.motors_ready.value:
                await shared_state.motor_controller.hold_motors()
                held_for_mode_change = True
                # print("hello")
            if shared_state.mode.value == OpMode.SEMI_AUTO and shared_state.target_id.value !=-1 and shared_state.detected_objects:
                
                target = next((t for t in shared_state.detected_objects if t['id'] == shared_state.target_id.value), None)
                if target:
                    if not shared_state.static_position_sent.value:
                        # print(f"static [TRACKING] ID: {target['id']} fx: {target['fx']}, fy: {target['fy']} pid {shared_state.PID.value}")
                        # Send to motor here for static position and get the feedback to enter PID mode
                        static_barrel_delta_h = H_HALF  + shared_state.barrel_offset_y.value
                        static_barrel_delta_w = W_HALF + shared_state.barrel_offset_x.value

                        cx_point = (target['fx'] - (target['w'] / 2)) + shared_state.barrel_offset_x_percentage.value * target['w']
                        cy_point = (target['fy'] - (target['h'] / 2)) + shared_state.barrel_offset_y_percentage.value * target['h']
                        
                        lock_delta_x = int(target['w'] / 2) * shared_state.pid_factor.value
                        lock_delta_y = int(target['h'] / 2) * shared_state.pid_factor.value
                        print(f"[static fov] -> {static_fov}")
                        offset_x = (cx_point - static_barrel_delta_w) 
                        offset_y = (cy_point - static_barrel_delta_h)
                        if shared_state.motors_ready.value == True:
                            YawPositionCmd,PitchPositionCmd = shared_state.motor_controller.pos_cmd(W_HALF,H_HALF,*static_fov,offset_x,offset_y)

                            if round(YawPositionCmd,1) != round(YawCurrentPos,1) or round(PitchPositionCmd,1) != round(PitchCurrentPos,1) and shared_state.PID.value == False:
                                motor_task = asyncio.create_task(run_static_control(shared_state.motor_controller,YawCurrentPos, PitchCurrentPos, YawPositionCmd, PitchPositionCmd, target['h'], shared_state.zoom_level.value))
                                # Await the result of the motor control task
                                YawCurrentPos,PitchCurrentPos  = await motor_task
                            if round(YawPositionCmd,1) == round(YawCurrentPos,1) and round(PitchPositionCmd,1) == round(PitchCurrentPos,1) and shared_state.PID.value == False:
                                with shared_state.lock:
                                    shared_state.PID.value = True
                                    shared_state.static_position_sent.value = True
                                p_time = time.time()                
                        # else:
                        #     shared_state.motor_controller.send_alert()
                if shared_state.PID.value and shared_state.enable_pid.value:
                    
                    crop, offset = shared_state.crop_center(frame_top, int(lock_delta_x), int(lock_delta_y), shared_state.barrel_offset_x.value,shared_state.barrel_offset_y.value)
                    # pid_model = face_pid_model if shared_state.face_model.value else person_pid_model
                    # with torch.cuda.stream(pid_stream):
                    with torch.inference_mode(), torch.amp.autocast("cuda", dtype=torch.float16):
                        pid_result = top_model.track(
                            source=crop,
                            imgsz=320,
                            persist=True,
                            iou=0.8,
                            max_det=1,
                            classes=[0, 80, 81],
                            conf=0.35,
                            verbose=False,
                            device="cuda",
                            tracker=pid_tracker,
                        )
                    
                    for top_result in pid_result:
                        if hasattr(top_result.boxes, 'id') and top_result.boxes.id is not None:
                            box = top_result.boxes.xywh[0].cpu().numpy()
                            current_h = int(box[3])
                            cx_crop, cy_crop = box[0], box[1]
                            cx_full, cy_full = shared_state.map_crop_coords_to_full(cx_crop, cy_crop, offset)
                            # print(f"[PID] x and y : {cx_full, cy_full}")

                            top_barrel_delta_w = W_HALF + shared_state.barrel_offset_x.value
                            top_barrel_delta_h = H_HALF + shared_state.barrel_offset_y.value
                            
                            # Draw bounding box with different color (purple)
                            x1, y1 = int(cx_full - box[2]/2), int(cy_full - box[3]/2)
                            x2, y2 = int(cx_full + box[2]/2), int(cy_full + box[3]/2)

                            zero_in_region_x = x1 + shared_state.barrel_offset_x_percentage.value * (x2 - x1)
                            zero_in_region_y = y1 + shared_state.barrel_offset_y_percentage.value * (y2 - y1)

                            hit_accuracy_x = abs(zero_in_region_x - top_barrel_delta_w)
                            hit_accuracy_y = abs(zero_in_region_y - top_barrel_delta_h)


                            shared_state.acc_x_percentage.value = max(0, 100 * (1 - hit_accuracy_x / W_HALF))
                            shared_state.acc_y_percentage.value = max(0, 100 * (1 - hit_accuracy_y / H_HALF))
                            
                            # Display hit accuracy on top of the bounding box
                            accuracy_text = f"X: {shared_state.acc_x_percentage.value:.1f}% Y: {shared_state.acc_y_percentage.value:.1f}%"
                            try:
                                payload = {"pos_err_x": shared_state.acc_x_percentage.value, "pos_err_y": shared_state.acc_y_percentage.value}
                                mq_publisher.publish("accuracy","accuracy",payload)
                            except Exception as e:
                                print(f'Publishing error as {e}')

                            # purple rectange on screen with percentage
                            cv2.rectangle(frame_top, (x1, y1), (x2, y2), (255, 0, 255), 2)  # Purple color
                            text_x = x1 + (x2 - x1 - 5) // 2  # Center the text horizontally
                            cv2.putText(frame_top, accuracy_text, (text_x, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                            if not shared_state.top_target_locked.value and shared_state.static_position_sent.value == True and shared_state.motors_ready.value == True:
                                error_x = zero_in_region_x - top_barrel_delta_w
                                error_y = zero_in_region_y - top_barrel_delta_h

                                # print(f"[PID error x and y] -> {error_x} and {error_y}")

                                MCmdx,MCmdy = shared_state.motor_controller.pos_cmd(W_HALF,H_HALF,*pid_fov,error_x,error_y)

                                print(f"[PID fov] -> {pid_fov}")
                                c_time = time.time()
                                delta_t = c_time - p_time
                                # print(f'before delta_t={delta_t}')
                                if(delta_t>0.5):
                                    delta_t=0.05
                                # print(f'after delta_t={delta_t}')
                                pid_task = asyncio.create_task(run_pid_control(shared_state.motor_controller,YawCurrentPos,PitchCurrentPos, MCmdx, MCmdy,delta_t, shared_state.pres[2], shared_state.pres[3], shared_state.zoom_level.value,current_h))
                                # Await the result of the pid control task
                                retval = await pid_task
                                if(retval):
                                    with shared_state.lock:
                                        vals = list(retval)
                                        for i in range(min(len(vals), 5)):
                                            shared_state.pres[i] = vals[i]

            elif shared_state.mode.value == OpMode.AUTO:
                # print(f'Current Mode=Auto')
                if shared_state.detected_objects:
                    # If no target is selected or current target is not in frame, select next target
                    if shared_state.target_id.value == -1 or not any(obj["id"] == shared_state.target_id.value for obj in shared_state.detected_objects):
                        if shared_state.target_iterator.value >= len(shared_state.detected_objects):
                            shared_state.target_iterator.value = 0
                        shared_state.target_id.value = shared_state.detected_objects[shared_state.target_iterator.value]["id"]
                        shared_state.static_position_sent.value = False
                        shared_state.PID.value = False
                        shared_state.pres = [0, 0, 0, 0, 0]
                        # print(f"[AUTO] Selected target ID: {shared_state.target_id.value} ")

                    # Get current target
                    target_face = next((obj for obj in shared_state.detected_objects if obj["id"] == shared_state.target_id.value), None)
                    if target_face:
                        if not shared_state.static_position_sent.value:
                            # print(f"[AUTO] Tracking ID: {target_face['id']} fx: {target_face['fx']}, fy: {target_face['fy']}")
                            static_barrel_delta_h = H_HALF  + shared_state.barrel_offset_y.value
                            static_barrel_delta_w = W_HALF + shared_state.barrel_offset_x.value

                            cx_point = (target_face['fx'] - (target_face['w'] / 2)) + shared_state.barrel_offset_x_percentage.value * target_face['w']
                            cy_point = (target_face['fy'] - (target_face['h'] / 2)) + shared_state.barrel_offset_y_percentage.value * target_face['h']
                            
                            lock_delta_x = int(target_face['w'] / 2) * shared_state.pid_factor.value
                            lock_delta_y = int(target_face['h'] / 2) * shared_state.pid_factor.value

                            offset_x = (cx_point - static_barrel_delta_w) 
                            offset_y = (cy_point - static_barrel_delta_h)
                            
                            if shared_state.motors_ready.value == True:
                                YawPositionCmd,PitchPositionCmd = shared_state.motor_controller.pos_cmd(W_HALF,H_HALF,*static_fov,offset_x,offset_y)
                                if round(YawPositionCmd,1) != round(YawCurrentPos,1) or round(PitchPositionCmd,1) != round(PitchCurrentPos,1) and shared_state.PID.value == False:
                                    motor_task = asyncio.create_task(run_static_control(shared_state.motor_controller,YawCurrentPos, PitchCurrentPos, YawPositionCmd, PitchPositionCmd, target_face['h'], shared_state.zoom_level.value))
                                    # Await the result of the motor control task
                                    YawCurrentPos,PitchCurrentPos  = await motor_task
                                if round(YawPositionCmd,1) == round(YawCurrentPos,1) and round(PitchPositionCmd,1) == round(PitchCurrentPos,1) and shared_state.PID.value == False:
                                    with shared_state.lock:
                                        shared_state.PID.value = True
                                        shared_state.static_position_sent.value = True
                                    p_time = time.time()
                                    firing = True
                                    zeroing = True
                            # else:
                            #     shared_state.motor_controller.send_alert()

                    if shared_state.PID.value and shared_state.enable_pid.value and shared_state.motors_ready.value == True:
                        # Use consistent rectangular crop dimensions
                        
                        crop, offset = shared_state.crop_center(frame_top, int(lock_delta_x), int(lock_delta_y), shared_state.barrel_offset_x.value,shared_state.barrel_offset_y.value)
                        # pid_model = face_pid_model if shared_state.face_model.value else person_pid_model
                        # with torch.cuda.stream(pid_stream):
                        with torch.inference_mode(), torch.amp.autocast("cuda", dtype=torch.float16):
                            pid_result = top_model.track(
                                source=crop,
                                imgsz=320,
                                persist=True,
                                iou=0.8,
                                max_det=1,
                                classes=[0, 80, 81],
                                conf=0.35,
                                verbose=False,
                                device="cuda",
                                tracker=pid_tracker,
                            )
                        
                        for top_result in pid_result:
                            if hasattr(top_result.boxes, 'id') and top_result.boxes.id is not None:
                                box = top_result.boxes.xywh[0].cpu().numpy()
                                current_h = int(box[3])
                                cx_crop, cy_crop = box[0], box[1]
                                cx_full, cy_full = shared_state.map_crop_coords_to_full(cx_crop, cy_crop, offset)

                                # Calculate hit accuracy
                                top_barrel_delta_w = W_HALF + shared_state.barrel_offset_x.value
                                top_barrel_delta_h = H_HALF + shared_state.barrel_offset_y.value

                                x1, y1 = int(cx_full - box[2]/2), int(cy_full - box[3]/2)
                                x2, y2 = int(cx_full + box[2]/2), int(cy_full + box[3]/2)

                                zero_in_region_x = x1 + shared_state.barrel_offset_x_percentage.value * (x2 - x1)
                                zero_in_region_y = y1 + shared_state.barrel_offset_y_percentage.value * (y2 - y1)

                                hit_accuracy_x = abs(zero_in_region_x - top_barrel_delta_w)
                                hit_accuracy_y = abs(zero_in_region_y - top_barrel_delta_h)

                                acc_x = max(0, 100 * (1 - hit_accuracy_x / W_HALF))
                                acc_y = max(0, 100 * (1 - hit_accuracy_y / H_HALF))
                            
                                # Draw bounding box and accuracy
                                accuracy_text = f"X: {acc_x:.1f}% Y: {acc_y:.1f}%"
                                
                                try:
                                    payload = {"pos_err_x": acc_x, "pos_err_y": acc_y}
                                    mq_publisher.publish("accuracy","accuracy",payload)
                                except Exception as e:
                                    print(f'Publishing error as {e}')
                                
                                cv2.rectangle(frame_top, (x1, y1), (x2, y2), (255, 0, 255), 2)
                                text_x = x1 + (x2 - x1 - 5) // 2  # Center the text horizontally
                                cv2.putText(frame_top, accuracy_text, (text_x, y1 - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                                if not shared_state.top_target_locked.value and shared_state.static_position_sent.value == True and shared_state.motors_ready.value == True:
                                    error_x = zero_in_region_x - top_barrel_delta_w
                                    error_y = zero_in_region_y - top_barrel_delta_h
                                    # print(f'Error = {error_x} and error {error_y}')
                            
                                    MCmdx,MCmdy = shared_state.motor_controller.pos_cmd(W_HALF,H_HALF,*pid_fov,error_x,error_y)

                                    c_time = time.time()
                                    delta_t = c_time - p_time
                                    if(delta_t>0.5):
                                        delta_t=0.2
                                    if zeroing:
                                        pid_task = asyncio.create_task(run_pid_control(shared_state.motor_controller,YawCurrentPos,PitchCurrentPos, MCmdx, MCmdy,delta_t, shared_state.pres[2], shared_state.pres[3], shared_state.zoom_level.value,current_h))
                                    # Await the result of the pid control task
                                    retval = await pid_task
                                    if(retval):
                                        with shared_state.lock:
                                            vals = list(retval)
                                            for i in range(min(len(vals), 5)):
                                                shared_state.pres[i] = vals[i]
                                    # Check if target is centered and wait before moving to next target
                                    # if acc_x > 99 and acc_y > 99:  # If accuracy is 100%
                                    #     current_time = time.time()
                                    #     if current_time - shared_state.last_target_switch_time.value > 1:  # Wait 1 second
                                    #         # mq_publisher.publish('',{"trigger_fired": True})
                                    #         shared_state.target_iterator.value += 1
                                    #         seconds = time.time() - tracking_start_time
                                    #         tracking_start_time = time.time()
                                    #         if shared_state.target_iterator.value >= len(shared_state.detected_objects):
                                    #             shared_state.target_iterator.value = 0
                                    #         shared_state.target_id.value = 0  # Reset target to trigger selection of next target
                                    #         shared_state.static_position_sent.value = False
                                    #         shared_state.PID.value = False
                                    #         shared_state.pres = [0, 0, 0, 0, 0]
                                    #         shared_state.last_target_switch_time.value = current_time
                                    #         print(f"[AUTO] Moving to next target. Iterator: {shared_state.target_iterator.value}")

                                    # -------------------------------------------------------------------------
                                    current_time = time.monotonic()
                                    if shared_state.solenoid_triggered.value == True and zeroing == False and current_time - shared_state.last_target_switch_time.value > shared_state.burst_time.value:
                                        shared_state.solenoid_triggered.value = False
                                        shared_state.hold_start_time.value = 0.0  # reset timer
                                        shared_state.target_iterator.value += 1
                                        seconds = time.time() - tracking_start_time
                                        tracking_start_time = time.time()
                                        if shared_state.target_iterator.value >= len(shared_state.detected_objects):
                                            shared_state.target_iterator.value = 0
                                        if len(top_result) == 0:
                                            shared_state.target_iterator.value = 0
                                        shared_state.target_id.value = 0
                                        shared_state.static_position_sent.value = False

                                    if hit_accuracy_x < shared_state.acc_percentage.value and hit_accuracy_y < shared_state.acc_percentage.value:  # Target centered
                                        # If first entry, initialize timer
                                        if shared_state.hold_start_time.value == 0.0:
                                            shared_state.hold_start_time.value = current_time
                                            holding_time = 0.0
                                        else:
                                            # print("cur_time =", current_time," hold_start =", shared_state.hold_start_time.value)
                                            holding_time = current_time - shared_state.hold_start_time.value

                                        # print(f"Holding time = {holding_time:.3f}")

                                        if holding_time > shared_state.auto_target_hold_time.value:  # Held for at least 1 sec
                                            if firing:
                                                zeroing = False
                                                # await shared_state.motor_controller.hold_motors()
                                                shared_state.last_target_switch_time.value = current_time
                                                firing = False
                                                threading.Thread(target=shared_state.fire_solenoid, daemon=True).start()

                                            # Move to next target after burst time
                                            if current_time - shared_state.last_target_switch_time.value > shared_state.burst_time.value:
                                                shared_state.hold_start_time.value = 0.0  # reset timer
                                                shared_state.target_iterator.value += 1
                                                seconds = time.time() - tracking_start_time
                                                tracking_start_time = time.time()
                                                if shared_state.target_iterator.value >= len(shared_state.detected_objects):
                                                    shared_state.target_iterator.value = 0
                                                shared_state.target_id.value = 0
                                                shared_state.static_position_sent.value = False
                                                shared_state.pres = [0, 0, 0, 0, 0]
                                                print(f"[AUTO] Moving to next target. Iterator: {shared_state.target_iterator.value}")
                                    # -------------------------------------------------------------
                                    # elif zeroing == False and hit_accuracy_x > 5 or hit_accuracy_y > 5:
                                    #     # print(f'Inside accuracy lost')
                                    #     zeroing = True
                                        
                                    else:
                                        # If not centered, reset the hold timer
                                        shared_state.hold_start_time.value = 0.0
                                # Visual Debug
                                # cv2.circle(frame_top, (960,540), 5, (0, 0, 255), -1)
                                # cv2.circle(frame_top, (int(cx_full), int(cy_full)), 5, (0, 0, 255), -1)
                                cv2.putText(frame_top, f"Target ID: {shared_state.target_id.value} , time : {seconds}", (int(zero_in_region_x) + 10, int(zero_in_region_y)),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                # pass
            elif shared_state.mode.value==OpMode.MANUAL or shared_state.mode.value==OpMode.THERMAL:
                # print(f'{shared_state.manual_x.value} > 0 and {shared_state.manual_y.value} > 0')
                if shared_state.manual_x.value > 0 and shared_state.manual_y.value > 0:
                    # print(f'Manual point recieved')
                    static_barrel_delta_h = H_HALF  + shared_state.barrel_offset_y.value
                    static_barrel_delta_w = W_HALF + shared_state.barrel_offset_x.value
                    # Use head position for tracking
                    offset_x = shared_state.manual_x.value - static_barrel_delta_w 
                    offset_y = shared_state.manual_y.value - static_barrel_delta_h  # This is already the head position from our earlier modification

                    YawPositionCmd,PitchPositionCmd = shared_state.motor_controller.pos_cmd(W_HALF,H_HALF,*static_fov,offset_x,offset_y)
                    # print(f'fx = {offset_x} and fy = {offset_y}')
                    # print(f'yaw_cmd = {-1 * YawPositionCmd} pitch_cmd = {-1 * PitchPositionCmd}')
                    # Publishing message to GUI
                    if shared_state.motors_ready.value == True:
                        yaw_percentage,pitch_percentage = compute_hit_accuracy_from_commanded(YawPositionCmd, YawCurrentPos, PitchPositionCmd, PitchCurrentPos)
                        payload = {"pos_err_x": 100 - yaw_percentage, "pos_err_y": 100 - pitch_percentage}
                        mq_publisher.publish("accuracy","accuracy",payload)
                        if round(YawPositionCmd,2) != round(YawCurrentPos,2) or round(PitchPositionCmd,2) != round(PitchCurrentPos,2) and shared_state.PID.value == False:
                            motor_task = asyncio.create_task(run_static_control(shared_state.motor_controller,YawCurrentPos, PitchCurrentPos, YawPositionCmd, PitchPositionCmd,None,None))
                            # Await the result of the motor control task
                            YawCurrentPos,PitchCurrentPos  = await motor_task
            elif shared_state.mode.value==OpMode.MANUAL_JS:
                # print(f'Demo mode turned on ')
                if shared_state.motors_ready.value == True:
                    motor_task = asyncio.create_task(run_velocity_control(shared_state.motor_controller,shared_state.js_twist.value, shared_state.js_pitch.value))
                    await motor_task

            elif shared_state.mode.value == OpMode.DEMO:
                # print(f'Demo mode turned on ')
                now = time.time()
                SinWave = 20*numpy.sin(now)
                PitchWave = 20*numpy.cos(now)
                YawPositionCmd = shared_state.mapping(-Globals.REVOLUTION,Globals.REVOLUTION,-Globals.YAW_REDUCTION,Globals.YAW_REDUCTION,SinWave)
                PitchPositionCmd = shared_state.mapping(-Globals.REVOLUTION,Globals.REVOLUTION,-Globals.PITCH_REDUCTION,Globals.PITCH_REDUCTION,PitchWave)
                CameraPositionCmd = shared_state.mapping(-Globals.REVOLUTION,Globals.REVOLUTION,-Globals.CAM_REDUCTION,Globals.CAM_REDUCTION,SinWave)
                if shared_state.motors_ready.value == True:
                    motor_task = asyncio.create_task(run_scan_mode(shared_state.motor_controller,YawPositionCmd, PitchPositionCmd,CameraPositionCmd))
                    await motor_task
                # else:
                #     shared_state.motor_controller.send_alert()
            elif shared_state.mode.value == OpMode.HOLD:
                if shared_state.motors_ready.value == True:
                    await shared_state.motor_controller.hold_motors()

            t2=time.time()

            # cv2.putText(frame_static, f"Target Distance: {target_position:.2f} m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            # cv2.putText(frame_static, f"Target Speed   : {target_speed:.2f} m/s", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if shared_state.show_gui:
                # Pre-compute center (avoid recompute)
                cx = frame_top.shape[1] // 2 + int(shared_state.barrel_offset_x.value)
                cy = frame_top.shape[0] // 2 + int(shared_state.barrel_offset_y.value)

                # Draw crosshair (1 ms)
                cv2.line(frame_top, (cx, 0), (cx, frame_top.shape[0]), (0,255,0), 1)
                cv2.line(frame_top, (0, cy), (frame_top.shape[1], cy), (0,255,0), 1)

                # Tiny FPS (0.6 scale, thickness 1) → 3–4 ms vs 12 ms
                fps_counter.update()
                cv2.putText(frame_top, f"FPS: {fps_counter.latest_fps:.0f}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 1)

                # NO HEAVY TEXT
                top_updated_writer.write(frame_top)
            await asyncio.sleep(0.01)
        except Exception as ex:
            logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
            
            error_str = traceback.format_exc()
            if(error_str and len(error_str)>0):
                logger.error(f'Exception Trace:{error_str}')
    await shared_state.motor_controller.stop_motors()
    
    cv2.destroyAllWindows()
    print("[PIDControlLoop] Stopped.")

loop = asyncio.new_event_loop()

def start_event_loop():
    asyncio.set_event_loop(loop)
    loop.run_forever()

# Create the background thread to run the event loop
threading.Thread(target=start_event_loop, daemon=True).start()

def sensor_logging_process(shared_state, filename="sensor_data.csv"):
    """
    Process function to read and log sensor data to a CSV file at high frequency.
    """
    reader = SensorReaderGPS()
    logger = SensorLogger()  
    mq_publisher = BaseMQ_New(queue=f"{SYS}_sensor_data.pub.q", connection_name=f"{SYS}_sensor_data-pub")  # queue unused for pub but harmless
    # mq_publisher = PublishMQ()
    
    if not reader.connect():
        print("[ERROR] Failed to initialize sensor reader. Logging aborted.")
        return

    print("[INFO] Starting sensor data logging...")

    packet_count = 0
    last_freq_time = time.time()

    try:
        while not shared_state.stop_flag.value:
            data = reader.read_data()
            if data is not None and data is not False:
                packet_count += 1
                # if shared_state.calibrate_encoders.value == True:
                #     shared_state.calibrate_encoders.value = False
                #     reader.calibrate()

                # Publish to MQTT (optional, remove if not needed)
                data["degrees0"] = shared_state.pitch_enc.value
                data["degrees1"] = shared_state.yaw_enc.value
                logger.log(data)
                if shared_state.fire_status.value:
                    mq_publisher.publish('sensor_logs',"sensor_data",data)
                    
                # Calculate and print frequency every second
                current_time = time.time()
                if current_time - last_freq_time >= 1.0:
                    # mq_publisher.publish('', {"one_sec_data": data})
                    mq_publisher.publish('sensor_logs',"one_sec_data",data)
                    # print(f'Data = {data}')
                    # print(f"[INFO] Sensor Frequency: {packet_count} packets/sec") 
                    packet_count = 0
                    last_freq_time = current_time
            elif data == False:
                break
    except KeyboardInterrupt:
        payload = {"keyboard_interrupt":"ctrl_c","message":"ctrl + c pressed"}
        mq_publisher.publish('error','error',payload)
        shared_state.stop_flag.value = 1
    except Exception as e:
        print(f"[ERROR] Sensor logging failed: {e}")

        
    finally:
        # print(f'Sensor data closed {reader.ser}')
        if reader.ser:
            print(f'from finally')
            reader.ser.close()
        

def launch():
    w=1920
    h=1080
    shared_static = SharedFrame(shape=(h,w, 3))
    shared_top = SharedFrame(shape=(h,w, 3))
    shared_top_updated = SharedFrame(shape=(h,w, 3)) # Creating New Memory Frame
    shared_state = SharedState(SYS,logger,MQ_PUB)
   
    cam_proc = Process(target=camera_reader, args=(shared_static.name, shared_top.name, shared_state, logger))
    det_proc = Process(target=yolo_inference_loop, args=(shared_static.name, shared_top.name, shared_state, logger))
    http_proc = Process(target=run_https_server, args=(shared_static.name, shared_top_updated.name, shared_state))
    msg_controller=MessageController(shared_state, shared_top, shared_static,SYS,logger)
    msg_thread=threading.Thread(target=msg_controller.run,daemon=True)
    # Start sensor logging process
    sensor_proc = Process(target=sensor_logging_process, args=(shared_state,), daemon=True)
    recorder_proc = Process(target=video_recorder_process,args=(shared_top.name, shared_static.name, shared_state, logger),daemon=True)
    
    top_cam_serial  = shared_state.camera_dict[Globals.TOP_SERIAL]['serial']
    static_cam_serial = shared_state.camera_dict[Globals.STATIC_SERIAL]['serial']
    
    stats_proc = Process(target=telemetry_process,args=("top", top_cam_serial, "static", static_cam_serial, 1.0, 1, 1),daemon=True)


    cam_proc.start()
    det_proc.start()
    http_proc.start()
    msg_thread.start()
    sensor_proc.start()  # Start sensor logging
    recorder_proc.start()
    stats_proc.start()

    asyncio.run_coroutine_threadsafe(pid_control_loop(shared_static.name, shared_top.name, shared_top_updated.name,shared_state), loop)
    asyncio.run_coroutine_threadsafe(shared_state.motor_controller.publish_health(), loop)
    # asyncio.run(pid_control_loop(shared_static.name, shared_top.name, shared_top_updated.name,shared_state))

    cam_proc.join()
    det_proc.join()
    http_proc.join()
    sensor_proc.join()
    recorder_proc.join()
    stats_proc.join()

    shared_static.close()
    shared_static.unlink()
    shared_top.close()
    shared_top.unlink()
    shared_top_updated.close()
    shared_top_updated.unlink()
    
    sys.exit(0)

if __name__=='__main__':
    launch()
