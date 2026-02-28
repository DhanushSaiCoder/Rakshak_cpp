from math import sin, cos, asin, atan2, radians, degrees
import Globals
from op_mode import OpMode
import cv2
from share_frame import SharedFrame
import time
import os
from MQ_Base import BaseMQ_New
import re
import inspect
import traceback
import numpy as np
from datetime import datetime
from ultralytics import YOLO
import torch
from scipy.interpolate import interp1d
from pathlib import Path

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


def camera_reader(shm_static, shm_top, shared_state,logger):

    # top_cam_reader = shared_state.camera_dict[Globals.TOP_SERIAL]
    # static_cam_reader = shared_state.camera_dict[Globals.STATIC_SERIAL]
    # # --------------------------------------------------------------------
    # left_cam_reader = shared_state.camera_dict[Globals.LEFT_SERIAL]
    # right_cam_reader = shared_state.camera_dict[Globals.RIGHT_SERIAL]

    # Define all camera pairs in desired order (first = top+static)
    CAM_PAIRS = [
        (shared_state.camera_dict[Globals.TOP_SERIAL],
        shared_state.camera_dict[Globals.STATIC_SERIAL]),
    ]
    current_idx = 0  # start at first pair (top+static)

    # Open initial pair
    top_cam_reader, static_cam_reader = CAM_PAIRS[current_idx]
    logger.info(f'top -> {top_cam_reader}<<------------->>static -> {static_cam_reader}')
    cam0 = cv2.VideoCapture(static_cam_reader["video"])
    cam2 = cv2.VideoCapture(top_cam_reader["video"])
    # w_set=1280
    # h_set=720
    for cam in (cam0, cam2):
        cam.set(cv2.CAP_PROP_BUFFERSIZE, 10)
        # cam.set(cv2.CAP_PROP_FRAME_WIDTH, w_set)
        # cam.set(cv2.CAP_PROP_FRAME_HEIGHT, h_set)
        # cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
 
    static_writer = SharedFrame(name=shm_static)
    top_writer = SharedFrame(name=shm_top)
    fps_counter=FPSCounter()
    # with shared_state.lock:

    # Static Cam ------------------------------------------------------------------------
    ret, frame = cam0.read()
    ret2, f2 = cam2.read()
    if(not ret):
        print(f'Unable To Read Camera Frames')
        payload = {"Camera_type":"Top_camera"}
        MQ_PUB.publish('error','Camera_error',payload)
        shared_state.stop_flag.value = 1
        return
    if(ret and ret2):
        payload = {"Camera_type":"Both","message":"camera_connected"}
        MQ_PUB.publish('error','error',payload)
    # Static cam
    h, w = frame.shape[:2]
    while not shared_state.stop_flag.value:
        try:
             #  Decide which pair index should be active based on mode
            if shared_state.mode.value == OpMode.SCAN:
                # new_idx = 0    # SCAN mode → use second pair
                pass
            elif shared_state.mode.value == OpMode.THERMAL:
                # new_idx = 2
                # print("Thermal")
                pass
            else:
                new_idx = 0   # DEMO/SEMI/AUTO → use first pair


            if new_idx != current_idx:
                # Release current cameras
                if cam0: cam0.release()
                if cam2: cam2.release()

                # update readers + reopen
                left_cam_reader, right_cam_reader = CAM_PAIRS[new_idx]
                cam0 = cv2.VideoCapture(right_cam_reader["video"])
                cam2 = cv2.VideoCapture(left_cam_reader["video"])
                current_idx = new_idx


            ret0, f0 = cam0.read()
            if not ret0:
                print("Failed to grab frame from cam0")
                payload = {"Camera_type":"Top_camera","message":"Top_camera_disconnected"}
                MQ_PUB.publish('error','Cameras_error',payload)
                shared_state.stop_flag.value = 1
            else:
                h, w = f0.shape[:2]
                if (w, h) != (1920, 1080):   # check resolution
                    f0 = cv2.resize(f0, (1920, 1080), interpolation=cv2.INTER_LINEAR)
            ret2, f2 = cam2.read()
            if not ret2:
                print("Failed to grab frame from cam1")
                payload = {"Camera_type":"Bottom_camera","message":"Bottom_camera_disconnected"}
                MQ_PUB.publish('error','Cameras_error',payload)
                shared_state.stop_flag.value = 1
            else:
                h, w = f2.shape[:2]
                if (w, h) != (1920, 1080):   # check resolution
                    f2 = cv2.resize(f2, (1920, 1080), interpolation=cv2.INTER_LINEAR)
            
            # Gamma neutral -----------------
            gamma=1.1
            inv_gamma = 1.0 / gamma
            table = np.linspace(0, 255, 256, dtype=np.uint8) ** inv_gamma  # Precompute
            # f0 =  cv2.LUT(f0, table.astype(np.uint8))
            # f2 =  cv2.LUT(f2, table.astype(np.uint8))
            # --------------------------------

            fps_counter.update()
            if shared_state.show_gui:
                cv2.putText(f0, f"FPS={fps_counter.latest_fps}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            #     cv2.imshow("Static Camera", f0)
            #     cv2.imshow("PID Camera", f2)
            #     if cv2.waitKey(1) & 0xFF == ord('q'):
            #         shared_state.stop_flag.value = 1
            #         break
            if ret0: static_writer.write(f0)
            if ret2: top_writer.write(f2)
            time.sleep(0.001)
        except KeyboardInterrupt:
            payload = {"keyboard_interrupt":"ctrl_c","message":"ctrl + c pressed"}
            MQ_PUB.publish('error','Keyboard_interrupt',payload)
            shared_state.stop_flag.value = 1
        except Exception as ex:
            logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
    
    cam0.release()
    cam2.release()
    static_writer.close()
    top_writer.close()

def video_recorder_process(shared_top_name: str, shared_static_name: str, shared_state, logger) -> None:
    """
    Records BOTH top and static cameras.
    Uses EXACT timestamp & directory from your original code.
    """
    shared_top = SharedFrame(shared_top_name)
    shared_static = SharedFrame(shared_static_name)

    out_top = None
    out_static = None
    recording = False
    fps = 11.0
    frame = shared_static.read()
    size = (frame.shape[1], frame.shape[0])

    # === ENSURE SAVE DIR EXISTS ===
    save_dir = os.path.join(shared_state.save_dir, 'videos')  # .value is bytes
    os.makedirs(save_dir, exist_ok=True)
    logger.info(f'recordings dir --> {save_dir}')
    while not shared_state.stop_flag.value:
        try:
            # === START RECORDING ===
            if shared_state.record_start_flag.value and not recording:
                timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')


                # File paths
                top_path = os.path.join(save_dir, f"rec_top_{timestamp}.mp4")
                static_path = os.path.join(save_dir, f"rec_static_{timestamp}.mp4")

                # Create writers
                out_top = cv2.VideoWriter(top_path, fourcc, fps, size)
                out_static = cv2.VideoWriter(static_path, fourcc, fps, size)

                recording = True
                shared_state.record_start_flag.value = False
                logger.info(f"[VIDEO RECORDER] Started: {timestamp}")
                logger.info(f"   Top   → {top_path}")
                logger.info(f"   Static→ {static_path}")

            # === STOP RECORDING ===
            elif shared_state.record_stop_flag.value and recording:
                for writer in [out_top, out_static]:
                    if writer:
                        writer.release()
                out_top = out_static = None
                recording = False
                shared_state.record_stop_flag.value = False
                logger.info("[VIDEO RECORDER] Recording stopped.")

            # === WRITE FRAMES ===
            if recording:
                frame_top = shared_top.read()
                frame_static = shared_static.read()

                if frame_top is not None and out_top:
                    out_top.write(frame_top)
                if frame_static is not None and out_static:
                    out_static.write(frame_static)

            time.sleep(0.001)
        except Exception as e:
            logger.error(f"[VIDEO RECORDER] Error: {e}")
            time.sleep(0.01)

    # Final cleanup
    for writer in [out_top, out_static]:
        if writer:
            writer.release()
    logger.info("[VIDEO RECORDER] Process terminated.")

def take_snapshot(shared_state, shared_top, shared_static, logger):
    """
    Takes snapshot of both cameras and saves as JPG.
    Non-blocking, called from RabbitMQ handler.
    """
    save_dir = os.path.join(shared_state.save_dir, 'images')  # .value is bytes
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    os.makedirs(save_dir, exist_ok=True)

    frame_top = shared_top.read()
    frame_static = shared_static.read()

    if frame_top is not None:
        path_top = os.path.join(save_dir, f"snap_top_{timestamp}.jpg")
        success = cv2.imwrite(path_top, frame_top)
        if success:
            logger.info(f"[SNAPSHOT] Top saved: {path_top}")
        else:
            logger.error(f"[SNAPSHOT] Failed to save top: {path_top}")

    if frame_static is not None:
        path_static = os.path.join(save_dir, f"snap_static_{timestamp}.jpg")
        success = cv2.imwrite(path_static, frame_static)
        if success:
            logger.info(f"[SNAPSHOT] Static saved: {path_static}")
        else:
            logger.error(f"[SNAPSHOT] Failed to save static: {path_static}")

def yolo_inference_loop(shm_static, shm_top, shared_state,logger):
    mq_publisher=BaseMQ_New(queue=f"{SYS}_infrence.pub.q", connection_name=f"{SYS}_infrence-pub")
    reader = SharedFrame(name=shm_static)
    model = Path(shared_state.ns.static_model)
    static_model = YOLO(model,verbose=False).to("cuda")
    # person_model = YOLO(Globals.PERSON_MODEL,verbose=False).to("cuda")
    fps_counter=FPSCounter()
    # cv2.namedWindow("Static Camera")
    # cv2.setMouseCallback("Static Camera", shared_state.select_face)
    want_detections = True
    gpu_in = cv2.cuda_GpuMat()
    # with shared_state.lock: torch.cuda.amp.autocast(dtype=torch.float16)
    while not shared_state.stop_flag.value:
        try:
            if shared_state.change_static_path.value == True:
                shared_state.change_static_path.value = False
                model = Path(shared_state.ns.static_model)
                static_model = YOLO(model,verbose=False).to("cuda")
                
            frame_static = reader.read()
            # resized = cv2.resize(frame_static, (640, 360))
            gpu_in.upload(frame_static)
            gpu_resized = cv2.cuda.resize(gpu_in, (640, 360))
            resized = gpu_resized.download()  # YOLO needs CPU numpy
            # if shared_state.face_model.value:
            #     with torch.inference_mode():
            #         results = face_model.track(source=resized, persist=True, device="cuda",classes=[0,80,81], conf=0.35, iou=0.5,max_det=10, tracker="trackers/bytetrack.yaml")
            # else:
            with torch.inference_mode():
                results = static_model.track(source=resized, persist=True, device="cuda",classes=[0,80,81], conf=0.35, iou=0.5,max_det=10, tracker="trackers/bytetrack.yaml")
                
            
            # shared_state.latest_detections = results
            fps_counter.update()
            # Scale boxes back to original size
            scale_x = frame_static.shape[1] / 640
            scale_y = frame_static.shape[0] / 360

            # if shared_state.show_gui:
                # for r in results:
                #     if hasattr(r, 'boxes'):
                #         for box in r.boxes.xyxy.cpu().numpy():
                #             x1, y1, x2, y2 = map(int, box)
                #             cv2.rectangle(frame_static, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # cv2.putText(frame_static, f"-", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            detected_objects = []
            if want_detections:
                for result in results:
                    if hasattr(result.boxes, 'id') and result.boxes.id is not None:
                        track_ids = result.boxes.id.int().cpu().tolist()
                        boxes = result.boxes.xywh.cpu().tolist()
                        class_ids = result.boxes.cls.int().cpu().tolist()
                        # get class nanes dict
                        # class_names = result.names 
                        
                        for tid, box, cls_id in zip(track_ids, boxes, class_ids):
                            # Calculate head position (assuming head is in upper 1/3 of person bounding box)
                            head_y = box[1] - (box[3] * 0.25)  # Move up by 25% of height
                            
                            # Distance = (Real height * Focal length) / Pixel height
                            # distance = (Globals.KNOWN_HEIGHT * shared_state.focal_length.value) / (box[3] * scale_y)  # in meters
                            # print(f'Focal_length in distance calculation is ----> {shared_state.focal_length.value}')
                            depth = get_depth_estimate(shared_state.zoom_level.value, (box[3] * scale_y),shared_state)
                            depth = shared_state.depth_factor.value * depth
                            bearing_offset = (shared_state.source_heading.value - 10) % 360
                            lat, lon = calculate_target_gps(shared_state.source_lat.value,shared_state.source_lon.value, depth, bearing_offset)
                            lat, lon = calculate_target_gps(16.6864,82.9875, depth, bearing_offset)
                            # print(f'Depth = {depth} , val = {shared_state.depth_factor.value}')
                            # print(f"[DEBUG] Using zoom={shared_state.zoom_level.value}, height={(box[3] * scale_y)} -> Depth={depth}")
                            detected_objects.append({
                                "id": tid,
                                "cls": cls_id,
                                "fx": box[0]*scale_x,  # Center x position
                                "fy": box[1]*scale_y,  # Center y position
                                "w": box[2]*scale_x,
                                "h": box[3]*scale_y,
                                "hh": head_y,  # Head y position stored for future use
                                "distance":depth,
                                "selected_id": shared_state.target_id.value,
                                "latitude":lat,
                                "longitude":lon,
                            })
                    # Sort detected objects by x-coordinate (left to right)
                    detected_objects.sort(key=lambda x: x["fx"])

                    # === Draw Boxes ===
                    # shared_state.detected_objects.clear()
                    for obj in detected_objects:
                        x, y, w, h = obj["fx"], obj["fy"], obj["w"], obj["h"]
                        x1, y1 = int(x - w / 2), int(y - h / 2)
                        x2, y2 = int(x + w / 2), int(y + h / 2)
                        color = (0, 0, 255) if obj["id"] == shared_state.target_id.value else (0, 255, 0)
                        cv2.rectangle(frame_static, (x1, y1), (x2, y2), color, 2)
                        # cv2.putText(frame_static, f"ID: {obj['id']}", (x1, y1 - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        cv2.putText(frame_static, f"ID: {obj['id']} | cls = {obj['cls']}", (x1, y1 - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    det_dict={"detected_objects":detected_objects}
                    mq_publisher.publish("bbox_data","detections",det_dict)
                    with shared_state.lock:
                        shared_state.detected_objects[:] = detected_objects[:]  # Replace old data
                        # for obj in detected_objects:
                        #     x, y, w, h = obj["fx"], obj["fy"], obj["w"], obj["h"]
                        #     x1, y1 = int(x - w / 2), int(y - h / 2)
                        #     x2, y2 = int(x + w / 2), int(y + h / 2)
                        #     color = (0, 0, 255) if obj["id"] == shared_state.target_id else (0, 255, 0)
                        #     cv2.rectangle(frame_static, (x1, y1), (x2, y2), color, 2)
                        #     cv2.putText(frame_static, f"ID: {obj['id']}", (x1, y1 - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                            # shared_state.detected_objects.append(obj)
                            
                    # shared_state.detected_objects.=detected_objects
            # cv2.imshow("Static Camera", frame_static)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     shared_state.stop_flag.value = 1
            #     break
            time.sleep(0.001)
        except KeyboardInterrupt:
            payload = {"keyboard_interrupt":"ctrl_c","message":"ctrl + c pressed"}
            MQ_PUB.publish('error','error',payload)
            shared_state.stop_flag.value = 1
        except Exception as ex:
            logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
            error_str = traceback.format_exc()
            if(error_str and len(error_str)>0):
                logger.error(f'Exception Trace:{error_str}')
    cv2.destroyAllWindows()


def calculate_target_gps(lat1, lon1, distance, bearing):
    if lat1 == 0.0 and lon1 == 0.0:
        return 0.0,0.0
    R = 6371000  # Earth's radius in meters
    lat1 = radians(lat1)  # Convert to radians
    lon1 = radians(lon1)
    bearing = radians(bearing)
    
    # Calculate target latitude
    lat2 = asin(sin(lat1) * cos(distance / R) + 
                cos(lat1) * sin(distance / R) * cos(bearing))
    
    # Calculate target longitude
    lon2 = lon1 + atan2(sin(bearing) * sin(distance / R) * cos(lat1),
                         cos(distance / R) - sin(lat1) * sin(lat2))
    
    # Convert back to degrees
    lat2 = degrees(lat2)
    lon2 = degrees(lon2)
    
    return lat2, lon2


def get_depth_estimate(zoom_level, detected_height, shared_state):
    dataset = shared_state.datasets[shared_state.target_type]
    zoom_data = dataset.get(str(zoom_level), {})
    height_list = zoom_data.get("h", [])
    # print(f'Height list = {height_list}')
    depth_list = zoom_data.get("depth", [])
    if not (height_list and depth_list):
        print(f"[WARN] No data for zoom level {zoom_level} in dataset {shared_state.target_type}")
        return 0.0
    try:
        sorted_pairs = sorted(zip(height_list, depth_list), key=lambda x: x[0], reverse=True)
        sorted_heights, sorted_depths = zip(*sorted_pairs)
        # Interpolate
        interp_func = interp1d(sorted_heights, sorted_depths, kind='linear', fill_value="extrapolate")
        depth = float(interp_func(detected_height))
        
        # Clamp negative or extreme results
        if depth < 0 or depth > 1000:  # Adjust max threshold if needed
            # print(f"[WARN] Invalid interpolated depth {depth:.2f} for height {detected_height}")
            return 0.0
        return depth
    except Exception as e:
        print(f"[ERROR] Depth estimation failed: {e}")
        return 0.0