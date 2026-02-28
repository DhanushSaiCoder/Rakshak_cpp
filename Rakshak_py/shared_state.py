import multiprocessing as mp
from multiprocessing import Value
import ctypes
ctypes.CDLL('libX11.so').XInitThreads()  # Fix XCB crash in multithreaded OpenCV
from ctypes import c_int,Array,c_bool,c_double,c_float
from motor_controller import MotorController,GearReductionDeg
from data_manager import DataManager
import Globals
import time
from utils import getCamPorts
from zoom_controller import ZoomController
from solenoid_controller import TriggerController
import numpy as np
from scipy.interpolate import interp1d
import json
from op_mode import OpMode
from pathlib import Path

# imports for state management
import os
import tempfile
from dataclasses import asdict, dataclass
from typing import Any, Dict

# Changes for state management a dataclass and and state store class which does save 
@dataclass
class RuntimeState:
    yaw_value: float = 1.0 # current yaw degree
    pitch_value: float = 1.0 # current pitch degree
    short_burst_value: float = 0.5 # solenoid hold for short time
    long_burst_value: float = 1.0 # solenoid hold for long burst time
    pid_factor_value: float = 3.0 # top camera zero in range based on bbox * factor
    depth_factor_value: float = 1.0 # depth calulation factor -> bboxh * factor
    auto_accuracy_value: int = 5 # autonomous when to go after next stage hold -> value
    auto_target_hold_time: float = 0.2 # how much time to hold befor trigger
    image_stabilization: bool = False # image stabilization default state
    auto_focus: bool = True # auto focus default state
    pid_enable: bool = True # pid flag which enables top camera zeroin we can disable to stop as well
    zoom_level: int = 1 # zoom level state
    digital_zoom_enabled: bool = False # digital zoom flag enabled or disabled
    top_model: str = Globals.DEFAULT_MODEL # top camera model path 
    static_model: str = Globals.DEFAULT_MODEL # static camera model path 
    op_mode: int = 1  # OpMode.SEMI_AUTO by default

class StateStore:
    def __init__(self, path: str, logger=None):
        self.path = path
        self.logger = logger

    def load(self) -> RuntimeState:
        try:
            if not os.path.exists(self.path):
                self.logger.info(f'[StateStore] No state found so returning run time data one')
                return RuntimeState()
            with open(self.path, "r") as f:
                self.logger.info(f'[StateStore] -> self.path->{self.path}')
                raw = json.load(f)
            return RuntimeState(**raw)
        except Exception as e:
            if self.logger:
                self.logger.error(f"[StateStore] load failed: {e} (ignoring, using defaults)")
            return RuntimeState()

    def save(self, state: RuntimeState) -> None:
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp_fd, tmp_path = tempfile.mkstemp(prefix=".state_", suffix=".json", dir=os.path.dirname(self.path))
        try:
            with os.fdopen(tmp_fd, "w") as f:
                json.dump(asdict(state), f, indent=2)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_path, self.path)  # atomic replace
        except Exception as e:
            try:
                os.unlink(tmp_path)
            except Exception:
                pass
            if self.logger:
                self.logger.error(f"[StateStore] save failed: {e}")



# =========== Shared state ===========================
class SharedState:
    def __init__(self,SYS,logger,mq_pub):
        self.manager = mp.Manager()
        self.lock = self.manager.Lock()
        self.stop_flag = Value(c_int, 0)
        self.sys = SYS
        self.logger = logger
        self.mq_pub = mq_pub

        self.show_gui = True

        #================ State regarding setup ======================= 
        # Pick a stable location (inside your existing logs dir is fine)
        self.state_dir = "states"
        self.state_path = os.path.join(self.state_dir, "runtime_state.json")
        self.state_store = StateStore(self.state_path, logger=self.logger)
        loaded = self.state_store.load()
        # print(f'loaded -> {loaded}')

        # ==================================================================
        # # Dataset loading for depth the json file
        self.dataset_manager = DataManager()
        self.dataset_manager.load_all()
        self.target_type = Globals.TARGET_TYPE  
        self.datasets = self.dataset_manager.datasets

        # Motor Velocity and acceleration
        self.Gear_Yaw = GearReductionDeg(Globals.YAW_REDUCTION)
        self.Gear_Pitch = GearReductionDeg(Globals.PITCH_REDUCTION)

        self.current_pitch_velocity_limit = Value(c_float, self.Gear_Pitch.vel(96))
        self.current_pitch_acceleration_limit = Value(c_float, self.Gear_Pitch.acc(96))
        self.current_yaw_velocity_limit = Value(c_float, self.Gear_Yaw.vel(96))
        self.current_yaw_acceleration_limit = Value(c_float, self.Gear_Yaw.acc(96))
        
        self.js_twist = Value(c_float, 0.0)
        self.js_pitch = Value(c_float, 0.0)

        # Motor controller Initialization
        self.motors_ready = Value(c_bool,False)
        self.motor_controller = MotorController(self,self.sys,self.logger)
        
        self.mode = Value(c_int,OpMode.SEMI_AUTO)
        self.zoom_level = Value(c_int,int(loaded.zoom_level)) #Value(c_int, 1) 
        self.last_time = time.time()
        self.detected_objects=self.manager.list()
        self.top_target_locked=Value(c_bool,False)
        self.static_position_sent=Value(c_bool,False)
        self.PID=Value(c_bool,False)
        self.zoom_set=Value(c_bool,False)
        self.digital_zoom_enabled = Value(c_bool, bool(loaded.digital_zoom_enabled))
        self.calib_set=Value(c_bool,False)
        self.stop_motor_flag=Value(c_bool,False)
        self.hold_start_time = Value(c_float,0.0)
        self.pres=self.manager.list()
        
        # Motor calibration required flag
        self.calibration_required = Value(c_bool,False)
        
        # Autonomous Iterator
        self.target_iterator = Value(c_int, 0)
        self.last_target_switch_time = Value(c_double,0.0)
        for x in [0, 0, 0, 0, 0]:
            self.pres.append(x)
        self.target_id=Value(c_int, -1)
        
        # Manual x and y 
        self.manual_x = Value(c_double,0.0)
        self.manual_y = Value(c_double,0.0)
        
        self.calib_pixel_x = Value(c_double,0.0)
        self.calib_pixel_y = Value(c_double,0.0)

        self.current_yaw_deg = Value(c_float,loaded.yaw_value)
        self.current_pitch_deg = Value(c_float,loaded.pitch_value)
        self.pitch_set=Value(c_bool,False)
        self.yaw_set=Value(c_bool,False)
        

        # Camera Snapshot and Record action
        self.save_dir = "logs" 
        self.record_start_flag = Value(c_bool,False)
        self.record_stop_flag = Value(c_bool,False)
        
        # Camera dictonary
        self.camera_dict =self.manager.dict()
        cam_dict = getCamPorts()
        self.camera_dict.update(cam_dict)

        # Zoom Controller
        self.zoom_ctrl=ZoomController()
    
        top_cam_serial = self.camera_dict[Globals.TOP_SERIAL]['serial']
        static_cam_serial = self.camera_dict[Globals.STATIC_SERIAL]['serial']
    
        self.zoom_ctrl.set_device_permissions(top_cam_serial,static_cam_serial)
        
        # Hit Accuracy text 
        self.acc_x_percentage = Value(c_float, 0.0)
        self.acc_y_percentage = Value(c_float, 0.0)
        self.pid_factor = Value(c_float,loaded.pid_factor_value)

        # Use person model flag
        self.face_model = Value(c_bool, True)
        # Focal length initialization
        self.focal_length = Value(c_float, 0.0)
        # self.publisher = PublishMQ()
        # Solenoid controller
        self.magnet_hold_time = Value(c_float,0.3) 
        self.short_burst_time = Value(c_float,loaded.short_burst_value)
        self.long_burst_time = Value(c_float,loaded.long_burst_value)
        self.burst_time = Value(c_float,1.0)
        self.fire_status = Value(c_bool,False)

        self.solenoid_triggered = Value(c_bool,False)
        self.switch_cameras = Value(c_bool, False)
        # Initialize controller
        try:
            self.trigger = TriggerController(port=Globals.TRIGGER_PORT, baudrate=115200)
        except Exception as e:
            print(f'Trigger aurdino error {e}')
        # Solenoid magnet states
        self.solenoid_state = self.manager.Value('str', 'OFF')  # Add solenoid state
        self.shooting_mode = "Safety"
        # self.shooting_mode = self.manager.Value('str', 'Safety')  # Make shooting_mode process-safe

        # Depth multiplication factor 
        self.depth_factor = Value(c_float,loaded.depth_factor_value)

        # PID enable flag
        self.enable_pid = Value(c_bool,loaded.pid_enable)

        # Encoder data calibration, ballestic, recoil and barrel offsets
        self.calibrate_encoders = Value(c_bool,False)
        self.ballistic_correction = Value(c_float,0.0)
        self.recoil_compensation = Value(c_float,0.0)
        self.barrel_offset_x = Value(c_int,0)
        self.barrel_offset_y = Value(c_int,0)
        self.barrel_offset_x_percentage = Value(c_float,0.5)
        self.barrel_offset_y_percentage = Value(c_float,0.5)

        # Encoder Positions
        self.pitch_enc = Value(c_double,0.0)
        self.yaw_enc = Value(c_double,0.0)

        # Source latitude and Longitude
        self.source_lat = Value(c_double,0.0)
        self.source_lon = Value(c_double,0.0)
        self.source_heading = Value(c_double,0.0)

        # variable for fire related
        self.auto_target_hold_time = Value(c_float,loaded.auto_target_hold_time)
        self.acc_percentage = Value(c_int,loaded.auto_accuracy_value)

        # Model swap and model publish list 
        self.SUPPORTED_EXTS = (".pt", ".onnx", ".engine", ".pth", ".trt")
        self.models_dir = Path(Globals.MODELS_DIR)
        self.all_models = self.list_available_models()
        # self.top_model = self.manager.Value('str', Globals.FACE_MODEL)
        # self.static_model = self.manager.Value('str', Globals.FACE_MODEL)
        self.ns = self.manager.Namespace()
        self.ns.top_model = str(loaded.top_model)
        self.ns.static_model = str(loaded.static_model)
        self.change_top_path = Value(c_bool,False)
        self.change_static_path = Value(c_bool,False)
        payload = {"top_model":self.ns.top_model,"static_model":self.ns.static_model,"all_models":self.all_models}
        self.mq_pub.publish('event','models_list',{"models_list":payload})

        self.publish_state()
        # save state here
        # self.persist_runtime_state()


    def list_available_models(self) -> list[str]:
        if not self.models_dir.exists() or not self.models_dir.is_dir():
            return []
        models = [
            p.name for p in self.models_dir.iterdir()
            if p.is_file() and p.suffix.lower() in self.SUPPORTED_EXTS
        ]
        # print(f'payload -> {payload}')
        return sorted(models, key=str.lower)
    
    # =========== state related functionalities ============
    def snapshot_runtime_state(self) -> RuntimeState:
        # print(f'[State saved]-------------------------------> True -> {self.zoom_level.value}')
        return RuntimeState(
            zoom_level=int(self.zoom_level.value),
            digital_zoom_enabled=bool(self.digital_zoom_enabled.value),
            top_model=str(self.ns.top_model),
            static_model=str(self.ns.static_model),
            op_mode=int(OpMode.SEMI_AUTO),
            yaw_value = float(self.current_yaw_deg.value),
            pitch_value = float(self.current_pitch_deg.value),
            short_burst_value = float(self.short_burst_time.value),
            long_burst_value = float(self.long_burst_time.value),
            pid_factor_value = float(self.pid_factor.value),
            depth_factor_value = float(self.depth_factor.value),
            auto_accuracy_value = int(self.acc_percentage.value),
            auto_target_hold_time = float(self.auto_target_hold_time.value),
            # auto_focus = bool(self.auto) -> need to implement functionality when loaded apply
            pid_enable = bool(self.enable_pid.value),
        )

    def persist_runtime_state(self) -> None:
        self.state_store.save(self.snapshot_runtime_state())

    def publish_state(self) -> None:
        payload = {
            "zoom_level": int(self.zoom_level.value),
            "digital_zoom_enabled": bool(self.digital_zoom_enabled.value),
            "top_model": str(self.ns.top_model),
            "static_model": str(self.ns.static_model),
            "op_mode": int(OpMode.SEMI_AUTO),
            "all_models": self.all_models,
            "yaw_value" : float(self.current_yaw_deg.value),
            "pitch_value" : float(self.current_pitch_deg.value),
            "short_burst_value" : float(self.short_burst_time.value),
            "long_burst_value" : float(self.long_burst_time.value),
            "pid_factor_value" : float(self.pid_factor.value),
            "depth_factor_value" : float(self.depth_factor.value),
            "auto_accuracy_value" : int(self.acc_percentage.value),
            "auto_target_hold_time" : float(self.auto_target_hold_time.value),
            # auto_focus : bool(self.auto) -> need to implement functionality when loaded apply
            "pid_enable" : bool(self.enable_pid.value),
        }
        print(f'states published-> {payload}')
        self.mq_pub.publish("event", "runtime_state", payload)
    # ==========================================================

    def load_calib_json(self,path):
        """Load calibration JSON into np.array dict just like np.load output."""
        with open(path, "r") as f:
            data = json.load(f)

        calib = {
            "ret": float(data.get("ret", 0.0)),
            "camera_matrix": np.array(data["camera_matrix"]),
            # standardize naming to match your existing code
            "distortion_coeffs": np.array(data.get("dist_coeffs", [])),
            "rvecs": [np.array(r) for r in data.get("rvecs", [])],
            "tvecs": [np.array(t) for t in data.get("tvecs", [])],
        }
        return calib

    def get_static_fov(self):
        entry = self.dataset_manager.get(
            "fov_data",
            self.zoom_level.value,
            subkey="static_fov"
        )
        return (
            tuple(entry["horizontal"]),
            tuple(entry["vertical"])
        )
    def get_pid_i(self):
        return tuple(
            self.dataset_manager.get("pid_data", self.zoom_level.value)
        )

    def fire_solenoid(self):
        try:
            print(f'Mode = {self.shooting_mode} sb {self.short_burst_time.value} and {self.long_burst_time.value}')
            self.fire_status.value = True
            if self.shooting_mode == "Short-Burst":
                self.burst_time.value = self.short_burst_time.value 
                self.trigger.solenoid_on()
                time.sleep(self.short_burst_time.value)
                self.trigger.solenoid_off()
                self.mq_pub.publish("event","trigger_fired",True)
            elif self.shooting_mode == "Burst":
                # self.fire_status.value = True
                self.burst_time.value = self.long_burst_time.value
                self.trigger.solenoid_on()
                time.sleep(self.long_burst_time.value)
                self.trigger.solenoid_off()
                # self.publisher.publish('',{"trigger_fired": True})
                self.mq_pub.publish("event","trigger_fired",True)
            else:
                self.burst_time.value = 0.0
                print(f'In safety cannot trigger')
                payload = {"Mode":"Safety mode","message":"In Safety Couldn't trigger"}
                self.mq_pub.publish('error','Trigger_mode',payload)
        except Exception as e:
            self.logger.error(f"fire_solenoid error: {e}", exc_info=True)
        finally:
            self.fire_status.value = False
            self.solenoid_triggered.value = True
            # print(f'Burst time = {self.burst_time.value}')

            
    def mapping(self,Min1,Max1,Min2,Max2,cmd):
        # Variable1 = np.array([Min1,Max1])
        # Variable2 = np.array([Min2,Max2])	
        # MappedValue = interp1d(Variable1,Variable2,kind='linear')
        # return float(MappedValue(cmd))
            # Prevent division by zero
        if Max1 == Min1:
            return 0.0
        return (cmd - Min1) * (Max2 - Min2) / (Max1 - Min1) + Min2  # Trying pure alzebra to reduce interp1d overhead

    def crop_center(self,frame, delta_x, delta_y, offset_x, offset_y):
        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2
        cx = cx + offset_x
        cy = cy + offset_y
        x1, y1 = max(cx - delta_x, 0), max(cy - delta_y, 0)
        x2, y2 = min(cx + delta_x, w), min(cy + delta_y, h)
        return frame[y1:y2, x1:x2], (x1, y1)

    # === Helper: Map crop-based pixel to full frame ===
    def map_crop_coords_to_full(self,cx_crop, cy_crop, offset):
        return int(cx_crop + offset[0]), int(cy_crop + offset[1])
    
    def get_calibration_data(self, zoom_level):
        return (
            self.calib_data.get(zoom_level, None),
            self.calib_data_top.get(zoom_level, None)
        )


