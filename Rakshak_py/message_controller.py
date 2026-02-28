from MQ_Base import BaseMQ_New
import inspect
from op_mode import OpMode
import Globals
import traceback
import time


class MessageController:

    def __init__(self,shared_state, shared_top, shared_static,system,logger):
        self.shared_state=shared_state
        self.shared_top = shared_top
        self.shared_static = shared_static
        self.system = system
        # New Rabbit mq variables
        self.mq_sub = BaseMQ_New(queue="rakshak_sub.q", connection_name="rakshak-sub")
        # Client listens to calibration messages coming from backend
        self.mq_sub.bind(["rakshak.gui_1.*"])
        self.mq_pub = BaseMQ_New(queue=f"{self.system}.pub.q", connection_name=f"{self.system}-pub")  # queue unused for pub but harmless
        self.logger = logger
    def on_client_msg(self, rk, msg):
        self.logger.info(f"[CLIENT RECEIVED] rk={rk} → {msg}")
        try:
            event = msg['msg_type']
            payload = msg['payload']
            if event == "operating_mode_control":
                if not self.shared_state.motors_ready.value:
                    self.shared_state.motor_controller.send_alert()
                if(payload == "Semi-Auto"):
                    self.shared_state.manual_x.value = 0 
                    self.shared_state.manual_y.value = 0
                    self.shared_state.target_id.value = -1
                    self.shared_state.PID.value = False
                    self.shared_state.static_position_sent.value = False
                    # self.shared_state.mode.value=OpMode.HOLD
                    self.shared_state.mode.value=OpMode.SEMI_AUTO
                elif(payload =="Autonomous"):
                    self.shared_state.manual_x.value = 0 
                    self.shared_state.manual_y.value = 0
                    self.shared_state.target_id.value = -1
                    self.shared_state.PID.value = False
                    self.shared_state.static_position_sent.value = False
                    self.shared_state.mode.value=OpMode.AUTO
                elif(payload =="Manual"):
                    self.shared_state.switch_cameras.value = True
                    self.shared_state.manual_x.value = 0 
                    self.shared_state.manual_y.value = 0
                    self.shared_state.target_id.value = -1
                    self.shared_state.PID.value = False
                    self.shared_state.static_position_sent.value = False
                    # self.shared_state.mode.value=OpMode.HOLD
                    self.shared_state.mode.value=OpMode.MANUAL
                elif(payload =="Manual_JS"):
                    # self.shared_state.switch_cameras.value = True
                    self.shared_state.manual_x.value = 0 
                    self.shared_state.manual_y.value = 0
                    self.shared_state.target_id.value = -1
                    self.shared_state.PID.value = False
                    self.shared_state.static_position_sent.value = False
                    self.shared_state.mode.value=OpMode.MANUAL_JS
                elif(payload =="Scan"):
                    self.shared_state.stop_motor_flag.value=True
                    self.shared_state.switch_cameras.value = True
                    self.shared_state.manual_x.value = 0 
                    self.shared_state.manual_y.value = 0
                    self.shared_state.target_id.value = -1
                    self.shared_state.PID.value = False
                    self.shared_state.static_position_sent.value = False
                    self.shared_state.mode.value=OpMode.SCAN
                elif(payload =="Demo"):
                    # self.shared_state.switch_cameras.value = True
                    self.shared_state.manual_x.value = 0 
                    self.shared_state.manual_y.value = 0
                    self.shared_state.target_id.value = -1
                    self.shared_state.PID.value = False
                    self.shared_state.static_position_sent.value = False
                    self.shared_state.mode.value=OpMode.DEMO
                elif(payload =="Thermal"):
                    # self.shared_state.switch_cameras.value = True
                    self.shared_state.manual_x.value = 0 
                    self.shared_state.manual_y.value = 0
                    self.shared_state.target_id.value = -1
                    self.shared_state.PID.value = False
                    self.shared_state.static_position_sent.value = False
                    self.shared_state.mode.value=OpMode.THERMAL
            elif event == "joystick_control" and self.shared_state.mode.value == OpMode.MANUAL_JS:
                self.shared_state.js_pitch.value = payload['pitch_joystick_control']
                self.shared_state.js_twist.value = payload['yaw_joystick_control']
            elif event == "request_states":
                print(f'states requested')
                self.shared_state.publish_state()
            elif event == "request_models":
                # print(f'models -> {self.shared_state.all_models}')
                payload = {"top_model":self.shared_state.ns.top_model,"static_model":self.shared_state.ns.static_model,"all_models":self.shared_state.all_models}
                # payload = {"models_list":self.shared_state.all_models}
                self.mq_pub.publish('event','models_list',{"models_list":payload})
            elif event == "zoom_control":
                self.shared_state.manual_x.value = 0 
                self.shared_state.manual_y.value = 0
                zoom_level = payload
                self.shared_state.zoom_level.value=zoom_level
                self.shared_state.zoom_set.value=True
                top_cam_serial  = self.shared_state.camera_dict[Globals.TOP_SERIAL]['serial']
                static_cam_serial = self.shared_state.camera_dict[Globals.STATIC_SERIAL]['serial']
                if zoom_level <= 30:
                    if self.shared_state.digital_zoom_enabled.value:
                        self.shared_state.zoom_ctrl.digital_zoom(1, top_cam_serial, static_cam_serial)
                        self.shared_state.zoom_ctrl.turn_off_digital_zoom(top_cam_serial,static_cam_serial)
                        self.shared_state.digital_zoom_enabled.value = False
                    self.shared_state.zoom_ctrl.setZoom(zoom_level, top_cam_serial, static_cam_serial)
                else:
                    self.shared_state.zoom_ctrl.setZoom(30, top_cam_serial, static_cam_serial)
                    if not self.shared_state.digital_zoom_enabled.value:
                        self.shared_state.zoom_ctrl.turn_on_digital_zoom(top_cam_serial,static_cam_serial)
                        self.shared_state.digital_zoom_enabled.value = True
                    self.shared_state.zoom_ctrl.digital_zoom(zoom_level, top_cam_serial, static_cam_serial)
            elif event == 'selected_bbox':
                if not self.shared_state.motors_ready.value:
                    self.shared_state.motor_controller.send_alert()
                msg = {"pos_err_x": 0, "pos_err_y": 0}
                self.mq_pub.publish("accuracy","accuracy",msg)
                self.shared_state.top_target_locked.value = False  # Reset top cam print flag
                self.shared_state.static_position_sent.value = False
                self.shared_state.PID.value = False
                self.shared_state.target_id.value=payload
                self.shared_state.pres = [0, 0, 0, 0, 0]
            elif event == "sendfxfy_action":
                    if not self.shared_state.motors_ready.value:
                        self.shared_state.motor_controller.send_alert()
                    msg = {"pos_err_x": 0, "pos_err_y": 0}
                    self.mq_pub.publish("accuracy","accuracy",msg)
                    self.shared_state.manual_x.value = payload["manual_center_x"]
                    self.shared_state.manual_y.value = payload["manual_center_y"]
            elif event == "calibration_action":
                self.shared_state.calib_set.value=True
                if not self.shared_state.motors_ready.value:
                    self.shared_state.motor_controller.send_alert()
                if("calibration_center_x" in payload and "calibration_center_y" in payload):
                    self.shared_state.calib_pixel_x.value = payload["calibration_center_x"]
                    self.shared_state.calib_pixel_y.value = payload["calibration_center_y"]
                else:
                    self.shared_state.calib_pixel_x.value = 0.0
                    self.shared_state.calib_pixel_y.value = 0.0
            elif event == "calibrate_encoder_action":
                print("[INFO] Encoder calibration requested")
                self.shared_state.calibrate_encoders.value = payload
            elif event == "camera_barrel_offset_action":
                print('inside ooffset action', payload)
                fx = int(payload["off_set_fx"])
                fy = int(payload["off_set_fy"])
                
                self.shared_state.barrel_offset_x.value = fx
                self.shared_state.barrel_offset_y.value = fy
            elif event == "barrel_zeroin_action":
                percentage_x = float(payload.get("x_pct", 0.0)) / 100 # recieves the percentage of a box in x
                percentage_y = float(payload.get("y_pct", 0.0)) / 100 # recieves the percentage of a box in y 
                
                self.shared_state.barrel_offset_x_percentage.value = percentage_x
                self.shared_state.barrel_offset_y_percentage.value = percentage_y
            elif event == "stop_motor_action":
                self.shared_state.manual_x.value = 0 
                self.shared_state.manual_y.value = 0
                self.shared_state.stop_motor_flag.value=True
            elif event=="set_pitch_action":
                if not self.shared_state.motors_ready.value:
                    self.shared_state.motor_controller.send_alert()
                self.shared_state.manual_x.value = 0 
                self.shared_state.manual_y.value = 0
                self.shared_state.current_pitch_deg.value =payload["pitch_value"]
                self.shared_state.current_pitch_velocity_limit.value = payload["Pitch_Velocity"]
                self.shared_state.current_pitch_acceleration_limit.value = payload["Pitch_Acceleration"]
                self.shared_state.pitch_set.value=True
            elif event =="set_yaw_action":
                if not self.shared_state.motors_ready.value:
                    self.shared_state.motor_controller.send_alert()
                self.shared_state.manual_x.value = 0 
                self.shared_state.manual_y.value = 0
                self.shared_state.current_yaw_deg.value = payload["yaw_value"]
                self.shared_state.current_yaw_velocity_limit.value = payload["Yaw_Velocity"]
                self.shared_state.current_yaw_acceleration_limit.value = payload["Yaw_Acceleration"]
                self.shared_state.yaw_set.value=True
            elif event == "trigger_snapshot_action":
                # take_snapshot(self.shared_state,self.shared_top,self.shared_static)
                print("snap shot function called but commented while constructing modularity")
            elif event == "trigger_recording_action":
                if payload["recording_action"] == 'START_RECORDING':
                    self.shared_state.record_start_flag.value = True
                    print(f'shared_state.record_start_flag.value={self.shared_state.record_start_flag.value}')
                elif payload["recording_action"] == 'STOP_RECORDING':
                    self.shared_state.record_stop_flag.value = True
            elif event == "camera_settings_action":
                top_cam_serial  = self.shared_state.camera_dict[Globals.TOP_SERIAL]['serial']
                static_cam_serial = self.shared_state.camera_dict[Globals.STATIC_SERIAL]['serial']
                action = payload["camera_settings_action"]
                # enabled = msg["enabled"]
                if action == "IMAGE_STABILIZATION":
                    if payload["enabled"]:
                        self.shared_state.zoom_ctrl.enable_image_stabilization(top_cam_serial,static_cam_serial)
                        # enable_stabilization()
                    else:
                        self.shared_state.zoom_ctrl.disable_image_stabilization(top_cam_serial,static_cam_serial)
                elif action == "AUTO_FOCUS":
                    if payload["enabled"]:
                        self.shared_state.zoom_ctrl.toggle_autofocus(top_cam_serial,static_cam_serial)
                    else:
                        self.shared_state.zoom_ctrl.manual_focus(top_cam_serial,static_cam_serial)
                elif action == "DETECTION_DISPLAY":
                    if payload["enabled"]:
                        pass
                    else:
                        pass
            elif event == "change_model":
                camera = payload.get("camera",None)
                path = payload.get("payload",None)
                # print(f'camer->{camera} path -> {path}')
                if camera == "Top_Cam" and path is not None:
                    self.shared_state.ns.top_model = path
                    self.shared_state.change_top_path.value = True
                elif camera == "Static_Cam" and path is not None:
                    self.shared_state.ns.static_model = path
                    self.shared_state.change_static_path.value = True
            elif event == "change_model_action":
                enabled_raw = payload.get("enabled", False)
                self.shared_state.face_model.value = enabled_raw
            elif event == "global_config_action":
                action = payload['action']
                print(f'payload = {payload}')
                if action == "PID_FACTOR":
                    factor = payload.get("pid_factor",1.0)
                    self.shared_state.pid_factor.value = factor
                elif action == "DEPTH_FACTOR":
                    factor = payload.get("depth_factor",1.0)
                    self.shared_state.depth_factor.value = factor
                elif action == "MAGNETS_TIME":
                    delay = payload.get("magnets_time",0.3)
                    self.shared_state.magnet_hold_time.value = delay
                    print(f'msg ={delay} and time = {self.shared_state.magnet_hold_time.value}')
                elif action == "BALLISTIC_RECOIL_VALUES":
                    self.shared_state.ballistic_correction.value = payload.get("ballistic_correction", 0)
                    self.shared_state.recoil_compensation.value = payload.get("recoil_compensation", 0)
                elif action == "ENABLE_CALIBRATION":
                    global calibration_mode
                    if payload["enabled"]:
                        calibration_mode = True
                        self.shared_state.calibration_required.value = True
                    else:
                        calibration_mode = False
                        self.shared_state.calibration_required.value = False
                elif action == 'ENABLE_PID':
                    enabled = payload["enabled"]
                    print(f'pid settings action = {enabled}')
                    self.shared_state.enable_pid.value = enabled
                elif action == "shooting_mode_action":
                    self.shared_state.shooting_mode = payload["shooting_mode_action"]
                elif action == "fire_action":
                    if payload["fire_action"] == "FIRE":
                        print(f'In rabbitmq fire')
                        self.shared_state.fire_solenoid()
                elif action == "laser_on_action":
                    if payload["laser_on_action"] == True:
                        # print(f'In rabbitmq fire')
                        self.shared_state.trigger.laser_on()
                        time.sleep(2)
                        self.shared_state.trigger.laser_off()
                elif action == "fire_settings_action":
                    mode = payload["fire_settings_action"]
                    duration = payload.get("duration")  # Default 1 sec if not provided
                    print("dureation = ",duration)
                    # Update shooting mode
                    if mode == "SHORT_BURST":
                        self.shared_state.short_burst_time.value = duration
                    elif mode == "LONG_BURST":
                        self.shared_state.long_burst_time.value = duration
                    else:
                        print("Unknown fire settings action:", mode)

                    print(f"Updated settings → Mode: {self.shooting_mode}, Duration: {duration}")
                elif action == "set_fov_action":
                    zoom = payload["zoom_level"]
                    fov_data = {
                        "horizontal": [payload["horizontal_neg"], payload["horizontal_pos"]],
                        "vertical": [payload["vertical_neg"], payload["vertical_pos"]]
                    }
                    self.shared_state.dataset_manager.update_zoom("fov_data", zoom, fov_data, subkey="static_fov")
                    self.shared_state.dataset_manager.load("fov_data")
                elif action == "set_pid_action":
                    zoom = payload["zoom_level"]
                    pitch_i = payload.get("pitch_i", 0.0)
                    yaw_i = payload.get("yaw_i", 0.0)
                    self.shared_state.dataset_manager.update_zoom("pid_data", zoom, [yaw_i,pitch_i])
            self.shared_state.persist_runtime_state()                
        except Exception as ex:
            self.logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
            error_str = traceback.format_exc()
            if(error_str and len(error_str)>0):
                self.logger.error(f'Exception Trace:{error_str}')

    def run(self):
        self.mq_sub.subscribe(self.on_client_msg)