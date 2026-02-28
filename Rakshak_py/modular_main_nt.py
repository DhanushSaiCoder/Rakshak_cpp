import cv2
import time
import numpy as np

from ultralytics import YOLO
import moteus # type: ignore
from scipy.interpolate import interp1d
import numpy
import torch
# If your input sizes are fixed-ish, let cuDNN pick fastest algos
torch.backends.cudnn.benchmark = True

import multiprocessing as mp

from multiprocessing import Value
from multiprocessing import shared_memory
from multiprocessing import Process, Queue
import threading

import ctypes
ctypes.CDLL('libX11.so').XInitThreads()  # Fix XCB crash in multithreaded OpenCV
from multiprocessing import Value
from ctypes import c_int,Array,c_bool,c_double,c_float
import warnings
warnings.filterwarnings("ignore", message="The value of the smallest subnormal*")
from stream_server import run_https_server
from ultralytics.utils import LOGGER
LOGGER.setLevel('WARNING') 
import asyncio
from share_frame import SharedFrame
from zoom_controller import ZoomController
import sys
import json
from enum import IntEnum, Enum
import math
from math import sin, cos, asin, atan2, radians, degrees
import re
from MQ_Base import BaseMQ_New
from datetime import datetime
import os
from utils import getCamPorts
import inspect

from log_util import setup_combined_logger

import traceback
import Globals
from solenoid_controller import TriggerController
from modular_sensor_reader import SensorReader,SensorLogger
from gps_yaw.modular_reader import SensorReaderGPS
import json

logger=setup_combined_logger(__name__)


if not Globals.SYSTEM_ID:
    raise ValueError("[BaseMQ Init] SYSTEM_ID not found in globals.py")

SYS = Globals.SYSTEM_ID.strip()
SYS = re.sub(r"[^a-zA-Z0-9_.-]", "", SYS)  # sanitize

if not SYS:
    raise ValueError("[BaseMQ Init] SYSTEM_ID is empty or invalid after sanitization")

SYS = SYS.lower()  # normalize for routing keys, queue names

# Paths for depth calculation
DATA_DIR = "data"
DATASETS_CONFIG = {
    "person": (os.path.join(DATA_DIR, "person_depth_data.json"), "person_data"),
    "target": (os.path.join(DATA_DIR, "target_depth_data.json"), "target_data"),
    "pid_data": (os.path.join(DATA_DIR, "pid_data.json"), "pid_data"),
    "fov_data": (os.path.join(DATA_DIR, "fov_data_m.json"), "fov_data"),
    # "dynamic_zero_pos": (os.path.join(DATA_DIR, "fov_data_m.json"), "fov_data"),
}
# MQ_SUB = BaseMQ_New(queue=f"{SYS}.q", connection_name=f"{SYS}-sub")
MQ_PUB = BaseMQ_New(queue=f"{SYS}.pub.q", connection_name=f"{SYS}-pub")  # queue unused for pub but harmless
# Bind backend to receive ONLY bbox.* (what the client sends)
# MQ_SUB.bind([
#     f"rakshak.{SYS}.*",   # targeted commands
#     "rakshak.all.*",  # optional broadcast channel
# ])



class MotorController:
    def __init__(self,shared_state):
        self.YawMotor = moteus.Controller(2)
        #self.CamMotor = moteus.Controller(2)
        self.PitchMotor = moteus.Controller(1)
        # self.mq_publisher_limit=PublishMQ()
        self.mq_publisher_limit = BaseMQ_New(queue=f"{SYS}-Motor_controller.pub.q", connection_name=f"{SYS}-Motor_controller-pub")  # queue unused for pub but harmless
        self.shared_state=shared_state
        self.pid_dict = self.shared_state.datasets.get("pid_data", {})
        # self.load_fov_dict()
        self.manual_yaw_limit = self.shared_state.mapping(-Globals.REVOLUTION,Globals.REVOLUTION,-Globals.YAW_REDUCTION,Globals.YAW_REDUCTION,Globals.YAW_LIMIT)
        self.manual_pitch_limit = self.shared_state.mapping(-Globals.REVOLUTION,Globals.REVOLUTION,-Globals.PITCH_REDUCTION,Globals.PITCH_REDUCTION,Globals.PITCH_LIMIT)
        print(f"Manual yaw limit = {self.manual_yaw_limit} ------> {self.manual_pitch_limit}")
        
    async def hold_motors(self):
        # print(f'Holding motors')
        try:
            YawCmd = await self.YawMotor.set_position(
                            position=math.nan,
                            query=True)
            PitchCmd = await self.PitchMotor.set_position(
                        position=math.nan,
                        query=True)
        except Exception as ex:
           logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
    
    def pos_cmd(self, w,h,HFOV,VFOV,offset_x,offset_y):
        rev_deg=Globals.REVOLUTION 
        HMap = self.shared_state.mapping(-w,w,HFOV[0],HFOV[1],offset_x)
        VMap = self.shared_state.mapping(-h,h,VFOV[0],VFOV[1],offset_y)
        if abs(HMap) > Globals.YAW_LIMIT or abs(VMap) > Globals.PITCH_LIMIT:
            HMap = Globals.YAW_LIMIT if HMap > 0 else -Globals.YAW_LIMIT
            VMap = Globals.PITCH_LIMIT if VMap > 0 else -Globals.PITCH_LIMIT
            # self.mq_publisher_limit.publish('',{"limit_reached":True})
            payload = {"limit_reached":True}
            self.mq_publisher_limit.publish("event","motor_limit",payload)
        
        YawPositionCmd = self.shared_state.mapping(-rev_deg,rev_deg,-Globals.YAW_REDUCTION,Globals.YAW_REDUCTION,HMap)
        PitchPositionCmd = self.shared_state.mapping(-rev_deg,rev_deg,-Globals.PITCH_REDUCTION,Globals.PITCH_REDUCTION,VMap)
        
        return (YawPositionCmd,PitchPositionCmd)

    async def stop_motors(self):
        try:
            YawStates = await self.YawMotor.set_stop(query=True)
            PitchStates = await self.PitchMotor.set_stop(query=True)
            #camState = await self.CamMotor.set_stop(query=True)
            print(f"Inside Motor Stop function {YawStates.values[moteus.Register.POSITION]} --- {PitchStates.values[moteus.Register.POSITION]}")
            print("Pitch Fault = ",PitchStates.values[moteus.Register.FAULT])
            print("Yaw Fault = ",YawStates.values[moteus.Register.FAULT])
            return (YawStates,PitchStates)
        except Exception as ex:
            logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
    
    async def Calibrate(self,width,height):
        done=True
        # Reset Motor Faults upon initializaiton
        await self.YawMotor.set_stop()
        await self.PitchMotor.set_stop()
       # await self.CamMotor.set_stop()
        # Calibrate motor to 0 degrees 
        # await self.YawMotor.set_output_exact(position = 0.0)
        # await self.PitchMotor.set_output_exact(position = 0.0)
        
        if self.shared_state.calib_pixel_x.value != 0 and self.shared_state.calib_pixel_y.value != 0:
            width = width/2
            height = height/2
            print(f'x = {self.shared_state.calib_pixel_x.value} y = {self.shared_state.calib_pixel_y.value}')
            pixel_x = self.shared_state.calib_pixel_x.value - width
            pixel_y = self.shared_state.calib_pixel_y.value - height
            print(f'width {width} height {height}')
            fov_entry = self.shared_state.dataset_manager.get(
                            "fov_data",
                            self.shared_state.zoom_level.value,
                            subkey="static_fov"
                        )

            HFOV = tuple(fov_entry["horizontal"])  # e.g., (-32.0, 32.0)
            VFOV = tuple(fov_entry["vertical"])    # e.g., (-18.0, 18.0)
            # print(f'HFOV {HFOV} VFOV {VFOV}')
            yawPos,pitchPos = self.pos_cmd(width,height,HFOV,VFOV,pixel_x,pixel_y)
            await self.YawMotor.set_output_exact(position = -1 * (yawPos))
            await self.PitchMotor.set_output_exact(position = -1 * (-(-pitchPos)))
            #await self.CamMotor.set_output_exact(position = 0.0)
            print(f'Inside dynamic calibration')
        # else: 
        #     await self.YawMotor.set_output_exact(position = 0.0)
        #     await self.PitchMotor.set_output_exact(position = 0.0)
            
            i = 0
            while True:
                try:
                    YawCmd = await self.YawMotor.set_position(
                                position=math.nan,
                                query=True)
                    PitchCmd = await self.PitchMotor.set_position(
                                position=math.nan,
                                query=True)
                    
                    # yaw_fault=
                    # print(f'Iter = {i}')
                    # Logs
                    # # Yaw
                    print("Yaw Position = ",YawCmd.values[moteus.Register.POSITION])
                    # print("Yaw Velocity = ",YawCmd.values[moteus.Register.VELOCITY])
                    # print("Yaw Torque = ",YawCmd.values[moteus.Register.TORQUE])
                    # print("Yaw Temperature = ",YawCmd.values[moteus.Register.TEMPERATURE])
                    # print("Yaw Voltage = ",YawCmd.values[moteus.Register.VOLTAGE])
                    # print("Yaw Fault = ",YawCmd.values[moteus.Register.FAULT])
                    # # Pitch
                    print("Pitch Position = ",PitchCmd.values[moteus.Register.POSITION])
                    # print("Pitch Velocity = ",PitchCmd.values[moteus.Register.VELOCITY])
                    # print("Pitch Torque = ",PitchCmd.values[moteus.Register.TORQUE])
                    # print("Pitch Temperature = ",PitchCmd.values[moteus.Register.TEMPERATURE])
                    # print("Pitch Voltage = ",PitchCmd.values[moteus.Register.VOLTAGE])
                    # print("Pitch Fault = ",PitchCmd.values[moteus.Register.FAULT])
                    if i == 1:
                        print("Calibration Successfull !!!")
                        self.mq_publisher_limit.publish("event","calib_status",{"calib_status":True})
                        await asyncio.sleep(0.5)
                        print("Disarming Motors")
                        # publish_message({"yaw_fault":res[0],"pitch_fault":res[1],"yaw_pos":res[2],"pitch_pos":res[3],"yaw_voltage":res[4],"pitch_voltage":res[5]})
                        await self.YawMotor.set_stop()
                        await self.PitchMotor.set_stop()
                        break
                    i += 1
                    await asyncio.sleep(0.5)
                except Exception as ex:
                    done=False
                    logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
        return done
    
    async def Scan_mode(self,yaw_pos_cmd,pitch_pos_cmd,cam_pos_cmd):
        # print(f'Scan mode controller,yaw = {yaw_pos_cmd}, pit = {pitch_pos_cmd} ,cam = {cam_pos_cmd}')
       # CamState = await self.CamMotor.set_position(position = cam_pos_cmd,
        #                        velocity_limit = 2.5/2,
         #                       accel_limit = 2.5/2,
          #                      query=True)
        YawState = await self.YawMotor.set_position(position = yaw_pos_cmd,
                                    velocity_limit = Globals.VELOCITY_LIMIT,
                                    accel_limit = Globals.ACCELERATION_LIMIT,
                                    query=True)
        PitchState = await self.PitchMotor.set_position(position = pitch_pos_cmd, #20
                                velocity_limit = Globals.VELOCITY_LIMIT,
                                accel_limit = Globals.ACCELERATION_LIMIT,
                                query=True)

    async def set_yaw(self):
        try:
            yaw = await self.YawMotor.custom_query({moteus.Register.POSITION: moteus.F32})
            position = yaw.values[moteus.Register.POSITION]
            YawPositionCmd = Globals.YAW_POS_CONST * (self.shared_state.mapping(-Globals.REVOLUTION,Globals.REVOLUTION,-Globals.YAW_REDUCTION,Globals.YAW_REDUCTION,self.shared_state.current_yaw_deg.value))
            if -self.manual_yaw_limit < position+YawPositionCmd < self.manual_yaw_limit:
                velocity_limit = self.shared_state.current_yaw_velocity_limit.value
                accel_limit = self.shared_state.current_yaw_acceleration_limit.value
                # print(f'Yaw Velo = {velocity_limit}, {accel_limit}')
                await self.YawMotor.set_position(position= position + YawPositionCmd, velocity_limit=velocity_limit, accel_limit=accel_limit, query=True)
                
            else:
                print("Reached to max yaw Calibration required",-self.manual_yaw_limit)
                # self.mq_publisher_limit.publish('',{"limit_reached":True})
                payload = {"limit_reached":True}
                self.mq_publisher_limit.publish("event","motor_limit",payload)
                # print(f'Yaw Request={self.shared_state.current_yaw_deg.value}')
        except Exception as ex:
            logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')

    async def set_pitch(self):
        try:
            pitch = await self.PitchMotor.custom_query({moteus.Register.POSITION: moteus.F32})
            position = pitch.values[moteus.Register.POSITION]
            PitchPositionCmd = Globals.PITCH_POS_CONST * (self.shared_state.mapping(-Globals.REVOLUTION,Globals.REVOLUTION,-Globals.PITCH_REDUCTION,Globals.PITCH_REDUCTION,self.shared_state.current_pitch_deg.value))
            if -self.manual_pitch_limit < position+PitchPositionCmd < self.manual_pitch_limit:
                velocity_limit = self.shared_state.current_pitch_velocity_limit.value
                accel_limit = self.shared_state.current_pitch_acceleration_limit.value
                # print(f'Pitch Request={self.shared_state.current_pitch_deg.value}')
                # print(f'Pitch Velo = {velocity_limit}, {accel_limit}')
                await self.PitchMotor.set_position(position=position + PitchPositionCmd, velocity_limit=velocity_limit, accel_limit=accel_limit, query=True)
            else:
                print("Reached to max pitch calibration required")
                # self.mq_publisher_limit.publish('',{"limit_reached":True})
                payload = {"limit_reached":True}
                self.mq_publisher_limit.publish("event","motor_limit",payload)
        except Exception as ex:
            logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
    
    async def publish_health(self):
        # mq_publisher=PublishMQ()
        mq_publisher = BaseMQ_New(queue=f"{SYS}-Motor_health.pub.q", connection_name=f"{SYS}-Motor_health-pub")  # queue unused for pub but harmless
        while not self.shared_state.stop_flag.value:
            try:
                health_data=[]
                for motor in [self.PitchMotor,self.YawMotor]:
                    result = await motor.custom_query({
                        moteus.Register.POSITION: moteus.F32,
                        moteus.Register.VELOCITY: moteus.F32,
                        moteus.Register.TORQUE: moteus.F32,
                        moteus.Register.VOLTAGE: moteus.F32,
                        moteus.Register.TEMPERATURE: moteus.F32,
                        moteus.Register.FAULT: moteus.F32,
                        moteus.Register.D_CURRENT: moteus.F32,
                        moteus.Register.Q_CURRENT: moteus.F32,
                    })

                    if result is not None and hasattr(result, 'values'):
                        # timestamp_ms = time.time() * 1000
                        position = result.values[moteus.Register.POSITION]
                        velocity = result.values[moteus.Register.VELOCITY]
                        torque = result.values[moteus.Register.TORQUE]
                        voltage = result.values[moteus.Register.VOLTAGE]
                        temperature = result.values[moteus.Register.TEMPERATURE]
                        fault = result.values[moteus.Register.FAULT]
                        d_current = result.values[moteus.Register.D_CURRENT]
                        q_current = result.values[moteus.Register.Q_CURRENT]
                        total_current = math.sqrt(d_current ** 2 + q_current ** 2)
                        
                        timestamp_str = time.strftime('%H:%M:%S', time.localtime())
                        
                        if motor is self.PitchMotor:
                            conversion = Globals.REVOLUTION/Globals.PITCH_REDUCTION
                            self.shared_state.pitch_enc.value = float(position)
                        elif motor is self.YawMotor:
                            conversion = Globals.REVOLUTION/Globals.YAW_REDUCTION
                            self.shared_state.yaw_enc.value = float(position)

                        data = {
                            "motor_id": motor.id,
                            "timestamp_str": timestamp_str,
                            "position": position,
                            "velocity": velocity,
                            "torque": torque,
                            "voltage": voltage,
                            "temperature": temperature,
                            "fault": fault,
                            "d_current": d_current,
                            "q_current": q_current,
                            "total_current": total_current
                        }
                        health_data.append(data)
                mq_publisher.publish("health", "health_data", health_data)
            except Exception as ex:
                logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
            await asyncio.sleep(0.1)


    async def StaticControl(self,YawCurrentPos,PitchCurrentPos,YawPositionCmd,PitchPositionCmd):
        # print(f'Inside static')
        try:
            YawSetPosition = await self.YawMotor.set_position(
                                                        position= Globals.YAW_POS_CONST * YawPositionCmd, #math.nan,
                                                        # position=math.nan,
                                                        velocity_limit = Globals.VELOCITY_LIMIT,
                                                        accel_limit = Globals.ACCELERATION_LIMIT,
                                                        # kp_scale = Globals.KP_SCALE_YAW,
                                                        query=True)
            YawCurrentPos = Globals.YAW_POS_CONST * YawSetPosition.values[moteus.Register.POSITION]

            PitchSetPosition = await self.PitchMotor.set_position(
                                                            position= Globals.PITCH_POS_CONST * (-(-PitchPositionCmd)),#math.nan,
                                                            # position=math.nan,
                                                            velocity_limit = Globals.VELOCITY_LIMIT,
                                                            accel_limit = Globals.ACCELERATION_LIMIT,
                                                            # kp_scale = Globals.KP_SCALE_PITCH,
                                                            query=True)
            PitchCurrentPos = Globals.PITCH_POS_CONST * (-(-PitchSetPosition.values[moteus.Register.POSITION]))
            # print("STATICCCCCCCCCCCCCCCCCCCCCCCCCC")
            # print(f'Yaw = {YawCurrentPos} Pitch = {PitchCurrentPos}')
            return YawCurrentPos, PitchCurrentPos
        except Exception as ex:
            logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')

    async def PIDControl(self,YawCurrentPos,PitchCurrentPos,error_x,error_y,delta_t,PrevIx,PrevIy,zoom,height):
        # print(f'pid control')
        # P Controller x = 35.706691400602885           y = 21.0491446668607678
        # print(f'Error x = {error_x} and error_y {error_y}')
        try:
            kix,kiy = self.shared_state.dataset_manager.get("pid_data", zoom)
            # kix,kiy = self.pid_dict[zoom] #0.8(present working gains)
            # print(f'xi,yi = {error_x} and {error_y}')
            # I Controller x
            Ix = PrevIx + (error_x * delta_t)
            IX = kix * Ix

            # I Controller y
            Iy = PrevIy + (error_y * delta_t)
            IY = kiy * Iy
            # print("previy = ",PrevIy)
            # print("Iy = ", Iy)
            # print("IY = ",IY)
            yawcmd = YawCurrentPos+IX
            pitchcmd = -(-PitchCurrentPos-IY)
            # print(f'yaw = {yawcmd} and pitch = {pitchcmd}')
            # print(f'Yaw = {YawCurrentPos+IX} = {self.manual_yaw_limit} Pitch = {YawCurrentPos+IX} = {self.manual_pitch_limit}')
            if any([abs(pitchcmd) >= self.manual_pitch_limit,abs(yawcmd) >= self.manual_yaw_limit]):
                print("Reached to max Calibration required")
                # print(f'Yaw = {YawCurrentPos} = {self.manual_yaw_limit} Pitch = {PitchCurrentPos} = {self.manual_pitch_limit}')
                # self.mq_publisher_limit.publish('',{"limit_reached":True})
                payload = {"limit_reached":True}
                self.mq_publisher_limit.publish("event","motor_limit",payload)
                yawcmd = math.nan
                pitchcmd = math.nan
                return None
            else:
                YawSetPosition = await self.YawMotor.set_position(
                                                            position = Globals.YAW_POS_CONST * yawcmd, #math.nan,
                                                            velocity_limit = Globals.VELOCITY_LIMIT,
                                                            accel_limit = Globals.ACCELERATION_LIMIT,
                                                            # kp_scale = Globals.KP_SCALE_YAW,
                                                            query=True)
                YawCurrentPos = Globals.YAW_POS_CONST * YawSetPosition.values[moteus.Register.POSITION]

                PitchSetPosition = await self.PitchMotor.set_position(
                                                                position= Globals.PITCH_POS_CONST * pitchcmd, #-Py,#math.nan,
                                                                velocity_limit = Globals.VELOCITY_LIMIT,
                                                                accel_limit = Globals.ACCELERATION_LIMIT,
                                                                # kp_scale = Globals.KP_SCALE_PITCH,
                                                                query=True)
                PitchCurrentPos = Globals.PITCH_POS_CONST * (-(-PitchSetPosition.values[moteus.Register.POSITION]))
                # print(f'PIDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD')
                # print("Current POsition yaw = ", YawCurrentPos)
                # print("YAW Error = ", error_x)
                # print("errorCorrectionYaw =", IX)
                return YawCurrentPos, PitchCurrentPos,Ix,Iy,IX,IY
        except Exception as ex:
            logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
        return None
    
    async def StaticDynamicControl(self,YawCurrentPos,PitchCurrentPos,YawPositionCmd,PitchPositionCmd,height,zoom):
        # print(f'Dynamic static control')
        try:    
            pos_entry = Globals.POS_DATA.get(zoom)
            height_list = pos_entry["height"]
            yaw_list = pos_entry["yaw_pos"]
            pitch_list = pos_entry["pitch_pos"]
            
            input_values = height_list
            output1 = yaw_list
            output2 = pitch_list
            # Create interp functions
            interp_output1 = interp1d(input_values,output1,kind='quadratic',fill_value = "extrapolate")
            interp_output2 = interp1d(input_values,output2,kind='quadratic',fill_value = "extrapolate")
            YawSetPosition = await self.YawMotor.set_position(
                                                        position= interp_output1(height)+YawPositionCmd, #math.nan,
                                                        velocity_limit = Globals.VELOCITY_LIMIT,
                                                        accel_limit = Globals.ACCELERATION_LIMIT,
                                                        query=True)
            YawCurrentPos = YawSetPosition.values[moteus.Register.POSITION]

            PitchSetPosition = await self.PitchMotor.set_position(
                                                            position= -(-PitchPositionCmd+interp_output2(height)),#math.nan,
                                                            velocity_limit = Globals.VELOCITY_LIMIT,
                                                            accel_limit = Globals.ACCELERATION_LIMIT,
                                                            query=True)
            PitchCurrentPos = -(-PitchSetPosition.values[moteus.Register.POSITION])
            return YawCurrentPos-interp_output1(height), PitchCurrentPos+interp_output2(height)
        except Exception as ex:
            logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
    
    async def PIDDynamicControl(self,YawCurrentPos,PitchCurrentPos,error_x,error_y,delta_t,PrevIx,PrevIy,zoom,h):
        # print(f'Dynamic pid control')
        try:
            kix,kiy = self.shared_state.dataset_manager.get("pid_data", zoom)

            pos_entry = Globals.POS_DATA.get(zoom)

            height_list = pos_entry["height"]
            yaw_list = pos_entry["yaw_pos"]
            pitch_list = pos_entry["pitch_pos"]

            input_values = height_list
            output1 = yaw_list
            output2 = pitch_list

            # Create interp functions
            interp_output1 = interp1d(input_values,output1,kind='quadratic',fill_value = "extrapolate")
            interp_output2 = interp1d(input_values,output2,kind='quadratic',fill_value = "extrapolate")
            
            # I Controller x
            Ix = PrevIx + (error_x * delta_t)
            IX = kix * Ix
            Iy = PrevIy + (error_y * delta_t)
            IY = kiy * Iy

            yawcmd = YawCurrentPos+IX+interp_output1(h)
            pitchcmd = -(-PitchCurrentPos-IY+interp_output2(h))
            if any([abs(pitchcmd) >= self.manual_pitch_limit,abs(yawcmd) >= self.manual_yaw_limit]):
                    print("Reached to max Calibration required")
                    # print(f'Yaw = {YawCurrentPos} = {self.manual_yaw_limit} Pitch = {PitchCurrentPos} = {self.manual_pitch_limit}')
                    # self.mq_publisher_limit.publish('',{"limit_reached":True})
                    payload = {"limit_reached":True}
                    self.mq_publisher_limit.publish("event","motor_limit",payload)
                    yawcmd = math.nan
                    pitchcmd = math.nan
                    return None
            else:
                YawSetPosition = await self.YawMotor.set_position(
                                                            position= yawcmd, #math.nan,
                                                            velocity_limit = Globals.VELOCITY_LIMIT,
                                                            accel_limit = Globals.ACCELERATION_LIMIT,
                                                            query=True)
                YawCurrentPos = YawSetPosition.values[moteus.Register.POSITION]

                PitchSetPosition = await self.PitchMotor.set_position(
                                                                position= pitchcmd, #-Py,#math.nan,
                                                                velocity_limit = Globals.VELOCITY_LIMIT,
                                                                accel_limit = Globals.ACCELERATION_LIMIT,
                                                                query=True)
                PitchCurrentPos = -(-PitchSetPosition.values[moteus.Register.POSITION])
                return YawCurrentPos, PitchCurrentPos,Ix,Iy,IX,IY
        except Exception as ex:
            logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')

class OpMode(IntEnum):
    SEMI_AUTO=1
    AUTO=2
    MANUAL=3
    SCAN=4
    DEMO=5
    THERMAL=6


class DatasetManager:
    def __init__(self):
        self.datasets = {}
    
    def _convert_keys_to_int_tuples(self, data_dict):
        converted = {}
        for k, v in data_dict.items():
            try:
                converted[int(k)] = v  # ✅ Preserve v as-is (a dict)
            except ValueError:
                converted[k] = v
        return converted
    
    def load_all(self):
        for name in DATASETS_CONFIG:
            self.load(name)

    def load(self, dataset_name):
        path, root_key = DATASETS_CONFIG[dataset_name]
        if not os.path.exists(path):
            print(f"[ERROR] {dataset_name}: file not found at {path}")
            self.datasets[dataset_name] = {}
            return

        try:
            with open(path, 'r') as f:
                full_data = json.load(f)
            data = full_data.get(root_key, {})
            if not data:
                print(f"[WARNING] Root key '{root_key}' missing in {path}")
    
            # Automatically convert string keys to int for specific datasets
            if dataset_name == "fov_data":
                if "static_fov" in data:
                    data["static_fov"] = self._convert_keys_to_int_tuples(data["static_fov"])
                if "pid_fov" in data:
                    data["pid_fov"] = self._convert_keys_to_int_tuples(data["pid_fov"])

            elif dataset_name == "pid_data":
                data = self._convert_keys_to_int_tuples(data)

            self.datasets[dataset_name] = data
        except json.JSONDecodeError:
            print(f"[ERROR] {dataset_name}: JSON decode error in {path}")
            self.datasets[dataset_name] = {}


    def get(self, dataset_name, zoom_level=None, subkey=None):
        root = self.datasets.get(dataset_name, {})
        if subkey:
            root = root.get(subkey, {})
        if zoom_level is None:
            return root
        # allow int or str keys
        return root.get(str(zoom_level), root.get(int(zoom_level)))


    def update_zoom(self, dataset_name, zoom_level, data, subkey=None):
        zoom_key = str(zoom_level)
        if dataset_name not in DATASETS_CONFIG:
            print(f"[ERROR] Unknown dataset: {dataset_name}")
            return

        path, root_key = DATASETS_CONFIG[dataset_name]

        # Load existing file content
        try:
            with open(path, 'r') as f:
                file_data = json.load(f)
        except Exception as e:
            print(f"[ERROR] Reading {path} failed: {e}")
            return

        # Ensure root key exists
        if root_key not in file_data:
            file_data[root_key] = {}

        # Navigate to subkey if needed
        if subkey:
            if subkey not in file_data[root_key]:
                file_data[root_key][subkey] = {}
            file_data[root_key][subkey][zoom_key] = data
        else:
            file_data[root_key][zoom_key] = data

        # Write back to JSON
        try:
            with open(path, 'w') as f:
                json.dump(file_data, f, indent=2)
            print(f"[INFO] {dataset_name} zoom {zoom_key} updated in {path}")
        except Exception as e:
            print(f"[ERROR] Writing to {path} failed: {e}")
            return

        # Update in-memory
        if subkey:
            self.datasets.setdefault(dataset_name, {}).setdefault(subkey, {})[int(zoom_key)] = data
        else:
            self.datasets.setdefault(dataset_name, {})[int(zoom_key)] = data


class SharedState:
    def __init__(self, mode="Semi-Auto", show_gui=True):
        self.manager = mp.Manager()
        self.lock = self.manager.Lock()
        self.stop_flag = Value(c_int, 0)
       
        self.mode = Value(c_int,OpMode.SEMI_AUTO)
        self.show_gui = show_gui
        self.zoom_level = Value(c_int, 1)
        self.last_time = time.time()
        self.detected_objects=self.manager.list()
        self.top_target_locked=Value(c_bool,False)
        self.static_position_sent=Value(c_bool,False)
        self.PID=Value(c_bool,False)
        self.zoom_set=Value(c_bool,False)
        self.digital_zoom_enabled = Value(c_bool,False)
        self.calib_set=Value(c_bool,False)
        self.stop_motor_flag=Value(c_bool,False)
        self.hold_start_time = Value(c_float,0.0)
        self.pres=self.manager.list()
        
        # # Dataset loading for depth the json file
        self.dataset_manager = DatasetManager()
        self.dataset_manager.load_all()
        self.target_type = Globals.TARGET_TYPE  
        self.datasets = self.dataset_manager.datasets

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

        self.current_yaw_deg = Value(c_float,0.0)
        self.current_pitch_deg = Value(c_float,0.0)
        self.pitch_set=Value(c_bool,False)
        self.yaw_set=Value(c_bool,False)
        self.motor_controller = MotorController(self)

        # Camera Snapshot and Record action
        self.save_dir = "logs" 
        self.record_start_flag = Value(c_bool,False)
        self.record_stop_flag = Value(c_bool,False)

        # Motor Velocity and acceleration
        self.current_pitch_velocity_limit = Value(c_float, 50.0)
        self.current_pitch_acceleration_limit = Value(c_float, 50.0)
        self.current_yaw_velocity_limit = Value(c_float, 50.0)
        self.current_yaw_acceleration_limit = Value(c_float, 50.0)
        
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
        self.pid_factor = Value(c_float,2.0)

        # Use person model flag
        self.face_model = Value(c_bool, True)
        # Focal length initialization
        self.focal_length = Value(c_float, 0.0)
        # self.publisher = PublishMQ()
        # Solenoid controller
        self.magnet_hold_time = Value(c_float,0.3) 
        self.short_burst_time = Value(c_float,0.5)
        self.long_burst_time = Value(c_float,1.0)
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
        self.depth_factor = Value(c_float,1.0)

        # PID enable flag
        self.enable_pid = Value(c_bool,True)

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
        self.auto_target_hold_time = Value(c_float,0.2)
        self.acc_percentage = Value(c_int,5)

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
            # print(f'Mode = {self.shooting_mode} sb {self.short_burst_time.value} and {self.long_burst_time.value}')
            self.fire_status.value = True
            if self.shooting_mode == "Short-Burst":
                self.burst_time.value = self.short_burst_time.value 
                self.trigger.solenoid_on()
                time.sleep(self.short_burst_time.value)
                self.trigger.solenoid_off()
                MQ_PUB.publish("event","trigger_fired",True)
            elif self.shooting_mode == "Burst":
                # self.fire_status.value = True
                self.burst_time.value = self.long_burst_time.value
                self.trigger.solenoid_on()
                time.sleep(self.long_burst_time.value)
                self.trigger.solenoid_off()
                # self.publisher.publish('',{"trigger_fired": True})
                MQ_PUB.publish("event","trigger_fired",True)
            else:
                self.burst_time.value = 0.0
                print(f'In safety cannot trigger')
        except Exception as e:
            logger.error(f"fire_solenoid error: {e}", exc_info=True)
        finally:
            self.fire_status.value = False
            self.solenoid_triggered.value = True
            # print(f'Burst time = {self.burst_time.value}')

            
    def mapping(self,Min1,Max1,Min2,Max2,cmd):
        Variable1 = numpy.array([Min1,Max1])
        Variable2 = numpy.array([Min2,Max2])	
        MappedValue = interp1d(Variable1,Variable2,kind='linear')
        return float(MappedValue(cmd))

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
    
    # def get_calibration_data(self,zoom_level):
    #     print(f'./camera_params/{zoom_level}x.npz')
    #     return self.calib_data.get(zoom_level, None)
    
    def get_calibration_data(self, zoom_level):
        return (
            self.calib_data.get(zoom_level, None),
            self.calib_data_top.get(zoom_level, None)
        )

    def select_face(self,event, x, y, flags, param):
        # global target_id, detected_objects, top_target_locked, static_position_sent, pres, PID
        # print("Clicked")
        if event == cv2.EVENT_LBUTTONDOWN:
            for obj in self.detected_objects:
                x1 = int(obj["fx"] - obj["w"] / 2)
                y1 = int(obj["fy"] - obj["h"] / 2)
                x2 = int(obj["fx"] + obj["w"] / 2)
                y2 = int(obj["fy"] + obj["h"] / 2)

                if x1 <= x <= x2 and y1 <= y <= y2:
                    with self.lock:
                        self.target_id.value = obj["id"]
                        self.top_target_locked.value = False  # Reset top cam print flag
                        self.static_position_sent.value = False
                        self.PID.value = False
                        self.pres = [0, 0, 0, 0, 0]
                    logger.info(f"[SELECTED] ID: {self.target_id.value}, fx: {obj['fx']}, fy: {obj['fy']}")
                    return



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


def camera_reader(shm_static, shm_top, shared_state):

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
        MQ_PUB.publish('error','error',payload)
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
                payload = {"Camera_type":"Top_camera","message":"camera_disconnected"}
                MQ_PUB.publish('error','error',payload)
                shared_state.stop_flag.value = 1
            else:
                h, w = f0.shape[:2]
                if (w, h) != (1920, 1080):   # check resolution
                    f0 = cv2.resize(f0, (1920, 1080), interpolation=cv2.INTER_LINEAR)
            ret2, f2 = cam2.read()
            if not ret2:
                print("Failed to grab frame from cam1")
                payload = {"Camera_type":"Bottom_camera","message":"camera_disconnected"}
                MQ_PUB.publish('error','error',payload)
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
            MQ_PUB.publish('error','error',payload)
            shared_state.stop_flag.value = 1
        except Exception as ex:
            logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
    
    cam0.release()
    cam2.release()
    static_writer.close()
    top_writer.close()


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
    
def yolo_inference_loop(shm_static, shm_top, shared_state):
    mq_publisher=BaseMQ_New(queue=f"{SYS}_infrence.pub.q", connection_name=f"{SYS}_infrence-pub")
    reader = SharedFrame(name=shm_static)
    face_model = YOLO(Globals.FACE_MODEL,verbose=False).to("cuda")
    person_model = YOLO(Globals.PERSON_MODEL,verbose=False).to("cuda")
    fps_counter=FPSCounter()
    # cv2.namedWindow("Static Camera")
    # cv2.setMouseCallback("Static Camera", shared_state.select_face)
    want_detections = True
    gpu_in = cv2.cuda_GpuMat()
    # with shared_state.lock: torch.cuda.amp.autocast(dtype=torch.float16)
    while not shared_state.stop_flag.value:
        try:
            frame_static = reader.read()
            # resized = cv2.resize(frame_static, (640, 360))
            gpu_in.upload(frame_static)
            gpu_resized = cv2.cuda.resize(gpu_in, (640, 360))
            resized = gpu_resized.download()  # YOLO needs CPU numpy
            if shared_state.face_model.value:
                with torch.inference_mode():
                    results = face_model.track(source=resized, persist=True, device="cuda",classes=[0,80,81], conf=0.35, iou=0.5,max_det=10, tracker="trackers/bytetrack.yaml")
            else:
                with torch.inference_mode():
                    results = person_model.track(source=resized, persist=True, device="cuda",classes=[0,80,81], conf=0.35, iou=0.5,max_det=10, tracker="trackers/bytetrack.yaml")
                
            
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
                        color = (0, 0, 255) if obj["id"] == shared_state.target_id else (0, 255, 0)
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
    face_pid_model  = YOLO(Globals.FACE_MODEL,  verbose=False).to(device)
    person_pid_model= YOLO(Globals.PERSON_MODEL,verbose=False).to(device)
    pid_tracker="trackers/bytetrack.yaml"

    dummy = np.zeros((256, 256, 3), np.uint8)
    for model in (face_pid_model, person_pid_model):
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
    motor_controller  = shared_state.motor_controller
    fps_counter    = FPSCounter()

    # Initial motor stop
    yaw_state, pitch_state = await motor_controller.stop_motors()
    YawCurrentPos  = Globals.YAW_POS_CONST * yaw_state.values[moteus.Register.POSITION]
    PitchCurrentPos = Globals.PITCH_POS_CONST * pitch_state.values[moteus.Register.POSITION]

    p_time =time.time()
    
    tracking_start_time = time.time()
    seconds = 0
    firing = False
    zeroing = True
    
    logger.info(f'pid_control_loop proc started')
    while not shared_state.stop_flag.value:
        
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
                motor_task = asyncio.create_task(run_calibration(shared_state.motor_controller,W_FULL,H_FULL))
                rval=await motor_task
                # if rval == True:
                #     mq_publisher.publish("event","calib_status",{"calib_status":rval})
                shared_state.current_yaw_deg.value=0.0
                shared_state.current_pitch_deg.value=0.0
                print('Calibration Done')

            elif(shared_state.stop_motor_flag.value): # TBT
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
                await shared_state.motor_controller.set_pitch()
            elif(shared_state.yaw_set.value):
                shared_state.yaw_set.value=False
                # print(f'Yaw Received')
                YawCurrentPos = 420
                PitchCurrentPos = 420
                await shared_state.motor_controller.set_yaw()

            if(frame_top is None):
                continue
            fps_counter.update()
            t1=time.time()
            p_time =time.time()
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
                        
                        offset_x = (cx_point - static_barrel_delta_w) 
                        offset_y = (cy_point - static_barrel_delta_h)

                        YawPositionCmd,PitchPositionCmd = motor_controller.pos_cmd(W_HALF,H_HALF,*static_fov,offset_x,offset_y)

                        if round(YawPositionCmd,1) != round(YawCurrentPos,1) or round(PitchPositionCmd,1) != round(PitchCurrentPos,1) and shared_state.PID.value == False:
                            motor_task = asyncio.create_task(run_static_control(motor_controller,YawCurrentPos, PitchCurrentPos, YawPositionCmd, PitchPositionCmd, target['h'], shared_state.zoom_level.value))
                            # Await the result of the motor control task
                            YawCurrentPos,PitchCurrentPos  = await motor_task
                        if round(YawPositionCmd,1) == round(YawCurrentPos,1) and round(PitchPositionCmd,1) == round(PitchCurrentPos,1) and shared_state.PID.value == False:
                            with shared_state.lock:
                                shared_state.PID.value = True
                                shared_state.static_position_sent.value = True
                            p_time = time.time()                

                if shared_state.PID.value and shared_state.enable_pid.value:
                    lock_delta_x = int(target['w'] / 2) * shared_state.pid_factor.value
                    lock_delta_y = int(target['h'] / 2) * shared_state.pid_factor.value
                    crop, offset = shared_state.crop_center(frame_top, int(lock_delta_x), int(lock_delta_y), shared_state.barrel_offset_x.value,shared_state.barrel_offset_y.value)
                    pid_model = face_pid_model if shared_state.face_model.value else person_pid_model
                    # with torch.cuda.stream(pid_stream):
                    with torch.inference_mode(), torch.amp.autocast("cuda", dtype=torch.float16):
                        pid_result = pid_model.track(
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
                            # print(f"[PID] Tracking ID: {pid_id}, Total results: {len(pid_result)}")

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

                            if not shared_state.top_target_locked.value and shared_state.static_position_sent.value == True:
                                error_x = zero_in_region_x - top_barrel_delta_w
                                error_y = zero_in_region_y - top_barrel_delta_h

                                MCmdx,MCmdy = motor_controller.pos_cmd(W_HALF,H_HALF,*pid_fov,error_x,error_y)

                                # print(f"[PID INPUT] X error: {error_x}, Y error: {error_y} {height} === {width}")
                                c_time = time.time()
                                delta_t = c_time - p_time
                                if(delta_t>0.5):
                                    delta_t=0.2
                                # print(f'delta_t={delta_t}')
                                pid_task = asyncio.create_task(run_pid_control(motor_controller,YawCurrentPos,PitchCurrentPos, MCmdx, MCmdy,delta_t, shared_state.pres[2], shared_state.pres[3], shared_state.zoom_level.value,current_h))
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
                            

                            offset_x = (cx_point - static_barrel_delta_w) 
                            offset_y = (cy_point - static_barrel_delta_h)
                            
                            YawPositionCmd,PitchPositionCmd = motor_controller.pos_cmd(W_HALF,H_HALF,*static_fov,offset_x,offset_y)
                            if round(YawPositionCmd,1) != round(YawCurrentPos,1) or round(PitchPositionCmd,1) != round(PitchCurrentPos,1) and shared_state.PID.value == False:
                                motor_task = asyncio.create_task(run_static_control(motor_controller,YawCurrentPos, PitchCurrentPos, YawPositionCmd, PitchPositionCmd, target_face['h'], shared_state.zoom_level.value))
                                # Await the result of the motor control task
                                YawCurrentPos,PitchCurrentPos  = await motor_task
                            if round(YawPositionCmd,1) == round(YawCurrentPos,1) and round(PitchPositionCmd,1) == round(PitchCurrentPos,1) and shared_state.PID.value == False:
                                with shared_state.lock:
                                    shared_state.PID.value = True
                                    shared_state.static_position_sent.value = True
                                p_time = time.time()
                                firing = True
                                zeroing = True

                    if shared_state.PID.value and shared_state.enable_pid.value:
                        # Use consistent rectangular crop dimensions
                        lock_delta_x = int(target_face['w'] / 2) * shared_state.pid_factor.value
                        lock_delta_y = int(target_face['h'] / 2) * shared_state.pid_factor.value
                        crop, offset = shared_state.crop_center(frame_top, int(lock_delta_x), int(lock_delta_y), shared_state.barrel_offset_x.value,shared_state.barrel_offset_y.value)
                        pid_model = face_pid_model if shared_state.face_model.value else person_pid_model
                        # with torch.cuda.stream(pid_stream):
                        with torch.inference_mode(), torch.amp.autocast("cuda", dtype=torch.float16):
                            pid_result = pid_model.track(
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

                                if not shared_state.top_target_locked.value and shared_state.static_position_sent.value == True:
                                    error_x = zero_in_region_x - top_barrel_delta_w
                                    error_y = zero_in_region_y - top_barrel_delta_h
                                    # print(f'Error = {error_x} and error {error_y}')
                            
                                    MCmdx,MCmdy = motor_controller.pos_cmd(W_HALF,H_HALF,*pid_fov,error_x,error_y)

                                    c_time = time.time()
                                    delta_t = c_time - p_time
                                    if(delta_t>0.5):
                                        delta_t=0.2
                                    if zeroing:
                                        pid_task = asyncio.create_task(run_pid_control(motor_controller,YawCurrentPos,PitchCurrentPos, MCmdx, MCmdy,delta_t, shared_state.pres[2], shared_state.pres[3], shared_state.zoom_level.value,current_h))
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
                                                # await motor_controller.hold_motors()
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
                if shared_state.manual_x.value > 0 and shared_state.manual_y.value > 0:
                    # print(f'Manual point recieved')
                    static_barrel_delta_h = H_HALF  + shared_state.barrel_offset_y.value
                    static_barrel_delta_w = W_HALF + shared_state.barrel_offset_x.value
                    # Use head position for tracking
                    offset_x = shared_state.manual_x.value - static_barrel_delta_w 
                    offset_y = shared_state.manual_y.value - static_barrel_delta_h  # This is already the head position from our earlier modification

                    YawPositionCmd,PitchPositionCmd = motor_controller.pos_cmd(W_HALF,H_HALF,*static_fov,offset_x,offset_y)
                    yaw_percentage,pitch_percentage = compute_hit_accuracy_from_commanded(YawPositionCmd, YawCurrentPos, PitchPositionCmd, PitchCurrentPos)
                    print(f'fx = {offset_x} and fy = {offset_y}')
                    print(f'yaw_cmd = {-1 * YawPositionCmd} pitch_cmd = {-1 * PitchPositionCmd}')
                    # Publishing message to GUI
                    payload = {"pos_err_x": 100 - yaw_percentage, "pos_err_y": 100 - pitch_percentage}
                    mq_publisher.publish("accuracy","accuracy",payload)
                    if round(YawPositionCmd,2) != round(YawCurrentPos,2) or round(PitchPositionCmd,2) != round(PitchCurrentPos,2) and shared_state.PID.value == False:
                        motor_task = asyncio.create_task(run_static_control(motor_controller,YawCurrentPos, PitchCurrentPos, YawPositionCmd, PitchPositionCmd,None,None))
                        # Await the result of the motor control task
                        YawCurrentPos,PitchCurrentPos  = await motor_task
                

            elif shared_state.mode.value==OpMode.DEMO:
                # print(f'Demo mode turned on ')
                now = time.time()
                SinWave = 20*numpy.sin(now)
                PitchWave = 20*numpy.cos(now)
                YawPositionCmd = shared_state.mapping(-Globals.REVOLUTION,Globals.REVOLUTION,-Globals.YAW_REDUCTION,Globals.YAW_REDUCTION,SinWave)
                PitchPositionCmd = shared_state.mapping(-Globals.REVOLUTION,Globals.REVOLUTION,-Globals.PITCH_REDUCTION,Globals.PITCH_REDUCTION,PitchWave)
                CameraPositionCmd = shared_state.mapping(-Globals.REVOLUTION,Globals.REVOLUTION,-Globals.CAM_REDUCTION,Globals.CAM_REDUCTION,SinWave)
                motor_task = asyncio.create_task(run_scan_mode(motor_controller,YawPositionCmd, PitchPositionCmd,CameraPositionCmd))
                await motor_task

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

def calibrate(shared_state):
    calib_task_future = asyncio.run_coroutine_threadsafe(shared_state.motor_controller.Calibrate(), loop)
    calib_task_result = calib_task_future.result()

class MessageController:

    def __init__(self,shared_state, shared_top, shared_static):
        self.shared_state=shared_state
        self.shared_top = shared_top
        self.shared_static = shared_static
        # self.routing_keys = [f"camera.{loc.value}.{action.value}"
        #                      for loc in CameraLocation for action in CameraAction]
        self.routing_keys=["action_key"]
        # New Rabbit mq variables
        self.mq_sub = BaseMQ_New(queue="rakshak_sub.q", connection_name="rakshak-sub")
        # Client listens to calibration messages coming from backend
        self.mq_sub.bind(["rakshak.gui_1.*"])
    
    def on_client_msg(self, rk, msg):
        logger.info(f"[CLIENT RECEIVED] rk={rk} → {msg}")
        try:
            event = msg['msg_type']
            payload = msg['payload']
            if event == "operating_mode_control":
                if(payload == "Semi-Auto"):
                    self.shared_state.manual_x.value = 0 
                    self.shared_state.manual_y.value = 0
                    self.shared_state.target_id.value = -1
                    self.shared_state.PID.value = False
                    self.shared_state.static_position_sent.value = False
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
                    self.shared_state.mode.value=OpMode.MANUAL
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
                msg = {"pos_err_x": 0, "pos_err_y": 0}
                MQ_PUB.publish("accuracy","accuracy",msg)
                self.shared_state.top_target_locked.value = False  # Reset top cam print flag
                self.shared_state.static_position_sent.value = False
                self.shared_state.PID.value = False
                self.shared_state.target_id.value=payload
                self.shared_state.pres = [0, 0, 0, 0, 0]
            elif event == "sendfxfy_action":
                    msg = {"pos_err_x": 0, "pos_err_y": 0}
                    MQ_PUB.publish("accuracy","accuracy",msg)
                    self.shared_state.manual_x.value = payload["manual_center_x"]
                    self.shared_state.manual_y.value = payload["manual_center_y"]
            elif event == "calibration_action":
                self.shared_state.calib_set.value=True
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
                self.shared_state.stop_motor_flag.value=True
            elif event=="set_pitch_action":
                self.shared_state.manual_x.value = 0 
                self.shared_state.manual_y.value = 0
                self.shared_state.current_pitch_deg.value =payload["pitch_value"]
                self.shared_state.current_pitch_velocity_limit.value = payload["Pitch_Velocity"]
                self.shared_state.current_pitch_acceleration_limit.value = payload["Pitch_Acceleration"]
                self.shared_state.pitch_set.value=True
            elif event =="set_yaw_action":
                self.shared_state.manual_x.value = 0 
                self.shared_state.manual_y.value = 0
                self.shared_state.current_yaw_deg.value = payload["yaw_value"]
                self.shared_state.current_yaw_velocity_limit.value = payload["Yaw_Velocity"]
                self.shared_state.current_yaw_acceleration_limit.value = payload["Yaw_Acceleration"]
                self.shared_state.yaw_set.value=True
            elif event == "trigger_snapshot_action":
                take_snapshot(self.shared_state,self.shared_top,self.shared_static)
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
                        
        except Exception as ex:
            logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
            error_str = traceback.format_exc()
            if(error_str and len(error_str)>0):
                logger.error(f'Exception Trace:{error_str}')

    def run(self):
        self.mq_sub.subscribe(self.on_client_msg)

async def main(shared_static_name, shared_top_name, shared_state):
    # pid_task=asyncio.create_task(pid_control_loop(shared_static_name, shared_top_name, shared_state))
    health_task = asyncio.create_task(shared_state.motor_controller.publish_health())
    health_result=await health_task
    # await pid_task
    # await pid_control_loop(shared_static_name, shared_top_name, shared_state)

loop = asyncio.new_event_loop()

def start_event_loop():
    asyncio.set_event_loop(loop)
    loop.run_forever()

# Create the background thread to run the event loop
threading.Thread(target=start_event_loop, daemon=True).start()

async def pid_control_task(shared_static_name, shared_top_name, shared_state):
    await pid_control_loop(shared_static_name, shared_top_name, shared_state)

async def motor_health_task(shared_state):
    await shared_state.motor_controller.publish_health()

def video_recorder_process(shared_top_name: str, shared_static_name: str, shared_state) -> None:
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

def take_snapshot(shared_state, shared_top, shared_static):
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
        MQ_PUB.publish('error','error',payload)
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
    stop_flag = Value(c_int, 0)
    shared_static = SharedFrame(shape=(h,w, 3))
    shared_top = SharedFrame(shape=(h,w, 3))
    shared_top_updated= SharedFrame(shape=(h,w, 3)) # Creating New Memory Frame
    shared_state = SharedState()
   
    cam_proc = Process(target=camera_reader, args=(shared_static.name, shared_top.name, shared_state))
    det_proc = Process(target=yolo_inference_loop, args=(shared_static.name, shared_top.name, shared_state))
    http_proc = Process(target=run_https_server, args=(shared_static.name, shared_top_updated.name, shared_state))
    msg_controller=MessageController(shared_state, shared_top, shared_static)
    msg_thread=threading.Thread(target=msg_controller.run,daemon=True)
    # Start sensor logging process
    sensor_proc = Process(target=sensor_logging_process, args=(shared_state,), daemon=True)
    recorder_proc = Process(target=video_recorder_process,args=(shared_top.name, shared_static.name, shared_state),daemon=True)

    cam_proc.start()
    det_proc.start()
    http_proc.start()
    msg_thread.start()
    sensor_proc.start()  # Start sensor logging
    recorder_proc.start()

    asyncio.run_coroutine_threadsafe(pid_control_loop(shared_static.name, shared_top.name, shared_top_updated.name,shared_state), loop)
    asyncio.run_coroutine_threadsafe(shared_state.motor_controller.publish_health(), loop)

    cam_proc.join()
    det_proc.join()
    http_proc.join()
    sensor_proc.join()
    recorder_proc.join()

    shared_static.close()
    shared_static.unlink()
    shared_top.close()
    shared_top.unlink()
    shared_top_updated.close()
    shared_top_updated.unlink()

    sys.exit(0)

if __name__=='__main__':
    launch()
