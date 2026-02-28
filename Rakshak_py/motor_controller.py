import moteus # type: ignore
from scipy.interpolate import interp1d
import numpy
from MQ_Base import BaseMQ_New
import Globals
import re
import math 
import inspect
import asyncio
import time 
from LoggerModule.logger import Logger

class GearReductionDeg:
	def __init__(self,reduction):
		self.scale = reduction/360
	def pos(self, deg):
		return self.scale * deg
	def vel(self, deg_per_sec):
		return self.scale * deg_per_sec
	def acc(self, deg_per_sec2):
		return self.scale * deg_per_sec2
    
class MotorController:
    def __init__(self,shared_state,system,logger):

        # self.YawMotor = moteus.Controller(2)
        # self.CamMotor = moteus.Controller(2)
        # self.PitchMotor = moteus.Controller(1)
        # self.mq_publisher_limit=PublishMQ()
        self.YawMotor = moteus.Controller(2)
        self.PitchMotor = moteus.Controller(1)

        self.SYS = system
        self.logger = logger
        self.mq_publisher_limit = BaseMQ_New(queue=f"{self.SYS}-Motor_controller.pub.q", connection_name=f"{self.SYS}-Motor_controller-pub")  # queue unused for pub but harmless
        self.shared_state=shared_state
        self.pid_dict = self.shared_state.datasets.get("pid_data", {})
        # self.load_fov_dict()
        self.manual_yaw_limit = self.shared_state.mapping(-Globals.REVOLUTION,Globals.REVOLUTION,-Globals.YAW_REDUCTION,Globals.YAW_REDUCTION,Globals.YAW_LIMIT)
        self.manual_pitch_limit = self.shared_state.mapping(-Globals.REVOLUTION,Globals.REVOLUTION,-Globals.PITCH_REDUCTION,Globals.PITCH_REDUCTION,Globals.PITCH_LIMIT)
        
        self.yaw_velocity_limit = self.shared_state.current_yaw_velocity_limit.value 
        self.yaw_accel_limit = self.shared_state.current_yaw_acceleration_limit.value
        self.pitch_velocity_limit = self.shared_state.current_pitch_velocity_limit.value
        self.pitch_accel_limit = self.shared_state.current_pitch_acceleration_limit.value
        self.logger = Logger(file_type="csv", compress=True, file_keyword="motor_logs")
        self.logger.headers("motor_id","timestamp_str","position","velocity","torque","voltage","temperature","fault","d_current","q_current","total_current")
        self.logger.start()
        self.first_time = True

        print(f"Manual yaw limit = {self.manual_yaw_limit} ------> {self.manual_pitch_limit}")
    
    async def probe_motors(self) -> bool:
        try:
            for motor in (self.YawMotor, self.PitchMotor):
                r = await asyncio.wait_for(
                    motor.custom_query({moteus.Register.POSITION: moteus.F32}),
                    timeout=0.5
                )
                if not r or not hasattr(r, "values"):
                    self.shared_state.motors_ready.value = False
                    return False

            self.shared_state.motors_ready.value = True
            return True

        except Exception as ex:
            self.shared_state.motors_ready.value = False
            self.logger.error(f"probe_motors: {ex}")
            return False
        
    def send_alert(self):
        payload = {"Motor_controller":"FD CAN error","message":"Failed to get motors"}
        self.mq_publisher_limit.publish('error','error',payload)
    
    async def hold_motors(self):
        # print(f'Holding motors')
        try:
            if not self.shared_state.motors_ready.value:
                return
            YawCmd = await self.YawMotor.set_position(
                            position=math.nan,
                            query=True)
            PitchCmd = await self.PitchMotor.set_position(
                        position=math.nan,
                        query=True)
        except Exception as ex:
           self.logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
    
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
            if not self.shared_state.motors_ready.value:
                self.send_alert()
                return None,None
            YawStates = await self.YawMotor.set_stop(query=True)
            PitchStates = await self.PitchMotor.set_stop(query=True)
            #camState = await self.CamMotor.set_stop(query=True)
            print(f"Inside Motor Stop function {YawStates.values[moteus.Register.POSITION]} --- {PitchStates.values[moteus.Register.POSITION]}")
            print("Pitch Fault = ",PitchStates.values[moteus.Register.FAULT])
            print("Yaw Fault = ",YawStates.values[moteus.Register.FAULT])
            return (YawStates,PitchStates)
        except Exception as ex:
            self.logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
    
    async def Calibrate(self,width,height):
        if not self.shared_state.motors_ready.value:
            self.send_alert()
            return
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
            await self.YawMotor.set_output_exact(position = Globals.YAW_POS_CONST * (yawPos))
            await self.PitchMotor.set_output_exact(position = Globals.PITCH_POS_CONST * (-(-pitchPos)))
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
                    self.logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
        return done
    
    def joystick_to_velocity(self,x):	
        return 0 if -0.25 <= abs(x)  <= 0.25 else x**2 * math.sin(x)

    async def velocity_control(self,twist, pitch):
        yaw_velocity = self.joystick_to_velocity(twist)
        pitch_velocity = self.joystick_to_velocity(-pitch)
        yawcmd_vel = self.yaw_velocity_limit * yaw_velocity
        pitchcmd_vel = self.pitch_velocity_limit * pitch_velocity
        # print(f'Yaw->{yaw_velocity} pitch -> {pitch_velocity}')
        yawState = await self.YawMotor.set_position(position = math.nan,
                                                    velocity = yawcmd_vel,
                                                    velocity_limit = self.yaw_velocity_limit,
                                                    accel_limit = self.yaw_accel_limit,)
        pitchState = await self.PitchMotor.set_position(position = math.nan,
                                                    velocity = pitchcmd_vel,
                                                    velocity_limit = self.pitch_velocity_limit,
                                                    accel_limit = self.pitch_accel_limit,)

    async def Scan_mode(self,yaw_pos_cmd,pitch_pos_cmd,cam_pos_cmd):
        # print(f'Scan mode controller,yaw = {yaw_pos_cmd}, pit = {pitch_pos_cmd} ,cam = {cam_pos_cmd}')
       # CamState = await self.CamMotor.set_position(position = cam_pos_cmd,
        #                        velocity_limit = 2.5/2,
         #                       accel_limit = 2.5/2,
          #                      query=True)
        if not self.shared_state.motors_ready.value:
            self.send_alert()
            return
        YawState = await self.YawMotor.set_position(position = yaw_pos_cmd,
                                    velocity_limit = self.yaw_velocity_limit,
                                    accel_limit = self.yaw_accel_limit,
                                    query=True)
        PitchState = await self.PitchMotor.set_position(position = pitch_pos_cmd, #20
                                velocity_limit = self.pitch_velocity_limit,
                                accel_limit = self.pitch_accel_limit,
                                query=True)

    async def set_yaw(self):
        try:
            if not self.shared_state.motors_ready.value:
                self.send_alert()
                return
            yaw = await self.YawMotor.custom_query({moteus.Register.POSITION: moteus.F32})
            position = yaw.values[moteus.Register.POSITION]
            YawPositionCmd = Globals.YAW_POS_CONST * (self.shared_state.mapping(-Globals.REVOLUTION,Globals.REVOLUTION,-Globals.YAW_REDUCTION,Globals.YAW_REDUCTION,self.shared_state.current_yaw_deg.value))
            if -self.manual_yaw_limit < position+YawPositionCmd < self.manual_yaw_limit:
                velocity_limit = self.yaw_velocity_limit
                accel_limit = self.yaw_accel_limit
                
                # print(f'requested pos = {self.shared_state.current_yaw_deg.value}')
                # print(f'Yaw Request={position + YawPositionCmd}')
                await self.YawMotor.set_position(position= position + YawPositionCmd, velocity_limit=velocity_limit, accel_limit=accel_limit, query=True)
                
            else:
                print("Reached to max yaw Calibration required",-self.manual_yaw_limit)
                # self.mq_publisher_limit.publish('',{"limit_reached":True})
                payload = {"limit_reached":True}
                self.mq_publisher_limit.publish("event","motor_limit",payload)
                # print(f'Yaw Request={self.shared_state.current_yaw_deg.value}')
        except Exception as ex:
            self.logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')

    async def set_pitch(self):
        try:
            if not self.shared_state.motors_ready.value:
                self.send_alert()
                return
            pitch = await self.PitchMotor.custom_query({moteus.Register.POSITION: moteus.F32})
            position = pitch.values[moteus.Register.POSITION]
            PitchPositionCmd = Globals.PITCH_POS_CONST * (self.shared_state.mapping(-Globals.REVOLUTION,Globals.REVOLUTION,-Globals.PITCH_REDUCTION,Globals.PITCH_REDUCTION,self.shared_state.current_pitch_deg.value))
            if -self.manual_pitch_limit < position+PitchPositionCmd < self.manual_pitch_limit:
                velocity_limit = self.pitch_velocity_limit
                accel_limit = self.pitch_accel_limit
                # print(f'Pitch Request={position+PitchPositionCmd}')
                # print(f'Pitch Velo = {velocity_limit}, {accel_limit}')
                await self.PitchMotor.set_position(position=position + PitchPositionCmd, velocity_limit=velocity_limit, accel_limit=accel_limit, query=True)
            else:
                print("Reached to max pitch calibration required")
                # self.mq_publisher_limit.publish('',{"limit_reached":True})
                payload = {"limit_reached":True}
                self.mq_publisher_limit.publish("event","motor_limit",payload)
        except Exception as ex:
            self.logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
    
    async def publish_health(self):
        # mq_publisher=PublishMQ()
        mq_publisher = BaseMQ_New(queue=f"{self.SYS}-Motor_health.pub.q", connection_name=f"{self.SYS}-Motor_health-pub")  # queue unused for pub but harmless
        while not self.shared_state.stop_flag.value:
            if not self.shared_state.motors_ready.value:
                # self.send_alert()
                await asyncio.sleep(0.5)
                continue
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
                        # moteus.Register.D_CURRENT: moteus.F32,
                        # moteus.Register.Q_CURRENT: moteus.F32,
                    })

                    if result is not None and hasattr(result, 'values'):
                        # timestamp_ms = time.time() * 1000
                        position = result.values[moteus.Register.POSITION]
                        velocity = result.values[moteus.Register.VELOCITY]
                        torque = result.values[moteus.Register.TORQUE]
                        voltage = result.values[moteus.Register.VOLTAGE]
                        temperature = result.values[moteus.Register.TEMPERATURE]
                        fault = result.values[moteus.Register.FAULT]
                        # d_current = result.values[moteus.Register.D_CURRENT]
                        # q_current = result.values[moteus.Register.Q_CURRENT]
                        # total_current = math.sqrt(d_current ** 2 + q_current ** 2)
                        
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
                            # "d_current": d_current,
                            # "q_current": q_current,
                            # "total_current": total_current
                        }
                        try:
                            self.logger.publish(data)
                        except Exception as ex:
                            print(f'errrrrrrrrrrr->{ex}')
                            self.logger.error(f'[Logger module err]{inspect.currentframe().f_code.co_name} : {ex}')
                        health_data.append(data)
                # print(f'data -< {health_data}')
                mq_publisher.publish("health", "health_data", health_data)
            except Exception as ex:
                self.logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
            await asyncio.sleep(0.1)


    async def StaticControl(self,YawCurrentPos,PitchCurrentPos,YawPositionCmd,PitchPositionCmd):
        # print(f'Inside static')
        try:
            if not self.shared_state.motors_ready.value:
                self.send_alert()
                return
            YawSetPosition = await self.YawMotor.set_position(
                                                        position= Globals.YAW_POS_CONST * YawPositionCmd, #math.nan,
                                                        # position=math.nan,
                                                        velocity_limit = self.yaw_velocity_limit,
                                                        accel_limit = self.yaw_accel_limit,
                                                        # kp_scale = Globals.KP_SCALE_YAW,
                                                        query=True)
            YawCurrentPos = Globals.YAW_POS_CONST * YawSetPosition.values[moteus.Register.POSITION]

            PitchSetPosition = await self.PitchMotor.set_position(
                                                            position= Globals.PITCH_POS_CONST * (-(-PitchPositionCmd)),#math.nan,
                                                            # position=math.nan,
                                                            velocity_limit = self.pitch_velocity_limit,
                                                            accel_limit = self.pitch_accel_limit,
                                                            # kp_scale = Globals.KP_SCALE_PITCH,
                                                            query=True)
            PitchCurrentPos = Globals.PITCH_POS_CONST * (-(-PitchSetPosition.values[moteus.Register.POSITION]))
            # print("STATICCCCCCCCCCCCCCCCCCCCCCCCCC")
            # print(f'--->Static current Yaw = {YawCurrentPos} Pitch = {PitchCurrentPos}')
            return YawCurrentPos, PitchCurrentPos
        except Exception as ex:
            self.logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')

    async def PIDControl(self,YawCurrentPos,PitchCurrentPos,error_x,error_y,delta_t,PrevIx,PrevIy,zoom,height):
        # print(f'--->pid current pos yaw {YawCurrentPos} and pitch, {PitchCurrentPos}')
        # P Controller x = 35.706691400602885           y = 21.0491446668607678
        # print(f'Error x = {error_x} and error_y {error_y}')
        try:
            if not self.shared_state.motors_ready.value:
                self.send_alert()
                return
            kix,kiy = self.shared_state.dataset_manager.get("pid_data", zoom)
            # kix,kiy = self.pid_dict[zoom] #0.8(present working gains)
            # print(f'kix,kiy = {kix} and {kiy}')
            # I Controller x
            # kix = 0
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
                # print(f'res->{abs(pitchcmd)} == {self.manual_pitch_limit} and {abs(yawcmd)} == {self.manual_yaw_limit}')
                # self.mq_publisher_limit.publish('',{"limit_reached":True})
                payload = {"limit_reached":True}
                self.mq_publisher_limit.publish("event","motor_limit",payload)
                yawcmd = math.nan
                pitchcmd = math.nan
                return None
            else:
                YawSetPosition = await self.YawMotor.set_position(
                                                            position =  Globals.YAW_POS_CONST * yawcmd, #math.nan,
                                                            velocity_limit = self.yaw_velocity_limit,
                                                            accel_limit = self.yaw_accel_limit,
                                                            # kp_scale = Globals.KP_SCALE_YAW,
                                                            query=True)
                YawCurrentPos = Globals.YAW_POS_CONST * YawSetPosition.values[moteus.Register.POSITION]

                PitchSetPosition = await self.PitchMotor.set_position(
                                                                position= Globals.PITCH_POS_CONST * pitchcmd, #-Py,#math.nan,
                                                                velocity_limit = self.pitch_velocity_limit,
                                                                accel_limit = self.pitch_accel_limit,
                                                                # kp_scale = Globals.KP_SCALE_PITCH,
                                                                query=True)
                PitchCurrentPos = Globals.PITCH_POS_CONST * (-(-PitchSetPosition.values[moteus.Register.POSITION]))
                # print(f'PIDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD')
                # print("Current POsition yaw = ", YawCurrentPos)
                # print("YAW Error = ", IX)
                # print("errorCorrectionYaw =", IX)
                return YawCurrentPos, PitchCurrentPos,Ix,Iy,IX,IY
        except Exception as ex:
            self.logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
        return None
    
    async def StaticDynamicControl(self,YawCurrentPos,PitchCurrentPos,YawPositionCmd,PitchPositionCmd,height,zoom):
        # print(f'Dynamic static control')
        try:
            if not self.shared_state.motors_ready.value:
                self.send_alert()
                return   
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
                                                        velocity_limit = Globals.YAW_VELOCITY_LIMIT,
                                                        accel_limit = Globals.YAW_ACCELERATION_LIMIT,
                                                        query=True)
            YawCurrentPos = YawSetPosition.values[moteus.Register.POSITION]

            PitchSetPosition = await self.PitchMotor.set_position(
                                                            position= -(-PitchPositionCmd+interp_output2(height)),#math.nan,
                                                            velocity_limit = Globals.PITCH_VELOCITY_LIMIT,
                                                            accel_limit = Globals.PITCH_ACCELERATION_LIMIT,
                                                            query=True)
            PitchCurrentPos = -(-PitchSetPosition.values[moteus.Register.POSITION])
            return YawCurrentPos-interp_output1(height), PitchCurrentPos+interp_output2(height)
        except Exception as ex:
            self.logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
    
    async def PIDDynamicControl(self,YawCurrentPos,PitchCurrentPos,error_x,error_y,delta_t,PrevIx,PrevIy,zoom,h):
        # print(f'Dynamic pid control')
        try:
            if not self.shared_state.motors_ready.value:
                self.send_alert()
                return
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
                                                            velocity_limit = Globals.YAW_VELOCITY_LIMIT,
                                                            accel_limit = Globals.YAW_ACCELERATION_LIMIT,
                                                            query=True)
                YawCurrentPos = YawSetPosition.values[moteus.Register.POSITION]

                PitchSetPosition = await self.PitchMotor.set_position(
                                                                position= pitchcmd, #-Py,#math.nan,
                                                                velocity_limit = Globals.PITCH_VELOCITY_LIMIT,
                                                                accel_limit = Globals.PITCH_ACCELERATION_LIMIT,
                                                                query=True)
                PitchCurrentPos = -(-PitchSetPosition.values[moteus.Register.POSITION])
                return YawCurrentPos, PitchCurrentPos,Ix,Iy,IX,IY
        except Exception as ex:
            self.logger.error(f'{inspect.currentframe().f_code.co_name} : {ex}')
