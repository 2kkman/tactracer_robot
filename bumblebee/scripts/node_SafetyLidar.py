import cmd
import numpy as np
import rospy
import threading
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String, Header
from sensor_msgs.msg import PointCloud2, PointField, PointCloud, LaserScan

from tta_blb.srv import utilboxData, utilboxDataResponse
from MotorControl import MOTOR_CONTROL_CMD, MOTOR_DRIVER
from UtilBLB import *

class SafetyLidar:
    def __init__(self):
        self.logdisplay = False
        self.data_received = False
        self.prev_avg_distance = None
        self.object_detected = False
        
        self.motor_finish = False
        self.recovery_flag = False
        self.mbid = ModbusID.MOTOR_H.value
        self.warning_area = 2.4
        self.danger_area = 1.0
        self.offset = 10000
        
        self.cmd_ = None
        self.cmd_position = None
        self.cmd_velocity = None
        self.cmd_acceleration = None
        self.cmd_deceleration = None
        self.cur_position = None
        
        self.prev_recvDataMap = {}
        
        self.safety_lidar_service_ = rospy.Service('/BLB_SAFETY/safeToggle', utilboxData, self.safetyToggle)
        
        self.publisher_ = rospy.Publisher(TopicName.CMD_DEVICE.name, String, queue_size=10)
         
        self.subscriber_device = rospy.Subscriber(TopicName.CMD_DEVICE.name, String, self.callbackDevice) 
        self.subscriber_detect = rospy.Subscriber("/detect_3D", PointCloud2, self.callback)
        self.subscriber_state = rospy.Subscriber(f'{TopicName.MB_.name}{self.mbid}', String, self.callbackStates)
        
        self.timer_ = rospy.Timer(rospy.Duration(1), self.check_data_received)
        
        
    def safetyToggle (self, req):
        try:
            if req.message.lower() == 'start':
                rospy.logdebug(f'Got request: {req.message}')

                self.subscriber_ = rospy.Subscriber("/detect_3D", PointCloud2, self.callback)
                rospy.loginfo("Starting PointCloud2 subscription.")
                self.logdisplay = True
                return utilboxDataResponse(True, "PointCloud2 subscription started.")

            elif req.message.lower() == 'stop':
                rospy.loginfo("Stopping PointCloud2 subscription.")
                self.logdisplay = False
                return utilboxDataResponse(False, "PointCloud2 subscription stopped.")
            
            else:
                return utilboxDataResponse(False, "Invalid command or action already in desired state.")
         
        except rospy.ServiceException as e:
            rospy.logerr(f'Service call failed: {e}')
                 
        
    def callbackStates(self, data):
        recvDataMap = getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
        self.cmd_finish = recvDataMap.get(MonitoringField.ST_CMD_FINISH.name)
        self.cur_position = recvDataMap.get(MonitoringField.CUR_POS.name)
        
        if self.cmd_finish is None:
            return
        
        self.motor_finish = True if self.cmd_finish == f'{ServoParam.MOVE_TYPE_ABS.value}' else False
        if self.motor_finish:
            rospy.loginfo_once("Object detection is finished.")
        
    def callbackDevice(self, data):
        recvDataMap = getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
        if recvDataMap is None:
            return
        
        mbid = int(recvDataMap.get(MotorWMOVEParams.MBID.name))
        if mbid != ModbusID.MOTOR_H.value:
            return
    
        self.prev_recvDataMap = recvDataMap
        if recvDataMap.get(MotorWMOVEParams.CMD.name) is not None:
            self.cmd_ = recvDataMap.get(MotorWMOVEParams.CMD.name)
        if recvDataMap.get(MotorWMOVEParams.POS.name) is not None :
            self.cmd_position = recvDataMap.get(MotorWMOVEParams.POS.name)
        if recvDataMap.get(MotorWMOVEParams.SPD.name) is not None :
            self.cmd_velocity = recvDataMap.get(MotorWMOVEParams.SPD.name)
        if recvDataMap.get(MotorWMOVEParams.ACC.name) is not None :
            self.cmd_acceleration = recvDataMap.get(MotorWMOVEParams.ACC.name)
        if recvDataMap.get(MotorWMOVEParams.DECC.name) is not None :
            self.cmd_deceleration = recvDataMap.get(MotorWMOVEParams.DECC.name) 
        
    def get_closest_point_distance(self, point_cloud):
        # 포인트 클라우드 데이터를 numpy 배열로 변환
        gen = pc2.read_points(point_cloud, skip_nans=True)
        points = np.array(list(gen))

        if len(points) == 0:
            return float('inf')  # 포인트 클라우드가 비어있는 경우

        # 가장 가까운 점의 거리 계산
        distances = np.linalg.norm(points[:, :3], axis=1)
        closest_point_distance = np.min(distances)
        return closest_point_distance

    def adjust_stop_position(self, cmd_position, offset=10000):
        """
        Adjusts the stop position based on the command position value.

        Parameters:
        - cmd_position: The command position value.
        - offset: The offset value. Default is 10000.
        """
        if cmd_position > 0:
            # cmd_position이 양수일 때는 위치에서 정지
            stop_position = cmd_position - offset
        elif cmd_position < 0:
            # cmd_position이 음수일 때는 위치에서 정지
            stop_position = cmd_position + offset
        else:
            # cmd_position이 0일 경우 기본 정지 위치 설정 (예: 현재 위치)
            stop_position = 0
        return stop_position
         
    def callback(self, data):
        """
        

        Args:
            data (_type_): _description_
        """
        motorctl = MOTOR_CONTROL_CMD()
        rospy.logdebug_once(f'Current MBID: {self.mbid}')
        
        distances = self.get_closest_point_distance(data)

        # 물체의 최소 거리가 위험 영역 침범시 경고 메시지 출력 및 비상정지
        if distances < self.danger_area:
            self.object_detected = True  # 객체 감지 상태 업데이트
            rospy.logdebug_throttle_identical(10, f'Object detected at Danger Area.{distances:.2f} meters. Stopping the motor.')
            current_cmd = getMotorDefaultString(self.mbid, MotorCmdField.WSTOP.name)
            recvDataMap = getDic_strArr(current_cmd, sDivFieldColon, sDivItemComma)
            if recvDataMap == self.prev_recvDataMap:
                return
            motorctl.MotorTopic(current_cmd)
            self.prev_recvDataMap = current_cmd
            
            
        # 물체의 최소거리가 경고영역 침범시 경고 메시지 출력 및 감속운행
        elif distances < self.warning_area and not self.object_detected:
            self.object_detected = True  # 객체 감지 상태 업데이트
            rospy.logdebug_throttle_identical(10, f'Object detected at Warning Area.{distances:.2f} meters.')
            if self.cmd_ == MotorCmdField.WMOVE.name:
                self.set_stop_position = self.adjust_stop_position(int(self.cmd_position), self.offset)
                # print(f'set_stop_position: {set_stop_position}')    
                current_cmd = getMotorMoveString(mbid = self.mbid,
                                                isMode = True,
                                                pos = self.set_stop_position if self.cmd_position is not None else 0,
                                                spd = 500 if self.cmd_velocity is not None else 0,
                                                acc = 1000 if self.cmd_acceleration is not None else 0,
                                                decc= 1000 if self.cmd_deceleration is not None else 0)
                recvDataMap = getDic_strArr(current_cmd, sDivFieldColon, sDivItemComma)

                if recvDataMap == self.prev_recvDataMap:
                    return
                motorctl.MotorTopic(current_cmd)
                self.prev_recvDataMap = recvDataMap 
                # print(f'prev_recvDataMap: {self.prev_recvDataMap}')
            
        # 감지되었던 물체가 영역을 벗어난 경우
        elif distances >= self.warning_area and self.object_detected:

            if self.cmd_position is None:
                return
            self.object_detected = False  # 객체 감지 상태 업데이트
            self.set_stop_position = self.adjust_stop_position(int(self.cmd_position), -self.offset)
            
            current_cmd = getMotorMoveString(mbid = self.mbid,
                                            isMode = True,
                                            pos = self.set_stop_position,
                                            spd = 1000,
                                            acc = 1000,
                                            decc= 1000) 
            recvDataMap = getDic_strArr(current_cmd, sDivFieldColon, sDivItemComma)

            if recvDataMap == self.prev_recvDataMap:
                return
            motorctl.MotorTopic(current_cmd)
            self.prev_recvDataMap = recvDataMap 
            rospy.logdebug(f'recvDataMap: {recvDataMap}')
            
            # motor_driver.SetSpeedTarget(mbid = self.mbid,
            #                             speed = 1000, 
            #                             acc = self.cmd_acceleration, 
            #                             decc =self.cmd_deceleration)
            rospy.loginfo_once("Recovery mode is activated.")
            rospy.logdebug_throttle_identical(10, f'Object detected at {distances:.2f} meters.')
                            
        if self.logdisplay:
            data_points = data.width * data.height
            rospy.logdebug_throttle_identical(10, f'Received a PointCloud2 message with {data_points} data points.')     

    # 데이터 수신 확인
    def check_data_received(self, event):
        # 10초 간격으로 데이터 수신 확인
        if not self.data_received:
            rospy.logdebug_throttle_identical(10, f'No PointCloud2 data received in the last 10 seconds.')
            
        self.data_received = False  # 플래그 리셋
        
if __name__=='__main__':
    rospy.init_node('SafetyLidar', anonymous=True, log_level=rospy.DEBUG)
    SafetyLidar()
    rospy.loginfo("Safe Toggle Service Ready.")
    rospy.spin()

