#!/usr/bin/env python3
import os
import sys
import rospy
import rosnode
import json
from std_srvs.srv import *
from rospy_tutorials.srv import *
from turtlesim.srv import *
from tta_blb.srv import *
from UtilBLB import *
from Util import *
import serial
import subprocess
from varname import *
import termios, sys
import time
from varname import *
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import Imu, Temperature, RelativeHumidity
import tf

publish_topic_name = TopicName.ARD_CARRIER.name
frame_id_Range = "map"
param_ARD_show = False
currentMachine = GetMachineStr()
portArd_M = "/dev/ttCARD"
serArd_M = None
listPortArd = [portArd_M]
listArdInstance = [serArd_M]
baudArd = 115200  # 아두이노 통신속도
lineArdData = []
pub_ka = rospy.Publisher(publish_topic_name, String, queue_size=1)
temp_pub = rospy.Publisher(TopicName.BLB_TRAY_TEMPC.name, Temperature, queue_size=10)
humid_pub = rospy.Publisher(TopicName.BLB_TRAY_HUMID.name, RelativeHumidity, queue_size=10)
seq = 0
seqAHT = 0
rebootCnt = 0

def prtMsg(sendbuf):
    # global param_ARD_show
    # if rospy.has_param(nameof(param_ARD_show)):
    #     param_ARD_show = rospy.get_param(nameof(param_ARD_show))
    # else:
    #     rospy.set_param(nameof(param_ARD_show), param_ARD_show)
    rospy.loginfo(sendbuf)

    # if param_ARD_show:
    #     rospy.loginfo(sendbuf)


def open_ard():
    global serArd_M
    global rebootCnt
    bOK = False
    while bOK == False:
        try:
            path = os.path.realpath(portArd_M)
            prtMsg(f"Prevent reboot ard port... {portArd_M},{path}")
            if None != portArd_M:
                with open(path) as f:
                    attrs = termios.tcgetattr(f)
                    attrs[2] = attrs[2] & ~termios.HUPCL
                    termios.tcsetattr(f, termios.TCSAFLUSH, attrs)

            prtMsg(f"Initialing ard port... {portArd_M},{path}")
            serArd_M = serial.Serial(portArd_M, baudArd, timeout=None)
            prtMsg(f"Success Open port! {serArd_M}")
            bOK = True
        except Exception as e:
            prtMsg(f"Failed {serArd_M} to Open port {portArd_M} - {e}")
            time.sleep(1)
    time.sleep(1)
    # SendArd("T:1")


def SendArd(strData):
    global serArd_M
    msg = f"{strData} from {sys._getframe(1).f_code.co_name}-{sys._getframe(0).f_code.co_name}"

    if serArd_M == None:
        prtMsg("=> No serial port : " + msg)
        return
    else:
        prtMsg("=> " + msg)
    serArd_M.write(bytes(strData, encoding="ascii"))
    serArd_M.write(bytearray([0x0D]))
    serArd_M.write(bytearray([0x0A]))
    return


def SendKeepAlive(sendbuf):
    global pub_ka
    if pub_ka is not None:
        pub_ka.publish(sendbuf)
        prtMsg(sendbuf)


def publish_ImuData(dicARD: dict):
    global pub_IMU
    global seq
    Imu_msg = Imu()
    Imu_msg.header = getROS_Header(frame_id_Range)
    GAV_SVN = dicARD.get(TRAY_ARD_Field.GAV_SVN.name, None)
    GLA_SVN = dicARD.get(TRAY_ARD_Field.GLA_SVN.name, None)
    GOR_SVN = dicARD.get(TRAY_ARD_Field.GOR_SVN.name, None)
    if GAV_SVN == None or GLA_SVN == None or GOR_SVN == None:
        return

    GAV_SVNarr = GAV_SVN.split(sDivItemComma)  # angular_velocity
    GLA_SVNarr = GLA_SVN.split(sDivItemComma)  # linear_acceleration
    GOR_SVNarr = GOR_SVN.split(sDivItemComma)  # orientation , roll,pitch,yaw

    roll = float(GOR_SVNarr[0])
    pitch = float(GOR_SVNarr[1])
    yaw = float(GOR_SVNarr[2])
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    # 쿼터니언 설정
    Imu_msg.orientation.x = quaternion[0]
    Imu_msg.orientation.y = quaternion[1]
    Imu_msg.orientation.z = quaternion[2]
    Imu_msg.orientation.w = quaternion[3]

    Imu_msg.linear_acceleration.x = float(GLA_SVNarr[0])
    Imu_msg.linear_acceleration.y = float(GLA_SVNarr[1])
    Imu_msg.linear_acceleration.z = float(GLA_SVNarr[2])

    Imu_msg.angular_velocity.x = float(GAV_SVNarr[0])
    Imu_msg.angular_velocity.y = float(GAV_SVNarr[1])
    Imu_msg.angular_velocity.z = float(GAV_SVNarr[2])
    seq += 1
    Imu_msg.header.stamp = rospy.Time.now()
    Imu_msg.header.seq = seq

    pub_IMU.publish(Imu_msg)


def publish_TempHumidData(dicARD: dict):
    global temp_pub
    global humid_pub
    global seqAHT
    temperature = dicARD.get(TRAY_ARD_Field.TEMPAHT_N.name, None)
    humidity = dicARD.get(TRAY_ARD_Field.HUMIDAHT_N.name, None)
    if temperature == None or humidity == None:
        return
    seqAHT += 1
    timeStamp = rospy.Time.now()
    temp_msg = Temperature()
    temp_msg.header.stamp = timeStamp
    temp_msg.header.frame_id = frame_id_Range
    temp_msg.header.seq = seqAHT
    temp_msg.temperature = float(temperature)
    temp_msg.variance = 0.0  # 필요에 따라 설정

    # 습도 메시지 생성 및 설정
    humid_msg = RelativeHumidity()
    humid_msg.header.stamp = timeStamp
    humid_msg.relative_humidity = float(humidity)
    humid_msg.header.frame_id = frame_id_Range
    humid_msg.header.seq = seqAHT
    humid_msg.variance = 0.0  # 필요에 따라 설정

    # 퍼블리시
    temp_pub.publish(temp_msg)
    humid_pub.publish(humid_msg)


def parsing_data(data):
    tmp = "".join(data).replace('\n','')  # String 으로
    try:
        dicTmp = getDic_strArr(tmp, sDivFieldColon, sDivEmart)
        publish_ImuData(dicTmp)
        # publish_TempHumidData(dicTmp)
        dicTmp[CALLBELL_FIELD.TIMESTAMP.name] = getDateTime().timestamp()
        data_out = json.dumps(dicTmp)
        # print(getCurrentTime()+' ' + tmp)
        SendKeepAlive(data_out)  # 이 부분 TRY CATCH 루틴 만들것.
    except Exception as e:
        prtMsg(e)


def poll_arduino():
    global serArd_M
    global param_ARD_show
    global lineArdData
    global rebootCnt
    rate = rospy.Rate(1000)  # run 1000 times per second
    while not rospy.is_shutdown():
        # if rospy.has_param(nameof(param_ARD_show)):
        #     param_ARD_show = rospy.get_param(nameof(param_ARD_show))
        # else:
        #     rospy.set_param(nameof(param_ARD_show), param_ARD_show)
        try:
            if serArd_M.readable():
                for c in serArd_M.read():
                    if c == 13:  # 라인의 끝을 만나면..
                        # 데이터 처리 함수로 호출
                        parsing_data(lineArdData)
                        # line 변수 초기화
                        del lineArdData[:]
                    # elif c == 10: #라인의 끝을 만나면.. KEEPALIVE 메세지를 보낸다.
                    #     SendArd('K')
                    else:
                        lineArdData.append(chr(c))
                        # print(chr(c))
                rebootCnt = 0
        except Exception as e:
            print(traceback.format_exc())
            # prtMsg(e)
            prtMsg(serArd_M)
            rebootCnt +=1
        if rebootCnt > 10:
            open_ard()
        rate.sleep()


class DataRecorder:
    def __init__(self):
        self.CMD_service = rospy.Service(
            ServiceBLB.CMDARD_QBI.value, Kill, self.SendCMD
        )
        # self.CHARGE_ENABLE_service = rospy.Service(ServiceBLB.CHARGE_ENABLE.value, SetBool, self.CHARGE_SET)
        # self.USB_ENABLE_service = rospy.Service(ServiceBLB.USB_ENABLE.value, SetBool, self.USB_SET)
        # self.BREAK_ENABLE_service = rospy.Service(ServiceBLB.NEST_BREAK_ENABLE.value, SetBool, self.BV_SET)
        # self.GPIO_service = rospy.Service(ServiceBLB.NEST_GPO_SET.value, utilboxKeyVal, self.GPO_SET)

        rospy.loginfo(f"{currentMachine} Arduino Started")

    def SendCMD(self, req):
        msg = req.name
        SendArd(msg)
        resp = KillResponse()
        return resp

    # def GPO_SET(self, req):
    #     reqNumber = req.a
    #     reqStatus = req.b
    #     sendStr = f'D:{reqNumber},{reqStatus}'
    #     SendArd(sendStr)
    #     resp = AddTwoIntsResponse()
    #     resp.sum = 0
    #     return resp

    # def CHARGE_SET(self, req):
    #     reqbool = req.data
    #     if reqbool:
    #       SendArd('CHARGE:0')
    #     else:
    #       SendArd('CHARGE:1')
    #     return SetBoolResponse(True, f'Charging:{reqbool}')
    # def BV_SET(self, req):
    #     reqbool = req.data
    #     if reqbool:
    #       SendArd('BV:0')
    #     else:
    #       SendArd('BV:1')
    #     return SetBoolResponse(True, f'BREAK:{reqbool}')


if __name__ == "__main__":
    rospy.init_node(f"{currentMachine}_ARD", anonymous=True)

    DataRecorder()
    open_ard()

    poll_arduino()
    rospy.spin()
