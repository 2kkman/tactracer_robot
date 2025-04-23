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
baudArd = 38400  # 아두이노 통신속도
lineArdData = []
pub_ka = rospy.Publisher(publish_topic_name, String, queue_size=1)
# temp_pub = rospy.Publisher(TopicName.BLB_TRAY_TEMPC.name, Temperature, queue_size=10)
# humid_pub = rospy.Publisher(TopicName.BLB_TRAY_HUMID.name, RelativeHumidity, queue_size=10)
seq = 0
seqAHT = 0
rebootCnt = 0

def prtMsg(sendbuf):
    rospy.loginfo(sendbuf)

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
            serArd_M.dtr = False
            serArd_M.rts = False
            prtMsg(f"Success Open port! {serArd_M}")
            bOK = True
        except Exception as e:
            prtMsg(f"Failed {serArd_M} to Open port {portArd_M} - {e}")
            time.sleep(1)
    time.sleep(1)

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

def parsing_data(data):
    tmp = "".join(data).replace('\n','')  # String 으로
    try:
        dicTmp = getDic_strArr(tmp, sDivFieldColon, sDivEmart)
        dicTmp[CALLBELL_FIELD.TIMESTAMP.name] = getDateTime().timestamp()
        data_out = json.dumps(dicTmp)
        SendKeepAlive(data_out)
    except Exception as e:
        prtMsg(e)

def poll_arduino():
    global serArd_M
    global param_ARD_show
    global lineArdData
    global rebootCnt
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        try:
            if serArd_M.readable():
                for c in serArd_M.read():
                    if c == 13:  # 라인의 끝을 만나면..
                        parsing_data(lineArdData)
                        del lineArdData[:]
                    else:
                        lineArdData.append(chr(c))
                rebootCnt = 0
        except Exception as e:
            print(traceback.format_exc())
            prtMsg(serArd_M)
            rebootCnt +=1
        if rebootCnt > 10:
            open_ard()
        rate.sleep()


class DataRecorder:
    def __init__(self):
        self.CMD_service = rospy.Service(ServiceBLB.CMDARD_QBI.value, Kill, self.SendCMD)
        rospy.loginfo(f"{currentMachine} Arduino Started")

    def SendCMD(self, req):
        msg = req.name
        SendArd(msg)
        #return TriggerResponse(success=True, message=msg)
        resp = KillResponse()
        return resp

if __name__ == "__main__":
    rospy.init_node(f"{currentMachine}_ARD", anonymous=False)
    DataRecorder()
    open_ard()
    poll_arduino()
    rospy.spin()