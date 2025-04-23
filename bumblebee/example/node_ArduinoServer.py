#!/usr/bin/env python3

import rospy
import rosnode

#/opt/ros/noetic/share/std_srvs/srv/Empty.srv , SetBool.srv, Trigger.srv
from std_srvs.srv import *

#/opt/ros/noetic/share/rospy_tutorials/srv/AddTwoInts.srv
# input : a,b (정수) , return sum (정수)
from rospy_tutorials.srv import *

#/opt/ros/noetic/share/turtlesim/srv/Kill.srv
# input : string, return : 없음
from turtlesim.srv import *
from tta_blb.srv import *
#import srvSingleParam, srvSingleParamResponse

import serial
import os
import subprocess
from varname import *
import termios, sys
import time
from Util import *
from UtilGPIO import *
from SPG_Keys import *
#from ServoNano import *
from varname import *
from std_msgs.msg import Header
from std_msgs.msg import String

publish_topic_name = 'KEEP_ARD'

param_ARD_show = False

portArd_M = '/dev/ttCARD_M' # 아두이노 시리얼 포트
#portArd_M = '/dev/ttyACM0' # 아두이노 메가 정품 시리얼 포트
serArd_M = None
listPortArd = [portArd_M]
listArdInstance = [serArd_M]
baudArd = 115200 # 아두이노 통신속도
lineArdData = []
pub_ka = rospy.Publisher(publish_topic_name, String, queue_size=1)

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
    bOK = False
    while bOK == False:
        try:
            path = os.path.realpath(portArd_M)
            prtMsg(f'Prevent reboot ard port... {portArd_M},{path}')
            if None != portArd_M:
                with open(path) as f:
                    attrs = termios.tcgetattr(f)
                    attrs[2] = attrs[2] & ~termios.HUPCL
                    termios.tcsetattr(f, termios.TCSAFLUSH, attrs)

            prtMsg(f'Initialing ard port... {portArd_M},{path}')
            serArd_M = serial.Serial(portArd_M, baudArd, timeout=None)
            prtMsg(f'Success Open port! {serArd_M}')
            bOK = True
        except Exception as e:
            prtMsg(f'Failed {serArd_M} to Open port {portArd_M} - {e}')
            time.sleep(1)
    time.sleep(1)

def SendArd(strData):
    global serArd_M
    msg = f'{strData} from {sys._getframe(1).f_code.co_name}-{sys._getframe(0).f_code.co_name}'

    if serArd_M == None:
        prtMsg('=> No serial port : ' + msg)
        return
    else:
        prtMsg('=> ' + msg)
    serArd_M.write(bytes(strData, encoding='ascii'))
    serArd_M.write(bytearray([0x0D]))
    serArd_M.write(bytearray([0x0A]))
    return

def SendKeepAlive(sendbuf):
    global pub_ka
    if pub_ka is not None:
        pub_ka.publish(sendbuf)
        prtMsg(sendbuf)

def parsing_data(data):
    tmp = ''.join(data) #String 으로
    try:
      #print(getCurrentTime()+' ' + tmp)
      SendKeepAlive(tmp) #이 부분 TRY CATCH 루틴 만들것.
    except Exception as e:
      prtMsg(e)

def poll_arduino():
      global serArd_M
      global param_ARD_show
      global lineArdData

      rate = rospy.Rate(1000) #run 1000 times per second

      while not rospy.is_shutdown():
        # if rospy.has_param(nameof(param_ARD_show)):
        #     param_ARD_show = rospy.get_param(nameof(param_ARD_show))
        # else:
        #     rospy.set_param(nameof(param_ARD_show), param_ARD_show)
        try:
            if serArd_M.readable():
                for c in serArd_M.read():
                    if c == 13: #라인의 끝을 만나면..
                        #데이터 처리 함수로 호출
                        parsing_data(lineArdData)
                        #line 변수 초기화
                        del lineArdData[:]
                    elif c == 10: #라인의 끝을 만나면.. KEEPALIVE 메세지를 보낸다.
                        SendArd('K')
                    else:
                        lineArdData.append(chr(c))
                        #print(chr(c))
        except Exception as e:
            print(traceback.format_exc())
            #prtMsg(e)
            prtMsg(serArd_M)

        rate.sleep()


class DataRecorder():
    def __init__(self):
        self.CMD_service = rospy.Service(ServiceList.CMDARD.value, Kill, self.SendCMD)
        self.CHARGE_ENABLE_service = rospy.Service(ServiceList.NEST_CHARGE_ENABLE.value, SetBool, self.CHARGE_SET)
        self.USB_ENABLE_service = rospy.Service(ServiceList.NEST_USB_ENABLE.value, SetBool, self.USB_SET)
        self.BREAK_ENABLE_service = rospy.Service(ServiceList.NEST_BREAK_ENABLE.value, SetBool, self.BV_SET)
        self.GPIO_service = rospy.Service(ServiceList.NEST_GPO_SET.value, utilboxKeyVal, self.GPO_SET)

        rospy.loginfo('Nest Arduino Started')

    def SendCMD(self, req):
      msg = req.name
      SendArd(msg)
      resp = KillResponse()
      return resp

    def GPO_SET(self, req):
        reqNumber = req.a
        reqStatus = req.b
        sendStr = f'D:{reqNumber},{reqStatus}'
        SendArd(sendStr)
        resp = AddTwoIntsResponse()
        resp.sum = 0
        return resp

    def CHARGE_SET(self, req):
        reqbool = req.data
        if reqbool is True:
          SendArd('CHARGE:0')
        else:
          SendArd('CHARGE:1')
        return SetBoolResponse(True, f'Charging:{reqbool}')

    def USB_SET(self, req):
        reqbool = req.data
        if reqbool is True:
          SendArd('USB:0')
        else:
          SendArd('USB:1')
        return SetBoolResponse(True, f'USB HUB:{reqbool}')

    def BV_SET(self, req):
        reqbool = req.data
        if reqbool is True:
          SendArd('BV:0')
        else:
          SendArd('BV:1')
        return SetBoolResponse(True, f'BREAK:{reqbool}')

if __name__ == "__main__":
    rospy.init_node('node_ARD', anonymous = True)
    DataRecorder()
    open_ard()
    poll_arduino()
    rospy.spin()
