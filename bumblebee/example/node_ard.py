#! /usr/bin/env python3
from concurrent.futures import thread
from varname import *
import rosnode
import rospy
from bitstring import BitArray, ConstBitStream
from hanging_threads import start_monitoring
import os
import os.path
import datetime
import serial
import time
from time import sleep 
import termios, sys
import signal
from threading import Thread
import sys
import math
import traceback
import actionlib
from actionlib_tutorials.msg import SPGStrAction, SPGStrFeedback, SPGStrResult
from os import system
import subprocess
from glob import glob
from parse import *
from std_msgs.msg import Header
from std_msgs.msg import String
from std_srvs.srv import *
# from std_srvs.srv import Trigger, TriggerResponse
from turtlesim.srv import *
# from rospy_tutorials.srv import *
from tta_blb.srv import *
import os.path  
import os
from pathlib import Path

def get_size(folder: str) -> int:
    return sum(p.stat().st_size for p in Path(folder).rglob('*'))
timestampLastAliveFromNest = datetime.datetime.now()
last485ResetTime = datetime.datetime.now() 
listPortArd = []
# Seeeduino M0
outputFolder = '/root/SpiderGo'
portArd = '/dev/ttyACM0'    # Serial portr
portArd2 = '/dev/ttyACM1'    # Serial portr
baudArd = 115200            # Serial Baudrate
exitArd = False             # Exit Variable 

lineArdData = []

# ROS
param_ARD_show = True
publish_topic_name = 'KEEP_UTIL'
pub_ka = rospy.Publisher(publish_topic_name, String, queue_size=1)

''' 인자로 밀리초 및 datetime 개체 넘기면 현재 시간에서 해당시간을 넘겼으면 TRUE, 아직 안되었으면 FALSE '''
def isTimeExceeded(lastLogTime : datetime, iMilliSeconds : int):
  td = datetime.datetime.now() - lastLogTime
  tds = td.total_seconds()*1000
  if tds > iMilliSeconds:
    return True
  else:
    return False

''' 네스트에서 오는 아두이노 메세지를 구독한다 '''
sub_topic_name = 'KEEP_ARD'
def callbackNestArd(data):
    global timestampLastAliveFromNest
    timestampLastAliveFromNest = datetime.datetime.now()
    print(f'Update time stamp : {timestampLastAliveFromNest} - {data}')

rospy.Subscriber(sub_topic_name, String, callbackNestArd)


def handler(signum, frame):
    exitArd = True

# def parsing_data(data):
#     tmp = ''.join(data)
#     # print(type(tmp))
#     # print(tmp)
#     try:
#         sSplit1 = ':'
#         sSplit2 = ','
#         sSplit3 = '`'

#         recvDataMap = getDic_strArr(tmp, sSplit1 ,sSplit3)

#         sRESULT = getValueFromMap(recvDataMap,'RESULT')
#         # print("RESULT : " + sRESULT)
#         # if sRESULT is None:
#         #     return

#         checkMap = getValueFromMap(recvDataMap, 'CMDN')
#         # print("CMDN : " + checkMap)
#         if checkMap is None:
#             return

#         checkMap = getValueFromMap(recvDataMap, 'MSGN')
#         # print("MSGN : " + checkMap)
#         if checkMap is None:
#             return

#         sHUMIDN = getValueFromMap(recvDataMap, 'HUMIDN')
#         # print("HUMIDN : " + sHUMIDN)
#         # if sHUMIDN is None:
#         #     return

#         sTEMPN = getValueFromMap(recvDataMap, 'TEMPN')
#         # print("TEMPN : " + sTEMPN)
#         # if sTEMPN is None:
#         #     return
#         # print(type(my_dict))
#         sCMD = recvDataMap['CMDN']
#         sMSG = recvDataMap['MSGN']
#         # print("CMD : " + sCMD)
#         # print("MSG : " + sMSG)

#         if sCMD == 'D':
#             dicArd = recvDataMap
 
#     except Exception as e:
#         print(e)

def parsing_data(data):
    tmp = ''.join(data) 
    spg_dir_size=get_size(outputFolder)
    finalData = f'{tmp}`IMG_TOSEND:{spg_dir_size}'
    try:
      SendKeepAlive(finalData) 
    except Exception as e:
      prtMsg(e)

def SendKeepAlive(sendbuf):
    global pub_ka
    if pub_ka is not None:
        pub_ka.publish(sendbuf)
        #print(sendbuf)

def poll_arduino():
      global serArduino
      global param_ARD_show
      global timestampLastAliveFromNest
      pub = rospy.Publisher(publish_topic_name, String, queue_size=10)
      sendStr = f'LOGO:25'
      SendArd(sendStr)

      global lineArdData
      means = []
      rate = rospy.Rate(1000) #send 1 time per second

      while not rospy.is_shutdown():
        # if rospy.has_param(nameof(param_ARD_show)):
        #     param_ARD_show = rospy.get_param(nameof(param_ARD_show))
        # else:
        #     rospy.set_param(nameof(param_ARD_show), param_ARD_show)
        try:
            if timestampLastAliveFromNest != None:
                ''' 메세지 받은 후 15초가 지났는데 메세지 안 올라오면 재부팅한다  '''
                if isTimeExceeded(timestampLastAliveFromNest, 15000):
                    #os.system(f'/root/.reBoot -&')
                    timestampLastAliveFromNest = datetime.datetime.now()
                    rospy.loginfo('메세지 받은 후 15초가 지났는데 메세지 안 올라오면 재부팅한다')
                    os.system(f'/root/.bootStart -&')

            if serArduino.readable():
                for c in serArduino.read():
                    if c == 13: 
                        parsing_data(lineArdData)
                      
                        del lineArdData[:]
                    elif c == 10: 
                        aaaa = 0
                    else:
                        lineArdData.append(chr(c))
                        #print(chr(c))
        except Exception as e:
            print(traceback.format_exc())
            #prtMsg(e)
            prtMsg(serArduino)

        rate.sleep()

class CommandProcess():
    def __init__(self):
        self.set_cmd_service = rospy.Service('/utilbox/CMD', Kill, self.SendCMD)
        self.set_SCR_service = rospy.Service('/utilbox/SCR', Kill, self.SendSCR)
        self.set_gpo_service = rospy.Service('/utilbox/GPO', utilboxKeyVal, self.GPO_SET)
        self.set_pwm_service = rospy.Service('/utilbox/PWM', utilboxKeyVal, self.PWM_SET)
        self.set_pwm_service = rospy.Service('/utilbox/LOGO', utilboxVal, self.LOGO_SET)
        self.set_enable_service = rospy.Service('/utilbox/SERVO', SetBool, self.ENABLE_SET)
        self.set_enable_service = rospy.Service('/utilbox/FAN', SetBool, self.FAN_SET)
        self.set_enable_service = rospy.Service('/utilbox/LIDAR', SetBool, self.LIDAR_SET)
        self.set_enable_service = rospy.Service('/utilbox/LIGHT', SetBool, self.LIGHT_SET)
        # self.servo_service = rospy.Service('/nestARD/SERVO_SET', Trigger, self.SERVO_SET)
        rospy.loginfo('Utilbox Arduino Started')

    def SendCMD(self, req):
        msg = req.name
        SendArd(msg)
        resp = KillResponse()
        return resp

    def SendSCR(self, req):
        msg = req.name
        os.system(f'/root/catkin_ws/src/tta_blb/scripts/.{msg} -&')
        resp = KillResponse()
        return resp

    def GPO_SET(self, req):
        reqPin = req.pin
        reqVal = req.value
        sendStr = f'D:{reqPin},{reqVal}'
        SendArd(sendStr)
        # resp = AddTwoIntsResponse()
        resp = utilboxKeyValResponse(True, f'SET DO Key:{reqPin}, Value:{reqVal}')
        
        # resp.success = True if reqVal==1 else False
        return resp

    def PWM_SET(self, req):
        reqPin = req.pin
        reqVal = req.value
        sendStr = f'P:{reqPin},{reqVal}'
        SendArd(sendStr)
        # resp = AddTwoIntsResponse()
        resp = utilboxKeyValResponse(True, f'SET PWM  Key:{reqPin}, Value:{reqVal}')
        # resp.success = True if reqVal==1 else False
        return resp

    def ENABLE_SET(self, req):
        reqbool = req.data
        if reqbool is True:
          SendArd('S:170') 
          time.sleep(0.1)
        else:
          SendArd('S:0')
          time.sleep(0.1)
        return SetBoolResponse(True, f'ENABLE_SET:{reqbool}') 

    def FAN_SET(self, req):
        reqbool = req.data
        if reqbool is True:
          SendArd('D:5,0')
        else:
          SendArd('D:5,1')
        return SetBoolResponse(True, f'FAN_SET:{reqbool}')    

    def LIDAR_SET(self, req):
        reqbool = req.data
        if reqbool is True:
          SendArd('D:6,0')
        else:
          SendArd('D:6,1')
        return SetBoolResponse(True, f'LIDAR_SET:{reqbool}')         

    def LIGHT_SET(self, req):
        reqbool = req.data
        if reqbool is True:
          SendArd('D:7,0')
        else:
          SendArd('D:7,1')
        return SetBoolResponse(True, f'LIGHT_SET:{reqbool}') 

    def LOGO_SET(self, req):
        reqVal = req.value
        sendStr = f'LOGO:{reqVal}'
        SendArd(sendStr)
        # resp = AddTwoIntsResponse()
        resp = utilboxValResponse(True, f'SET LOGO Value:{reqVal}')
        # resp.success = True if reqVal==1 else False
        return resp


def getDic_strArr(strTmp, spliterItemValue, spliterLine):
    dicReturn = {}
    file_list = strTmp.split(sep=spliterLine)
    for i in file_list:
        iCurrent = i.split(spliterItemValue, 1)
    if len(iCurrent) > 1:
        dicReturn[iCurrent[0]] = iCurrent[1]
        #print(iCurrent[0])
    return dicReturn

def getCurrentTime(spliter = ':'):
    strTime = datetime.datetime.now().strftime('%H:%M:%S:%f')
    if spliter != None:
        return strTime.replace(':' , spliter)
    return strTime

def getValueFromMap(mapTmp, sKey):
    chkHasKey = sKey in mapTmp.keys()
    if chkHasKey == False:
        return ''
    else:
        return mapTmp[sKey]

def prtMsg(sendbuf):
    global param_ARD_show
    if rospy.has_param(nameof(param_ARD_show)):
        param_ARD_show = rospy.get_param(nameof(param_ARD_show))
    else:
        rospy.set_param(nameof(param_ARD_show), param_ARD_show)

    if param_ARD_show:
        rospy.loginfo(sendbuf)

def SendArd(strData):
    global serArduino
    msg = f'{strData} from {sys._getframe(1).f_code.co_name}-{sys._getframe(0).f_code.co_name}'

    if serArduino == None:
        prtMsg('=> No serial port : ' + msg)
        return
    else:
        prtMsg('=> ' + msg)
    serArduino.write(bytes(strData, encoding='ascii'))
    serArduino.write(bytearray([0x0D]))
    serArduino.write(bytearray([0x0A]))
    return

def open_ard():
    global serArduino
    bOK = False
    while bOK == False:
        try:
            path = os.path.realpath(portArd)
            prtMsg(f'Prevent reboot ard port... {portArd},{path}')
            if None != portArd:
                with open(path) as f:
                    attrs = termios.tcgetattr(f)
                    attrs[2] = attrs[2] & ~termios.HUPCL
                    termios.tcsetattr(f, termios.TCSAFLUSH, attrs)

            prtMsg(f'Initialing ard port... {portArd},{path}')
            serArduino = serial.Serial(portArd, baudArd, timeout=None)
            prtMsg(f'Success Open port! {serArduino}')
            bOK = True
        except Exception as e:
            prtMsg(f'Failed {serArduino} to Open port {portArd} - {e}')
            time.sleep(1)
    time.sleep(1)

class OdysseyArduino():
    def __init__(self):

        self.ser = serial.Serial(self.portArd, self.baudArd, timeout=None)
        self.path = os.path.realpath(self.portArd)
        # self.prtMsg(f'Prevent reboot ard port... {self.portArd},{self.path}')

        self.process = None
        print('ttt')

    # def readThread(serArduino):
    #     # 쓰레드 종료될때까지 계속 돌림
    #     while not exitThread:
    #         #데이터가 있있다면
    #         if serArduino.readable():
    #             for c in serArduino.read():
    #                 #line 변수에 차곡차곡 추가하여 넣는다.
    #                 listPortArd.append(chr(c))

    #                 if c == 13: #라인의 끝을 만나면..
    #                     #데이터 처리 함수로 호출
    #                     parsing_data(listPortArd)
    #                     del listPortArd[:]      
    #                 elif c == 10: #라인의 끝을 만나면.. 
    #                     #line 변수 초기화
    #                     aaaa=11
    #                 else:
    #                     pass

    def openARD(serArduino):
        while not exitArd:
            try:
                # print(serArduino.port)
                if None != portArd:
                    with open(serArduino.port) as f:
                        attrs = termios.tcgetattr(f)
                        attrs[2] = attrs[2] & ~termios.HUPCL
                        termios.tcsetattr(f, termios.TCSAFLUSH, attrs)

                prtMsg(f'Initialing ard port... {[portArd]},{serArduino.port}')
                serArduino = serial.Serial(portArd, baudArd, timeout=None)
                prtMsg(f'Success Open port! {serArduino}')
                bOK = True
            except Exception as e:
                prtMsg(f'Failed {serArduino} to Open port {portArd} - {e}')
                keyIdx = str(e).find('multiple')
                #485통신 포트가 사망한 경우 리셋한다.
                if keyIdx > 0:
                    td =  datetime.datetime.now() - last485ResetTime
                    if td.total_seconds() > 20:
                        os.system(f'/root/.bootStart -&')
                        last485ResetTime= datetime.datetime.now()
                time.sleep(1)
        time.sleep(1)

if __name__ == "__main__":
    rospy.init_node('node_ARD', anonymous = False)
    currentPort = portArd
    if os.path.exists(portArd) == False:
        currentPort= portArd2
    serArduino = serial.Serial(currentPort, baudArd, timeout=None)
    # signal.signal(signal.SIGINT, handler)
    # serArduino = serial.Serial(portArd, baudArd, timeout=0)
    # threadARD = Thread(target=OdysseyArduino.readThread, args=(serArduino,))
    # threadARD.setDaemon(True)
    # threadARD.start()
    CommandProcess()
    #open_ard()
    poll_arduino()
    # OdysseyArduino.openARD(serArduino)
    rospy.spin()