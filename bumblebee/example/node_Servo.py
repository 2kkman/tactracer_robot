#! /usr/bin/env python3
import paho.mqtt.client as mqtt
import json
import copy
import rosclean
import schedule
import numpy
from curses import keyname
from random import randint
from curses.ascii import isdigit
from nturl2path import pathname2url
import gc
from signal import alarm
import struct
import sys
import traceback
import os
from uuid import getnode
import rosnode
import rospy
import threading
from varname import *
import time
from sensor_msgs.msg import LaserScan # LaserScan 메시지 사용준비
from std_msgs.msg import String
import minimalmodbus as minimalmodbus
import serial
from Util import *
from UtilGPIO import *
from SPG_Keys import *
from ServoNano import *
#import seaborn as sns
import pandas as pd
import numpy as np
from io import StringIO
import subprocess
import rosparam
import roslib
from std_srvs.srv import *
from tta_blb.srv import *
from turtlesim.srv import *
from collections import deque

'''
2023-02-13
지시정보 체계 변경 -> 엔코더랑 매핑하기.
1. 연사테스트
2. 합성테스트
3. 
'''
mqtt.Client.connected_flag=False#create flag in class
def on_connect(client, userdata, flags, rc):
    if rc==0:
        client.connected_flag=True #set flag
        rospy.loginfo("connected OK")
    else:
        rospy.loginfo("Bad connection Returned code=",rc)

def on_message(client, userdata, msg):
    rospy.loginfo(msg.topic, msg.payload)

def publish(msg,topic):
#    while True:
 #       time.sleep(1)
    global msg_count
    msg = f"messages: {msg_count}"
    result = client.publish(topic, msg)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        rospy.loginfo(f"Send `{msg}` to topic `{topic}`")
    else:
        rospy.loginfo(f"Failed to send message to topic {topic}")
    msg_count += 1

spdLimit = 100
SpdBoost = False #태그 인식시 속도 가감속 플래그
SpdDown = False
differenceMM = 0 #RFID태그위치와 현재위치사이 거리 mm 단위
chkLidar = False
lidarErrPoint = ''
# 모스키토 broker 정보 #1
#broker_address = 'iot.tactracer.com'
broker_address = 'www.i9man.com'
broker_port = 1883
clientID = get_hostname()
topic = f'spg2040/{clientID}/'
#client = mqtt.Client(client_id=clientID, clean_session=False, userdata=None, protocol=mqtt.MQTTv311,transport="tcp")
client = mqtt.Client(clientID)
client.username_pw_set(username="ttracer", password="tt2015")
client.on_connect=on_connect  #bind call back function
client.loop_start()
rospy.loginfo("Connecting to broker ",broker_address)
client.connect(broker_address, broker_port)      #connect to broker
while not client.connected_flag: #wait in loop
    rospy.loginfo("In wait loop for MQTT SVR")
    time.sleep(1)
#print("in Main Loop")
#client.loop_stop()    #Stop loop 

needToHome = False
strREADY = '1'
BMS_Stat = {}
lock = threading.Lock()
lockARD_N = threading.Lock()
lockARD_U = threading.Lock()
lockLD06 = threading.Lock()
dirPath = os.path.dirname(__file__)
filePath_rosparam = f'{dirPath}/param_ros.yaml'
filePath_ModbusSetupHoming = f'{dirPath}/SERVO_Homing.txt'
filePath_ModbusSetupHomingTest = f'{dirPath}/SERVO_HomingTest.txt'
filePath_ModbusSetupInit = f'{dirPath}/SERVO_Init.txt'
filePath_ModbusAlarm = f'{dirPath}/SERVO_ALARM.txt'
filePath_map_default = f'{dirPath}/MAPPING_RFID.txt'
filePath_mapSPD_default = f'{dirPath}/MAPPING_SPD.txt'
filePath_param_parse = f'{dirPath}/param_parser.txt'
filePath_param_default = f'{dirPath}/param_default.txt'
filePath_CMD_RECONNECT = f'{dirPath}/CMD_RECONNECT.txt'
filePath_CMD_GETHOME = f'{dirPath}/CMD_GETHOME.txt'
sDivFieldColon = StrParser.sDivColon.value
sDivItemComma = StrParser.sDivComma.value
sDivEmart =StrParser.sDivEmart.value
sDivSlash =StrParser.sDivSlash.value
sDivSemiCol = StrParser.sDivSemiColon.value
remain_nodes = 0
''' V - 엔코더 : 113000 일때 1미터(1000mm) 이동함, 즉 1mm 은 113펄스 '''
param_VEncoderPulse = 113
param_VMax_Limit = 113000
param_VMin_Limit = -4000
''' H - 엔코더 : 180000 일때 500mm 이동함, 즉 1mm 은 360펄스 '''
param_HEncoderPulse = 360
param_HMax_Limit = 0
param_HMin_Limit = 0
param_DRV_Rate = 5
param_Return_Time = 5
param_Tilt_Angle = 135 #틸팅 초기화 각도

filePath_param_default = f'{dirPath}/param_default.txt'
epcNot = 'E2009A3110001AF000000107'
param_InvRFID = False
param_BMS_Show = True
param_DRV_show = False

#requestFlag_SetHomeV = False
#requestFlag_SetHomeH = False
SetHomeRange = 9999999999
cali_rpm_V = 15
cali_rpm_H = 5

safetyVoltage = rospy.get_param("~safetyVoltage", default=27)
startVoltage = rospy.get_param("~startVoltage", default=31)

servo_keepalive = 0
servo_keepalive_MAX = 10

start_addr = ServoMonitor.ServoMonitorStatus_Addrs.value[0]
arrExcel = []
sExcelPath = f'/root/SpiderGo/{getDateTime().year}.xlsx'
sExcelMapPath = f'{dirPath}/MAP.xlsx'


def SaveConfig():
    dicConfigTmp = {}
    dicConfigTmp[nameof(param_VMax_Limit)] = param_VMax_Limit
    dicConfigTmp[nameof(param_VMin_Limit)] = param_VMin_Limit
    dicConfigTmp[nameof(param_HEncoderPulse)] = param_HEncoderPulse
    dicConfigTmp[nameof(param_VEncoderPulse)] = param_VEncoderPulse
    dicConfigTmp[nameof(param_HMax_Limit)] = param_HMax_Limit
    dicConfigTmp[nameof(param_HMin_Limit)] = param_HMin_Limit
    dicConfigTmp[nameof(param_DRV_Rate)] = param_DRV_Rate
    dicConfigTmp[nameof(param_Return_Time)] = param_Return_Time
    dicConfigTmp[nameof(param_Tilt_Angle)] = param_Tilt_Angle
    
    saveDic_ToFile(dicConfigTmp, filePath_param_default,sDivEmart)

def LoadConfig():
    global param_VEncoderPulse
    global param_VMax_Limit
    global param_VMin_Limit
    global param_HEncoderPulse
    global param_HMax_Limit
    global param_HMin_Limit
    global param_DRV_Rate
    global param_Return_Time
    global param_Tilt_Angle
    
    dicConfigTmp = {}
    dicConfigTmp = getDic_FromFile(filePath_param_default, sDivEmart)
    param_VMax_Limit = (int)(dicConfigTmp[nameof(param_VMax_Limit)])
    param_VMin_Limit=(int)(dicConfigTmp[nameof(param_VMin_Limit)])
    param_HEncoderPulse=(int)(dicConfigTmp[nameof(param_HEncoderPulse)])
    param_VEncoderPulse=(int)(dicConfigTmp[nameof(param_VEncoderPulse)])
    param_HMax_Limit=(int)(dicConfigTmp[nameof(param_HMax_Limit)])
    param_HMin_Limit=(int)(dicConfigTmp[nameof(param_HMin_Limit)])
    param_DRV_Rate=(int)(dicConfigTmp[nameof(param_DRV_Rate)])
    param_Return_Time = (int)(dicConfigTmp[nameof(param_Return_Time)])
    param_Tilt_Angle = (int)(dicConfigTmp[nameof(param_Tilt_Angle)])

if os.path.exists(filePath_param_default) == False:
    SaveConfig()
LoadConfig()

#subprocess.Popen(['~/.rr'])
#os.system()
suspend = False
def SetSuspend(tf):
    global suspend
    CamLightEnable(False)
    if tf != suspend:
        suspend = tf
        message = f'{sys._getframe(0).f_code.co_name} with {tf} from {sys._getframe(1).f_code.co_name} - runFromLaunch : {runFromLaunch}'        
        rospy.loginfo(message)    

skip_V_Direction = False
evtDock_V = threading.Event() #V호밍 포인트
evtDock_H = threading.Event() #H호밍 포인트
df_fromSPGMAP : pd.DataFrame = pd.DataFrame()

dicServiceTimeStamp = {}
iCntLoop = 0 #레일 주행 완주 횟수
write_delay = 0.01
dicInit = None
lastEPC : str = None
instrument_Stat = {"H" : {}, "V":{}}
port485 = '/dev/ttC485' # 시리얼 포트
#port485 = '/dev/ttyUSB0' # 시리얼 포트
instrumentH :minimalmodbus.Instrument = None
instrumentV :minimalmodbus.Instrument = None
instrumentBMS :minimalmodbus.Instrument = None

curCurrentBMS = 'CurCadc'
curPackBMS = 'PackStatus'
listTmp = ['Cell_Num','Run_Time','HSOC','Voltage',curCurrentBMS,'Temp1','Temp2','Temp3','Temp4','Temp5','Temp6','Tmax','Tmin','Vmax','Vmin','VmaxminNo',
           'RSOC','FCC','RC','CycleCount','PROTECT','ALARM',curPackBMS,'VCell1','VCell2','VCell3','VCell4','VCell5','VCell6','VCell7','VCell8','VCell9',]
lenthList = len(listTmp)
modbusOpmode=6008
#sCmdRemote = 'ssh root@172.30.1.24 "./.rrStart"'
#os.system(sCmdRemote)


instruments = {}
dicReport: Dict[str,float] = {}
dicRFID: Dict[str,str] = {}
dicSPDMAP: Dict[str,str] = {}

dicSD: Dict[str,str] = {}
dicARD_N: Dict[str,str] = {}
dicARD_U: Dict[str,str] = {}
dicLD06: Dict[float,float] = {}
drvCaption_H = 'H'
drvCaption_V = 'V'
dirCaption_R = 'R'
dirCaption_D = 'D'
dirCaption_U = 'U'
dirCaption_L = 'L'
dirCaption_F = 'F'
dirCaption_S = 'S' #Smoothing + Reinit
dirCaption_B = 'B'
dirCaption_H = 'H' #Homing 명령
dirCaption_M = 'M' #MotorResume 명령 : 중지되었던 모터를 다시 구동 (설정된 거리까지)
dirCaption_Q = 'Q' # Set to Zero
dirCaption_SKIPV = 'SKIPV' #RFID TEST용 명령. 이 값이 0인 경우 자동주행에서 위 아래 명령은 모두 무시한다.
dirCaption_REP = '0' #데모용 반복 운행 명령어.

dirCaption_Backward = dirCaption_R
dirCaption_Foward = dirCaption_L

dirCaption_VMAX = 'VMAX' # V캘리브레이션
dirCaption_HMAX = 'HMAX' # H캘리브레이션
dirCaption_HMIN = 'HMIN' # H캘리브레이션
dirCaption_SAVE = 'SAVE' # 환경설정 세이브
dirCaption_NOLIMIT_V = 'NOLIMIT_V' # V리밋해제
dirCaption_NOLIMIT_H = 'NOLIMIT_H' # V리밋해제
dirCaption_TILT = 'TILT' # 모터 틸팅
dirCaption_NODE = 'NODE' # 작업노드 변경

curLoc_H=f'CURLOC_{drvCaption_H}'
oldLoc_H=f'OLDLOC_{drvCaption_H}'
curLoc_V=f'CURLOC_{drvCaption_V}'
oldLoc_V=f'OLDLOC_{drvCaption_V}'
drvID_H = 5
drvID_V = 6
mapping = False
total_length_rail = -1
drvDic = {drvCaption_H : drvID_H, drvCaption_V:drvID_V, dirCaption_L:drvID_H,  dirCaption_R:drvID_H,  dirCaption_D:drvID_V,  dirCaption_U:drvID_V }
EncoderDic={drvID_H:drvCaption_H,drvID_V:drvCaption_V}
EncoderCurrent = {drvCaption_H : 0, drvCaption_V: 0 }
drvIDMoving : str = ''

CMD_Queue = deque() #모드버스 통신 동기화를 위한 Write 버퍼

#publish_topic_name = 'scan_alarm'
publish_topic_goal  : str = 'GOAL' #테스트용 변수
dic_CurrentNode = {}
dic_485 = {}
utilard_keepalive = getDateTime()
lastUpdateAlarm = getDateTime()
lastLogTime = getDateTime()
lastActionTime = getDateTime()
lastStopTime = getDateTime()
lastBMSPollingTime = getDateTime()
lastLD06Time = getDateTime()
publish_topic_name = NEST_TOPIC.SERVO.value

publish_topic_name = 'KEEPALIVE'
nodeName = f'node_{publish_topic_name}'
alarm_interval = 5000
rospy.init_node(nodeName,anonymous=False) # 485 이름의 노드 생성
runFromLaunch = rospy.get_param("~reset_24V", default=False)
restart_Time = try_parse_int(rospy.get_param("~restart_Time", default=15000))
rospy.loginfo(f'reset_24V : {runFromLaunch}')
pub_cmd = rospy.Publisher(NEST_TOPIC.CMD.value, String, queue_size=10)
pub_alarm = rospy.Publisher(NEST_TOPIC.ALARM.value, String, queue_size=10)
pub_fb = rospy.Publisher(NEST_TOPIC.FEEDBACK.value, String, queue_size=10)
pub_kpalive = rospy.Publisher(publish_topic_name, String, queue_size=10)
pub_encoder = rospy.Publisher(NEST_TOPIC.ENCODER.value, String, queue_size=10)
bPublishEncoder = True

def SaveExcel():
    global arrExcel
    global dicReport
    resultSave = False
    if len(dicReport) > 0:
        try:
            #rospy.loginfo(f'{sys._getframe(0).f_code.co_name} - {arrExcel}')
            rospy.loginfo(f'Call {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}')
            arrExcel[0].append(dicReport.copy())
            df_stat = pd.DataFrame(arrExcel[0])
            #데이터프레임 엑셀파일로 추출
            df_stat.to_excel(sExcelPath)
            resultSave = True
        except Exception as e:
            message = f'{traceback.format_exc()} in {sys._getframe(0).f_code.co_name} from {sys._getframe(1).f_code.co_name}'        
            rospy.loginfo(message)
    return resultSave
    
def LoadExcel():
    global arrExcel
    try:
        rospy.loginfo(f'Call {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}')
        df = pd.read_excel(sExcelPath, engine = "openpyxl", index_col = 0)
        arrExcel.clear()
        arrExcel.append(df.to_dict('records'))        
    except Exception as e:
        message = f'{traceback.format_exc()} in {sys._getframe(0).f_code.co_name} from {sys._getframe(1).f_code.co_name}'        
        rospy.loginfo(message)
        arrExcel.append([])
LoadExcel()
rate = rospy.Rate(param_DRV_Rate)

#tmpData = getStr_FromFile(filePath_CMD_RECONNECT)
#df_fromSPGMAP = str2frame(tmpData,'\t')

#rospy.wait_for_service(ServiceList.SDV5LOCK.value, 20)
try:
    for s in ServiceList:
        sCurrent = s.value
        if sCurrent.startswith('/nest'):
            rospy.wait_for_service(sCurrent, 100)
            rospy.loginfo(f'Service {s.value}:{sCurrent} is up')
except Exception as e:
    message = traceback.format_exc()
    rospy.logfatal(message)

targ_H = 'TARG_H'
targ_V = 'TARG_V'
rospy.loginfo(f'dirname: {os.getcwd()}', )
dic_ServoMonitorAlarm = getDic_FromFile(filePath_ModbusAlarm ,sDivItemComma)
dicAlarmCode = getDic_FromFile(f'{dirPath}/SERNO_NANO.ini',sDivSlash)
dicParamParse = getDic_FromFile(filePath_param_parse,sDivEmart)

alarmAddr = 0xB03
alarmLen = 31

iCnt_MovingStamp = 0
e_stop_addr = ServoKey.E_STOP.value
e_stop_param = ServoParam.E_STOP_NOW.value
e_stop_resume = ServoParam.E_RESUME.value
e_spd_change = ServoParam.E_SPD_CHANGE.value
drvID_current= cmdIDCurrent = LastActionCmd = ''
dic_ServoMonitorStatus = {}
#print((COMMON_CMD.__members__.keys()))
#print(dijkstra(None, '0', '8'))
#dijkstraEx(None, 850, '0', '57')
list_Alarm = {}
# print(service_setbool_client(ServiceList.SDV16.value, False, SetBool))
# print(service_setbool_client(ServiceList.SDV16.value, True, SetBool))


def isOperation():
    if len_df_fromSPGMAP == None:
        return False
    #return not(len_df_fromSPGMAP == 0 and getValueFromMap(dic_485,targ_H) == '1' and getValueFromMap(dic_485,targ_V) == '1')
    return not(len_df_fromSPGMAP == 0 and not IsMoving())
    #return not(getValueFromMap(dic_485,targ_H) == '1' and getValueFromMap(dic_485,targ_V) == '1')

def service_setbool_client(serviceName, enable, serviceType):
    global dicServiceTimeStamp
    bResult = None
    if isServiceExist(serviceName) == False:
        rospy.loginfo(f'Service not found : {serviceName}')
        return False
    #rospy.wait_for_service(serviceName, 2)
    dtNow = getDateTime()
    serviceNameID = None
    if enable == None:
        serviceNameID = f'{serviceNameID}'
    else:
        serviceNameID = f'{serviceNameID}{enable}'
    serviceLastTimeStamp = dicServiceTimeStamp.get(serviceNameID)
    dicServiceTimeStamp[serviceNameID] = dtNow
    if serviceLastTimeStamp != None and not isTimeExceeded(serviceLastTimeStamp, 1000):
        return False
    try:
        setbool_proxy = rospy.ServiceProxy(serviceName, serviceType)
        sos = None

        #print(type(serviceType))
        if enable == None:
            if serviceType == Trigger:
                sos = TriggerRequest()
            else:
                sos = EmptyRequest()
            bResult = setbool_proxy(sos)
            return bResult

        responseResult = setbool_proxy(enable)
        #print(responseResult)
        rospy.loginfo(f'Service({serviceType}) called : {serviceName} - {enable} : from {sys._getframe(1).f_code.co_name}-{sys._getframe(2).f_code.co_name}')
        return responseResult
    except Exception as e:
        rospy.loginfo(e)
        return False


def UpLoadingStatus():
    tempC =service_setbool_client(ServiceList.IsEnableUpload.value, None, Trigger)
    return tempC.message,tempC.success

def UpLoadEnable(enable):
    return service_setbool_client(ServiceList.EnableUpload.value, enable, SetBool)

# print(GetUpLoadingStatus())
# SetUpLoading(False)

def PowerLock(enable):
    if service_setbool_client(ServiceList.SDV5LOCK.value, enable, SetBool):
        time.sleep(1)
    if service_setbool_client(ServiceList.SDV24LOCK.value, enable, SetBool):
        time.sleep(1)
    return service_setbool_client(ServiceList.SDV16LOCK.value, enable, SetBool)
#PowerLock(True)

def Power24V(enable):
    global dicSD
    keyStr = 'ONOFF_V24'
    if dicSD.get(keyStr) == '1' and enable:
        return True
    if dicSD.get(keyStr) == '0' and not enable:
        return True
    return service_setbool_client(ServiceList.SDV24.value, enable, SetBool)

def Power16V(enable):
    # global dicSD
    # keyStr = 'ONOFF_V16'
    # if dicSD.get(keyStr) == '1' and enable:
    #     return True
    # if dicSD.get(keyStr) == '0' and not enable:
    #     return True
    return service_setbool_client(ServiceList.SDV16.value, enable, SetBool)

def LidarEnable(enable):
    global bPublishEncoder
    bPublishEncoder = enable
    if enable == True:
        return service_setbool_client(ServiceList.LIDAR_START.value, None, Trigger)
    else:
        return service_setbool_client(ServiceList.LIDAR_STOP.value, None, Trigger)

def CamEnable(enable):
    return True
    if enable == True:
        if service_setbool_client(ServiceList.CAM_R_ENABLE.value, None, Empty):
            time.sleep(2)
        service_setbool_client(ServiceList.CAM_L_ENABLE.value, None, Empty)
    else:
        if service_setbool_client(ServiceList.CAM_R_DISABLE.value, None, Empty):
            time.sleep(2)
        service_setbool_client(ServiceList.CAM_L_DISABLE.value, None, Empty)

def CamSaveControl(enable): #없어졌음.
    resultValue = None      
    if enable == True:
        resultValue = service_setbool_client(ServiceList.CAM_SAVE_START.value, None, Trigger)
    else:
        resultValue = service_setbool_client(ServiceList.CAM_SAVE_STOP.value, None, Trigger)
    rospy.loginfo(resultValue)
    return resultValue

def CamFocus(strValue, iCamFocus):
    message = f'{sys._getframe(0).f_code.co_name} from {sys._getframe(1).f_code.co_name} - runFromLaunch : {runFromLaunch}'        
    rospy.loginfo(message)
    
    responseResult = None
    serviceName = ServiceList.CAM_PREFIX_FOCUS.value
    if isServiceExist(serviceName) == False:
        rospy.loginfo(f'Service not found : {serviceName}')
        return False
    try:
        # callData = utilboxCam()
        # callData.message = strValue
        # callData.value = 5
        setbool_proxy = rospy.ServiceProxy(serviceName, utilboxCam)
        responseResult = setbool_proxy(strValue,iCamFocus)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
    return responseResult

def CamAuto(strValue, imgCnt):
    message = f'{sys._getframe(0).f_code.co_name} from {sys._getframe(1).f_code.co_name} - runFromLaunch : {runFromLaunch}'        
    rospy.loginfo(message)

    responseResult = None
    serviceName = ServiceList.CAM_PREFIX.value
    if isServiceExist(serviceName) == False:
        rospy.loginfo(f'Service not found : {serviceName}')
        return False
    try:
        # callData = utilboxCam()
        # callData.message = strValue
        # callData.value = 5
        setbool_proxy = rospy.ServiceProxy(serviceName, utilboxCam)
        responseResult = setbool_proxy(strValue,imgCnt)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
    return responseResult

def CamTest(strValue, imgCnt):
    message = f'{sys._getframe(0).f_code.co_name} from {sys._getframe(1).f_code.co_name} - runFromLaunch : {runFromLaunch}'        
    rospy.loginfo(message)
    responseResult = None
    serviceName = ServiceList.CAM_PREFIX_TEST.value
    if isServiceExist(serviceName) == False:
        rospy.loginfo(f'Service not found : {serviceName}')
        return False
    try:
        # callData = utilboxCam()
        # callData.message = strValue
        # callData.value = 5
        setbool_proxy = rospy.ServiceProxy(serviceName, utilboxCam)
        responseResult = setbool_proxy(strValue,try_parse_int(imgCnt))
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
    return responseResult

CamEnable(False)
testCnt = 0
# while True:
#     testCnt +=1
#     timeStamp = getCurrentTime(spliter = ' ')
#     CamEnable(True)

#     CamFileNamePrefix(f'T{timeStamp}-{testCnt}')
#     ResultA = False
#     ResultB = False
#     time.sleep(1)
#     while ResultA == False:
#         ResultA = CamSaveControl(True)
#         print("1 :",ResultA , testCnt)
#     time.sleep(1)
#     while ResultB == False:
#         ResultB = CamSaveControl(True)
#         print("2 :", ResultB , testCnt)

    #CamSaveControl(True)
    #CamSaveControl(False)
    # CamEnable(False)

def SetHome():
    message = f'{sys._getframe(0).f_code.co_name} from {sys._getframe(1).f_code.co_name} - runFromLaunch : {runFromLaunch}'        
    rospy.loginfo(message)
    ModbusInitAll(filePath_ModbusSetupHoming, sDivItemComma, None)

def SetHomeTest():
    message = f'{sys._getframe(0).f_code.co_name} from {sys._getframe(1).f_code.co_name} - runFromLaunch : {runFromLaunch}'        
    rospy.loginfo(message)    
    ModbusInitAll(filePath_ModbusSetupHomingTest, sDivItemComma, None)

def SetHomeH():
    message = f'{sys._getframe(0).f_code.co_name} from {sys._getframe(1).f_code.co_name} - runFromLaunch : {runFromLaunch}'        
    rospy.loginfo(message)
    ModbusInitAll(filePath_ModbusSetupHoming, sDivItemComma, drvCaption_H)
    SetLimit(param_HMin_Limit, param_HMax_Limit,drvCaption_H)

def SetHomeV():
    message = f'{sys._getframe(0).f_code.co_name} from {sys._getframe(1).f_code.co_name} - runFromLaunch : {runFromLaunch}'        
    rospy.loginfo(message)
    ModbusInitAll(filePath_ModbusSetupHoming, sDivItemComma, drvCaption_V)
    SetLimit(param_VMin_Limit, param_VMax_Limit,drvCaption_V)

def InitBMS():
    dicBMS = getDic_FromFile(f'{dirPath}/BMS8S.ini','/')
    keys = dicBMS.keys()
    for k in keys:
        kVal =dicBMS[k]
        instrumentBMS.write_register(int(f'0x{k}',16),int(kVal), 0,6) #CELL Number
        time.sleep(0.1)

def MotorResume()-> int:
    iReturn = 0
    chkStatusH,chkStatusV = getMultiEx(dic_485, 'QS')
    if chkStatusH == '0':
        WriteReg(modbusOpmode, 31, drvCaption_H)
        iReturn += 1
    if chkStatusV == '0':
        WriteReg(modbusOpmode, 31, drvCaption_V)
        iReturn += 3
    return iReturn

# ''' 현재 자동운행모드인지 확인 '''
# def IsAutoOperation()-> bool:
#     return True if dic_485.get(SPG_Keys.OPMODE.name) == 1 else False

''' 유틸함수 개발. 현재 주행중인지 확인하는 함수
TARG_H 와 V 를 사용, None 이 들어오면 H와 V 모두 리턴, 하나만 들어오면 한개만 리턴하는 함수.
getMulti 함수 참고할 것. '''
def isMotorFinished(strKey):
  chkHasKey = strKey in dic_485.keys()
  if chkHasKey:
    return isTrue(dic_485[strKey]), None
  ret1 = getValueFromMap(dic_485, targ_H)
  ret2 = getValueFromMap(dic_485, targ_V)
  return isTrue(ret1),isTrue(ret2)

def SetAlarm(bReturn : bool,prtStatus = True):
    global list_Alarm
    keyName= sys._getframe(1).f_code.co_name
    list_Alarm[keyName] = bReturn
    if prtStatus:
        rospy.loginfo(list_Alarm)

''' 현재 네스트 수평 위치정보 확인 '''
def GetCurLocV()-> str:
    return dic_485.get(curLoc_V, '')

def GetCurLocH()-> str:
    return dic_485.get(curLoc_H, '')

def GetDirection() -> int:
    oldLoc_H_int = try_parse_int(dic_485.get(oldLoc_H, ''))
    curLoc_H_int = try_parse_int(GetCurLocH())
    return curLoc_H_int - oldLoc_H_int

def IsMovingH() -> bool:
    oldLoc_H_int = abs(try_parse_int(dic_485.get(oldLoc_H, '')))
    curLoc_H_int = abs(try_parse_int(GetCurLocH()))
    if abs(curLoc_H_int - oldLoc_H_int) > 1000:
        return True
    return False

def IsMovingV() -> bool:
    oldLoc_V_int = abs(try_parse_int(dic_485.get(oldLoc_V, '')))
    curLoc_V_int = abs(try_parse_int(GetCurLocV()))
    if abs(curLoc_V_int - oldLoc_V_int) > 1000:
        return True
    return False

def IsMoving() -> bool:
    return (IsMovingH() or IsMovingV())

    # curLoc = int(curLocTmp)
    # return curLoc
    # if total_length_rail > 0:
    #     if curLoc < 0:
    #         curLoc = total_length_rail + curLoc
    # return curLoc % total_length_rail

#region RFID Control
def RFIDControl(bEnable : bool):
    #return
    sv = 1 if bEnable is True else 0
    #param_cmd = f'{NEST_TOPIC.RFID.value}{StrParser.sDivSlash.value}I{StrParser.sDivColon.value}{sv}'
    #param_cmd = f'I{StrParser.sDivColon.value}{sv}'
    param_cmd = getCMDStrFromParams(getCurrentTime(''), 'I', sv)
    pub_cmd.publish(param_cmd)
    #rospy.set_param('param_InvRFID', start)
    #prtMsg(f'=>RFID:{start}')

def callbackRFID(data):
    global dicRFID
    global df_fromSPGMAP
    global lastEPC
    global mapping
    global SpdBoost
    global SpdDown
    global differenceMM

    AutoStop = False
    try:
        recvData = data.data
        recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        sEPCKey = 'EPC'
        sEPC = getValueFromMap(recvDataMap,sEPCKey)
        #if sEPC != None and sEPC != lastEPC:
        if sEPC != None:
            curLoc = GetCurLocH()
            # if sEPC == epcNot:
            #     tmpData = getStr_FromFile(filePath_CMD_RECONNECT)
            #     df_fromSPGMAP.join(str2frame(tmpData,'\t'))
            #    
            # el
            #if lastEPC != sEPC:
            if True:
                differenceEPC=0
                chkStatusH,chkStatusV = getMultiEx(dic_485, 'TARG')
                savedEPC = dicRFID.get(sEPC, '')
                # if lastEPC != sEPC and chkStatusH:                 
                #     rospy.loginfo(f'RFID Recv : {recvData},{sEPC} at Pos : {curLoc}')
                if mapping:
                    dicRFID[curLoc] = sEPC
                    if lastEPC != sEPC:
                        rospy.loginfo(f'RFID Mapping : {sEPC} at {curLoc}')

                #if lastEPC != sEPC and is_digit(curLoc) and chkStatusH != strREADY:
                if chkStatusH != strREADY: #수평모터 동작중
                    mapSPD = try_parse_int(dicSPDMAP.get(sEPC,''))
                    if LastActionCmd == dirCaption_Backward: #복귀방향
                        if mapSPD > 0:
                            if not SpdDown:
                                rospy.loginfo(f'Slow Tag Detected from Mapping : {sEPC} spd : {mapSPD} at {curLoc}')
                                #ChargeNestEnable(False)
                                motorMove(0, mapSPD,dirCaption_R,False, None, None)
                                SpdDown = True
                        else:
                            if not SpdBoost:
                                rospy.loginfo(f'Fastest Tag Detected from Mapping : {sEPC} at {curLoc}')
                                motorMove(0, 100,dirCaption_R,False,None, None)
                                SpdBoost = True

                if lastEPC != sEPC and is_digit(curLoc) and chkStatusH != strREADY and savedEPC != '':
                    #rospy.loginfo(f'RFID Detected : {sEPC} - {curLoc}')
                    #differenceEPC = abs(savedEPC) - 
                    curLocABS = abs(int(curLoc))
                    curLocEPC = abs(int(savedEPC))
                    differenceEPC =abs(curLocEPC-curLocABS)
                    differenceMM = differenceEPC / param_HEncoderPulse
                    rospy.loginfo(f'RFID Recv with {LastActionCmd} : {recvData},{sEPC} at Pos : {curLoc} : saved {savedEPC}, difference : {differenceEPC}({differenceMM : 0.2f} mm)')
                elif savedEPC != '' and lastEPC != sEPC :                     
                    rospy.loginfo(f'Invalid RFID  : {sEPC} with savedEPC : {savedEPC}, curLoc : {curLoc}')
                lastEPC = sEPC
                if AutoStop is True and sEPC is not None:
                    rospy.loginfo(f'Stop by {sEPC}')
                    motorStop(drvCaption_H, True)
            #SendFeedback(recvData)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        #print (e)
        #SendFeedback(e)

UIN24V = '0'
def callbackSD(data):
    global dicSD
    global lock
    global UIN24V
    try:
        recvData = data.data
        recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivEmart)
        lock.acquire()
        try:
            dicSD.update(recvDataMap)
        finally:
            # Lock을 해제해서 다른 Thread도 사용할 수 있도록 만든다.
            lock.release()
            #rospy.loginfo(dicSD)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
    UIN24V = dicSD.get('UIN_V24','0')

def callbackKEEP_ARD(data):
    global dicARD_N
    global lockARD_N
    try:
        recvData = data.data
        recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivEmart)
        lockARD_N.acquire()
        try:
            dicARD_N.update(recvDataMap)
        finally:
            # Lock을 해제해서 다른 Thread도 사용할 수 있도록 만든다.
            lockARD_N.release()
            #rospy.loginfo(dicSD)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)


def callbackUTIL_ARD(data):
    global utilard_keepalive
    utilard_keepalive = getDateTime()
    global dicARD_U
    global lockARD_U
    try:
        recvData = data.data
        recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivEmart)
        lockARD_U.acquire()
        try:
            dicARD_U.update(recvDataMap)
        finally:
            # Lock을 해제해서 다른 Thread도 사용할 수 있도록 만든다.
            lockARD_U.release()
            #rospy.loginfo(dicSD)
        IMUChk = dicARD_U.get('GAV_SV', None)
        if IMUChk != None:
            file_list = IMUChk.split(sep=',')  
        
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)

def callback_scheduler(data):
    global dicLD06
    global lockLD06
    global lastLD06Time
    try:
        recvData = data.data
        recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivEmart)
        lockLD06.acquire()
        try:
            dicLD06.update(recvDataMap)
            lastLD06Time = getDateTime()
        finally:
            # Lock을 해제해서 다른 Thread도 사용할 수 있도록 만든다.
            lockLD06.release()
            #rospy.loginfo(dicSD)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)

def callbackscan_alarm(data):
    global dicLD06
    global lockLD06
    global lastLD06Time
    try:
        recvData = data.data
        recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        lockLD06.acquire()
        try:
            dicLD06.update(recvDataMap)
            lastLD06Time = getDateTime()
        finally:
            # Lock을 해제해서 다른 Thread도 사용할 수 있도록 만든다.
            lockLD06.release()
            #rospy.loginfo(dicSD)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)

LastMoveRange = param_VMax_Limit
V_close_alert = 22600 * 2
def CheckSafetyLidar(moveRange = None):
    global LastMoveRange
    global lastLD06Time
    global dicLD06
    global lockLD06
    global remain_nodes
    global lastLD06Time
    moveRange = param_VMax_Limit
    # #return True
    # if moveRange == None:
    #     moveRange = LastMoveRange  
    # else:
    #     if moveRange > param_VMax_Limit:
    #         return False        
    #     LastMoveRange = moveRange
    #대영정공 원래배치는 44~67
    #파손으로 인해 변경된 범위 123~137 
    #CES2023 에서 40~56
    #여기에 45를 더 할 것
    camAngelOffset = 315
    startAngle = 40#(40+camAngelOffset) % 360
    endAngle = 56#(56+camAngelOffset) % 360
    moveLength = moveRange / param_VEncoderPulse / 1000
    curV = abs(int(dic_485.get(curLoc_V, '0'))) / param_VEncoderPulse / 1000
    checkAngle = []
    if startAngle > endAngle:
        checkAngle = list(range( startAngle, 359)) + list(range( 0, endAngle))
    else:
        checkAngle=list(range( startAngle, endAngle))

    lockLD06.acquire()
    for k,v in dicLD06.items():
        angle = int(k)
        distance = float(v)
        finalRange = (moveLength - curV)
        if angle in checkAngle:
            # if moveRange == V_close_alert:
            #     finalRange = moveLength
            if distance < moveLength:
                lockLD06.release()
                if remain_nodes > 0 or not runFromLaunch:
                    #print(f'(angle:{k}-distance:{v})')
                    dicErr = {}
                    for curAng in checkAngle:
                        dicErr[curAng] = distance
                    message = f'moveLength : {moveLength : 0.2f}, curV = {curV : 0.2f}, , Last Lidar Data : {lastLD06Time}, Lidar Info = {dicErr.items()} from {sys._getframe(1).f_code.co_name}'
                    #message = f'moveLength : {moveLength : 0.2f}, curV = {curV : 0.2f}, Lidar Info = {sorted(dicLD06.items())} from {sys._getframe(1).f_code.co_name}'
                    rospy.loginfo(message)
                return False
    lockLD06.release()
    LastMoveRange = param_VMax_Limit
    return True


#endregion


#print(isTimeExceeded(lastLogTime, 1000))

def SetLastActionTime(logmsg = True):
    global lastActionTime
    lastActionTime = getDateTime()
    if logmsg:
        message = f'Update Action Stamp : {lastActionTime} - {sys._getframe(1).f_code.co_name}'
        rospy.loginfo(message)

gpio_bouncetime : int = 50
boolGPIO = False

# for idx,entry in enumerate(ServoMonitorStatus):
#     dic_ServoMonitorStatus[entry.value] = entry.name
#     print(entry.name, entry.value)

# print (dic_ServoMonitorStatus)

#데이터 처리할 함수
def parsing_BitData(value :int , addr_Modbus : int) ->Dict[str,str]:
    global seq
    sDivField = ":"
    sDivItem = ","
    addr_Modbus = str(addr_Modbus)
    #pub = rospy.Publisher('BMS', String, queue_size=10)
    #strBit = bin(int(value))[2:].rjust(16,'0')
    strBit = bin(value)[2:]
    dicReturn = {}

    #print(value)
    if addr_Modbus in dicAlarmCode:
        iDataTotalLength = 0
        kVal =dicAlarmCode[addr_Modbus]
        file_list = kVal.split(sep=',')
        for i in file_list:
          iCurrent = i.split(':', 1)
          itemName = iCurrent[0]
          itemLen = (int)(iCurrent[1])
          iDataTotalLength = iDataTotalLength + itemLen

        iDataPos = 0
        strBit=strBit.rjust(iDataTotalLength,'0')
        strBit = f'0b{strBit}'
        strBMSDataPartBit = ConstBitStream(strBit)
        #print(strBMSDataPartBit)

        for i in file_list:
          iCurrent = i.split(':', 1)
          itemName = iCurrent[0]
          #print(iCurrent)
          itemLen = (int)(iCurrent[1])
          signHeader = 'uint'
          if(itemLen >= 8):
              signHeader = 'int'
          itemHex = strBMSDataPartBit.read(f'{signHeader}:{itemLen}')
          dicReturn[itemName] = itemHex
    strResult = getStr_fromDic(dicReturn,sDivField,sDivItem)
    #print(strResult)
    return dicReturn
#print(parsing_BitData(1600, 5000))

def SendFeedbackFinish(strStatus:str,cmdIDtoFinish:str) -> bool:
    global dic_CurrentNode
    global LastActionCmd
    #global lastActionTime
    sendbuf = ''
    if cmdIDtoFinish != '' and cmdIDtoFinish != None:
        lastActionTimeTmp= getDateTime()
        
        if strStatus == CmdStatus.Finished.name:
            SetLastActionTime()
        lastActionTimeStr = str(lastActionTimeTmp).replace(':','').replace('-','')
        currentTimeStr = str(getDateTime()).replace(':','').replace('-','')
        sLastSeenCurrent = dic_CurrentNode.get(SPG_Keys.LASTSEEN.name)
        dicSend = {}

        if sLastSeenCurrent != None and sLastSeenCurrent == cmdIDtoFinish:
            dicSend = dic_CurrentNode

        dicSend[SPG_Keys.ID.name] = cmdIDtoFinish
        dicSend[SPG_Keys.STATUS.name] = strStatus
        dicSend[SPG_Keys.DEBUG.name] = sys._getframe(1).f_code.co_name
        dicSend[SPG_Keys.CURRTIME.name] = currentTimeStr.split()[1]
        dicSend[SPG_Keys.RECVTIME.name] = lastActionTimeStr.split()[1]
        dicSend[SPG_Keys.LASTCMD.name] = LastActionCmd
        #sendbuf = f'ID:{cmdIDtoFinish},STATUS:{strStatus},DEBUG:{sys._getframe(1).f_code.co_name},CURRTIME:{currentTimeStr.split()[1]},RECVTIME:{lastActionTimeStr.split()[1]},LASTCMD:{LastActionCmd}'
        sendbuf = getStr_fromDic(dicSend, sDivFieldColon, sDivItemComma)
        #cmdIDtoFinish = ''
        return SendFeedback(sendbuf)

def SendAlarm(alarmDict : Dict)-> bool:
    global pub_alarm
    bReturn = True
    sendbuf = getStr_fromDic(alarmDict, sDivFieldColon, sDivItemComma)
    try:
        if pub_alarm is not None:
            pub_alarm.publish(sendbuf)
    except Exception as e:
        message = traceback.format_exc()
        bReturn = False
        rospy.loginfo(message)
    #print(f'{sys._getframe(0).f_code.co_name}=>{sendbuf}')
    return bReturn

def SendFeedback(sendbuf : str) -> bool:
    global pub_fb
    try:
        if pub_fb is not None:
            pub_fb.publish(sendbuf)
            rospy.loginfo(sendbuf)
            return True
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
    rospy.loginfo(f'{sys._getframe(0).f_code.co_name}=>{sendbuf}')
    return False

def SetLimit(VMin, VMax,drvCaption):
    iMin = int(VMin)
    iMax = int(VMax)
    iMin1, iMin2 = splitSignedInt(iMin)
    iMax1, iMax2 = splitSignedInt(iMax)
    lsRTUParam = [iMin1,iMin2,iMax1,iMax2]
    WriteRegEx(ServoMonitor.ServoMonitorField_LimitAddr.value,lsRTUParam,drvCaption)
    rospy.loginfo(f'Set {drvCaption} for min = {iMin},Set max = {iMax} from {sys._getframe(1).f_code.co_name}')

def InitMap():
    global currOperationPos
    global df_fromSPGMAP    
    global chkLidar
    global lidarErrPoint
    lidarErrPoint = ''
    del df_fromSPGMAP
    df_fromSPGMAP = pd.DataFrame()
    currOperationPos = 0
    chkLidar = True
    SetSuspend(False)
    rospy.loginfo(f'Call {sys._getframe(0).f_code.co_name} from {sys._getframe(1).f_code.co_name}')        

#MAP MERGE 단위테스트 부터 해보자-_-
def MapMerge(newMap, bAppend):
    global currOperationPos
    global df_fromSPGMAP    
    if len(df_fromSPGMAP) == 0:
        currOperationPos = 0
    df_fromSPGMAPTmp = None
    if bAppend:
        df_fromSPGMAPTmp = pd.concat([df_fromSPGMAP,newMap], ignore_index=True)        
    else:
        df_fromSPGMAPTmp = pd.concat([newMap,df_fromSPGMAP], ignore_index=True)
    df_fromSPGMAP = df_fromSPGMAPTmp.dropna(subset=[SPG_Keys.START.name])
    #df_fromSPGMAP = df_fromSPGMAPTmp
    df_fromSPGMAP.reset_index(inplace=True)
    rospy.loginfo(df_fromSPGMAP)
    rospy.loginfo(f'Call {sys._getframe(0).f_code.co_name} from {sys._getframe(1).f_code.co_name}')        

# tmpData = getStr_FromFile(filePath_CMD_RECONNECT)                    
# curMap =str2frame(tmpData,'\t')
# curMap2 = pd.read_excel(sExcelMapPath, engine = "openpyxl", index_col = 0)
# MapMerge(curMap2, True)
# MapMerge(curMap, True)

def GetFullMapInfo(curMap):
    #tmpData = getStr_FromFile(filePath_CMD_TESTMAIN)          
    #curMap =str2frame(tmpData,'\t')
    filePath_CMD_TESTCLASS = f'{dirPath}/CMD_TESTCLASS.txt'
    tmpData2 = getStr_FromFile(filePath_CMD_TESTCLASS)          
    curMap2 =str2frame(tmpData2,'\t')
    curMap3 = pd.merge(left = curMap , right = curMap2, how = "inner", on = "PROFILE")
    lenMap1 = len(curMap)
    lenMap2 = len(curMap2)
    lenMap3 = len(curMap3)
    if lenMap1 != lenMap3:
        rospy.loginfo(f'프로필이 정의되지 않았습니다 {lenMap1},{lenMap2},{lenMap3}')
        rospy.loginfo(curMap)
        rospy.loginfo(curMap2)
        rospy.loginfo(curMap3)
        
    curMap3.dropna(subset=[])
    df_fromSPGMAP=curMap3.sort_values(by='LASTSEEN', ascending=True)
    df_fromSPGMAP.reset_index(inplace=True)
    rospy.loginfo(df_fromSPGMAP)
    currOperationPos = 0

    lsDicArray = []
    iPrevPos = 0

    while currOperationPos != len(df_fromSPGMAP):
        '''
        지시정보를 분석 후 변환하여 새로 생성해서 리턴.
        생성할 필드 LASTSEEN,START,HEIGHT,HEIGHT,SCAN_3D,MULTI_CAM,SPD_H,MOVE_,TIER,LENGTH,ANGLE,ACC,DECC
        해서 리턴.
        '''
        dicTmp = {} #필드값을 map 형태로 저장할 임시변수

        #1 - 송신된 지시정보 필드값 수집.
        sLastSeen = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.LASTSEEN.name)
        sStart = str(df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.START.name))
        iLENGTH = int(df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.LENGTH.name))
        sProfile = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.PROFILE.name)
        sHEIGHT = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.HEIGHT.name)
        sTier = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.TIER.name)
        sSPD_H = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.SPD_H.name)
        sSPD_D = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.SPD_D.name)
        sSPD_U = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.SPD_U.name)

        sACC_H = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.ACC_H.name)
        sACC_V = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.ACC_V.name)
        sDECC_H = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.DECC_H.name)
        sDECC_V = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.DECC_V.name)

        sACT_H = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.ACT_H.name)
        sACT_D = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.ACT_D.name)
        sACT_U = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.ACT_U.name)
        

        #2 - 지시정보 해석하여 새로운 지시정보 생성, 수평운동 map 개체 생성 후 복사하여 수직운동 속성에도 반영.
        # LASTSEEN	START  SCAN_3D	MULTI_CAM	SPD_H	MOVE_	TIER	LENGTH	ANGLE	ACC	DECC
        # 형태로 생성되어야 함.
        
        #2-1 수평운동 지시정보부터 생성
        dicTmp[SPG_Keys.LASTSEEN.name] = f'{sLastSeen}00'
        dicTmp[SPG_Keys.START.name] = sStart
        dicTmp[SPG_Keys.SPD_H.name] = sSPD_H
        dicTmp[SPG_Keys.TIER.name] = sTier    
        dicTmp[SPG_Keys.LENGTH.name] = iLENGTH    
        dicTmp[SPG_Keys.ACC.name] = sACC_H
        dicTmp[SPG_Keys.DECC.name] = sDECC_H
        dicTmp[SPG_Keys.ACT.name] = sACT_H
        dicTmp[SPG_Profile.PROFILE.name] = sProfile
        
        if iLENGTH < iPrevPos:
            dicTmp[SPG_Keys.MOVE_.name] = 'L'
        else:
            dicTmp[SPG_Keys.MOVE_.name] = 'R'
        lsDicArray.append(dicTmp)
        
        currOperationPos +=1    
        if sHEIGHT == '0':
            continue
        
        iPrevPosV = 0
        #2-2 하강운동 지시정보 파싱
        lsHeight = sHEIGHT.split(sep='/')
        iVCnt = 0
        for curHeight in lsHeight:
            iVCnt +=1
            dicTmpD = copy.deepcopy(dicTmp)
            padLeftStr = str(iVCnt).rjust(2,'0')
            dicTmpD[SPG_Keys.LASTSEEN.name] = f'{sLastSeen}{padLeftStr}'
            #dicTmpD[SPG_Keys.TIER.name] = iVCnt    
            dicTmpD[SPG_Keys.TIER.name] = sTier    
            iLENGTH_V = try_parse_int(curHeight,-1)
            if iLENGTH_V == -1:
                iLENGTH_V = param_VMax_Limit        
            elif abs(iLENGTH_V) < 10000:
                iLENGTH_V = (iLENGTH_V*param_VEncoderPulse) + (param_VEncoderPulse*20)
            dicTmpD[SPG_Keys.LENGTH.name] = str(iLENGTH_V)
                            
            dicTmpD[SPG_Keys.ACC.name] = sACC_V
            dicTmpD[SPG_Keys.DECC.name] = sDECC_V
            dicTmpD[SPG_Keys.ACT.name] = sACT_D
            dicTmpD[SPG_Keys.MOVE_.name] = dirCaption_D
            dicTmpD[SPG_Keys.SPD_H.name] = sSPD_D
            lsDicArray.append(dicTmpD)

        #2-3 상승운동 지시정보 파싱
        dicTmpU = copy.deepcopy(dicTmp)
        dicTmpU[SPG_Keys.LASTSEEN.name] = f'{sLastSeen}99'
        dicTmpU[SPG_Keys.SPD_H.name] = sSPD_U
        dicTmpU[SPG_Keys.TIER.name] = sTier
        dicTmpU[SPG_Keys.LENGTH.name] = param_VMin_Limit #맨 위로 올린다
        dicTmpU[SPG_Keys.ACC.name] = '1000'
        dicTmpU[SPG_Keys.DECC.name] = '300'
        dicTmpU[SPG_Keys.ACT.name] = sACT_U
        dicTmpU[SPG_Keys.MOVE_.name] = dirCaption_U
        dicTmpU[SPG_Keys.SPD_H.name] = sSPD_U
        lsDicArray.append(dicTmpU)
        

    df = pd.DataFrame(lsDicArray)
    rospy.loginfo(df)
    return df

def callbackCmd(data):
    global LastActionCmd
    global cmdIDCurrent
    global dicReport
    global drvID_current
    global mapping
    global dicRFID
    global dic_CurrentNode
    global currOperationPos
    global isOperationNow
    global suspend
    global needToHome
    global dirCaption_REP
    global param_VEncoderPulse
    global param_VMax_Limit
    global param_VMin_Limit
    global param_HEncoderPulse
    global param_HMax_Limit
    global param_HMin_Limit
    global df_fromSPGMAP
    global skip_V_Direction

    try:
        success = False
        recvData = data.data
        last_dish_washed = ''
        #rate = rospy.Rate(1)
        recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        rospy.loginfo(recvDataMap)
        if cmdIDCurrent != '':
            SendFeedbackFinish(CmdStatus.Canceled.name,f'{cmdIDCurrent} is not valid msg id.')
            cmdIDCurrent = ''
        cmdIDCurrent = sID = getValueFromMap(recvDataMap, 'ID')

        curLoc = GetCurLocH()
        isdigit_PosH = is_digit(curLoc)
        sDir = recvDataMap.get(SPG_Keys.MOVE_.name, '')
        iACC = recvDataMap.get(SPG_Keys.ACC.name, '0')
        iDECC = recvDataMap.get(SPG_Keys.DECC.name, '0')
        ''' 초기 레일 RFID 태그 매핑 '''

        #messagepub : String = String()

        ''' 회피동작 '''
        if SPG_CMD.AVOID.name in recvDataMap.keys() or sDir == dirCaption_NODE: #현재 작업 노드 변경
            avoidValue = '1'
            if sDir == dirCaption_NODE:
                sSPD = recvDataMap[SPG_Keys.SPD_H.name]
                currOperationPos = int(sSPD)
            else:
                avoidValue = recvDataMap[SPG_CMD.AVOID.name]

            #requestFlag_SetHomeV = False

            if isTrue(avoidValue):
                if suspend:
                    return

                SetSuspend(True)
                ''' 일단 정지 '''
                motorStop(dirCaption_S,True)
                # strResultH,strResultV = isMotorFinished(None)
                # motorStop(dirCaption_S,False)
                # tmpLocalV = (int)(GetCurLocV())
                # tmpLocalH = (int)(GetCurLocH())

                # if not IsAutoOperation():
                #     motorStop(drvCaption_H,False)
                #     return

                # if not strResultH:
                #     motorStop(drvCaption_H,False)
                # elif not strResultV or abs(tmpLocalV) > 5000:
                #     ''' 일단 회피기동 '''
                #     motorStop(drvCaption_V,False)
                #     returnDockOLD(10)
                    # ''' 앞단의 노드 중 주행방향이 L이나 R인 노드로 거슬러 올라간다.'''
                    # len_df_fromSPGMAP = len(df_fromSPGMAP)
                    # if len_df_fromSPGMAP > 0:
                    #     bNodeFound = False
                    #     iCntTmp = 1
                    #     while not bNodeFound:
                    #         iIdxSearch = currOperationPos - iCntTmp
                    #         dictInfo = getNodeInfo(iIdxSearch)
                    #         node_sDir =dictInfo.get(SPG_Keys.MOVE_.name)
                    #         if node_sDir == dirCaption_R or node_sDir == dirCaption_L:
                    #             currOperationPos = iIdxSearch-1
                    #             bNodeFound = True
                    #         else:
                    #             iCntTmp += 1
            else:
                SetSuspend(False)
            return

        # if 'MAP' in recvDataMap.keys():
        #     return
        #curLoc = getValueFromMap(dic_485,curLoc_H)
        # ''' Homing 상태면 모든 메세지는 캔슬한다. 나중에 손 보자-_- '''
        # if requestFlag_SetHomeV or requestFlag_SetHomeH:
        #     SendFeedbackFinish(CmdStatus.Canceled.name,cmdIDCurrent)
        #     cmdIDCurrent = ''
        #     return

        if SPG_CMD.MAPPING.name in recvDataMap.keys():
            if IsDockingH():
                rospy.loginfo('Mapping Start.')
                rospy.loginfo(dicRFID)
                dicRFID.clear()
                SetLimit(0,0, drvCaption_H)
                SendFeedbackFinish(CmdStatus.Started.name,cmdIDCurrent)
                motorMove(SetHomeRange,cali_rpm_H * 10,dirCaption_Foward, False, None,None)
                mapping = True
            else:
                SendFeedbackFinish(CmdStatus.Canceled.name,f'{cmdIDCurrent}_매핑전 충전소에 도킹시키세요')
                rospy.loginfo('Not Docked')
            return
        
        ''' 하강 노드 스킵. RFID 태그 점검용 '''
        if SPG_CMD.SKIPV.name in recvDataMap.keys():
            SKIPVValue = recvDataMap[SPG_CMD.SKIPV.name]
            rospy.loginfo(f'SKIPVValue : {SKIPVValue}')
            if isTrue(SKIPVValue):
                skip_V_Direction = True
            else:
                skip_V_Direction = False
            return

        ''' 네스트 아두이노 명령어 서비스 호출 CMDARD '''
        if SPG_CMD.ARD_N.name in recvDataMap.keys():
            cmd_ard = recvDataMap[SPG_CMD.ARD_N.name]
            service_setbool_client(ServiceList.CMDARD.value, cmd_ard, Kill)
            return

        ''' 유틸박스 아두이노 명령어 서비스 호출 CMDARD_UTIL '''
        if SPG_CMD.ARD_U.name in recvDataMap.keys():
            cmd_ard = recvDataMap[SPG_CMD.ARD_U.name]
            service_setbool_client(ServiceList.CMDARD_UTIL.value, cmd_ard, Kill)
            return

        ''' 오토포커스 사진 촬영 '''
        if SPG_CMD.CAM_T.name in recvDataMap.keys():
            cmd_ard = recvDataMap[SPG_CMD.CAM_T.name]
            CamTest(f'TEST_{GetCurLocH()}_{GetCurLocV()}', cmd_ard)
            return

        ''' 데모 반복 '''
        if SPG_CMD.REP.name in recvDataMap.keys():
            dirCaption_REP = recvDataMap[SPG_CMD.REP.name]
            rospy.loginfo(f'Repeat Demo Value : {dirCaption_REP}')
            return

        if SPG_CMD.RETURN.name in recvDataMap.keys(): #원점으로 복귀명령 - 복귀 플래그 On 시키고 리턴.
            needToHome = True
            return
        #     #if tmpData == drvCaption_H or tmpData == drvCaption_V:
        #     if tmpData == drvCaption_V:
        #         requestFlag_SetHomeV = True
        #     elif tmpData == drvCaption_H:
        #         requestFlag_SetHomeH = True
        #         InitMap()
        #     return

        if SPG_CMD.MAP.name in recvDataMap.keys():
            #if remain_nodes ==0:
            if True:
                tmpData = recvDataMap[SPG_CMD.MAP.name]
                InitMap()
                dicReport.clear()
                dicReport[ReportFields.DATE.name] = try_parse_int(getCurrentDate('')) #작업일자
                dicReport[ReportFields.TIMESTAMP_START.name] = try_parse_int(getCurrentTime('')) #작업시작시각
                dicReport[ReportFields.VOLTAGE_START.name] = try_parse_float(UIN24V) #작업시작 전압
                dicReport[ReportFields.VOLTAGE_END.name] = -1 #작업종료 전압
                dicReport[ReportFields.IMG_TRIED.name] = 0 #촬영 목표치
                dicReport[ReportFields.IMG_PASSED.name] = 0 #촬영 성공
                dicReport[ReportFields.IMG_SKIPPED.name] = 0 #촬영 실패
                #dicReport[ReportFields.IMG_SKIPPED_LOG.name] = '' #촬영 실패사유
                curMap : pd.DataFrame = pd.DataFrame()
                FromExcel = False

                CamON = True
                #회귀명령어 세트인경우 로컬 로드
                if tmpData == 'HOME':
                    tmpData = getStr_FromFile(filePath_CMD_RECONNECT)                    
                    curMap =str2frame(tmpData,'\t')
                elif len(tmpData) > 100:
                    curMap = str2frame(tmpData,sDivSlash,sDivSemiCol,True)
                else:
                    curMap = pd.read_excel(sExcelMapPath, engine = "openpyxl", index_col = 0)
                    FromExcel = True
                
                sPROFILE = curMap.loc[currOperationPos].get(SPG_Profile.PROFILE.name)
                if sPROFILE != None and not FromExcel:
                    curMap = GetFullMapInfo(curMap)
                curLen = len(curMap)
                if  curLen > 100:
                    curMap.to_excel(sExcelMapPath)
                else:   
                    CamON = False
                #CamEnable(CamON)
                MapMerge(curMap,True)
                #SaveExcel()
                #df_fromSPGMAP = str2frame(tmpData,'/',sDivSemiCol,True)
                SendFeedbackFinish(CmdStatus.Started.name,cmdIDCurrent)
            else:
                SendFeedbackFinish(CmdStatus.Canceled.name,cmdIDCurrent)
            #항목 ENUM 인덱스 생성하기.
            #START 열만 골라서 확인한다.
            #list_STOP_SPG = df[SPG_Keys.STOP.name].values.tolist()
            #list_STOP_RFID = df_fromRFID[SPG_Keys.STOP.name].values.tolist()
            #differ_all = set(list_STOP_SPG) ^ set(list_STOP_RFID)


            # print(list_STOP_SPG)
            # print(list_STOP_RFID)
            # if len(differ_all) == 0:
            #     print(differ_all)
            #     df_fromSPGMAP = df
            #     cmdIDCurrent = ''


            # print(df_fromSPGMAP)
            # print(df)

            # TESTDATA = StringIO("""col1;col2;col3
            #     1;4.4;99
            #     2;4.5;200
            #     3;4.7;65
            #     4;3.2;140
            #     """)
            #if os.
            #df = pd.read_csv(filePath_map_default)
            rospy.loginfo(df_fromSPGMAP)
            return

        sRange = recvDataMap.get(SPG_Keys.LENGTH.name,'') + '000'
        sSPD = recvDataMap.get(SPG_Keys.SPD_H.name,'')
        iACC = recvDataMap.get(SPG_Keys.ACC.name, '0')
        iDECC = recvDataMap.get(SPG_Keys.DECC.name, '0')
        
        if sDir in drvDic.keys():
            drvAddr = drvDic[sDir]
            drvID_current = EncoderDic[drvAddr]
        DirUpper = sDir.upper()
        LastActionCmd = sDir
        lastActionTimeStr = str(lastActionTime).replace(':','').replace('-','')
        currentTimeStr = str(getDateTime()).replace(':','').replace('-','')
        sendbuf = f'ID:{cmdIDCurrent},STATUS:ACK,CURRTIME:{currentTimeStr.split()[1]},RECVTIME:{lastActionTimeStr.split()[1]},LASTCMD:{LastActionCmd}'
        #lastActionTime = getDateTime()
        #SetLastActionTime()
        SendFeedback(sendbuf)

        ''' 명령어 Q - 모터 초기화 '''
        if sDir == dirCaption_Q:
            SetHome()
            # if sSPD == '0':
            #     SetHome()
            # else:
            #     SetHomeTest()

        # ''' 명령어 Q - V모터 초기화 '''
        # if sDir.startswith('ONOFF'):
        #     iCurrent = sDir.split('_', 1)

        # if sDir == dirCaption_H:
        #     InitMap()
        #     suspend = False
        #     returnDock(1)
        #     returnNest(1)

        ''' 명령어 Limit 지정 및 기본값으로 저장'''
        if sDir == dirCaption_VMAX:
            tmpLocal = (int)(GetCurLocV())
            param_VMax_Limit = abs(tmpLocal)
            SetLimit(param_VMin_Limit, param_VMax_Limit,drvCaption_V)
        if sDir == dirCaption_HMAX:
            tmpLocal = (int)(GetCurLocH())
            param_HMax_Limit = tmpLocal
            SetLimit(param_HMin_Limit, param_HMax_Limit,drvCaption_H)
        if sDir == dirCaption_HMIN:
            tmpLocal = (int)(GetCurLocH())
            param_HMin_Limit = tmpLocal
            SetLimit(param_HMin_Limit,param_HMax_Limit,drvCaption_H)
        if sDir == dirCaption_SAVE:
            SaveConfig()

        if sDir == 'NESTCHARGE_NC':
            ChargeNestEnable(not isTrue(sSPD))
        if sDir == 'BREAK_V_NC':
            BreakEnable(not isTrue(sSPD))
        if sDir == 'ONOFF_V24':
            Power24V(isTrue(sSPD))
        if sDir.startswith('LOCK'):
            PowerLock(isTrue(sSPD))
        if sDir == 'ONOFF_V16':
            Power16V(isTrue(sSPD))
        if sDir == dirCaption_NOLIMIT_H:
            SetLimit(0,0,drvCaption_H)
        if sDir == dirCaption_NOLIMIT_V:
            SetLimit(0,0,drvCaption_V)
        if sDir == dirCaption_TILT: #모터 틸팅
            service_setbool_client(ServiceList.CMDARD_UTIL.value, f'S:{sSPD}', Kill)

        ''' 중지되었던 모터를 다시 구동 (설정된 거리까지) '''
        if sDir == dirCaption_M:
            MotorResume()
        elif sDir == dirCaption_B or sDir == dirCaption_F or sDir == dirCaption_S:
            if sDir == dirCaption_S:
                ModbusInitAll(filePath_ModbusSetupInit, sDivItemComma,None)
                motorStop(sDir, False)
                SendFeedbackFinish(CmdStatus.Finished.name,cmdIDCurrent)
            else:
                motorStop(drvID_current, False)
                if isOperation(): #자동운행중 수동정지 명령어가 들어오면 알람ON
                    SetSuspend(True)
                SendFeedbackFinish(CmdStatus.Finished.name,cmdIDCurrent)
            cmdIDCurrent = ''
        elif sDir == dirCaption_R or sDir == dirCaption_L or sDir == dirCaption_U or sDir == dirCaption_D:
            motorMove(sRange, sSPD, sDir,False,iACC,iDECC)

    except Exception as e:
        SetAlarm(False)
        message = traceback.format_exc()
        #SendFeedback(message)
        rospy.logdebug(message)
        #EndThread()
        #print (e)
        SendFeedback(e)
rospy.Subscriber(publish_topic_goal, String, callbackCmd)
SendFeedbackFinish('0', 'Boot Start')

lastUpdateBMS = getDateTime()
def getSERVOData(instrumentCurrent:minimalmodbus.Instrument) -> list:
    global CMD_Queue
    global instruments
    global remain_nodes
    global suspend
    global lastUpdateBMS
    global dirCaption_REP
    global df_fromSPGMAP
    global filePath_CMD_RECONNECT
    global currOperationPos
    
    resultList = []
    cmd_buf = None
    infoStr = ''
    isFirstAccess = True
    try:
        while len(CMD_Queue) > 0:
            cmd_buf = list(CMD_Queue.popleft())
            addr = cmd_buf[0]
            valueList = cmd_buf[1]
            drvID = cmd_buf[2]
            caller = cmd_buf[3]
            drvAddr = drvDic[drvID]
            infoStr = f'{addr}:{valueList}:{drvID} from {caller}'
            instrumentTmp : minimalmodbus.Instrument= instruments[drvID]
            chkStatusH,chkStatusV = getMultiEx(dic_485,'TARG')
            #가로 주행전 확인해야할 사항
            #
            # if drvID == drvCaption_H and len(valueList) > 7 and chkStatusH == strREADY: 
            #     if (not IsDockingV()) and runFromLaunch: #도킹이 되지 않은 상태이고 정상운영모드에서 좌우 명령어가 들어오면 도킹먼저
                    
            #         time.sleep(1)
            #         if chkStatusV == strREADY: #현재 V 가 운행중이 아니라면 상승명령어 전송
            #             instruments[drvCaption_V].write_registers(6008,[15]) 
            #             time.sleep(write_delay)
            #             instruments[drvCaption_V].write_registers(6000,[64536, 6000, 0, 300, 0, 500, 0, 125, 95])
            #             print('도킹되지 않은 상태에서는 좌우로 움직일 수 없습니다. 먼저 도킹을 시도합니다.')
            #         else:
            #             print('도킹 진행중 입니다.')
            #         break
            #     safetyVoltage = rospy.get_param("safetyVoltage", default=27)
            #     #if runFromLaunch and float(UIN5V) < safetyVoltage and not IsDockingH() and len(df_fromSPGMAP) > 5:
            #     if float(UIN24V) < safetyVoltage and not IsDockingH() and len(df_fromSPGMAP) > 5: #테스트용 코드-_- 왔다갔다 지시정보 돌리다가 33볼트 아래로 떨어지면 가동되겠지 뭐;
            #         curidx = currOperationPos if currOperationPos == 0 else currOperationPos -1
            #         sStart = str(df_fromSPGMAP.loc[curidx][SPG_Keys.START.name])
            #         if sStart != 'RESET':
            #             #스마트 충전기능 구현
            #             #한참 일하고 있는데 안전 전압보다 낮아진 상태면?복귀할 것
            #             msgErr = f'현재전압:{UIN24V}V 이 안전전압 {safetyVoltage}V 보다 낮습니다. 스마트 충전을 시작합니다.'
            #             rospy.loginfo(msgErr)
            #             #1.지시정보를 편집한다. -> 복귀해야할 지점 윗부분 지시정보를 모두 삭제, 
            #             myArray = list(range(0, currOperationPos-1))
            #             if len(myArray) > 0 and currOperationPos > 0:
            #                 df_fromSPGMAP.drop(df_fromSPGMAP.index[myArray], inplace=True)
            #             #2.컴백해야할 지시정보를 남은 지시정보 앞단에 배치한다
            #             tmpData = getStr_FromFile(filePath_CMD_GETHOME)                    
            #             curMap =str2frame(tmpData,'\t')
            #             MapMerge(curMap,False)
            #             print(df_fromSPGMAP)
            #             currOperationPos = 0
            #             #3.현재 명령어 큐를 다 비우고 반복 플래그는 끈다
            #             dirCaption_REP = '0'
            #             CMD_Queue.clear()
            #             print('맵 수정 완료!')
            #             break
                    
                # #if runFromLaunch and float(UIN5V) < startVoltage and IsDockingH():
                # if IsDockingH(): #디버그용-_-
                #     if float(UIN24V) < startVoltage:
                #         msgErr = f'현재노드인덱스:{currOperationPos},현재전압:{UIN24V}V 이 안전전압 {startVoltage}V 보다 낮아 출발할 수 없습니다.'
                #         ChargeNestEnable(True)
                #         if remain_nodes > 0: #자동운행 모드일때는 익셉션
                #             #아래 부분도 Suspend 타입을 다양화 하자. - 전압알람, 근접알람, 진동알람 등.
                #             #SetSuspend(True)
                #             time.sleep(10)
                #             raise Exception(msgErr)
                #         # else: #수동운행일때는 에러메세지만 내고 수행하지 않음 -> 수동명령어는 늘 되어야 함.
                #         #     rospy.loginfo (f'{msgErr} from {sys._getframe(0).f_code.co_name}')
                #         #     continue
                #     else:
                #         ChargeNestEnable(False)
            rospy.loginfo (f'Trying to write : {infoStr}')
            cmd_buf = None
            if len(valueList) == 1:
                instrumentTmp.write_register(addr,valueList[0])
                isFirstAccess = False
                time.sleep(write_delay)
            else:
                if not isFirstAccess:
                    time.sleep(0.1)
                instrumentTmp.write_registers(addr,valueList)
                time.sleep(write_delay)
    except Exception as e:
        if cmd_buf != None:
            CMD_Queue.appendleft(cmd_buf)
            rospy.loginfo(f'write_registers error : {cmd_buf} - {e}')
    try:
        if not isFirstAccess:
            time.sleep(0.1)
        lsALMItem = ServoMonitor.ServoMonitorStatus_Addrs.value
        len_items = len(lsALMItem)
        #start_addr = ServoMonitor.ServoMonitorStatus_Addrs.value[0]
        resultList =instrumentCurrent.read_registers(start_addr, len_items, 3)
        #print(type(resultList)) -> <class 'list'>
    except Exception as e:
        #rospy.loginfo(f'{sys._getframe(0).f_code.co_name} - {e}')
        pass
    td = getDateTime() - lastUpdateBMS
     
    
    if td.total_seconds() > 1:
        try:
            lastUpdateBMS = getDateTime()
            time.sleep(write_delay * 1)
            resultBMS =instrumentBMS.read_registers(0x1000,lenthList , 3)
            if lenthList == len(resultBMS):
                for i in range(len(listTmp)):
                    if ( str( type( listTmp[i] ) ) != "<class 'int'>"):
                        BMS_Stat[listTmp[i]] = resultBMS[i]
                numberOfCells = BMS_Stat.get(listTmp[0])
                if numberOfCells != 8:
                    InitBMS()
                else:
                    try:
                        lock.acquire()
                        dic_485.update(BMS_Stat)
                    finally:
                        lock.release()
            else:
                rospy.loginfo(f'BMS Polling Error : {resultBMS}')
        except Exception as ex:
            rospy.loginfo (f'{ex} - Try Failed in Read BMS\r\n{traceback.format_exc()}\r\n')
            time.sleep(write_delay*1)
    return resultList

if not runFromLaunch:
    Power24V(True)

last485ResetTime = getDateTime()
    # 당분간 스무드 옵션은 무시하자c. e_stop 으로 모두 통일
    # if td.total_seconds() < 1: #이전 Stop 과 시간차가 1초 이내인 경우는 에러메세지 출력후 리턴
err485Cnt = 0
def ModbusSetup(Force24V_Reset) -> bool:
    global instrumentH
    global instrumentV
    global instruments
    global instrumentBMS
    global servo_keepalive
    global list_Alarm
    global last485ResetTime
    global err485Cnt
    bReturn = False
    try:
        if Force24V_Reset:
            rospy.loginfo(Power24V(False))
            time.sleep(3)
            rospy.loginfo(Power24V(True))
            time.sleep(6)
            #list_Alarm.clear()

        instrumentH = minimalmodbus.Instrument(port485, drvDic[drvCaption_H],minimalmodbus.MODE_RTU)
        instrumentH.serial.close()
        instrumentV = minimalmodbus.Instrument(port485, drvDic[drvCaption_V],minimalmodbus.MODE_RTU)
        instrumentV.serial.close()
        #instrumentV.clear_buffers_before_each_transaction = True
        #instrumentH.clear_buffers_before_each_transaction = True
        instrumentV.serial.parity = instrumentH.serial.parity = serial.PARITY_NONE
        instrumentV.serial.stopbits = instrumentH.serial.stopbits = 2
        instrumentV.serial.timeout = instrumentH.serial.timeout = 0.5  # seconds
        instrumentV.serial.baudrate = instrumentH.serial.baudrate = 9600  # Baud

        instrumentBMS = minimalmodbus.Instrument(port485,1,minimalmodbus.MODE_RTU)
        instrumentBMS.serial.close()
        instrumentBMS.clear_buffers_before_each_transaction = True
        instrumentBMS.serial.bytesize = instrumentV.serial.bytesize = instrumentH.serial.bytesize = 8

        instrumentBMS.serial.baudrate = 9600
        instrumentBMS.serial.parity = serial.PARITY_NONE
        instrumentBMS.serial.stopbits = 2
        instrumentBMS.serial.timeout = 0.5
        msg = f'{sys._getframe(0).f_code.co_name} called from {sys._getframe(1).f_code.co_name}'
        rospy.loginfo(msg)
        instruments = {drvCaption_H :instrumentH, drvCaption_V:instrumentV}
        time.sleep(1)
        resultTest = getSERVOData(instrumentV)
        rospy.loginfo(f'Init OK - {sys._getframe(0).f_code.co_name}, {resultTest}')
        servo_keepalive = 0
        bReturn = True
    except Exception as e:
        rospy.loginfo(f'{servo_keepalive}-{sys._getframe(0).f_code.co_name} - {e}')
        keyIdx = str(e).find('485')
        #485통신 포트가 사망한 경우 리셋한다.
        if keyIdx > 0 and err485Cnt > 10:
            td =  getDateTime() - last485ResetTime
            if td.total_seconds() > 20:
                os.system('~/resetall.sh')
                last485ResetTime= getDateTime()
                err485Cnt = 0
                ModbusInitAll(filePath_ModbusSetupInit, sDivItemComma,None)
    SetAlarm(bReturn)
    return bReturn


def WriteReg(addr, value,drvID):
    listTmp = []
    listTmp.append(value)
    return WriteRegEx(addr,listTmp,drvID)
    # if drvID is None or drvID == '':
    #     rospy.loginfo('No DrvID~!')
    #     return
    # bPass = False
    # valueInt = (int)(value)
    # loop =1
    # iCnt = 1
    # while loop != 0:
    #     try:
    #         rospy.loginfo(f'Trying count {iCnt} - {addr:#02X}:{valueInt:#02X}:{drvID}')
    #         instrumentTmp = instruments[drvID]
    #         instrumentTmp.write_register(addr,valueInt, 0,6)
    #         #print (f'WriteReg addr : {addr} to {value}')
    #         bPass = True
    #         loop = 0
    #         rospy.loginfo (f'Trying OK! {iCnt} - {addr}:{valueInt}:{drvID}')
    #     except Exception as e:
    #         rospy.loginfo(f'Try Failed at {iCnt}\r\n{e}\r\n{traceback.format_exc()}')
    #         #rospy.loginfo (f'Try Failed at {iCnt}\r\n{e}\r\n')
    #         #print(e)
    #         #EndThread()
    #         time.sleep(write_delay*2)
    #     if bPass:
    #         break
    #     else:
    #         time.sleep(write_delay)
    #         loop = loop - 1
    #         iCnt = iCnt + 1
    # #if bPass is False:
    # #    Reset485(1)
    # rospy.loginfo(f'WriteReg : {bPass} at {iCnt} - {addr},{valueInt},{drvID} from {sys._getframe(1).f_code.co_name}-{sys._getframe(0).f_code.co_name}')
    # return bPass

def WriteRegEx(addr : int, valueList : List ,drvID:str) -> bool:
    global cmdIDCurrent
    global CMD_Queue

    if drvID is None or drvID == '':
        SendData()
        rospy.loginfo(f'No DrvID~! : {sys._getframe(2).f_code.co_name} - {sys._getframe(1).f_code.co_name}')
        return False
    bPass = False
    cmd_buf = []
    cmd_buf.append(addr)
    cmd_buf.append(valueList)
    cmd_buf.append(drvID)
    cmd_buf.append(f'{sys._getframe(2).f_code.co_name} - {sys._getframe(1).f_code.co_name}')
    CMD_Queue.append(tuple(cmd_buf))
    return bPass

def ModbusInitAll(strFilePath, strSpliter, EncoderID):
    global instruments
    msg = f'{sys._getframe(0).f_code.co_name} called from {sys._getframe(1).f_code.co_name}'
    rospy.loginfo(msg)
    bReturn = False
    try:
        file_list = getLines_FromFile(strFilePath)
        for i in file_list:
            iCurrent = i.split(sDivItemComma, 1)
            if len(iCurrent) > 1:
                k = iCurrent[0]
                v = iCurrent[1]
                listCur = [k,v]
                listConverted = []
                # for curTmp in listCur:
                #     chkNumber = if_Number(curTmp)
                #     if chkNumber is True:
                #         chkHex = if_hex(curTmp)
                #         if chkHex is True:
                #             listConverted.append(int())
                # curVal = curKey = None

                keyType = if_Number(k)
                valType = if_Number(v)
                if keyType and valType:
                    curKey = int(k,0)
                    curVal = int(v,0)
                    if EncoderID == drvCaption_V or EncoderID == drvCaption_H:
                        WriteReg(curKey,curVal,EncoderID)
                    else:
                        for drvID in EncoderCurrent.keys():
                            WriteReg(curKey,curVal,drvID)
        bReturn = True
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
    SetAlarm(bReturn)
    return True


'''
instrumentH = minimalmodbus.Instrument(port485, drvDic[drvCaption_H],'rtu')
instrumentV = minimalmodbus.Instrument(port485, drvDic[drvCaption_V],'rtu')

instrumentV.serial.baudrate = instrumentH.serial.baudrate = 57600  # Baud
instrumentV.serial.bytesize = instrumentH.serial.bytesize = 8
instrumentV.serial.parity = instrumentH.serial.parity = serial.PARITY_NONE
instrumentV.serial.stopbits = instrumentH.serial.stopbits = 1
instrumentV.serial.timeout = instrumentH.serial.timeout = 1  # seconds
instruments = {"H" :instrumentH, "V":instrumentV}
instrument_Stat = {"H" : {}, "V":{}}

#추후 재접속이 필요한 경우를 위해 참조용으로 남겨둔 코드.
#기존 USB Serial 에서는 리셋시 232포트가 목록에서 없어졌다가 다시 나타나는 일이 잦았음.
def InitSDRetry():
    bOK = False
    sdPort = port485Header
    while bOK == False:
        try:
            rospy.loginfo(f'Chekc serial/485 port... {sdPort}')
            if serBMS != None:
                serBMS.close()
            serBMS = serial.Serial(sdPort, baud, timeout=None)
            bOK = True
        except Exception as e:
            print(f'Failed to Open port {sdPort} - {e}')
            time.sleep(1)
    global pub
    global rate
    rospy.init_node(nodeName, anonymous = False,log_level=rospy.DEBUG)
    pub = rospy.Publisher(publish_topic_name, String, queue_size=1)
    rate = rospy.Rate(param_SD_Rate)

InitSDRetry()
'''

def motorStop(drvID, bSmooth):
    global lastStopTime
    global lastLD06Time
    global mapping
    global SpdBoost
    global SpdDown
    SpdBoost = False
    SpdDown = False
    td = getDateTime() - lastStopTime
    # 당분간 스무드 옵션은 무시하자c. e_stop 으로 모두 통일
    # if td.total_seconds() < 1: #이전 Stop 과 시간차가 1초 이내인 경우는 에러메세지 출력후 리턴
    #     msg = f'Too short interval:{td.total_seconds()} from {sys._getframe(1).f_code.co_name}-{sys._getframe(0).f_code.co_name}'
    #     rospy.logdebug(msg)
    #     return False
    lastStopTime = getDateTime()
    isAlarmNow : bool = chkAlarm(drvID)
    # STOP 을 Alarm Clear 기능으로 사용한다. 아래는 리드샤인 모터.
    if isAlarmNow:
        if drvID == dirCaption_S:
            WriteReg(ServoKey.CLEAR_ALARM.value ,ServoParam.CLEAR_ALARM_NOW.value,drvCaption_H)
            WriteReg(ServoKey.CLEAR_ALARM.value ,ServoParam.CLEAR_ALARM_NOW.value,drvCaption_V)
        else:
            WriteReg(ServoKey.CLEAR_ALARM.value ,ServoParam.CLEAR_ALARM_NOW.value,drvID)
        time.sleep(write_delay)
        #알람클리어 이후에는 초기화를 다시 해줘야 한다.
        #ModbusInitAll(filePath_ModbusSetupInit, sDivItemComma,drvID)

    if drvID == dirCaption_S:
        WriteReg(e_stop_addr,e_stop_param,drvCaption_H)
        RFIDControl(False)
        time.sleep(0.1)
        WriteReg(e_stop_addr,e_stop_param,drvCaption_V)
    else:
        if drvID == drvCaption_H:
            RFIDControl(False)
            if bSmooth: #속도변경하는 경우
                lsRTUParam = [0,0]
                WriteRegEx(ServoMonitor.ServoMonitorField_SpeedAddr.value,lsRTUParam,drvCaption_H)
                WriteReg(e_stop_addr,e_spd_change,drvCaption_H)
        
        if drvID == '':
            WriteReg(e_stop_addr,e_stop_param,drvCaption_H)
            WriteReg(e_stop_addr,e_stop_param,drvCaption_V)
        else:
            WriteReg(e_stop_addr,e_stop_param,drvID)
        #WriteReg(e_stop_addr,e_stop_param,drvIDMoving)
        #if drvIDMoving is not None:
        #    WriteReg(e_stop_addr,e_stop_param,drvIDMoving)

    # if drvID == drvCaption_V:# or drvID == drvCaption_B:
    #     if bSmooth is False:
    #         rospy.logdebug('No BreakV - Smooth mode')
    #     else:
    #         #GPIO.output(PIN_BCM.BREAK_O.value, True)
    #         rospy.logdebug('BreakV - Hard')
    #         #Break_V(True)
    if mapping:
    #if True:
        #dicRFID = {'303443D11C27104000000710': 28575, '303443D11C27104000000741': 73566, '303443D11C27104000000745': 136583}
        if len(dicRFID) > 0:
            mapping = False
            dicRFIDTEMP = {}
            dicRFIDFinal = {}
            for k,v in dicRFID.items():
                if v in dicRFIDTEMP.keys():
                    dicRFIDTEMP[v].append(k)
                else:
                    dicRFIDTEMP[v] = []
                    dicRFIDTEMP[v].append(k)
            
            for k,v in dicRFIDTEMP.items():
                sum = 0
                for strPos in v:
                    sum = sum + int(strPos)
                average = sum / len(v)
                dicRFIDFinal[k] = int(average)
            rospy.loginfo(f'Save the map of {len(dicRFIDFinal)}: {dicRFIDFinal}')
            saveDic_ToFile(dicRFIDFinal, filePath_map_default, sDivEmart)
            #     velKey = sorted(dicRFID.values(), reverse=False)
    rospy.loginfo(f'Stop OK : Check Safety : {CheckSafetyLidar()}')
    return True

def chkAlarm(drvID):
    alarmIs = getValueFromMap(dic_485, f'FAULT_{drvID}')
    if alarmIs == '1' or alarmIs == '0':
        WriteReg(modbusOpmode, 15, drvID)
    return alarmIs == '1'

def motorMove(Range, Spd, Dir, isAbsolutePos,iACC : str,  iDECC : str):
    #msg = f'{sys._getframe(0).f_code.co_name} => Range:{Range},Spd:{Spd},Dir:{Dir},isAbsolutePos:{isAbsolutePos} from {sys._getframe(1).f_code.co_name}-{sys._getframe(2).f_code.co_name}'
    msg = f'{sys._getframe(0).f_code.co_name} => Range:{Range},Spd:{Spd},Dir:{Dir},isAbsolutePos:{isAbsolutePos} from {sys._getframe(1).f_code.co_name}'
    rospy.loginfo(msg)
    if Range == None:
        return False
    global EncoderCurrent
    global instrument_Stat
    global LastActionCmd
    global dockH_time
    #global lastActionTime
    global cmdIDCurrent
    DirUpper = Dir.upper()
    iPosType = ServoParam.MOVE_TYPE_REL.value
    dwellTime = 0
    pathNumber = 0x10
    EncoderID = ''

    #가감속 - 작을수록 스무드 / 15000 이면 풀악셀, 급브레이크, 보통 500
    #2023-01-06 전시회에서 대표님이 정한 가감속 1000,1000
    accTime = 1000
    decTime = 1000
    if Dir == dirCaption_L or Dir == dirCaption_R:
        dockH_time = getDateTime()
        EncoderID = drvCaption_H
        RFIDControl(True)
        #정방향일때는 충전단자를 내린다.
        #복귀방향일때는 충전단자를 올린다.
        #ChargeNestEnable(Dir == dirCaption_Backward)
        #if Dir == dirCaption_Foward:
        ChargeNestEnable(False)
            
    # elif Dir == dirCaption_R:
    #     ChargeNestEnable(False)
    #     EncoderID = drvCaption_H
    #     RFIDControl(True)
    elif Dir == dirCaption_D:
        #수직운동의 경우 가감속 파라미터를 다르게 준다.
        accTime = 500
        decTime = 500
        EncoderID = drvCaption_V
        #ChargeNestEnable(True)
    elif Dir == dirCaption_U:
        accTime = 500
        decTime = 125
        EncoderID = drvCaption_V

    try:
        sACC = str(iACC)
        sDECC = str(iDECC)
        if iACC != None and sACC.isdecimal():
            accTime = int(sACC)
        if iDECC != None and sDECC.isdecimal():
            decTime = int(sDECC)
    except Exception as e:    
        msg = f'Error {e} from {sys._getframe(0).f_code.co_name}'
        rospy.loginfo(msg)

    sLogMsg ='도킹되어있는 상태에서는 위로 올라가는 명령어는 취소한다.'
    if dirCaption_U == Dir and IsDockingV() and CheckSafetyLidar():
        SendFeedbackFinish(CmdStatus.Canceled.name,f'{cmdIDCurrent}_{sLogMsg}')
        return False

    sLogMsg =' 충전중에 백워드 명령어는 취소한다.'
    if dirCaption_Backward == Dir and IsDockingH():
        SendFeedbackFinish(CmdStatus.Canceled.name,f'{cmdIDCurrent}_{sLogMsg}')
        return False

    if chkAlarm(EncoderID): #체크 알람은 추후 구현. 현재는 알람이 없으면 1, 알람이 있으면 0 리턴, 에러코드 체크로 바꾸면 될 듯.
        SendFeedbackFinish(CmdStatus.Alarm.name,cmdIDCurrent)
        return False
    else:
        ''' 직전 명령어와 동일한 방향코드에 직전 명령어 수신 후 0.5초가 지나지 않은 상태라면 알람  '''
        if LastActionCmd == Dir and isTimeExceeded(lastActionTime, 500) == False:
            SendFeedbackFinish(CmdStatus.Alarm.name,cmdIDCurrent)
            return False

        if isAbsolutePos is True:
            iPosType = ServoParam.MOVE_TYPE_ABS.value
        #motorMove 에서 예외처리로 False 가 되지 않으면 
        SetLastActionTime()
        LastActionCmd = Dir
        
        try:
            iDir = -1
            iRange = int(Range)
            if Dir == dirCaption_L or Dir == dirCaption_U : #모터 구동방향 결정 (시계,반시계)
                if not isAbsolutePos: #절대값 이동모드에서는 모터구동방향 의미가 없다
                    iRange = iRange * iDir
            iSpd = int(Spd)
            if iSpd > spdLimit: #최고속도 제한걸어둠. 최고 1500RPM
                iSpd = spdLimit
            iRange1, iRange2 = splitSignedInt(iRange)
            iRPM = int(map(iSpd,0,100, 0,3000))
            lsRTUParam = [iRange1,iRange2,0,iRPM,0,accTime,0,decTime, iPosType]
            '''이 부분에 V브레이크 컨트롤 추가'''
            if EncoderID == drvCaption_V:
                RFIDControl(False)
                BreakEnable(False)
                Power16V(False)
                RFIDControl(False)
            else:
                chkStatusH,chkStatusV = getMultiEx(dic_485,'TARG')
                rospy.loginfo(f'서보운행현황-수평모터:{chkStatusH},수직모터:{chkStatusV},{currOperationPos}')
                if chkStatusH != strREADY: #서보 운행 도중 속도변경시
                    lsRTUParam = [0,iRPM]
                    WriteRegEx(ServoMonitor.ServoMonitorField_SpeedAddr.value,lsRTUParam,EncoderID)
                    WriteReg(e_stop_addr,e_spd_change,drvCaption_H)
                    return True
                time.sleep(1)
            ''' MODBUS 로 제어명령어 송신'''
            WriteRegEx(ServoMonitor.ServoMonitorField_StartAddr.value,lsRTUParam,EncoderID)
            return True
        except Exception as e:
            #rospy.loginfo(e)
            rospy.loginfo(traceback.format_exc())
            return False

#region CallBack-Interrupt Functions
def CallBackDockInterruptV(channel):
    global LastActionCmd
    global lastActionTime
    global evtDock_V
    global mapping
    isRising = True
    bReturn = False
    global remain_nodes

    td = getDateTime() - lastActionTime
    curLoc = GetCurLocV()
    if evtDock_V.isSet() == False:
        evtDock_V.set()

    ''' 자동주행중인데 마지막 커맨드가 위 방향이 아닌데 올라오면 무효 '''
    #if remain_nodes > 1 and LastActionCmd != dirCaption_U:
    if LastActionCmd != dirCaption_U:
        rospy.loginfo(f'V-Rising edge ignored at LocV {curLoc}, remains are {remain_nodes}, last cmd : {LastActionCmd} - mapping : {mapping}')
        return False
            # if GPIO.input(PIN_BCM.NEST_DOCK_I.value):
    #     rospy.loginfo("V-Rising edge detected")
    # else:
    #     #서보가 주행중이 아닌데 V가 떨어지는 이벤트라면 문제가 있음.
    #     isRising = False
    #     alarmAdmin('서보가 주행중이 아닌데 V가 떨어지는 이벤트라면 문제가 있음.')
    strResult1,strResult2 = isMotorFinished(drvCaption_V)
    try:
        if GPIO.input(PIN_BCM.MAIN_DOCK_I.value):     # if port 19 == 1
            rospy.loginfo(f"V-Rising edge detected - channel:{channel} at LocV {curLoc}, remains are {remain_nodes}, last cmd : {LastActionCmd}")
        else:                  # if port 25 != 1
            isRising = False
        td = getDateTime() - lastActionTime
        # if LastActionCmd != dirCaption_U:
        #     rospy.loginfo(f'Move up command is ignored : {channel},{LastActionCmd}')
        #     SetAlarm(True)
        #     return

        rospy.loginfo(f'현재 주행 모터V : {strResult1}, {strResult2}, 현재 GPIO : {isRising} : 현재 V좌표 : {curLoc} , 남은노드 : {remain_nodes}')
        if td.total_seconds() > 0.1 and not mapping:
            BreakEnable(True)
            motorStop(drvCaption_V,False)
            LastActionCmd = dirCaption_F
            lastActionTime = getDateTime()
            rospy.loginfo(f'Mainbody attached - {channel}, isRising : {isRising}, Last ActionCmd is {LastActionCmd}, curLoc = {curLoc}')
            if float(UIN24V) > safetyVoltage: #안전 전압을 넘을때만 네스트에서 본체를 충전한다
                Power16V(True)
            #현재 좌표를 초기화 한다.
            if curLoc != '0':
                SetHomeV()
        else:
            msg = f'mapping : {mapping} or too short interval:{td.total_seconds()} from {sys._getframe(0).f_code.co_name}'
            rospy.loginfo(msg)
        bReturn = True
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
    SetAlarm(bReturn)
    return bReturn

# def CallBackDockInterruptH(channel):
#     global iCntLoop
#     global lastEPC
#     global dic_485
#     global LastActionCmd
#     global dicRFID
#     global mapping
#     global total_length_rail
#     global suspend
#     global lastActionTime
#     global remain_nodes
#     global needToHome
#     global SpdBoost
#     global currOperationPos
#     isRising = True
#     td = getDateTime() - lastActionTime
#     strResult1,strResult2 = isMotorFinished(drvCaption_V)
#     #print(strResult1)
#     bReturn = False

#     if evtDock_H.isSet() == False:
#         evtDock_H.set()

#     statusH = GPIO.input(PIN_BCM.NEST_DOCK_I.value)
#     rospy.loginfo(f'H-Rising edge detected - Status : {statusH} , LastActionCmd = {LastActionCmd}')
#     isRising = statusH
#     # if statusH == False:
#     #     rospy.loginfo("H-Rising edge detected")
#     # else:                  # if port 25 != 1
#     #     isRising = False
#     if LastActionCmd != dirCaption_Backward:
#         rospy.loginfo(f'복귀 방향이 아닌데 신호가 올라오면 노이즈로 간주하여 무시합니다, 채널 = {channel}, statusH = {statusH}, isRising : {isRising}, Last ActionCmd is {LastActionCmd}')
#         return

#     #print(statusH)
#     spdLimit = 1000
#     try:
#         curLoc = GetCurLocH()
#         spdCurrent = abs(int(dic_485.get('CURSPD_H', '0')))
#         if spdCurrent > spdLimit:
#             rospy.loginfo(f'현재속도 {spdCurrent}, 속도가 {spdLimit} 보다 높아 노이즈로 간주하여 무시합니다.')
#             return
#         rospy.loginfo(f'현재 주행 모터H : {strResult1}, {strResult2}, 속도 : {spdCurrent} 현재 GPIO : {statusH} : 현재 H좌표 : {curLoc}, 남은노드 : {remain_nodes}')
#         curLocH_INT = abs((int)(curLoc))
#         #if remain_nodes < 2 or curLocH_INT < 10000:
#         if (remain_nodes == 0 or suspend) and not mapping:
#             if td.total_seconds() > 0.2:
#                 rospy.loginfo(f'네스트 충전소 안착! - {channel}, H Pos : {curLoc}, status = {statusH}, isRising : {isRising}, Last ActionCmd is {LastActionCmd}')
#                 evtDock_H.set()
#                 SpdBoost = False
#                 motorStop(drvCaption_H,True)
#                 SetHomeH()
#                 LastActionCmd = dirCaption_F
#                 #ChargeNestEnable(True)
#                 lastEPC = None
#                 iCntLoop += 1
#                 rospy.loginfo(f'Loop Count : {iCntLoop}')
#                 if suspend:
#                     SetSuspend(False)
#                     #InitMap()
#                 else:
#                     currOperationPos = 0
#             else:
#                 msg = f'Too short interval:{td.total_seconds()} - {sys._getframe(0).f_code.co_name}'
#                 rospy.loginfo(msg)
#             bReturn = True
#         else:
#             rospy.loginfo(f'Excepection Handling {nameof(suspend)}-{suspend} , {nameof(mapping)}-{mapping}')
#     except Exception as e:
#         message = traceback.format_exc()
#         rospy.loginfo(message)
#     SetAlarm(bReturn)
#     return bReturn
#endregion

#region GPIO
def GPIO_Init() -> bool:
    bReturn = False
    try:
        GPIO.setmode(GPIO.BCM)
        for s in PIN_BCM:
            sCurrent = str(s)
            if sCurrent.endswith("I"):
                #GPIO.setup(s.value, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
                GPIO.setup(s.value, GPIO.IN, pull_up_down = GPIO.PUD_UP)
                rospy.loginfo(f'Pin {s.value}:{sCurrent} to Input')
                #GPIO.setup(s.value, GPIO.OUT)  # 서보핀 출력으로 설정
                #print(f'Pin {s.value}:{sCurrent} to Output')
            elif sCurrent.endswith("PWM") or sCurrent.endswith("O"):
            #elif sCurrent.endswith("O"):
                rospy.loginfo(f'Trying for Pin {s.value}:{sCurrent} to Output')
                GPIO.setup(s.value, GPIO.OUT)  # 서보핀 출력으로 설정
        GPIO.add_event_detect(PIN_BCM.MAIN_DOCK_I.value, GPIO.FALLING, callback=CallBackDockInterruptV, bouncetime=gpio_bouncetime)
        #GPIO.add_event_detect(PIN_BCM.NEST_DOCK_I.value, GPIO.FALLING, callback=CallBackDockInterruptH, bouncetime=1000)
        bReturn = True
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
    SetAlarm(bReturn)
    return bReturn

#endregion

# os.system('~/.rfidstart')
# os.system('~/.30105start')
# os.system('~/.imustart')


#함수 정의끝, 초기화 루틴 실행 시작
GPIO_Init()
dockH_time = getDateTime()

def IsDockingH():
    # chargeStatus = GetChargingStatus()
    # isCharging = chargeStatus != CHARGE_Status.DISCHARGING.value and chargeStatus != CHARGE_Status.UNKNOWN.value
    # return isCharging
    isDock = bool(GPIO.input(PIN_BCM.NEST_DOCK_I.value))
    return not isDock

def IsDockingV():
    isDock = bool(GPIO.input(PIN_BCM.MAIN_DOCK_I.value))
    return not isDock

    #isDock = not isDock
    # curV = GetCurLocV()
    # if curV == '':
    #     return isDock
    # iRange = abs(int(curV))
    # if iRange < 7000 or isDock:
    #     return True
    # else:
    #     return False

def IsDockingVSensor():
    return not bool(GPIO.input(PIN_BCM.MAIN_DOCK_I.value))

def IsChargeConnected():
    isConn = bool(GPIO.input(PIN_BCM.NEST_CONNECTOR_I.value))
    return not isConn

def GetChargingStatus():
    curPack = dic_485.get(curPackBMS, '')
    curBMS = str(dic_485.get(curCurrentBMS, ''))
    if curBMS == '' or curPack == '':
        return CHARGE_Status.UNKNOWN.value #상태 알 수 없음

    fCurrent = (float)(curBMS.strip())
    iPackStatus = curPack

    if iPackStatus >= 127:
        return CHARGE_Status.CHARGING.value #충전중
    if iPackStatus <= 3 and fCurrent <= 0:
        return CHARGE_Status.CHARGEFULL.value #완충
    return CHARGE_Status.DISCHARGING.value #방전중

    # if curBMS != None and curBMS != '':
    #     num = (curBMS)
    #     if num > 127 or num == 3:
    #         return True
    # return False

# def IsChargedFULL():
#     curBMS = dic_485.get(curCurrentBMS, '')
#     if curBMS != None and curBMS != '':
#         num = (curBMS)
#         if num == 3:
#             return True
#     return False

def IsBreakON():
    return not bool(GPIO.input(PIN_BCM.BREAK_V_I.value))

rospy.loginfo(f'Charge Dock : {IsChargeConnected()}')
rospy.loginfo(f'Is_V_BreakON : {IsBreakON()}')

def BreakEnable(enable):
    resultCmd = None
    cmd_ard='BV:0' if enable else 'BV:1'
    break_on = IsBreakON()
    if enable == break_on:
        return True
    resultCmd = service_setbool_client(ServiceList.CMDARD.value, cmd_ard, Kill)
    return resultCmd

BreakEnable(True)
# Power24V(False)
# time.sleep(2)
# Power24V(True)
# time.sleep(2)
ModbusSetup(runFromLaunch)
if runFromLaunch:
    ModbusInitAll(filePath_ModbusSetupInit, sDivItemComma,None)
#ModbusInitAll(filePath_ModbusSetupHoming, sDivItemComma, None)

def CamLightEnable(enable):
    #return enable
    cmd_ard='D:7,0' if enable else 'D:7,1'
    resultCmd = service_setbool_client(ServiceList.CMDARD_UTIL.value, cmd_ard, Kill)
    return resultCmd

def TiltBox(start,end):
    enable = start > 0 and end > 0        
    cmd_ard=f'SW:{start},{end}' if enable else f'S:{start}'
    resultCmd = service_setbool_client(ServiceList.CMDARD_UTIL.value, cmd_ard, Kill)
    return resultCmd

def TiltDelay(tiltDelay):   
    cmd_ard=f'SWD:{tiltDelay}'
    resultCmd = service_setbool_client(ServiceList.CMDARD_UTIL.value, cmd_ard, Kill)
    return resultCmd

# TiltBox(30,140)
# TiltDelay(50)
#TiltBox(110,140)
TiltDelay(50)
#TiltBox(140,0)


def ChargeNestEnable(enable):
    cmd_ard='CHARGE:0' if enable else 'CHARGE:1'
    charge_connected = IsChargeConnected()
    if enable == charge_connected:
        return True
    return service_setbool_client(ServiceList.CMDARD.value, cmd_ard, Kill)
dataPrevious = []
CMDARD_UTIL = '/utilbox/CMD'

def SetAlarmLED(enable : LED_Status):
    global dataPrevious
    dataToSend = enable
    if dataPrevious != dataToSend:
        for currentTmp in dataToSend:
            service_setbool_client(ServiceList.CMDARD.value, currentTmp, Kill)
            time.sleep(0.05)
    dataPrevious = dataToSend
    return

def SendData():
    return service_setbool_client(ServiceList.CMDARD_RUNSCRIPT.value, 'sendStart', Kill)

# BreakEnable(True)
# BreakEnable(False)
# ChargeNestEnable(True)
# ChargeNestEnable(False)

rospy.Subscriber(NEST_TOPIC.RFID.value, String, callbackRFID)
rospy.Subscriber(NEST_TOPIC.SD.value, String, callbackSD)
rospy.Subscriber(NEST_TOPIC.UTIL_ARD.value, String, callbackUTIL_ARD)
rospy.Subscriber(NEST_TOPIC.KEEP_ARD.value, String, callbackKEEP_ARD)
rospy.Subscriber(NEST_TOPIC.scan_alarm.value, String, callbackscan_alarm)
rospy.Subscriber(NEST_TOPIC.scheduler.value, String, callback_scheduler)
#맵이 없으면 자동으로 돌아다니며 맵을 생성한다.

def alarmAdmin(strValue : str):
    strFinal = f'-{sys._getframe(0).f_code.co_name}:{strValue}-{sys._getframe(1).f_code.co_name}'
    rospy.logfatal(strFinal)
    SetAlarmLED(LED_Status.ERROR_FATAL.value)

#region 도킹 확인 - 도킹된 상태가 아니면 저속으로 살살 감을것

def returnNest():
    global needToHome
    callData = String()
    SetLimit(0,0, drvCaption_H)
    SetLimit(0,0, drvCaption_V)
    callData.data = 'MOVE_:S'
    callbackCmd(callData)
    callData.data = 'MOVE_:Q'
    callbackCmd(callData)
    callData.data = 'MAP:HOME'
    callbackCmd(callData)    
    needToHome = False
    return

# def returnNest(iSpd):
#     global evtDock_H
#     global requestFlag_SetHomeH
#     # if not IsDockingV():
#     #     return

#     requestFlag_SetHomeH = True
#     return

# def returnDock(iSpd):
#     global evtDock_V
#     global requestFlag_SetHomeV
#     requestFlag_SetHomeV = True
#     return

#     #curLoc = getValueFromMap(dic_485,curLoc_V)
#     curLoc = GetCurLocV()
#     isdigit_PosV = is_digit(curLoc)
#     isDocked = IsDockingV()
#     rospy.loginfo(f'V-Dock status = {isDocked}, curLoc = {curLoc}')
#     if isDocked: #네스트에 도킹으로 안착된 상태
#         if isdigit_PosV:
#             iRange = abs(int(curLoc))
#             if iRange == 0: #엔코더 절대값이 0인 경우는 이미 도킹 완료된 상태이므로 리턴.
#                 rospy.loginfo('V-Dock already finished~!')
#             else:   #엔코더 절대값이 0이 넘는 경우 - 알람,
#                 alarmAdmin('Invalid V-Encoder Value! - SetHome')
#         else:   #네스트에는 안착되어있는데 엔코더 값이 null 인 경우 - 경고메세지만 출력하고 리턴
#             rospy.loginfo('Encoder value was not found, but V-Dock already finished~! Keep going.')
#         SetHomeV() #도킹되어있으면 무조건 0으로 초기화.
#     else:   #네스트 홈 충전기에서 떨어진 상태
#         rospy.loginfo('Going for V-Dock Trigger...')
#         if isdigit_PosV and curLoc != '0': #엔코더 값이 수신 된 경우
#             iRange = int(curLoc)
#             if abs(iRange) > 0: #엔코더 절대값이 0이 넘는 경우
#                 iDir = dirCaption_U if iRange > 0 else dirCaption_D
#                 iRange = abs(iRange)
#                 rospy.loginfo(f'V-Docking Direction : {iDir}, Range : {iRange}')
#                 #motorMove(iRange, iSpd, iDir, False) #복귀 명령 - 상대좌표
#                 motorMove(0, iSpd, iDir, True) #복귀 명령 - 절대좌표 어느것이 좋을지는 실기로 테스트 필요
#         else: #엔코더값이 없는 경우 - 보통 초기화 과정
#             SetLimit(0, 0,drvCaption_V) #Limit 해제.
#             motorMove(SetHomeRange, cali_rpm_V, dirCaption_U, False)

#         evtDock_V.clear()
#         time.sleep(1)
#         isDocked = IsDockingV()
#         if isDocked:
#             rospy.loginfo(f'V-Dock Already Finished , isDocked : {isDocked}')
#         # else:
#         #     evtDock_V.wait(100)
#         #     dockVResult = evtDock_V.isSet()
#         #     rospy.loginfo(f'V-Dock Finished : {dockVResult}')


# def returnDockOLD(iSpd):
#     global evtDock_V
#     isDocked = IsDockingV()
#     rospy.loginfo(f'V-Dock status = {isDocked}')
#     if isDocked:
#         rospy.loginfo('V-Dock already finished~!')
#     else:
#         rospy.loginfo('Waiting for V-Dock Trigger...')
#         motorMove(param_VMin_Limit, iSpd, dirCaption_U, False)
#         evtDock_V.clear()
#         evtDock_V.wait(10)
#         if evtDock_V.isSet():
#             rospy.loginfo(IsDockingV()) #정상작동
#         else:
#             rospy.loginfo(type(IsDockingV())) #Timeout, 관리자 조치 필요.
#         rospy.loginfo('V-Dock Finished~!')

def LoadRailInfo():
    global dicRFID
    global dicSPDMAP
    dicSPDMAP = getDic_FromFile(filePath_mapSPD_default, sDivEmart)
    dicRFID = getDic_FromFile(filePath_map_default, sDivEmart)
    if dicRFID != None and len(dicRFID) > 0:
        rospy.loginfo(f'RFID Info Load : {dicRFID}' )
    else:
        alarmAdmin('RFID Info not found.')
#endregion

#경로정보를 읽어들인다.
LoadRailInfo()


currOperationPos = 0
def getNodeInfo(curPos= None):
    global df_fromSPGMAP
    global currOperationPos
    if curPos == None:
        curPos = currOperationPos
    dicTmp = df_fromSPGMAP.loc[curPos].to_dict()
    return dicTmp

def CheckServoStatus():
    global needToHome
    chkH,chkV = getMultiEx(dic_485,'RTSO')
    if servo_keepalive >= servo_keepalive_MAX or chkH != '1' or chkV != '1':
        return ModbusSetup(False)
        if ModbusSetup(False):
            ModbusInitAll(filePath_ModbusSetupInit, sDivItemComma,drvCaption_H)
            ModbusInitAll(filePath_ModbusSetupInit, sDivItemComma,drvCaption_V)
            #needToHome = True
        return False
    else:
        return True

def loggingCurrentInfo():
    global df_fromSPGMAP
    global currOperationPos
    lenMap = len(df_fromSPGMAP)
    if lenMap != 0:
        idxRate = 100*currOperationPos / lenMap
        rospy.loginfo(f'현재 작업 인덱스 : {currOperationPos} - 진행률 : {idxRate : 0.1f}%, 현재 위치 : {GetCurLocH()},{GetCurLocV()}')

#BreakEnable(False)
CamEnable(False)
CamLightEnable(False)
SetAlarmLED(LED_Status.ON_NODESERVO.value)

#메인바디와 네스트가 분리되어 있는 경우 도킹먼저 시킨다.
if runFromLaunch:
    needToHome = True

while not rospy.is_shutdown():
    bReturn = True
    try:
        dtNow = getDateTime()
        if CheckServoStatus() == False:
            continue

        if servo_keepalive <= servo_keepalive_MAX:
            moving_check = []
            strResult_HV = ''
            len_df_fromSPGMAP = len(df_fromSPGMAP)

            for k,v in EncoderCurrent.items():
                drvID_Tmp = k
                instrumentCurrent = instruments[drvID_Tmp]
                instrumentStatus = instrument_Stat[drvID_Tmp]
                try:
                    resultList=getSERVOData(instrumentCurrent)
                    resultMulti = []
                    #resultMulti.append()

                    for intTmp in resultList:
                        bytesTmp = intTmp.to_bytes(2,byteorder='big')
                        resultMulti.append(bytesTmp[0])
                        resultMulti.append(bytesTmp[1])

                    #resultMulti = bytearray(resultList)
                    if len(resultMulti) > 0:
                        resultMulti.pop()
                        #print(type(resultMulti))
                        resultSingle = int.from_bytes(resultMulti, byteorder='big')
                        dicCurrent = parsing_BitData(resultSingle, start_addr)
                        instrumentStatus = getMergedDic(instrumentStatus, dicCurrent)
                        dicTmp = dicKeysMerge(instrumentStatus)
                        strResult = getStr_fromDic(dicTmp,sDivFieldColon,sDivEmart, True, f'_{drvID_Tmp}')
                        strResult_HV = f'{strResult_HV}{sDivEmart}{strResult}'
                        servo_keepalive = 0
                    else:
                        servo_keepalive += 1
                        rospy.loginfo (f'{servo_keepalive} - Empty Data in Read in {drvID_Tmp}')
                        #time.sleep(write_delay*1)
                except Exception as e:
                    servo_keepalive += 1
                    rospy.loginfo (f'{servo_keepalive} - Try Failed in Read\r\n{traceback.format_exc()}\r\n')
                    #time.sleep(write_delay*1)
                time.sleep(write_delay*2)
            #현재 운영상태값 확인
            #TARG_H 와 TARG_V 가 모두 1이고 dfSequence 의 길이가 0 이어야 READY 상태이다.
            dicServo = getDic_strArr(strResult_HV, sDivFieldColon, sDivEmart)
            H_pos_prev = dic_485.get(curLoc_H, '')
            try:
                lock.acquire()
                dic_485.update(dicServo)
            finally:
                lock.release()

            isOperationNow : bool = isOperation()
            #isOperationNow : bool = not(len_df_fromSPGMAP == 0 and getValueFromMap(dic_485,targ_H) == '1' and getValueFromMap(dic_485,targ_V) == '1')
            #strResult_HV = f'OPMODE{sDivField}{int(isOperationNow)}{sDivEmart}REMAIN_NODES{sDivField}{len(dfSequence)}{strResult_HV}'
            #dic_485 = getMergedDic(dic_485,getDic_strArr(strResult_HV, sDivFieldColon, sDivEmart))
            dic_485[PIN_BCM.MAIN_DOCK_I.name] = int(IsDockingV())
            dic_485[PIN_BCM.NEST_DOCK_I.name] = int(IsDockingH())
            dic_485[PIN_BCM.BREAK_V_I.name] = int(IsBreakON())

            dic_485['OPMODE'] = int(isOperationNow)
            dic_485['SUSPEND'] = int(suspend)
            remain_nodes = len_df_fromSPGMAP - currOperationPos

            dic_485['REMAIN_NODES'] = remain_nodes
            dic_485['RFID_ERR_DISTANCE'] = f'{differenceMM : 0.2f}'
            dic_485['CHARGE_STATE'] = GetChargingStatus()
            dic_485['LIDAR_ERR'] = lidarErrPoint
            dic_485[oldLoc_H] = H_pos_prev
            
            # dic_485['ROS_STATUS'] = rospy.get_master().getPid()
            # dic_485['ROS_NODES'] = rosnode.get_node_names()
            # dic_485['ROS_NAME'] = rospy.get_name()

            #curLoc = dic_485.get(curLoc_H)
            curLocH = GetCurLocH()
            
            curLocH_INT = 0
            curLocH_LENGTH = 0
            if curLocH != '':
                curLocH_INT = (int)(curLocH)
                curLocH_LENGTH = curLocH_INT / param_HEncoderPulse

            curLocV = GetCurLocV()
            curLocV_INT = 0
            curLocV_LENGTH = 0

            ischarge16 = dicSD.get('ONOFF_V16' , '')

            if curLocV != '':
                curLocV_INT = abs((int)(curLocV))
                curLocV_LENGTH = round(curLocV_INT / param_VEncoderPulse)

            if bPublishEncoder:
                encoderStr = f'{curLocH_LENGTH}{sDivEmart}{curLocV_LENGTH}'
                pub_encoder.publish(encoderStr)

            if evtDock_H.isSet() == False and curLocH == '0':
                evtDock_H.set()

            if evtDock_V.isSet() == False and curLocV == '0':
                evtDock_V.set()
            
            # if requestFlag_SetHomeV or requestFlag_SetHomeH:
            #     suspend = False
            
            #dic_485['REMAIN_NODES'] = len(dfSequence)
            td = getDateTime() - lastLogTime
            ChargingStatusCur = GetChargingStatus()
            if td.total_seconds()  > 1:
                rospy.loginfo(f'Test ismoving : {IsMoving()}-{dockH_time}, LastActionTime:{lastActionTime}')
                #운영리포트와 아두이노 정보는 1초에 한번씩 업데이트 해서 내보낸다.
                if len(dicARD_N) > 0:
                    try:
                        lockARD_N.acquire()
                        dic_485.update(dicARD_N)
                    finally:
                        lockARD_N.release()
                
                if len(dicARD_U) > 0:            
                    try:
                        lockARD_U.acquire()
                        dic_485.update(dicARD_U)
                    finally:
                        lockARD_U.release()
                
                if len(dicReport) > 0:            
                    try:
                        lockARD_U.acquire()
                        dic_485.update(dicReport)
                    finally:
                        lockARD_U.release()                
                
                #루틴이 끝났는데 일정 시간동안 움직임이 없으면 충전소로 강제이동한다.
                if isTimeExceeded(lastActionTime, param_Return_Time*1000*60) and not IsMoving() and not IsDockingH():
                #if isTimeExceeded(lastActionTime, param_Return_Time*1000*60) and not isMoving():
                    #엔코더값 확인
                    chargeStatus = GetChargingStatus()
                    if (chargeStatus == CHARGE_Status.DISCHARGING.value or chargeStatus == CHARGE_Status.UNKNOWN.value):
                        SetLastActionTime()
                        returnNest()
                        rospy.loginfo(f'{param_Return_Time}분이 지나 충전하러 복귀합니다 - 현재 충전상태 : {chargeStatus}')
                    else:
                        rospy.loginfo(f'chargeStatus = {chargeStatus}, curLocH == {curLocH}, lastActionTime = {lastActionTime}')
                
                td2 = dtNow - utilard_keepalive
                # if requestFlag_SetHomeH or  requestFlag_SetHomeV:
                #     SetAlarmLED(LED_Status.ON_DATA.value) #위치 캘리중 - 그린 블링크
                if needToHome and len(CMD_Queue) ==0:
                    returnNest()
                elif suspend:
                    #SetAlarmLED(LED_Status.ON_OPERATION.value) #자동운전중 - 그린 순차점등
                    SetAlarmLED(LED_Status.ON_REDSOLID.value) #자동운전중 - 그린 
                elif remain_nodes > 0 and curLocV_INT < 1000:
                    #SetAlarmLED(LED_Status.ON_OPERATION.value) #자동운전중 - 그린 순차점등
                    SetAlarmLED(LED_Status.ON_GREENSOLID.value) #자동운전중 - 그린 
                elif remain_nodes > 0 and curLocV_INT >= 1000:
                    SetAlarmLED(LED_Status.ON_BLUESOLID.value) #자동운전중 - 그린 순차점등
                elif IsDockingH() and ChargingStatusCur == CHARGE_Status.DISCHARGING.value:
                    SetAlarmLED(LED_Status.ERROR_CHARGE.value) #도킹은 되었지만 충전은 안됨 - 레드 순차점등
                elif ChargingStatusCur == CHARGE_Status.CHARGEFULL.value:
                    SetAlarmLED(LED_Status.ON_CHARGEFULL.value) #완충 - 그린 솔리드
                elif ChargingStatusCur == CHARGE_Status.CHARGING.value:
                    SetAlarmLED(LED_Status.CHARGING.value) #충전중 - 레드 솔리드
                elif td2.total_seconds() > 5:
                    SetAlarmLED(LED_Status.ON_NODESERVO.value) #RPI만 로드됨 - 블루순차점등
                else:
                    SetAlarmLED(LED_Status.ON_UTILBOXLOADED.value) #유틸박스에서도 로드 완료 - 블루솔리드, RPI 노드가 죽을 경우 블루 블링크로 바뀜
                #strResult = strResult_HV[1:]

        #if rospy.has_param(nameof(param_BMS_Show)): #bms 스캔 여부
        #Tmp = rospy.get_param(nameof(param_BMS_Show))
                if param_BMS_Show:
                    for drvID in dicParamParse.keys():
                        if drvID in dic_485:
                            beforeParse = dic_485[drvID]
                            fomula = dicParamParse[drvID]
                            fomulaFinal = f'{beforeParse}{fomula}'
                            FinalValue = eval(fomulaFinal)
                            dic_485[drvID]=f'{FinalValue : 0.2f}'

                strResult = getStr_fromDic(dic_485,sDivFieldColon,sDivEmart)
                data_out=json.dumps(dic_485)
                if param_DRV_show:
                    rospy.loginfo(strResult)
                pub_kpalive.publish(strResult)
                client.publish(topic, data_out)
                lastLogTime = getDateTime()
            #1초마다 무조건 수행하는 루틴 끝
            chkStatusH,chkStatusV = getMultiEx(dic_485, 'TARG')
            # #하방운동시 라이다.
            # if LastActionCmd == dirCaption_D and not CheckSafetyLidar(V_close_alert):
            #     motorStop(drvCaption_V,False)
            #     SetSuspend(True)
            
            # 홈 명령어로 대체.
            # if requestFlag_SetHomeV and len(CMD_Queue) ==0:
            #     SetLimit(0,0,drvCaption_V)
            #     motorStop(dirCaption_S,False)                
            #     tmpData = getStr_FromFile(filePath_CMD_RECONNECT)
            #     InitMap()
            #     MapAppend(str2frame(tmpData,'\t'))
            #     requestFlag_SetHomeV = False
                # if IsDockingV():
                #     ModbusInitAll(filePath_ModbusSetupHoming, sDivItemComma, drvCaption_V)
                #     SetLimit(param_VMin_Limit, param_VMax_Limit,drvCaption_V) #Limit 설정.
                #     requestFlag_SetHomeV = False
                # else:
                #     #BreakEnable(False)
                #     if CallBackDockInterruptV.__name__ in list_Alarm:
                #         motorMove(param_VMin_Limit, cali_rpm_V, dirCaption_U, True)
                #     else:
                #         Power16V(False)
                #         ModbusInitAll(filePath_ModbusSetupHoming, sDivItemComma, drvCaption_V)
                #         SetLimit(0, 0,drvCaption_V) #Limit 해제.
                #         motorMove(SetHomeRange, cali_rpm_V, dirCaption_U, False)
                #     requestFlag_SetHomeV = False
            # elif requestFlag_SetHomeH:
            #     if remain_nodes ==0 and IsDockingV():
            #         requestFlag_SetHomeH = False
            #         if IsDockingH():
            #             SetHomeH()
            #         else:
            #             SetLimit(0,0,drvCaption_H)
            #             motorStop(dirCaption_S,False)
            #             tmpData = getStr_FromFile(filePath_CMD_GETHOME)
            #             MapAppend(str2frame(tmpData,'\t'))

                        # if CallBackDockInterruptH.__name__ in list_Alarm:
                        #    motorMove(0, cali_rpm_H * 8, dirCaption_Backward, True)
                        # else:
                        #     ''' 충전 도크 해제 '''
                        #     # curLoc = GetCurLocH()
                        #     # if curLoc == '':
                        #     #     ModbusInitAll(filePath_ModbusSetupHoming, sDivItemComma, drvCaption_H)
                        #     #     motorMove(SetHomeRange, cali_rpm_H, dirCaption_L, False)
                        #     # else:
                        #     #     motorMove(0, cali_rpm_H, dirCaption_L, True)
                        #     ModbusInitAll(filePath_ModbusSetupHoming, sDivItemComma, drvCaption_H)
                        #     SetLimit(0, 0,drvCaption_H) #Limit 해제.
                        #     if not motorMove(SetHomeRange, cali_rpm_H * 8, dirCaption_Backward, False):
                        #         continue
                        #requestFlag_SetHomeH = False
                # else:
                #     SetLimit(0,0,drvCaption_V)
                #     tmpData = getStr_FromFile(filePath_CMD_RECONNECT)
                #     df_fromSPGMAP = str2frame(tmpData,'\t')
                    # if CallBackDockInterruptV.__name__ in list_Alarm:
                    #     tmpData = getStr_FromFile(filePath_CMD_RECONNECT)
                    #     df_fromSPGMAP = str2frame(tmpData,'\t')
                    #     requestFlag_SetHomeV = False



            #RFID가 켜져있으면 RFID를 끈다.
            # param_InvRFID = rospy.get_param(nameof(param_InvRFID))
            # if param_InvRFID and chkStatusH == strREADY:
            #     RFIDControl(False)

            #세이프티 - 도킹된 상태인데 chkStatusH 나 chkStatusV 가 1이 아닌 경우
            # if IsDockingH() == True and chkStatusH != strREADY:
            #     print(drvCaption_H, '세이프티 - 도킹된 상태인데 chkStatusH 나 chkStatusV 가 1이 아닌 경우')
            #     motorStop(drvCaption_H, True)
            # elif IsDockingV() == True and chkStatusV != strREADY:
            #     print(drvCaption_V, '세이프티 - 도킹된 상태인데 chkStatusH 나 chkStatusV 가 1이 아닌 경우')
            #     motorStop(drvCaption_V, True)
            
            #센서 고장으로 인해 주석처리 
            if IsDockingH():
                #motorStop(drvCaption_H, False)
                td = getDateTime() - dockH_time                
                # cuV = try_parse_float(UIN24V)
                # sus = safetyVoltage > cuV
                # SetSuspend(sus)                
                if td.total_seconds() > 3:
                    if abs(int(GetCurLocH())) > 1000:
                        motorStop(drvCaption_H,True)
                        SetHomeH()
                    if not IsChargeConnected():
                        ChargeNestEnable(True)
                    #dockH_time = getDateTime()
            else:
                dockH_time = getDateTime()
            
                    
            # else: #멈추고 지시정보도 없는데 충전중이 아니면 복귀해야 함.
            #     ModbusInitAll(filePath_ModbusSetupInit, sDivItemComma,None)
                
            if IsDockingV() and ischarge16 != '1':
                if float(UIN24V) > safetyVoltage: #안전 전압을 넘을때만 네스트에서 본체를 충전한다
                    Power16V(True)
            
            #자동주행 운행부분
            if (chkStatusH == strREADY) and strREADY == chkStatusV: #양쪽 모터가 모두 strReady 이면 정지한 상태 
                if not suspend  and len(CMD_Queue) ==0: #회피기동중인 경우 대기한다.
                    len_df_fromSPGMAP = len(df_fromSPGMAP)
                    if len_df_fromSPGMAP > currOperationPos and cmdIDCurrent == '':
                        sMovePrev_ = dirCaption_F
                        sLastSeenPrev = sTierPrev = sStartPrev = '0000'
                        sTier = str(df_fromSPGMAP.loc[currOperationPos].get(SPG_Keys.TIER.name,''))
                        sStart = str(df_fromSPGMAP.loc[currOperationPos].get(SPG_Keys.START.name,''))
                        sLastSeen = str(df_fromSPGMAP.loc[currOperationPos].get(SPG_Keys.LASTSEEN.name,''))
                        sLENGTH = str(df_fromSPGMAP.loc[currOperationPos].get(SPG_Keys.LENGTH.name,''))
                        sSPD = str(df_fromSPGMAP.loc[currOperationPos].get(SPG_Keys.SPD_H.name,''))
                        sMOVE_ = str(df_fromSPGMAP.loc[currOperationPos].get(SPG_Keys.MOVE_.name,''))                       
                        sACC = str(df_fromSPGMAP.loc[currOperationPos].get(SPG_Keys.ACC.name,None))
                        sDECC = str(df_fromSPGMAP.loc[currOperationPos].get(SPG_Keys.DECC.name,None))

                        if currOperationPos > 0 :
                            sMovePrev_ = str(df_fromSPGMAP.loc[currOperationPos-1].get(SPG_Keys.MOVE_.name,''))
                            sStartPrev = str(df_fromSPGMAP.loc[currOperationPos-1].get(SPG_Keys.START.name,''))
                            sLastSeenPrev = str(df_fromSPGMAP.loc[currOperationPos-1].get(SPG_Keys.LASTSEEN.name,''))
                            sTierPrev = str(df_fromSPGMAP.loc[currOperationPos-1].get(SPG_Keys.TIER.name,''))

                        iTier = try_parse_int(sTier)
                        iLastSeenDetail = try_parse_int(sLastSeenPrev[-2:])

                        #액션필드 : sACT 예제값 : T100;135;20/S , 1차구분자 / 2차구분자 ;
                        #1차 구분자로 나눈 뒤 첫 글자로 명령어 구분하고, 두번째 글자부터는 2차구분자로 나눈다
                        sACT = df_fromSPGMAP.loc[currOperationPos].get(SPG_Keys.ACT.name,None)
                        sLogMsg = f'라이다:{chkLidar}-{sLastSeen}'
                        
                        #라이다 세이프티 체크 : 이전 운동이 수평이동이고 이번 운동이 D 라면 내려가기전에 검사해야함
                        if sMovePrev_ == dirCaption_Foward and sMOVE_ == dirCaption_D:
                            if skip_V_Direction:
                                chkLidar = False
                            else:
                                chkLidar = CheckSafetyLidar(param_VMax_Limit)
                                if not chkLidar:
                                    CamLightEnable(True)
                                    TiltBox(35,0)
                                    iColumnNumber = int(int(sLastSeen) / 100)
                                    prefixStr = f'{sStart}_ERROR_{iColumnNumber}_V0'
                                    CamAuto(prefixStr,1)
                                    dicReport[ReportFields.IMG_SKIPPED.name] += sLogMsg
                                
                        elif sMOVE_ == dirCaption_Foward or sMOVE_ == dirCaption_Backward and sStart != 'RESET':                            
                            loggingCurrentInfo()
                            if abs(int(GetCurLocV())) > 6001:
                                motorMove(param_VMin_Limit,'10',dirCaption_U, True, '500','100')
                                continue
                            #if float(UIN24V) < safetyVoltage and not isCharging:
                            if float(UIN24V) < safetyVoltage and not IsDockingH():
                                rospy.loginfo(f'긴급복귀전압 : {safetyVoltage}, 스마트 충전 모드 On 현재전압: {UIN24V}, 구동가능 전압 : {startVoltage}')
                                motorMove('99999999','50',dirCaption_Backward, False, '500','100')
                                continue
                            if float(UIN24V) < startVoltage and IsDockingH():
                            #if float(UIN24V) < startVoltage and isCharging:
                                rospy.loginfo(f'현재전압: {UIN24V}, 구동가능 전압 : {startVoltage}')
                                time.sleep(2)
                                continue
                            else:
                                rospy.loginfo(f'현재전압: {UIN24V},구동가능전압 : {startVoltage},긴급복귀전압 : {safetyVoltage}')
                            chkLidar = True
                        
                        if sACT != None and chkLidar:
                            file_list = str(sACT).split(sep='/')
                            for i in file_list:
                                if len(i) > 0:
                                    cmd = i[:1]
                                    cmd_param = i[1:]

                                    if cmd == 'L':
                                        CamLightEnable(isTrue(cmd_param))
                                    elif cmd == 'S' and iLastSeenDetail > 0:
                                        LidarEnable(isTrue(cmd_param))
                                    #elif cmd == 'C' and iLastSeenDetail > 0:
                                    elif cmd == 'C':
                                        val = try_parse_int(cmd_param, None)
                                        iV = (iLastSeenDetail % iTier)
                                        if iV == 0:
                                            iV= iTier
                                        iLevel = int((iLastSeenDetail - 1) / iTier) + 1

                                        if val == 0: #연사
                                            CamEnable(True)
                                            dicReport[ReportFields.IMG_PASSED.name] += 1
                                        elif val == None: #연사종료
                                            CamEnable(False)
                                        else: #연사
                                            iColumnNumber = int(int(sLastSeen) / 100)
                                            prefixStr = f'{sStart}_{iLevel}_{iColumnNumber}_V{iV}'
                                            CamAuto(prefixStr,1)
                                            #CamFocus(prefixStr,val)
                                            dicReport[ReportFields.IMG_PASSED.name] += 1
                                            dicReport[ReportFields.IMG_TRIED.name] += 1
                                            #dicReport[ReportFields.IMG_SKIPPED.name] += 1

                                    elif cmd == 'T':
                                        param_list = cmd_param.split(sep=';')
                                        len_param_list = len(param_list)
                                        if len_param_list == 1:
                                            TiltBox(int(param_list[0]), 0)
                                        elif len_param_list == 2:
                                            TiltBox(int(param_list[0]), int(param_list[1]))
                                        elif len_param_list == 3:
                                            TiltDelay(int(param_list[2]))
                                            TiltBox(int(param_list[0]), int(param_list[1]))

                        dic_CurrentNode=getNodeInfo()
                        rospy.loginfo(df_fromSPGMAP.loc[currOperationPos])
                        #chkLidar = CheckSafetyLidar(int(sLENGTH))
                        if chkLidar == False and sMOVE_ == dirCaption_D and sStart != 'RESET':
                            currOperationPos += 1
                            dicReport[ReportFields.IMG_TRIED.name] += 1
                            dicReport[ReportFields.IMG_SKIPPED.name] += 1
                            lidarErrPoint += f'{sStart} '
                            SendFeedbackFinish(CmdStatus.Canceled.name,sLogMsg)
                            #이 부분에 실패 노드 로그 기록!                            
                        # elif skip_V_Direction and (sMOVE_ == dirCaption_D):
                        #     dicReport[ReportFields.IMG_TRIED.name] += 1
                        #     dicReport[ReportFields.IMG_SKIPPED.name] += 1
                        #     currOperationPos += 1
                        #     SendFeedbackFinish(CmdStatus.Canceled.name,sLastSeen)
                        else:
                            cmdIDCurrent = sLastSeen
                            SendFeedbackFinish(CmdStatus.Started.name,sLastSeen)
                            if chkLidar:
                                motorMove(sLENGTH,sSPD,sMOVE_, True, sACC,sDECC)
                            currOperationPos += 1

                            if len_df_fromSPGMAP == currOperationPos:
                                if dicReport.get(ReportFields.VOLTAGE_END.name,'0') == -1:
                                    dicReport[ReportFields.VOLTAGE_END.name] = try_parse_float(UIN24V)
                                    dicReport[ReportFields.TIMESTAMP_END.name] = try_parse_int(getCurrentTime(''))
                                    SaveExcel()
                                    rospy.loginfo(f'Job Result : {dicReport}')

                                rospy.loginfo('자동운행 끝')
                                if len_df_fromSPGMAP < 5 or dirCaption_REP == '0': #초기 컬큘레이트 지시
                                    InitMap()
                                    CamEnable(False)
                                else:
                                    currOperationPos = 0

                                SendData()
                else:
                    if IsMovingH():
                        SetLastActionTime(False)                    
                        if IsChargeConnected():
                            ChargeNestEnable(False)
                if cmdIDCurrent != '' and iCnt_MovingStamp > 0:
                #if cmdIDCurrent != '':
                    iStampTmp = iCnt_MovingStamp
                    iCnt_MovingStamp = 0
                    SendFeedbackFinish(f'{CmdStatus.Finished.name}_{iStampTmp}',cmdIDCurrent)
                    cmdIDCurrent = ''
                else:
                    iCnt_MovingStamp = iCnt_MovingStamp+1
            else:   ## 주행중이 아닐때만 체크할 것  
                if IsMovingH():
                    SetLastActionTime(False)
                    if IsChargeConnected():
                        ChargeNestEnable(False)

                # elif not IsDockingH() and needToHome:
                #     requestFlag_SetHomeH = True
                #     needToHome = False
                # elif not IsDockingH() and isTimeExceeded(lastActionTime, restart_Time) and not mapping:
                #     rospy.signal_shutdown('Self Recovery')
                '''여기에 전압체크 조건 추가 '''
                #SetLastActionTime(False)

            if len(moving_check) == 1:
                drvIDMoving = moving_check[0]
            elif len(moving_check) == 0:
                drvIDMoving = None
            else:
                drvIDMoving = dirCaption_B

        else:
            #timeout 이 나서 서보를 다시 초기화 해야할 상황.
            bReturn = False
    except Exception as e:
        bReturn = False
        rospy.loginfo(traceback.format_exc())
        rospy.signal_shutdown(e)
        #sCmd = '/root/.rrStart -&'
        #os.system(sCmd)

    SetAlarm(bReturn,param_DRV_show)
    if isTimeExceeded(lastUpdateAlarm, 1000):
        SendAlarm(list_Alarm)
        lastUpdateAlarm = getDateTime()
    rate.sleep()