#!/usr/bin/env python3
import bluetooth
from threading import Thread
import threading
import rosclean
import schedule
import numpy
from curses import keyname
from random import randint
from curses.ascii import isdigit
from nturl2path import pathname2url
import gc
from signal import alarm
import sys
import traceback
import os
from uuid import getnode
import rosnode
import rospy
from varname import *
import time
from sensor_msgs.msg import LaserScan  # LaserScan 메시지 사용준비
from std_msgs.msg import String
import minimalmodbus as minimalmodbus
import serial
from Util import *
from std_srvs.srv import *
from rospy_tutorials.srv import *

# from SPG_Keys import *
# from ServoNano import *
# import seaborn as sns
import pandas as pd
import numpy as np
from io import StringIO
from UtilBLB import *
import rosparam
import roslib
from collections import deque
from turtlesim.srv import *
from tta_blb.srv import *
import rosparam

"""
구분자 정의 : 
1차 구분자 - 이마트 어퍼스트로피 `
2차구분자 - 콜론 :
3차구분자 - 쉼표 (레지스터 단위 구분)
4차구분자 - 슬래쉬 : 레지스터 연속값 구분
"""

dicSICode = {}
dicSICode[0] = 'DEFAULT'
dicSICode[136] = 'ICS'
dicSICode[33] = 'HOME'
dicSICode[34] = 'ESTOP'
dicSICode[37] = 'POT'
dicSICode[38] = 'NOT'
dicSI_TotalStatus = {}

CMD_Queue = deque()  # 모드버스 통신 동기화를 위한 Write 버퍼
# publish_topic_goal  : str = 'CMD' #테스트용 변수
# publish_topic_name = 'MB'
# publish_topic_ACK = 'ACK'
runFromLaunch = rospy.get_param(f"~{ROS_PARAMS.startReal.name}", default=False)
nodeName = f"node_BMS"
rospy.init_node(nodeName, anonymous=False)  # 485 이름의 노드 생성
fastModbus = isTrue(rospy.get_param("~fastModbus", default=False))
lsSlowDevices = ["1"]
machineName = GetMachineStr()
rospy.loginfo(f"Modbus Fast:{fastModbus},Machine Name:{machineName}")
# time.sleep(10)

param_DRV_Rate = 100
port485 = "/dev/ttC485"
if fastModbus:
    if machineName == UbuntuEnv.ITX.name:
        port485 = "/dev/ttyS0"  # 시리얼 포트
        # port485 = "/dev/ttyUSB0"  # 시리얼 포트
    else:
        port485 = "/dev/ttC485"  # USB 포트

# dirPath = os.path.dirname(__file__)
dic_485ctl = {}
dic_485cmd = {}
dic_485poll = {}
dic_485Inverted = {}
dic_485POTInfo = {}
dic_485pollLen = {}
dic_485pollRate = {}
dic_topics = {}
dicConfigTmp = {}
dicConfigInitReverse = {}
dicCali = {}  # 캘리브레이션 변수 저장.
dicCaliFinalPos = {}  # 캘리브레이션 변수 저장.

dicCaliPotEncoder = {}  # 스텝별 엔코더값 저장.
dicCaliNotLoad = {}  # 스텝별 엔코더값 저장.
dicCurrentEncoder = {}  # 모터별 엔코더값 저장.
dicInitTimeStamp = {}  # 모터별 초기화 명령 호출 시각 저장.
dic_485ack = {}
# dic_485topic = {}
rate = rospy.Rate(param_DRV_Rate)
poll_common = [POLL_COMMON.START.name, POLL_COMMON.RATE.name, POLL_COMMON.LEN.name]
# poll_START = poll_common[0]
# poll_RATE = poll_common[1]
# poll_LEN = poll_common[2]
# motorOpCheck = {}
 
# BMS
lastLogTime = getDateTime()
lock = threading.Lock()
lockPollingTime = threading.Lock()
iTryCheckModbus = 0

def WriteRegEx(addr: int, valueList: List, drvID: str) -> bool:
    global CMD_Queue
    bPass = True
    cmd_buf = []
    try:
        cmd_buf.append(addr)
        cmd_buf.append(valueList)
        cmd_buf.append(drvID)
        cmd_buf.append(
            f"{sys._getframe(2).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
        )
        CMD_Queue.append(tuple(cmd_buf))
    except Exception as e:
        bPass = False
        rospy.loginfo(traceback.format_exc())
        rospy.signal_shutdown(e)
    return bPass


def WriteReg(addr, value, drvID):
    listTmp = []
    listTmp.append(value)
    return WriteRegEx(addr, listTmp, drvID)


bReset = False


def callbackCmd(data):
    # TODO : 응답메세지 발행하는 것도 만들기.
    global dic_485ack
    global pub_flag
    global lock
    global bReset
    global dicSetupRequested
    global dicCali
    dtnow = 1
    try:
        lsResult = []
        success = False
        recvData = (str)(data.data)
        logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
        rospy.loginfo(logmsg)
        if is_json(recvData):
            # JSON 문자열을 파이썬 객체로 변환
            parsed_json = json.loads(recvData)
            if isinstance(parsed_json, dict):  # dict 객체 인경우 (단일제어)
                lsResult.append(parsed_json)
            elif isinstance(parsed_json, list):  # dict 의 배열 인경우 (복수제어)
                lsResult.extend(parsed_json)
        else:
            if recvData == "RESET" and not bReset:
                bReset = True
                return

            recvDataTmp = getDic_strArr(recvData.upper(), sDivFieldColon, sDivItemComma)
            lsResult.append(recvDataTmp)            
        print(lsResult)
        for recvDataMap in lsResult:
            sMBID = recvDataMap.get(MotorWMOVEParams.MBID.name, "")
            intMBID = int(sMBID)
            sCMDValue = recvDataMap.get(
                MotorWMOVEParams.CMD.name, None
            )  # 등록된 모드버스ID 인 경우!
            dicMBID = dic_485cmd.get(sMBID, None)  # 등록된 모드버스ID 인 경우!

            if dicMBID == None or sCMDValue == None:
                logmsg = f"{sMBID}:{dicMBID}:{sCMDValue} : 미등록 모드버스 또는 명령어가 틀립니다 {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
                rospy.loginfo(logmsg)
                continue
            # if sCMDValue == MotorCmdField.WINIT.name:
            #     bSetupRequested = dicSetupRequested.get(sMBID, None)
            #     if bSetupRequested:
            #         return
            #     else:
            #         dicSetupRequested[sMBID] = True

            sCMDParam = dicMBID.get(sCMDValue, None)
            """
            sCMD : 'WMOVE'
            sCMDParam : '0x6200:MODE/POS_H/POS_L/SPD/ACC/DECC/0/0x10'
            sCMDParamSplit[0] = Write 주소
            sCMDParamSplit[1] = 파라미터 셋 ( / 로 스플릿 한 후 _H 와 _L 로 끝나는 필드는 예외처리 )
            """
            writeAddress = 0
            if sCMDParam == None:  # 등록된 명령어인 경우
                if sCMDValue == MotorCmdField.WCALI.name:
                    dicCali[sMBID] = MOTOR_CALI_STATUS.A_REQUESTED
            else:
                sCMDParamTotalSplit = sCMDParam.split(sDivItemComma)
                for sCMDParamTmp in sCMDParamTotalSplit:  # 쉼표로 나눔 (명령어가 여러개)
                    sCMDParamSplit = sCMDParamTmp.split(sDivFieldColon)
                    writeAddress = try_parse_int(sCMDParamSplit[0], -1)
                    writeParamsArray = sCMDParamSplit[1].split(
                        sDivSlash
                    )  # 슬래쉬로 나눔 (값이 배열인 경우)
                    writeValueArray = []
                    for sParam in writeParamsArray:
                        bSplitedParam = False
                        iParamValue = try_parse_int(sParam, -1)
                        bNeedToReplace = True if iParamValue == -1 else False
                        checkWord = sParam
                        if bNeedToReplace:
                            if sParam.find("_") > 0:
                                checkWord = sParam.split("_")[0]
                                bSplitedParam = True
                            sParamValue = recvDataMap.get(checkWord, "-1")
                            iParamFinalValue = try_parse_int(sParamValue)
                            if bSplitedParam:
                                param_H, param_L = splitSignedInt(iParamFinalValue)
                                if sParam.find("_H") > 0:
                                    writeValueArray.append(param_H)
                                else:
                                    writeValueArray.append(param_L)
                            else:
                                writeValueArray.append(iParamFinalValue)
                        else:
                            writeValueArray.append(iParamValue)
                    print(writeValueArray)
                    WriteRegEx(writeAddress, writeValueArray, sMBID)

                if sCMDValue == MotorCmdField.WMOVE.name:
                    dtnow = getDateTime().timestamp()
                    lock.acquire()
                    try:
                        if dic_485ack.get(intMBID, None) == None:
                            dic_485ack[intMBID] = []
                        dic_485ack[intMBID].append(dtnow)
                    finally:
                        # Lock을 해제해서 다른 Thread도 사용할 수 있도록 만든다.
                        lock.release()
                    pub_flag.publish(f"{dtnow}{sDivFieldColon}0{sDivFieldColon}{sMBID}")
                    rospy.loginfo(f"Insert Ack : {sMBID} - {dic_485ack}")
                    # TODO : Polling Time Table 수정할 것.

    except Exception as e:
        message = traceback.format_exc()
        rospy.logdebug(message)
        # SendFeedback(e)
    return dtnow


rospy.Subscriber(TopicName.CMD_DEVICE.name, String, callbackCmd)
pub_flag = rospy.Publisher(f"{TopicName.ACK.name}", String, queue_size=ROS_TOPIC_QUEUE_SIZE)
#pub_flag.impl.transport.set_nodelay(True)
"""
2023-04-21
RS485 MODBUS 범용 제어 패키지 - 최병진

1. id - 프로필파일명으로 구성된 파일(ModbusConfig.txt) 로드하여 인스턴스 생성, modbustopc_{id} 형태의 퍼블리셔 생성,
1A. 파일에서 W로 시작하는 부분은 명령어 수행, R로 시작하는부분은 데이터 최대한 빨리 폴링. r로 시작하면 데이터 폴링주기 1초.
1B. 서보모터와 GPI 박스는 빠른 폴링, BMS와 NTC는 느린 폴링
2. 생성된 모드버스 객체는 dic_485ctl id - instance 형태로 저장
3. subcribe 로 명령어 수신. 명령어는 string 형태로 수신되며 map 으로 파싱, writereg 시에 반영.
4. loop 부분에서 R로 시작하는 비트만큼 읽어들인 후 (비트의 총합 / 16(워드)) modbustopc_{id} 토픽으로 발행.
"""

# getTxtPath 함수 만들자.
# dirPath2 = getConfigPath()
dirPath2 = getConfigPath(UbuntuEnv.ITX.name)
filePath_modbusconfig = f"{dirPath2}/ModbusConfig_{machineName}.txt"
filePath_CaliPotConfig = f"{dirPath2}/ModbusCaliPot_{machineName}.txt"
filePath_CaliNotConfig = f"{dirPath2}/ModbusCaliNot_{machineName}.txt"

strLiftFilePath = f"{dirPath2}/LIFTDOWN.txt"

print(filePath_modbusconfig, strLiftFilePath)
filePath_ModbusAlarm = f"{dirPath2}/SERVO_ALARM.txt"
# print(os.path.isfile(filePath_modbusconfig) )
filePath_ModbusSetupInit = f"{dirPath2}/SERVO_Init.txt"
filePath_param_parse = f"{dirPath2}/MODBUSDATA.txt"
dicParamParse = getDic_FromFile(filePath_param_parse, sDivEmart)
dic_ServoMonitorAlarm = getDic_FromFile(
    filePath_ModbusAlarm, sDivItemComma
)  # 리드샤인 isv 모터 알람코드

"""
Setup()
1. ModbusConfig.txt 라인단위로 리딩, id - 프로필 파일명을 map 으로 만들고 퍼블리셔 객체 생성 -> dic_topic
2. Subscribe Callback 등록
3. loop
"""
dicModbus = {}
# bSetupRequested = False
dicSetupRequested = {}
dicMB_Exception_count = {}


def LoadCaliData():
    global dicCaliPotEncoder
    global dicCaliNotLoad
    global dicCaliFinalPos

    try:
        with open(filePath_CaliPotConfig, "r") as f:
            dicCaliFinalPos = json.load(f)
            print(dicCaliFinalPos)
            dicCaliPotEncoder.clear()
            for mbid, cali_pos in dicCaliFinalPos.items():
                dicReturn = {}
                dicReturn[MOTOR_CALI_STATUS.D_CALI_COMPLETED] = cali_pos
                dicCaliPotEncoder[mbid] = dicReturn

        with open(filePath_CaliNotConfig, "r") as f:
            dicCaliNotLoad = json.load(f)
            print(dicCaliNotLoad)

    except Exception as e:
        message = traceback.format_exc()
        logmsg = f"{message} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
        rospy.loginfo(logmsg)

    return dicCaliPotEncoder


LoadCaliData()


def Setup(mbidCur=None):
    global dic_485ctl
    global dic_485cmd
    global dic_485poll
    global dic_485pollLen
    global dic_topics
    global dicConfigTmp
    global dicModbus
    global dic_485ack
    global lock
    global dicConfigInitReverse
    global dicInitTimeStamp
    global iTryCheckModbus
    # ModbusConfig.txt 를 읽어들인다.
    dicConfigTmp = getDic_FromFile(filePath_modbusconfig, sDivEmart)
    for mbid, mbName in dicConfigTmp.items():
        if mbidCur is not None and mbidCur != mbid:
            continue
        # 만일 주석이라면 건너뛴다.
        if try_parse_int(mbid) == 0:
            continue

        lastInitTime = dicInitTimeStamp.get(mbid, DATETIME_OLD)
        if not isTimeExceeded(lastInitTime, 5000):
            continue
        
        try:
            # 모드에 따라 디바이스 선별하여 초기화
            if fastModbus:
                if mbid in lsSlowDevices:
                    continue
            else:
                if mbid not in lsSlowDevices:
                    continue
            rospy.loginfo(f"Init Modbus ID for : {mbid}")
            checkList = dic_485ack.get(mbid, None)
            int485id = int(mbid)
            if checkList == None:
                lock.acquire()
                try:
                    if dic_485ack.get(int485id, None) == None:
                        dic_485ack[int485id] = []
                finally:
                    # Lock을 해제해서 다른 Thread도 사용할 수 있도록 만든다.
                    lock.release()
            else:
                dic_485ack[int485id].clear()
            # mbName = dicConfigTmp[dic485ID]
            sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            instrumentH: minimalmodbus.Instrument = None
            if fastModbus:
                instrumentH = minimalmodbus.Instrument(
                    port485, (int)(mbid), minimalmodbus.MODE_RTU
                )
                instrumentH.serial.close()
                instrumentH.serial.parity = serial.PARITY_NONE
                instrumentH.serial.stopbits = serial.STOPBITS_ONE
                instrumentH.serial.baudrate = 115200  # Baud
                instrumentH.serial.timeout = 0.1  # seconds
                instrumentH.clear_buffers_before_each_transaction = True
            else:
                # BMS 장치의 블루투스 주소
                bms_address = "20:20:06:01:01:A1"  # BMS 장치의 MAC 주소로 변경

                # 블루투스 소켓 생성
                sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

                # BMS에 연결 (포트는 일반적으로 1번을 사용, 필요에 따라 변경)
                port = 1
                sock.connect((bms_address, port))
                # instrumentH.serial.baudrate = 9600  # Baud
                # instrumentH.serial.timeout = 0.5  # seconds

            if dicSetupRequested.get(mbid, None) == None:
                dicSetupRequested[mbid] = False
                dicMB_Exception_count[mbid] = 0

            

            bInitReverse = False
            mbNameUpper = mbName.upper()
            if mbNameUpper != mbName:
                bInitReverse = True
                mbName = mbNameUpper
            dicConfigInitReverse[mbid] = bInitReverse
            filePath_modbus_cmd = f"{dirPath2}/{mbName}.txt"
            dicConfigcmd = getDic_FromFile(filePath_modbus_cmd, sDivEmart)
            poll485 = []
            for cmdKey in dicConfigcmd.keys():
                # checkCmd = try_parse_int(cmdKey, -1)
                checkCmd = str(cmdKey).find("W")
                if checkCmd != 0:
                    dicPollData = getDic_strArr(
                        dicConfigcmd[cmdKey], sDivFieldColon, sDivItemComma
                    )
                    iCurrent = cmdKey.split(sep=sDivFieldColon)

                    lenCurrent = 0
                    for fieldTmp in dicPollData.keys():

                        lenCurrent += (int)(dicPollData[fieldTmp])

                    if len(iCurrent) > 1:
                        addrStart = iCurrent[0]
                        pollRate = iCurrent[1]
                        dicPollData[POLL_COMMON.START.name] = addrStart
                        dicPollData[POLL_COMMON.RATE.name] = pollRate
                    else:
                        dicPollData[POLL_COMMON.START.name] = checkCmd
                        dicPollData[POLL_COMMON.RATE.name] = 0
                    dicPollData[POLL_COMMON.LEN.name] = lenCurrent

                    poll485.append(dicPollData)
                # print(dic_485poll)

            bMotorOK = False
        #for iTryCheckModbus in range(1):
            try:
                # 이 부분에 모터 초기화 통신 및 알람 푸쉬 루틴 구현

                # RS로 시작하면 서보모터여서 별도의 초기화 필요
                # ICS 모터인 경우 리딩 매핑작업이 되어있는지 검사 (0f10 값이 0 이면 공장출하값)
                # 공장출하상태시 WSETUP 커맨드 날림.
                # 예외처리 할 것.
                # checkAddr 이 -1 로 나오는 경우 구성정보 자체가 잘못되었으므로 알람이 가야됨.
                # 이 경우 Invalid Config 로 알람낼것.
                #checkAddrIsCCW = poll485[0].get(POLL_COMMON.START.name, "-1")
                checkAddrIsCCW = '7' if mbName.startswith("RS2") else 'D'
                checkAddrValCCW = int(checkAddrIsCCW, 16)
                checkAddrIsPOTChanged = '0x149' if mbName.startswith("RS2") else '0x409'
                checkAddrValPOTChanged = int(checkAddrIsPOTChanged, 16)
                pub_kpalive = rospy.Publisher(
                    f"{TopicName.MB_.name}{mbid}", String, queue_size=ROS_TOPIC_QUEUE_SIZE
                )
                dic_topics[mbid] = pub_kpalive
                dic_485ctl[mbid] = sock
                dic_485cmd[mbid] = dicConfigcmd
                dic_485poll[mbid] = poll485
                if mbName.startswith("RS"):
                    dicInitTimeStamp[mbid] = getDateTime()
                    checkMBresult = instrumentH.read_registers(checkAddrValCCW, 1, 3)
                    dic_485Inverted[mbid] = checkMBresult[0]
                    checkMBresult = instrumentH.read_registers(checkAddrValPOTChanged, 1, 3)
                    dic_485POTInfo[mbid] = checkMBresult[0]
                    #rospy.loginfo(checkMBresult)
                    # if mbName.startswith("RS6"):
                    #   initMotorSetup(mbid)
                    # checkresult = instrumentH.read_registers(0x0F10, 1, 3)
                    # if len(checkresult) > 0 and checkresult[0] == 0:
                    #     rospy.loginfo(f"WSETUP called for {mbid},{checkresult}")
                    #     initMotorSetup(mbid)
                    initMotor(mbid)
                    rospy.loginfo(
                        f"Check MB({mbid}) result OK at try {iTryCheckModbus+1}"
                    )
                    bMotorOK = True
                    #break
            except Exception as e:
                exc_type, exc_value, exc_traceback = sys.exc_info()
                line_number = exc_traceback.tb_lineno
                dicCurrentEncoder[mbid] = getDateTime()

                rospy.loginfo(
                    f"Check MB({mbid}) result Error at {line_number}, try {iTryCheckModbus+1} : {e}"
                )
                # time.sleep(0.5)

            if not bMotorOK:
                dic_topics[mbid].publish(
                    f"{MonitoringField_EX.ALM_CD.name}{sDivFieldColon}{AlarmCodeList.NOT_CONNECTED.value}{sDivItemComma}{MotorWMOVEParams.MBID.name}{sDivFieldColon}{mbid}"
                )
                dic_485ack.pop(int485id)
                rospy.loginfo(dic_485ack)

        except Exception as e:
            message = traceback.format_exc()
            logmsg = f"{message} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
            rospy.loginfo(logmsg)
        # resultTest = getSERVOData(instrumentV)
        # print(dic_485ctl,dic_485cmd)
        # if len(dic_485ack) == 0:
        #   # time.sleep(5) #모터에 접속 불가시 ALM_CD:-1 메세지를 발행하고 3초 쉰다.
        #   # sendbuf = f'{MonitoringField_EX.ALM_CD.name}{sDivFieldColon}-1'
        #   # dic_topics[modbusID].publish(sendbuf)
        #   # pub_kpalive.publish()
        #   return False
        # print(dicConfigcmd)
        # return True
        # # print(dic_485poll)
    print(dic_485Inverted)
    print(dic_485POTInfo)
    
    return True


def motorACKClear(mbid=None):
    global dic_485ack
    if mbid == None:
        for modbusID in dic_485poll.keys():  # 장치ID 별로 엑세스
            if modbusID in lsSlowDevices:
                continue
            dic_485ack[int(modbusID)].clear()
    else:
        dic_485ack[int(mbid)].clear()


def initMotor(mbid):
    global dicCali
    callData = String()
    bInitReverse = dicConfigInitReverse[mbid]
    dicCali[mbid] = MOTOR_CALI_STATUS.Z_NOT_COMPLETED
    initStr = (
        MotorCmdField.WINITREVERSE.name if bInitReverse else MotorCmdField.WINIT.name
    )
    callData.data = (
        f"{MotorWMOVEParams.MBID.name}:{mbid},{MotorWMOVEParams.CMD.name}:{initStr}"
    )
    callbackCmd(callData)
    logmsg = f"{mbid} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
    rospy.loginfo(logmsg)
    


def initMotorSetup(mbid):
    # callData = String()
    # callData.data = f"{MotorWMOVEParams.MBID.name}:{mbid},{MotorWMOVEParams.CMD.name}:{MotorCmdField.WSETUP.name}"
    # callbackCmd(callData)
    initMotorCustom(mbid, MotorCmdField.WSETUP)


def stopMotor(mbid):
    initMotorCustom(mbid, MotorCmdField.WSTOP)

def stopAllMotors():
  for modbusIDTmp in dic_485poll.keys():  # 장치ID 별로 엑세스
    if modbusIDTmp in lsSlowDevices:
        continue
    stopMotor(modbusIDTmp)
  

def initMotorCustom(mbid, cmdParam: MotorCmdField):
    callData = String()
    callData.data = f"{MotorWMOVEParams.MBID.name}:{mbid},{MotorWMOVEParams.CMD.name}:{cmdParam.name}"
    callbackCmd(callData)


callData = String()
# callData.data = 'MBID:31,CMD:WSTOP'
"""
rostopic pub -1 /CMD_DEVICE std_msgs/String -- 'MBID:31,CMD:WMOVE,MODE:1,POS:0,SPD:1500,ACC:10,DECC:10'
rostopic pub -1 /CMD_DEVICE std_msgs/String -- 'MBID:31,CMD:WSPD,MODE:1,POS:9,SPD:3000,ACC:1000,DECC:1000'
rostopic pub -1 /CMD_DEVICE std_msgs/String -- 'MBID:31,CMD:WSTOP'
rostopic pub -1 /CMD_DEVICE std_msgs/String -- 'MBID:31,CMD:WZERO'
rostopic pub -1 /CMD_DEVICE std_msgs/String -- 'MBID:31,CMD:WOFF'
"""
# callData.data = 'MBID:31,CMD:WINIT'
# callbackCmd(callData)

# #callData.data = 'MBID:31,CMD:WOFF'
# callData.data = 'MBID:31,CMD:WMOVE,MODE:0x41,POS:100000000,SPD:1500,ACC:10,DECC:10'
# callbackCmd(callData)

Setup()


# class MotorCtl:
#     def __init__(self):
#         self.set_saveImage_service = rospy.Service(
#             "/carrier/motormove", modbus_motor, self.MotorMove
#         )
#         self.CMD_service = rospy.Service(
#             ServiceBLB.CMD_DEVICE.value, SetBool, self.SendCMD
#         )
#         self.Status_service = rospy.Service('/get_modbus', Trigger, self.handle_custom_service)

#     def handle_custom_service(self,req):
#         listReturn = []
#         lock.acquire()
#         try:
#             for modbusID in dic_485poll.keys():  # 장치ID 별로 엑세스
#                 cmdData = dic_485poll[modbusID]
#                 # 한 디바이스에서 2개 이상 다른 주소로 Polling시 cmdData 원소가 여러개가 된다.
#                 for dicRecord in cmdData:
#                     start_addr = 1
#                     if POLL_COMMON.START.name not in dicRecord.keys():
#                         start_addr = 0
#                     if try_parse_int(dicRecord[POLL_COMMON.START.name], MIN_INT) == MIN_INT:
#                         strStartAddr = dicRecord[POLL_COMMON.START.name][1:]
#                         intStart_addr = try_parse_int(strStartAddr)
#                         len_items = (dicRecord[POLL_COMMON.LEN.name]) / 16
#                         rcv = pollRead(modbusID,intStart_addr,int(len_items))
#                         rcv[MotorWMOVEParams.MBID.name] = modbusID
#                         dicTmpU = copy.deepcopy(rcv)
#                         listReturn.append(rcv)
                        
#             # 반환할 값 설정
#             response = json.dumps(listReturn)
#             # response = {
#             #     'time': {
#             #         'secs': 1724738649,
#             #         'nsecs': 779511451
#             #     }
#             # }
#             #return TriggerResponse(success=True, message=str(response))
#         finally:
#             lock.release()
#         return TriggerResponse(success=True, message=response)

#     def SendCMD(self, req):
#         msg = req.name
#         callData.data = msg
#         respID = callbackCmd(callData)
#         return SetBoolResponse(True, f'{callData}')

#     def MotorMove(self, req):
#         global callData
#         # global motorOpCheck
#         global dic_485ack
#         req_Cmd = req.cmd
#         req_mbID = req.mb_id
#         req_mode = req.mode
#         req_pos = req.pos
#         req_spd = req.spd
#         req_acc = req.acc
#         req_decc = req.decc
#         # sCmd = (f'{MotorWMOVEParams.MBID.name}{sDivFieldColon}{req_mbID}{sDivItemComma}'
#         #         f'{MotorWMOVEParams.CMD.name}{sDivFieldColon}{req_Cmd}{sDivItemComma}'
#         #         f'{MotorWMOVEParams.MODE.name}{sDivFieldColon}{req_mode}{sDivItemComma}'
#         #         f'{MotorWMOVEParams.POS.name}{sDivFieldColon}{req_pos}{sDivItemComma}'
#         #         f'{MotorWMOVEParams.SPD.name}{sDivFieldColon}{req_spd}{sDivItemComma}'
#         #         f'{MotorWMOVEParams.ACC.name}{sDivFieldColon}{req_acc}{sDivItemComma}'
#         #         f'{MotorWMOVEParams.DECC.name}{sDivFieldColon}{req_decc}'
#         # )
#         isabsolute = True
#         if req_mode == str(ServoParam.MOVE_TYPE_REL.value):
#             isabsolute = False
#         sCmd = getMotorMoveString(
#             req_mbID, isabsolute, req_pos, req_spd, req_acc, req_decc
#         )
#         callData.data = sCmd
#         respID = callbackCmd(callData)
#         # print(f'{sCmd}')
#         # respID = int(getCurrentTime().replace(sDivFieldColon, '') )
#         # respID=getDateTime().timestamp()

#         # respID=getDateTime()
#         resp = modbus_motorResponse(str(respID))
#         # dic_485ack[req_mbID].append(respID)
#         # motorOpCheck[respID] = getDateTime()
#         # pub_flag.publish(f'{respID}{sDivFieldColon}0{sDivFieldColon}{req_mbID}')
#         return resp


def print_time(chk):
    return
    print(f"{chk} - {getDateTime()}")


cntLoop = 0
# exception_read_count = 0
lastUpdateTimeStamp = getDateTime()
lastAlarmTimeStamp = getDateTime()
#MotorCtl()

# bChangeBaudRate = False
# iChangeBaudRate = 384
# if bChangeBaudRate:
#     instrumentTmp : minimalmodbus.Instrument= dic_485ctl[1]
#     instrumentTmp.write_register()
#     WriteReg(0x2101, iChangeBaudRate,1)
#     WriteReg(0xFE, 5,2)
return485data = {}
calispeed = 100
caliAcc = 1000
caliDecc = 1000
caliMaxRange = 10000000


def pollRead(modbusID,start_addr,len_items):
  global return485data
  #global dic_485ctl
  instrumentTmp = dic_485ctl[modbusID]
  modbus_request = create_modbus_request(1, 3,start_addr, (int)(len_items))
  instrumentTmp.send(modbus_request)
  # print(f"Try to read :{modbusID}")
  response = instrumentTmp.recv(10240)
  slave_addr, function_code, resultList = parse_modbus_response(response)  
  if len(resultList) == 0:
      print(f"Empty Read data MBID:{modbusID}-1")
  # time.sleep(read_delay * 100)
  for intTmp in resultList:
      bytesTmp = intTmp.to_bytes(2, byteorder="big")
      resultMulti.append(bytesTmp[0])
      resultMulti.append(bytesTmp[1])
  print_time(f"{modbusID}-2")
  strBMSDataPartByte = bytearray(resultMulti)
  strBMSDataPartBit = ConstBitStream(strBMSDataPartByte)
  readBytes = 0
  print_time(3.3)
  idxTmp = 0
  for itemName, itemLenStr in dicRecord.items():
      # idxTmp += 1
      # print(f"{modbusID}-{idxTmp}-{itemName}-{itemLenStr}")
      if itemName in poll_common:
          continue
      itemLen = (int)(itemLenStr)
      readBytes += itemLen
      signHeader = "uint"
      if itemLen >= 8:
          signHeader = "int"

      if strBMSDataPartBit.len >= readBytes:
          itemHex = strBMSDataPartBit.read(f"{signHeader}:{itemLen}")
          # keyName = f'{itemName}_{modbusID}'
          keyName = itemName
          if itemName in dicParamParse.keys():
              fomula = dicParamParse[itemName]
              fomulaFinal = f"{itemHex}{fomula}"
              FinalValue = eval(fomulaFinal)
              FinalValueStr = f"{FinalValue :.2f}"
              return485data[keyName] = FinalValueStr
          else:
              return485data[keyName] = itemHex
      else:

          rospy.loginfo(
              f"Empty Read data MBID:{strBMSDataPartBit.len}, {readBytes}"
          )  
#   if start_addr == 1029:
#     print(return485data)

  return return485data

stopAllMotors()
while not rospy.is_shutdown():
    bReturn = True
    if bReset:
        break
    td = getDateTime() - lastUpdateTimeStamp
    if td.total_seconds() >= 1:
        lastUpdateTimeStamp = getDateTime()
        # rospy.loginfo(f'Loop : {cntLoop}, {dicModbus}')
        cntLoop = 0
    else:
        cntLoop += 1

    try:
        print_time(0)
        while len(CMD_Queue) > 0:
            cmd_buf = list(CMD_Queue.popleft())
            addr = cmd_buf[0]
            valueList = cmd_buf[1]
            drvID = cmd_buf[2]
            caller = cmd_buf[3]
            infoStr = f"Addr:{addr},Value:{valueList},MB_ID:{drvID} from {caller}"
            instrumentTmp: minimalmodbus.Instrument = dic_485ctl[drvID]
            rospy.loginfo(f"Trying to write : {infoStr}")
            if len(valueList) == 1:
                instrumentTmp.write_register(addr, valueList[0])
            else:
                instrumentTmp.write_registers(addr, valueList)
            time.sleep(MODBUS_WRITE_DELAY)
            # if len(valueList) == 1:
            #     instrumentTmp.write_register(addr,valueList[0])
            #     isFirstAccess = False
            #     time.sleep(write_delay)
            # else:
            #     if not isFirstAccess:
            #         time.sleep(0.1)
            #     instrumentTmp.write_registers(addr,valueList)
            #     time.sleep(write_delay)
            bSetupRequested = dicSetupRequested.get(drvID, None)
            if bSetupRequested:
                dicSetupRequested[drvID] = False
            print_time(1)

    except Exception as e:
        # if cmd_buf != None:
        #     CMD_Queue.appendleft(cmd_buf)
        rospy.loginfo(f"write_registers error : {cmd_buf} - {e}")
        pass
        # bReturn = False
        # rospy.loginfo(traceback.format_exc())
        # dtNow = getDateTime()
        # td = getDateTime() - lastLogTime
    # TODO : Polling Read Part
    try:
        print_time(2)
        for modbusID in dic_485poll.keys():  # 장치ID 별로 엑세스
            return485data.clear()
            cmdData = dic_485poll[modbusID]
            # print(type(cmdData))
            print_time(f"{modbusID}-0")

            # 한 디바이스에서 2개 이상 다른 주소로 Polling시 cmdData 원소가 여러개가 된다.
            for dicRecord in cmdData:
                start_addr = 1
                if POLL_COMMON.START.name not in dicRecord.keys():
                    start_addr = 0
                if try_parse_int(dicRecord[POLL_COMMON.START.name], MIN_INT) == MIN_INT:
                    continue
                start_addr = (int)(dicRecord[POLL_COMMON.START.name], 16)
                len_items = (int)(dicRecord[POLL_COMMON.LEN.name]) / 16
                pollrate = (int)(dicRecord[POLL_COMMON.RATE.name])
                # if pollrate == -1:
                #     curExtraDic = dicSI_TotalStatus.get(modbusID, {})
                #     curExtraData = curExtraDic.get(start_addr,{})
                #     if len(curExtraData) == 0:
                #         pollrate = 1000
                #     else:
                #         continue
                    
                poll_id = f"{modbusID}_{start_addr}"
                int485id = int(modbusID)

                # 현재 모터가 동작중 상태플래그
                bOnMoving = False
                lsAckList = dic_485ack.get(int485id, None)
                if lsAckList != None and len(lsAckList) > 0:
                    bOnMoving = True
                    #if pollrate < 5000:
                    pollrate = 10
                    strCurPos = ""
                    # rospy.loginfo(
                    #     f"Ack Check : {int485id} - {dic_485ack}, {dicCurrentEncoder}"
                    # )

                if pollrate > 500:
                    lastPublishedTime = dic_485pollRate.get(
                        poll_id, DATETIME_OLD
                    )
                    # 모터가 동작중일때는 저속 폴링 디바이스는 건너 뛴다
                    if fastModbus:
                        if modbusID in lsSlowDevices:
                            continue
                    else:
                        if modbusID not in lsSlowDevices:
                            continue

                    # if bOnMoving and modbusID in lsSlowDevices:
                    # continue

                    # 지정된 폴링주기에 이르지 못한 경우도 건너뛴다
                    if not isTimeExceeded(lastPublishedTime, pollrate):
                        continue
                    # print(dic_485ack)
                    # Lock을 해제해서 다른 Thread도 사용할 수 있도록 만든다.

                resultMulti = []
                resultList = []
                try:
                    # MonitoringField_EX.ST_ENABLE 속성을 확인 후
                    # if modbusID == "15":
                    #     #     # print(
                    #     #     #     f"현재데이터:{return485data},요소수:{len(return485data)},읽기지시:{dicRecord}"
                    #     #     # )
                    #     print(f"{len(return485data)} at {id(return485data)}")
                    # print(return485data, len(return485data), modbusID)
                    pollRead(modbusID,start_addr,len_items)
                    bInitReverse = dicConfigInitReverse[modbusID]
                    # POT NOT가 뒤바뀐 보드는 두개 값을 바꿔줘야함
                    if bInitReverse:
                        if (
                            MonitoringField.DI_POT.name in return485data.keys()
                            and MonitoringField.DI_NOT.name in return485data.keys()
                        ):
                            # print(type(return485data))
                            pot_tmp = return485data[MonitoringField.DI_POT.name]
                            not_tmp = return485data[MonitoringField.DI_NOT.name]
                            if not_tmp != pot_tmp:
                                return485data[MonitoringField.DI_POT.name] = not_tmp
                                return485data[MonitoringField.DI_NOT.name] = pot_tmp
                            # print(type(return485data))
                    dicModbus.update(return485data)

                    currentEnableStatus = dicModbus.get(
                        MonitoringField_EX.ST_ENABLE.name, -1
                    )
                    currentAlarmStatus = dicModbus.get(
                        MonitoringField_EX.ALM_CD.name, -1
                    )
                    dic_485pollRate[poll_id] = getDateTime()

                    # BMS, NTP 인 경우는 그냥 넘어감
                    if modbusID in lsSlowDevices:
                        dicMB_Exception_count[modbusID] = 0
                    # currentEnableStatus 이 -1 인 경우 - 연결자체가 안 된경우 (이럴땐 setup() 을 다시 해줘야 함)
                    elif currentEnableStatus == -1:
                        raise Exception("Modbus Timeout")
                    # currentEnableStatus 가 0 인 경우 - 통신은 되는데 초기화가 안 된경우 (초기화만 해주면 됨)
                    elif currentEnableStatus == 0 and currentAlarmStatus == 0:
                        initMotor(modbusID)
                        dicMB_Exception_count[modbusID] = 0
                    # currentEnableStatus 가 1 이면 정상
                    elif currentEnableStatus == 1:
                        dicMB_Exception_count[modbusID] = 0
                    # 그 외는 환경설정 에러 - 알람발생
                    else:
                        
                        if isTimeExceeded(lastAlarmTimeStamp, 1000):
                          max_length = max(len(value) for value in dic_485ack.values())
                          if max_length > 0:
                            print(f"{modbusID} - 모터 알람 혹은 초기화 명령어가 동작하지 않는 상태 - {return485data}")
                            #한개의 모터라도 알람 발생할 경우 모터 일괄정지.
                            stopAllMotors()
                            lastAlarmTimeStamp = getDateTime()
                    print_time(2.4)

                    isPot = isTrue(dicModbus.get(MonitoringField.DI_POT.name, None))
                    isNot = isTrue(dicModbus.get(MonitoringField.DI_NOT.name, None))
                    isHome = isTrue(dicModbus.get(MonitoringField.DI_HOME.name, None))
                    isMotorStopped = isTrue(
                        dicModbus.get(MonitoringField.ST_CMD_FINISH.name, None)
                    ) or not isTrue(
                        dicModbus.get(MonitoringField.ST_RUNNING.name, None)
                    )
                    cur_pos = dicModbus.get(MonitoringField.CUR_POS.name, None)
                    dicCurrentEncoder[modbusID] = cur_pos
                    # dictAppend(dicCaliEncoder,modbusID, MOTOR_CALI_STATUS.A_REQUESTED, cur_pos)

                    caliStatus = dicCali.get(
                        modbusID, MOTOR_CALI_STATUS.Z_NOT_COMPLETED
                    )
                    # 캘리브레이션 파트 -
                    if caliStatus == MOTOR_CALI_STATUS.A_REQUESTED and isMotorStopped:
                        # 모터가 멈춰 있다면! 정방향으로 운전명령어 내린 후 dicCali 상태를 업데이트 한다
                        # 일단 파라미터는 임의 하드코딩이고 나중에 MBID 마다 적절한 파라미터를 정의하자.

                        # POT나 NOT가 감지된 상태에서는 CALI 하지 않는다.
                        if isPot or isNot:
                            print(
                                f"POT:{isPot}/NOT:{isNot} 감지되었습니다. 수동운전으로 POT/NOT 해제 후 다시 캘리브레이션 명령어를 넣어주세요."
                            )
                            dicCali[modbusID] = MOTOR_CALI_STATUS.X_CANCELLED
                        else:
                            sCmd = getMotorMoveString(
                                modbusID,
                                False,
                                caliMaxRange,
                                calispeed,
                                caliAcc,
                                caliDecc,
                            )
                            # 정방향 명령어 송출 완료!
                            callData.data = sCmd
                            respID = callbackCmd(callData)
                            dictAppend(
                                dicCaliPotEncoder,
                                modbusID,
                                MOTOR_CALI_STATUS.A_REQUESTED,
                                cur_pos,
                            )
                            print(dicCaliPotEncoder)
                            dicCali[modbusID] = MOTOR_CALI_STATUS.B_JOGCW_REQUESTED

                    elif caliStatus == MOTOR_CALI_STATUS.B_JOGCW_REQUESTED:
                        # if isHome and not isMotorStopped:
                        if isNot:  # 정방향으로 운전중인데 not 가 감지되었다면
                            prePos = dicCaliPotEncoder.get(modbusID)[
                                MOTOR_CALI_STATUS.A_REQUESTED
                            ]
                            differenceEnc = abs(
                                int(cur_pos) - int(prePos)
                            )  # 이전 단계와의 엔코더 격차를 구한다
                            # 정방향으로 1000 이상 움직였는데 마주친 것이 NOT 라면 POT와 NOT 를 바꾸어야 한다.
                            if differenceEnc > 1000:
                                dicConfigInitReverse[modbusID] = (
                                    not dicConfigInitReverse[modbusID]
                                )
                                if dicConfigInitReverse[modbusID]:
                                    dicConfigTmp[modbusID] = dicConfigTmp[
                                        modbusID
                                    ].lower()
                                else:
                                    dicConfigTmp[modbusID] = dicConfigTmp[
                                        modbusID
                                    ].upper()
                                saveDic_ToFile(
                                    dicConfigTmp, filePath_modbusconfig, None, True
                                )
                                initMotor(modbusID)
                                # 전단계로 보낸다.
                                dicCali[modbusID] = MOTOR_CALI_STATUS.A_REQUESTED
                        elif isPot:
                            stopMotor(
                                modbusID
                            )  # 정방향인데 POT를 만났으니 아마 멈출거다. 호출안해도 될지도
                            initMotorCustom(
                                modbusID, MotorCmdField.WZERO
                            )  # 엔코더값 0으로 초기화 한다.
                            dictAppend(
                                dicCaliPotEncoder,
                                modbusID,
                                MOTOR_CALI_STATUS.B_JOGCW_REQUESTED,
                                cur_pos,
                            )
                            # sCmd = getMotorMoveString(
                            #     modbusID, False, -100000000, 100, 2000, 2000
                            # )
                            # #역방향 명령어 송출 완료!
                            # callData.data = sCmd
                            # respID = callbackCmd(callData)
                            dicCali[modbusID] = MOTOR_CALI_STATUS.C_JOGCCW_REQUESTED
                        else:
                            print(f"B.아마도 계속 운전중일거야 - 현재위치 : {cur_pos}")
                    elif caliStatus == MOTOR_CALI_STATUS.C_JOGCCW_REQUESTED:
                        # 역방향으로 캘리시작
                        # if isMotorStopped:  #모터가 멈춘 상태 (다 돌아갔음)
                        if isNot:
                            dicCali[modbusID] = MOTOR_CALI_STATUS.D_CALI_COMPLETED
                            dictAppend(
                                dicCaliPotEncoder,
                                modbusID,
                                MOTOR_CALI_STATUS.D_CALI_COMPLETED,
                                abs(cur_pos),
                            )
                            dicCaliFinalPos[modbusID] = abs(cur_pos)
                            saveDic_ToFile(
                                dicCaliFinalPos, filePath_CaliPotConfig, sDivEmart, True
                            )
                            initMotorCustom(
                                modbusID, MotorCmdField.WZERO
                            )  # 엔코더값 0으로 초기화 한다.

                            print(f"Cali Finished MBID:{dicCaliPotEncoder}")
                        elif isPot:
                            if len(CMD_Queue) == 0 and isMotorStopped:
                                sCmd = getMotorMoveString(
                                    modbusID,
                                    False,
                                    -caliMaxRange,
                                    calispeed,
                                    caliAcc,
                                    caliDecc,
                                )
                                # 역방향 명령어 송출 완료!
                                callData.data = sCmd
                                respID = callbackCmd(callData)
                                print(f"역방향 명령어 송출 완료! : {respID}")
                            else:
                                print("현재 POT , 곧 반대쪽으로 출발 예정")
                        else:
                            print(f"C - POT:{isPot}, NOT:{isNot}, HOME:{isHome}")

                    # else:
                    #   print(f'C.아마도 계속 운전중일거야 - 현재위치 : {cur_pos}')

                except Exception as e:
                    # exception_read_count += 3
                    dicMB_Exception_count[modbusID] += 3
                    # rospy.loginfo(
                    #     f"{dicConfigTmp[modbusID]}:{poll_id}-{e} : Count:{dicMB_Exception_count[modbusID]}"
                    # )
                    if dicMB_Exception_count[modbusID] >= 10:
                        if len(dic_485ack) == 0:
                            # if exception_read_count >= 10:
                            # raise ModuleNotFoundError
                            dicMB_Exception_count[modbusID] = 0
                            # initMotor(modbusID)
                            if Setup(modbusID):
                                rospy.loginfo(f"Try call initMotor with MBID : {modbusID}")
                        else:
                            Setup(modbusID)
                            strAlm = f"{MonitoringField_EX.ALM_CD.name}{sDivFieldColon}{AlarmCodeList.NOT_CONNECTED.value}{sDivItemComma}{MotorWMOVEParams.MBID.name}{sDivFieldColon}{modbusID}"
                            dicAlm = getDic_strArr(strAlm,sDivFieldColon,sDivItemComma)
                            return485data.update(dicAlm)
                    # else:
                    #     time.sleep(MODBUS_EXCEPTION_DELAY)
                    #     pass
            # 복수개의 read 지시를 마친 후 타는 루틴
            # print(return485data)
            if len(return485data) > 0:
                # return485data[f'LASTSEEN_{modbusID}'] = getDateTime().timestamp()
                volt = float(return485data.get(MonitoringField_BMS.Voltage.name, 0))
                curr = float(return485data.get(MonitoringField_BMS.CurCadc.name, 0))
                return485data[MonitoringField_BMS.WATT.name] = round(volt*curr)
                return485data[MonitoringField.LASTSEEN.name] = (
                    getDateTime().timestamp()
                )
                return485data[MotorWMOVEParams.MBID.name] = modbusID
                
                # dicCaliPotInfo = dicCaliPotEncoder.get(modbusID, {})
                # not_pos_mbid = dicCaliNotLoad.get(modbusID,0)
                # caliPosCurrent = dicCaliPotInfo.get(MOTOR_CALI_STATUS.D_CALI_COMPLETED, -1)
                # if caliPosCurrent > 0:
                #     return485data[MonitoringField.NOT_POS.name] = not_pos_mbid
                #     return485data[MonitoringField.POT_POS.name] = caliPosCurrent
                # else:
                #     return485data[MonitoringField.POT_POS.name] = return485data[MonitoringField.NOT_POS.name] = caliPosCurrent                    
                    
                # return485data[MotorWMOVEParams.MBID.name] = modbusID
                # isCCW = dic_485Inverted.get(modbusID, MIN_INT)
                # return485data[MonitoringField.IS_CCW.name] = isCCW
                # if isTrue(isCCW):
                #     rpm = int(return485data.get(MonitoringField.CUR_SPD.name,MIN_INT))
                #     return485data[MonitoringField.CUR_SPD.name] = -rpm
                    
                # POTInfo = dic_485POTInfo.get(modbusID,MIN_INT)
                # return485data[MonitoringField.POT_ALLOCATION.name] = dicSICode.get(POTInfo,MIN_INT)
                # alarm_name = "NONE"
                # alarm_code = return485data.get(f"{MonitoringField.ALM_CD.name}", '-1')
                # if alarm_code != '-1':
                #     hexAlmCode = hex(int(alarm_code))[2:].rjust(3,'0')
                #     alarm_name = dic_ServoMonitorAlarm.get(hexAlmCode,"UNKNOWN")
                #     return485data[MonitoringField.ALM_NM.name] = alarm_name
                
                # dicSIStatus = {}
                # for key, value in return485data.items():
                #     if key.startswith('SI'):
                #         iVal = int(value)
                #         statusSTR = dicSICode.get(iVal,value)
                #         dicSIStatus[key] = statusSTR
                # if len(dicSIStatus) > 0:
                #     return485data.update(dicSIStatus)
                    
                #     dicSI_TotalStatus[modbusID] = 
                sendbuf = getStr_fromDic(return485data, sDivFieldColon, sDivItemComma)
                dic_topics[modbusID].publish(sendbuf)
                int485id = int(modbusID)
                # motorOpCheck = dic_485ack[int485id]

                # for cmdID in list(motorOpCheck.keys()):
                #     timeStamp = motorOpCheck[cmdID]
                #     if isTimeExceeded(timeStamp, 100):
                motorModel = dicConfigTmp.get(modbusID, "")
                flagPOT = return485data.get(f"{MonitoringField.DI_POT.name}", None)
                flagNOT = return485data.get(f"{MonitoringField.DI_NOT.name}", None)
                flagSPD = return485data.get(f"{MonitoringField.CUR_SPD.name}", None)
                # if modbusID == '28':
                #     print(modbusID)

                isMotorFinished = None
                if motorModel.upper().startswith("RS2"):
                  isMotorFinished = flipBoolan(return485data.get(f"{MonitoringField.ST_RUNNING.name}", None))
                  # if modbusID == '30':
                  #   print(isMotorFinished)
                #   if not isMotorFinished:
                #       if isNot and abs(cur_pos - not_pos_mbid) < PULSES_PER_ROUND/2:
                #           isMotorFinished = True
                #       elif isPot and abs(cur_pos - caliPosCurrent) < PULSES_PER_ROUND/2:
                #           isMotorFinished = True
                else:
                  isMotorFinished = isTrue(return485data.get(f"{MonitoringField.ST_CMD_FINISH.name}", None))

                # if alarm_code == '-1':
                #   isMotorFinished = True                  

                motorFlag = 1 if isMotorFinished else 0
                lsAckList = dic_485ack.get(int485id, None)
                if lsAckList != None and len(lsAckList) > 0:
                    ts = dic_485ack[int485id][0]
                    datetimeobj = datetime.fromtimestamp(ts)

                    # flagSPDIncrease = True
                    # if flagSPD is not None:
                    #     flagSPDIncrease = True if flagSPD > 0 else False

                    # if (flagNOT == 1 and not flagSPDIncrease) or (
                    #     flagPOT == 1 and flagSPDIncrease
                    # ):
                    #     motorFlag = 1
                    #     datetimeobj = datetime.fromtimestamp(0)
                    #     rospy.loginfo(
                    #         f"NOT/POT({flagNOT}:{flagPOT}) Detected at {int485id}, Spd:{flagSPD}"
                    #     )

                    if isTimeExceeded(datetimeobj, MODBUS_EXECUTE_DELAY_ms):
                        if motorFlag == 1:
                            # if modbusID == '30':
                            #   print(isMotorFinished)

                            cmdID = dic_485ack[int485id].pop()
                            # 주어진 타임스탬프
                            given_timestamp = cmdID
                            # 현재 시간 타임스탬프
                            current_timestamp = time.time()
                            # 타임스탬프 차이 계산
                            time_difference = current_timestamp - given_timestamp
                            # 밀리초 단위로 변환
                            time_difference_ms = time_difference * 1000
                            rospy.loginfo(
                                f"Popping Ack at {int485id},time:{time_difference_ms:.1f},NOT/POT({flagNOT}:{flagPOT}),Spd:{flagSPD},MotorFlag:{motorFlag}"
                            )
                            pub_flag.publish(
                                f"{cmdID}{sDivFieldColon}{motorFlag}{sDivFieldColon}{modbusID}"
                            )
            # else:
            #     print("Read Error : " + modbusID)
            #     dicMB_Exception_count[modbusID] += 3

            print_time(4)
            """
            rostopic echo /MB_31
            """
        print_time(5)
        #print(dic_485ack)
    except Exception as e:
        bReturn = False
        rospy.loginfo(traceback.format_exc())
        rospy.signal_shutdown(e)

        # sCmd = '/root/.rrStart -&'
        # os.system(sCmd)
    rate.sleep()
