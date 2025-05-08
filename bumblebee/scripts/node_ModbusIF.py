#!/usr/bin/env python3

from colorama import Fore, Style
import statistics
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
import struct
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


def LoadCurrentPos():
    dicPos = {}
    try:
        # 파일이 존재하는지 확인
        if not os.path.exists(strFileLastPos):
            rospy.loginfo(f"File not found : {strFileLastPos}.")
            return
        # 파일 읽기
        with open(strFileLastPos, "r") as f:
            dicPos = json.load(f)

        # JSON 데이터 처리
        os.remove(strFileLastPos)
        for mbid, posTmp in dicPos.items():            
            dicLoc=getMotorLocationSetString(mbid, posTmp)
            callData = String()
            callData.data = dicLoc
            callbackCmd(callData)
            #time.sleep(MODBUS_WRITE_DELAY)
    except Exception as e:
        sMsg = traceback.format_exc()
        rospy.loginfo(sMsg)

def PrintColorDict(data):
  output = ", ".join(
      f"{key}: {Fore.GREEN}{value}{Style.RESET_ALL}" if value else f"{key}: {Fore.RED}{value}{Style.RESET_ALL}"
      for key, value in data.items()
  )
  print(f'{output} {getCurrentTime()}')        

dicSICode = {}
dicSICode[0] = 'GPI'
dicSICode[136] = 'ICS'
dicSICode[33] = 'HOME'
dicSICode[34] = 'ESTOP'
dicSICode[37] = 'POT'
dicSICode[38] = 'NOT'
dicSI_TotalStatus = {}
pollrate_min = 300
CMD_Queue = deque()  # 모드버스 통신 동기화를 위한 Write 버퍼
# publish_topic_goal  : str = 'CMD' #테스트용 변수
# publish_topic_name = 'MB'
# publish_topic_ACK = 'ACK'
nodeName = f"node_{TopicName.CMD_DEVICE.name}"
rospy.init_node(nodeName, anonymous=False)  # 485 이름의 노드 생성
fastModbus = isTrue(rospy.get_param("~fastModbus", default=True))
testModbus = isTrue(rospy.get_param("~testMode", default=False))
runFromLaunch = isTrue(rospy.get_param(f"~{ROS_PARAMS.startReal.name}", default=False))
lsSlowDevices = ["1"]
lsNotMotor = ["80"]
machineName = GetMachineStr()
rospy.loginfo(f"testModbus:{testModbus},Modbus Fast:{fastModbus},Machine Name:{machineName}")
# time.sleep(10)

param_DRV_Rate = 100
lastAlarmMsg = ""
port485 = ""
isRealMachine = get_hostname().find(UbuntuEnv.ITX.name) >= 0
if fastModbus:
    if isRealMachine:#machineName == UbuntuEnv.ITX.name:
        port485 = "/dev/ttyS0"  # 시리얼 포트
        port485 = "/dev/ttC485M"  # USB 포트
        #port485 = "/dev/ttyUSB0"  # 시리얼 포트
    else:
        port485 = "/dev/ttC485M"  # USB 포트

# dirPath = os.path.dirname(__file__)
dic_485Torque = {}
dic_485OverLoad = {}
dic_485ctl = {}
dic_485cmd = {}
dic_485poll = {}
dic_485Inverted = {}
#dic_485AutoGain = {}
#dic_485Inertia_Ratio = {}
#dic_485TorQueLimitInfo = {}
dic_485pollLen = {}
dic_485pollRate = {}
dic_topics = {}
dic_status = {}
dicConfigTmp = {}
dicConfigInitReverse = {}
dicCali = {}  # 캘리브레이션 변수 저장.
dicCaliFinalPos = {}  # 캘리브레이션 변수 저장.

dicCaliPotEncoder = {}  # 스텝별 엔코더값 저장.
dicCaliNotLoad = {}  # 스텝별 엔코더값 저장.
dicCurrentEncoder = {}  # 모터별 엔코더값 저장.
dicInitTimeStamp = {}  # 모터별 초기화 명령 호출 시각 저장.
dic_485ack = {}
dic_485_laststart_timestamp = {}
dic_485_lastpos_started = {}
dic_485_lastpos_target = {}
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

def TorqueDataRead(mbidStr):
    global dic_485Torque
    global dic_485OverLoad
    return dic_485Torque.get(mbidStr, []), dic_485OverLoad.get(mbidStr,[])

def TorqueDataInsert(mbidStr, torqueData, ovrLoadData):
    global dic_485Torque
    global dic_485OverLoad
    lsTorqueData = [torqueData]
    lsovrLoadData = [ovrLoadData]
    if mbidStr in dic_485Torque.keys():
        dic_485Torque[mbidStr] = dic_485Torque[mbidStr] + lsTorqueData
        dic_485OverLoad[mbidStr] = dic_485OverLoad[mbidStr] + lsovrLoadData
    # else:
    #     dic_485Torque[mbidStr] = lsTorqueData

def TorqueDataClear(mbidStr):
    global dic_485Torque
    global dic_485OverLoad
    if mbidStr in dic_485Torque.keys():
        dic_485Torque[mbidStr].clear()
        dic_485OverLoad[mbidStr].clear()
    elif mbidStr == None:
        dic_485Torque.clear()
        dic_485OverLoad.clear()
        

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
    global dic_485Torque
    global dic_485OverLoad
    dtnow = 1
    try:
        lsResult = []
        success = False
        recvData = (str)(data.data)
        logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
        if not testModbus:
          #rospy.loginfo(logmsg)
          SendInfoHTTP(logmsg)
          logger_motor.info(logmsg)
          
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
        #print(lsResult)
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
                            if sParamValue == '-1' and sCMDValue == MotorCmdField.WSTOP.name:
                              sParamValue = '50'
                              
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
                    #print(writeValueArray)
                    WriteRegEx(writeAddress, writeValueArray, sMBID)

                #CMD_Queue
                
                # if sCMDValue == MotorCmdField.WGAIN.name or sCMDValue == MotorCmdField.WRATIO.name:
                #     while(len(CMD_Queue) > 0):
                #         time.sleep(0.1)
                #     Setup(sMBID)
                    
                if sCMDValue == MotorCmdField.WMOVE.name:
                    dtnow = getDateTime().timestamp()
                    lock.acquire()
                    try:
                        if dic_485ack.get(intMBID, None) == None:
                            dic_485ack[intMBID] = []
                            
                        dic_485ack[intMBID].append(dtnow)
                        dic_485_laststart_timestamp[sMBID] = dtnow                        
                        dic_485_lastpos_started[sMBID] = dic_status[sMBID][MonitoringField.CUR_POS.name]
                        dic_485_lastpos_target[sMBID] = recvDataMap.get(MotorWMOVEParams.POS.name, MIN_INT)
                        
                    finally:
                        # Lock을 해제해서 다른 Thread도 사용할 수 있도록 만든다.
                        lock.release()
                    pub_flag.publish(f"{dtnow}{sDivFieldColon}0{sDivFieldColon}{sMBID}")
                    dic_485Torque[sMBID] = []
                    dic_485OverLoad[sMBID] = []
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
alarm_dict = {f"{member.value:X}".zfill(3): member.name for member in AlarmCodeList}
for k, v in alarm_dict.items():
    dic_ServoMonitorAlarm.setdefault(k, v)

"""
Setup()
1. ModbusConfig.txt 라인단위로 리딩, id - 프로필 파일명을 map 으로 만들고 퍼블리셔 객체 생성 -> dic_topic
2. Subscribe Callback 등록
3. loop
"""
dicModbus = {}
# bSetupRequested = False
dicSetupRequested = {}
dicMB_alive = {}
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
isSetupNow = True

def Setup(mbidCur=None):
    global isSetupNow
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
    #global dic_485TorQueLimitInfo
    # ModbusConfig.txt 를 읽어들인다.
    isSetupNow = True
    dicConfigTmp = getDic_FromFile(filePath_modbusconfig, sDivEmart)
    for mbid, mbName in dicConfigTmp.items():
        if mbidCur is not None and not is_equal(mbidCur,mbid):
            continue
        iMbid = try_parse_int(mbid)
        # 만일 주석이라면 건너뛴다.
        if iMbid == 0:
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
            instrumentH: minimalmodbus.Instrument = None
            instrumentH = minimalmodbus.Instrument(
                port485, (int)(mbid), minimalmodbus.MODE_RTU
            )
            instrumentH.serial.close()
            instrumentH.serial.parity = serial.PARITY_NONE
            instrumentH.serial.stopbits = serial.STOPBITS_ONE
            if fastModbus:
                instrumentH.serial.baudrate = 115200  # Baud
                instrumentH.serial.timeout = 0.1  # seconds
            else:
                instrumentH.serial.baudrate = 115200  # Baud
                instrumentH.serial.timeout = 0.5  # seconds

            if dicSetupRequested.get(mbid, None) == None:
                dicSetupRequested[mbid] = False

            instrumentH.clear_buffers_before_each_transaction = True

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
            if mbid in lsSlowDevices:
                continue
            try:
                # 이 부분에 모터 초기화 통신 및 알람 푸쉬 루틴 구현

                # RS로 시작하면 서보모터여서 별도의 초기화 필요
                # ICS 모터인 경우 리딩 매핑작업이 되어있는지 검사 (0f10 값이 0 이면 공장출하값)
                # 공장출하상태시 WSETUP 커맨드 날림.
                # 예외처리 할 것.
                # checkAddr 이 -1 로 나오는 경우 구성정보 자체가 잘못되었으므로 알람이 가야됨.
                # 이 경우 Invalid Config 로 알람낼것.
                #checkAddrIsCCW = poll485[0].get(POLL_COMMON.START.name, "-1")
                # checkAddrAutoGain = '7' if mbName.startswith("RS2") else '7'
                # checkAddrValAutoGain = int(checkAddrAutoGain, 16)
                # checkAddrIsRatio = '9' if mbName.startswith("RS2") else '9'
                # checkAddrVaRatio = int(checkAddrIsRatio, 16)
                checkAddrIsCCW = '7' if mbName.startswith("RS2") else 'D'
                checkAddrValCCW = int(checkAddrIsCCW, 16)
                # checkAddrIsPOTChanged = '0x149' if mbName.startswith("RS2") else '0x409'
                # checkAddrValPOTChanged = int(checkAddrIsPOTChanged, 16)
                # # 토크 리밋에 관련된 변수 설정. 추후개발예정
                # checkAddrIsToqLimit = '0x6014' if mbName.startswith("RS2") else '0x1B'
                # checkAddrValToqLimit = int(checkAddrIsToqLimit, 16)

                # checkAddrIsToqLimit = '0x6014' if mbName.startswith("RS2") else '0x1B'
                # checkAddrValToqLimit = int(checkAddrIsToqLimit, 16)
                pub_kpalive = rospy.Publisher(
                    f"{TopicName.MB_.name}{mbid}", String, queue_size=ROS_TOPIC_QUEUE_SIZE
                )
                dic_topics[mbid] = pub_kpalive
                dic_485ctl[mbid] = instrumentH
                dic_485cmd[mbid] = dicConfigcmd
                dic_485poll[mbid] = poll485
                if mbName.startswith("RS"):
                    dicInitTimeStamp[mbid] = getDateTime()
                    checkMBresult = instrumentH.read_registers(checkAddrValCCW, 1, 3)
                    dic_485Inverted[mbid] = checkMBresult[0]
                    # checkMBresult = instrumentH.read_registers(checkAddrValPOTChanged, 1, 3)
                    # dic_485POTInfo[mbid] = checkMBresult[0]
                    # checkMBresult = instrumentH.read_registers(checkAddrValToqLimit, 1, 3)
                    # dic_485TorQueLimitInfo[mbid] = checkMBresult[0]
                    # checkMBresult = instrumentH.read_registers(checkAddrValToqLimit, 1, 3)
                    # dic_485TorQueLimitInfo[mbid] = checkMBresult[0]
                    # checkresultAG = instrumentH.read_registers(checkAddrValAutoGain, 1, 3)
                    # dic_485AutoGain[mbid] = checkresultAG[0]
                    # checkresultRatio = instrumentH.read_registers(checkAddrVaRatio, 1, 3)
                    # dic_485Inertia_Ratio[mbid] = checkresultRatio[0]
                    initMotor(mbid)
                    rospy.loginfo(f"Check MB({mbid}) result OK at try {iTryCheckModbus+1}")
                    # if is_equal(mbid, ModbusID.MOTOR_H.value):
                    #     rospy.loginfo(f"Check MB({mbid}) result OK at try {iTryCheckModbus+1}")
                    #     print(dic_485Inertia_Ratio,dic_485AutoGain)
                    bMotorOK = True
                    #break
            except Exception as e:
                exc_type, exc_value, exc_traceback = sys.exc_info()
                line_number = exc_traceback.tb_lineno
                dicCurrentEncoder[mbid] = getDateTime()
                logMsgErr = f"Check MB({mbid}) result Error at {line_number}, try {iTryCheckModbus+1} : {e}"
                rospy.loginfo(logMsgErr)
                time.sleep(0.1)
            if bMotorOK:
                LoadCurrentPos()
                logMsgErr = f"Check MB({mbid}) OK"
                dicMB_Exception_count[mbid] = 0
                rospy.loginfo(logMsgErr)
            else:
                dic_topics[mbid].publish(
                    f"{MonitoringField_EX.ALM_CD.name}{sDivFieldColon}{AlarmCodeList.NOT_CONNECTED.value}{sDivItemComma}{MotorWMOVEParams.MBID.name}{sDivFieldColon}{mbid}"
                )
                dic_485ack.pop(int485id)
                rospy.loginfo(dic_485ack)

        except Exception as e:
            message = traceback.format_exc()
            logmsg = f"{message} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
            rospy.loginfo(logmsg)
    rospy.loginfo(dic_485Inverted)
    #rospy.loginfo(dic_485POTInfo)    
    isSetupNow = False
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


def initMotor(mbidAll=None):
    global dicCali
    lsMbid = []
    callData = String()
    callData2 = String()
    callData3 = String()

    if mbidAll == None:
        lsMbid.extend(dic_485poll.keys())
    else:
        lsMbid.append(mbidAll)
    
    for mbid in lsMbid:  # 장치ID 별로 엑세스
        bInitReverse = dicConfigInitReverse[mbid]
        imbid = try_parse_int(mbid)
        dicCali[mbid] = MOTOR_CALI_STATUS.Z_NOT_COMPLETED
        initStr = (
            MotorCmdField.WINITREVERSE.name if bInitReverse else MotorCmdField.WINIT.name
        )
        callData.data = (f"{MotorWMOVEParams.MBID.name}:{mbid},{MotorWMOVEParams.CMD.name}:{initStr}")
        dicTorqueTmp = getDic_FromFile(filePath_Torqueconfig, sDivEmart)
        callbackCmd(callData)
        tql=dicTorqueTmp.get(mbid, MIN_INT)
        pot_pos=int(dicCaliFinalPos.get(mbid, MAX_INT))
        not_pos=int(dicCaliNotLoad.get(mbid, MIN_INT))
        
        # if tql != MIN_INT:
        #     dic_485TorQueLimitInfo[mbid] = tql        
        #     callData2.data = getMotorTorqueString(mbid, tql)
        #     callbackCmd(callData2)
        
        if pot_pos != MAX_INT and not_pos != MIN_INT:
            #callData3.data = getMotorSetPOTNOTString(mbid, not_pos,pot_pos)
            callData3.data = getMotorSetPOTNOTString(mbid, -pot_pos,pot_pos)
            callbackCmd(callData3)        
        logmsg = f"{mbid} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
        rospy.loginfo(logmsg)
    
def initMotorSetup(mbid):
    # callData = String()
    # callData.data = f"{MotorWMOVEParams.MBID.name}:{mbid},{MotorWMOVEParams.CMD.name}:{MotorCmdField.WSETUP.name}"
    # callbackCmd(callData)
    initMotorCustom(mbid, MotorCmdField.WSETUP)
    
def resetPulseNot(mbid):
    return stopMotor(mbid, decc=EMERGENCY_DECC, resetPulse=True)

def stopMotor(mbid, decc=EMERGENCY_DECC,resetPulse=False):
    callData = String()
    #callData.data = f"{MotorWMOVEParams.MBID.name}:{mbid},{MotorWMOVEParams.CMD.name}:{cmdParam.name}"
    callData.data = getMotorStopString(mbid, decc)
    callbackCmd(callData)
    if resetPulse:
        not_pos_mbid = dicCaliNotLoad.get(mbid,0)
        initWLoc(mbid, not_pos_mbid)
        #initMotorCustom(mbid, MotorCmdField.WZERO)  # 엔코더값 0으로 초기화 한다.
        #time.sleep(MODBUS_WRITE_DELAY)

def alarmClear(mbidAll=None):
    lsMbid = []
    callData = String()

    if mbidAll == None:
        lsMbid.extend(dic_485poll.keys())
    else:
        lsMbid.append(mbidAll)
    
    for mbid in lsMbid:            
        dicLoc=getMotorSimpleCmdString(mbid, MotorCmdField.WALM_C)
        callData = String()
        callData.data = dicLoc
        callbackCmd(callData)
    
    
def stopAllMotors(decc=EMERGENCY_DECC,resetPulse=False):
  log_all_frames()
  for modbusIDTmp in dic_485poll.keys():  # 장치ID 별로 엑세스
    if modbusIDTmp in lsNotMotor:
        continue
    #int485id = int(modbusIDTmp)
    #lsAckList = dic_485ack.get(int485id, None)
    stopMotor(modbusIDTmp,decc,resetPulse)

    # if lsAckList is None:
    #   dic_485ack[int485id] = []
    # else:
    #   if len(dic_485ack[int485id]) > 0:
    #     for cmdID in dic_485ack[int485id]:
    #       # 주어진 타임스탬프
    #       given_timestamp = cmdID
    #       # 현재 시간 타임스탬프
    #       current_timestamp = time.time()
    #       # 타임스탬프 차이 계산
    #       time_difference = current_timestamp - given_timestamp
    #       # 밀리초 단위로 변환
    #       time_difference_ms = time_difference * 1000
    #       rospy.loginfo(
    #           f"Popping Alarm Ack:{int485id},time:{time_difference_ms:.1f}"
    #       )
    #       cmdID = dic_485ack[int485id].pop()
    #       pub_flag.publish(
    #           f"{cmdID}{sDivFieldColon}1{sDivFieldColon}{modbusIDTmp}"
    #       )    

def initWLoc(mbid, pos ):
    callData = String()
    callData.data = f"{MotorWMOVEParams.MBID.name}:{mbid},{MotorWMOVEParams.CMD.name}:{MotorCmdField.WLOC.name},{MotorWMOVEParams.POS.name}:{pos}"
    callbackCmd(callData)

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


class MotorCtl:
    def __init__(self):
        self.set_saveImage_service = rospy.Service("/carrier/motormove", modbus_motor, self.MotorMove)
        
        self.CMD_service = rospy.Service(ServiceBLB.CMD_DEVICE.value, Kill, self.SendCMD)
        self.ESTOP_service = rospy.Service(ServiceBLB.CMD_ESTOP.value, Kill, self.SendESTOP)
        self.AlarmClear_service = rospy.Service(ServiceBLB.CMD_ALMC.value, Kill, self.SendAlarmAllClear)
        self.SavePos_service = rospy.Service(ServiceBLB.CMD_SAVE.value, Kill, self.SavePos)
        self.Status_service = rospy.Service('/get_modbus', Trigger, self.handle_custom_service)

    def handle_custom_service(self,req):
        listReturn = []
        lock.acquire()
        try:
            for modbusID in dic_485poll.keys():  
                # 장치ID 별로 엑세스
                cmdData = dic_485poll[modbusID]
                # 한 디바이스에서 2개 이상 다른 주소로 Polling시 cmdData 원소가 여러개가 된다.
                for dicRecord in cmdData:
                    start_addr = 1
                    if POLL_COMMON.START.name not in dicRecord.keys():
                        start_addr = 0
                    if try_parse_int(dicRecord[POLL_COMMON.START.name], MIN_INT) == MIN_INT:
                        strStartAddr = dicRecord[POLL_COMMON.START.name][1:]
                        intStart_addr = try_parse_int(strStartAddr)
                        len_items = (dicRecord[POLL_COMMON.LEN.name]) / 16
                        rcv = pollRead(modbusID,intStart_addr,int(len_items))
                        rcv[MotorWMOVEParams.MBID.name] = modbusID
                        dicTmpU = copy.deepcopy(rcv)
                        listReturn.append(rcv)

            # 반환할 값 설정
            response = json.dumps(listReturn)
            # response = {
            #     'time': {
            #         'secs': 1724738649,
            #         'nsecs': 779511451
            #     }
            # }
            #return TriggerResponse(success=True, message=str(response))
        finally:
            lock.release()
        return TriggerResponse(success=True, message=response)

    def SendAlarmAllClear(self, req):
        alarmClear()
        resp = KillResponse()
        return resp
    
    def SendESTOP(self, req):
        msg = req.name
        resetPulseHeader = msg[0]
        resetPulse = isTrue(resetPulseHeader)
        stopSmoothTimeMs = int(msg[1:])
        stopAllMotors(stopSmoothTimeMs,resetPulse)
        resp = KillResponse()
        return resp
    
    def SavePos(self, req):
        dicPos = {}
        for mbid in dic_485poll.keys():
            mbid_instance = ModbusID.from_value(mbid)
            cur_pos =dicCurrentEncoder.get(mbid)
            if cur_pos is not None:
                dicPos[mbid] = cur_pos
        
        with open(strFileLastPos, "w") as file:
            json.dump(dicPos, file, indent=4, sort_keys=True)
        resp = KillResponse()
        return resp
    
    def SendCMD(self, req):
        print(log_all_frames())
        msg = req.name
        callData.data = msg
        respID = callbackCmd(callData)
        resp = KillResponse()
        return resp

    def MotorMove(self, req):
        global callData
        # global motorOpCheck
        global dic_485ack
        req_Cmd = req.cmd
        req_mbID = req.mb_id
        req_mode = req.mode
        req_pos = req.pos
        req_spd = req.spd
        req_acc = req.acc
        req_decc = req.decc
        # sCmd = (f'{MotorWMOVEParams.MBID.name}{sDivFieldColon}{req_mbID}{sDivItemComma}'
        #         f'{MotorWMOVEParams.CMD.name}{sDivFieldColon}{req_Cmd}{sDivItemComma}'
        #         f'{MotorWMOVEParams.MODE.name}{sDivFieldColon}{req_mode}{sDivItemComma}'
        #         f'{MotorWMOVEParams.POS.name}{sDivFieldColon}{req_pos}{sDivItemComma}'
        #         f'{MotorWMOVEParams.SPD.name}{sDivFieldColon}{req_spd}{sDivItemComma}'
        #         f'{MotorWMOVEParams.ACC.name}{sDivFieldColon}{req_acc}{sDivItemComma}'
        #         f'{MotorWMOVEParams.DECC.name}{sDivFieldColon}{req_decc}'
        # )
        isabsolute = True
        if req_mode == str(ServoParam.MOVE_TYPE_REL.value):
            isabsolute = False
        sCmd = getMotorMoveString(
            req_mbID, isabsolute, req_pos, req_spd, req_acc, req_decc
        )
        callData.data = sCmd
        respID = callbackCmd(callData)
        # print(f'{sCmd}')
        # respID = int(getCurrentTime().replace(sDivFieldColon, '') )
        # respID=getDateTime().timestamp()

        # respID=getDateTime()
        resp = modbus_motorResponse(str(respID))
        # dic_485ack[req_mbID].append(respID)
        # motorOpCheck[respID] = getDateTime()
        # pub_flag.publish(f'{respID}{sDivFieldColon}0{sDivFieldColon}{req_mbID}')
        return resp


def print_time(chk):
    return
    print(f"{chk} - {getDateTime()}")


cntLoop = 0
# exception_read_count = 0
lastUpdateTimeStamp = getDateTime()
lastAlarmTimeStamp = getDateTime()
MotorCtl()

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
  instrumentTmp: minimalmodbus.Instrument = dic_485ctl[modbusID]
  # print(f"Try to read :{modbusID}")
  resultList = instrumentTmp.read_registers(
      start_addr, (int)(len_items), 3
  )
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
              FinalValueStr = f"{FinalValue :.1f}"
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
        lsModifiedMbid = []
        while len(CMD_Queue) > 0:
            cmd_buf = list(CMD_Queue.popleft())
            addr = cmd_buf[0]
            valueList = cmd_buf[1]
            drvID = cmd_buf[2]
            caller = cmd_buf[3]
            if len(valueList) <= 1:
                valueStr = ','.join([f"0x{val:02X}" for val in valueList])  # 16진수 출력
            else:
                valueStr = ','.join([str(val) for val in valueList])  # 10진수 출력

            infoStr = f"Addr:0x{addr:04X},Value:{valueStr},MB_ID:{drvID} from {caller}"
            if drvID not in lsModifiedMbid:
                lsModifiedMbid.append(drvID)
            #infoStr = f"Addr:0x{addr:04X},Value:{valueList},MB_ID:{drvID} from {caller}"
            instrumentTmp: minimalmodbus.Instrument = dic_485ctl[drvID]
            if not testModbus:
              rospy.loginfo(f"Trying:{infoStr}")
            if len(valueList) == 1:
                instrumentTmp.write_register(addr, valueList[0])
            else:
                instrumentTmp.write_registers(addr, valueList)
            dicMB_alive[drvID] = True
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
        
        if testModbus and all(dicMB_alive.values()):
          PrintColorDict(dicMB_alive)
          #print(dicMB_alive)
    except Exception as e:
        dicMB_alive[drvID] = False
        if cmd_buf != None and testModbus:
            #CMD_Queue.appendleft(cmd_buf)
            # # Value를 기준으로 먼저 정렬하고, Value가 같을 경우 Key를 기준으로 정렬
            # sorted_data = dict(sorted(dicMB_alive.items(), key=lambda x: (x[1], x[0])))
            # print(sorted_data)
            #print(dicMB_alive)
            # 한 줄에 모든 값을 출력
            PrintColorDict(dicMB_alive)
        else:
          rospy.loginfo(f"write_registers error : {cmd_buf} - {e}")
          pass
        # bReturn = False
        # rospy.loginfo(traceback.format_exc())
        # dtNow = getDateTime()
        # td = getDateTime() - lastLogTime
    for drvIDExecuted in lsModifiedMbid:
        for poll_id in dic_485pollRate.keys():
            poll_id_str = str(poll_id)    
            if poll_id_str.startswith(str(drvIDExecuted)) :
                dic_485pollRate[poll_id] = DATETIME_OLD
    lsModifiedMbid.clear()
    
    if not testModbus and not isSetupNow:
      # TODO : Polling Read Part
      try:
          print_time(2)
          for modbusID in dic_485poll.keys():  # 장치ID 별로 엑세스
              return485data.clear()
              cmdData = dic_485poll[modbusID]
              # print(type(cmdData))
              print_time(f"{modbusID}-0")

              # 한 디바이스에서 2개 이상 다른 주소로 Polling시 cmdData 원소가 여러개가 된다.
              readLineIdx = -1
              for dicRecord in cmdData:
                  readLineIdx += 1
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
                  
                  #서빙텔레스코프가 동작할때는 TOF센서 감지빈도를 최대화 한다
                  if int485id == ModbusID.TOF.value:
                    lsAck11 = dic_485ack.get(ModbusID.TELE_SERV_MAIN.value, None)
                    if lsAck11 != None and len(lsAck11) > 0:
                        pollrate = 10
                        
                  if lsAckList != None and len(lsAckList) > 0:
                      bOnMoving = True
                      #if pollrate < 5000:
                      pollrate = 10
                      strCurPos = ""
                      # rospy.loginfo(
                      #     f"Ack Check : {int485id} - {dic_485ack}, {dicCurrentEncoder}"
                      # )

                  if pollrate > pollrate_min:
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
                      
                      #움직이는 동안에는 전송딜레이를 최소화 하기 위해
                      #포지션데이터와 센서데이터만 전송한다. (포지션+센서 데이터는 readLineIdx 가 무조건 0 이다.)
                      if pollrate == 10 and readLineIdx > 1:
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
                          MonitoringField.ST_ENABLE.name, -1
                      )
                      currentAlarmCode = dicModbus.get(
                          MonitoringField.ALM_CD.name, -1
                      )
                      currentAlarmCaption = dicModbus.get(
                          MonitoringField.ALM_NM.name, -1
                      )
                      dic_485pollRate[poll_id] = getDateTime()

                      mbiderr_cnt = try_parse_float(dicMB_Exception_count.get(modbusID), -1)
                      if mbiderr_cnt > 10:
                          Setup(modbusID)
                      # BMS, NTP 인 경우는 그냥 넘어감
                      elif modbusID in lsSlowDevices or modbusID in lsNotMotor:
                          dicMB_Exception_count[modbusID] = 0
                      # currentEnableStatus 이 -1 인 경우 - 연결자체가 안 된경우 (이럴땐 setup() 을 다시 해줘야 함)
                      elif currentEnableStatus == -1:
                          raise Exception("Modbus timeout")
                      # currentEnableStatus 가 0 인 경우 - 통신은 되는데 초기화가 안 된경우 (초기화만 해주면 됨)
                      elif currentEnableStatus == 0 and currentAlarmCode == 0:
                          #initMotor()
                          Setup(modbusID)
                          dicMB_Exception_count[modbusID] = 0
                      # currentEnableStatus 가 1 이면 정상
                      elif currentEnableStatus == 1:
                          dicMB_Exception_count[modbusID] = 0
                      # 그 외는 환경설정 에러 - 알람발생
                      else:
                          if isTimeExceeded(lastAlarmTimeStamp, 1000):
                            max_length = max(len(value) for value in dic_485ack.values())
                            if max_length > 0:
                              #한개의 모터라도 알람 발생할 경우 모터 일괄정지.
                              stopAllMotors()
                              alarmClear()
                              lastAlarmTimeStamp = getDateTime()
                              curAlarmMsg = f"{modbusID}번모터;{currentAlarmCaption};{currentAlarmCode}"
                              if curAlarmMsg != lastAlarmMsg:
                                print(lastAlarmMsg)
                                SendAlarmHTTP(lastAlarmMsg)
                                lastAlarmMsg=curAlarmMsg
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

                              if isHome:
                                  dicCali[modbusID] = MOTOR_CALI_STATUS.C_JOGCCW_REQUESTED
                                  dictAppend(
                                      dicCaliPotEncoder,
                                      modbusID,
                                      MOTOR_CALI_STATUS.B_JOGCW_REQUESTED,
                                      cur_pos,
                                  )
                              else:
                                  dictAppend(
                                      dicCaliPotEncoder,
                                      modbusID,
                                      MOTOR_CALI_STATUS.A_REQUESTED,
                                      cur_pos,
                                  )
                                  dicCali[modbusID] = MOTOR_CALI_STATUS.B_JOGCW_REQUESTED
                              print(dicCaliPotEncoder)
                                  
                      elif caliStatus == MOTOR_CALI_STATUS.B_JOGCW_REQUESTED:
                          prePos = dicCaliPotEncoder.get(modbusID)[
                              MOTOR_CALI_STATUS.A_REQUESTED
                          ]
                          differenceEnc = abs(
                              int(cur_pos) - int(prePos)
                          )  # 이전 단계와의 엔코더 격차를 구한다
                          # if isHome and not isMotorStopped:
                          if isNot:  # 정방향으로 운전중인데 not 가 감지되었다면
                              # 정방향으로 10000 이상 움직였는데 마주친 것이 NOT 라면 POT와 NOT 를 바꾸어야 한다.
                              if differenceEnc > roundPulse:
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
                          elif isHome and differenceEnc > roundPulse:
                              dicCali[modbusID] = MOTOR_CALI_STATUS.C_JOGCCW_REQUESTED
                              dictAppend(
                                  dicCaliPotEncoder,
                                  modbusID,
                                  MOTOR_CALI_STATUS.B_JOGCW_REQUESTED,
                                  cur_pos,
                              )
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
                          prePos = dicCaliPotEncoder.get(modbusID)[
                              MOTOR_CALI_STATUS.B_JOGCW_REQUESTED
                          ]
                          differenceEnc = abs(
                              int(cur_pos) - int(prePos)
                          )  # 이전 단계와의 엔코더 격차를 구한다
                          
                          # 역방향으로 캘리시작
                          # if isMotorStopped:  #모터가 멈춘 상태 (다 돌아갔음)
                          if isNot or (isHome and differenceEnc > roundPulse):
                              dicCali[modbusID] = MOTOR_CALI_STATUS.D_CALI_COMPLETED
                              dictAppend(
                                  dicCaliPotEncoder,
                                  modbusID,
                                  MOTOR_CALI_STATUS.D_CALI_COMPLETED,
                                  abs(cur_pos),
                              )
                              dicCaliFinalPos[modbusID] = abs(cur_pos)
                              saveDic_ToFile(dicCaliFinalPos, filePath_CaliPotConfig, sDivEmart, True)
                              initMotorCustom(modbusID, MotorCmdField.WZERO)  # 엔코더값 0으로 초기화 한다.

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
                      #rospy.loginfo(traceback.format_exc()){addr:04X}
                      rospy.loginfo(f"{dicConfigTmp[modbusID]}:{poll_id}-{e} : Count:{dicMB_Exception_count[modbusID]}")
                      #rospy.loginfo(f"{dicConfigTmp[modbusID]}:{int(poll_id):04X}-{e} : Count:{dicMB_Exception_count[modbusID]}")
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
                  return485data[MonitoringField.LASTSEEN.name] = getDateTime().timestamp()
                  return485data[MotorWMOVEParams.MBID.name] = modbusID
                  if modbusID in lsNotMotor:
                      sendbuf = json.dumps(return485data)
                      dic_topics[modbusID].publish(sendbuf)
                      continue
                  dicCaliPotInfo = dicCaliPotEncoder.get(modbusID, {})
                  not_pos_mbid = dicCaliNotLoad.get(modbusID,0)
                  caliPosCurrent = dicCaliPotInfo.get(MOTOR_CALI_STATUS.D_CALI_COMPLETED, -1)
                  if caliPosCurrent > 0:
                      return485data[MonitoringField.NOT_POS.name] = not_pos_mbid
                      return485data[MonitoringField.POT_POS.name] = caliPosCurrent
                  else:
                      return485data[MonitoringField.POT_POS.name] = return485data[MonitoringField.NOT_POS.name] = caliPosCurrent                    
                  NoValue = -1
                  isCCW = dic_485Inverted.get(modbusID, NoValue)
                #   autogain = dic_485AutoGain.get(modbusID, NoValue)
                #   inertia_ratio = dic_485Inertia_Ratio.get(modbusID, NoValue)
                #   ToqLimit = dic_485TorQueLimitInfo.get(modbusID,NoValue)
                #   return485data[MonitoringField.AUTO_GAIN.name] = autogain
                #   return485data[MonitoringField.INERTIA_RATIO.name] = inertia_ratio
                  return485data[MonitoringField.IS_CCW.name] = isCCW
                #   return485data[MonitoringField.TOQ_LIMIT.name] = ToqLimit
                  rpm = int(return485data.get(MonitoringField.CUR_SPD.name,NoValue))
                  isCCW = int(return485data.get(MonitoringField.IS_CCW.name,NoValue))
                  cur_current = int(return485data.get(MonitoringField.CUR_CURRENT.name, NoValue))
                  cur_current = abs(cur_current) if cur_current != NoValue else cur_current

                  cur_torque = int(return485data.get(MonitoringField.CUR_TORQUE.name,NoValue))
                  cur_torque = (cur_torque) if cur_torque != NoValue else cur_torque

                  cur_ovrLoad = int(return485data.get(MonitoringField.OVER_LOAD.name,NoValue))
                  cur_ovrLoad = abs(cur_ovrLoad) if cur_ovrLoad != NoValue else cur_ovrLoad
                  
                  #CCW 로 설정된 모터는 토크,속도,전류의 부호를 교정한다.
                  if isTrue(isCCW):
                      if rpm != NoValue:
                          rpm = -rpm
                  
                  return485data[MonitoringField.CUR_SPD.name] = rpm
                  return485data[MonitoringField.CUR_CURRENT.name] = cur_current
                  return485data[MonitoringField.OVER_LOAD.name] = cur_ovrLoad
                  return485data[MonitoringField.CUR_TORQUE.name] = cur_torque
                  laststart = DATETIME_OLD
                  if dic_485_laststart_timestamp.get(modbusID) is not None:
                      laststart = dic_485_laststart_timestamp.get(modbusID)
                  return485data[MonitoringField.LASTSTART.name] = laststart
                  
                  cur_pos = return485data.get(MonitoringField.CUR_POS.name)
                  if cur_pos is not None:
                    lastpos = return485data[MonitoringField.CUR_POS.name]
                    if dic_485_lastpos_started.get(modbusID) is None:
                        return485data[MonitoringField.LAST_STARTED_POS.name] = cur_pos
                        return485data[MonitoringField.LAST_TARGET_POS.name] = cur_pos
                    else:
                        lastpos = dic_485_lastpos_started.get(modbusID)
                        lasttarget = dic_485_lastpos_target.get(modbusID)
                        return485data[MonitoringField.LAST_STARTED_POS.name] = lastpos
                        return485data[MonitoringField.LAST_TARGET_POS.name] = lasttarget
                        
                  if modbusID in dic_485Torque.keys() and cur_torque != MIN_INT:
                      TorqueDataInsert(modbusID, cur_torque,cur_ovrLoad)
                  #POTInfo = dic_485POTInfo.get(modbusID,MIN_INT)
                  
                  #return485data[MonitoringField.POT_ALLOCATION.name] = dicSICode.get(POTInfo,MIN_INT)
                  isESTOP = return485data.get(f"{MonitoringField.DI_ESTOP.name}", '-1')
                  alarm_name = "NONE"
                  alarm_code = return485data.get(f"{MonitoringField.ALM_CD.name}", '-1')
                  if alarm_code != '-1':
                      hexAlmCode = hex(int(alarm_code))[2:].rjust(3,'0').upper()
                      alarm_name = dic_ServoMonitorAlarm.get(hexAlmCode,"UNKNOWN")
                      return485data[MonitoringField.ALM_NM.name] = alarm_name
                    
                  #이 부분은 개발기에서 마무리하고 테스트하자.
                #   if isTrue(isESTOP):
                #     return485data[MonitoringField.ALM_NM.name] = AlarmCodeList.ESTOP_ERROR.name
                #     return485data[MonitoringField.ALM_CD.name] = AlarmCodeList.ESTOP_ERROR.value
                    
                  dicSIStatus = {}
                  for key, value in return485data.items():
                      if key.startswith('SI'):
                          iVal = int(value)
                          statusSTR = dicSICode.get(iVal,value)
                          dicSIStatus[key] = statusSTR
                  if len(dicSIStatus) > 0:
                      return485data.update(dicSIStatus)
                      
                  #     dicSI_TotalStatus[modbusID] = 
                  dicCurStatus = dic_status.get(modbusID)
                  if dicCurStatus is None:
                    dic_status[modbusID] = {}
                  dic_status[modbusID].update(return485data)
                  sendbuf = getStr_fromDic(dic_status[modbusID], sDivFieldColon, sDivItemComma)
                  dic_topics[modbusID].publish(sendbuf)
                  int485id = int(modbusID)
                  # motorOpCheck = dic_485ack[int485id]

                  # for cmdID in list(motorOpCheck.keys()):
                  #     timeStamp = motorOpCheck[cmdID]
                  #     if isTimeExceeded(timeStamp, 100):
                  motorModel = dicConfigTmp.get(modbusID, "")
                  flagPOT = return485data.get(f"{MonitoringField.DI_POT.name}", None)
                  flagNOT = return485data.get(f"{MonitoringField.DI_NOT.name}", None)
                  flagESTOP = return485data.get(f"{MonitoringField.DI_ESTOP.name}", None)
                  flagHOME = return485data.get(f"{MonitoringField.DI_HOME.name}", None)
                  flagSPD = return485data.get(f"{MonitoringField.CUR_SPD.name}", None)
                  flagPOS = return485data.get(f"{MonitoringField.CUR_POS.name}", None)
                  if isTrue(flagNOT):
                    cur_pos = try_parse_int(return485data.get(f"{MonitoringField.CUR_POS.name}"), MIN_INT)
                    if cur_pos != MIN_INT:  
                        if modbusID in dicCaliNotLoad.keys():
                            not_pos_mbid = dicCaliNotLoad.get(modbusID,0)
                            if not is_within_range(cur_pos, not_pos_mbid, roundPulse):
                                resetPulseNot(modbusID)
                            
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
                              lsTorqueData,lsovrLoadData = TorqueDataRead(modbusID)
                              meanTorque = -1
                              maxTorque = -1
                              maxOvr = -1
                              minTorque = -1
                              meanOvr = -1
                              if len(lsTorqueData) > 0:
                                  meanTorque = round(statistics.mean(lsTorqueData) )
                                  maxTorque = max(lsTorqueData)
                                  minTorque = min(lsTorqueData)
                              if len(lsovrLoadData) > 0:
                                  meanOvr = round(statistics.mean(lsovrLoadData) )
                                  maxOvr = max(lsovrLoadData)
                              TorqueDataClear(modbusID)
                              try:
                                rospy.loginfo(
                                    f"Popping Ack:{int485id},time:{time_difference_ms:.1f},NOT/POT({flagNOT}:{flagPOT}),Spd:{flagSPD},Pos:{flagPOS},MotorFlag:{motorFlag},TORQUE_MAX:{maxTorque},TORQUE_AVE:{meanTorque},OVERLOAD_MAX:{maxOvr},OVERLOAD_AVE:{meanOvr},TORQUE_MIN:{minTorque}"
                                )
                              except Exception as e:
                                  SendAlarmHTTP(e,False)
                              lastpos = dic_485_lastpos_started.get(modbusID)
                              lasttarget = dic_485_lastpos_target.get(modbusID)                          
                              ackMsg = f"{cmdID}{sDivFieldColon}{motorFlag}{sDivFieldColon}{modbusID}{sDivFieldColon}{maxTorque}{sDivFieldColon}{meanTorque}{sDivFieldColon}{minTorque}{sDivFieldColon}{maxOvr}{sDivFieldColon}{lastpos}{sDivFieldColon}{lasttarget}{sDivFieldColon}{cur_pos}{sDivFieldColon}{flagSPD}{sDivFieldColon}{flagHOME}{sDivFieldColon}{flagPOT}{sDivFieldColon}{flagNOT}{sDivFieldColon}{flagESTOP}"
                              rospy.loginfo(ackMsg)
                              pub_flag.publish(ackMsg)
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
    else:
      #print(getCurrentTime())
      if len(CMD_Queue) == 0:
        stopAllMotors()

    rate.sleep()
