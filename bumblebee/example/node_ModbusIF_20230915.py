#! /usr/bin/env python3
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
#from ServoNano import *
#import seaborn as sns
import pandas as pd
import numpy as np
from io import StringIO
from UtilBLB import *
import rosparam
import roslib
from collections import deque
from tta_blb.srv import *
import rosparam

'''
구분자 정의 : 
1차 구분자 - 이마트 어퍼스트로피 `
2차구분자 - 콜론 :
3차구분자 - 쉼표 (레지스터 단위 구분)
4차구분자 - 슬래쉬 : 레지스터 연속값 구분
'''
CMD_Queue = deque() #모드버스 통신 동기화를 위한 Write 버퍼
#publish_topic_goal  : str = 'CMD' #테스트용 변수
#publish_topic_name = 'MB'
#publish_topic_ACK = 'ACK'
fastModbus = rospy.get_param("~fast", default=False)
rospy.loginfo(f'Set Modbus Fast Mode : {fastModbus}')
nodeName = f'node_{TopicName.CMD_DEVICE.name}'
rospy.init_node(nodeName,anonymous=False) # 485 이름의 노드 생성
write_delay = 0.001
read_delay = 0.002
exception_delay = 0.01
param_DRV_Rate = 100
#port485 = '/dev/ttyS0' # 시리얼 포트
port485 = '/dev/ttyUSB0' # USB 포트
#dirPath = os.path.dirname(__file__)
dic_485ctl = {}
dic_485cmd = {}
dic_485poll = {}
dic_485pollLen = {}
dic_485pollRate = {}
dic_topics = {}
dicConfigTmp = {}
dic_485ack = {}
#dic_485topic = {}
rate = rospy.Rate(param_DRV_Rate)
poll_common = ['START','RATE','LEN']
poll_START = poll_common[0]
poll_RATE = poll_common[1]
poll_LEN = poll_common[2]
#motorOpCheck = {}

#BMS 
lastLogTime = datetime.datetime.now()
lock = threading.Lock()

def WriteRegEx(addr : int, valueList : List ,drvID:str) -> bool:
    global CMD_Queue
    bPass = True
    cmd_buf = []
    try:
        cmd_buf.append(addr)
        cmd_buf.append(valueList)
        cmd_buf.append(drvID)
        cmd_buf.append(f'{sys._getframe(2).f_code.co_name} - {sys._getframe(1).f_code.co_name}')
        CMD_Queue.append(tuple(cmd_buf))
    except Exception as e:
        bPass = False
        rospy.loginfo(traceback.format_exc())
        rospy.signal_shutdown(e)    
    return bPass

def WriteReg(addr, value,drvID):
    listTmp = []
    listTmp.append(value)
    return WriteRegEx(addr,listTmp,drvID)

def callbackCmd(data):
    #TODO : 응답메세지 발행하는 것도 만들기.
    global dic_485ack
    global pub_flag
    global lock
    dtnow = 1
    try:
        success = False
        recvData = data.data
        logmsg = f'{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}'
        rospy.loginfo(logmsg)
        last_dish_washed = ''
        #rate = rospy.Rate(1)
        recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        print(recvDataMap)
        sMBID =recvDataMap.get(MotorWMOVEParams.MBID.name, '') 
        sCMDValue = recvDataMap.get(MotorWMOVEParams.CMD.name, None)#등록된 모드버스ID 인 경우!
        dicMBID = dic_485cmd.get(sMBID, None)#등록된 모드버스ID 인 경우!
        
        if dicMBID == None or sCMDValue == None:
            return
        
        sCMDParam = dicMBID.get(sCMDValue, None)
        '''
        sCMD : 'WMOVE'
        sCMDParam : '0x6200:MODE/POS_H/POS_L/SPD/ACC/DECC/0/0x10'
        sCMDParamSplit[0] = Write 주소
        sCMDParamSplit[1] = 파라미터 셋 ( / 로 스플릿 한 후 _H 와 _L 로 끝나는 필드는 예외처리 )
        '''
        writeAddress = 0
        if sCMDParam != None: #등록된 명령어인 경우
            sCMDParamTotalSplit = sCMDParam.split(sDivItemComma)
            for sCMDParamTmp in sCMDParamTotalSplit:#쉼표로 나눔 (명령어가 여러개)
                sCMDParamSplit = sCMDParamTmp.split(sDivFieldColon)
                writeAddress = try_parse_int(sCMDParamSplit[0], -1)
                writeParamsArray = sCMDParamSplit[1].split(sDivSlash) #슬래쉬로 나눔 (값이 배열인 경우)
                writeValueArray = []
                for sParam in writeParamsArray:
                    bSplitedParam = False
                    iParamValue = try_parse_int(sParam, -1)
                    bNeedToReplace = True if iParamValue == -1 else False
                    checkWord = sParam
                    if bNeedToReplace:
                        if sParam.find('_') > 0:
                            checkWord = sParam.split('_')[0]
                            bSplitedParam = True
                        sParamValue = recvDataMap.get(checkWord, '-1')
                        iParamFinalValue = try_parse_int(sParamValue)
                        if bSplitedParam:
                            param_H, param_L = splitSignedInt(iParamFinalValue)
                            if sParam.find('_H') > 0:
                                writeValueArray.append(param_H)
                            else:
                                writeValueArray.append(param_L)
                        else:
                            writeValueArray.append(iParamFinalValue)
                    else:
                        writeValueArray.append(iParamValue)
                print(writeValueArray)
                WriteRegEx(writeAddress,writeValueArray,sMBID)
            if sCMDValue == MotorCmdField.WMOVE.name:
                dtnow = datetime.datetime.now().timestamp()
                lock.acquire()
                try:
                    dic_485ack[int(sMBID)].append(dtnow)
                finally:
                    # Lock을 해제해서 다른 Thread도 사용할 수 있도록 만든다.
                    lock.release()                
                pub_flag.publish(f'{dtnow}{sDivFieldColon}0{sDivFieldColon}{sMBID}')
                rospy.loginfo(f'Insert Ack : {sMBID} - {dic_485ack}')
                #TODO : Polling Time Table 수정할 것.
                
    except Exception as e:
        message = traceback.format_exc()
        rospy.logdebug(message)
        #SendFeedback(e)
    return dtnow

rospy.Subscriber(TopicName.CMD_DEVICE.name, String, callbackCmd)
pub_flag = rospy.Publisher(f'{TopicName.ACK.name}', String, queue_size=10)
'''
2023-04-21
RS485 MODBUS 범용 제어 패키지 - 최병진

1. id - 프로필파일명으로 구성된 파일(ModbusConfig.txt) 로드하여 인스턴스 생성, modbustopc_{id} 형태의 퍼블리셔 생성,
1A. 파일에서 W로 시작하는 부분은 명령어 수행, R로 시작하는부분은 데이터 최대한 빨리 폴링. r로 시작하면 데이터 폴링주기 1초.
1B. 서보모터와 GPI 박스는 빠른 폴링, BMS와 NTC는 느린 폴링
2. 생성된 모드버스 객체는 dic_485ctl id - instance 형태로 저장
3. subcribe 로 명령어 수신. 명령어는 string 형태로 수신되며 map 으로 파싱, writereg 시에 반영.
4. loop 부분에서 R로 시작하는 비트만큼 읽어들인 후 (비트의 총합 / 16(워드)) modbustopc_{id} 토픽으로 발행.
'''

#getTxtPath 함수 만들자.
dirPath2 = getConfigPath()
filePath_modbusconfig = f'{dirPath2}/ModbusConfig.txt'
#print(os.path.isfile(filePath_modbusconfig) )
filePath_ModbusSetupInit = f'{dirPath2}/SERVO_Init.txt'
filePath_param_parse = f'{dirPath2}/MODBUSDATA.txt'
dicParamParse = getDic_FromFile(filePath_param_parse,sDivEmart)
'''
Setup()
1. ModbusConfig.txt 라인단위로 리딩, id - 프로필 파일명을 map 으로 만들고 퍼블리셔 객체 생성 -> dic_topic
2. Subscribe Callback 등록
3. loop
'''
dicModbus = {}
def Setup():
    global dic_485ctl
    global dic_485cmd
    global dic_485poll
    global dic_485pollLen
    global dic_topics
    global dicConfigTmp
    global dicModbus
    global dic_485ack
    global lock
    
    #global dic_485topic
    dicConfigTmp = getDic_FromFile(filePath_modbusconfig, sDivEmart)
    
    for mbid,mbName in dicConfigTmp.items():
        print(f'Init Modbus ID for : {mbid}')
        checkList = dic_485ack.get(mbid, None)
        if checkList == None:
            int485id = int(mbid)
            lock.acquire()
            try:
                if dic_485ack.get(int485id, None) == None:
                    dic_485ack[int485id] = []
            finally:
                # Lock을 해제해서 다른 Thread도 사용할 수 있도록 만든다.
                lock.release()              
        # mbName = dicConfigTmp[dic485ID]
        pub_kpalive = rospy.Publisher(f'{TopicName.MB_.name}{mbid}', String, queue_size=10)
        dic_topics[mbid] = pub_kpalive
        instrumentH : minimalmodbus.Instrument = None
        instrumentH = minimalmodbus.Instrument(port485, (int)(mbid),minimalmodbus.MODE_RTU)
        instrumentH.serial.close()
        instrumentH.serial.parity = serial.PARITY_NONE
        instrumentH.serial.stopbits = serial.STOPBITS_ONE
        instrumentH.serial.timeout = 0.5  # seconds
        if fastModbus:
            instrumentH.serial.baudrate = 115200  # Baud
        else:            
            instrumentH.serial.baudrate = 9600  # Baud
        #instrumentH.serial.baudrate = 115200  # Baud
        #instrumentH.clear_buffers_before_each_transaction = False
        instrumentH.clear_buffers_before_each_transaction = True
        filePath_modbus_cmd = f'{dirPath2}/{mbName}.txt'
        dicConfigcmd = getDic_FromFile(filePath_modbus_cmd, sDivEmart)
        dic_485ctl[mbid] = instrumentH
        dic_485cmd[mbid] = dicConfigcmd
        poll485 = []
        for cmdKey in dicConfigcmd.keys():
            #checkCmd = try_parse_int(cmdKey, -1)
            checkCmd = str(cmdKey).find('W')
            if checkCmd != 0:
                dicPollData = getDic_strArr(dicConfigcmd[cmdKey], sDivFieldColon, sDivItemComma)
                iCurrent = cmdKey.split(sep=sDivFieldColon)

                lenCurrent = 0
                for fieldTmp in dicPollData.keys():
                    lenCurrent += (int)(dicPollData[fieldTmp])                

                if len(iCurrent) > 1:
                    addrStart = iCurrent[0]
                    pollRate = iCurrent[1]
                    dicPollData[poll_START] = addrStart
                    dicPollData[poll_RATE] = pollRate
                else:
                    dicPollData[poll_START] = checkCmd
                    dicPollData[poll_RATE] = 0
                dicPollData[poll_LEN] = lenCurrent
                
                poll485.append(dicPollData)           
            #print(dic_485poll)
        dic_485poll[mbid] = poll485
        if mbName.startswith('RS'): #RS로 시작하면 서보모터여서 별도의 초기화 필요
            initMotor(mbid)
            #ICS 모터인 경우 리딩 매핑작업이 되어있는지 검사 (0f10 값이 0 이면 공장출하값)
            #공장출하상태시 WSETUP 커맨드 날림.
            if mbName.startswith('RS6') == False: 
                checkresult = instrumentH.read_registers(0x0f10,1,3)
                if len(checkresult) > 0 and checkresult[0] == 0:
                    rospy.loginfo(f'WSETUP called for {mbid},{checkresult}')
                    initMotorSetup(mbid)
            
        #resultTest = getSERVOData(instrumentV)
    #print(dic_485ctl,dic_485cmd)
    print(dicConfigcmd)
    #print(dic_485poll)

def initMotor(mbid):
    callData = String()        
    callData.data = f'{MotorWMOVEParams.MBID.name}:{mbid},{MotorWMOVEParams.CMD.name}:{MotorCmdField.WINIT.name}'
    callbackCmd(callData)
  
def initMotorSetup(mbid):
    callData = String()        
    callData.data = f'{MotorWMOVEParams.MBID.name}:{mbid},{MotorWMOVEParams.CMD.name}:{MotorCmdField.WSETUP.name}'
    callbackCmd(callData)
  
callData = String()        
#callData.data = 'MBID:31,CMD:WSTOP'
'''
rostopic pub -1 /CMD_DEVICE std_msgs/String -- 'MBID:31,CMD:WMOVE,MODE:1,POS:0,SPD:1500,ACC:10,DECC:10'
rostopic pub -1 /CMD_DEVICE std_msgs/String -- 'MBID:31,CMD:WSPD,MODE:1,POS:9,SPD:3000,ACC:1000,DECC:1000'
rostopic pub -1 /CMD_DEVICE std_msgs/String -- 'MBID:31,CMD:WSTOP'
rostopic pub -1 /CMD_DEVICE std_msgs/String -- 'MBID:31,CMD:WZERO'
rostopic pub -1 /CMD_DEVICE std_msgs/String -- 'MBID:31,CMD:WOFF'
'''
# callData.data = 'MBID:31,CMD:WINIT'
# callbackCmd(callData)

# #callData.data = 'MBID:31,CMD:WOFF'
# callData.data = 'MBID:31,CMD:WMOVE,MODE:0x41,POS:100000000,SPD:1500,ACC:10,DECC:10'
# callbackCmd(callData)

'''
서비스 클래스 정의부
'''
Setup()
class MotorCtl():
    def __init__(self):
        self.set_saveImage_service = rospy.Service('/carrier/motormove', modbus_motor, self.MotorMove)
    
    def MotorMove(self, req):
        global callData
        #global motorOpCheck
        global dic_485ack
        req_Cmd = req.cmd
        req_mbID = req.mb_id
        req_mode = req.mode
        req_pos = req.pos
        req_spd = req.spd
        req_acc = req.acc
        req_decc = req.decc
        sCmd = (f'{MotorWMOVEParams.MBID.name}{sDivFieldColon}{req_mbID}{sDivItemComma}'
                f'{MotorWMOVEParams.CMD.name}{sDivFieldColon}{req_Cmd}{sDivItemComma}'
                    f'{MotorWMOVEParams.MODE.name}{sDivFieldColon}{req_mode}{sDivItemComma}'
                        f'{MotorWMOVEParams.POS.name}{sDivFieldColon}{req_pos}{sDivItemComma}'
                        f'{MotorWMOVEParams.SPD.name}{sDivFieldColon}{req_spd}{sDivItemComma}'
                        f'{MotorWMOVEParams.ACC.name}{sDivFieldColon}{req_acc}{sDivItemComma}'
                        f'{MotorWMOVEParams.DECC.name}{sDivFieldColon}{req_decc}'
        )
        # sCmd = f'{MotorParamField.MBID.name}{sDivFieldColon}{req_mbID}{sDivItemComma}\
        #     {MotorParamField.CMD.name}{sDivFieldColon}{req_Cmd}{sDivItemComma}\
        #         {MotorParamField.MBID.name}{sDivFieldColon}{req_mbID}{sDivItemComma}\
        #             {MotorParamField.MODE.name}{sDivFieldColon}{req_mode}{sDivItemComma}\
        #                 {MotorParamField.POS.name}{sDivFieldColon}{req_pos}{sDivItemComma}\
        #                 {MotorParamField.SPD.name}{sDivFieldColon}{req_spd}{sDivItemComma}\
        #                 {MotorParamField.ACC.name}{sDivFieldColon}{req_acc}{sDivItemComma}\
        #                 {MotorParamField.DECC.name}{sDivFieldColon}{req_decc}'
        callData.data = sCmd
        respID = callbackCmd(callData)
        print(sCmd)
        #respID = int(getCurrentTime().replace(sDivFieldColon, '') )
        #respID=datetime.datetime.now().timestamp()
        #respID=datetime.datetime.now()
        resp = modbus_motorResponse(str(respID))
        #dic_485ack[req_mbID].append(respID)
        #motorOpCheck[respID] = datetime.datetime.now()
        #pub_flag.publish(f'{respID}{sDivFieldColon}0{sDivFieldColon}{req_mbID}')
        return resp

# if __name__ == '__main__':
#     rospy.init_node('spg_ImageSaver', anonymous=True, log_level=rospy.INFO)
#     try:
#         ServiceUI()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass

def print_time(chk):
    return
    print(f'{chk} - {datetime.datetime.now()}')

cntLoop = 0
exception_read_count = 0
lastUpdateTimeStamp = datetime.datetime.now()
MotorCtl()

# bChangeBaudRate = False
# iChangeBaudRate = 384
# if bChangeBaudRate:
#     instrumentTmp : minimalmodbus.Instrument= dic_485ctl[1]
#     instrumentTmp.write_register()
#     WriteReg(0x2101, iChangeBaudRate,1)
#     WriteReg(0xFE, 5,2)
lsSlowDevices = ['1','2']
while not rospy.is_shutdown():
    bReturn = True
    td = datetime.datetime.now() - lastUpdateTimeStamp
    if td.total_seconds() >= 1:
        lastUpdateTimeStamp = datetime.datetime.now()
        #rospy.loginfo(f'Loop : {cntLoop}, {dicModbus}')  
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
            infoStr = f'Addr:{addr},Value:{valueList},MB_ID:{drvID} from {caller}'
            instrumentTmp : minimalmodbus.Instrument= dic_485ctl[drvID]
            rospy.loginfo (f'Trying to write : {infoStr}')
            if len(valueList) == 1:
                instrumentTmp.write_register(addr,valueList[0])
            else:
                instrumentTmp.write_registers(addr,valueList)
            time.sleep(write_delay)
            # if len(valueList) == 1:
            #     instrumentTmp.write_register(addr,valueList[0])
            #     isFirstAccess = False
            #     time.sleep(write_delay)
            # else:
            #     if not isFirstAccess:
            #         time.sleep(0.1)
            #     instrumentTmp.write_registers(addr,valueList)
            #     time.sleep(write_delay)
            print_time(1)
    
    except Exception as e:
        if cmd_buf != None:
            CMD_Queue.appendleft(cmd_buf)
            rospy.loginfo(f'write_registers error : {cmd_buf} - {e}')
        # bReturn = False
        # rospy.loginfo(traceback.format_exc()) 
        # dtNow = datetime.datetime.now()
        # td = datetime.datetime.now() - lastLogTime
    #TODO : Polling Read Part
    try:
        print_time(2)
        for modbusID in dic_485poll.keys(): #장치ID 별로 엑세스
            return485data = {}
            cmdData = dic_485poll[modbusID]
            #print(type(cmdData))
            print_time(f'{modbusID}-0')
            for dicRecord in cmdData:
                #cmdDataDic = getDic_strArr(cmdData, sDivFieldColon, sDivItemComma)
                start_addr = 1
                if poll_START not in dicRecord.keys():
                    start_addr = 0
                start_addr = (int)(dicRecord[poll_START],16)
                len_items = (int)(dicRecord[poll_LEN]) / 16
                pollrate = (int)(dicRecord[poll_RATE])
                poll_id = f'{modbusID}_{start_addr}'
                int485id = int(modbusID)
                
                #현재 모터가 동작중 상태플래그
                bOnMoving = False
                if len(dic_485ack[int485id]) > 0:
                    bOnMoving = True
                    pollrate = 10
                    rospy.loginfo(f'Ack Check : {int485id} - {dic_485ack}')
                if pollrate > 500:
                    lastPublishedTime = dic_485pollRate.get(poll_id,DATETIME_OLD)
                    
                    #모터가 동작중일때는 저속 폴링 디바이스는 건너 뛴다
                    if bOnMoving and modbusID in lsSlowDevices:
                        continue
                    
                    #지정된 폴링주기에 이르지 못한 경우도 건너뛴다
                    if not isTimeExceeded(lastPublishedTime,pollrate):
                        continue
                    #print(dic_485ack)
                    dic_485pollRate[poll_id] = datetime.datetime.now()
                resultMulti = []
                resultList = []
                instrumentTmp : minimalmodbus.Instrument= dic_485ctl[modbusID]
                try:
                    print_time(f'{modbusID}-1')
                    resultList =instrumentTmp.read_registers(start_addr, (int)(len_items), 3)
                    for intTmp in resultList:
                        bytesTmp = intTmp.to_bytes(2,byteorder='big')
                        resultMulti.append(bytesTmp[0])
                        resultMulti.append(bytesTmp[1])
                    print_time(f'{modbusID}-2')
                    strBMSDataPartByte = bytearray(resultMulti)
                    strBMSDataPartBit = ConstBitStream(strBMSDataPartByte)
                    readBytes = 0
                    print_time(3.3)
                    for itemName, itemLenStr in dicRecord.items():
                        if itemName in poll_common:
                            continue
                        itemLen = (int)(itemLenStr)
                        readBytes += itemLen
                        signHeader = 'uint'
                        if(itemLen >= 8):
                            signHeader = 'int'
                        if strBMSDataPartBit.len >= readBytes:
                            itemHex = strBMSDataPartBit.read(f'{signHeader}:{itemLen}')
                            #keyName = f'{itemName}_{modbusID}'
                            keyName = itemName
                            if itemName in dicParamParse.keys():
                                fomula = dicParamParse[itemName]
                                fomulaFinal = f'{itemHex}{fomula}'
                                FinalValue = eval(fomulaFinal)
                                dicModbus[keyName] = return485data[keyName] = f'{FinalValue :.1f}'
                            else:
                                dicModbus[keyName] = return485data[keyName] = itemHex
                    print_time(2.4)
                    exception_read_count = 0             
                except Exception as e:
                    exception_read_count += 3
                    rospy.loginfo(f'{dicConfigTmp[modbusID]}:{poll_id}-{e} : Count:{exception_read_count}')
                    if exception_read_count >= 10:
                        #raise ModuleNotFoundError
                        exception_read_count =0
                        Setup()
                    else:
                        time.sleep(exception_delay)
                        pass
            #print(return485data)
            if len(return485data) > 0:
                #return485data[f'LASTSEEN_{modbusID}'] = datetime.datetime.now().timestamp()
                return485data[MonitoringField.LASTSEEN.name] = datetime.datetime.now().timestamp()
                return485data[MotorWMOVEParams.MBID.name] = modbusID
                
                sendbuf = getStr_fromDic(return485data, sDivFieldColon, sDivItemComma)
                dic_topics[modbusID].publish(sendbuf)
                int485id = int(modbusID)
                #motorOpCheck = dic_485ack[int485id]
                
                # for cmdID in list(motorOpCheck.keys()):
                #     timeStamp = motorOpCheck[cmdID]
                #     if isTimeExceeded(timeStamp, 100):
                motorFlag = return485data.get(f'{MonitoringField.ST_CMD_FINISH.name}', None)
                if len(dic_485ack[int485id]) > 0:
                    ts = dic_485ack[int485id][0]
                    datetimeobj = datetime.datetime.fromtimestamp(ts)
                    if isTimeExceeded(datetimeobj,500):
                        if motorFlag == 1:
                            rospy.loginfo(f'Popping Ack : {int485id} - {dic_485ack}, MotorFlag = {motorFlag}')
                            cmdID = dic_485ack[int485id].pop()
                            pub_flag.publish(f'{cmdID}{sDivFieldColon}{motorFlag}{sDivFieldColon}{modbusID}')
            time.sleep(read_delay*1)
            print_time(4)        
            '''
            rostopic echo /MB_31
            '''
        print_time(5)
    except Exception as e:
        bReturn = False
        rospy.loginfo(traceback.format_exc())
        rospy.signal_shutdown(e)
        #sCmd = '/root/.rrStart -&'
        #os.system(sCmd)
    #rate.sleep()

