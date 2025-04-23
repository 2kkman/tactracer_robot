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
#import rospy
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
import subprocess
import rosparam
import roslib
from collections import deque
#from ServoELD2 import *

'''
구분자 정의 : 
1차 구분자 - 이마트 어퍼스트로피 `
2차구분자 - 콜론 :
3차구분자 - 쉼표 (레지스터 단위 구분)
4차구분자 - 슬래쉬 : 레지스터 연속값 구분
'''
# sDivFieldColon = StrParser.sDivColon2.value
# sDivItemComma = StrParser.sDivComma3.value
# sDivEmart =StrParser.sDivEmart1.value
# sDivSlash =StrParser.sDivSlash.value
# sDivSemiCol = StrParser.sDivSemiColon.value
CMD_Queue = deque() #모드버스 통신 동기화를 위한 Write 버퍼
publish_topic_goal  : str = 'CMD' #테스트용 변수
publish_topic_name = 'MB'
nodeName = f'node_{publish_topic_name}'
#rospy.init_node(nodeName,anonymous=False) # 485 이름의 노드 생성
write_delay = 0.01
param_DRV_Rate = 100
port485 = '/dev/ttyS0' # 시리얼 포트
#port485 = '/dev/ttyUSB0' # USB 포트
dirPath = os.path.dirname(__file__)
dic_485ctl = {}
dic_485cmd = {}
dic_485poll = {}
dic_485pollLen = {}
dic_topics = {}
#dic_485topic = {}
#rate = rospy.Rate(param_DRV_Rate)

#BMS 
lastLogTime = getDateTime()

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
        print(traceback.format_exc())
        #rospy.signal_shutdown(e)    
    return bPass

def WriteReg(addr, value,drvID):
    listTmp = []
    listTmp.append(value)
    return WriteRegEx(addr,listTmp,drvID)

def callbackCmd(data):
    #TODO : 응답메세지 발행하는 것도 만들기.
    try:
        success = False
        recvData = data.data
        logmsg = f'{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}'
        print(logmsg)
        last_dish_washed = ''
        #rate = rospy.Rate(1)
        recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        print(recvDataMap)
        sMBID =recvDataMap.get(BUM_Keys.MBID.name, '') 
        sCMDValue = recvDataMap.get(BUM_Keys.CMD.name, None)#등록된 모드버스ID 인 경우!
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
    except Exception as e:
        message = traceback.format_exc()
        #rospy.logdebug(message)
        #SendFeedback(e)

#rospy.Subscriber(publish_topic_goal, String, callbackCmd)

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

filePath_modbusconfig = f'{dirPath}/ModbusConfig.txt'
filePath_ModbusSetupInit = f'{dirPath}/SERVO_Init.txt'

'''
Setup()
1. ModbusConfig.txt 라인단위로 리딩, id - 프로필 파일명을 map 으로 만들고 퍼블리셔 객체 생성 -> dic_topic
2. Subscribe Callback 등록
3. loop
'''
def Setup():
    global dic_485ctl
    global dic_485cmd
    global dic_485poll
    global dic_485pollLen
    global dic_topics
    dicConfigTmp = {}
    #global dic_485topic
    dicConfigTmp = getDic_FromFile(filePath_modbusconfig, sDivEmart)
    for dic485ID in dicConfigTmp.keys():
        #print(dic485ID)
        # pub_kpalive = rospy.Publisher(f'{publish_topic_name}_{dic485ID}', String, queue_size=10)
        # dic_topics[dic485ID] = pub_kpalive
        instrumentH : minimalmodbus.Instrument = None
        instrumentH = minimalmodbus.Instrument(port485, (int)(dic485ID),minimalmodbus.MODE_RTU)
        instrumentH.serial.close()
        instrumentH.serial.parity = serial.PARITY_NONE
        instrumentH.serial.stopbits = 2
        instrumentH.serial.timeout = 0.5  # seconds
        instrumentH.serial.baudrate = 9600  # Baud
        instrumentH.clear_buffers_before_each_transaction = False
        filePath_modbus_cmd = f'{dirPath}/{dicConfigTmp[dic485ID]}.txt'
        dicConfigcmd = getDic_FromFile(filePath_modbus_cmd, sDivEmart)
        dic_485ctl[dic485ID] = instrumentH
        dic_485cmd[dic485ID] = dicConfigcmd
        poll485 = []
        for cmdKey in dicConfigcmd.keys():
            checkCmd = try_parse_int(cmdKey, -1)
            if checkCmd != -1:
                dicPollData = getDic_strArr(dicConfigcmd[cmdKey], sDivFieldColon, sDivItemComma)
                lenCurrent = 0
                for fieldTmp in dicPollData.keys():
                    lenCurrent += (int)(dicPollData[fieldTmp])                
                dicPollData['START'] = checkCmd
                dicPollData['LEN'] = lenCurrent
                poll485.append(dicPollData)                
            #print(dic_485poll)
        dic_485poll[dic485ID] = poll485
    
        #resultTest = getSERVOData(instrumentV)
    print(dic_485ctl,dic_485cmd)
    #print(dic_485poll)
        
Setup()
callData = String()        
#callData.data = 'MBID:31,CMD:WSTOP'
'''
rostopic pub -1 /CMD std_msgs/String -- 'MBID:31,CMD:WMOVE,MODE:1,POS:0,SPD:1500,ACC:10,DECC:10'
rostopic pub -1 /CMD std_msgs/String -- 'MBID:31,CMD:WSTOP'
rostopic pub -1 /CMD std_msgs/String -- 'MBID:31,CMD:WZERO'
rostopic pub -1 /CMD std_msgs/String -- 'MBID:31,CMD:WOFF'
rostopic pub -1 /CMD std_msgs/String -- 'MBID:31,CMD:WALM_C'
rostopic pub -1 /CMD std_msgs/String -- 'MBID:31,CMD:WINIT'
'''
callData.data = 'MBID:31,CMD:WSTOP'

callbackCmd(callData)

# callData.data = 'MBID:31,CMD:WOFF'
# callData.data = 'MBID:31,CMD:WMOVE,MODE:0x41,POS:100000000,SPD:1500,ACC:10,DECC:10'
# callbackCmd(callData)


while not rospy.is_shutdown():
    bReturn = True
    try:
        while len(CMD_Queue) > 0:
            cmd_buf = list(CMD_Queue.popleft())
            addr = cmd_buf[0]
            valueList = cmd_buf[1]
            drvID = cmd_buf[2]
            caller = cmd_buf[3]
            infoStr = f'{addr}:{valueList}:{drvID} from {caller}'
            instrumentTmp : minimalmodbus.Instrument= dic_485ctl[drvID]
            print (f'Trying to write : {infoStr}')
            cmd_buf = None
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
            
        dtNow = getDateTime()
        td = getDateTime() - lastLogTime
        
        for modbusID in dic_485poll.keys(): #장치ID 별로 엑세스
            return485data = {}
            cmdData = dic_485poll[modbusID]
            #print(type(cmdData))
            for dicRecord in cmdData:
                #cmdDataDic = getDic_strArr(cmdData, sDivFieldColon, sDivItemComma)
                start_addr = (int)(dicRecord['START'])
                len_items = (int)(dicRecord['LEN']) / 16
                
                instrumentTmp : minimalmodbus.Instrument= dic_485ctl[modbusID]
                resultList =instrumentTmp.read_registers(start_addr, (int)(len_items), 3)
                resultMulti = []
                for intTmp in resultList:
                    bytesTmp = intTmp.to_bytes(2,byteorder='big')
                    resultMulti.append(bytesTmp[0])
                    resultMulti.append(bytesTmp[1])
                    
                strBMSDataPartByte = bytearray(resultMulti)
                strBMSDataPartBit = ConstBitStream(strBMSDataPartByte)
                readBytes = 0
                for itemName, itemLenStr in dicRecord.items():
                    itemLen = (int)(itemLenStr)
                    readBytes += itemLen
                    signHeader = 'uint'
                    if(itemLen >= 8):
                        signHeader = 'int'
                    if strBMSDataPartBit.len >= readBytes:
                        itemHex = strBMSDataPartBit.read(f'{signHeader}:{itemLen}')
                        return485data[itemName] = itemHex
            #print(return485data)
            sendbuf = getStr_fromDic(return485data, sDivFieldColon, sDivItemComma)
            dic_topics[modbusID].publish(sendbuf)
                
            '''
            rostopic echo /MB_31
            '''
            
                    
                    
        # if td.total_seconds()  > 1: #BMS, NTP 등 느린주기 장치 엑세스
        #     lastLogTime = getDateTime()
        #     continue
        # dic_485cmd
        # if CheckServoStatus() == False:
        #     continue
        
    except Exception as e:
        bReturn = False
        print(traceback.format_exc())
        rospy.signal_shutdown(e)
        #sCmd = '/root/.rrStart -&'
        #os.system(sCmd)
    time.sleep(1)
    #rate.sleep()

