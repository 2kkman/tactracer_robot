#! /usr/bin/env python3
import os
import rosnode
import rospy
from varname import *
import time
from std_msgs.msg import String
from std_msgs.msg import Header
import minimalmodbus as minimalmodbus
import serial
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import statistics
from Util import *
from UtilBLB import *

# /opt/ros/noetic/share/std_srvs/srv/Empty.srv , SetBool.srv, Trigger.srv
from std_srvs.srv import *

dirPath = os.path.dirname(__file__)
filePath_param_parse = f"{dirPath}/param_SD.txt"
dicParamParse = getDic_FromFile(filePath_param_parse, sDivEmart)

lastupdate = getDateTime()
port485Header = "/dev/ttC"
# sDivField = sDivFieldColon
# sDivItem = sDivEmart
listTmp = [
    "USET",
    "ISET",
    "UOUT",
    "IOUT",
    "POWER",
    "UIN",
    "LOCK",
    "PROTECT",
    "CVCC",
    "ONOFF",
    "B_LED",
    "MODEL",
    "VERSION",
]
listSD = [ "V24","V48"]
# listSD = ['V16','V24','V12']
lenthList = len(listTmp)
listVCC = []
dicSD = {}
alarmLimit = 10
alarmCnt = 0
dic_485 = {}

param_SD_show = True
param_SD_Rate = 1
param_SD_Cmd = ""

publish_topic_name = TopicName.SD.name
nodeName = f"node_{publish_topic_name}"
# publish_topic_name = 'GOAL' #테스트용 변수

lastUpdateAlarm = getDateTime()
alarm_interval = 5000
lastActionTime = getDateTime()

pub = None
rate = None
isOK = True


def InitSDRetry():
    global alarmCnt
    alarmCnt = 0
    for sdType in listSD:
        bOK = False
        sdPort = f"{port485Header}{sdType}"
        sTopicName = f'{TopicName.SD.name}{sdType}'
        while bOK == False:
            try:
                rospy.loginfo(f"Starting SD port... {sdPort}")
                instrumentSD = minimalmodbus.Instrument(
                    sdPort, 1, minimalmodbus.MODE_RTU
                )
                if instrumentSD != None:
                    instrumentSD.serial.close()
                instrumentSD.serial.baudrate = 9600
                instrumentSD.serial.bytesize = 8
                instrumentSD.serial.parity = serial.PARITY_NONE
                instrumentSD.serial.stopbits = 1
                instrumentSD.serial.timeout = 5  # seconds

                # instrumentSD.clear_buffers_before_each_transaction = True
                dicSD[sdType] = instrumentSD
                dic_485[sdType] = {}
                bOK = True
            except Exception as e:
                print(f"Failed to Open port {sdPort} - {e}")
                time.sleep(1)
            rospy.loginfo(f"SD port result : {bOK}")
    global pub
    global rate
    rospy.init_node(
        nodeName, anonymous=False, log_level=rospy.DEBUG
    )  # 485 이름의 노드 생성
    pub = rospy.Publisher(publish_topic_name, String, queue_size=1)
    rate = rospy.Rate(param_SD_Rate)


InitSDRetry()


def callbackCmd(data):
    print(type(data))
    global param_SD_Cmd
    param_SD_show = False
    retryLimit = 5
    retryCnt = 0
    while retryCnt < retryLimit:
        try:
            param_SD_Cmd = str(data.data).upper()
            recvData = param_SD_Cmd
            rospy.logdebug(f"Recv data ({publish_topic_name}) : {recvData}")
            retryCnt = retryLimit
        except Exception as e:
            retryCnt += 1
            message = traceback.format_exc()
            rospy.loginfo(f"Try count : {retryCnt}, {message}")
    param_SD_show = True


rospy.Subscriber(TopicName.CMD_DEVICE.name, String, callbackCmd)


def GetServiceName(strParam):
    host = GetUbutuParam(UbuntuEnv.CONFIG.name)
    srv_name = f"/{host}/{strParam}"
    return srv_name


class DataRecorder:
    def __init__(self):
        self.V36_ENABLE_service = rospy.Service(
            ServiceBLB.SDV48.value, SetBool, self.SDV36_SET
        )
        self.V24_ENABLE_service = rospy.Service(
            ServiceBLB.SDV24.value, SetBool, self.SDV24_SET
        )
        # self.V5_ENABLE_service = rospy.Service(ServiceList.SDV5.value, SetBool, self.SDV5_SET)
        # self.V5_LOCK_service = rospy.Service(ServiceList.SDV5LOCK.value, SetBool, self.SDV5LOCK_SET)
        self.V36_LOCK_service = rospy.Service(
            ServiceBLB.SDV48LOCK.value, SetBool, self.SDV36LOCK_SET
        )
        self.V24_LOCK_service = rospy.Service(
            ServiceBLB.SDV24LOCK.value, SetBool, self.SDV24LOCK_SET
        )
        rospy.loginfo(
            "BLB StepDown Control Started : " + GetUbutuParam(UbuntuEnv.CONFIG.name)
        )

    def WriteRegSvr(self, portName, addr, writeValue):
        global dicSD
        instrumentTmp = dicSD[portName]
        instrumentTmp.write_register(addr, writeValue, 0, 6)

    def SDV36_SET(self, req):
        portName = "V36"
        msgTmp = String()
        reqbool = req.data
        enableSD = 0
        if reqbool:
            enableSD = 1
        msgTmp.data = f"SD/ONOFF:{portName}:{enableSD}"
        callbackCmd(msgTmp)
        return SetBoolResponse(isOK, f"SD{portName}:{reqbool}")

    def SDV36LOCK_SET(self, req):
        portName = "V36"
        msgTmp = String()
        reqbool = req.data
        enableSD = 0
        if reqbool:
            enableSD = 1
        msgTmp.data = f"SD/LOCK:{portName}:{enableSD}"
        callbackCmd(msgTmp)
        return SetBoolResponse(isOK, f"SD{portName}:{reqbool}")

    def SDV24_SET(self, req):
        portName = "V24"
        msgTmp = String()
        reqbool = req.data
        enableSD = 0
        if reqbool:
            enableSD = 1
        msgTmp.data = f"SD/ONOFF:{portName}:{enableSD}"
        callbackCmd(msgTmp)
        return SetBoolResponse(isOK, f"SD{portName}:{reqbool}")

    def SDV24LOCK_SET(self, req):
        portName = "V24"
        msgTmp = String()
        reqbool = req.data
        enableSD = 0
        if reqbool:
            enableSD = 1
        msgTmp.data = f"SD/LOCK:{portName}:{enableSD}"
        callbackCmd(msgTmp)
        return SetBoolResponse(isOK, f"SD{portName}:{reqbool}")

    def SDV5_SET(self, req):
        portName = "V5"
        msgTmp = String()
        reqbool = req.data
        enableSD = 0
        if reqbool:
            enableSD = 1
        msgTmp.data = f"SD/ONOFF:{portName}:{enableSD}"
        callbackCmd(msgTmp)
        return SetBoolResponse(isOK, f"SD{portName}:{reqbool}")

    def SDV5LOCK_SET(self, req):
        portName = "V5"
        msgTmp = String()
        reqbool = req.data
        enableSD = 0
        if reqbool:
            enableSD = 1
        msgTmp.data = f"SD/LOCK:{portName}:{enableSD}"
        callbackCmd(msgTmp)
        return SetBoolResponse(isOK, f"SD{portName}:{reqbool}")


DataRecorder()

while not rospy.is_shutdown():
    try:
        strResult = ""
        listVCCTmp = []
        if param_SD_Cmd != "":
            try:
                strArray = param_SD_Cmd.split(sDivSlash)
                """ SD/ONOFF:V24:0 """
                if len(strArray) == 2 and strArray[0] == publish_topic_name:
                    # recvDataMap = getDic_strArr(strArray[1], StrParser.sDivColon.value, StrParser.sDivEmart.value)
                    cmdArray = strArray[1].split(sDivFieldColon)
                    sItemName = cmdArray[0]
                    portName = cmdArray[1]
                    writeValue = int(cmdArray[2])
                    instrumentTmp = dicSD[portName]
                    addr = listTmp.index(sItemName)
                    instrumentTmp.write_register(addr, writeValue, 0, 6)
                    rospy.loginfo(f"CMD OK : {param_SD_Cmd}")
                    time.sleep(0.1)
                else:
                    rospy.loginfo(f"CMD Ignored : {param_SD_Cmd}")
                param_SD_Cmd = ""
            except Exception as e:
                print(f"Failed to Write CMD : {param_SD_Cmd}, {e}")

        for k, v in dicSD.items():
            drvID_current = k
            resultMulti = dicSD[drvID_current].read_registers(0, 28, 3)

            for i in range(len(listTmp)):
                if str(type(listTmp[i])) != "<class 'int'>":
                    fieldName = listTmp[i]
                    resultCurrent = resultMulti[i]
                    if fieldName in dicParamParse:
                        beforeParse = resultCurrent
                        fomula = dicParamParse[fieldName]
                        fomulaFinal = f"{beforeParse}{fomula}"
                        FinalValue = eval(fomulaFinal)
                        resultCurrent = FinalValue
                    dic_485[drvID_current][fieldName] = resultCurrent

            curVcc = getValueFromMap(dic_485[drvID_current], "UIN")
            if curVcc != None:
                listVCCTmp.append(int(curVcc))
            strResult += (
                getStr_fromDic(
                    dic_485[drvID_current],
                    sDivFieldColon,
                    sDivItemComma,
                    True,
                    f"_{drvID_current}",
                )
                + sDivItemComma
            )
        # sMsg = f'{strResult}SD_ID{StrParser.sDivColon.value}{drvID_current}'

        listVCC.append(statistics.mean(listVCCTmp))
        chargeStatus = 0
        lengthVCC = len(listVCC)
        charging_margin = 1
        if lengthVCC > 10:
            # sv = 5 if lengthVCC >= 5 else lengthVCC
            newVCC = listVCC[7:]
            listVCCMean = statistics.mean(listVCC)
            newVCCMean = statistics.mean(newVCC)
            if newVCCMean - listVCCMean > charging_margin:  # 충전중
                chargeStatus = 1
            elif listVCCMean - newVCCMean > charging_margin:  # 방전중
                chargeStatus = 2
            del listVCC[0]

        sMsg = strResult[:-1]
        # sMsg = f'{strResult}CHARGE_STATE{sDivField}{chargeStatus}'
        pub.publish(sMsg)
        if param_SD_show:
            rospy.logdebug(sMsg)
        isOK = True
        rate.sleep()
    except Exception as e:
        isOK = False
        alarmCnt += 1
        message = traceback.format_exc()
        rospy.loginfo(message)
        if alarmLimit < alarmCnt:
            rospy.logfatal(f"{nodeName} has alarms. Restart.")
            InitSDRetry()
            # rospy.signal_shutdown(f'{nodeName} has alarms. Restart.')
            # os.system('~/.sdstart')
