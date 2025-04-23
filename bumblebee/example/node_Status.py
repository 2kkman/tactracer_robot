#!/usr/bin/env python3
import copy
import math
import rospy
import roslib
from varname import *
from std_msgs.msg import Header
from std_msgs.msg import String
import argparse, socket, time, json, datetime, platform, psutil, requests, pprint, uuid, sys
from Util import *
from UtilBLB import *
from dataclasses import dataclass
import threading

lock = threading.Lock()
lastLogTime = getDateTime()


@dataclass
class MonitorData:
    datastr: str
    lastseen: datetime


dictMonitorData: Dict[str, MonitorData] = dict()
runFromLaunch = False
dictMonitorTopics = {
    "MB_15": [MonitoringField.ALM_CD.name, MonitoringField.ST_CMD_FINISH.name],
    "MB_29": [MonitoringField.ALM_CD.name, MonitoringField.ST_CMD_FINISH.name],
}
# print(MQTT_TOPIC_TEST.BLB_CALL.description)


def prtMsg(sendbuf):
    if runFromLaunch:
        rospy.loginfo(sendbuf)
    else:
        print(sendbuf)


# print(removeDictFromList("name", "아메리카노", items))


def GenerateReadableDicMotor(dictTmp):
    returnDic = {}
    # MBID 가 포함되어 있는 경우는 모터상태
    # dictTmp = {'MBID': '15', 'ALM_CD': '0', 'ST_CMD_FINISH': '1'}
    # TODO : MBID 에 대해 발행해야 할 토픽명을 가져오는 함수 필요
    # MBID 별로 토픽 인스턴스를 dict 형태로 저장해야 함
    # 1. "항목"
    for k, v in dictTmp.items():
        if k == None   or v == None:
            continue
        v_int = int(v)
        value, description_value_zero, description_value_nonZero = (
            MonitoringField_EX.GetEnumValue(k)
        )
        if value == None or description_value_zero == None:
            continue
        statusStr = ""
        if v_int == 1:
            statusStr = description_value_nonZero
        elif v_int == 0:
            statusStr = description_value_zero
        else:
            statusStr = value

        returnDic[value] = statusStr
    return returnDic


def talker():
    global dictMonitorTopics
    global dictMonitorData
    pubTopic = f'{TopicName.BLB_STATUS_MONITOR.name}_TEST'
    pub = rospy.Publisher(pubTopic, String, queue_size=10)
    rate = rospy.Rate(1)  # send 1 time per second
    recvDataMap = {}
    returnDic = {}
    returnRefined = {}
    while not rospy.is_shutdown():
        for mbid, lsMonitorItem in dictMonitorTopics.items():
            # print(returnDic)
            curItem = dictMonitorData.get(mbid, None)
            if curItem is None:
                continue
            recvData = curItem.datastr
            if is_json(recvData):
                recvDataMap = json.loads(recvData)
            else:
                recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
            for fieldTmp in lsMonitorItem:
                fieldValue = recvDataMap.get(fieldTmp, None)
                if fieldValue is None:
                    continue
                returnDic[fieldTmp] = fieldValue
            returnDic[MotorWMOVEParams.MBID.name] = GetLastString(mbid, "_")

        #         print()

        # returnDic["항목"] = ModbusID_EX.MOTOR_H.caption
        # returnDic["현재상태"] = "정상"
        # returnDic["알람명"] = "정상"
        # returnDic["카테고리"] = "모터"
        # returnDic["TEST"] = "TESTCODE"
        # returnDic["TEST2"] = "TESTCODE2"
        returnls = []
        # sendbuf = json.dumps(returnDic)
        # pub.publish(sendbuf)

        if len(returnDic) == 0:
            continue
        returnls.append(GenerateReadableDicMotor(returnDic))
        # returnls.append(returnDic)
        if len(returnls) > 0 and pub is not None:
            sendbuf = json.dumps(returnls)
            pub.publish(sendbuf)
            # prtMsg(sendbuf)

        rate.sleep()


def callbackCmd(data, topicName):
    global dictMonitorData
    global lock
    curtmp = MonitorData(datastr=data.data, lastseen=getDateTime())
    lock.acquire()
    try:
        dictMonitorData[topicName] = curtmp
    finally:
        # Lock을 해제해서 다른 Thread도 사용할 수 있도록 만든다.
        lock.release()


if __name__ == "__main__":
    try:
        rospy.init_node(f"node_Status", anonymous=True)
        idxCur = 1
        for topicname, listFields in dictMonitorTopics.items():
            rospy.Subscriber(topicname, String, callbackCmd, callback_args=topicname)
            print(f"Subscribe - {idxCur}:{topicname}-{listFields}")
        # rospy.Subscriber(TopicName.TABLE_ORDER_STATUS.name, String, callbackCmd2)
        # runFromLaunch = rospy.get_param("~startReal", default=False)
        # print(f"runFromLaunch : {runFromLaunch}")
        talker()
    except rospy.ROSInterruptException:
        pass
