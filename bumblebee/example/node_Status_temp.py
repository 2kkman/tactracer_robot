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
import pandas as pd

# 로봇의 부품과 상태 정보를 담은 딕셔너리 생성

# # pandas DataFrame 생성
# robot_parts_df = pd.DataFrame(data)

# # 생성된 DataFrame 출력
# print(robot_parts_df)

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
        if k == None or v == None:
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
    pubTopic = TopicName.BLB_STATUS_MONITOR.name
    pub = rospy.Publisher(pubTopic, String, queue_size=10)
    rate = rospy.Rate(0.5)  # send 1 time per second
    while not rospy.is_shutdown():
        data = {
            "rid": "blb_tester",
            "title": "범블비 주요부품 알람정보",
            "data": [
                {
                    "부품명": "주행모터",
                    "알람코드": "0",
                    "알람명": "없음",
                    "상태": "정지",
                    "카테고리": "서보",
                },
                {
                    "부품명": "RFID_L",
                    "알람코드": "-1",
                    "알람명": "Timeout",
                    "상태": "인식불가",
                    "카테고리": "센서",
                },
                {
                    "부품명": "카메라",
                    "알람코드": "0",
                    "알람명": "없음",
                    "상태": "정상",
                    "카테고리": "카메라",
                },
                {
                    "부품명": "라이다",
                    "알람코드": "0",
                    "알람명": "없음",
                    "상태": "정상",
                    "카테고리": "센서",
                },
            ],
            "lastseen": getCurrentTime("", True),
        }
        sendbuf = json.dumps(data)
        pub.publish(sendbuf)
        print(sendbuf)
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
