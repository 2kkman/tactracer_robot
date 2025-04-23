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
lock_MQTT = threading.Lock()
lastLogTime = getDateTime()


@dataclass
class MonitorData:
    datastr: str
    lastseen: datetime


dictMonitorData: Dict[str, MonitorData] = dict()
dictDataMQTT: Dict[str, str] = dict()
# rid = "blb_tester"
rid = device_ID
runFromLaunch = False
dictMonitorTopics = {
    "MB_26": [MonitoringField.ALM_CD.name, MonitoringField.ST_CMD_FINISH.name],
    # TopicName.RECEIVE_MQTT.name: ["ID", MonitoringField_EX.STATE.name],
    TopicName.ARUCO_RESULT.name: [ARUCO_RESULT_FIELD.CAM_ID.name],
    TopicName.RECEIVE_MQTT.name: ["ID", "STATE"],
    TopicName.RFID.name: [
        RFID_RESULT.DEVID.name,
        RFID_RESULT.PWR.name,
        RFID_RESULT.EPC.name,
    ],
}
lsInitField = ["검사명", "알람코드", "알람명", "카테고리"]
print(MonitoringField_EX.STATE.name)

print(dictMonitorTopics)
# print(MQTT_TOPIC_TEST.BLB_CALL.description)

rospy.init_node(f"node_Status", anonymous=True)
pubTopic = f"{TopicName.BLB_STATUS_MONITOR.name}_TEST"
pubTopicBMS = f"{TopicName.BLB_STATUS_MONITOR_BMS_NTP.name}"
pub = rospy.Publisher(pubTopic, String, queue_size=10)
pubBMS = rospy.Publisher(pubTopicBMS, String, queue_size=10)

idxCur = 1
