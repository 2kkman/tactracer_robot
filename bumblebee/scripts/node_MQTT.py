#! /usr/bin/env python3

# 표준 라이브러리 임포트
import copy
import gc
import json
import os
import random
import struct
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from curses import ascii, keyname
from signal import alarm
from uuid import getnode

# 서드 파티 라이브러리 임포트
import minimalmodbus
#import numpy as np
import pandas as pd
import paho.mqtt.client as mqtt
import rospy
from sensor_msgs.msg import Imu, LaserScan, RelativeHumidity, Temperature
from std_msgs.msg import String
import tf

# 로컬 애플리케이션/라이브러리 특정 모듈
from std_srvs.srv import *
from tta_blb.srv import *
from turtlesim.srv import *
from Util import *
from UtilBLB import *

"""
2023-02-13
지시정보 체계 변경 -> 엔코더랑 매핑하기.
1. 연사테스트
2. 합성테스트
3. 
"""
msg_count = 0
# print(device_ID)
nodeName = f"node_MQTT"
rospy.init_node(nodeName, anonymous=False)  # 485 이름의 노드 생성
pub_BLB_CMD = rospy.Publisher(TopicName.BLB_CMD.name, String, queue_size=1)
pub_mqtt2topic = rospy.Publisher(TopicName.RECEIVE_MQTT.name, String, queue_size=1)
pub_IMU = rospy.Publisher(TopicName.IMU.name, Imu, queue_size=10)
pub_resonant = rospy.Publisher(TopicName.RESONANT_TRAY.name, String, queue_size=1)
pub_topic2mqtt = rospy.Publisher(TopicName.SEND_MQTT.name, String, queue_size=1)
runFromLaunch = rospy.get_param(f"~{ROS_PARAMS.startReal.name}", default=True)
# dicPublishTopic = {}
# dicPublishTopic[TopicName.RECEIVE_MQTT.name] = pub_mqtt2topic
# dicPublishTopic[TopicName.IMU.name] = pub_IMU
# dicPublishTopic[TopicName.BLB_CMD.name] = pub_BLB_CMD

mqtt.Client.connected_flag = False  # create flag in class
seq = 0
acceleration_buffer = []


def publish_ImuData(dicARD: dict):
    """_summary_ : dic 개체를 받아 자이로센서 정보를 파싱하여 pub_IMU 인스턴스를 통해
    ros1 IMU 메세지를 발행한다.

    Args:
        dicARD (dict): IMU 메세지가 포함된 아두이노 스트링
    """
    Imu_msg = Imu()
    frame_id_Range = "map"
    Imu_msg.header = getROS_Header(frame_id_Range)
    GAV_SVN = dicARD.get(TRAY_ARD_Field.GAV_SVN.name, None)
    GLA_SVN = dicARD.get(TRAY_ARD_Field.GLA_SVN.name, None)
    GOR_SVN = dicARD.get(TRAY_ARD_Field.GOR_SVN.name, None)
    if GAV_SVN == None or GLA_SVN == None or GOR_SVN == None:
        return

    GAV_SVNarr = GAV_SVN.split(sDivItemComma)  # angular_velocity
    GLA_SVNarr = GLA_SVN.split(sDivItemComma)  # linear_acceleration
    GOR_SVNarr = GOR_SVN.split(sDivItemComma)  # orientation , roll,pitch,yaw

    roll = float(GOR_SVNarr[0])
    pitch = float(GOR_SVNarr[1])
    yaw = float(GOR_SVNarr[2])
    IMU_roll = roll
    IMU_pitch = pitch
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    # 쿼터니언 설정
    Imu_msg.orientation.x = quaternion[0]
    Imu_msg.orientation.y = quaternion[1]
    Imu_msg.orientation.z = quaternion[2]
    Imu_msg.orientation.w = quaternion[3]

    Imu_msg.linear_acceleration.x = float(GLA_SVNarr[0])
    Imu_msg.linear_acceleration.y = float(GLA_SVNarr[1])
    Imu_msg.linear_acceleration.z = float(GLA_SVNarr[2])

    Imu_msg.angular_velocity.x = float(GAV_SVNarr[0])
    Imu_msg.angular_velocity.y = float(GAV_SVNarr[1])
    Imu_msg.angular_velocity.z = float(GAV_SVNarr[2])
    seq += 1
    Imu_msg.header.stamp = rospy.Time.now()
    Imu_msg.header.seq = seq

    pub_IMU.publish(Imu_msg)
    acceleration_buffer.append(Imu_msg)


def on_connect(client2, userdata, flags, reason_code, properties):
    print(userdata, flags, reason_code, properties)
    logmsg = f"{sys._getframe(0).f_code.co_name}-Params:{client2},{userdata},{flags},{reason_code},{properties}"
    rospy.loginfo(logmsg)
    if reason_code.is_failure:
        rospy.loginfo(
            f"Failed to connect: {reason_code}. loop_forever() will retry connection"
        )
    else:
        client2.connected_flag = True  # set flag
        rospy.loginfo("connected OK")
        client2.subscribe("BLB/#")


def on_disconnect(client2, userdata, flags, reason_code,properties):
    logmsg = (
        f"{sys._getframe(0).f_code.co_name}-Params:{client2},userdata:{userdata},flags:{flags},reason_code:{reason_code},properties:{properties}"
    )
    rospy.loginfo(logmsg)
    global connected_flag
    connected_flag = False  # set flag
    print("disconnected OK")


def on_message(client2, userdata, msg):
    global pub_mqtt2topic
    try:
        sTOPIC_MQTT = msg.topic
        sPAYLOAD = msg.payload.decode("utf-8")
        rospy.loginfo(
            f"{sys._getframe(0).f_code.co_name}-{sTOPIC_MQTT},{userdata},{sPAYLOAD}"
        )
        # rospy.loginfo(payload)
        # MQTT 토픽별 처리루틴
        if sTOPIC_MQTT.startswith(MQTT_TOPIC_VALUE.IMU.value):
            # 아두이노에서 WIFI 로 MQTT IMU 메세지를 발행하면 ROS1 표준 IMU 메세지로 바꿔 발행한다
            recvDataIMU = getDic_strArr(sPAYLOAD, sDivFieldColon, sDivEmart)
            publish_ImuData(recvDataIMU)
            return
        elif sTOPIC_MQTT == MQTT_TOPIC_VALUE.BLB_CALL.value:
            # 호출벨 웹페이지에서 테이블 번호만 달랑 수신한 경우 해당 노드로 이동시킨다.
            nodeID = sPAYLOAD
            data_out = {}
            data_out[BLB_CMD.ID.name] = device_ID
            data_out[BLB_CMD.LEVEL.name] = -3
            node_num = GetKoreanFromNumber(nodeID)
            if if_Number(nodeID):
                node_int = int(nodeID)
                # # TTS 옵션이 on 이 되어있고 노드번호가 90번 이내인 경우 TTS로 안내방송
                # # (즉 1~89는 일반 테이블, 이후는 충전소나 부엌등 특수 지점)
                # # 호출벨을 누른 경우
                # if runFromLaunch and node_int < 90:
                #     SendMsgToMQTT(
                #         pub_topic2mqtt,
                #         MQTT_TOPIC_VALUE.TTS.value,
                #         f"테이블 {node_num}번, {node_num}번 테이블에서 호출합니다.",
                #     )
                # # 부엌, 충전소에서 호출한 경우
                # if runFromLaunch and node_int > 90:
                #     SendMsgToMQTT(
                #         pub_topic2mqtt,
                #         MQTT_TOPIC_VALUE.TTS.value,
                #         f"범블비를 {node_num}에서 호출하였습니다.",
                #     )

                # TRAY_A (첫번째로 방문할 테이블 번호) - 에 nodeID 세팅
                data_out[BLB_CMD.TRAY_A.name] = nodeID
                # 이동명령어 세팅
                data_out[BLB_CMD.STATE.name] = BLB_CMD_STATUS.MOVE.name
            else:
                # 테이블 번호가 아닌 다른 데이터
                data_out[BLB_CMD.STATE.name] = nodeID  # 도어 닫힘 확인버튼을 누른 경우
                data_out[BLB_CMD.TRAY_A.name] = 0

            data_out[BLB_CMD.TRAY_B.name] = 0
            data_out[BLB_CMD.TIME.name] = getCurrentTime(spliter="")
            sendbuf = json.dumps(data_out)
            pub_BLB_CMD.publish(sendbuf)
            return
        elif is_json(sPAYLOAD):
            # 그외의 경우에
            data_out = json.loads(sPAYLOAD)
            data_out[CALLBELL_FIELD.TIMESTAMP.name] = getCurrentTime(spliter="")
            nodeID = sTOPIC_MQTT.split("_")[-1]
            if sTOPIC_MQTT.find(CALLBELL_FIELD.CALLBELL.name) > 0:
                #'BLB/mcu_relay_100' : 교차로에서 발행하는 메세지
                #'BLB/mcu_relay_CALLBELL_1' : 호출벨에서 온 메세지인 경우 토픽명에 CALLBELL 이란 단어가 있음.
                if data_out.get(CALLBELL_FIELD.BTN_RED.value, None) == 1:
                    # SendMsgToMQTT(
                    #     MQTT_TOPIC_VALUE.TTS.value,
                    #     f"{GetKoreanFromNumber(nodeID)}번 테이블에서 직원을 호출하셨습니다.",
                    # )
                    return
                    # SendMsgToMQTT(mqtt_topic_TTS,f'{nodeID}번 테이블에서 직원을 호출하셨습니다.')
                elif data_out.get(CALLBELL_FIELD.BTN_BLUE.value, None) == 1:
                    if data_out.get(CALLBELL_FIELD.CALL_STATE.value, None) == 1:
                        # SendMsgToMQTT(
                        #     MQTT_TOPIC_VALUE.TTS.value,
                        #     f"테이블 {GetKoreanFromNumber(nodeID)}번에서 로봇 호출을 취소하셨습니다.",
                        #)
                        return
            else:
                recvPayload = {}
                recvPayload[MQTT_FIELD.TOPIC.name] = sTOPIC_MQTT
                recvPayload[MQTT_FIELD.PAYLOAD.name] = sPAYLOAD
                data_out = json.dumps(recvPayload)
                pub_mqtt2topic.publish(data_out)
    except Exception as e:
        rospy.loginfo(traceback.format_exc())


def publish(msg, topic):
    global msg_count
    # msg = f"messages: {msg_count}"
    result = client.publish(topic, msg)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        rospy.loginfo(f"Send `{msg}` to topic `{topic}`")
    else:
        rospy.loginfo(f"Failed to send message to topic {topic}")
    msg_count += 1


rate = rospy.Rate(1)

# 모스키토 broker 정보 #1
broker_address = "iot.tactracer.com"
# broker_address = 'www.i9man.com'
broker_port = 1883
clientID = get_hostname()
# clientID = f"publish-{random.randint(0, 1000)}"
topic_to_MQTT = f"BLB/{clientID}/"
# client = mqtt.Client(client_id=clientID, clean_session=False, userdata=None, protocol=mqtt.MQTTv311,transport="tcp")
# version = '1'
# mytransport = 'websockets' # or 'tcp'
# if version == '5':
#     client = mqtt.Client(client_id=clientID,
#                          transport=mytransport,
#                          protocol=mqtt.MQTTv5)
# if version == '3':
#     client = mqtt.Client(client_id=clientID,
#                          transport=mytransport,
#                          protocol=mqtt.MQTTv311,
#                          clean_session=True)
# client = mqtt.Client(clientID)
client = mqtt.Client(
    mqtt.CallbackAPIVersion.VERSION2,
    client_id=clientID,
    clean_session=False,
    userdata=GetMachineStr(),
)
username = rospy.get_param("~username", default="ttracer")
password = rospy.get_param("~password", default="tt2015")
client.username_pw_set(username=username, password=password)
client.on_connect = on_connect  # bind call back function
client.on_disconnect = on_disconnect
# client.subscribe(topic_to_MQTT,qos=0)
client.on_message = on_message

def startToConnect():
  try:
    rospy.loginfo(f"Connecting to broker : {broker_address}:{broker_port}")
    client.connect(broker_address, broker_port)  # connect to broker
    client.loop_start()
    while not client.connected_flag:  # wait in loop
        rospy.loginfo(f"In wait loop for MQTT SVR info {broker_address}:{broker_port}")
        time.sleep(1)
  except Exception as e:
      message = traceback.format_exc()
      rospy.loginfo(message)
      
# client.loop_forever()


def callbackTopic2mqtt(data):
    # TODO : 응답메세지 발행하는 것도 만들기.
    try:
        recvData = data.data
        if is_json(recvData):
            recvDataMap = json.loads(recvData)
        else:
            recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        # logmsg = f'{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}'
        logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name}"

        sDIVSTR_FIELD = recvDataMap.get(MQTT_FIELD.DIVSTR_FIELD.name, sDivFieldColon)
        sDIVSTR_ITEM = recvDataMap.get(MQTT_FIELD.DIVSTR_ITEM.name, sDivItemComma)
        sTOPIC_MQTT = recvDataMap.get(MQTT_FIELD.TOPIC.name, None)
        sPAYLOAD = recvDataMap.get(MQTT_FIELD.PAYLOAD.name, None)
        # recvPayload = getDic_strArr(sPAYLOAD, sDIVSTR_FIELD, sDIVSTR_ITEM)
        # if sPAYLOAD == None or sTOPIC_MQTT == None or recvPayload==None:
        if sPAYLOAD == None or sTOPIC_MQTT == None:
            return

        # data_out=json.dumps(recvDataMap)
        publish(sPAYLOAD, sTOPIC_MQTT)
        rospy.loginfo(logmsg)
    except Exception as e:
        rospy.loginfo(traceback.format_exc())


rospy.Subscriber(TopicName.SEND_MQTT.name, String, callbackTopic2mqtt)
startToConnect()

while not rospy.is_shutdown():
    try:
        dtNow = getDateTime()
        print(f"{dtNow}:MQTT is alive")
        if msg_count > 5:
          msg_count = 0
          startToConnect()
          
    except Exception as e:
        bReturn = False
        rospy.loginfo(traceback.format_exc())
        rospy.signal_shutdown(e)
        # sCmd = '/root/.rrStart -&'
        # os.system(sCmd)
    rate.sleep()
