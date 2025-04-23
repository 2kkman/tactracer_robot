#!/usr/bin/env python3
import rospy
from typing import *

# /opt/ros/noetic/share/std_srvs/srv/Empty.srv , SetBool.srv, Trigger.srv
from std_srvs.srv import *

# /opt/ros/noetic/share/rospy_tutorials/srv/AddTwoInts.srv
# input : a,b (정수) , return sum (정수)
from rospy_tutorials.srv import *
import json
import copy

# /opt/ros/noetic/share/turtlesim/srv/Kill.srv
# input : string, return : 없음
from turtlesim.srv import *
from tta_blb.srv import *

# import srvSingleParam, srvSingleParamResponse
import serial
import os
import subprocess
from varname import *
import termios, sys
import time
from Util import *
from UtilBLB import *

# from SPG_Keys import *
# from ServoNano import *
from varname import *
from std_msgs.msg import Header
from std_msgs.msg import String
from typing import List
from dataclasses import dataclass, field
from collections.abc import Sequence
import copy
import threading
import tf

# from imusensor.MPU9250 import MPU9250
from sensor_msgs.msg import Imu, Temperature, RelativeHumidity

from std_msgs.msg import Header
from varname import *

IMU_roll = 0
IMU_pitch = 0

frame_id_Range = "map"
topic_name = ""
param_IMU_show = True
param_ARD_show = False

enableDummyArduino = True

lock = threading.Lock()
lastUpdateTimeStamp = getDateTime()
lastCmdTimeStamp = getDateTime()
cntLoop = 0
cmdIdx = 0
dirPrev = True
dicServiceTimeStamp = {}

node_CHARGING_STATION = 99
node_KITCHEN_STATION = 98
node_NOT_TABLE = 0

node_current = node_CHARGING_STATION
node_target = 0
node_direction = True
# dirPath = os.path.dirname(__file__)
dirPath = getConfigPath(UbuntuEnv.ITX.name)
strFilePath = f"{dirPath}/CROSS.txt"

topic_sub_topic2mqtt = TopicName.SEND_MQTT.name
topic_pub_mqtt2topic = TopicName.RECEIVE_MQTT.name
topic_pub_topiclist = TopicName.TOPIC_LIST.name
mqtt_topic_call = "BLB/CALL"

# NODEMCU 에 자이로를 장착하여 아래 노드에 MQTT 로 보내면 ROS1 IMU 포맷으로 변환하여 퍼블리시 한다
mqtt_topic_IMU = "BLB/IMU"

mqtt_topic_TTS = "/home/member"
stateDic = {}
dicARD_CARRIER = {}

StateInfo: Dict[str, list] = {}
StateSet: Dict[int, int] = {}
nodeStateOpen = [0, 1]  # 인덱스가 0,1 일때는 Open 상태, 2,3 일땐 Close

file_list = getLines_FromFile(strFilePath)

portArd_M = "/dev/ttCARD_M"  # 아두이노 시리얼 포트
# portArd_M = '/dev/ttyACM0' # 아두이노 메가 정품 시리얼 포트
serArd_M = None
listPortArd = [portArd_M]
listArdInstance = [serArd_M]
baudArd = 115200  # 아두이노 통신속도
lineArdData = []
dic_485ex = {}  # 모터 모니터링 데이터
activated_motors = []  # 현재 모니터링 중인 모터
listBLB = []  # 경로 지시정보
listTable = []  # 순차서빙 테이블 리스트

flag_liftdown = False
flag_liftup = False
is_lifted = False
is_docked = False
flag_WaitConfirm = False

runFromLaunch = rospy.get_param(f"~{ROS_PARAMS.startReal.name}", default=False)
pub_ka = rospy.Publisher(TopicName.KEEPALIVE.name, String, queue_size=1)
pub_IMU = rospy.Publisher(TopicName.IMU.name, Imu, queue_size=10)
pub_cmdDevice = rospy.Publisher(TopicName.CMD_DEVICE.name, String, queue_size=1)  #
seq = 0
node_name = "node_CtlCenter"

waitCross = False
curStartState = None
curTargetState = None
nStart = ""
nStartStatus = ""
nTarget = ""
nEndStatus = ""
nEncoder = 0
nDirection = ""
lsTopicList = []
curBLB_Status = BLB_STATUS_FIELD.CHARGING
curOP_Status = BLB_OP_FIELD.MANUAL

flag_req_doorOpen = False
flag_req_doorClose = False

timestamp_touchinit = getDateTime()
bInit = False
pub_status = rospy.Publisher(
    TopicName.BLB_STATUS.name, String, queue_size=1
)  # UI로 명령어 처리결과 및 상태값 송신
pub_topic2mqtt = rospy.Publisher(topic_sub_topic2mqtt, String, queue_size=1)
pub_topiclist = rospy.Publisher(topic_pub_topiclist, String, queue_size=1)

# 전체 지도 맵을 읽어들인다.
""" 예제
99 98 2200
98 1 1800
1 2 4200
2 3 2400
3 4 5000
4 5 5000
"""
strFilePathShortCut = f"{dirPath}/SHORTCUT.txt"
graph = LoadGraph(strFilePathShortCut)


def prtMsg(sendbuf):
    if runFromLaunch:
        rospy.loginfo(sendbuf)
    else:
        print(sendbuf)


def SendKeepAlive(sendbuf):
    global pub_ka
    if pub_ka is not None:
        pub_ka.publish(sendbuf)
        prtMsg(sendbuf)


def SendStatus(blb_status: BLB_STATUS_FIELD):
    global pub_status
    dicStatusData = {}
    dicStatusData[BLB_STATUS.ID.name] = deviceID
    dicStatusData[BLB_STATUS.STATUS.name] = blb_status.name
    dicStatusData[BLB_STATUS.NODE_CURRENT.name] = node_current
    dicStatusData[BLB_STATUS.NODE_TARGET.name] = node_target
    sendbuf = json.dumps(dicStatusData)
    if pub_status is not None:
        pub_status.publish(sendbuf)
        prtMsg(sendbuf)


def movePrepareBLB(nodeFrom: int, nodeTo: int, isForward: bool, inputIdx: int):
    """1개의 노드를 이동한다. 노드 이동 전 분기점에 문제 없는지 검사한다.
    연결되어있는지 여부는 검사하지 않는다. (호출전 별도 검사)
    1. nodeFrom 을 기반으로

    Args:
        nodeFrom (int): 현재 노드
        nodeTo (int): 이동할 노드
        isForward (bool): 진행방향이 Forward 인지 여부,
    """
    """
  0. 노드정보 확인. StateInfo - dict 개체 활용
  1. 만일 둘다 테이블이고 서로 연결되어있다면 문제가 없다.
  2. 둘중 하나 이상이 분기기라면 연결상태로 만듬.
  
  현재출발지가 테이블이면 그냥 가면 됨.
  분기기나 엘베인 경우
  1. 출발지 노드의 진입점 배열에 출발지 ID가 있는지 확인
  2. 당연히 있을거고, (없다면 익셉션) - 상태값이 0인지 1인지 확인 후 SetState 로 출발 노드 세팅
  """
    """ 
  둘이 연결되어있다고 가정.

  """
    scmFrom = StateInfo.get(nodeFrom, None)
    scmTo = StateInfo.get(nodeTo, None)

    scmList = [nodeFrom, nodeTo]  # 시작노드와 목적지 노드
    nodeReadyList = [0, 0]
    nodeSetResult = [False, False]
    nodeSetResult2 = [-1, -1]
    iCnt = 0
    directionCheckCnt = 0
    for scmCur in scmList:
        scmCurValue = StateInfo.get(scmCur, None)
        if (
            scmCurValue == None
        ):  # 테이블이나 분기기, 엑세스 도어처럼 특수 노드가 아닌 경우는 그냥 출발.
            nodeSetResult[iCnt] = True
            nodeReadyList[iCnt] = -1
        else:
            setStateKey = scmCur
            if iCnt == 1:
                setStateKey = scmList[0]
            else:
                setStateKey = scmList[1]

            if setStateKey in scmCurValue:
                # nodeReadyList[iCnt] = (int)(scmCurValue.index(setStateKey)/2)
                nodeReadyList[iCnt] = scmCurValue.index(setStateKey)
            if setStateKey < 0:
                nodeSetResult[iCnt] = False
                # 익셉션 발생시켜야 함.
            else:
                bIsOpen = nodeReadyList[iCnt] in nodeStateOpen
                nodeSetResult[iCnt] = setStateBranch(scmCur, bIsOpen)
                nodeSetResult2[iCnt] = 1 if bIsOpen else 0
        # directionCheckCnt += nodeReadyList[iCnt]
        iCnt += 1
    result = False in nodeSetResult

    # 진입방향과 출구방향이 같으면 역방향, 그 외는 정방향으로 나가야 됨
    if -1 not in nodeReadyList and nodeReadyList[0] % 2 == inputIdx % 2:
        isForward = not isForward

    if result:
        print("ERROR")
    else:
        # print(f"경로설정 성공! 이동합니다.{scmList},포워딩:{isForward} : 진출 {nodeReadyList[0] } ,진입 {nodeReadyList[1] }")
        print(
            f"경로설정 성공! 이동합니다.{scmList},포워딩:{isForward} : 진출 {nodeReadyList[0] } ,진입 {nodeReadyList[1] }"
        )
    # return not result,nodeReadyList[1]
    return not result, nodeSetResult2[0], nodeSetResult2[1], isForward

    """
  iPos = 0 일때 Open, iPos = 1일때 Close 상태
  bStraightPath = False 라는건 백워드로 가야한다는 뜻.
  """


def getSeqMap(startNode: int, endNode: int) -> list:
    """노드의 시작지점과 끝 지점을 받아 제어시퀀스를 리턴한다

    Args:
        startNode (int): 시작노드
        endNode (int): 도착노드

    Returns:
        _type_: 딕셔너리의 배열
    """
    curStart = startNode
    curEnd = endNode
    lastNode = curStart
    lastInputIdx = -1

    lsPath, pathCost = getPathBLB(graph, curStart, curEnd)

    print(f"경로정보:{lsPath}, 구간별거리:{pathCost}")
    moveForward = True
    numberOfPath = len(lsPath)
    # indx = 0
    # while indx < range(numberOfPath):
    #   curTarget = lsPath[indx]
    #   resultMove, inputIDX = movePrepareBLB(curStart, curTarget, moveForward,lastInputIdx)

    listSeqMapOrg = []
    listSeqMapOptimized = []

    for idx in range(numberOfPath):
        curTarget = lsPath[idx]
        if lastNode == curTarget:
            moveForward = not moveForward
        # scmCurValue = StateInfo.get(scmCur, None)
        # if scmCurValue == None: #테이블이나 분기기, 엑세스 도어처럼 특수 노드가 아닌 경우는 그냥 출발.
        #   nodeSetResult[iCnt] = True
        #   nodeReadyList[iCnt] = -1
        resultMove, nodeSetResult_IN, nodeSetResult_OUT, isForward = movePrepareBLB(
            curStart, curTarget, moveForward, lastInputIdx
        )
        # print(f'{curStart} -> {curTarget} : {resultMove}, 진입인덱스 : {inputIDX}' )
        print(
            f"{resultMove}, 출발지&제어:{curStart}/{nodeSetResult_IN}, 도착지제어:{curTarget}/{nodeSetResult_OUT}, 방향:{isForward}, 엔코더:{pathCost[idx]}"
        )
        if resultMove:
            dicMap = {}
            dicMap[SeqMapField.START_NODE.name] = curStart
            dicMap[SeqMapField.START_STATUS.name] = nodeSetResult_IN
            dicMap[SeqMapField.END_NODE.name] = curTarget
            dicMap[SeqMapField.END_STATUS.name] = nodeSetResult_OUT
            dicMap[SeqMapField.DIRECTION.name] = isForward
            dicMap[SeqMapField.DISTANCE.name] = pathCost[idx]
            listSeqMapOrg.append(dicMap)
            lastNode = curStart  # 이전 노드를 기억해둔다
            curStart = curTarget  # 이동 완료한 노드가 새 출발점이 된다
        else:
            print("ERROR")
            break

    listSeqMapOrg_B = copy.deepcopy(listSeqMapOrg)

    dicFirst = listSeqMapOrg[0]
    newStart = dicFirst[SeqMapField.START_NODE.name]
    newStartStatus = dicFirst[SeqMapField.START_STATUS.name]
    newTarget = dicFirst[SeqMapField.END_NODE.name]
    newEndStatus = dicFirst[SeqMapField.END_STATUS.name]
    newEncoder = 0
    newDirection = dicFirst[SeqMapField.DIRECTION.name]

    while len(listSeqMapOrg) > 0:
        dicMap_0 = listSeqMapOrg.pop(0)
        if (
            dicMap_0[SeqMapField.END_STATUS.name] == -1
        ):  # 논스톱으로 지나야 하는 구간인 경우
            newEncoder = dicMap_0[SeqMapField.DISTANCE.name]
            # newEncoder += dicMap_0[SeqMapField.DISTANCE.name]
            newTarget = dicMap_0[SeqMapField.END_NODE.name]
            newEndStatus = dicMap_0[SeqMapField.END_STATUS.name]
            if len(listSeqMapOrg) == 0:
                dicMap_new = {}
                dicMap_new[SeqMapField.START_NODE.name] = newStart
                dicMap_new[SeqMapField.START_STATUS.name] = newStartStatus
                dicMap_new[SeqMapField.END_NODE.name] = newTarget
                dicMap_new[SeqMapField.END_STATUS.name] = newEndStatus
                dicMap_new[SeqMapField.DISTANCE.name] = newEncoder
                # dicMap_new[SeqMapField.DIRECTION.name] = newDirection
                listSeqMapOptimized.append(dicMap_new)
        else:
            nStart = dicMap_0[SeqMapField.START_NODE.name]
            nStartStatus = dicMap_0[SeqMapField.START_STATUS.name]
            nTarget = dicMap_0[SeqMapField.END_NODE.name]
            nEndStatus = dicMap_0[SeqMapField.END_STATUS.name]
            nEncoder = dicMap_0[SeqMapField.DISTANCE.name]
            nDirection = dicMap_0[SeqMapField.DIRECTION.name]

            dicMap_new = {}
            dicMap_new[SeqMapField.START_NODE.name] = newStart
            dicMap_new[SeqMapField.START_STATUS.name] = newStartStatus
            dicMap_new[SeqMapField.END_NODE.name] = nTarget
            dicMap_new[SeqMapField.END_STATUS.name] = nEndStatus
            dicMap_new[SeqMapField.DISTANCE.name] = newEncoder + nEncoder
            dicMap_new[SeqMapField.DISTANCE.name] = newEncoder
            dicMap_new[SeqMapField.DIRECTION.name] = newDirection
            listSeqMapOptimized.append(dicMap_new)

            if len(listSeqMapOrg) > 0:
                dicFirst = listSeqMapOrg[0]
                newStart = dicFirst[SeqMapField.START_NODE.name]
                newStartStatus = dicFirst[SeqMapField.START_STATUS.name]
                newTarget = dicFirst[SeqMapField.END_NODE.name]
                newEndStatus = dicFirst[SeqMapField.END_STATUS.name]
                newEncoder = 0
                newDirection = dicFirst[SeqMapField.DIRECTION.name]

    print(f"원래경로:{listSeqMapOrg_B}")
    print("END")
    return listSeqMapOptimized


def IsDoorMoving():
    if enableDummyArduino:
        return False

    global dicARD_CARRIER
    doorStatusClose = dicARD_CARRIER.get(CARRIER_STATUS.I_DOOR_1_BOTTOM.name, None)
    doorStatusOpen = dicARD_CARRIER.get(CARRIER_STATUS.I_DOOR_1_TOP.name, None)
    doorStatusHome = dicARD_CARRIER.get(CARRIER_STATUS.I_DOOR_1_HOME.name, None)
    if isTrue(doorStatusClose) or isTrue(doorStatusOpen) or isTrue(doorStatusHome):
        return False
    else:
        return True


def SendMsgToMQTT(topicTmp, payloadTmp):
    global stateDic
    global pub_topic2mqtt
    global StateInfo
    dicSendMqttTopic = {}
    dicSendMqttTopic[MQTT_FIELD.TOPIC.name] = topicTmp
    dicSendMqttTopic[MQTT_FIELD.PAYLOAD.name] = payloadTmp
    data_out = json.dumps(dicSendMqttTopic)
    logmsg = f"Send {data_out} msg from {sys._getframe(0).f_code.co_name}-{sys._getframe(1).f_code.co_name}"
    rospy.loginfo(logmsg)
    pub_topic2mqtt.publish(data_out)


def GetLiftControlUp():
    return GetLiftControl(True)


def GetLiftControlDown():
    return GetLiftControl(False)


def GetLiftControl(isUp: bool):
    # global flag_liftup
    listBLBTmp = []
    global is_docked
    global is_lifted
    strLiftFilePath = ""
    if isUp:
        strLiftFilePath = f"{dirPath}/LIFTUP.txt"
        if is_docked:
            return
    else:
        strLiftFilePath = f"{dirPath}/LIFTDOWN.txt"
        if is_lifted:
            return

    with open(strLiftFilePath, "r") as f:
        list_ex_load = json.load(f)
        print(list_ex_load)
        for lsTmp in list_ex_load:
            listBLBTmp.append(lsTmp)
    return listBLBTmp


def getNodeState(nodeID):
    global stateDic
    global StateInfo
    nodeIDStr = str(nodeID)
    scmCurValue = StateInfo.get(int(nodeIDStr), None)
    if (
        scmCurValue == None
    ):  # 테이블이나 분기기, 엑세스 도어처럼 특수 노드가 아닌 경우는 -1 리턴
        return -1
    nodeInfoDic = stateDic.get(nodeIDStr, None)
    if nodeInfoDic == None:
        return -2  # 정보가 들어온 것이 없음. MQTT + ROS 통신에 문제가 있음

    nodeState = nodeInfoDic.get("STATE", None)
    if nodeState == None:
        return (
            -3
        )  # 데이터는 들어왔지만 상태값을 나타내는 필드가 없음. 프로그램 로직 문제.
    return nodeState


def setNodeStateEx(nodeID, statusVal):
    global StateSet
    StateSet[int(nodeID)] = int(statusVal)


def setNodeState(nodeID, statusVal):
    global stateDic
    global pub_topic2mqtt
    global StateInfo
    nodeIDStr = str(nodeID)
    scmCurValue = StateInfo.get(int(nodeIDStr), None)
    if (
        scmCurValue == None
    ):  # 테이블이나 분기기, 엑세스 도어처럼 특수 노드가 아닌 경우는 -1 리턴
        return

    if statusVal == 0 or statusVal == 1:
        mqttTopic = f"BLB/CROSS_CMD/set"
        # mqttTopic = f'BLB/mcu_relay_{nodeID}/set'
        SendMsgToMQTT(mqttTopic, f"E:{statusVal}")
        # #mqttTopic = f'BLB/mcu_relay_CMD/set'
        # dicSendMqttTopic = {}
        # dicSendMqttTopic[MQTT_FIELD.TOPIC.name] = mqttTopic
        # dicSendMqttTopic[MQTT_FIELD.PAYLOAD.name] = f'E:{statusVal}'
        # #dicSendMqttTopic[MQTT_FIELD.PAYLOAD.name] = f'E:{statusVal},ID:{nodeID}'
        # data_out=json.dumps(dicSendMqttTopic)
        # pub_topic2mqtt.publish(data_out)
        time.sleep(0.1)


def service_setbool_client(serviceName, enable, serviceType):
    global dicServiceTimeStamp
    bResult = None
    if isServiceExist(serviceName) == False:
        rospy.loginfo(f"Service not found : {serviceName}")
        return False
    # rospy.wait_for_service(serviceName, 2)
    dtNow = getDateTime()
    serviceNameID = None
    if enable == None:
        serviceNameID = f"{serviceNameID}"
    else:
        serviceNameID = f"{serviceNameID}{enable}"
    serviceLastTimeStamp = dicServiceTimeStamp.get(serviceNameID)
    dicServiceTimeStamp[serviceNameID] = dtNow
    if serviceLastTimeStamp != None and not isTimeExceeded(serviceLastTimeStamp, 1000):
        return False
    try:
        setbool_proxy = rospy.ServiceProxy(serviceName, serviceType)
        sos = None
        # print(type(serviceType))
        if enable == None:
            if serviceType == Trigger:
                sos = TriggerRequest()
            else:
                sos = EmptyRequest()
            bResult = setbool_proxy(sos)
            return bResult

        responseResult = setbool_proxy(enable)
        # print(responseResult)
        rospy.loginfo(
            f"Service({serviceType}) called : {serviceName} - {enable} : from {sys._getframe(1).f_code.co_name}-{sys._getframe(2).f_code.co_name}"
        )
        return responseResult
    except Exception as e:
        rospy.loginfo(e)
        return False


def RFIDControl(enable):
    rfid_B = service_setbool_client(ServiceBLB.NEST_RFID_INV_B.value, enable, SetBool)
    rfid_F = service_setbool_client(ServiceBLB.NEST_RFID_INV_F.value, enable, SetBool)
    print(rfid_B, rfid_F)
    return rfid_B


def SendCMDArd(enable):
    return service_setbool_client(ServiceBLB.CMDARD_QBI.value, enable, Kill)


def DoorStop():
    SendCMDArd(f"O:0,10")


def TrayStop():
    SendCMDArd(f"Y:0,10")


def DoorOpen(spd=10):
    SendCMDArd(f"O:2{sDivItemComma}{spd}")


def DoorClose(spd=10):
    SendCMDArd(f"O:1{sDivItemComma}{spd}")


def TrayClose(spd=10):
    SendCMDArd(f"Y:1{sDivItemComma}{spd}")


def TrayOpen(spd=10):
    SendCMDArd(f"Y:2{sDivItemComma}{spd}")


def callbackAck(data):
    global activated_motors
    global node_target
    global node_current
    # TODO : 응답메세지 발행하는 것도 만들기.
    try:
        recvData = data.data
        logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name} : {activated_motors}"
        lsResult = recvData.split(
            sDivFieldColon
        )  # idx 0 - timestamp, 1 - 완료여부 , 2 - MBID

        mbid = lsResult[2]
        flag = lsResult[1]
        ts = float(lsResult[0])
        datetimeobj = datetime.fromtimestamp(ts)
        dtnow = getDateTime()
        finishTime = dtnow - datetimeobj

        # 이 부분에서 현재 어떠한 상태인지 알 수 있음. (하강인지 상승인지 등)
        if flag == "0":
            activated_motors.append(mbid)
        else:
            activated_motors.remove(mbid)
            if mbid == (str)(ModbusID.MOTOR_H.value) and node_target != 0:
                node_current = node_target
                node_target = 0
            rospy.loginfo(f"Finish : {finishTime}, curr_node:{node_current}")
        # rospy.loginfo(logmsg)
        # last_dish_washed = ''
        # #rate = rospy.Rate(1)
        # recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        # print(recvDataMap)
    except Exception as e:
        message = traceback.format_exc()
        rospy.logdebug(message)
        # SendFeedback(e)


def callbackBLB_CMD(data):  # UI에서 날라오는 명령어를 모니터링
    global listBLB
    global cmdIdx
    global listTable
    global flag_WaitConfirm
    global curBLB_Status
    global lock
    global bInit
    global timestamp_touchinit
    try:
        recvDataMap = {}
        recvData = (str)(data.data)
        logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
        rospy.loginfo(logmsg)
        if is_json(recvData):
            recvDataMap = json.loads(recvData)
        else:
            recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivEmart)
        rospy.loginfo(recvDataMap)
        STATE = recvDataMap.get(BLB_CMD.STATE.name, "")
        LEVEL = recvDataMap.get(BLB_CMD.LEVEL.name, "")
        if STATE.find(BLB_CMD_STATUS.INIT.name) >= 0:
            # SendStatus(BLB_STATUS_FIELD.INIT)
            timestamp_touchinit = getDateTime()
            bInit = True
            return
        elif STATE.find(BLB_CMD_STATUS.EVENT.name) >= 0:
            strSpd = ",0"
            print(logmsg)
            if (
                LEVEL.find(BLB_UI_EVENTS.up_released.name) >= 0
                or LEVEL.find(BLB_UI_EVENTS.down_released.name) >= 0
            ):
                TrayStop()
            elif LEVEL.find(BLB_UI_EVENTS.down_pressed.name) >= 0:
                TrayClose()
            elif LEVEL.find(BLB_UI_EVENTS.up_pressed.name) >= 0:
                TrayOpen()
            elif LEVEL.find(BLB_UI_EVENTS.level_reset.name) >= 0:
                SendCMDArd("Y:3" + strSpd)

            if (
                LEVEL.find(BLB_UI_EVENTS.A_released.name) >= 0
                or LEVEL.find(BLB_UI_EVENTS.B_released.name) >= 0
            ):
                DoorStop()
            elif LEVEL.find(BLB_UI_EVENTS.A_pressed.name) >= 0:
                DoorClose()
            elif LEVEL.find(BLB_UI_EVENTS.B_pressed.name) >= 0:
                DoorOpen()
            # if LEVEL.find(BLB_UI_EVENTS.down_pressed.name) >= 0:
            #   SendCMDArd('Y:2')
            return

        # rate = rospy.Rate(1)
        if CheckAllKeysExist(BLB_CMD, recvDataMap):
            ID = auto()  # Device ID
            TRAY_A = (int)(recvDataMap[BLB_CMD.TRAY_A.name])
            TRAY_B = (int)(recvDataMap[BLB_CMD.TRAY_B.name])
            LEVEL = (int)(recvDataMap[BLB_CMD.LEVEL.name])
            TIME = (str)(recvDataMap[BLB_CMD.TIME.name])
            STATE = (str)(recvDataMap[BLB_CMD.STATE.name])
            print(
                f"From {node_current} to {TRAY_A}, Cur_listTable:{listTable},cmdIdx:{cmdIdx}"
            )
            if STATE.find(BLB_CMD_STATUS.CONFIRM.name) >= 0:
                flag_WaitConfirm = False
            elif STATE.find(BLB_CMD_STATUS.MOVE.name) >= 0:
                # flag_WaitConfirm = False
                lock.acquire()
                if not TRAY_A in listTable and not TRAY_B in listTable:
                    if len(listTable) > 0 and listTable[-1] == node_CHARGING_STATION:
                        listTable.pop()
                    if TRAY_A != node_current and TRAY_A != node_NOT_TABLE:
                        listTable.append(TRAY_A)
                    if TRAY_A != TRAY_B and TRAY_B != node_NOT_TABLE:
                        listTable.append(TRAY_B)
                    # Move 명령인데 기존 리스트 테이블이 있는 경우 충전소로 가라고 안내
                    if len(listTable) > 0:
                        listTable.append(node_CHARGING_STATION)
                    if node_current == node_KITCHEN_STATION:
                        flag_WaitConfirm = False
                lock.release()

                print(listTable)
            # 명령어에 따라 시퀀수맵 생성할 것.
        mbid = recvDataMap.get(MotorWMOVEParams.MBID.name, None)

        if mbid != None:
            dic_485ex[mbid] = recvDataMap
        # print(recvDataMap)
        # TODO : 응답메세지 발행하는 것도 만들기.

    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        # SendFeedback(e)


def callbackModbus(data):
    # TODO : 응답메세지 발행하는 것도 만들기.
    try:
        recvData = data.data
        logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
        # rospy.loginfo(logmsg)
        last_dish_washed = ""
        # rate = rospy.Rate(1)
        recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        mbid = recvDataMap.get(MotorWMOVEParams.MBID.name, None)
        if mbid != None:
            dic_485ex[mbid] = recvDataMap

        # print(recvDataMap)
    except Exception as e:
        message = traceback.format_exc()
        rospy.logdebug(message)
        # SendFeedback(e)


def callbackARD_CARRIER(data):
    # TODO : 응답메세지 발행하는 것도 만들기.
    try:
        recvData = data.data
        logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
        recvDataMap = json.loads(recvData)
        dicARD_CARRIER.update(recvDataMap)
        # rospy.loginfo(recvDataMap)
    except Exception as e:
        message = traceback.format_exc()
        rospy.logdebug(message)
        # SendFeedback(e)


def getLoadWeight():
    global dicARD_CARRIER
    strWeightValue = dicARD_CARRIER.get(CARRIER_STATUS.LOAD1.name, "-1")
    return strWeightValue


def callbackRFID(data):
    """
    주행모드 사전 설정되어있어야 함. global 변수명 기재.
    1. 수동운전 - 자동멈춤
    2. 수동운전 - 엔코더 변경
    3. 자동운전 - 엔코더 변경
    * 교차로 태그
    *
    """
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
        sEPCKey = "EPC"
        sEPC = getValueFromMap(recvDataMap, sEPCKey)
        print(sEPC)
        return
        # if sEPC != None and sEPC != lastEPC:
        if sEPC != None:
            curLoc = GetCurLocH()
            # if sEPC == epcNot:
            #     tmpData = getStr_FromFile(filePath_CMD_RECONNECT)
            #     df_fromSPGMAP.join(str2frame(tmpData,'\t'))
            #
            # el
            # if lastEPC != sEPC:
            if True:
                differenceEPC = 0
                chkStatusH, chkStatusV = getMultiEx(dic_485, "TARG")
                savedEPC = dicRFID.get(sEPC, "")
                # if lastEPC != sEPC and chkStatusH:
                #     rospy.loginfo(f'RFID Recv : {recvData},{sEPC} at Pos : {curLoc}')
                if mapping:
                    dicRFID[curLoc] = sEPC
                    if lastEPC != sEPC:
                        rospy.loginfo(f"RFID Mapping : {sEPC} at {curLoc}")

                # if lastEPC != sEPC and is_digit(curLoc) and chkStatusH != strREADY:
                if chkStatusH != strREADY:  # 수평모터 동작중
                    mapSPD = try_parse_int(dicSPDMAP.get(sEPC, ""))
                    if LastActionCmd == dirCaption_Backward:  # 복귀방향
                        if mapSPD > 0:
                            if not SpdDown:
                                rospy.loginfo(
                                    f"Slow Tag Detected from Mapping : {sEPC} spd : {mapSPD} at {curLoc}"
                                )
                                # ChargeNestEnable(False)
                                motorMove(0, mapSPD, dirCaption_R, False, None, None)
                                SpdDown = True
                        else:
                            if not SpdBoost:
                                rospy.loginfo(
                                    f"Fastest Tag Detected from Mapping : {sEPC} at {curLoc}"
                                )
                                motorMove(0, 100, dirCaption_R, False, None, None)
                                SpdBoost = True

                if (
                    lastEPC != sEPC
                    and is_digit(curLoc)
                    and chkStatusH != strREADY
                    and savedEPC != ""
                ):
                    # rospy.loginfo(f'RFID Detected : {sEPC} - {curLoc}')
                    # differenceEPC = abs(savedEPC) -
                    curLocABS = abs(int(curLoc))
                    curLocEPC = abs(int(savedEPC))
                    differenceEPC = abs(curLocEPC - curLocABS)
                    differenceMM = differenceEPC / param_HEncoderPulse
                    rospy.loginfo(
                        f"RFID Recv with {LastActionCmd} : {recvData},{sEPC} at Pos : {curLoc} : saved {savedEPC}, difference : {differenceEPC}({differenceMM : 0.2f} mm)"
                    )
                elif savedEPC != "" and lastEPC != sEPC:
                    rospy.loginfo(
                        f"Invalid RFID  : {sEPC} with savedEPC : {savedEPC}, curLoc : {curLoc}"
                    )
                lastEPC = sEPC
                if AutoStop is True and sEPC is not None:
                    rospy.loginfo(f"Stop by {sEPC}")
                    motorStop(drvCaption_H, True)
            # SendFeedback(recvData)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        # print (e)
        # SendFeedback(e)


rospy.Subscriber(
    f"{TopicName.MB_.name}{ModbusID.BAL_ARM1.value}", String, callbackModbus
)
rospy.Subscriber(
    f"{TopicName.MB_.name}{ModbusID.TELE_SERV_MAIN.value}", String, callbackModbus
)
rospy.Subscriber(
    f"{TopicName.MB_.name}{ModbusID.MOTOR_H.value}", String, callbackModbus
)
rospy.Subscriber(
    f"{TopicName.MB_.name}{ModbusID.TELE_SERV_INNER.value}", String, callbackModbus
)
rospy.Subscriber(
    f"{TopicName.MB_.name}{ModbusID.ROTATE_MAIN_540.value}", String, callbackModbus
)
rospy.Subscriber(TopicName.RFID.name, String, callbackRFID)
rospy.Subscriber(TopicName.ARD_CARRIER.name, String, callbackARD_CARRIER)
rospy.Subscriber(TopicName.ACK.name, String, callbackAck)
rospy.Subscriber(TopicName.BLB_CMD.name, String, callbackBLB_CMD)  # UI에서 명령어 수신


def gotoNode(deviceID, nodeID, bTTS):
    data_out = {}
    data_out[BLB_CMD.ID.name] = deviceID
    data_out[BLB_CMD.LEVEL.name] = -3

    if not runFromLaunch:
        bTTS = False

    if if_Number(nodeID):
        node_int = int(nodeID)
        data_out[BLB_CMD.TRAY_A.name] = nodeID
        data_out[BLB_CMD.STATE.name] = BLB_CMD_STATUS.MOVE.name
        node_num = GetKoreanFromNumber(nodeID)
        if bTTS and node_int < 90:
            SendMsgToMQTT(
                mqtt_topic_TTS,
                f"테이블 {node_num}번, {node_num}번 테이블에서 호출합니다.",
            )
        if bTTS and node_int > 90:
            SendMsgToMQTT(mqtt_topic_TTS, f"범블비를 {node_num}에서 호출하였습니다.")
    else:
        data_out[BLB_CMD.STATE.name] = nodeID
        data_out[BLB_CMD.TRAY_A.name] = 0

    data_out[BLB_CMD.TRAY_B.name] = 0
    data_out[BLB_CMD.TIME.name] = getCurrentTime(spliter="")
    sendbuf = json.dumps(data_out)
    callData = String()
    callData.data = sendbuf
    callbackBLB_CMD(callData)


def callbackTopic2mqtt(data):
    # TODO : 응답메세지 발행하는 것도 만들기.
    global stateDic
    # global pub_IMU
    try:
        recvData = data.data
        if is_json(recvData):
            recvDataMap = json.loads(recvData)
        else:
            recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        # logmsg = f'{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}'
        # logmsg = f'{recvData} from {sys._getframe(0).f_code.co_name}'
        # sDIVSTR_FIELD =recvDataMap.get(MQTT_FIELD.DIVSTR_FIELD.name, sDivFieldColon)
        # sDIVSTR_ITEM =recvDataMap.get(MQTT_FIELD.DIVSTR_ITEM.name, sDivItemComma)
        sTOPIC_MQTT = recvDataMap.get(MQTT_FIELD.TOPIC.name, None)
        sPAYLOAD = recvDataMap.get(MQTT_FIELD.PAYLOAD.name, None)
        # recvPayload = getDic_strArr(sPAYLOAD, sDIVSTR_FIELD, sDIVSTR_ITEM)
        if sPAYLOAD == None or sTOPIC_MQTT == None:
            return
        data_out = {}
        if sTOPIC_MQTT.startswith(mqtt_topic_IMU):
            recvDataIMU = getDic_strArr(sPAYLOAD, sDivFieldColon, sDivEmart)
            publish_ImuData(recvDataIMU)
        elif sTOPIC_MQTT == mqtt_topic_call:
            gotoNode(deviceID, sPAYLOAD, True)
        elif is_json(sPAYLOAD):
            data_out = json.loads(sPAYLOAD)
            data_out[CALLBELL_FIELD.TIMESTAMP.name] = getCurrentTime(spliter="")
            nodeID = sTOPIC_MQTT.split("_")[-1]
            if sTOPIC_MQTT.find(CALLBELL_FIELD.CALLBELL.name) > 0:
                #'BLB/mcu_relay_100' : 교차로에서 발행하는 메세지
                #'BLB/mcu_relay_CALLBELL_1' : 호출벨에서 온 메세지인 경우 토픽명에 CALLBELL 이란 단어가 있음.
                if data_out.get(CALLBELL_FIELD.BTN_RED.value, None) == 1:
                    SendMsgToMQTT(
                        mqtt_topic_TTS,
                        f"{GetKoreanFromNumber(nodeID)}번 테이블에서 직원을 호출하셨습니다.",
                    )
                    # SendMsgToMQTT(mqtt_topic_TTS,f'{nodeID}번 테이블에서 직원을 호출하셨습니다.')
                elif data_out.get(CALLBELL_FIELD.BTN_BLUE.value, None) == 1:
                    if data_out.get(CALLBELL_FIELD.CALL_STATE.value, None) == 1:
                        SendMsgToMQTT(
                            mqtt_topic_TTS,
                            f"테이블 {GetKoreanFromNumber(nodeID)}번에서 로봇 호출을 취소하셨습니다.",
                        )
                    else:
                        # SendMsgToMQTT(f'테에이블 {nodeID}번 로봇 호출입니다.')
                        gotoNode(deviceID, nodeID, True)

            # print(type(data_out))
            # print(type(stateDic))
            else:
                stateDic[nodeID] = data_out
            # SendMsgToMQTT() publish (data_out,sTOPIC_MQTT)
            # rospy.loginfo(stateDic)
        else:
            print(sPAYLOAD)
    except Exception as e:
        rospy.loginfo(traceback.format_exc())


rospy.Subscriber(topic_pub_mqtt2topic, String, callbackTopic2mqtt)

"""
CROSS.txt 예제 (교차로 정보)
100 -1 2 3 91

#2023-09-05에 업데이트
#100번 크로스 상태0 일때 시작점은 2과 , 끝점은 3과 연결된 상태.
#100번 크로스 상태1 일때 시작점은 끊김(-1) , 끝점은 91와 연결된 상태.
"""
for i in file_list:
    if i.find("#") >= 0:
        continue
    splitI = i.split(" ")
    if len(splitI) > 4:
        nodeID = (int)(splitI[0])
        stateNode = [
            (int)(splitI[1]),
            (int)(splitI[2]),
            (int)(splitI[3]),
            (int)(splitI[4]),
        ]
        StateInfo[nodeID] = stateNode
print(StateInfo)

listBLB.clear()
listBLB = GetLiftControlUp()
print(listBLB)
if __name__ == "__main__":
    rospy.init_node(node_name, anonymous=False)
    # setNodeStateEx(100, 0)
    # setNodeStateEx(200, 0)

    rate = rospy.Rate(100)  # 루틴을 최대한 빨리 돈다 100hz
    rospy.loginfo(f"{node_name} Started")

    # 현재 생성된 토픽 리스트를 모아 토픽명 배열을 만든다.
    lsTmp = rospy.get_published_topics()
    print(lsTmp)

    for lsCur in lsTmp:
        topic_name = (str)(lsCur[0])
        topic_type = (str)(lsCur[1])

        if topic_type.find("std_msgs/String") >= 0:
            lsTopicList.append(topic_name)
    # lsTopicList 토픽명 배열 완성!

    while not rospy.is_shutdown():
        # bSkip = False
        try:
            dtNow = getDateTime()
            # 이전 루틴과 현재시간 사이의 길이
            td = dtNow - lastUpdateTimeStamp

            # listBLB = []  # 경로 지시정보
            # listTable = []  # 순차서빙 테이블 리스트
            # listTable 에 따라 구체적으로 어떤 모터를 어떻게 움직여야 하는지 정의하는 listBLB 가 생성된다.
            lnMap = 0
            if listBLB != None:
                lnMap = len(listBLB)
            lnTables = len(listTable)
            if lnMap >= 0 and cmdIdx < lnMap and not waitCross:
                if len(activated_motors) == 0 and isTimeExceeded(
                    lastCmdTimeStamp, 2000
                ):
                    dicInfo = listBLB[cmdIdx]
                    if isinstance(dicInfo, dict):  # 주행모드
                        nStart = dicInfo[SeqMapField.START_NODE.name]  # 출발노드ID
                        nStartStatus = dicInfo[
                            SeqMapField.START_STATUS.name
                        ]  # 출발을 위해 설정해야하는 상태값 -1 이면 don't care
                        nTarget = dicInfo[SeqMapField.END_NODE.name]  # 도착노드ID
                        if nTarget == node_CHARGING_STATION:
                            curBLB_Status = BLB_STATUS_FIELD.HOMING

                        nEndStatus = dicInfo[
                            SeqMapField.END_STATUS.name
                        ]  # 도착전에 세팅되어야 하는 상태값
                        nEncoder = (
                            dicInfo[SeqMapField.DISTANCE.name] * 1
                        )  # 이동거리 (엔코더)
                        nDirection = dicInfo.get(
                            SeqMapField.DIRECTION.name, None
                        )  # 진행방향 (정/빽)
                        node_current = (int)(nStart)
                        node_target = (int)(nTarget)
                        node_direction = isTrue(nDirection)

                        if nDirection == None:
                            nDirection = dirPrev
                        if isTrue(nDirection) == False:
                            nEncoder = nEncoder * -1

                        setNodeStateEx(nStart, nStartStatus)
                        setNodeStateEx(nTarget, nEndStatus)
                        # setNodeState(nStart, nStartStatus)
                        # setNodeState(nTarget, nEndStatus)
                        waitCross = True
                    elif isinstance(dicInfo, list):
                        for dicCurrent in dicInfo:
                            print(dicCurrent)
                            if (
                                dicCurrent[MotorWMOVEParams.MBID.name]
                                == ModbusID.MOTOR_H.value
                            ):
                                print(type(dicCurrent))
                                # dicCurrent[MotorWMOVEParams.MODE.name] = 1

                            cmdCurrent = getStr_fromDic(
                                dicCurrent, sDivFieldColon, sDivItemComma
                            )
                            rospy.loginfo(f"Moving motor in list: {dicCurrent}")
                            # curBLB_Status = BLB_STATUS_FIELD.MOVING
                            pub_cmdDevice.publish(cmdCurrent)

                    lastCmdTimeStamp = getDateTime()
                    cmdIdx += 1
                    rospy.loginfo(f"cmdIdx: {cmdIdx}")
            else:
                if lnTables > 0 and not waitCross:
                    doorStatusClose = dicARD_CARRIER.get(
                        CARRIER_STATUS.I_DOOR_1_BOTTOM.name, ""
                    )
                    doorStatusOpen = dicARD_CARRIER.get(
                        CARRIER_STATUS.I_DOOR_1_TOP.name, ""
                    )
                    # 모터 부분 주행은 끝났음.
                    if flag_WaitConfirm:  # 사용자가 UI 터치하는 동안 대기
                        if isTrue(doorStatusOpen) or enableDummyArduino:
                            if node_current == node_KITCHEN_STATION:
                                curBLB_Status = BLB_STATUS_FIELD.READY
                            else:
                                curBLB_Status = BLB_STATUS_FIELD.CONFIRM
                        if isTrue(doorStatusClose) and not flag_req_doorOpen:
                            flag_req_doorOpen = True
                            DoorOpen()
                        # bSkip = True
                    elif flag_liftdown:  # 하강 플랙이 ON이면 하강도킹 진행
                        if is_lifted:
                            if len(activated_motors) == 0 and isTimeExceeded(
                                lastCmdTimeStamp, 2000
                            ):
                                # 아두이노 도어 열을것
                                flag_liftdown = (
                                    False  # 하강 명령어가 들어갔으니 플래그 OFF
                                )
                                flag_WaitConfirm = True
                            else:
                                curBLB_Status = BLB_STATUS_FIELD.LIFTING_DOWN
                        else:
                            listBLB = GetLiftControlDown()
                            curBLB_Status = BLB_STATUS_FIELD.MOVING
                            cmdIdx = 0
                            flag_liftup = True
                            is_docked = False
                            is_lifted = True
                        # bSkip = True
                    elif flag_liftup:
                        if is_lifted:
                            if isTrue(doorStatusClose) or enableDummyArduino:
                                listBLB = GetLiftControlUp()
                                curBLB_Status = BLB_STATUS_FIELD.LIFTING_UP
                                cmdIdx = 0
                                is_docked = True
                                is_lifted = False
                                flag_liftup = (
                                    False  # 상승 명령어가 들어갔으니 플래그 OFF
                                )
                            elif isTrue(doorStatusOpen) and not flag_req_doorClose:
                                flag_req_doorClose = True
                                DoorClose()
                    else:  # 수평모터 구동전 문 닫혀있는지 반드시 확인
                        # if len(activated_motors) == 0 and isTimeExceeded(lastCmdTimeStamp, 2000):

                        if isTrue(doorStatusClose) or enableDummyArduino:
                            tableTarget = listTable.pop(0)
                            if tableTarget != node_NOT_TABLE or enableDummyArduino:
                                listBLB = getSeqMap(node_current, tableTarget)
                                flag_req_doorClose = False
                                flag_req_doorOpen = False
                                if tableTarget != node_CHARGING_STATION:
                                    flag_liftdown = True
                                    lastCmdTimeStamp = getDateTime()
                                cmdIdx = 0
                                print(listBLB, listTable)
                        elif isTrue(doorStatusOpen) and not flag_req_doorClose:
                            flag_req_doorClose = True
                            DoorClose()

            if IsDoorMoving():
                curBLB_Status = BLB_STATUS_FIELD.DOOR_MOVING

            if td.total_seconds() >= 1:
                # ROSQBI 가 부팅된 메세지를 수신받으면 bInit
                # 플래그가 설정되고 10초 후에 터치 초기화 이벤트 메세지를 보낸다.
                if bInit:
                    tdTmp = getDateTime() - timestamp_touchinit
                    if tdTmp.total_seconds() > 10:
                        bInit = False
                        SendStatus(BLB_STATUS_FIELD.INIT)

                SendStatus(curBLB_Status)

                if len(lsTopicList) > 0 and pub_topiclist is not None:
                    pub_topiclist.publish(sDivEmart.join(lsTopicList))

                lastUpdateTimeStamp = getDateTime()
                if len(StateSet) > 0:
                    dicSendMqttTopic = {}
                    # mqttTopic = f'BLB/mcu_relay_CMD/set'
                    mqttRequestTopic = f"BLB/CROSS_CMD/set"
                    dicSendMqttTopic[MQTT_FIELD.TOPIC.name] = mqttRequestTopic
                    dicSendMqttTopic[MQTT_FIELD.PAYLOAD.name] = json.dumps(StateSet)
                    # dicSendMqttTopic[MQTT_FIELD.PAYLOAD.name] = f'E:{statusVal},ID:{nodeID}'
                    data_out = json.dumps(dicSendMqttTopic)
                    pub_topic2mqtt.publish(data_out)
                # rospy.loginfo(f'Loop : {cntLoop}, Cur node : {node_current}, next node:{node_target}, dir:{node_direction} wait:{flag_WaitConfirm}, State:{stateDic}')
                if waitCross:
                    curBLB_Status = BLB_STATUS_FIELD.WAITING_CROSS
                    rospy.loginfo(
                        f"cmdIdx:{cmdIdx},CrossStatus:{nStart},{curStartState}/{nStartStatus}|{nTarget},{curTargetState}/{nEndStatus},CurNode:{node_current},NextNode:{node_target},{listTable}"
                    )
                else:
                    rospy.loginfo(
                        f"cmdIdx:{cmdIdx},Loop:{cntLoop},Curnode:{node_current},NextNode:{node_target},Dir:{node_direction},Wait:{flag_WaitConfirm},Tables:{listTable}"
                    )
                    if node_target == node_CHARGING_STATION:
                        curBLB_Status = BLB_STATUS_FIELD.HOMING
                    elif node_current == node_CHARGING_STATION and node_target == 0:
                        curBLB_Status = BLB_STATUS_FIELD.CHARGING
                cntLoop = 0
            else:
                cntLoop += 1

            if waitCross:
                curStartState = getNodeState(nStart)
                curTargetState = getNodeState(nTarget)
                # while curStartState != nStartStatus or curTargetState != nEndStatus:
                # if curStartState != nStartStatus or curTargetState != nEndStatus:
                if curStartState == nStartStatus and curTargetState == nEndStatus:
                    if nEncoder > 650000:
                        nEncoder = 650000

                    sendbuf = getMotorMoveString(
                        ModbusID.MOTOR_H.value, True, nEncoder, 2500, 1000, 700
                    )
                    lastCmdTimeStamp = getDateTime()
                    # activated_motors.append(ModbusID.MOTOR_H.value)
                    pub_cmdDevice.publish(sendbuf)
                    curBLB_Status = BLB_STATUS_FIELD.MOVING
                    # rospy.loginfo(f'Moving motor : {dicInfo}')
                    rospy.loginfo(f"Moving motor : {nStart}->{nTarget}")
                    dirPrev = nDirection
                    nStart = ""
                    nStartStatus = ""
                    nTarget = ""
                    nEndStatus = ""
                    nEncoder = 0
                    waitCross = False
        except Exception as e:
            bReturn = False
            rospy.loginfo(traceback.format_exc())
            rospy.signal_shutdown(e)
        # rate.sleep()

    rospy.spin()
