#!/usr/bin/env python3
import node_Status_import
from node_Status_import import *

def getFullData(rid, title, dictData):
    data = {
        "rid": rid,
        "title": title,
        "data": dictData,
        "lastseen": getCurrentTime("", True),
    }
    return data

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
        v_int = try_parse_int(v)
        if k == MotorWMOVEParams.MBID.name:
            # rospy.loginfo(f'checked : {k}')
            mbid_caption = ModbusID_EX.GetEnumValue(v_int)
            if mbid_caption is not None:
                returnDic["부품명"] = mbid_caption["caption"]
                returnDic["카테고리"] = "서보모터"
        elif k == RFID_RESULT.DEVID.name:
            caption = f"RFID_{v}"
            returnDic["부품명"] = caption
            returnDic["카테고리"] = "RFID"
            if RFID_RESULT.PWR.name in dictTmp.keys():
                returnDic["상태"] = dictTmp[RFID_RESULT.PWR.name]
            if RFID_RESULT.EPC.name in dictTmp.keys():
                returnDic["상태"] = dictTmp[RFID_RESULT.EPC.name]

        elif k == "ID":
            caption = f"분기기{v}"
            returnDic["부품명"] = caption
            returnDic["카테고리"] = "분기기"

        elif k == ARUCO_RESULT_FIELD.CAM_ID.name:
            caption = f"카메라{v}"
            returnDic["부품명"] = caption
            returnDic["카테고리"] = "카메라"
            returnDic["상태"] = "정상"

        if k == "STATE":
            print(getCurrentTime(), k, v)
            returnDic["상태"] = v

        (
            value,
            description_value_negative,
            description_value_zero,
            description_value_nonZero,
        ) = MonitoringField_EX.GetEnumValue(k)
        if value == None or description_value_zero == None:
            continue
        statusStr = ""
        if v_int == 1:
            statusStr = description_value_nonZero
        elif v_int == -1:
            statusStr = description_value_negative
        elif v_int == 0:
            statusStr = description_value_zero
        else:
            statusStr = value

        # MBID 필드가 있는 경우 모터의 이름을 알아내고,
        # ID 필드가 있는 경우, 분기기로 처리할 것

        # MBID 로 모터 이름 가져오기
        # else:
        #   rospy.loginfo(f'skipped : {k}')

        if k == MonitoringField_EX.ALM_CD.name:
            returnDic["알람코드"] = v
        returnDic[value] = statusStr
    return returnDic


def callbackCmd(data, topicName):
    curtmp = ""
    datastr = data.data
    node_Status_import.lock.acquire()
    try:
        if topicName == TopicName.RECEIVE_MQTT.name:
            mqttDic = json.loads(datastr)
            curRawValue = mqttDic.get(MQTT_FIELD.PAYLOAD.name, "")
            curtmp = MonitorData(curRawValue, lastseen=getDateTime())
            print(mqttDic)
        elif topicName == TopicName.ARUCO_RESULT.name:
            camKeepAlive = json.loads(datastr)
            curRawValue = camKeepAlive.get(ARUCO_RESULT_FIELD.CAM_ID.name, "")
            curtmp = MonitorData(curRawValue, lastseen=getDateTime())
            print(camKeepAlive)
        elif topicName == TopicName.BLB_STATUS_MONITOR_BMS_NTP.name:
          curDic = getDic_strArr(datastr, sDivFieldColon, sDivItemComma)
          filtered_dic = {key: curDic[key] for key in curDic if key in MonitoringField_BMS.__members__}
          finalDict=getFullData(rid, '범블비 배터리 정보', filtered_dic)
          print(f"Publish BMS: {finalDict}")
          sendbuf = json.dumps(finalDict)
          pubBMS.publish(sendbuf)
        else:
            curtmp = MonitorData(datastr, lastseen=getDateTime())
        node_Status_import.dictMonitorData[topicName] = curtmp
    finally:
        # Lock을 해제해서 다른 Thread도 사용할 수 있도록 만든다.
        node_Status_import.lock.release()


# def callbackMQTT_CROSS(data):
#     #먼저 cross.txt 에 정의된 항목을 모두 읽어 확인한다.
#     fieldName = GetLastString(sys._getframe(0).f_code.co_name, '_')
#     dataDic=json.loads(data.data)
#     topicValue = dataDic.get(MQTT_FIELD.TOPIC.name, None) #BLB/CROSS_100
#     payLoadValue = dataDic.get(MQTT_FIELD.PAYLOAD.name, None)
#     payLoadDic = json.loads(payLoadValue)
#     #crossID = GetLastString(topicValue, '_') #100
#     crossID = payLoadDic.get(CALLBELL_FIELD.ID.name, None)
#     crossState = payLoadDic.get(BLB_CMD.STATE.name, None)
#     lsDataField = [fieldName, ]
#     node_Status_import.lock_MQTT.acquire()
#     try:
#         node_Status_import. dictMonitorData[topicName] = curtmp
#     finally:
#         # Lock을 해제해서 다른 Thread도 사용할 수 있도록 만든다.
#         node_Status_import.lock_MQTT.release()

rospy.Subscriber(
    'BMS',
    String,
    callbackCmd,
    callback_args=TopicName.BLB_STATUS_MONITOR_BMS_NTP.name,
)
rospy.Subscriber(
    TopicName.ARUCO_RESULT.name,
    String,
    callbackCmd,
    callback_args=TopicName.ARUCO_RESULT.name,
)
print(dictMonitorTopics)
for topicname, listFields in dictMonitorTopics.items():
    print(f"Subscribe - {idxCur}:{topicname}-{listFields}")
    rospy.Subscriber(topicname, String, callbackCmd, callback_args=topicname)
