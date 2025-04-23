#!/usr/bin/env python3
from node_Status_callback import *

if __name__ == "__main__":
    try:
        rate = rospy.Rate(1)  # send 1 time per second

        # rospy.Subscriber(TopicName.TABLE_ORDER_STATUS.name, String, callbackCmd2)
        # runFromLaunch = rospy.get_param("~startReal", default=False)
        # print(f"runFromLaunch : {runFromLaunch}")

        # 모니터링 해야할 토픽들을 정의하고 LastSeen 을 업데이트 한다
        # 일정이상 보내지 않는 경우 지정된 통신끊김 데이터를 모니터링 대상 장치와 결합시켜 발송한다.
        while not rospy.is_shutdown():
            returnls = []
            for mbid, lsMonitorItem in node_Status_import.dictMonitorTopics.items():
                recvDataMap = {}
                returnDic = {}
                returnRefined = {}
                # print(returnDic)
                curItem = node_Status_import.dictMonitorData.get(mbid, None)
                if curItem is None:
                    continue
                recvData = curItem.datastr
                if is_json(recvData):
                    recvDataMap = json.loads(recvData)
                else:
                    recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)

                for fieldTmp in lsMonitorItem:
                    fieldValue = recvDataMap.get(fieldTmp, None)
                    if fieldValue is not None:
                        # continue
                        returnDic[fieldTmp] = fieldValue
                returnDic[MotorWMOVEParams.MBID.name] = GetLastString(mbid, "_")
                if mbid == TopicName.RECEIVE_MQTT.name or mbid == TopicName.RFID.name:
                    if not MonitoringField_EX.ALM_CD.name in recvDataMap.keys():
                        returnDic[MonitoringField_EX.ALM_CD.name] = "0"

                if len(returnDic) == 0:
                    continue
                dicTmp = GenerateReadableDicMotor(returnDic)
                returnls.append(dicTmp)

            if len(returnls) > 0 and pub is not None:
                title = "범블비 주요부품 알람정보"
                finalDict = getFullData(rid, title, returnls)
                print(f"Publish : {finalDict}")
                sendbuf = json.dumps(finalDict)
                pub.publish(sendbuf)
                # prtMsg(sendbuf)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
