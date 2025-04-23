from node_CtlCenter_callback import *

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
            dtNow = datetime.datetime.now()
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
                        nEncoder = dicInfo[SeqMapField.DISTANCE.name] * 1  # 이동거리 (엔코더)
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

                    lastCmdTimeStamp = datetime.datetime.now()
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
                                flag_liftdown = False  # 하강 명령어가 들어갔으니 플래그 OFF
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
                                flag_liftup = False  # 상승 명령어가 들어갔으니 플래그 OFF
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
                                    lastCmdTimeStamp = datetime.datetime.now()
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
                    tdTmp = datetime.datetime.now() - timestamp_touchinit
                    if tdTmp.total_seconds() > 10:
                        bInit = False
                        SendStatus(BLB_STATUS_FIELD.INIT)

                SendStatus(curBLB_Status)

                if len(lsTopicList) > 0 and pub_topiclist is not None:
                    pub_topiclist.publish(sDivEmart.join(lsTopicList))

                lastUpdateTimeStamp = datetime.datetime.now()
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
                    lastCmdTimeStamp = datetime.datetime.now()
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
