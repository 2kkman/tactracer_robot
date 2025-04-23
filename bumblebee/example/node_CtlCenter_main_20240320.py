"""
2024-03-20 주행정보 자료 구조 리뉴얼 전 백업
기존 설계안은 전역변수를 사용하여 코드의 유지보수 및 가독성이 떨어지고
동시에 여러개의 분기기를 컨트롤 하는것이 불가.
분기기 컨트롤 정보는 Dict 로 변경하여 한번에 컨트롤 할수 있도록 업데이트
하기전 백업해둔 버전임. (이 파일)

from node_CtlCenter_import import *
from node_CtlCenter_func import *
from node_CtlCenter_callback import *
import node_CtlCenter_globals
"""

node_CtlCenter_globals.listBLB.clear()
node_CtlCenter_globals.listBLB = GetLiftControlUp()
print(node_CtlCenter_globals.listBLB)
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
            node_CtlCenter_globals.lsTopicList.append(topic_name)
    # lsTopicList 토픽명 배열 완성!

    sendInit = getMotorSimpleCmdString(ModbusID.MOTOR_H.value, MotorCmdField.WSTOP)
    pub_cmdDevice.publish(sendInit)

    while not rospy.is_shutdown():
        # bSkip = False
        try:
            dtNow = datetime.datetime.now()
            # 이전 루틴과 현재시간 사이의 길이
            td_local = dtNow - node_CtlCenter_globals.lastUpdateTimeStamp

            # listBLB = []  # 경로 지시정보
            # listTable = []  # 순차서빙 테이블 리스트
            # listTable 에 따라 구체적으로 어떤 모터를 어떻게 움직여야 하는지 정의하는 listBLB 가 생성된다.
            lnMap = 0

            # 경로지시정보 리스트가 비어있는지 확인
            if node_CtlCenter_globals.listBLB != None:
                lnMap = len(node_CtlCenter_globals.listBLB)
            lnTables = len(node_CtlCenter_globals.listTable)

            if (
                lnMap >= 0
                and node_CtlCenter_globals.cmdIdx
                < lnMap  # cmdIdx 가 lnMap 보다 같거나 크면 지시정보 수행 완료 상태이므로 넘어감
                and not node_CtlCenter_globals.waitCross  # waitCross 면 분기소 동작중이므로 대기.
            ):
                # 동작중인 모터가 없고 동작지시명령 수신부터 2초이상 지났을때
                if len(node_CtlCenter_globals.activated_motors) == 0 and isTimeExceeded(
                    node_CtlCenter_globals.lastCmdTimeStamp, 2000
                ):
                    # dicInfo_local 에 지금부터 수행해야할 모터 제어정보 입력
                    dicInfo_local = node_CtlCenter_globals.listBLB[
                        node_CtlCenter_globals.cmdIdx
                    ]
                    print(type(dicInfo_local), dicInfo_local)

                    # dicInfo_local 는 dict 또는 list 가 될 수 있으며
                    # dict 인 경우는 캐리어 모터를 움직이는 경우 (모터 1개만 제어)
                    # list 인 경우는 밸런싱 암 및 견인모터를 움직이는 경우 (다중 모터 제어)
                    if isinstance(dicInfo_local, dict):  # 주행모드
                        # 주행관련 글로벌 변수에 지시정보 입력
                        node_CtlCenter_globals.nStart = dicInfo_local[
                            SeqMapField.START_NODE.name
                        ]  # 출발노드ID
                        node_CtlCenter_globals.nStartStatus = dicInfo_local[
                            SeqMapField.START_STATUS.name
                        ]  # 출발을 위해 설정해야하는 상태값 -1 이면 don't care
                        # 만일 분기기에 대기중이라면 분기기 상태가 지정된 값이 될때까지 출발을 보류

                        # 도착지 세팅(도착노드ID)
                        node_CtlCenter_globals.nTarget = dicInfo_local[
                            SeqMapField.END_NODE.name
                        ]

                        # 이동해야할 곳이 충전스테이션이면 로봇 상태정보를 HOMING 으로 변경
                        if node_CtlCenter_globals.nTarget == node_CHARGING_STATION:
                            node_CtlCenter_globals.curBLB_Status = (
                                BLB_STATUS_FIELD.HOMING
                            )

                        # 도착전에 세팅되어야 하는 상태값
                        node_CtlCenter_globals.nEndStatus = dicInfo_local[
                            SeqMapField.END_STATUS.name
                        ]

                        # 이동거리 설정
                        node_CtlCenter_globals.nEncoder = (
                            dicInfo_local[SeqMapField.DISTANCE.name] * 1
                        )

                        # # 진행방향 가져오기
                        # node_CtlCenter_globals.nDirection = dicInfo_local.get(
                        #     SeqMapField.DIRECTION.name, None
                        # )

                        # 현재노드를 시작 노드로 변경
                        node_CtlCenter_globals.node_current = (int)(
                            node_CtlCenter_globals.nStart
                        )

                        # 목적 노드를 지시정보대로 세팅
                        node_CtlCenter_globals.node_target = (int)(
                            node_CtlCenter_globals.nTarget
                        )

                        # # 정역방향 세팅
                        # node_CtlCenter_globals.node_direction = isTrue(
                        #     node_CtlCenter_globals.nDirection
                        # )

                        # # 정역방향 지시정보가 별도로 없는 경우 이전 주행과 같은 방향으로 세팅
                        # if node_CtlCenter_globals.nDirection is None:
                        #     node_CtlCenter_globals.nDirection = (
                        #         node_CtlCenter_globals.dirPrev
                        #     )

                        # # 역방향일때는 진행거리에 -1 을 곱한다.
                        # if isTrue(node_CtlCenter_globals.nDirection) is False:
                        #     node_CtlCenter_globals.nEncoder *= -1

                        # 시작지점 위치와 분기기 상태를 세팅한다 (분기기 및 리프트에서 이 정보를 모니터링)
                        setNodeStateEx(
                            node_CtlCenter_globals.nStart,
                            node_CtlCenter_globals.nStartStatus,
                        )
                        # 종료지점도 마찬가지
                        setNodeStateEx(
                            node_CtlCenter_globals.nTarget,
                            node_CtlCenter_globals.nEndStatus,
                        )
                        # setNodeState(nStart, nStartStatus)
                        # setNodeState(nTarget, nEndStatus)

                        # 분기기 처리 대기 변수를 세팅한다.
                        node_CtlCenter_globals.waitCross = True

                    # 지시정보가 list, 즉 밸런싱 및 견인모터인 경우
                    # 다중제어로 한 리스트에는 복수 모터의 지시정보가 들어있고 동시에 실행시킨다.
                    elif isinstance(dicInfo_local, list):
                        for dicTmp in dicInfo_local:
                            # 모터 지시정보 확인
                            print(type(dicTmp), dicTmp)

                            # 지시정보 설정 오류 (이 부분에서는 주행 모터 컨트롤이 들어가면 안됨)
                            # TODO : 일단 print 정도만 실행, 추후 구체적인 액션코드 추가
                            if (
                                dicTmp[MotorWMOVEParams.MBID.name]
                                == ModbusID.MOTOR_H.value
                            ):
                                print(type(dicTmp))
                                # dicCurrent[MotorWMOVEParams.MODE.name] = 1

                            # MODBUS_IF 로 던질 제어명령어 생성
                            cmdCurrent_local = getStr_fromDic(
                                dicTmp, sDivFieldColon, sDivItemComma
                            )
                            rospy.loginfo(f"제어명령어 송신 : {dicTmp}")
                            pub_cmdDevice.publish(cmdCurrent_local)
                    else:
                        # 진입해서는 안될 부분. 지시정보 생성에 오류발생.
                        print(type(dicTmp), dicTmp)
                        sErrMsg = (
                            f"지시정보 생성 오류 발생 : 진입해서는 안될 부분 - {dicTmp}"
                        )
                        rospy.loginfo(sErrMsg)
                        raise Exception(sErrMsg)  # 예외를 발생시킴
                    # 공통부분
                    # 1.모터지시정보를 내렸으니 지시정보 시각 갱신
                    node_CtlCenter_globals.lastCmdTimeStamp = datetime.datetime.now()
                    # 2.지시정보 던진 후 인덱스 증가
                    node_CtlCenter_globals.cmdIdx += 1
                    rospy.loginfo(f"cmdIdx: {node_CtlCenter_globals.cmdIdx}")

            # 모터 주행중 혹은 분기기 처리 대기중인 경우
            else:
                # 분기기 처리 대기중은 아니고
                #
                if lnTables > 0 and not node_CtlCenter_globals.waitCross:
                    doorStatusClose_local = node_CtlCenter_globals.dicARD_CARRIER.get(
                        CARRIER_STATUS.I_DOOR_1_BOTTOM.name, ""
                    )
                    doorStatusOpen_local = node_CtlCenter_globals.dicARD_CARRIER.get(
                        CARRIER_STATUS.I_DOOR_1_TOP.name, ""
                    )
                    # 모터 부분 주행은 끝났음.
                    if (
                        node_CtlCenter_globals.flag_WaitConfirm
                    ):  # 사용자가 UI 터치하는 동안 대기
                        if (
                            isTrue(doorStatusOpen_local)
                            or node_CtlCenter_globals.enableDummyArduino
                        ):
                            if (
                                node_CtlCenter_globals.node_current
                                == node_KITCHEN_STATION
                            ):
                                node_CtlCenter_globals.curBLB_Status = (
                                    BLB_STATUS_FIELD.READY
                                )
                            else:
                                node_CtlCenter_globals.curBLB_Status = (
                                    BLB_STATUS_FIELD.CONFIRM
                                )
                        if (
                            isTrue(doorStatusClose_local)
                            and not node_CtlCenter_globals.flag_req_doorOpen
                        ):
                            node_CtlCenter_globals.flag_req_doorOpen = True
                            DoorOpen()
                        # bSkip = True
                    elif (
                        node_CtlCenter_globals.flag_liftdown
                    ):  # 하강 플랙이 ON이면 하강도킹 진행
                        if node_CtlCenter_globals.is_lifted:
                            if len(
                                node_CtlCenter_globals.activated_motors
                            ) == 0 and isTimeExceeded(
                                node_CtlCenter_globals.lastCmdTimeStamp, 2000
                            ):
                                # 아두이노 도어 열을것
                                node_CtlCenter_globals.flag_liftdown = (
                                    False  # 하강 명령어가 들어갔으니 플래그 OFF
                                )
                                node_CtlCenter_globals.flag_WaitConfirm = True
                            else:
                                node_CtlCenter_globals.curBLB_Status = (
                                    BLB_STATUS_FIELD.LIFTING_DOWN
                                )
                        else:
                            node_CtlCenter_globals.listBLB = GetLiftControlDown()
                            node_CtlCenter_globals.curBLB_Status = (
                                BLB_STATUS_FIELD.MOVING
                            )
                            node_CtlCenter_globals.cmdIdx = 0
                            node_CtlCenter_globals.flag_liftup = True
                            node_CtlCenter_globals.is_docked = False
                            node_CtlCenter_globals.is_lifted = True
                        # bSkip = True
                    elif node_CtlCenter_globals.flag_liftup:
                        if node_CtlCenter_globals.is_lifted:
                            if (
                                isTrue(doorStatusClose_local)
                                or node_CtlCenter_globals.enableDummyArduino
                            ):
                                node_CtlCenter_globals.listBLB = GetLiftControlUp()
                                node_CtlCenter_globals.curBLB_Status = (
                                    BLB_STATUS_FIELD.LIFTING_UP
                                )
                                node_CtlCenter_globals.cmdIdx = 0
                                node_CtlCenter_globals.is_docked = True
                                node_CtlCenter_globals.is_lifted = False
                                node_CtlCenter_globals.flag_liftup = (
                                    False  # 상승 명령어가 들어갔으니 플래그 OFF
                                )
                            elif (
                                isTrue(doorStatusOpen_local)
                                and not node_CtlCenter_globals.flag_req_doorClose
                            ):
                                node_CtlCenter_globals.flag_req_doorClose = True
                                DoorClose()
                    else:  # 수평모터 구동전 문 닫혀있는지 반드시 확인
                        # if len(activated_motors) == 0 and isTimeExceeded(lastCmdTimeStamp, 2000):

                        if (
                            isTrue(doorStatusClose_local)
                            or node_CtlCenter_globals.enableDummyArduino
                        ):
                            tableTarget_local = node_CtlCenter_globals.listTable.pop(0)
                            if (
                                tableTarget_local != node_NOT_TABLE
                                or node_CtlCenter_globals.enableDummyArduino
                            ):
                                # 경로구성
                                node_CtlCenter_globals.listBLB = getSeqMap(
                                    node_CtlCenter_globals.node_current,
                                    tableTarget_local,
                                )
                                node_CtlCenter_globals.flag_req_doorClose = False
                                node_CtlCenter_globals.flag_req_doorOpen = False
                                if tableTarget_local != node_CHARGING_STATION:
                                    node_CtlCenter_globals.flag_liftdown = True
                                    node_CtlCenter_globals.lastCmdTimeStamp = (
                                        datetime.datetime.now()
                                    )
                                node_CtlCenter_globals.cmdIdx = 0
                                print(
                                    node_CtlCenter_globals.listBLB,
                                    node_CtlCenter_globals.listTable,
                                )
                        elif (
                            isTrue(doorStatusOpen_local)
                            and not node_CtlCenter_globals.flag_req_doorClose
                        ):
                            node_CtlCenter_globals.flag_req_doorClose = True
                            DoorClose()

            if IsDoorMoving():
                node_CtlCenter_globals.curBLB_Status = BLB_STATUS_FIELD.DOOR_MOVING

            if td_local.total_seconds() >= 1:
                # ROSQBI 가 부팅된 메세지를 수신받으면 bInit == True 상태가 됨
                # 플래그가 설정되고 10초 후에 터치 초기화 이벤트 메세지를 보낸다.
                if node_CtlCenter_globals.bInit:
                    tdTmp_local = (
                        datetime.datetime.now()
                        - node_CtlCenter_globals.timestamp_touchinit
                    )
                    if tdTmp_local.total_seconds() > 10:
                        node_CtlCenter_globals.bInit = False
                        SendStatus(BLB_STATUS_FIELD.INIT)
                        # 초기화 이벤트 메세지 보낸 후에는 이 루틴에 다시 진입하지 않도록
                        # node_CtlCenter_globals.bInit = False

                # 1초마다 현재 로봇 상태값 송신
                SendStatus(node_CtlCenter_globals.curBLB_Status)

                # 현재 토픽 리스트를 문자열로 만들어 TOPIC_LIST 토픽에 문자열로 발행
                # 아직은 쓸모가 없다
                if (
                    len(node_CtlCenter_globals.lsTopicList) > 0
                    and pub_TOPIC_LIST is not None
                ):
                    pub_TOPIC_LIST.publish(
                        sDivEmart.join(node_CtlCenter_globals.lsTopicList)
                    )

                # lastupdatetimestamp 갱신
                node_CtlCenter_globals.lastUpdateTimeStamp = datetime.datetime.now()

                """
                StateInfo: Dict[str, list] = {}
                StateSet: Dict[int, int] = {}
                nodeStateOpen = [0, 1]  # 인덱스가 0,1 일때는 Open 상태, 2,3 일땐 Close
                
                """
                # 현재 범블비가 어디있는지 (키 0) MQTT로 브릿지나 wifi호출벨에 알린다.
                setNodeStateEx(0, node_CtlCenter_globals.node_current)

                if len(node_CtlCenter_globals.StateSet) > 0:
                    dicSendMqttTopic_local = {}
                    dicSendMqttTopic_local[MQTT_FIELD.TOPIC.name] = (
                        MQTT_TOPIC_VALUE.MSG_CROSS_REQUEST.value
                    )
                    dicSendMqttTopic_local[MQTT_FIELD.PAYLOAD.name] = json.dumps(
                        node_CtlCenter_globals.StateSet
                    )
                    # dicSendMqttTopic[MQTT_FIELD.PAYLOAD.name] = f'E:{statusVal},ID:{nodeID}'
                    data_out = json.dumps(dicSendMqttTopic_local)
                    pub_topic2mqtt.publish(data_out)
                # rospy.loginfo(f'Loop : {cntLoop}, Cur node : {node_current}, next node:{node_target}, dir:{node_direction} wait:{flag_WaitConfirm}, State:{stateDic}')
                cmd_pos = -1
                cur_pos = -1
                str_mbid_motor_h = (str)(ModbusID.MOTOR_H.value)

                if (
                    node_CtlCenter_globals.dic_485ex.get(str_mbid_motor_h, None)
                    is not None
                ):
                    # print(node_CtlCenter_globals.dic_485ex[str_mbid_motor_h])
                    # print(type(node_CtlCenter_globals.dic_485ex[str_mbid_motor_h]))
                    cmd_pos = node_CtlCenter_globals.dic_485ex[str_mbid_motor_h].get(
                        MonitoringField.CMD_POS.name, -1
                    )
                    cur_pos = node_CtlCenter_globals.dic_485ex[str_mbid_motor_h].get(
                        MonitoringField.CUR_POS.name, -1
                    )
                if node_CtlCenter_globals.waitCross:
                    node_CtlCenter_globals.curBLB_Status = (
                        BLB_STATUS_FIELD.WAITING_CROSS
                    )
                    # rospy.loginfo(
                    print(
                        f"cmdIdx:{node_CtlCenter_globals.cmdIdx},CrossStatus:{node_CtlCenter_globals.nStart},{node_CtlCenter_globals.curStartState}/{node_CtlCenter_globals.nStartStatus}|{node_CtlCenter_globals.nTarget},{node_CtlCenter_globals.curTargetState}/{node_CtlCenter_globals.nEndStatus},CurNode:{node_CtlCenter_globals.node_current},NextNode:{node_CtlCenter_globals.node_target},cmdpos={cmd_pos},curpos={cur_pos},{node_CtlCenter_globals.listTable}"
                    )
                else:
                    # rospy.loginfo(
                    print(
                        f"cmdIdx:{node_CtlCenter_globals.cmdIdx},Loop:{node_CtlCenter_globals.cntLoop},Curnode:{node_CtlCenter_globals.node_current},NextNode:{node_CtlCenter_globals.node_target},Wait:{node_CtlCenter_globals.flag_WaitConfirm},cmdpos={cmd_pos},curpos={cur_pos},Heading:{node_CtlCenter_globals.moveForward},{node_CtlCenter_globals.listTable}"
                    )
                    if node_CtlCenter_globals.node_target == node_CHARGING_STATION:
                        node_CtlCenter_globals.curBLB_Status = BLB_STATUS_FIELD.HOMING
                    elif (
                        node_CtlCenter_globals.node_current == node_CHARGING_STATION
                        and node_CtlCenter_globals.node_target == 0
                    ):
                        node_CtlCenter_globals.curBLB_Status = BLB_STATUS_FIELD.CHARGING
                node_CtlCenter_globals.cntLoop = 0
            else:
                node_CtlCenter_globals.cntLoop += 1

            if node_CtlCenter_globals.waitCross:
                node_CtlCenter_globals.curStartState = getNodeState(
                    node_CtlCenter_globals.nStart
                )
                node_CtlCenter_globals.curTargetState = getNodeState(
                    node_CtlCenter_globals.nTarget
                )
                # while curStartState != nStartStatus or curTargetState != nEndStatus:
                # if curStartState != nStartStatus or curTargetState != nEndStatus:

                if (
                    node_CtlCenter_globals.curStartState
                    == node_CtlCenter_globals.nStartStatus
                    and node_CtlCenter_globals.curTargetState
                    == node_CtlCenter_globals.nEndStatus
                ):
                    # 데모시 현재 안전장치가 없으므로 엔코더 최대값을 제한한다
                    #     if nEncoder > 650000:
                    #         nEncoder = 650000
                    sendbuf = getMotorMoveString(
                        ModbusID.MOTOR_H.value,
                        False,
                        node_CtlCenter_globals.nEncoder,
                        1500,
                        1000,
                        700,
                    )
                    node_CtlCenter_globals.lastCmdTimeStamp = datetime.datetime.now()
                    # activated_motors.append(ModbusID.MOTOR_H.value)
                    node_CtlCenter_globals.lastSendbuf_h = sendbuf
                    pub_cmdDevice.publish(sendbuf)
                    node_CtlCenter_globals.curBLB_Status = BLB_STATUS_FIELD.MOVING
                    # rospy.loginfo(f'Moving motor : {dicInfo}')
                    rospy.loginfo(
                        f"Moving motor : {node_CtlCenter_globals.nStart}->{node_CtlCenter_globals.nTarget}"
                    )
                    # node_CtlCenter_globals.dirPrev = node_CtlCenter_globals.nDirection
                    node_CtlCenter_globals.nStart = ""
                    node_CtlCenter_globals.nStartStatus = ""
                    node_CtlCenter_globals.nTarget = ""
                    node_CtlCenter_globals.nEndStatus = ""
                    node_CtlCenter_globals.nEncoder = 0
                    node_CtlCenter_globals.waitCross = False
        except Exception as e:
            bReturn = False
            rospy.loginfo(traceback.format_exc())
            rospy.signal_shutdown(e)
        # rate.sleep()

    rospy.spin()
