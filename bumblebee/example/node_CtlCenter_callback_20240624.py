#!/usr/bin/env python3
from node_CtlCenter_func_control import *
lock = threading.Lock()

'''
20240624 - 현재 callback 함수내에서 처리하는 기능들이 많아 CPU 부하가 높아짐.
Callback Thread 부하를 최소화 하기 위해 callback 은 memory access 역할만 함.
나머지는 main loop 에서 처리할 것.
이 파일은 callback 중심처리 설계의 백업본.
'''

def callbackAck(data):
    # TODO : 응답메세지 발행하는 것도 만들기.
    """_summary_
        /ACK 토픽
        모터 동작 지시/완료 메세지를 수신하여
        모터 상태정보 글로벌 변수에 반영한다.
        node_CtlCenter_globals.node_target
        node_CtlCenter_globals.node_current
    data example :
        data: "1707275205.2646:0:26"
        data: "1707275205.265836:0:30"
        data: "1707275205.2646:1:26"
        data: "1707275205.265836:1:30"
    """
    try:
        recvData = data.data
        logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name} : {node_CtlCenter_globals.activated_motors}"
        rospy.loginfo(logmsg)

        # Colon 으로 메세지를 쪼개어 파싱한다.
        # idx 0 - timestamp, 1 - 완료여부 , 2 - MBID
        lsResult = recvData.split(sDivFieldColon)

        mbid_tmp = lsResult[2]  # 모드버스 ID
        flag = lsResult[1]  # 0 이면 미완료, 1 이면 완료
        if mbid_tmp == str(ModbusID.BAL_ARM1.value):
          rospy.loginfo(f'Ack Modbus Time Stamp:{node_CtlCenter_globals.dicModbusCallbackCount[mbid_tmp]},{mbid_tmp}:{flag}')
        
        timestamp_local = float(
            lsResult[0]
        )  # TimeStamp - 모터 구동지시 시각 (완료 시각 아님)
        datetimeobj = datetime.datetime.fromtimestamp(
            timestamp_local
        )  # TimeStamp 를 datetime 으로 변환
        dtnow = datetime.datetime.now()
        finishTime_local = (
            dtnow - datetimeobj
        )  # 모터 구동지시 시각으로 부터 경과한 시간

        # 이 부분에서 현재 어떠한 상태인지 알 수 있음. (하강인지 상승인지 등)
        if flag == "0":
            # 모터 구동지시 명령수신 -> 활성모터 리스트에 추가.
            node_CtlCenter_globals.activated_motors.append(mbid_tmp)
        else:
            # 모터 구동 완료 ACK 수신 - 활성모터 리스트에서 제거
            node_CtlCenter_globals.activated_motors.remove(mbid_tmp)

            # 수평모터 구동이 완료된 경우 다음목적지로 이동해야 한다.
            if mbid_tmp == (str)(ModbusID.MOTOR_H.value):
                # 다음 목적지가 없으면 (node_target == 0) 운행 종료
                # 다음목적지가 있으면 현재 노드를 node_target 으로 변경하고 node_target = 0 처리
                if node_CtlCenter_globals.node_target != 0:
                    node_CtlCenter_globals.node_current = (
                        node_CtlCenter_globals.node_target
                    )
                    node_CtlCenter_globals.node_target = 0
            rospy.loginfo(
                f"모터번호{mbid_tmp}/구동시간 : {finishTime_local:.1f}, curr_node:{node_CtlCenter_globals.node_current}"
            )
        # if len(node_CtlCenter_globals.activated_motors)==0:
        #     node_CtlCenter_globals.status_bal = STATUS_BALANCING.EXTEND_FINISHED
    except Exception as e:
        message = traceback.format_exc()
        rospy.logdebug(message)
        # SendFeedback(e)


def callbackBLB_CMD(data):
    """_summary_
        /BLB_CMD 토픽, 즉 UI, 혹은 MQTT 등에서 날라오는 명령어를 모니터링
        data: "{\"ID\": \"Bumblebee1\", \"TRAY_A\": \"2\", \"TRAY_B\": \"1\", \"LEVEL\": \"0\",\
          \ \"STATE\": \"CONFIRM\", \"TIME\": \"20240207_141000\"}"

    Args:
        data (_type_): _description_
    """
    try:
        recvDataMap = {}
        recvData = (str)(data.data)
        logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
        rospy.loginfo(logmsg)
        # recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivEmart)
        # 커맨드 형태가 JSON 인 경우 (범블비 UI에서 날아오는 경우)
        # 와 구분자로 된 데이터 파싱하여 dict 개체 생성
        if is_json(recvData):
            recvDataMap = json.loads(recvData)
        else:
            recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivEmart)
        rospy.loginfo(recvDataMap)

        # STATE 값에 따라 처리하는 기능들이 달라진다.
        PROFILE = recvDataMap.get(BLB_CMD_CUSTOM.PROFILE.name, "")
        # STATE 값에 따라 처리하는 기능들이 달라진다.
        STATE = recvDataMap.get(BLB_CMD.STATE.name, "")
        # 트레이의 레벨을 정하는 필드지만 LAB TEST 에서는 사용하지 않는다
        LEVEL = recvDataMap.get(BLB_CMD.LEVEL.name, "")

        # INIT - 명령어
        # QBI 터치패널 특성상 UI가 완전히 로딩 된 후 터치패널 초기화 명령어를 내려야 한다.
        # QBI UI가 로딩되면 INIT 값을 가진 명령어를 보내고 이를 처리하는 부분
        # 처리후 함수 종료 return
        if STATE.find(BLB_CMD_STATUS.INIT.name) >= 0:
            SendStatus(BLB_STATUS_FIELD.INIT)
            node_CtlCenter_globals.timestamp_touchinit = datetime.datetime.now()
            node_CtlCenter_globals.bInit = True
            return

        elif STATE.find(BLB_CMD_STATUS.EVENT.name) >= 0:
            # QBI UI에서 보내는 다양한 버튼 이벤트들을 처리한다
            # 처리후 함수 종료 return
            strPad_local = ",0"
            rospy.loginfo(logmsg)

            # UI의 트레이 업이나 다운버튼을 떼면 Tray 높이 조절을 정지한다
            if (
                LEVEL.find(BLB_UI_EVENTS.up_released.name) >= 0
                or LEVEL.find(BLB_UI_EVENTS.down_released.name) >= 0
            ):
                TrayStop()
            # UI의 아래방향 버튼이 눌리면 트레이 다운
            elif LEVEL.find(BLB_UI_EVENTS.down_pressed.name) >= 0:
                TrayClose()
            # UI의 위방향 버튼이 눌리면 트레이 업
            elif LEVEL.find(BLB_UI_EVENTS.up_pressed.name) >= 0:
                TrayOpen()
            # UI의 리셋 버튼이 눌리면 트레이 높이 초기화
            elif LEVEL.find(BLB_UI_EVENTS.level_reset.name) >= 0:
                SendCMDArd("Y:3" + strPad_local)

            # UI의 A나 B버튼을 떼면 도어 동작 정지
            if (
                LEVEL.find(BLB_UI_EVENTS.A_released.name) >= 0
                or LEVEL.find(BLB_UI_EVENTS.B_released.name) >= 0
            ):
                DoorStop()
            # UI의 A버튼 누르면 도어 닫힘
            elif LEVEL.find(BLB_UI_EVENTS.A_pressed.name) >= 0:
                DoorClose()
            # UI의 B버튼 누르면 도어 열림
            elif LEVEL.find(BLB_UI_EVENTS.B_pressed.name) >= 0:
                DoorOpen()
            # if LEVEL.find(BLB_UI_EVENTS.down_pressed.name) >= 0:
            #   SendCMDArd('Y:2')
            return
        # EVENT 처리 명령부분 끝, 처리 후 함수 종료

        # 모든 명령어 필드가 다 있는 경우 (QBI 에서 보낼때는 모든 필드를 채워서 보낸다.)
        # (QBI UI 주방 페이지에서 테이블 2개 지정 후 동작시키는 경우)
        if CheckAllKeysExist(BLB_CMD, recvDataMap):
            ID = (str)(recvDataMap[BLB_CMD.ID.name])
            TRAY_A = (int)(recvDataMap[BLB_CMD.TRAY_A.name])
            TRAY_B = (int)(recvDataMap[BLB_CMD.TRAY_B.name])
            LEVEL = (int)(recvDataMap[BLB_CMD.LEVEL.name])
            TIME = (str)(recvDataMap[BLB_CMD.TIME.name])
            STATE = (str)(recvDataMap[BLB_CMD.STATE.name])
            rospy.loginfo(
                f"From {node_CtlCenter_globals.node_current} to {TRAY_A},"
                f" Cur_listTable:{node_CtlCenter_globals.listTable},"
                f"cmdIdx:{node_CtlCenter_globals.cmdIdx}"
            )
            # QBI UI 에서 확인 메세지를 누른 경우
            if STATE.find(BLB_CMD_STATUS.CONFIRM.name) >= 0:
                #리프트 하강 테스트 코드
                if not node_CtlCenter_globals.flag_WaitConfirm:
                  node_CtlCenter_globals.is_lifted = False
                  node_CtlCenter_globals.status_bal = STATUS_BALANCING.READY
                  node_CtlCenter_globals.listBLB = GetLiftControlDown()
                  node_CtlCenter_globals.cmdIdx = 0
                  node_CtlCenter_globals.flag_liftup = True
                  node_CtlCenter_globals.is_docked = False
                  node_CtlCenter_globals.is_lifted = True
                else:
                  node_CtlCenter_globals.flag_WaitConfirm = False
            # 범블비 이동명령어인 경우!
            elif STATE.find(BLB_CMD_STATUS.MOVE.name) >= 0:
                # flag_WaitConfirm = False
                # 글로벌 상태정보를 업데이트 하는 부분!
                # 스레드 락으로 아래 코드의 동시실행을 방지한다
                node_CtlCenter_globals.lock.acquire()

                # node_CtlCenter_globals.listTable 에 지정한 TRAY 둘다 없는 경우!
                if (
                    not TRAY_A in node_CtlCenter_globals.listTable
                    and not TRAY_B in node_CtlCenter_globals.listTable
                ):
                    # 1 다음 목적지가 충전소인 경우 못가게 막는다.
                    if (
                        len(node_CtlCenter_globals.listTable) > 0
                        and node_CtlCenter_globals.listTable[-1]
                        == node_CHARGING_STATION
                    ):
                        # 충전소 진행 명령어 삭제
                        node_CtlCenter_globals.listTable.pop()

                    # 2 현재 테이블이 TRAY_A 와 다르고 TRAY_A 번호가 유효한 경우
                    if (
                        TRAY_A != node_CtlCenter_globals.node_current
                        and TRAY_A != node_NOT_TABLE
                    ):
                        # 지시 테이블 리스트에 TRAY_A 테이블 값 추가
                        node_CtlCenter_globals.listTable.append(TRAY_A)

                    # 3 TRAY_A와 TRAY_B가 다르고 TRAY_B도 유효한 테이블이라면
                    # TRAY_A 이후 TRAY_B 이동지시를 추가한다
                    if TRAY_A != TRAY_B and TRAY_B != node_NOT_TABLE:
                        node_CtlCenter_globals.listTable.append(TRAY_B)

                    # 위 단계를 거치고 나면 범블비 운행 리스트 테이블이 생성되고 충전소를 마지막 이동경로로 추가
                    if len(node_CtlCenter_globals.listTable) > 0:
                        node_CtlCenter_globals.listTable.append(node_CHARGING_STATION)

                    # 현 위치가 부엌인 경우 flag_WaitConfirm 상태를 변경하여 트레이 도어
                    # 를 바로 닫는다.
                    if node_CtlCenter_globals.node_current == node_KITCHEN_STATION:
                        node_CtlCenter_globals.flag_WaitConfirm = False
                node_CtlCenter_globals.lock.release()
                # 스레드락 해제

                # 생성된 범블비 이동 경로 확인
                rospy.loginfo(node_CtlCenter_globals.listTable)
            # 모든 명령어 필드가 다 있는 경우 처리 끝, 모든 필드가 없는 경우에는 무시된다.
        # mbid_tmp = recvDataMap.get(MotorWMOVEParams.MBID.name, None)

        # if mbid_tmp != None:
        #     node_CtlCenter_globals.dic_485ex[mbid_tmp] = recvDataMap
        # # print(recvDataMap)
        # # TODO : 응답메세지 발행하는 것도 만들기.
        elif PROFILE != "":  # 사용자 커스텀 명령어 GetCustomFileControl 활용
            lsMotorOperation = []
            if isFileExist(PROFILE):
                lsMotorOperation = GetCustomFileControl(PROFILE)
            else:
                #문자열 자체로 들어오는 경우 2000`1000`90
                paramArmControl = PROFILE.split(sep=sDivItemComma)
                paramInt1 = int(paramArmControl[0])
                paramInt2 = int(paramArmControl[1])
                if len(paramArmControl) == 3:
                    lsMotorOperation = GetStrArmExtendMain(paramInt1,paramInt2,int(paramArmControl[2]))
                else:
                    if paramInt1 >0 and paramInt2>0:
                        cmdTmp = getMotorHomeString(ModbusID.BAL_ARM1.value)
                        SendCMD_Device(cmdTmp)
                        time.sleep(TOPIC_CMD_RATE)
                        cmdTmp = getMotorHomeString(ModbusID.BAL_ARM2.value)
                        SendCMD_Device(cmdTmp)

                    cmdTmp = getMotorMoveString(ModbusID.BAL_ARM1.value,True,GetDestPoint(paramInt1,ModbusID.BAL_ARM1),3,DEFAULT_ACC,DEFAULT_DECC)
                    time.sleep(TOPIC_CMD_RATE)
                    SendCMD_Device(cmdTmp)

                    cmdTmp = getMotorMoveString(ModbusID.BAL_ARM2.value,True,GetDestPoint(paramInt2,ModbusID.BAL_ARM2),DEFAULT_SPD_MID,DEFAULT_ACC,DEFAULT_DECC)
                    time.sleep(TOPIC_CMD_RATE)
                    SendCMD_Device(cmdTmp)
                    # lsArm = []
                    # lsArm.extend(GetDicBalArmRealTime(STROKE_BAL_EXTEND, 5))
                    # lsMotorOperation.append(lsArm)
                
            if len(lsMotorOperation) > 0:
                node_CtlCenter_globals.status_bal = STATUS_BALANCING.READY
                node_CtlCenter_globals.listBLB.clear()
                node_CtlCenter_globals.listBLB.extend(lsMotorOperation)
                node_CtlCenter_globals.cmdIdx = 0
        else:
            rospy.loginfo(recvDataMap)

    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        # SendFeedback(e)

def callbackModbus(data):
    """_summary_
        /MB_??, 즉 /MB_ 로 시작하는 모드버스 데이터를 받아 STATUS 데이터를 업데이트 한다
        node_CtlCenter_globals.dic_485ex
    Args:
        data: "ALM_CD:0,NA1:0,ST_CMD_FINISH:1,ST_HOME_FINISH:0,ST_FAULTY:0,ST_ENABLE:1,
        ST_RUNNING:1,CUR_SPD_RAW:2,CUR_TORQUE:0,CUR_CURRENT:0,CUR_SPD:0,BUS_VOLTAGE:24,
        BUS_TEMP:35,OVER_LOAD:0,CMD_POS:100000,CUR_POS:100000,DI_NOT:0,DI_POT:0,DI_HOME:0,
        DI_ESTOP:1,DI_02:0,DI_01:0,DO_02:3,DO_01:1,LASTSEEN:1707275473.4336,MBID:15"
    """
    #global lock
    try:
        recvData = data.data
        if node_CtlCenter_globals.runFromLaunch:
            # 필요한 경우 호출함수와 수신 데이터를 로그로 남긴다
            logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
            rospy.loginfo(logmsg)

        # KV 구분자 콜론, 아이템구분자 쉼표인 문자열을 dict 로 파싱한다
        recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)

        # 수신 메세지에는 무조건 MBID 필드가 있으며 없으면 무시된다.
        mbid = recvDataMap.get(MotorWMOVEParams.MBID.name, None)
        if mbid is None:
          return      

        # 모니터링 변수를 업데이트 한다        
        node_CtlCenter_globals.dic_485ex[mbid] = recvDataMap
        countTmp = node_CtlCenter_globals.dicModbusCallbackCount.get(mbid, 0)
        node_CtlCenter_globals.dicModbusCallbackCount[mbid] = countTmp+1
        if mbid == str(ModbusID.BAL_ARM1.value):
          node_CtlCenter_globals.dicModbusCallbackTimeStamp13[node_CtlCenter_globals.dicModbusCallbackCount[mbid]] = datetime.datetime.now()
        
        #밸런싱암 관련 명령어가 나간지 0.5초 경과하지 않으면 무시한다.
        if not isTimeExceeded(node_CtlCenter_globals.lastCmdBalanceStamp, MODBUS_EXECUTE_DELAY_ms):
            return
        
        recvDataMapSrvTele = node_CtlCenter_globals.dic_485ex.get(str(ModbusID.TELE_SERV_MAIN.value), None)
        recvDataMapBalTele = node_CtlCenter_globals.dic_485ex.get(str(ModbusID.TELE_BALANCE.value), None)
        recvDataMapArm1 = node_CtlCenter_globals.dic_485ex.get(str(ModbusID.BAL_ARM1.value), None)
        recvDataMapArm2 = node_CtlCenter_globals.dic_485ex.get(str(ModbusID.BAL_ARM2.value), None)
        if None in (recvDataMapSrvTele, recvDataMapBalTele, recvDataMapArm1, recvDataMapArm2,recvDataMap):
            return
          
        # isRunBalTele = isActivatedMotor(ModbusID.TELE_BALANCE.value)
        # isRunTelSrv = isActivatedMotor(ModbusID.TELE_SERV_MAIN.value)
        
        
        # pot_cur_arm1 = int(recvDataMapArm1.get(str(MonitoringField.POT_POS.name), -1))
        # pot_cur_srvTele,not_cur_srvTele = GetPotNotServo(ModbusID.TELE_SERV_MAIN)
        # pot_cur_balTele,not_cur_balTele = GetPotNotServo(ModbusID.TELE_BALANCE)

        #target_cur_arm1 = int(node_CtlCenter_globals.dicTargetPos.get(str(ModbusID.BAL_ARM1.value), MIN_INT))
        
        #pot_cur_arm2 = int(recvDataMapArm2.get(str(MonitoringField.POT_POS.name), -1))
        
        
        #cmd_spd_BalTele = int(recvDataMapBalTele.get(str(MonitoringField.CMD_SPD.name), -1))
        #1. 2관절의 주행 완료 시간 추론 (현재 RPM 과 POS 로 계산)
        #arm1 에 남은 거리 및 2관절 전개 완료 시간추론
        #time_togo_arm2 = calculate_rpm_time(round_togo_arm2,spd_cur_arm2)
        # isIncreaseSrvTele = True if spd_cur_srvTele >= 0 else False
        # isIncreaseBalTele = True if spd_cur_balTele >= 0 else False
        # isIncreaseArm1 = True if spd_cur_arm1 >= 0 else False
        # isIncreaseArm2 = True if spd_cur_arm2 >= 0 else False
        
        #밸런싱암 수축 완료된 후 밸런싱 암 컨트롤 시작.
        isFinishedMotorTele = isFinishedMotor(ModbusID.TELE_BALANCE)
        arm1GetMotorStatus = getMotorStatus(ModbusID.BAL_ARM1)
        arm2GetMotorStatus = getMotorStatus(ModbusID.BAL_ARM2)
        if isFinishedMotorTele and arm2GetMotorStatus == STATUS_MOTOR.PENDING_CCW and arm1GetMotorStatus == STATUS_MOTOR.PENDING_CCW and node_CtlCenter_globals.status_bal != STATUS_BALANCING.EXTEND_SPD_RAISED:
            target_cur_arm2 = int(node_CtlCenter_globals.dicTargetPos.get(str(ModbusID.BAL_ARM2.value), MIN_INT))
            target_cur_TeleSrv = int(node_CtlCenter_globals.dicTargetPos.get(str(ModbusID.TELE_SERV_MAIN.value), MIN_INT))
            #cur_pos_arm2 = max(0, int(recvDataMapArm2.get(str(MonitoringField.CUR_POS.name), MIN_INT)))
            #cur_pos_BalTele = max(0, int(recvDataMapBalTele.get(str(MonitoringField.CUR_POS.name), MIN_INT)))
            cur_pos_SrvTele = max(0, int(recvDataMapSrvTele.get(str(MonitoringField.CUR_POS.name), MIN_INT)))
            cur_pos_arm2 = max(0, int(recvDataMapArm2.get(str(MonitoringField.CUR_POS.name), MIN_INT)))
            pulse_togo_arm2 = target_cur_arm2 - cur_pos_arm2
            #pulse_togo_BalTele = target_cur_balTele - cur_pos_BalTele
            pulse_togo_SrvTele = target_cur_TeleSrv - cur_pos_SrvTele
            #if isRunARM1 and isRunARM2 and abs(spd_cur_arm2) < 10 and abs(spd_cur_arm1) < 10 and not isRunBalTele and node_CtlCenter_globals.status_bal != STATUS_BALANCING.EXTEND_SPD_RAISED:
            spd_cur_arm2 = int(recvDataMapArm2.get(str(MonitoringField.CUR_SPD.name),MIN_INT))
            spd_cur_arm1 = int(recvDataMapArm1.get(str(MonitoringField.CUR_SPD.name),MIN_INT))
            spd_cur_srvTele = int(recvDataMapSrvTele.get(str(MonitoringField.CUR_SPD.name),MIN_INT))
            node_CtlCenter_globals.status_bal = STATUS_BALANCING.EXTEND_SPD_RAISED
            round_togo_SrvTele = pulse_togo_SrvTele/roundPulse    
            time_togo_SrvTele = calculate_rpm_time(round_togo_SrvTele,spd_cur_srvTele)
            round_togo_arm2 = pulse_togo_arm2/roundPulse
            targetRPM_arm2 = calculate_targetRPM_fromtime(round_togo_arm2,time_togo_SrvTele)+1
            cmdSpdChange = getMotorSpeedString(ModbusID.BAL_ARM2.value,True, abs(targetRPM_arm2),DEFAULT_ZERO,DEFAULT_DECC)
            node_CtlCenter_globals.dicTargetRPM[str(ModbusID.BAL_ARM2.value)] = str(abs(targetRPM_arm2))
            node_CtlCenter_globals.lastSpdArm1TimeStamp = datetime.datetime.now()
            print(f'Spd Change Time Stamp{node_CtlCenter_globals.lastSpdArm1TimeStamp}:{node_CtlCenter_globals.dicModbusCallbackCount}')
            SendCMD_Device(cmdSpdChange)
            return
        # else:
        #   rospy.loginfo(f'isFinishedMotorTele:{isFinishedMotorTele},arm1GetMotorStatus:{arm1GetMotorStatus},arm2GetMotorStatus:{arm1GetMotorStatus}')          
        
        #밸런싱암 완료 후 밸런싱 텔레스코프 시작하기.
        isFinishedMotorBal1 = isFinishedMotor(ModbusID.BAL_ARM1)
        isFinishedMotorBal2 = isFinishedMotor(ModbusID.BAL_ARM2)
        teleBalaMotorStatus = getMotorStatus(ModbusID.TELE_BALANCE)
        if isFinishedMotorBal1 and isFinishedMotorBal2 and teleBalaMotorStatus == STATUS_MOTOR.PENDING_CW and node_CtlCenter_globals.status_bal != STATUS_BALANCING.EXTEND_FINISHED:
        #if isFinishedMotor(ModbusID.BAL_ARM1) and isFinishedMotor(ModbusID.BAL_ARM2) and mbid == str(ModbusID.TELE_BALANCE.value) and isRunBalTele and abs(spd_cur_balTele) <10  and node_CtlCenter_globals.status_bal != STATUS_BALANCING.EXTEND_FINISHED:
        #if abs(spd_cur_arm1) < 10 and abs(spd_cur_arm2) < 10 and mbid == str(ModbusID.TELE_BALANCE.value) and isRunBalTele and abs(spd_cur_balTele) <10  and node_CtlCenter_globals.status_bal != STATUS_BALANCING.EXTEND_FINISHED:
            #pulse_togo_arm1 = target_cur_arm1 - cur_pos_arm1
            target_cur_arm2 = int(node_CtlCenter_globals.dicTargetPos.get(str(ModbusID.BAL_ARM2.value), MIN_INT))
            target_cur_balTele = int(node_CtlCenter_globals.dicTargetPos.get(str(ModbusID.TELE_BALANCE.value), MIN_INT))
            target_cur_TeleSrv = int(node_CtlCenter_globals.dicTargetPos.get(str(ModbusID.TELE_SERV_MAIN.value), MIN_INT))
            cur_pos_arm2 = max(0, int(recvDataMapArm2.get(str(MonitoringField.CUR_POS.name), MIN_INT)))
            cur_pos_BalTele = max(0, int(recvDataMapBalTele.get(str(MonitoringField.CUR_POS.name), MIN_INT)))
            cur_pos_SrvTele = max(0, int(recvDataMapSrvTele.get(str(MonitoringField.CUR_POS.name), MIN_INT)))
            pulse_togo_arm2 = target_cur_arm2 - cur_pos_arm2
            pulse_togo_BalTele = target_cur_balTele - cur_pos_BalTele
            pulse_togo_SrvTele = target_cur_TeleSrv - cur_pos_SrvTele
            #round_togo_arm1 = pulse_togo_arm1/roundPulse
            round_togo_arm2 = pulse_togo_arm2/roundPulse
            round_togo_BalTele = pulse_togo_BalTele/roundPulse
            round_togo_SrvTele = pulse_togo_SrvTele/roundPulse                    
            spd_cur_srvTele = int(recvDataMapSrvTele.get(str(MonitoringField.CUR_SPD.name),MIN_INT))
            time_togo_SrvTele = calculate_rpm_time(round_togo_SrvTele,spd_cur_srvTele)
            targetRPM_telBal = int(1.1*calculate_targetRPM_fromtime(round_togo_BalTele,time_togo_SrvTele)+1)
            cmdSpdChange2 = getMotorSpeedString(ModbusID.TELE_BALANCE.value,True, targetRPM_telBal,DEFAULT_ZERO,DEFAULT_DECC)
            node_CtlCenter_globals.status_bal = STATUS_BALANCING.EXTEND_FINISHED
            SendCMD_Device(cmdSpdChange2)
            spd_cur_arm2 = int(recvDataMapArm2.get(str(MonitoringField.CUR_SPD.name),MIN_INT))
            spd_cur_arm1 = int(recvDataMapArm1.get(str(MonitoringField.CUR_SPD.name),MIN_INT))
            rospy.loginfo(f'현재암 속도:{spd_cur_arm1},{spd_cur_arm2}, 밸런싱암전개 목표시간:{time_togo_SrvTele:.1f},밸런싱암전개RPM:{targetRPM_telBal}')
            return
            
        isRunARM1 = isActivatedMotor(ModbusID.BAL_ARM1.value)
        isRunARM2 = isActivatedMotor(ModbusID.BAL_ARM2.value)
        spd_cur_balTele = int(recvDataMapBalTele.get(str(MonitoringField.CUR_SPD.name),MIN_INT))
        
        # if mbid == str(ModbusID.BAL_ARM1.value):
        #     print(f'Time Stamp:{node_CtlCenter_globals.dicModbusCallbackCount[mbid]},spd_cur_balTele:{spd_cur_balTele},isRunARM1:{isRunARM1},isRunARM2:{isRunARM2},status:{node_CtlCenter_globals.status_bal}')
        
        #1,2관절 수평 맞추기
        if mbid == str(ModbusID.BAL_ARM1.value) and (getMotorStatus(ModbusID.TELE_BALANCE) == STATUS_MOTOR.PENDING_CCW or getMotorStatus(ModbusID.TELE_BALANCE) == STATUS_MOTOR.STOPPED or getMotorStatus(ModbusID.TELE_BALANCE) == STATUS_MOTOR.PENDING_CW) and (isRunARM1 or isRunARM2) and (node_CtlCenter_globals.status_bal != STATUS_BALANCING.EXTEND_FINISHED or node_CtlCenter_globals.status_bal != STATUS_BALANCING.SHRINK_FINISHED): #cur_pos_arm1 > 100 and cur_pos_arm2 > 100 and 
        #if abs(spd_cur_balTele) <10 and mbid == str(ModbusID.BAL_ARM1.value) and (isRunARM1 or isRunARM2): #cur_pos_arm1 > 100 and cur_pos_arm2 > 100 and 
        #if td_local.total_seconds() >= 1 and (isRunARM1 or isRunARM2) and node_CtlCenter_globals.status_bal != STATUS_BALANCING.EXTEND_FINISHED:
            #rospy.loginfo(f'Lock Start:{datetime.datetime.now()}')
            lock.acquire()
            td_local = datetime.datetime.now() - node_CtlCenter_globals.lastSpdArm1TimeStamp
            lock.release()
            #rospy.loginfo(f'Lock End:{datetime.datetime.now()}, td_local : {td_local}')
            if td_local.total_seconds() < PI_TIME_RATE:
              #rospy.loginfo(f'Waiting for time rate:{node_CtlCenter_globals.dicModbusCallbackCount[mbid]}')
              return
              
            # if not isRunARM2:
            #   cmdSpdChange = getMotorSpeedString(ModbusID.BAL_ARM1.value,True,DEFAULT_SPD_FAST,0,DEFAULT_DECC)
            #   SendCMD_Device(cmdSpdChange)
            #   node_CtlCenter_globals.lastSpdArm1TimeStamp = datetime.datetime.now()
            #   return
            spd_cur_arm2 = int(recvDataMapArm2.get(str(MonitoringField.CUR_SPD.name),MIN_INT))
            if abs(spd_cur_arm2) <= (DEFAULT_SPD_MID * 0.2) and not isFinishedMotor(ModbusID.BAL_ARM2):
              rospy.loginfo(f'Spd2 is low- {spd_cur_arm2} / Time Stamp:{node_CtlCenter_globals.dicModbusCallbackCount[mbid]}')
              return
            
            rospy.loginfo(f'Time:{td_local.total_seconds():.2f},Modbus {mbid} Time Stamp:{node_CtlCenter_globals.dicModbusCallbackCount[mbid]}')
            spd_cur_arm2_target = int(node_CtlCenter_globals.dicTargetRPM.get(str(mbid), MIN_INT))
            if spd_cur_arm2_target == MIN_INT:
              rospy.loginfo(f'Err : data not found in TargetPOS MBID:{mbid}')
              spd_cur_arm2_target = spd_cur_arm2
            
            cur_pos_arm1 = max(0, int(recvDataMapArm1.get(str(MonitoringField.CUR_POS.name), -1)))
            pot_cur_arm1,not_cur_arm1 = GetPotNotServo(ModbusID.BAL_ARM1)
            pot_cur_arm2,not_cur_arm2 = GetPotNotServo(ModbusID.BAL_ARM2)
            cur_pos_arm2 = max(0, int(recvDataMapArm2.get(str(MonitoringField.CUR_POS.name), MIN_INT)))
            spd_cur_arm1 = int(recvDataMapArm1.get(str(MonitoringField.CUR_SPD.name),MIN_INT))
            new_rpm = calculate_speed_adjustmentExp(cur_pos_arm1, cur_pos_arm2, not_cur_arm1, pot_cur_arm1, not_cur_arm2, pot_cur_arm2, spd_cur_arm1, spd_cur_arm2_target, ARM_EXP_RATE, PI_TIME_RATE, PULSES_PER_ROUND)
            # if abs(spd_cur_arm1) > 100:
            #   new_rpm2 = adjust_rpm(new_rpm,spd_cur_arm1,0.5)
            #   if new_rpm != new_rpm2:
            #     print(f'변동폭이 심하지 않게 새롭게 조정된 RPM 값 : {new_rpm}->{new_rpm2}')
            #     new_rpm = new_rpm2
            #isIncreaseArm1newRPM = True if new_rpm > 0 else False
            
            # if (isIncreaseArm1 or isIncreaseArm2) and not isIncreaseArm1newRPM:
            #     print(f'chk1-RPM1:{spd_cur_arm1},RPM2:{spd_cur_arm2},NEWRPM:{new_rpm}')
            #     new_rpm = 1
            # if (not isIncreaseArm1 or not isIncreaseArm2) and isIncreaseArm1newRPM:
            #     print(f'chk2-RPM1:{spd_cur_arm1},RPM2:{spd_cur_arm2},NEWRPM:{new_rpm}')
            #     new_rpm = 1
                
            # if isIncreaseArm1newRPM and isIncreaseArm1:
            #     print(f'속도변경 : {spd_cur_arm1} -> {new_rpm}')
            # else:
            #     new_rpm = 0
            print(f'속도변경 : {spd_cur_arm1} -> {new_rpm}')
            cmdSpdChange = getMotorSpeedString(ModbusID.BAL_ARM1.value,True,new_rpm,DEFAULT_ZERO,DEFAULT_DECC)
            SendCMD_Device(cmdSpdChange)
            node_CtlCenter_globals.lastSpdArm1TimeStamp = datetime.datetime.now()
            cur_timestamp = node_CtlCenter_globals.dicModbusCallbackTimeStamp13[node_CtlCenter_globals.dicModbusCallbackCount['13']]
            prev_timestamp = node_CtlCenter_globals.dicModbusCallbackTimeStamp13[node_CtlCenter_globals.dicModbusCallbackCount['13']-1]
            td_tmp = cur_timestamp - prev_timestamp
            ts = node_CtlCenter_globals.dicModbusCallbackCount['13']
            print(f'timestamp:{ts},td_tmp:{td_tmp.total_seconds():.2f},prev_timestamp:{prev_timestamp},cur_timestamp:{cur_timestamp}')
            
            #node_CtlCenter_globals.status_bal = STATUS_BALANCING.EXTEND_SPD_RAISED
            
        # if (isRunARM1 or isRunARM2 or isRunBalTele or isRunTelSrv) and node_CtlCenter_globals.status_bal != STATUS_BALANCING.EXTEND_FINISHED:
        #     target_tmp_arm1 = mapRangeExp(cur_pos_arm2,not_cur_arm2,pot_cur_arm2,not_cur_arm1,pot_cur_arm1,1)
        #     target_nextSec_arm2 = cur_pos_arm2+(spd_cur_arm2*PULSES_PER_ROUND/60)
        #     target_nextSec_arm1 = cur_pos_arm1+(spd_cur_arm1*PULSES_PER_ROUND/60)
            
        #     if cur_pos_arm2 >= int(pot_cur_arm2) / 2 and node_CtlCenter_globals.status_bal != STATUS_BALANCING.EXTEND_SPD_RAISED:
        #         print()


        # #밸런싱암 전개 부분 테스트 코드 시작
        # #if isRunARM2 and mbid == str(ModbusID.BAL_ARM2.value):
        # if (isRunARM1 or isRunARM2 or isRunBalTele or isRunTelSrv) and node_CtlCenter_globals.status_bal != STATUS_BALANCING.EXTEND_FINISHED:
        #     if cur_pos_arm2 >= int(pot_cur_arm2) / 2 and node_CtlCenter_globals.status_bal != STATUS_BALANCING.EXTEND_SPD_RAISED:
        #       # 밸런스 상태 플래그 변경
        #       node_CtlCenter_globals.status_bal = STATUS_BALANCING.EXTEND_SPD_RAISED
        #       targetRPM_arm1 = calculate_targetRPM_fromtime(round_togo_arm1,time_togo_arm2)
        #       cmdSpdChange = getMotorSpeedString(ModbusID.BAL_ARM1.value,True, targetRPM_arm1,500,500)
        #       rospy.loginfo(f'1관절속도:{cmdSpdChange},남은시간:{time_togo_arm2},남은바퀴수:{round_togo_arm1}')
        #       pub_cmdDevice.publish(cmdSpdChange)
        #     elif not isRunARM2 and cmd_spd_BalTele < 10:              
        #       if target_cur_balTele > 0 and node_CtlCenter_globals.status_bal != STATUS_BALANCING.EXTEND_FINISHED:
        #         node_CtlCenter_globals.status_bal = STATUS_BALANCING.EXTEND_FINISHED
        #         time_togo_SrvTele = calculate_rpm_time(round_togo_SrvTele,spd_cur_srvTele)
        #         targetRPM_telBal = calculate_targetRPM_fromtime(round_togo_BalTele,time_togo_SrvTele)+1
        #         cmdSpdChange = getMotorSpeedString(ModbusID.TELE_BALANCE.value,True, targetRPM_telBal,DEFAULT_ACC,DEFAULT_DECC)
        #         pub_cmdDevice.publish(cmdSpdChange)
        #         rospy.loginfo(f'밸런싱암전개 목표시간:{time_togo_SrvTele},밸런싱암전개RPM:{targetRPM_telBal}')
    except Exception as e:
        message = traceback.format_exc()
        print(f'ERR callbackModbus:{message}')
        #prtMsg(message)
        # SendFeedback(e)


def callbackARD_CARRIER(data):
    """_summary_
        아두이노에서 ARD_CARRIER 토픽으로 JSON 메세지를 발행한걸 수신하여 관련 전역변수에 업데이트 한다.
    Args: data : 
        "{\"\\nI_TRAY_2_BOTTOM\": \"0\", \"I_TRAY_2_HOME\": \"0\", \"I_TRAY_2_TOP\": \"0\"\
        , \"I_DOOR_1_BOTTOM\": \"0\", \"I_DOOR_1_HOME\": \"0\", \"I_DOOR_1_TOP\": \"0\"\
        , \"I_LIMIT_BOTTOM\": \"0\", \"O_V12_NC\": \"0\", \"O_V5_NC\": \"0\", \"LOAD\":\
        \ \"0.73\", \"GOR_SVN\": \"148.71,-25.84,0.00\", \"GAV_SVN\": \"-0.00,0.06,-0.02\"\
        , \"GLA_SVN\": \"5.38,5.77,-9.50\", \"GAV_TUN\": \"0.08\", \"TEMPN\": \"33.04\"}"

    """

    try:
        recvData = data.data
        recvDataMap = json.loads(recvData)
        node_CtlCenter_globals.dicARD_CARRIER.update(recvDataMap)
        # 배포모드로 실행시 로깅한다.
        if node_CtlCenter_globals.runFromLaunch:
            logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
            rospy.loginfo(logmsg)
    except Exception as e:
        message = traceback.format_exc()
        rospy.logdebug(message)
        # SendFeedback(e)


def callback_ARUCO_RESULT(data):
    """_summary_
        트레이에서 ARUCO_RESULT 토픽으로 JSON 메세지를 발행한걸 수신하여 관련 전역변수에 업데이트 한다.
    Args: 
    data: "{\"DIFF_X\": 16.59, \"DIFF_Y\": 23.56, \"ANGLE\": 162.89, \"CAM_ID\": 0, \"MARKER_VALUE\"\
  : 10, \"X\": 0.2423, \"Y\": 0.2155, \"Z\": 3.3798}"
    """
    try:
        recvData = data.data
        recvDataMap = json.loads(recvData)
        OnScanMarker = recvDataMap.get(ARUCO_RESULT_FIELD.IS_MARKERSCAN.name, None)
        if OnScanMarker is not None:
            node_CtlCenter_globals.bOnScanMarker = OnScanMarker

        node_CtlCenter_globals.dicARUCO_Result.update(recvDataMap)
        # 배포모드로 실행시 로깅한다.
        # if not node_CtlCenter_globals.runFromLaunch:
        #     logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
        #     rospy.loginfo(logmsg)
    except Exception as e:
        message = traceback.format_exc()
        rospy.logdebug(message)
        # SendFeedback(e)


def getLoadWeight():
    """현재까지 수신된 가장 마지막 로드셀 값

    Returns:
        _type_: _description_
    """
    strWeightValue = node_CtlCenter_globals.dicARD_CARRIER.get(
        CARRIER_STATUS.LOAD.name, "-1"
    )
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
    AutoStop = False
    try:
        recvData = data.data
        recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        sEPCKey = "EPC"
        sEPC = getValueFromMap(recvDataMap, sEPCKey)
        rospy.loginfo(sEPC)
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
                if AutoStop and sEPC is not None:
                    rospy.loginfo(f"Stop by {sEPC}")
                    motorStop(drvCaption_H, True)
            # SendFeedback(recvData)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        # print (e)
        # SendFeedback(e)


def gotoNode(deviceID, nodeID, bTTS):
    """지정한 범블비를 nodeID 번호를 가진 테이블로 이동시키는 명령어를
    생성하고 명령어처리 함수를 호출한다.

    Args:
        deviceID (_type_): 범블비 ID
        nodeID (_type_): 테이블 번호 (1~89)
        bTTS (_type_): 음성방송 여부
    """
    data_out = {}
    data_out[BLB_CMD.ID.name] = deviceID
    data_out[BLB_CMD.LEVEL.name] = -3

    # 개발 모드일때는 TTS를 끈다.
    bTTS = runFromLaunch

    # nodeID 가 달랑 숫자만 있는 경우는 웹페이지에서 호출한 경우 (HA,호출벨 등)
    if if_Number(nodeID):
        node_int = int(nodeID)
        # TRAY_A (첫번째로 방문할 테이블 번호) - 에 nodeID 세팅
        data_out[BLB_CMD.TRAY_A.name] = nodeID
        # 이동명령어 세팅
        data_out[BLB_CMD.STATE.name] = BLB_CMD_STATUS.MOVE.name
    else:
        # 테이블 번호가 아닌 다른 데이터 (어떤 경우인지 추후 확인하자)
        data_out[BLB_CMD.STATE.name] = nodeID
        data_out[BLB_CMD.TRAY_A.name] = 0

    data_out[BLB_CMD.TRAY_B.name] = 0
    data_out[BLB_CMD.TIME.name] = getCurrentTime(spliter="")

    # timestamp 추가후 callbackBLB_CMD 함수의 파라미터로 JSON 데이터를 넘긴다
    sendbuf = json.dumps(data_out)
    callData = String()
    callData.data = sendbuf
    callbackBLB_CMD(callData)


def callbackTopic2mqtt(data):
    """RECEIVE_MQTT 에 수신된 메세지를 파싱하여 범블비를 제어한다
    node_MQTT 노드에서 MQTT 를 수신하여 RECEIVE_MQTT 로 발행하고 그 메세지를 여기서 수신하여 제어함

    Args: 
    웹에서 수신 
    data: "{\"TOPIC\": \"BLB/CALL\", \"PAYLOAD\": \"98\"}"
    
    main 루틴에서 수신 (크로스 브릿지 및 리프트 아두이노 참고용)
    data: "{\"TOPIC\": \"BLB/CROSS_CMD/set\", \"PAYLOAD\": \"{\\\"99\\\": -1, \\\"2\\\": -1,\
    \ \\\"1\\\": -1}\"}"
    """
    try:
        recvData = data.data
        if is_json(recvData):
            recvDataMap = json.loads(recvData)
        else:
            recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        # 배포모드로 실행시 로깅한다.
        if node_CtlCenter_globals.runFromLaunch:
            logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
            rospy.loginfo(logmsg)
        # logmsg = f'{recvData} from {sys._getframe(0).f_code.co_name}'

        # 수신된 MQTT 메세지는 TOPIC 과 PAYLOAD 로 이루어져있다.
        # topic 값과 payload 값을 파싱한다.
        sTOPIC_MQTT = recvDataMap.get(MQTT_FIELD.TOPIC.name, None)
        sPAYLOAD = recvDataMap.get(MQTT_FIELD.PAYLOAD.name, None)

        # 유효하지 않은 메세지면 함수 종료
        if sPAYLOAD == None or sTOPIC_MQTT == None:
            return

        data_out = {}
        # MQTT 토픽별 처리루틴
        if is_json(sPAYLOAD):
            # 그외의 경우에
            data_out = json.loads(sPAYLOAD)
            data_out[CALLBELL_FIELD.TIMESTAMP.name] = getCurrentTime(spliter="")
            nodeID = sTOPIC_MQTT.split("_")[-1]
            if sTOPIC_MQTT.find(CALLBELL_FIELD.CALLBELL.name) > 0:
                #'BLB/mcu_relay_100' : 교차로에서 발행하는 메세지
                #'BLB/mcu_relay_CALLBELL_1' : 호출벨에서 온 메세지인 경우 토픽명에 CALLBELL 이란 단어가 있음.
                if data_out.get(CALLBELL_FIELD.BTN_RED.value, None) == 1:
                    if node_CtlCenter_globals.runFromLaunch:
                        SendMsgToMQTT(
                            pub_topic2mqtt,
                            MQTT_TOPIC_VALUE.TTS.value,
                            f"{GetKoreanFromNumber(nodeID)}번 테이블에서 직원을 호출하셨습니다.",
                        )
                        # SendMsgToMQTT(mqtt_topic_TTS,f'{nodeID}번 테이블에서 직원을 호출하셨습니다.')
                elif data_out.get(CALLBELL_FIELD.BTN_BLUE.value, None) == 1:
                    if data_out.get(CALLBELL_FIELD.CALL_STATE.value, None) == 1:
                        if node_CtlCenter_globals.runFromLaunch:
                            SendMsgToMQTT(
                                pub_topic2mqtt,
                                MQTT_TOPIC_VALUE.TTS.value,
                                f"테이블 {GetKoreanFromNumber(nodeID)}번에서 로봇 호출을 취소하셨습니다.",
                            )
                    else:
                        # SendMsgToMQTT(f'테에이블 {nodeID}번 로봇 호출입니다.')
                        gotoNode(device_ID, nodeID, True)

            # print(type(data_out))
            # print(type(stateDic))
            else:
                node_CtlCenter_globals.stateDic[nodeID] = data_out
            # SendMsgToMQTT() publish (data_out,sTOPIC_MQTT)
            # rospy.loginfo(stateDic)
        else:
            rospy.loginfo(sPAYLOAD)
    except Exception as e:
        rospy.loginfo(traceback.format_exc())

for item in ModbusID:
    subTopicName = f'{TopicName.MB_.name}{item.value}'
    rospy.Subscriber(subTopicName, String, callbackModbus, queue_size=ROS_TOPIC_QUEUE_SIZE)
    node_CtlCenter_globals.numSubTopics += 1
    rospy.loginfo(f'모드버스 구독번호 {node_CtlCenter_globals.numSubTopics}:{subTopicName}')
rospy.Subscriber(TopicName.RFID.name, String, callbackRFID, queue_size=ROS_TOPIC_QUEUE_SIZE)
rospy.Subscriber(TopicName.ARD_CARRIER.name, String, callbackARD_CARRIER, queue_size=ROS_TOPIC_QUEUE_SIZE)
rospy.Subscriber(TopicName.ACK.name, String, callbackAck, queue_size=ROS_TOPIC_QUEUE_SIZE)
rospy.Subscriber(TopicName.BLB_CMD.name, String, callbackBLB_CMD, queue_size=ROS_TOPIC_QUEUE_SIZE)  # UI에서 명령어 수신
rospy.Subscriber(TopicName.RECEIVE_MQTT.name, String, callbackTopic2mqtt, queue_size=ROS_TOPIC_QUEUE_SIZE)
rospy.Subscriber(TopicName.ARUCO_RESULT.name, String, callback_ARUCO_RESULT, queue_size=ROS_TOPIC_QUEUE_SIZE)
