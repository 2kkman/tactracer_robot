#!/usr/bin/env python3
from platform import node
from node_CtlCenter_services import *
#from tactracer_robot.bumblebee.scripts.node_ModbusIF import stopMotor
last_execution_time = 0  # 마지막 실행 시간을 저장할 전역 변수
def get_mpv_process_info():
    mpv_procs = [p for p in psutil.process_iter(['name', 'cpu_percent', 'memory_percent']) if 'mpv' in p.info['name']]
    
    total_cpu = sum(p.info['cpu_percent'] for p in mpv_procs)
    total_mem = sum(p.info['memory_percent'] for p in mpv_procs)
    proc_count = len(mpv_procs)

    print(f"mpv 프로세스 수: {proc_count}")
    print(f"총 CPU 사용량: {total_cpu:.2f}%")
    print(f"총 메모리 사용량: {total_mem:.2f}%")
    
def CheckActivateMotorTorque():
    global last_execution_time  # 전역 변수 사용
    current_time = time.time()  # 현재 시간 (초 단위)

    # 마지막 실행 시간으로부터 n초 이내라면 실행하지 않음
    if current_time - last_execution_time < 3:
        return

    last_execution_time = current_time  # 실행 시간 업데이트
      
    for item in ModbusID:
      if not isISVMotor(item.value):
        continue
      target = int(node_CtlCenter_globals.dicTargetPos.get(str(item.value), MIN_INT))
      laststart = datetime.fromtimestamp(try_parse_float(GetItemsFromModbusTable(item, MonitoringField.LASTSTART)))
      motor_toque = int(GetItemsFromModbusTable(item, MonitoringField.CUR_TORQUE))
      if not isActivatedMotor(item.value) or target == MIN_INT or laststart == DATETIME_OLD:
        continue
      cmd_pos,cur_pos = GetPosServo(item)
      if abs(target - cur_pos) < roundPulse*10:
        continue
      finishTime_local = (getDateTime() - laststart).total_seconds()  # 경과 시간 (초 단위)
      if finishTime_local < 3:
        continue
      rospy.loginfo(f'MBID:{item.value},TOQ:{motor_toque}')
      if motor_toque < 30:
        StopEmergency(ALM_User.MOTOR_TORQUE_INVALID.value)

def SaveTableNodeInfo():
    for linkKey,lsArucoLink in node_CtlCenter_globals.ScanInfo.items():
        #ScanMode 에서 주행중 아르코 마커가 발견된경우!
        filtered_dict = node_CtlCenter_globals.dfLinkPosInfo[node_CtlCenter_globals.dfLinkPosInfo[TableInfo.LINK_ID.name].astype(str) == str(linkKey)].to_dict(orient="records")
        if len(lsArucoLink) > 0 and len(filtered_dict) > 0:
            dicLinkPos = filtered_dict[0]
            started_node_mm = pulseH_to_distance(dicLinkPos[SeqMapField.START_NODE.name])
            #노드 정보 업데이트 (새로 발견된 아르코마커 정보를 노드에 추가)
            #노드간 길이가 10cm 이내인 경우 같은 노드로 통합        
            dictArucoAll = groupFromList(lsArucoLink,ARUCO_RESULT_FIELD.MARKER_VALUE.name)
            avg_values = calculate_average_values(dictArucoAll)
            lsNodeScannedmm = sorted(avg_values.values(), key=lambda x: x[BLB_LOCATION.DISTANCE_FROM_HOME.name])
            
            #Z가 너무 크면 아르코 마커 에러로 간주
            lsNodeScannedFiltered =  [item for item in lsNodeScannedmm if item[ARUCO_RESULT_FIELD.Z.name] < 3000]   
            if len(lsNodeScannedFiltered) > 0:
                #listNode = [(23, 1000), (24, 1500)] 이런 형태로 세팅해야 함.
                start_number = max(node_CtlCenter_globals.graph.keys())+1
                listNode = []
                listTables = []
                maxY = 2000
                for i in range(len(lsNodeScannedFiltered)):
                    dicTableInfoCur = lsNodeScannedFiltered[i]
                    diff_y = dicTableInfoCur[ARUCO_RESULT_FIELD.DIFF_Y.name]
                    table_id = dicTableInfoCur[ARUCO_RESULT_FIELD.MARKER_VALUE.name]
                    cur_pos_i = round(dicTableInfoCur[MonitoringField.CUR_POS.name])
                    serving_distance = mapRange(diff_y,-100,100,200,maxY)
                    node_id = start_number+i
                    serving_angle = dicTableInfoCur[BLB_LOCATION.ANGLE_360.name]
                    
                    #ShortCut (그래프) 업데이트 데이터 생성
                    listNode.append( (node_id, pulseH_to_distance(cur_pos_i)) )
                    # 1. listNode를 절대 위치 값으로 정렬
                    listNode.sort(key=lambda x: x[1])
                    
                    #테이블 정보 생성
                    dicTmpTable = {TableInfo.TABLE_ID.name : round(table_id),
                                TableInfo.NODE_ID.name : node_id,
                                TableInfo.SERVING_DISTANCE.name : round(serving_distance),
                                TableInfo.SERVING_ANGLE.name :(round(serving_angle)+360)%360,
                                TableInfo.MARKER_ANGLE.name : -1   
                                }
                    listTables.append(dicTmpTable)
                
                #테이블 정보 덮어쓰기. (갱신하는건 나중에)
                if len(listTables) > 0:
                    # 기존 파일 읽기
                    try:
                        df_existing = pd.read_csv(strFileTableNodeEx, sep=sDivTab)
                    except FileNotFoundError:
                        # 파일이 없으면 빈 DataFrame 생성
                        df_existing = pd.DataFrame()                    
                    df_unique = pd.DataFrame(listTables)
                    df_unique[df_unique.select_dtypes(include='int').columns] = df_unique.select_dtypes(include='int').astype(int)
                    # 기존 데이터와 새로운 데이터 병합 (TABLE_ID 기준)
                    if not df_existing.empty:
                        # TABLE_ID를 기준으로 중복된 값을 업데이트하고 새로운 레코드 추가
                        df_existing = pd.merge(
                            df_existing,
                            df_unique,
                            on=TableInfo.TABLE_ID.name,
                            how='outer',
                            suffixes=('_old', ''),
                            indicator=True
                        )

                        # 업데이트된 값이 있는 경우 최신 값으로 덮어쓰기
                        for column in df_unique.columns:
                            if column !=TableInfo.TABLE_ID.name:
                                df_existing[column] = df_existing.apply(
                                    lambda row: row[column] if row['_merge'] == 'right_only' else row[f"{column}_old"],
                                    axis=1
                                )

                        # 필요 없는 "_old" 컬럼과 "_merge" 컬럼 삭제
                        df_existing = df_existing[df_unique.columns]

                    else:
                        # 기존 파일이 없으면 새로운 데이터가 그대로 저장됨
                        df_existing = df_unique

                    # 병합된 데이터 저장
                    df_existing = df_existing.astype(int)
                    sorted_df = df_existing.sort_values(by=TableInfo.TABLE_ID.name, ascending=True)
                    sorted_df.to_csv(strFileTableNodeEx, index=False, sep=sDivTab)
                    #df_unique.to_csv(node_CtlCenter_globals.strFileTableNodeEx, index=False, sep=sDivTab) 
                
                g = Graph(node_CtlCenter_globals.graph)
                start_node = int(linkKey[0])
                end_node = int(linkKey[1:])
                dicChanges = g.add_node_between(start_node, end_node, listNode)
                for node, lsChanges in dicChanges.items():
                    if node in node_CtlCenter_globals.StateInfo.keys():
                        old_config = lsChanges[1][0]
                        new_config = lsChanges[0][0]
                        update_node_config(node_CtlCenter_globals.strFileCross, node,old_config, new_config )
                #old_config, new_config=g.add_node_between(start_node, end_node, listNode)
                
                # #교차로 연결 노드 업데이트
                # if start_node in node_CtlCenter_globals.StateInfo.keys():
                #     update_node_config(node_CtlCenter_globals.strFileCross, start_node,old_config, new_config )
                # else:
                #     update_node_config(node_CtlCenter_globals.strFileCross, end_node,old_config, new_config )
                #updated_graph = g.getGraph()
                #파일로 저장 후 ReLoad
                g.to_text_file(strFileShortCut)
                LoadMapRefresh()
   
def CheckModbusTimeOut(timeDelta=5):
    # 현재 시각
    now = getDateTime()
    # 5초 이상 지난 아이템들을 저장할 딕셔너리
    old_items = {}
    
    # 딕셔너리의 복사본을 만들어 순회
    items = dict(node_CtlCenter_globals.dicTopicCallbackTimeStamp)
    
    # 5초 이상 지난 아이템들을 찾기
    for mbid, timestamp in items.items():
        if (now - timestamp).total_seconds() > timeDelta:
            old_items[mbid] = timestamp
    
    return old_items

def InitCali():
    """
    범블비 실행 후 전개되어있던 암 모터를 수축하고 트레이를 끌어올리는 명령어 송신
    """
    if not node_CtlCenter_globals.bInitOK:
        if (
            len(node_CtlCenter_globals.dic_485ex)
            == node_CtlCenter_globals.numSubTopics - 2
        ):
            callData = String()
            callData.data = f"{BLB_CMD_CUSTOM.PROFILE.name}:0,0,0"
            callbackBLB_CMD(callData)
            rospy.loginfo(f"Motor Init : {callData}")
            node_CtlCenter_globals.bInitOK = True
        else:
            rospy.loginfo(f"Suspending Motor Init.")

lastUpdatePosStamp = getDateTime()

# def ArucoMarkerCheck():
#     #돌고 있는 모터가 없으면 종료
#     if has_common_element(getRunningMotorsBLB(),list_ArmControlMotors):
#         return

#     global lastUpdatePosStamp
#     lsAruco = node_CtlCenter_globals.lsARUCO_Result.copy()
#     node_CtlCenter_globals.lsARUCO_Result.clear()
#     if len(lsAruco) == 0:
#         return
#     #{"DIFF_X": 3.46, "DIFF_Y": 11.37, "ANGLE": -0.79, "CAM_ID": 2, "MARKER_VALUE": 1, "X": 0.0729, "Y": 0.1439, "Z": 4.3288}
#     dicArucoCur = lsAruco[-1]
#     updatePos = False
#     if len(node_CtlCenter_globals.lsARUCO_History) > 0:
#         dicArucoPrev = node_CtlCenter_globals.lsARUCO_History[-1]
#         updatePos = compare_dicts(dicArucoCur,dicArucoPrev,10)
#     else:
#         updatePos = True

#     if updatePos and isTimeExceeded(lastUpdatePosStamp, 1000):
#         node_CtlCenter_globals.lsARUCO_History.append(dicArucoCur)
#         lastUpdatePosStamp = getDateTime()
#         SendCMD_Device(GetStrArmExtendMain(dicArucoCur[ARUCO_RESULT_FIELD.X.name],dicArucoCur[ARUCO_RESULT_FIELD.Y.name],dicArucoCur[ARUCO_RESULT_FIELD.ANGLE.name]))
#         return

def GetDynamicMotorsInfo():
    recvDataMapH = node_CtlCenter_globals.dic_485ex.get(str(ModbusID.MOTOR_H.value), None)
    recvDataMapLift = node_CtlCenter_globals.dic_485ex.get(str(ModbusID.MOTOR_V.value), None)
    recvDataMapSrvTele = node_CtlCenter_globals.dic_485ex.get(str(ModbusID.TELE_SERV_MAIN.value), None)
    #recvDataMapBalTele = node_CtlCenter_globals.dic_485ex.get(str(ModbusID.TELE_BALANCE.value), None)
    recvDataMapArm1 = node_CtlCenter_globals.dic_485ex.get(str(ModbusID.BAL_ARM1.value), None)
    recvDataMapArm2 = node_CtlCenter_globals.dic_485ex.get(str(ModbusID.BAL_ARM2.value), None)
    recvDataMapRotate540 = node_CtlCenter_globals.dic_485ex.get(str(ModbusID.ROTATE_MAIN_540.value), None)
    #return recvDataMapH,recvDataMapLift,recvDataMapSrvTele,recvDataMapBalTele,recvDataMapArm1,recvDataMapArm2,recvDataMapRotate540
    return recvDataMapH,recvDataMapLift,recvDataMapSrvTele,recvDataMapArm1,recvDataMapArm2,recvDataMapRotate540

lastCamControl = getDateTime()
# def ControlArco():
#     global lastCamControl
#     if not isTimeExceeded(lastCamControl, 1000):
#         return False
    
#     #PrintCurrentPos()
    
#     lastCamControl = getDateTime()
#     if len(node_CtlCenter_globals.lsARUCO_Result) == 0:
#         return False
    
#     filtered_data = filter_outliers(node_CtlCenter_globals.lsARUCO_Result)
#     node_CtlCenter_globals.lsARUCO_Result.clear()
#     filtered_data2 = filter_outliers(filtered_data)

#     #rospy.loginfo(filtered_data2)
    
#     if len(filtered_data2) == 0:
#         return False
    
#     dictArco =filtered_data2[-1]
#     pos_x = dictArco[ARUCO_RESULT_FIELD.X.name]
#     pos_y = dictArco[ARUCO_RESULT_FIELD.Y.name]
#     pos_z = dictArco[ARUCO_RESULT_FIELD.Z.name]
    
#     pos_x = strToRoundedInt(dictArco[ARUCO_RESULT_FIELD.DIFF_X.name] * 40)
#     pos_y = strToRoundedInt(dictArco[ARUCO_RESULT_FIELD.DIFF_Y.name] * 40)
#     pos_z = strToRoundedInt(dictArco[ARUCO_RESULT_FIELD.Z.name])
#     pos_angle = strToRoundedInt(dictArco[ARUCO_RESULT_FIELD.ANGLE.name])
#     curDistance, curAngle, cur_angle_360  = GetCurrentPosDistanceAngle()
#     curX,curY = calculate_coordinates(curDistance,curAngle)
#     rospy.loginfo(f'현위치:{curX},{curY},마커위치:{pos_x},{pos_y} 차이:{abs(pos_x-curX)},{abs(curY-pos_y)}')
    
#     if abs(curX - pos_x) > 100:
#         callData = String()
#         callData.data = f"{BLB_CMD_CUSTOM.PROFILE.name}:{pos_x},{pos_y},{pos_angle}"
#         rospy.loginfo(dictArco)
#         callbackBLB_CMD(callData)
#         return True
#     else:
#         return False

lock = threading.Lock()
def ScanWall():
    return
    if len(node_CtlCenter_globals.dicSafety) == 0:
        return
    
    if isActivatedMotor(ModbusID.ROTATE_TESTER.value):
        return
    
    with lock:
        temp_dicSafety = node_CtlCenter_globals.dicSafety.copy()
        
    dictWallMap = copy.deepcopy(temp_dicSafety)
    node_CtlCenter_globals.dicSafety.clear()
    angle_pc2_dict = {}
    for enc, pc2list in dictWallMap.items():
        targetAngle = round(mapRange(enc,0,10000,0,360))
        angle_pc2_dict[targetAngle] = pc2list
    
    # Define the radius from the rotation center to the lidar (in meters)
    radius = 0.01  # 1 cm
    rospy.loginfo(angle_pc2_dict.keys())
    if len(angle_pc2_dict) > 5:
      node_CtlCenter_globals.merged_pc2 = merge_point_clouds(angle_pc2_dict, radius)
    
#수평모터 돌고 있을때 이벤트 처리
def MotorH_Event():
    #돌고 있는 모터가 없으면 종료
    if not isActivatedMotor(ModbusID.MOTOR_H):
        return
    # recvDataMapH = node_CtlCenter_globals.dic_485ex.get(str(ModbusID.MOTOR_H.value), None)
    # if recvDataMapH == None:
    #     return
    
    listEPC = list(node_CtlCenter_globals.dicRFIDTmp.keys())
    if len(listEPC) == 0:
        return
    
    EPCTmp = listEPC[-1]
    cmd_pos, cur_pos = GetPosServo(ModbusID.MOTOR_H)

def PrintStatusInfoEverySec(diffCharRate = 0.96):
    try:
        #현재 노드
        lsCurTable,curNode = GetCurrentTableNode()
        #현재 H Pos(cur_pos), 회전모터별 앵글, 서빙길이, 밸런싱 길이. (오차율)
        curMovingTargetTable,curMovingTargetNode = GetCurrentTargetTable()
        #bAPI_Result, dicBatteryInfo = API_robot_battery_status()    
        dicBatteryInfo ={}
        dicBatteryInfo.update(node_CtlCenter_globals.dic_BMS)
        #rospy.loginfodicBatteryInfo)
        volt = dicBatteryInfo.get(MonitoringField_BMS.Voltage.name,MIN_INT)
        battery_level = try_parse_float(dicBatteryInfo.get(MonitoringField_BMS.RSOC.name))
        ischarging = dicBatteryInfo.get(MonitoringField_BMS.battery_status.name,"UNKNOWN")
        watt = dicBatteryInfo.get(MonitoringField_BMS.WATT.name,MIN_INT)
        #dicCurNode = getTableServingInfo(curTable)
        # node_ID = dicCurNode.get(TableInfo.NODE_ID.name, "")
        cmd_pos, cur_pos = GetPosServo(ModbusID.MOTOR_H)
        cur_pos_mm = pulseH_to_distance(cur_pos)
        #curX,curY = GetLocNodeID(node_CtlCenter_globals.node_current)
        curX,curY = GetLocXY()
        #cmd_pos_srvInner, cur_pos_srvInner = GetPosServo(ModbusID.TELE_SERV_INNER)
        cmd_pos_srvMain, cur_pos_srvMain = GetPosServo(ModbusID.TELE_SERV_MAIN)
        #distanceSrvInnerMechanic = (cur_pos_srvInner *  1.46433333)/roundPulse
        distanceSrvMainMechanic = (cur_pos_srvMain * 1.875)/roundPulse
        #distMechanic = round(distanceSrvMainMechanic+distanceSrvInnerMechanic)
        distMechanic = distanceSrvMainMechanic
        curDistanceSrvTele, curAngle,cur_angle_360=GetCurrentPosDistanceAngle()
        curDistanceSrvTele2, curSrvPer, curBalPulse,curBalPer = GetArmStatus()
        pulseBalanceCalculated = GetpulseBalanceCalculated()
        cur_rpm = getMotorSpdDirection(ModbusID.MOTOR_H.value)
        cur_spd= calculate_speed_fromRPM(cur_rpm)
        # pot_cur_arm1,not_cur_arm1 ,cmdpos_arm1,cur_pos_arm1 =GetPotNotCurPosServo(ModbusID.BAL_ARM1)
        # pot_cur_telebal,not_cur_telebal ,cmd_pos_telebal,cur_pos_telebal =GetPotNotCurPosServo(ModbusID.TELE_BALANCE)
        # arm_degrees = mapRange(cmdpos_arm1, not_cur_arm1,pot_cur_arm1, 0, 90)
        # distance_arm = calculate_third_side(LENGTH_ARM1,LENGTH_ARM2,arm_degrees)
        # distance_tele_bal = CalculateLengthFromPulse(ModbusID.TELE_BALANCE,cur_pos_telebal)
        # distance_arm_total = distance_arm+distance_tele_bal
        # distanceBalanceTotalExpected = calculate_balance_distance(WEIGHT_BALARM_GRAM,WEIGHT_SERVARM_GRAM, curDistanceSrvTele)
        # accuBalnceLength = (distanceBalanceTotalExpected / distance_arm_total) * 100
        table_target = GetTableTarget()
        dicCurrentJob = GetCurrentJob(table_target)
        dicFirstJob = GetCurrentJob(table_target, True)
        dicLastJob = GetCurrentJob(table_target, False)
        jobStr = ""
        if len(dicCurrentJob) > 0:
            jobStr = f'주행방향:{dicCurrentJob.get(APIBLB_FIELDS_INFO.direction.name)},시작노드:{dicFirstJob.get(APIBLB_FIELDS_TASK.startnode.name)},목표노드:{dicLastJob.get(APIBLB_FIELDS_TASK.startnode.name)}'
        accuBalnceLength = (pulseBalanceCalculated / curBalPulse) * 100 if curBalPulse > 0 else 0
        doorStatus,doorArray = GetDoorStatus() #TRAYDOOR_STATUS.OPENED
        r1,r2,r_total = getLoadWeight()
        if doorStatus == TRAYDOOR_STATUS.OPENED and len(node_CtlCenter_globals.dicTTS) == 0:
            if r1 > WEIGHT_OCCUPIED:
                LightTrayCell(TraySector.Cell1.value,LightBlink.Solid.value,LightColor.BLUE.value)
            else:
                LightTrayCell(TraySector.Cell1.value,LightBlink.Normal.value,LightColor.BLUE.value)
            if r2 > WEIGHT_OCCUPIED:
                LightTrayCell(TraySector.Cell2.value,LightBlink.Solid.value,LightColor.BLUE.value)
            else:
                LightTrayCell(TraySector.Cell2.value,LightBlink.Normal.value,LightColor.BLUE.value)
        angle_y = node_CtlCenter_globals.dicARD_CARRIER.get(DataKey.Angle_Y.name, -100)
        tilt_angle = node_CtlCenter_globals.dicARD_CARRIER.get(CARRIER_STATUS.TILT_ANGLE.name, -1)
        detected_height = node_CtlCenter_globals.dicARD_CARRIER.get(MAIN_STATUS.DETECTED_HEIGHT.name, -1)
        lastLD_Raw = GetDistanceV()
        dyros = GetDynamicConfigROS()      
        dyros.pop('groups', None)
        if len(lastLD_Raw) > 0:
            distancePoints = int(lastLD_Raw[2])
            distanceSTD = round(float(lastLD_Raw[1]),4)
            distanceLD = round(float(lastLD_Raw[0]),3)
            dyros["DISTANCE_POINTS"] = distancePoints
            dyros["DISTANCE_STD"] = distanceSTD
            dyros["DISTANCE_LD"] = distanceLD
        
        tilt_status_name = GetTiltStatus().name
        tray_height = GetLiftCurPositionDown()
        dfReceived = GetDF(curMovingTargetTable)
        dyros["TRAY_HEIGHT"] = tray_height
        dyros["TARGET_NODE"] = curMovingTargetNode
        dyros["TARGET_TABLE"] = curMovingTargetTable    
        #result = ','.join(map(str, lsCurTable))
        dyros["CUR_TABLE"] = lsCurTable
        dyros["CUR_H_LOC_MM"] = cur_pos_mm
        dyros["CUR_NODE"] = curNode
        dyros["TILT_STATUS_NAME"] = tilt_status_name
        dyros["CROSS_STATE"] = str(node_CtlCenter_globals.stateDic)
        dyros[MAIN_STATUS.DETECTED_HEIGHT.name] = detected_height
        if dfReceived is not None:
            dicLast = dfReceived.iloc[-1]
            filtered = {k: int(v) if isinstance(v, np.integer) else v
                        for k, v in dicLast.items()
                        if isinstance(v, (int, str)) and v not in [None, ""]}
            dyros.update(filtered)
        
        if dyros is not None:
            pub_ros_config.publish(json.dumps(dyros, sort_keys=True))
        #rospy.loginfo(node_CtlCenter_globals.last_detect_status)
        
        ledInfo = GetLedStatus()
        multi_line_string = f"""{jobStr}
        펄스기반서빙길이:{distMechanic}mm,현재서빙길이:{curDistanceSrvTele}mm,현재밸런싱펄스:{curBalPulse},계산된밸런싱펄스:{pulseBalanceCalculated},메인회전각도:{curAngle},트레이각도:{cur_angle_360}
        틸트:{tilt_status_name},DOORLOCKED:{doorArray},현재 무게 :{r_total}g,현재테이블:{lsCurTable},현재노드:{curNode},현재H위치:{cur_pos_mm}mm,현재속도:{cur_rpm}rpm/{cur_spd}mm/s,다음테이블:{GetTableList()}
        틸팅각:{angle_y},서보각:{tilt_angle},LED:{ledInfo},작업상태:{node_CtlCenter_globals.robot.get_current_state().name}
        크로스대기:{GetWaitCrossFlag()},유저대기:{GetWaitConfirmFlag()},현재목적노드:{curMovingTargetNode},최종목적테이블:{GetTableTarget()},현재좌표:({curX},{curY}),현재트레이:{node_CtlCenter_globals.lsRackStatus},분기기:{node_CtlCenter_globals.stateDic}
        현재 리프트 하강지점(m):{tray_height},아르코추정하강지점:{GetLiftCurPositionAruco()},전압:{volt}V,소비전력:{watt}W,배터리:{battery_level}%({ischarging}),밸런스펄스:{node_CtlCenter_globals.dicWeightBal[0]},
        """
        if isRealMachine and battery_level < 15 and ischarging.startswith('Dis'):
            TTSAndroid(TTSMessage.ALARM_BATTERY.value, 60)
        #서빙전개율:{curSrvPer}%,밸런싱암전개율:{curBalPer}%,정확도:{accuBalnceLength:.1f}%,
        #남은 서빙 시간 : {GetRPMFromTimeAccDecc(node_CtlCenter_globals.listBLB)}
        # 토크맥스정보:{json.dumps(node_CtlCenter_globals.dicTorqueMax, indent=2)}
        # 오버로드정보:{json.dumps(node_CtlCenter_globals.dicOvrMax, indent=2)}
        # 주행타겟정보:{json.dumps(node_CtlCenter_globals.dicTargetPos, indent=2)}
        
        diffCharCnt = difflib.SequenceMatcher(None,node_CtlCenter_globals.multi_line_string, multi_line_string).ratio()
        if diffCharCnt < diffCharRate and len(getRunningMotorsBLB()) == 0:
            rospy.loginfo(multi_line_string)
            node_CtlCenter_globals.multi_line_string = multi_line_string
            
        #print(f'주행타겟정보:{json.dumps(node_CtlCenter_globals.dicTargetPos, indent=4)}')
        #pprint.pprint(f'')
        # dicPos = GetCurrentPos()
        # pub_motorPos.publish(json.dumps(dicPos, sort_keys=True))
    except Exception as e:
      rospy.loginfo(e)
      sMsg = traceback.format_exc()
      SendAlarmHTTP(sMsg,True,node_CtlCenter_globals.BLB_ANDROID_IP)
      StopEmergency(sMsg)   
    return

def KeepArmBalancing():
    if len(getRunningMotorsBLB()) > 0 or not isTimeExceeded(GetLastBalanceTimeStamp(), MODBUS_EXECUTE_DELAY_ms):
        return
    curDistanceSrvTele2, curSrvPer, curBalPulse, curBalPer = GetArmStatus()
    #암전개율이 10% 이하에서는 조절하지 않는다.
    #아래 코드 빠지면 div/0 로 무한루프
    if curSrvPer < 10:
      return
    
    pulseBalanceTotal = GetpulseBalanceCalculated()
    curBalPerResult = (curBalPulse / pulseBalanceTotal)*100 if pulseBalanceTotal > 0 else 0
    
    # if abs(curBalPerResult-100) > 1:
    #     lsFixed,estTime = GetDicBalArmFixedPulse(pulseBalanceTotal, True)
    #     lsFiltered = remove_dict_by_value(lsFixed, MotorWMOVEParams.SPD.name, DEFAULT_RPM_MIN)
    #     rospy.loginfo(lsFiltered)
    #     SendCMD_Device(lsFiltered)
    #     UpdateLastBalanceTimeStamp()
    #     TTSAndroid('중량이 변경되었습니다.',1000)

def CheckMotorAlarms():
    lsAlarmMBID,dic_AlmCDTable,dic_AlmNMTable=getBLBMotorStatus()
    for i in range(len(lsAlarmMBID)):
        intMbid = lsAlarmMBID[i]
        alm_cd = dic_AlmCDTable[intMbid]
        if str(intMbid) in node_CtlCenter_globals.lsSlowDevices:
            continue
        if alm_cd == -1 or alm_cd == '-1':
            continue
        alm_nm = dic_AlmNMTable[intMbid]
        motor_nm = ModbusID.from_value(intMbid).name
        logmsg = f'{intMbid}:{motor_nm},{alm_cd}-{alm_nm}'
        # alm_check = isActivatedMotor(intMbid)
        # if alm_check:# and not GetWaitConfirmFlag():
        if getRunningMotorsBLB():
            lsAlmClearCmd = getListedDic(getMotorSimpleCmdDic(intMbid, MotorCmdField.WALM_C))
            SendCMD_Device(lsAlmClearCmd)
            rospy.loginfo(f'Alarm:{alm_nm}:{alm_cd}:{logmsg}')
            StopEmergency(logmsg)
        else:
          key, value = logmsg.split(sDivFieldColon, 1)
          # key와 value의 앞뒤 공백 제거
          key = key.strip()
          value = value.strip()    
          SetWaitConfirmFlag(True, {key:value})

def CheckETCActions():
    lsCmdRelase = []
    releasePulse = round(roundPulse/10)
    cur_node = GetCurrentNode()
    DI_POT,DI_NOT,DI_HOME,SI_POT = GetPotNotHomeStatus(ModbusID.MOTOR_H)
    if isRealMachine and cur_node == node_KITCHEN_STATION and isTrue(DI_POT):
        #get_mpv_process_info()
        node_CtlCenter_globals.DefaultGndDistance = float(rospy.get_param(f"~{ROS_PARAMS.lidar_gnd_limit.name}", default=0.56))    
        #stateCharger = isChargerPlugOn()
        # node_pos = GetNodePos_fromNode_ID(cur_node)
        # cmd_pos,cur_pos=GetPosServo(ModbusID.MOTOR_H)
        # if cur_node == node_KITCHEN_STATION and isTrue(DI_POT):        
        #     if abs(cur_pos - node_pos) > roundPulse/2:
        #         dicLoc = getMotorLocationSetDic(ModbusID.MOTOR_H.value, node_pos)
        #         SendCMD_Device([dicLoc])
        #     if not isActivatedMotor(ModbusID.MOTOR_H.value):
        #         SetChargerPlug(True)
        #관절이 POT, NOT 에 있으면 0.1바퀴씩 풀어준다
        for modbus in lsReleaseMotors:
            DI_POT,DI_NOT,DI_HOME,SI_POT = GetPotNotHomeStatus(modbus)        
            cmdpos, curpos = GetPosServo(modbus)
            if DI_POT:
                lsCmdRelase.append(getMotorMoveDic(modbus.value,True, curpos-releasePulse,MAINROTATE_RPM_SLOWEST,ACC_DECC_SMOOTH,ACC_DECC_SMOOTH))
            elif DI_NOT:
                lsCmdRelase.append(getMotorMoveDic(modbus.value,True, curpos+releasePulse,MAINROTATE_RPM_SLOWEST,ACC_DECC_SMOOTH,ACC_DECC_SMOOTH))
    else:
        curNode_sse = node_CtlCenter_globals.dicLast_POSITION_INFO['NODE_ID']
        if GetCurrentNode() != curNode_sse and IsOrderEmpty():
            SetCurrentNode(curNode_sse)
            TTSAndroid(f'{curNode_sse}번 노드로 변경되었습니다.')
    if len(lsCmdRelase) > 0:
        SendCMD_DeviceService(lsCmdRelase)
    
def CheckETCAlarms():
    lsAlarmMBID,dic_AlmCDTable,dic_AlmNMTable=getBLBMotorStatus()
    for i in range(len(lsAlarmMBID)):
        intMbid = lsAlarmMBID[i]
        alm_cd = dic_AlmCDTable[intMbid]
        if str(intMbid) in node_CtlCenter_globals.lsSlowDevices:
            continue
        if alm_cd == -1 or alm_cd == '-1':
            continue
        alm_nm = dic_AlmNMTable[intMbid]
        motor_nm = ModbusID.from_value(intMbid).name
        logmsg = f'{intMbid}:{motor_nm},{alm_cd}-{alm_nm}'
        # alm_check = isActivatedMotor(intMbid)
        # if alm_check:# and not GetWaitConfirmFlag():
        lsAlmClearCmd = getListedDic(getMotorSimpleCmdDic(intMbid, MotorCmdField.WALM_C))
        SendCMD_Device(lsAlmClearCmd)
        rospy.loginfo(f'Alarm:{alm_nm}:{alm_cd}:{logmsg}')

        if len(getRunningMotorsBLB()) > 0:
          StopEmergency(logmsg)
        else:
          key, value = logmsg.split(sDivFieldColon, 1)
          # key와 value의 앞뒤 공백 제거
          key = key.strip()
          value = value.strip()    
          SetWaitConfirmFlag(True, {key:value})
    
def MotorBalanceControlEx(bSkip):
    if not hasattr(MotorBalanceControlEx, "onCaliR"):
        MotorBalanceControlEx.onCaliR = False
    if not hasattr(MotorBalanceControlEx, "onCaliT"):
        MotorBalanceControlEx.onCaliT = False
    
    #돌고 있는 모터가 없으면 종료
    if not has_common_element(getRunningMotorsBLB(),list_ArmControlMotors):
        return
    
    # 밸런싱암 관련 명령어가 나간지 0.5초 경과하지 않으면 무시한다.
    if not isTimeExceeded(GetLastBalanceTimeStamp(), MODBUS_EXECUTE_DELAY_ms):
        return
    #5축 제어 모터 정보가 제대로 들어왔는지 확인.
    #recvDataMapH,recvDataMapLift,recvDataMapSrvTele,recvDataMapBalTele,recvDataMapArm1,recvDataMapArm2,recvDataMapRotate540 = GetDynamicMotorsInfo()
    recvDataMapH,recvDataMapLift,recvDataMapSrvTele,recvDataMapArm1,recvDataMapArm2,recvDataMapRotate540 = GetDynamicMotorsInfo()
    # if None in (recvDataMapRotate540,recvDataMapSrvInner, recvDataMapSrvTele,recvDataMapBalTele,recvDataMapArm1,recvDataMapArm2):
    #     return
    #토크 체크 : 나중에 테스트 하자
    # if isRealMachine:
    #   CheckActivateMotorTorque()
    # 밸런싱암 수축 완료된 후 밸런싱 암 컨트롤 시작.
    #mbidstr_arm1 = str(ModbusID.BAL_ARM1.value)
    mbidstr_arm2 = str(ModbusID.BAL_ARM2.value)
    motorStatus_arm1 = getMotorStatus(ModbusID.BAL_ARM1)
    motorStatus_arm2 = getMotorStatus(ModbusID.BAL_ARM2)
    #motorStatus_teleBalance = getMotorStatus(ModbusID.TELE_BALANCE)
    #motorStatus_SrvInner = getMotorStatus(ModbusID.TELE_SERV_INNER)
    
    #isFinishedMotor_teleBalance = not isActivatedMotor(ModbusID.TELE_BALANCE.value)
    # isFinishedMotorArm1 = not isActivatedMotor(ModbusID.BAL_ARM1.value)
    # isFinishedMotorArm2 = not isActivatedMotor(ModbusID.BAL_ARM2.value)    

    pot_cur_arm1,not_cur_arm1, cmdpos_arm1,cur_pos_arm1 = GetPotNotCurPosServo(ModbusID.BAL_ARM1)
    pot_cur_arm2,not_cur_arm2 ,cmdpos_arm2,cur_pos_arm2 =GetPotNotCurPosServo(ModbusID.BAL_ARM2)
    DI_POT_arm1,DI_NOT_arm1,DI_HOME_arm1,SI_POT_arm1 = GetPotNotHomeStatus(ModbusID.BAL_ARM1)
    DI_POT_arm2,DI_NOT_arm2,DI_HOME_arm2,SI_POT_arm2 = GetPotNotHomeStatus(ModbusID.BAL_ARM2)
    pot_cur_SrvTele,not_cur_SrvTele ,cmdpos_SrvTele,cur_pos_SrvTele =GetPotNotCurPosServo(ModbusID.TELE_SERV_MAIN)
    #pot_cur_BalTele,not_cur_BalTele ,cmdpos_BalTele,cur_pos_BalTele =GetPotNotCurPosServo(ModbusID.TELE_BALANCE)
    #pot_cur_SrvInner,not_cur_SrvInner ,cmdpos_SrvInner,cur_pos_SrvInner =GetPotNotCurPosServo(ModbusID.TELE_SERV_INNER)    
    
    pot_cur_540,not_cur_540 ,cmdpos_540,cur_pos_540 =GetPotNotCurPosServo(ModbusID.ROTATE_MAIN_540)
    pot_cur_lift,not_cur_lift ,cmdpos_lift,cur_pos_lift =GetPotNotCurPosServo(ModbusID.MOTOR_V)
    pot_cur_h,not_cur_h ,cmdpos_h,cur_pos_h =GetPotNotCurPosServo(ModbusID.MOTOR_H)
    #isFinishedMotor_teleBalance = isFinishedMotor(ModbusID.TELE_BALANCE,2000)
    isFinishedMotorTray = not isActivatedMotor(ModbusID.ROTATE_SERVE_360.value)
    isFinishedMotorArm1 = not isActivatedMotor(ModbusID.BAL_ARM1.value)
    isFinishedMotorArm2 = not isActivatedMotor(ModbusID.BAL_ARM2.value)
    # isFinishedMotorArm1 = isFinishedMotor(ModbusID.BAL_ARM1, round(pot_cur_arm1*0.15))
    # isFinishedMotorArm2 = isFinishedMotor(ModbusID.BAL_ARM2,round(pot_cur_arm2*0.15))
    # isFinishedMotorArm1 = isFinishedMotor(ModbusID.BAL_ARM1, round(pot_cur_arm1*0.1))
    # isFinishedMotorArm2 = isFinishedMotor(ModbusID.BAL_ARM2,round(pot_cur_arm2*0.1))
    doorStatus,doorArray = GetDoorStatus()
    isClose1 = doorArray[0]
    isClose2 = doorArray[-1]

    #제로디바이드 발생
    cur_angle_540 = pulse_to_angle(cur_pos_540, pot_cur_540, MAX_ANGLE_BLBBODY)
    curDistanceSrvTele, cur_angle_540,cur_angle_360  = GetCurrentPosDistanceAngle()
    #len_SrvInner = CalculateLengthFromPulse(ModbusID.TELE_SERV_INNER,cur_pos_SrvInner)
    len_SrvTele = CalculateLengthFromPulse(ModbusID.TELE_SERV_MAIN,cur_pos_SrvTele)
    #len_SrvTotal = len_SrvInner+len_SrvTele
    len_SrvTotal = len_SrvTele
    cur_srvX,cur_srvY = calculate_coordinates(len_SrvTotal, cur_angle_540)
    if bSkip:
        rospy.loginfo(f'현재위치:{cur_srvX},{cur_srvY}, 메인각도:{cur_angle_540}')

    spd_cur_540 = int(recvDataMapRotate540.get(str(MonitoringField.CUR_SPD.name), MIN_INT))
    #spd_cur_balTele = int(recvDataMapBalTele.get(str(MonitoringField.CUR_SPD.name), MIN_INT))
    spd_cur_arm1 = int(recvDataMapArm1.get(str(MonitoringField.CUR_SPD.name), MIN_INT))
    spd_cur_arm2 = int(recvDataMapArm2.get(str(MonitoringField.CUR_SPD.name), MIN_INT))
    spd_cur_srvTele = int(recvDataMapSrvTele.get(str(MonitoringField.CUR_SPD.name), 0))
    spd_cur_H = abs(int(recvDataMapH.get(str(MonitoringField.CUR_SPD.name), 0)))

    targetPOS_V = node_CtlCenter_globals.dicTargetPos.get(str(ModbusID.MOTOR_V.value),MIN_INT)
    targetPOS_arm2 = node_CtlCenter_globals.dicTargetPos.get(str(ModbusID.BAL_ARM2.value),MIN_INT)
    targetPOS_arm1 = node_CtlCenter_globals.dicTargetPos.get(str(ModbusID.BAL_ARM1.value),MIN_INT)
    targetPOS_srv = node_CtlCenter_globals.dicTargetPos.get(str(ModbusID.TELE_SERV_MAIN.value),MIN_INT)
    #targetPOS_balTele = node_CtlCenter_globals.dicTargetPos.get(str(ModbusID.TELE_BALANCE.value),MIN_INT)
    isArm1ReachedToTarget = True if abs(int(targetPOS_arm1) - cur_pos_arm1) < roundPulse else False
    
    #isArm2ReachedToTarget = True if abs(targetPOS_arm2 - cur_pos_arm2) < roundPulse else False
    curTargetTable,curTarNode = GetCurrentTargetTable()
    #curTable,curNode = GetCurrentTargetTable()
    curNode = GetCurrentNode()
    
    dfReceived = GetDF(curTargetTable)
    # columns_to_keep = []
    # if dfReceived is not None:
    #   columns_to_keep = [field.name for field in APIBLB_FIELDS_TASK if field.name in dfReceived.columns]
    dicTagretTableInfoCurrent = getTableServingInfo(curTargetTable)
    isScanOn = isScanTableMode(curTargetTable)
    target540 = dicTagretTableInfoCurrent.get(TableInfo.SERVING_ANGLE.name, StateBranchValue.ERROR.value)
    isTeaching540 = True if target540 == StateBranchValue.ERROR.value else False
    #isScanning = isScanMode()
    targetServingDistance = dicTagretTableInfoCurrent.get(TableInfo.SERVING_DISTANCE.name, StateBranchValue.ERROR.value)
    isTeachingServingDistance = True if targetServingDistance == StateBranchValue.ERROR.value else False
    targetTable = dicTagretTableInfoCurrent.get(TableInfo.TABLE_ID.name, StateBranchValue.ERROR.value)
    tiltStutus = GetTiltStatus()
    modbusIDStr_H = str(ModbusID.MOTOR_H.value)
    modbusIDStr_V = str(ModbusID.TELE_SERV_MAIN.value)   
    isDistanceVReceived = len(GetDistanceV()) > 0
    marker_magin_cnt = 1
    
    #모터 ID 로 그룹화 하는 것이 아닌 각 기능별로 그룹화 할 것.
    if isScanOn:
        lsAruco = filter_recent_data(ARUCO_RESULT_FIELD.LASTSEEN.name,GetArucoMarkerInfo(),0.2)
        if len(lsAruco) > 0:
            dicAruco = lsAruco[0]
            marker_X = float(dicAruco[ARUCO_RESULT_FIELD.X.name])
            marker_Y = float(dicAruco[ARUCO_RESULT_FIELD.Y.name])
            marker_Y_abs = abs(marker_Y)
            angle_marker = int(dicAruco[ARUCO_RESULT_FIELD.ANGLE.name])
            value_marker = int(dicAruco[ARUCO_RESULT_FIELD.MARKER_VALUE.name])
            diff_X,diff_Y =get_camera_offset(CAMERA_DISTANCE_FROM_CENTER,9)
            ref_dict2 = { 'X' : ref_dict['X'] - diff_X, 'Y': ref_dict['Y'] + diff_Y, 'Z': ref_dict['Z'] }
            if isActivatedMotor(ModbusID.ROTATE_MAIN_540.value):
                isScanSpd =is_within_range(abs(spd_cur_540),MAINROTATE_RPM_SLOWEST,15)
                if MotorBalanceControlEx.onCaliR == False:
                    return
                curDistanceSrvTele, curAngle_540,cur_angle_360  = GetCurrentPosDistanceAngle()
                rospy.loginfo(f'Scan spd_cur_540:{spd_cur_540},isScanSpd:{isScanSpd},MarkerXY:{marker_X},{marker_Y},lastDifX={node_CtlCenter_globals.aruco_lastDiffX},lastDifY={node_CtlCenter_globals.aruco_lastDiffY}')
                if isScanSpd:
                    #ClearArucoTable()
                    rospy.loginfo(f'spd_cur_540 : {spd_cur_540}')
                    resultDiff1,diff_X1,diff_Y1= compare_dicts(dicAruco, ref_dict, CAM_LOCATION_MARGIN_OK)
                    #resultDiff2,diff_X2,diff_Y2= compare_dicts(dicAruco, ref_dict2, CAM_LOCATION_MARGIN_OK)
                    rospy.loginfo(f'Aruco X1 Check : resultDiff={resultDiff1},diff_X={diff_X1},diff_Y={diff_Y1},lastDifX={node_CtlCenter_globals.aruco_lastDiffX}')
                    #rospy.loginfo(f'Aruco X2 Check : resultDiff={resultDiff2},diff_X={diff_X2},diff_Y={diff_Y2}')
                    # rospy.loginfo(format_vars(currTime,resultDiff,diff_X,diff_Y))
                    
                    #if abs(diff_X1) < CAM_LOCATION_MARGIN_OK or (abs(diff_X1) < CAM_LOCATION_MARGIN_FINE and node_CtlCenter_globals.aruco_lastDiffX < abs(diff_X1)):
                    if abs(diff_X1) <= CAM_LOCATION_MARGIN_OK*marker_magin_cnt:
                        MotorBalanceControlEx.onCaliR = False
                        #StopAllMotors(ACC_DECC_SMOOTH)
                        StopMotor(ModbusID.ROTATE_MAIN_540.value, ACC_DECC_LONG)
                        time.sleep(MODBUS_EXCEPTION_DELAY)
                        lsMotorOperationNew = []
                        rospy.loginfo(f'Aruco X OK : cam diff_X={diff_X1},cam diff_Y={diff_Y1},lastDifY={node_CtlCenter_globals.aruco_lastDiffY}')
                        if abs(diff_Y1) <= CAM_LOCATION_MARGIN_OK*marker_magin_cnt:
                            MotorBalanceControlEx.onCaliT = False
                            curX,curY = calculate_coordinates(curDistanceSrvTele,curAngle_540)
                            diffX, diffY = calculate_position_shift(angle_marker, marker_coords_goldsample)
                            diffx_meter = ConvertArucoSizeToReal(diffX)
                            diffy_meter = ConvertArucoSizeToReal(diffY)    
                            if angle_marker > 0:
                                newX = curX + diffx_meter
                            else:
                                newX = curX - diffx_meter
                            newY = curY + diffy_meter
                            rospy.loginfo(f'현재위치XY:{curX,curY},XY보정마진:{diffx_meter,diffy_meter},타겟XY:{newX,newY}')
                            distanceFinal, angle_degrees_final = calculate_distance_and_angle(newX, newY)
                            df = pd.read_csv(strFileTableNodeEx, sep=sDivTab)
                            # 조건에 맞는 행의 MARKER_VALUE 업데이트
                            df.loc[df['TABLE_ID'] == curTargetTable, 'MARKER_VALUE'] = curTargetTable
                            df.to_csv(strFileTableNodeEx, index=False, sep=sDivTab)
                            isScanOn = False
                            dicFinalRotate = GetDicRotateMotorMain(angle_degrees_final,rotateRPM=MAINROTATE_RPM_SLOWEST)
                            angle_new = (180+ angle_marker)%360
                            lsMotorOperationNew = []
                            lsMotorOperationNew.append([dicFinalRotate])
                            lsMotorOperationNew.append(GetStrArmExtendMain(distanceFinal,angle_degrees_final,True))
                            lsMotorOperationNew.append([GetDicRotateMotorTray(angle_new)])
                            lsMotorOperationNew.append(GetListLiftDown(250000))
                            node_CtlCenter_globals.listBLB.clear()
                            node_CtlCenter_globals.listBLB.extend(lsMotorOperationNew)
                        elif diff_Y1 < 0:
                            lsMotorOperationNew.extend(GetStrArmExtendMain(1250,0,True))
                            MotorBalanceControlEx.onCaliT = True
                        else:
                            lsMotorOperationNew.extend(GetStrArmExtendMain(0,0,True))
                            MotorBalanceControlEx.onCaliT = True
                        node_CtlCenter_globals.listBLB.append(lsMotorOperationNew)
                    else:
                        node_CtlCenter_globals.aruco_lastDiffX = abs(diff_X1)

            elif isActivatedMotor(ModbusID.TELE_SERV_MAIN.value):
                rospy.loginfo(f'Scan spd_cur_srvTele:{spd_cur_srvTele},MarkerXY:{marker_X,marker_Y}')
                if MotorBalanceControlEx.onCaliT == False:
                    return
                if abs(spd_cur_srvTele) > 10:
                    curDistanceSrvTele, curAngle_540,cur_angle_360  = GetCurrentPosDistanceAngle()
                    resultDiff3,diff_X3,diff_Y3= compare_dicts(dicAruco, ref_dict, CAM_LOCATION_MARGIN_OK)
                    #resultDiff4,diff_X4,diff_Y4= compare_dicts(dicAruco, ref_dict2, CAM_LOCATION_MARGIN_OK)
                    rospy.loginfo(f'Check Y result Diff3:{resultDiff3},diff_X3={diff_X3},diff_Y3={diff_Y3},lastDifY={node_CtlCenter_globals.aruco_lastDiffY}')
                    #rospy.loginfo(f'Check Y result Diff4={resultDiff4},diff_X4={diff_X4},diff_Y4={diff_Y4}')
                    #ClearArucoTable()
                    lastY = node_CtlCenter_globals.aruco_lastDiffY
                    node_CtlCenter_globals.aruco_lastDiffY = abs(diff_Y3)
                    seekRange = int(pot_cur_540 / 2)
                    #if abs(diff_Y3) < CAM_LOCATION_MARGIN_OK or (lastY < abs(diff_Y3) and abs(diff_Y3) < CAM_LOCATION_MARGIN_FINE):
                    if abs(diff_Y3) <= CAM_LOCATION_MARGIN_OK*marker_magin_cnt:
                        StopAllMotors(ACC_DECC_SMOOTH)
                        rospy.loginfo(f'Aruco Y OK : resultDiff={resultDiff3},diff_X={diff_X3},diff_Y={diff_Y3},lastDifY={node_CtlCenter_globals.aruco_lastDiffY}')
                        if abs(diff_X3) > CAM_LOCATION_MARGIN_OK*marker_magin_cnt:
                            MotorBalanceControlEx.onCaliT = False
                            MotorBalanceControlEx.onCaliR = True                            
                            seekMargin = seekRange if diff_X3 < 0 else -seekRange
                            #StopAllMotors(ACC_DECC_LONG)
                            # if curAngle_540 > 180:
                            #     seekMargin = -seekMargin
                            node_CtlCenter_globals.dicTargetPos[modbusIDStr_V] = 210000
                            target_pulse=  cur_pos_540 + seekMargin                     
                            dicRotateNewVerySlow = getMotorMoveDic(ModbusID.ROTATE_MAIN_540.value, True, target_pulse,MAINROTATE_RPM_SLOWEST,ACC_540,DECC_540)
                            #dicRotateNewVerySlow = GetDicRotateMotorMain(curAngle_540_new,MAINROTATE_RPM_SLOWEST,False)
                            node_CtlCenter_globals.listBLB.clear()
                            node_CtlCenter_globals.aruco_lastDiffY = aruco_lastDiff_Default
                            SendCMD_DeviceService([dicRotateNewVerySlow])
                            time.sleep(MODBUS_EXCEPTION_DELAY)  
                        else:                      
                            MotorBalanceControlEx.onCaliT = False
                            MotorBalanceControlEx.onCaliR = False                            
                            #curDistanceSrvTele, curAngle_540,cur_angle_360
                            curX,curY = calculate_coordinates(curDistanceSrvTele,curAngle_540)
                            diffX, diffY = calculate_position_shift(angle_marker, marker_coords_goldsample)
                            diffx_meter = ConvertArucoSizeToReal(diffX)
                            diffy_meter = ConvertArucoSizeToReal(diffY)    
                            if angle_marker > 0:
                                newX = curX + diffx_meter
                            else:
                                newX = curX - diffx_meter
                            newY = curY + diffy_meter
                            distanceFinal, angle_degrees_final = calculate_distance_and_angle(newX, newY)
                            rospy.loginfo(f'현재위치XY:{curX,curY},XY보정마진:{diffx_meter,diffy_meter},타겟XY:{newX,newY}')
                            rospy.loginfo(f'현재:{curAngle_540,curDistanceSrvTele},보정:{angle_degrees_final,distanceFinal}')
                            df = pd.read_csv(strFileTableNodeEx, sep=sDivTab)
                            # 조건에 맞는 행의 MARKER_VALUE 업데이트
                            df.loc[df['TABLE_ID'] == curTargetTable, 'MARKER_VALUE'] = curTargetTable
                            df.to_csv(strFileTableNodeEx, index=False, sep=sDivTab)
                            isScanOn = False
                            dicFinalRotate = GetDicRotateMotorMain(angle_degrees_final,rotateRPM=MAINROTATE_RPM_SLOWEST)
                            rospy.loginfo(dicFinalRotate)
                            angle_new = (180+ angle_marker)%360
                            lsMotorOperationNew = []
                            lsMotorOperationNew.append([dicFinalRotate])
                            lsMotorOperationNew.append(GetStrArmExtendMain(distanceFinal,angle_degrees_final,True))
                            lsMotorOperationNew.append([GetDicRotateMotorTray(angle_new)])
                            lsMotorOperationNew.append(GetListLiftDown(200000))
                            node_CtlCenter_globals.listBLB.clear()
                            node_CtlCenter_globals.listBLB.extend(lsMotorOperationNew)
                    #else:
                    # elif marker_Y_abs > 0 and marker_Y_abs < 0.4 and node_CtlCenter_globals.aruco_lastDiffX == aruco_lastDiff_Default:
                    # # elif abs(diff_X3) > CAM_LOCATION_MARGIN_FINE:
                    #     targetPosTeleSrv=try_parse_int(node_CtlCenter_globals.dicTargetPos.get(modbusIDStr_V), MIN_INT)
                    #     if is_between(not_cur_SrvTele,200000,targetPosTeleSrv):
                    #         # StopAllMotors(ACC_DECC_SMOOTH)
                    #         # resultDiff4,diff_X4,diff_Y4= compare_dicts(dicAruco, ref_dict, CAM_LOCATION_MARGIN_OK)
                    #         # rospy.loginfo(f'한번에 보정하기. : resultDiff={resultDiff4},diff_X={diff_X4},diff_Y={diff_Y4},lastDifY={node_CtlCenter_globals.aruco_lastDiffY}')
                    #         # #curDistanceSrvTele, curAngle_540,cur_angle_360
                    #         # curX,curY = calculate_coordinates(curDistanceSrvTele,curAngle_540)
                    #         # #diffX, diffY = calculate_position_shift(angle_marker, marker_coords_goldsample)
                    #         # diffx_meter = ConvertArucoSizeToReal(diff_X4)
                    #         # diffy_meter = ConvertArucoSizeToReal(diff_Y4)    
                    #         # newX = curX + diffx_meter
                    #         # newY = curY + diffy_meter
                    #         # rospy.loginfo(f'현재위치XY:{curX,curY},XY보정마진:{diffx_meter,diffy_meter},타겟XY:{newX,newY}')
                    #         # distanceFinal, angle_degrees_final = calculate_distance_and_angle(newX, newY)
                    #         # df = pd.read_csv(strFileTableNodeEx, sep=sDivTab)
                    #         # # 조건에 맞는 행의 MARKER_VALUE 업데이트
                    #         # df.loc[df['TABLE_ID'] == curTargetTable, 'MARKER_VALUE'] = curTargetTable
                    #         # df.to_csv(strFileTableNodeEx, index=False, sep=sDivTab)
                    #         # isScanOn = False                            
                    #         # dicFinalRotate = GetDicRotateMotorMain(angle_degrees_final,rotateRPM=MAINROTATE_RPM_SLOWEST)
                    #         # angle_new = (180+ angle_marker)%360
                    #         # lsMotorOperationNew = []
                    #         # lsMotorOperationNew.append([dicFinalRotate])
                    #         # lsMotorOperationNew.append(GetStrArmExtendMain(distanceFinal,angle_degrees_final,True))
                    #         # lsMotorOperationNew.append([GetDicRotateMotorTray(angle_new)])
                    #         # lsMotorOperationNew.append(GetListLiftDown(200000))
                    #         # node_CtlCenter_globals.listBLB.clear()
                    #         # node_CtlCenter_globals.listBLB.extend(lsMotorOperationNew)
                            
                    #         seekMargin = seekRange if diff_X3 < 0 else -seekRange
                    #         StopAllMotors(ACC_DECC_LONG)
                    #         node_CtlCenter_globals.dicTargetPos[modbusIDStr_V] = 230000
                    #         target_pulse=  cur_pos_540 + seekMargin                     
                    #         dicRotateNewVerySlow = getMotorMoveDic(ModbusID.ROTATE_MAIN_540.value, True, target_pulse,MAINROTATE_RPM_SLOWEST,ACC_540,DECC_540)
                    #         #dicRotateNewVerySlow = GetDicRotateMotorMain(curAngle_540_new,MAINROTATE_RPM_SLOWEST,False)
                    #         node_CtlCenter_globals.listBLB.clear()
                    #         node_CtlCenter_globals.aruco_lastDiffY = aruco_lastDiff_Default
                    #         SendCMD_DeviceService([dicRotateNewVerySlow])
                    #         MotorBalanceControlEx.onCaliT = False
                    #         MotorBalanceControlEx.onCaliR = True                                                        
                    #         time.sleep(MODBUS_EXCEPTION_DELAY)
            # elif isActivatedMotor(ModbusID.TELE_SERV_MAIN.value) and spd_cur_srvTele > 0:
            #     if lsDF:
            #         resultDiff,diff_X,diff_Y= compare_dicts(dicAruco, ref_dict, CAM_LOCATION_MARGIN_OK)
            #         # rospy.loginfo(format_vars(currTime,resultDiff,diff_X,diff_Y))
            #         rospy.loginfo(f'resultDiff={resultDiff},diff_X={diff_X},diff_Y={diff_Y}')
            #         if compare_dicts(dicAruco, ref_dict, CAM_LOCATION_MARGIN_OK):
            #             StopAllMotors()
            #         elif compare_dicts(dicAruco, ref_dict, CAM_LOCATION_MARGIN_FINE):
            #             node_CtlCenter_globals.listBLB.clear()
            #             node_CtlCenter_globals.listBLB.extend(lsDF)
            #             StopAllMotors()

    
    if isActivatedMotor(modbusIDStr_H):
        #1.특정 포인트를 지나칠때마다 dataframe 의 sub-jobid 의 상태를 업데이트 해야함.
        if len(node_CtlCenter_globals.dicTargetPosFeedBack) > 0:
            #rospy.loginfo(node_CtlCenter_globals.dicTargetPosFeedBack)
            #dicTargetPosFeedBack value 가 sub-jobid 임.
            keys_to_pop = [key for key in node_CtlCenter_globals.dicTargetPosFeedBack if abs(cur_pos_h - key) <= roundPulse * 3]
            if keys_to_pop is not None and dfReceived is not None:
                for iPosCheckPoint in keys_to_pop:
                    strDetailcode = node_CtlCenter_globals.dicTargetPosFeedBack.pop(iPosCheckPoint,None)
                    rospy.loginfo(f'POP Value:{iPosCheckPoint},DetailCode:{strDetailcode}')
                    iDetailcode = try_parse_int(strDetailcode)
                    if strDetailcode is not None and iDetailcode > 0:
                        #Pause 플래그가 걸려있는지 미리 확인한다.
                        curState = APIBLB_STATUS_TASK.Running.value
                        lsPaused = dfReceived[dfReceived[APIBLB_FIELDS_NAVI.workstatus.name] == APIBLB_STATUS_TASK.Paused.value].tail(1).to_dict(orient='records')
                        if len(lsPaused) > 0:
                            curState = APIBLB_STATUS_TASK.Paused.value

                        # #detailcode 는 정수형태의 문자열로 오며 오름차순이므로 정수로 바꿔 비교한다
                        # dfReceived[key_detailcode] = dfReceived[key_detailcode].astype(int)
                        #방금 지나친 지점에 해당하는 detailcode 보다 낮은것들은 다 완료로 바꾼다
                        dfReceived.loc[dfReceived[key_detailcode] <= iDetailcode, APIBLB_FIELDS_NAVI.workstatus.name] = APIBLB_STATUS_TASK.Completed.value
                        #방금 지나친 지점에 해당하는 detailcode 보다 높은값들중 가장 작은걸 골라서 running 이나 pause 로 바꾼다
                        min_notStarted_detailcode = dfReceived[dfReceived[key_detailcode] > iDetailcode][key_detailcode].min()
                        dfReceived.loc[dfReceived[key_detailcode] == min_notStarted_detailcode, APIBLB_FIELDS_NAVI.workstatus.name] = curState
                        
                        PrintDF(dfReceived)
                        #서버로 전송
                        STATUS_TASK=APIBLB_STATUS_TASK.Running
                        if GetWaitConfirmFlag():
                          STATUS_TASK=APIBLB_STATUS_TASK.Paused
                        
                        resultAPI, nodeReturn = API_robot_navigation_info(dfReceived,STATUS_TASK)
                        if nodeReturn is None:
                            rospy.loginfo(resultAPI)
                        else:
                            SetCurrentNode(nodeReturn)
                        # DataFrame을 리스트로 변환
                        data_list = dfReceived.to_dict(orient="records")
                        # 리스트를 JSON 문자열로 변환
                        json_string = json.dumps(data_list, ensure_ascii=False)
                        pub_DF.publish(json_string)
                        node_current = GetCurrentNode()
                        UpdateXY_nodeInfo(node_current)
                        # if len(node_CtlCenter_globals.lsHistory_motorH) > 1:
                        #   node_CtlCenter_globals.lsHistory_motorH[-1][SeqMapField.END_NODE.name]=int(node_current)
                    # else:   #Stand alone mode
                    #     node_CtlCenter_globals.lsHistory_motorH.append(dicInfo_local_org)
            else:
              rospy.loginfo_throttle(10,node_CtlCenter_globals.dicTargetPosFeedBack)
        #end of if len(node_CtlCenter_globals.dicTargetPosFeedBack) > 0:
        dictDistanceV = GetDistanceV()
        #cur_spd= calculate_speed_fromRPM(spd_cur_H)
        #targetPOS_H = node_CtlCenter_globals.dicTargetPos.get(str(ModbusID.MOTOR_H.value),MIN_INT)
        # targetPOS_H = node_CtlCenter_globals.dicTargetPos.get(str(ModbusID.MOTOR_H.value),0)
        # targetpules_left = abs(targetPOS_H-cur_pos_h)
        # target_mm_left=pulseH_to_distance(targetpules_left)
        #if tiltStutus == TRAY_TILT_STATUS.TiltFace and targetPOS_H != MIN_INT and len(dictDistanceBox) > 0:
        if tiltStutus == TRAY_TILT_STATUS.TiltFace:
            isNeedtoReduceSlow = spd_cur_H > DEFAULT_RPM_SLOW * 1.1
            isNeedtoReduceSlower = spd_cur_H > DEFAULT_RPM_SLOWER * 1.1
            if len(dictDistanceV) > 0:
                #라이다 세이프티 컨트롤 부.
                #1.정상운행일때 장애물이 감지되면 느려지게 하기
                #distanceBox = try_parse_float(dictDistanceV.get(LIDAR_DISTANCE_PARAMS.distance.name))
                distanceV = try_parse_float(dictDistanceV[0])
                #objWidth = try_parse_float(dictDistanceV.get(LIDAR_DISTANCE_PARAMS.width.name))
                #lastseenBox = dictDistanceV.get(MonitoringField.LASTSEEN.name)
                lastseenVTs = try_parse_float(dictDistanceV[-1])
                lastseenV_datetime = datetime.fromtimestamp(lastseenVTs)
                time_differenceSecond = getDateTime().timestamp() - lastseenV_datetime.timestamp()
                #주행목표까지 1미터 이하, 장애물이 목표치보다 앞에 있고 속도는 300 보다 빠르고 장애물 마지막 감지시간이 0.5초 이내인 경우
                if distanceV < 1 and isNeedtoReduceSlower and time_differenceSecond < 0.3 and isTimeExceeded(GetLastBalanceTimeStamp(),1000):
                    #주행일때 - 1m 이하로 남았는데 계속 장애물이 감지되면 속도를 1로 만든다. (거의 멈춤)
                    rospy.loginfo(f'정지-현재속도:{spd_cur_H},감지된물체거리:{distanceV},TS:{getDateTime().timestamp()}')
                    dicSpd = getMotorSpeedDic(ModbusID.MOTOR_H.value, True, ALMOST_ZEROINT, DEFAULT_ACC, calculate_deceleration_factor(spd_cur_H))
                    SendCMD_Device([dicSpd])            
                    UpdateLastBalanceTimeStamp()
                    #ClearDistanceV()
                    TTSAndroid(TTSMessage.OBSTACLE_STOP.value,5000)
                    SetSuspend(True)

                #목표까지 1미터 이상 남았고, 장애물과 거리가 1.5m 이내이고 속도는 300 보다 빠르고 장애물 마지막 감지시간이 0.5초 이내인 경우
                elif distanceV < 1.5 and isNeedtoReduceSlow and time_differenceSecond < 0.3 and isTimeExceeded(GetLastBalanceTimeStamp(),1000):
                    rospy.loginfo(f'감속1,현재속도:{spd_cur_H},감지된물체거리:{distanceV},TS:{getDateTime().timestamp()}')
                    dicSpd = getMotorSpeedDic(ModbusID.MOTOR_H.value, True, DEFAULT_RPM_SLOW, ACC_MOVE_H,DECC_MOVE_H)
                    if node_CtlCenter_globals.last_detect_status is not None and node_CtlCenter_globals.last_detect_status["type"] != 'wall':
                      SendCMD_Device([dicSpd])
                      UpdateLastBalanceTimeStamp()
                      TTSAndroid(TTSMessage.OBSTACLE_SLOW.value,5000)
                      #ClearDistanceV()
                      SetSuspend(True)
                    #주행일때 - 1~2m 남았는데 속도가 RPM 보다 빠르면 속도를 변경한다.
                    #UpdateLidarDistanceBoxTimeStamp()
                #느려진 상태에서 2m 이내에 장애물이 없는 상태가 n초 이상 지속되면 속도 원복
                #
                elif not isNeedtoReduceSlow and IsSuspended() and time_differenceSecond > 3 :
                #elif not isNeedtoReduceSlow and isTimeExceeded(GetLastBalanceTimeStamp(),5000) and IsSuspended():
                    rospy.loginfo(f'속도원복-현재속도:{spd_cur_H},TS:{getDateTime().timestamp()}')
                    dicSpd = getMotorSpeedDic(ModbusID.MOTOR_H.value, True, SPD_MOVE_H, ACC_MOVE_H,DECC_MOVE_H)
                    SendCMD_Device([dicSpd])
                    UpdateLastBalanceTimeStamp()    
          
        # if curTilt != TRAY_TILT_STATUS.TiltTrayCenter and modbusIDStr_H in node_CtlCenter_globals.dicTargetPos:
        #     iTargetH = int(node_CtlCenter_globals.dicTargetPos[modbusIDStr_H])
        #     diffVal = abs(cur_pos_h - iTargetH)
        #     if diffVal < TILT_TIMING:
        #         TiltTrayCenter()
        return    
    if isActivatedMotor(ModbusID.TELE_SERV_MAIN.value):
        #서빙암 세이프티 캘리브레이션. 평소에는 불필요.
        #if len(node_CtlCenter_globals.dfDistanceArm) == 0 and isDistanceVReceived:
        if not isFileExist(node_CtlCenter_globals.strFileDistanceArm):
            lastLD_Raw = GetDistanceV()
            if len(lastLD_Raw)>0:
                dictTmp = {}
                dictTmp[DISTANCE_V.pulseV.name] = cur_pos_SrvTele
                dictTmp[DISTANCE_V.distanceSTD.name] = round(float(lastLD_Raw[1]),4)
                dictTmp[DISTANCE_V.distanceLD.name] = round(float(lastLD_Raw[0]),4)
                dictTmp[DISTANCE_V.timestampLD.name] = getCurrentTime()
                node_CtlCenter_globals.lsDistanceArmDicArr.append(dictTmp)

    
    #트레이 리프트 관련 제어
    if isActivatedMotor(ModbusID.MOTOR_V.value):
    #if GetWaitConfirmFlag() and isActivatedMotor(ModbusID.MOTOR_V.value):
        rpmLift = int(GetItemsFromModbusTable(ModbusID.MOTOR_V,MonitoringField.CUR_SPD),0)
        angle_y = node_CtlCenter_globals.dicARD_CARRIER.get(DataKey.Angle_Y.name,0)
        move_level = node_CtlCenter_globals.dicARD_CARRIER.get(DataKey.MOVE_LEVEL.name,0)
        if not isRealMachine and move_level is None:
            move_level = 0
        
        #리프트 모터 캘리브레이션, 평소에는 불필요
        if len(node_CtlCenter_globals.dfDistanceV) == 0 and isDistanceVReceived:
            lastLD_Raw = GetDistanceV()
            dictTmp = {}
            dictTmp[DISTANCE_V.pulseV.name] = cur_pos_lift
            dictTmp[DISTANCE_V.distanceSTD.name] = round(float(lastLD_Raw[1]),4)
            dictTmp[DISTANCE_V.distanceLD.name] = round(float(lastLD_Raw[0]),3)
            dictTmp[DISTANCE_V.timestampLD.name] = getCurrentTime()
            lsAruco = filter_recent_data(ARUCO_RESULT_FIELD.LASTSEEN.name,GetArucoMarkerInfo(2),0.5)
            aruco_Z = -1
            if len(lsAruco) > 0:
                dicAruco = lsAruco[0]
                aruco_Z = dicAruco[ARUCO_RESULT_FIELD.Z.name]
            dictTmp[DISTANCE_V.aruco_Z.name] = aruco_Z
            node_CtlCenter_globals.lsDistanceVDicArr.append(dictTmp)
            #rospy.loginfo(dictTmp)
            
        if isTimeExceeded(GetLastBalanceTimeStamp(), MODBUS_EXECUTE_DELAY_ms):
            # 트레이 흔들림 감지시 제어
            if abs(rpmLift) > (SPD_LIFT / 2) and move_level > 40.0 and isRealMachine:
              dicSpd = getMotorSpeedDic(ModbusID.MOTOR_V.value, True, ALMOST_ZEROINT, ACC_DECC_SMOOTH,ACC_DECC_SMOOTH)
              SendCMD_Device([dicSpd])
              TTSAndroid(TTSMessage.SHAKE_TRAY_DETECTED.value)
              UpdateLastCmdTimeStamp()
            elif abs(rpmLift) < 10 and move_level < 5 and isRealMachine:
              dicSpd = getMotorSpeedDic(ModbusID.MOTOR_V.value, True, SPD_LIFT, ACC_DECC_SMOOTH,ACC_DECC_SMOOTH)
              SendCMD_Device([dicSpd])
              #TTSAndroid(TTSMessage.SHAKE_TRAY_OK.value)
              UpdateLastCmdTimeStamp()
            
            #세이프티 커튼            
            if isDistanceVReceived and rpmLift > 0:
            #if not isTimeExceeded(node_CtlCenter_globals.lsDistanceLastSeen, 0.1) and isDistanceVReceived and rpmLift > 0:
                # lastLD_Raw = GetDistanceV()
                # distancePoints = int(lastLD_Raw[2])
                # distanceSTD = round(float(lastLD_Raw[1]),4)
                # distanceLD = round(float(lastLD_Raw[0]),3)
                # if distanceLD < 0.65 and distanceSTD > 0 and distancePoints > 2 and targetPOS_V == MIN_INT:
                    height_difference = calculate_relative_height_difference(node_CtlCenter_globals.last_detect_3d,90-angle_y)
                    # lidar_tilt_deg=90-angle_y
                    # tilt_rad = np.radians(lidar_tilt_deg)
                    # height_difference = distanceLD * np.cos(tilt_rad)
                    if height_difference is not None and targetPOS_V == pot_cur_lift:
                      cur_distance_fromGND = GetRangeV(cur_pos_lift,DISTANCE_V.pulseV.name, DISTANCE_V.distanceLD.name)
                      target_distance_fromGND = cur_distance_fromGND - height_difference + 0.15 #마진이 10
                      target_pulse = round(GetRangeV(target_distance_fromGND,DISTANCE_V.distanceLD.name, DISTANCE_V.pulseV.name))
                      node_CtlCenter_globals.dicTargetPos[str(ModbusID.MOTOR_V.value)] = target_pulse
                      TTSAndroid('물체가 감지되었습니다',1000)
                      #StopMotor(ModbusID.MOTOR_V.value)
                      SendCMD_Device([getMotorMoveDic(ModbusID.MOTOR_V.value,True,target_pulse,DEFAULT_RPM_MID, ACC_DECC_SMOOTH,ACC_DECC_SMOOTH)])
                      UpdateLastBalanceTimeStamp()
                      # rospy.loginfo(f'트레이 세이프티 정지!:{lastLD_Raw}')
            
            #리프트 하강시 중간 지점에서부터 도어 자동열림 기능 구현
            if rpmLift > 0 and not GetWaitConfirmFlag():
                openTargetPos = round(pot_cur_lift * DOOROPEN_PERCENT)
                #if cur_pos_lift > openTargetPos and node_CtlCenter_globals.robot.get_current_state() != Robot_Status.onServing:
                if cur_pos_lift > openTargetPos:
                    if curNode == node_KITCHEN_STATION:
                        SetWaitConfirmFlag(False,AlarmCodeList.JOB_COMPLETED)
                    else:
                        SetWaitConfirmFlag(True,AlarmCodeList.WAITING_USER)
                        LightTrayCell(TraySector.Cell1.value,LightBlink.Normal.value,LightColor.BLUE.value)                        
                    # if doorStatus == TRAYDOOR_STATUS.CLOSED:
                    #     rospy.loginfo(f"도어현재포지션:오픈포지션:POT-{cur_pos_lift}:{openTargetPos}:{pot_cur_lift}")
                    #     if dfReceived is None:
                    #         rospy.loginfo(f"dfReceived not found")
                    #         #추후 중량값 들어오고 있는 셀로 고치자.
                    #         if not isScanOn:
                    #             DoorOpen()
                    #         LightTrayCell(TraySector.Cell1.value,LightBlink.Normal.value,LightColor.BLUE.value)
                    #     else:
                    #         SetCurrentNode(dfReceived.iloc[-1][APIBLB_FIELDS_TASK.startnode.name])
                    #         dicFirst = dfReceived.iloc[0]
                    #         taskid_current = dicFirst[APIBLB_FIELDS_TASK.taskid.name]
                    #         dicTaskInfo = GetTaskChainHead(APIBLB_FIELDS_TASK.taskid.name, taskid_current, True)
                    #         trayrack = dicTaskInfo.get(APIBLB_FIELDS_TASK.trayrack.name)                            
                    #         if trayrack is None or GetCurrentNode() == node_KITCHEN_STATION:
                    #             DoorOpen()
                    #         else:
                    #             trayRackID = RackID.from_name_or_value(trayrack,True)
                    #             trayidx = trayRackID.value
                    #             #trayidx = (int)(trayrack[1:]) - 1
                    #             DoorOpen(trayidx)
                    #         #RemoveDF(curTargetTable)
                        UpdateLastBalanceTimeStamp()
                    return    
            #리프트 상승시 도어 열려있는 경우 닫힘 기능 구현
            if doorStatus == TRAYDOOR_STATUS.OPENED and rpmLift < -100 and not isScanOn:
                DoorClose()
                LightTrayCell(TraySector.Cell1.value,LightBlink.Solid.value,LightColor.OFF.value)
                UpdateLastBalanceTimeStamp()
                #TiltArucoScan()
                rospy.loginfo(f"도어닫음/상태:{doorStatus},속도:{rpmLift}")
        return    
    #targetPOS_arm1
    isArm1Expanded = abs(pot_cur_arm1 - cur_pos_arm1) < roundPulse or DI_POT_arm1
    isArm2Expanded = abs(pot_cur_arm2 - cur_pos_arm2) < roundPulse or DI_POT_arm2
    
    isArm1Folded = abs(not_cur_arm1 - cur_pos_arm1) < roundPulse or DI_NOT_arm1
    isArm2Folded = abs(not_cur_arm2 - cur_pos_arm2) < roundPulse or DI_NOT_arm2
    
    #접는중 - 밸런스텔레스코픽이 수축되고 난 후 펜딩되고 있던 1,2관절 속도를 둘다 높임
    #isPendingArm2 = motorStatus_arm2 == STATUS_MOTOR.PENDING_CCW or motorStatus_arm2 == STATUS_MOTOR.PENDING_CW
    #isPendingArm1 = motorStatus_arm1 == STATUS_MOTOR.PENDING_CCW or motorStatus_arm1 == STATUS_MOTOR.PENDING_CW
    isTimeOK = isTimeExceeded(GetLastBalanceTimeStamp(), MODBUS_EXECUTE_DELAY_ms)
    if isActivatedMotor(ModbusID.ROTATE_SERVE_360.value) and node_CtlCenter_globals.robot.get_current_state() == Robot_Status.cali_tray:
        DI_POT_tray,DI_NOT_tray,DI_HOME_tray,SI_POT_tray = GetPotNotHomeStatus(ModbusID.ROTATE_SERVE_360)
        #rospy.loginfo(DI_POT_tray,DI_NOT_tray,DI_HOME_tray)
        if DI_HOME_tray:
            #StopMotor(ModbusID.ROTATE_SERVE_360.value, DECC_540)
            StopMotor(ModbusID.ROTATE_SERVE_360.value, 500)
            time.sleep(1)
            dicLoc = getMotorLocationSetDic(ModbusID.ROTATE_SERVE_360.value, 0)
            cmdpos_360,cur_pos_360 =GetPosServo(ModbusID.ROTATE_SERVE_360)
            rospy.loginfo(f"Tray Cali OK at pulse :{cur_pos_360}")            
            SendCMD_DeviceService([dicLoc]) 
            node_CtlCenter_globals.robot.trigger_complete_calibration_tray()
            
    if isActivatedMotor(ModbusID.ROTATE_MAIN_540.value) and node_CtlCenter_globals.robot.get_current_state() == Robot_Status.cali_mainRotate:
        DI_POT_540,DI_NOT_540,DI_HOME_540,SI_POT_540 = GetPotNotHomeStatus(ModbusID.ROTATE_MAIN_540)
        #rospy.loginfo(DI_POT_tray,DI_NOT_tray,DI_HOME_tray)
        if DI_HOME_540:
            StopMotor(ModbusID.ROTATE_MAIN_540.value, 800)
            time.sleep(1)
            cmd_540,pos_540 =GetPosServo(ModbusID.ROTATE_MAIN_540)
            rospy.loginfo(f"Main Rotate Cali OK at pulse :{pos_540}")
            dicLoc = getMotorLocationSetDic(ModbusID.ROTATE_MAIN_540.value, 0)            
            SendCMD_DeviceService([dicLoc]) 
            node_CtlCenter_globals.robot.trigger_complete_calibration_mainRotate()
            
    # if (
    #     isFinishedMotor_teleBalance
    #     and isFinishedMotorArm1
    #     and isFinishedMotorArm2
    #     and targetPOS_arm1 != MIN_INT
    #     and isTimeOK
    #     and isArm1Expanded
    #     and not isArm1ReachedToTarget
    #     and spd_cur_srvTele < -10
    # ):  #밸런싱암 폴딩 완료 이후 밸런싱암 폴딩 로직
    #     lsArmSpdChange =[]
    #     targetRPM_arm2 = SPD_EXTEND_ARM2
    #     targetRPM_arm1 = SPD_ARM1
    #     cmdSpdChange1 = getMotorMoveDic(ModbusID.BAL_ARM1.value, True,targetPOS_arm1, abs(targetRPM_arm1), ACC_ARM1, DECC_ARM1)
    #     # node_CtlCenter_globals.dicTargetRPM[str(
    #     #     ModbusID.BAL_ARM1.value)] = str(abs(targetRPM_arm1))
    #     newTime=GetTimeFromRPM(ModbusID.BAL_ARM1, targetPOS_arm1,targetRPM_arm1)
    #     if abs(cur_pos_BalTele) > roundPulse or newTime <= 0:
    #         StopEmergency(ALM_error.ARM_BALANCING_ERROR.value)
    #         SendInfoHTTP(f'{ALM_error.ARM_BALANCING_ERROR.value},cur_pos_BalTele:{cur_pos_BalTele},newTime={newTime}')
    #         return        
    #     newRpmArm2 = GetRPMFromTime(ModbusID.BAL_ARM2,targetPOS_arm2,newTime )
    #     newTimeTeleMain = newTime+1.0
    #     newRPMSrvArm = GetRPMFromTime(ModbusID.TELE_SERV_MAIN,targetPOS_srv, newTimeTeleMain)
    #     cmdSpdChange2 = getMotorMoveDic(ModbusID.BAL_ARM2.value, True, targetPOS_arm2, abs(newRpmArm2), ACC_ARM2_FOLD, DECC_ARM2)
    #     # node_CtlCenter_globals.dicTargetRPM[str(
    #     #     ModbusID.BAL_ARM2.value)] = str(abs(targetRPM_arm2))
    #     cmdSpdSrvArm = getMotorSpeedDic(ModbusID.TELE_SERV_MAIN.value, True, abs(newRPMSrvArm), ACC_ST, DECC_ST)
    #     # node_CtlCenter_globals.dicTargetRPM[str(
    #     #     ModbusID.TELE_SERV_MAIN.value)] = str(abs(targetRPM_arm2))
    #     #node_CtlCenter_globals.lastSpdArm1TimeStamp = getDateTime()
    #     sMsg = f"Spd Change Time Stamp:{GetLastBalanceTimeStamp()}, newRPM:{newRPMSrvArm},newTime:{newTimeTeleMain}"
    #     rospy.loginfo(sMsg)
    #     SendInfoHTTP(sMsg)
    #     lsArmSpdChange.append(cmdSpdSrvArm)
    #     lsArmSpdChange.append(cmdSpdChange1)
    #     lsArmSpdChange.append(cmdSpdChange2)
    #     SendCMD_Device(lsArmSpdChange)
    #     UpdateLastBalanceTimeStamp()
    #     PrintCurrentPos()
    #     return

    # # 전개중 - 밸런싱암 전개 완료 후 정지 상태의 밸런싱 텔레스코프 전개.
    # if (
    #     isArm2Expanded
    #     and isArm1Expanded
    #     and targetPOS_balTele != MIN_INT
    #     and isTimeExceeded(GetLastBalanceTimeStamp(), MODBUS_EXECUTE_DELAY_ms)
    #     and isFinishedMotor_teleBalance
    #     #and not isArm1ReachedToTarget
    #     and spd_cur_srvTele > 10
    # ):
    #     # if not isArm1Expanded or not isArm2Expanded:
    #     #     StopEmergency(ALM_error.ARM_BALANCING_ERROR.value)
    #     #     return
        
    #     targetRPM_telBal = SPD_BALTELE
    #     newTimeBalTele=GetTimeFromRPM(ModbusID.TELE_BALANCE, targetPOS_balTele,targetRPM_telBal)
    #     newRpmSrv_Arm = GetRPMFromTime(ModbusID.TELE_SERV_MAIN,targetPOS_srv,newTimeBalTele )
    #     #cmdSpdChangeBal = getMotorSpeedDic(ModbusID.TELE_BALANCE.value,True,targetRPM_telBal,DEFAULT_ACC,DEFAULT_DECC)
    #     cmdSpdChangeBal = getMotorMoveDic(ModbusID.TELE_BALANCE.value,True,targetPOS_balTele, targetRPM_telBal,ACC_BT,DECC_BT)
    #     cmdSpdChangeSrv = getMotorSpeedDic(ModbusID.TELE_SERV_MAIN.value,True,round(newRpmSrv_Arm * 0.7),ACC_ST,DECC_ST)
    #     SendCMD_Device([cmdSpdChangeBal,cmdSpdChangeSrv])
    #     UpdateLastBalanceTimeStamp()
    #     sMsg = f"밸런싱암전개 목표시간2:{newTimeBalTele:.1f},서빙암 변경RPM:{newRpmSrv_Arm}"
    #     SendInfoHTTP(sMsg)
    #     rospy.loginfo(sMsg)
    #     return

def GetScanPath():
    lsTableInfo = []
    #[1, 13, 22]
    lsEdgeTables = find_dead_end_nodes_from_file(node_CtlCenter_globals.strFileShortCut)
    #GetTableFromNode()
    # if not isFileExist(file_path):
    # with open(file_path, 'w') as file:
    for node, connections in node_CtlCenter_globals.StateInfo.items():
        for connected_node in connections:
            if connected_node != -1:
                
                #file.write(f"{node} {connected_node} 1\n")
                dicTableTmp = {TableInfo.TABLE_ID.name : connected_node,
                              TableInfo.NODE_ID.name : connected_node,
                              TableInfo.SERVING_ANGLE.name : -1,
                              TableInfo.SERVING_DISTANCE.name : -1,
                              TableInfo.MARKER_ANGLE.name : -1,
                              TableInfo.MOVE_DISTANCE.name : -1,}
                lsTableInfo.append(dicTableTmp)
                AppendTableList(connected_node)
    
def save_graph_to_file(graph_dict, file_path):
    #초기화 작업이므로 기존 테이블을 모두 삭제한다.
    lsTableInfo = []
    # if not isFileExist(file_path):
    # with open(file_path, 'w') as file:
    for node, connections in graph_dict.items():
        for connected_node in connections:
            if connected_node != -1:
                #file.write(f"{node} {connected_node} 1\n")
                dicTableTmp = {TableInfo.TABLE_ID.name : connected_node,
                              TableInfo.NODE_ID.name : connected_node,
                              TableInfo.SERVING_ANGLE.name : -1,
                              TableInfo.SERVING_DISTANCE.name : -1,
                              TableInfo.MARKER_ANGLE.name : -1,
                              TableInfo.MOVE_DISTANCE.name : -1,}
                lsTableInfo.append(dicTableTmp)
                AppendTableList(connected_node)
    
    #가상 테이블 정보를 구성해서 저장.
    #SERVING_ANGLE 값 등이 0 보다 작은 경우 탐색모드로 구동하여 저장할 것.
    df_unique = pd.DataFrame(lsTableInfo)
    df_unique.to_csv(strFileTableNodeEx, index=False, sep=sDivTab) 

def HomeRequest():
    dictParam = {APIBLB_FIELDS_TASK.robotips.name:IP_MASTER,
                APIBLB_FIELDS_TASK.extraserviceruns.name:0,
                APIBLB_FIELDS_TASK.SPstatus.name:5
    }
    paramStr=urllib.parse.urlencode(dictParam)
    bReturn,strResult = API_call(svrIP=BLB_SVR_IP_DEFAULT,port=BLB_SVR_PORT_DEFAULT,serviceName=APIBLB_METHODS_GET.robot_serviceRunokno_action.name, fieldValue=paramStr)
    rospy.loginfo(f"API RESULT:{bReturn},{strResult}")
    
                    
def GenerateServingTableList():
    # 경로지시정보 리스트가 비어있는지 확인
    dicTask = None
    lnListBLB = 0 if node_CtlCenter_globals.listBLB == None else len(node_CtlCenter_globals.listBLB)
    lnTables = 0 if GetTableList() == None else len(GetTableList())
    #curTable,curNode = GetCurrentTargetTable()
    lsCurTable,curNode = GetCurrentTableNode()
    #defaultTable = GetTableFromNode(node_KITCHEN_STATION)
    
    # if lnListBLB == 0 and lnTables == 0 and curTable != defaultTable:
    #     node_CtlCenter_globals.listBLB = []
    #     node_CtlCenter_globals.listTable = []
    #     node_CtlCenter_globals.listTable.append(defaultTable)
    
    #서빙해야할 테이블 리스트
    #lnTables = len(GetTableList())
    isTimeExceededJob = isTimeExceeded(GetLastCmdTimeStamp(), MODBUS_EXECUTE_DELAY_ms)
    tableTarget_local = node_NOT_TABLE
    runningMotors = getRunningMotorsBLB()
    #서빙 테이블 오더에 따라 제어정보 생성
    if len(runningMotors) == 0 and isTimeExceededJob and lnListBLB==0: # 동작중인 모터가 없을때
        # 현재 서버에서 준 리스트 Job 을 조회한다
        if lnTables == 0:
            if IsEnableSvrPath():
                table_target = GetTableTarget()
                dfReceived = GetDF(table_target)
                if dfReceived is not None:
                    dicFirst = dfReceived.iloc[-1]
                    dicLast = dfReceived.iloc[-1]
                    workstatusFirst = dicLast.get(APIBLB_FIELDS_TASK.workstatus.name, None)
                    workstatusLast = dicLast.get(APIBLB_FIELDS_TASK.workstatus.name, None)
                    if workstatusLast is None or int(workstatusLast) != APIBLB_STATUS_TASK.Completed.value:
                        if workstatusFirst is not None and int(workstatusLast) != APIBLB_STATUS_TASK.Ideal.value:
                            rospy.loginfo_throttle(20, '작업이 완료되지 않았습니다')
                            return
                dicTask = GetTaskChainHead(APIBLB_FIELDS_TASK.taskrunok.name, 1, False)
                if len(dicTask) > 0:
                    table_TaskType = str(dicTask.get(APIBLB_FIELDS_TASK.tasktype.name))
                    table_targetTask = str(dicTask.get(APIBLB_FIELDS_TASK.workname.name))
                    taskid = str(dicTask.get(APIBLB_FIELDS_TASK.taskid.name))
                    
                    if is_equal(table_targetTask ,lsCurTable):
                        SetTaskCompleted(taskid,2)
                        return
                    if table_targetTask.startswith('T'):
                        table_targetTask = int(table_targetTask[1:])
                    
                    if table_targetTask is not None:
                    #if table_targetTask is not None and node_CtlCenter_globals.table_target != str(table_targetTask):
                        # if GetDF(table_targetTask) is None:
                        #   API_SetOrderTable(table_targetTask,table_TaskType,taskid)
                        AppendTableList(table_targetTask)
                        return
            
            if dicTask == None or len(dicTask) == 0:
              if GetWaitConfirmFlag() == False:
                if not IsEnableSvrPath():
                    return
                HomeCall()
              return

        lnTables = 0 if GetTableList() == None else len(GetTableList())
        #모터동작지시가 없지만 서빙 테이블 큐는 차 있을때
        if lnListBLB == 0 and lnTables > 0:
            tableTarget_local = PopTablelist(idx=0)
            SetTableTarget(tableTarget_local)
            table_target = GetTableTarget()
            table_current = GetLastTableHistory()
            if tableTarget_local == table_current:
            #if table_target == tableTarget_local:
              #현재랑 테이블 정보가 연속으로 오면 건너 뛴다.
              return
            rospy.loginfo(GetTableList())
            nodeTarget_local = SetCurrentTable(tableTarget_local, None)
            # if nodeTarget_local is None:
            #     #node_CtlCenter_globals.dfLinkPosInfo = pd.read_csv(strJsonLinkPosInfo, delimiter=sDivTab)
            #     lsEdgeTables = find_dead_end_nodes_from_file(node_CtlCenter_globals.strFileShortCut)
            #     if len(node_CtlCenter_globals.dfLinkPosInfo) == len(lsEdgeTables) * 2:
            #         SaveTableNodeInfo()
            #         ClearTableList()
            #         rospy.loginfo('테이블 스캐닝을 완료했습니다!')
            #         return
                
            #     rospy.loginfo('테이블 스캐닝 모드')
            #     lsSeqNodeScan = []
            #     #if not isFileExist(strFileTableNodeEx):
            #     AppendTableList(tableTarget_local)
            #     lsEdgeNodes = find_dead_end_nodes_from_file(node_CtlCenter_globals.strFileShortCut)
            #     if len(lsEdgeNodes) > 0 and lsEdgeNodes[0] == curNode:
            #         node1 = lsEdgeNodes.pop(0)
            #         lsEdgeNodes.append(node1)
                
            #     dicTmp90 = GetDicRotateMotorTray(90)
            #     dicTmp270 = GetDicRotateMotorTray(270)
            #     #node_CtlCenter_globals.listBLB.append([dicTmp])
            #     #listBLB.insert(0,[dicTmp])
            #     #SendCMD_Device([dicTmp2])
            #     iCnt = 0
            #     for nodeID in lsEdgeNodes:
            #         if nodeID == curNode:
            #             continue
            #         listSeqMapOptimized,listSeqMapOrg = getSeqMap(curNode,nodeID)
            #         lsSeqNodeScan.extend(listSeqMapOptimized)
            #         curNode = nodeID
            #     for lsDic in lsSeqNodeScan:
            #         iCnt += 1
            #         if iCnt % 2 == 0:
            #             node_CtlCenter_globals.listBLB.append([dicTmp90])
            #         else:
            #             node_CtlCenter_globals.listBLB.append([dicTmp270])
            #         node_CtlCenter_globals.listBLB.append(lsDic)                    
            #     rospy.loginfo(node_CtlCenter_globals.listBLB)
            #     #     lsEdgeTables = []
            #     #     for edgeNode in lsEdgeNodes:
            #     #       edgeTable = GetTableFromNode(edgeNode)
            #     #       lsEdgeTables.append(edgeTable)
            #     #     #lsEdgeTables.append(tableTarget_local)
            #     #     lsEdgeTables.sort()
            #     #     AppendTableList(lsEdgeTables)
            #     #     AppendTableList(tableTarget_local)
            #     #   else:
            #     #     save_graph_to_file(node_CtlCenter_globals.StateInfo, node_CtlCenter_globals.strFileCross)
            #     return
            #테이블 스캐닝 모드 끝. 제대로 트리거 만들어보자
            
              #cross.txt 정보에서 shortcut.txt 를 생성한다.
              #shortcut.txt 에는 cross 노드의 인접한 노드들의 정보를 기재한다.
              #인접한 노드들을 모두 AppendTableList한다.
              #거리는 -1 로 생성하며 -1 인 경우 300RPM 으로 주행한다. (DEV 에서는 구찮으니까 SPD_FAST 로 하자)
            dicTagretTableInfo = getTableServingInfo(tableTarget_local)
            ClearArucoTable()
            ArucoLastRecordClear()
            CamControl(False)            
            #현재 목적지 테이블을 세팅한다. 다음 오더를 시작할때 갱신된다
            infoTABLE_ID = dicTagretTableInfo.get(TableInfo.TABLE_ID.name, None)
            infoNODE_ID = dicTagretTableInfo.get(TableInfo.NODE_ID.name, None)
            infoSERVING_ANGLE = dicTagretTableInfo.get(TableInfo.SERVING_ANGLE.name, None)
            infoSERVING_DISTANCE = dicTagretTableInfo.get(TableInfo.SERVING_DISTANCE.name, None)
            infoMARKER_ANGLE = dicTagretTableInfo.get(TableInfo.MARKER_ANGLE.name, None)
            infoHEIGHT_LIFT = dicTagretTableInfo.get(TableInfo.HEIGHT_LIFT.name, None)
            
            if IsEnableSvrPath():
                dfChk = GetDF(tableTarget_local)
                if dfChk is None or len(dfChk) == 0:
                    #TODO : 주석
                    if GetTableTarget() != tableTarget_local:
                        InsertTableList(tableTarget_local,0)
                    #dicTask = GetTaskChainHead(APIBLB_FIELDS_TASK.taskrunok.name,1)
                    tableNo = try_parse_int(tableTarget_local,MIN_INT)
                    if tableNo == MIN_INT:
                        tableNo =tableTarget_local
                    else:
                        tableNo = f'T{tableTarget_local}'
                    dicTask = GetTaskChainHead(APIBLB_FIELDS_TASK.workname.name,tableNo, True)
                    if len(dicTask) > 0:
                        taskid = dicTask.get(APIBLB_FIELDS_TASK.taskid.name)
                        table_targetTask = dicTask.get(APIBLB_FIELDS_TASK.workname.name)
                        type_targetTask = int(dicTask.get(APIBLB_FIELDS_TASK.tasktype.name,MIN_INT))
                        previous_tasktype = get_last_field_value(node_CtlCenter_globals.dfLast, APIBLB_FIELDS_TASK.tasktype.name)
                        #회수 혹은 현금 Job 일때는 범블비에서 직접 셀 할당을 해야 한다.
                        if type_targetTask == APIBLB_TASKTYPE.CollectingEmptyPlattes.value or type_targetTask == APIBLB_TASKTYPE.CashPay.value:
                            rackIDCur,empty_cells = GetRackID_Empty()
                            if empty_cells < 2 or previous_tasktype == str(APIBLB_TASKTYPE.ServingTask.value):
                            # if rackIDCur == MIN_INT:
                                if empty_cells < 2 :
                                    sMsg = "음식이 있을때는 회수할 수 없습니다"
                                else:
                                    sMsg = "서빙 후 회수할 수 없습니다"
                                    
                                TTSAndroid(sMsg)
                                #rospy.loginfo(sMsg)
                                HomeRequest()
                                #StopEmergency(sMsg)
                                SetTableTarget(HOME_TABLE)
                                node_CtlCenter_globals.dfTaskChainInfo[APIBLB_FIELDS_TASK.taskrunok.name] = 2
                                HomeCall(True)
                                #node_CtlCenter_globals.dfTaskChainInfo.drop(node_CtlCenter_globals.dfTaskChainInfo, inplace=True)
                            else:
                                rackIDNew = rackIDCur.name
                                dicTask[APIBLB_FIELDS_TASK.trayrack.name] = rackIDNew
                                API_SetRackID(taskid,type_targetTask,rackIDNew)
                                SetTaskRackStatus(taskid,rackIDNew)
                        elif type_targetTask == APIBLB_TASKTYPE.HomeEmergencyCall.name:
                          SetWaitConfirmFlag(False,AlarmCodeList.OK)
                        #rackID_currentInfo = dicTask.get(APIBLB_FIELDS_TASK.trayrack.name)
                        #SetRackIDStatus(rackID_currentInfo, 1)
                        #if len(tasktype_currentInfo) < 2:
                        API_SetOrderTable(table_targetTask,type_targetTask,taskid)
                    else:
                        API_SetOrderTable(tableTarget_local,APIBLB_TASKTYPE.ServingTask.value,0)
                    #InsertTableList(tableTarget_local,0)
                    UpdateLastCmdTimeStamp()
                    return
                
                #TODO : 주석
                if nodeTarget_local == None:
                    nodeTarget_local = tableTarget_local
                    node_CtlCenter_globals.lsNodeHistory.append([nodeTarget_local,nodeTarget_local])
                
                #20250312-샤누이사 요청, HOME 으로 들어올때 플래그 세팅.
                if infoTABLE_ID == HOME_TABLE:
                    HomeRequest()
            #현재와 타겟이 같으면 무시하는 로직 추가
            if tableTarget_local != node_NOT_TABLE:# and tableTarget_local != node_KITCHEN_STATION:
                rospy.loginfo(f"경로생성:{curNode}->{nodeTarget_local}")
            #if nodeTarget_local != curNode:
                dfReceived = GetDF(tableTarget_local)
                if curNode != nodeTarget_local:
                  lsSeqNode, lsAllNodes = getSeqMap(curNode, nodeTarget_local)
                  #내 알고리즘으로 만들어진 로직
                  dfOrg_local = pd.DataFrame(lsSeqNode)
                else:
                  dicMap = {}   #같은 노드에서 테이블만 바뀔 경우 수기로 지시정보 만들어줌
                  dicMap[SeqMapField.START_NODE.name] = nodeTarget_local
                  dicMap[SeqMapField.START_STATUS.name] = -1
                  dicMap[SeqMapField.END_NODE.name] = nodeTarget_local
                  dicMap[SeqMapField.END_STATUS.name] = -1
                  dicMap[SeqMapField.DIRECTION.name] = 'N'
                  dicMap[SeqMapField.CROSS_STATUS.name] = {}
                  dicMap[SeqMapField.DISTANCE.name] = 0
                  lsSeqNode = [dicMap]
                  dfOrg_local = pd.DataFrame(lsSeqNode)
                  rospy.loginfo(dfOrg_local)
                if dfReceived is not None and not dfReceived.empty:
                    dicFirst = dfReceived.iloc[0]
                    dicLast = dfReceived.iloc[-1]
                    current_node = dicFirst[APIBLB_FIELDS_TASK.startnode.name]
                    last_node = dicLast[APIBLB_FIELDS_TASK.startnode.name]
                    #lsSeqNode, lsAllNodes = getSeqMap(current_node, last_node)
                    #dfOrg_local = pd.DataFrame(lsSeqNode)
                    dfOrg_local.to_csv(node_CtlCenter_globals.strGetSeq, index=False, sep=sDivTab)
                    dfReceived.to_csv(node_CtlCenter_globals.strRecvDF, index=False, sep=sDivTab)                 
                    dfReceived_robot = dfReceived.iloc[:-1]
                    #df_svr  = process_robot_path(dfReceived_robot)
                    PrintDF(dfReceived_robot)
                    rospy.loginfo(f'목표테이블:{tableTarget_local},현재위치:{GetCurrentNode()},원래경로:{curNode}->{nodeTarget_local},서버경로:{current_node}->{last_node}')
                    contains_nan = dfReceived.isnull().values.any()
                    contains_nan3 = dfOrg_local.isnull().values.any()
                    if contains_nan:
                      PrintDF(dfReceived)
                    if contains_nan3:
                      rospy.loginfo(dfOrg_local)
                    #df_result = df_svr.combine_first(dfOrg_local)
                    df_result = merge_dataframes(dfReceived_robot,dfOrg_local)
                    df_result.to_csv(node_CtlCenter_globals.strMixedDF, index=False, sep=sDivTab)
                    contains_nan2 = df_result.isnull().values.any()
                    if contains_nan2:
                      rows_with_nan = df_result[df_result.isnull().any(axis=1)]
                      rospy.loginfo(rows_with_nan)
                    #첫행의 값을 Running 으로 변경한다.
                    dfReceived.iloc[0, dfReceived.columns.get_loc(APIBLB_FIELDS_TASK.workstatus.name)] = APIBLB_STATUS_TASK.Running.value
                    dfReceived[APIBLB_FIELDS_TASK.orderstatus.name] = APIBLB_STATUS_TASK.Running.value
                    API_robot_navigation_info(dfReceived)
                    # DataFrame을 리스트로 변환
                    data_list = dfReceived.to_dict(orient="records")
                    # 리스트를 JSON 문자열로 변환
                    json_string = json.dumps(data_list, ensure_ascii=False)
                    pub_DF.publish(json_string)
                    STATUS_TASK = APIBLB_STATUS_TASK.NONE 
                    resultAPI, nodeReturn = API_robot_navigation_info(dfReceived,STATUS_TASK)
                    PrintDF(df_result)
                    node_CtlCenter_globals.listBLB.extend(df_result.to_dict(orient='records'))
                else:
                    # dfOrg_local = pd.DataFrame(lsSeqNode)
                    # dfOrg_local = dfOrg_local[APIBLB_FIELDS_TASK.endnode_list.name] = dfOrg_local[SeqMapField.END_NODE.name]
                    # dfOrg_local = dfOrg_local[APIBLB_FIELDS_TASK.detailcode_list.name] = 0
                    # df_result = dfOrg_local[APIBLB_FIELDS_TASK.distance_list.name] = dfOrg_local[SeqMapField.DISTANCE.name]
                    # rospy.loginfo(df_result)
                    # node_CtlCenter_globals.listBLB.extend(df_result.to_dict(orient='records'))
                    node_CtlCenter_globals.listBLB.extend(lsSeqNode)
                    dfPath = pd.DataFrame(lsSeqNode)
                    print(dfPath)   
                
                print(node_CtlCenter_globals.listBLB[-1])
                dicStartNode = node_CtlCenter_globals.listBLB[0]
                dicTarget = node_CtlCenter_globals.listBLB[-1]
                targetNode = dicTarget['END_NODE']
                targetNodeInfo= GetNodeDic_fromNodeID(targetNode)
                targetPOS = targetNodeInfo[MotorWMOVEParams.POS.name]
                
                targetNodeInfo= GetNodeDic_fromNodeID(targetNode)
                if targetNodeInfo is None:
                    rospy.loginfo(f"타겟노드 정보가 없습니다. {targetNode}")
                    raise ValueError(f"타겟노드 정보가 없습니다. {targetNode}")
                targetNodeType = targetNodeInfo[RFID_RESULT.EPC.name]
                if targetNodeType.find(strNOTAG) < 0:   #도그가 없는 가상 노드가 목적지인 경우 목적지에서 가장 가까운 도그가 있는 노드를 경유한다
                    dicNodeNear = GetNodeDicFromPos(dfNodeInfo,targetPOS,True)
                    nearNode = dicNodeNear[TableInfo.NODE_ID.name]
                    isPathValid = len(node_CtlCenter_globals.listBLB) > 1 and node_CtlCenter_globals.listBLB[-2][SeqMapField.END_NODE.name] == nearNode
                    if not isPathValid: 
                        lsNode1,listSeqMapOrg1=getSeqMap(dicStartNode['START_NODE'],nearNode)
                        lsNode2,listSeqMapOrg2=getSeqMap(nearNode,targetNode)
                        node_CtlCenter_globals.listBLB.clear()
                        node_CtlCenter_globals.listBLB.extend(lsNode1)
                        node_CtlCenter_globals.listBLB.extend(lsNode2)

                #if True:
                if nodeTarget_local not in node_CtlCenter_globals.lsNoLiftDownNodes:
                #if nodeTarget_local != node_CHARGING_STATION:
                    filtered_rows = dfOrg_local[abs(dfOrg_local[SeqMapField.DISTANCE.name]) < 10].to_dict(orient="records")
                    #if len(filtered_rows) == 0 and not isScanMode():
                    if not isScanMode():
                      lsLiftDown = GetLiftControl(False, infoSERVING_DISTANCE, infoSERVING_ANGLE, infoMARKER_ANGLE,infoHEIGHT_LIFT)
                      node_CtlCenter_globals.listBLB.extend(lsLiftDown)
                    #node_CtlCenter_globals.listBLB = lsLiftDown + node_CtlCenter_globals.listBLB
                try:
                    node_CtlCenter_globals.robot.trigger_start_serving()
                    if infoTABLE_ID == HOME_TABLE:
                        TTSAndroid(f'충전소로 복귀합니다.')
                    else:
                        TTSAndroid(f'{infoTABLE_ID}번 테이블 시작.')
                except:
                    pass
                
rospy.loginfo(os.path.splitext(os.path.basename(__file__))[0])