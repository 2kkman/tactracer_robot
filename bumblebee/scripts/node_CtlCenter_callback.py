#!/usr/bin/env python3
from node_CtlCenter_callback_BLB_CMD import *
lock = threading.Lock()

# df_chainlist = pd.read_csv(node_CtlCenter_globals.strCSV_TaskChain, delimiter=sDivTab)
# df_chainlist.drop(df_chainlist[df_chainlist[APIBLB_FIELDS_TASK.taskrunok.name] == 0].index, inplace=True)
# print(df_chainlist)

# df_chainlist.to_csv(node_CtlCenter_globals.strCSV_TaskChain, index=False, sep=sDivTab)

def callback(config):
    rospy.loginfo("Reconfigure Request: {0}".format(config))

def callBackLidarDistanceOnly(data,topic_name='' ):
    # TODO : 응답메세지 발행하는 것도 만들기.
    """_summary_
    data example :  "거리,표준편차 (0.01 미만일때 균일하다고 볼 수 있음),유효포인트"
        data: "1.9048054218292236,0.0062754712998867035,30"
    """
    try:
        recvData = f'{data.data}{sDivItemComma}{getDateTime().timestamp()}'
        distance_Info = recvData.split(sep=sDivItemComma)
        if len(distance_Info) < 4:
            return
        # if GetTiltStatus() == TRAY_TILT_STATUS.TiltFace and try_parse_int(distance_Info[-2]) < 10:
        #     return
        
        if not isFileExist(node_CtlCenter_globals.strFileDistanceArm) and isActivatedMotor(ModbusID.TELE_SERV_MAIN.value):
            AppendDistanceV(distance_Info)
        #node_CtlCenter_globals.lsDistanceV.append(distance_Info)        
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        SendAlarmHTTP(e,True,node_CtlCenter_globals.BLB_ANDROID_IP)
        # SendFeedback(e)

    
def callbackAck(data,topic_name='' ):
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
        #20240910 변경cmdID(TimeStamp):Flag:MBID:TORQUE_MAX:TORQUE_AVE
        data: "1707275205.265836:1:30:150,100"
        data: "1707275205.265836:1:30:120,90"
    """
    try:
        shaking_protection = [ModbusID.MOTOR_V,ModbusID.TELE_SERV_MAIN ]
        recvData = data.data
        logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name} : {node_CtlCenter_globals.activated_motors}"
        #log_all_frames(recvData)
        rospy.loginfo(logmsg)

        # Colon 으로 메세지를 쪼개어 파싱한다.
        # idx 0 - timestamp, 1 - 완료여부 , 2 - MBID
        lsResult = recvData.split(sDivFieldColon)
        torque_max = -1
        torque_ave = -1
        ovr_max = -1
        ovr_ave = -1
        last_started_pos = MIN_INT
        last_targeted_pos = MIN_INT

        mbid_tmp = lsResult[2]  # 모드버스 ID
        flag = lsResult[1]  # 0 이면 미완료, 1 이면 완료
        if len(lsResult) >= 9:
            #print(lsResult)
            torque_max = lsResult[3]  # 최대토크
            torque_ave = lsResult[4]  # 평균토크
            ovr_max = lsResult[5]  # 최대오버로드
            ovr_ave = lsResult[6]  # 평균오버로드
            last_started_pos = int(lsResult[7])  # 최대오버로드
            last_targeted_pos = int(lsResult[8])  # 평균오버로드
            SetTorqueData(mbid_tmp,torque_max,torque_ave,ovr_max,ovr_ave)
        
        # if mbid_tmp == str(ModbusID.BAL_ARM1.value):
        #     rospy.loginfo(f'Ack:{node_CtlCenter_globals.dicModbusCallbackCount[mbid_tmp]},{mbid_tmp}:{flag}:Max:{torque_max},Ave:{torque_ave}')
        #     #print('test')
        #node_CtlCenter_globals.flag_WaitConfirm
        mbid_instance = ModbusID.from_value(mbid_tmp)
        timestamp_local = float(lsResult[0])  # TimeStamp - 모터 구동지시 시각 (완료 시각 아님)
        datetimeobj = datetime.fromtimestamp(timestamp_local)  # TimeStamp 를 datetime 으로 변환
        dtnow = getDateTime()
        finishTime_local = (dtnow - datetimeobj)  # 모터 구동지시 시각으로 부터 경과한 시간
        #명령어 직전에 출발했던 포지션
        #last_started_pos = int(GetItemsFromModbusTable(mbid_instance,MonitoringField.LAST_STARTED_POS))
        #명령어 직전에 타겟으로 지정했던 포지션
        #last_targeted_pos = int(GetItemsFromModbusTable(mbid_instance,MonitoringField.LAST_TARGET_POS))
        #현재 포지션
        cmd_pos,cur_pos=GetPosServo(mbid_instance)
        #1바퀴 이상 돌았는지 여부.
        isCmdMove = True if abs(last_targeted_pos - last_started_pos) > roundPulse else False
        # #유효한 명령어가 아니면 플래그 처리를 하지 않는다.
        # if not isCmdMove:
        #     return
        # 이 부분에서 현재 어떠한 상태인지 알 수 있음. (하강인지 상승인지 등)
        isStarted = flag == "0"
        isStopped = not isStarted
        if mbid_tmp == str(ModbusID.MOTOR_H.value):          
        #   if len(node_CtlCenter_globals.lsHistory_motorH) > 0 and isStopped and isRealMachine:
        #     dicMotorH = node_CtlCenter_globals.lsHistory_motorH[-1]
        #     endNode = dicMotorH.get(SeqMapField.END_NODE.name)
        #     endNodeRFID = get_key_by_value(node_CtlCenter_globals.EPCNodeInfo, endNode)
        #     if endNodeRFID is not None:
        #       lastSeenEPC = node_CtlCenter_globals.dicRFIDTmp.get(endNodeRFID, DATETIME_OLD)
        #       cali_pulse = 200000
        #       if try_parse_int(torque_ave) < 0:
        #         cali_pulse = cali_pulse * -1
        #       dicCali = getMotorMoveDic(ModbusID.MOTOR_H.value, False, cali_pulse, DEFAULT_RPM_SLOWER,ACC_MOVE_H,DECC_MOVE_H)
        #       if isTimeExceeded(lastSeenEPC,10000):
        #         SendCMD_Device([dicCali])
        #         SendInfoHTTP(f'추가운행포지션:{cur_pos}')
        #         return
        #       else:
        #         RFIDControl(False)
        #     else:
        #       RFIDControl(False)
                  
          robot_status = node_CtlCenter_globals.robot.get_current_state()
          validOrder = IsOrderEmpty()
          isControlCmdEmpty = len(node_CtlCenter_globals.listBLB)
          isOrderRemain =len(GetTableList())
          
          #아르코 마커 테이블 위치 파악 루틴 - 일단 주석처리
          # if isStarted:
          #   if validOrder and robot_status == Robot_Status.idle:
          #       #SetCameraMode(CameraMode.MAIN_FHD)
          #       SetCameraMode(CameraMode.WIDE_HIGH)
          #       #node_CtlCenter_globals.robot.trigger_start_noding()
          #       node_CtlCenter_globals.trajectory.clear()
          #       current_angle_z = node_CtlCenter_globals.dicARD_CARRIER.get(DataKey.Angle_Z.name)
          #       distanceDiff = pulseH_to_distance(cur_pos)/1000.0
          #       node_CtlCenter_globals.position = Pose2D(x=0.0, y=0.0)
          #       theta = math.radians(90-current_angle_z)
          #       node_CtlCenter_globals.position.x += round(distanceDiff * math.cos(theta),3)
          #       node_CtlCenter_globals.position.y += round(distanceDiff * math.sin(theta),3)
          #       node_CtlCenter_globals.node_id = 1
          #       node_CtlCenter_globals.table_positions.clear()
          #       node_CtlCenter_globals.trajectory.append((node_CtlCenter_globals.node_id, node_CtlCenter_globals.position.x, node_CtlCenter_globals.position.y,current_angle_z,distanceDiff))
          #       CamControl(True)
          #       #TiltDown()
          #       TiltDiagonal()
          #       SetIMUInterval(500)
          # else:
          #   epc_pos = calculate_average_keys(node_CtlCenter_globals.dicEPC_Node)
          #   print(epc_pos)
          #   if not isControlCmdEmpty and not isOrderRemain and robot_status == Robot_Status.onNoding:
          #       node_CtlCenter_globals.robot.trigger_complete_noding()
          #       CamControl(False)
          #       TiltDown()                
          #       SetCameraMode(CameraMode.WIDE_LOW)
          #       SetIMUInterval(1000)
          #       if len(node_CtlCenter_globals.trajectory) > 0:
          #           df_unique = pd.DataFrame(node_CtlCenter_globals.table_positions)
          #           df_filtered = df_unique.loc[df_unique.groupby("marker_value")["distance"].idxmin()]
          #           print(df_filtered)
          #           df_filtered.to_csv(node_CtlCenter_globals.strCSV_TableLoc, index=False, sep=sDivTab) 
          #           save_positions(node_CtlCenter_globals.strCSV_NodeLoc,node_CtlCenter_globals.trajectory)
          #           # df_updated = update_node_bindings(node_CtlCenter_globals.strCSV_TableLoc,node_CtlCenter_globals.strCSV_NodeLoc)
          #           # print(df_updated)
          #           # df_updated.to_csv(node_CtlCenter_globals.strCSV_TableLoc, index=False, sep=sDivTab)
          #           #save_table_positions(node_CtlCenter_globals.table_positions,node_CtlCenter_globals.strCSV_TableLoc)
          #           #plot_positions_node(node_CtlCenter_globals.strPNG_NodeLoc,node_CtlCenter_globals.strCSV_NodeLoc)
          #           plot_positions(node_CtlCenter_globals.strPNG_NodeLoc,node_CtlCenter_globals.strCSV_NodeLoc,node_CtlCenter_globals.strCSV_TableLoc)
          
        if isStarted:   #START ACK
            if node_CtlCenter_globals.dicModbusShakeLevel.get(mbid_tmp) is None:
                node_CtlCenter_globals.dicModbusShakeLevel[mbid_tmp] = []
            else:
                node_CtlCenter_globals.dicModbusShakeLevel[mbid_tmp].clear()
          
            # 모터 구동지시 명령수신 -> 활성모터 리스트에 추가.
            appendRunningMotorsBLB(mbid_tmp)
            ClearDistanceV()
            SetSuspend(False)
            node_CtlCenter_globals.lsArucoDicArr.clear()
            node_CtlCenter_globals.lsDistanceArmDicArr.clear()
            if node_CtlCenter_globals.dicServoPos.get(mbid_tmp) is None:
              node_CtlCenter_globals.dicServoPos[mbid_tmp] = [0,0]  
            node_CtlCenter_globals.dicServoPos[mbid_tmp][0] = cur_pos
            #ClearArucoTable()
        else:   #STOP ACK
            shakePos = roundPulse/2
            lsShakeFinal = []
            
            if cur_pos > roundPulse:    #POT 친 경우는 - 를 먼저 하고 원복한다
                lsShakeFinal.append(getMotorMoveDic(mbid_tmp, True, cur_pos - shakePos, DEFAULT_RPM_SLOW, ACC_DECC_SMOOTH,ACC_DECC_SMOOTH ))
                lsShakeFinal.append(getMotorMoveDic(mbid_tmp, True, cur_pos, DEFAULT_RPM_SLOW, ACC_DECC_SMOOTH,ACC_DECC_SMOOTH ))
            else:
                lsShakeFinal.append(getMotorMoveDic(mbid_tmp, True, cur_pos + shakePos, DEFAULT_RPM_SLOW, ACC_DECC_SMOOTH,ACC_DECC_SMOOTH ))
                lsShakeFinal.append(getMotorMoveDic(mbid_tmp, True, cur_pos, DEFAULT_RPM_SLOW, ACC_DECC_SMOOTH,ACC_DECC_SMOOTH ))
            
            #공진 방지 모드, 텔레스코픽과 트레이모터는 끝난 후 앞뒤로 반바퀴씩 돌려준다.
            #not isRealMachine 으로 일단 개발기에서만 활성화 시킨다.
            #개발에 번거로울 수 있어 일단 아래 5행 주석처리
            # if mbid_instance in shaking_protection and not isRealMachine:
            #     #SendInfoHTTP(f'공진처리:{mbid_tmp}')
            #     for dicMove in lsShakeFinal:
            #         SendCMD_Device([dicMove])
            #         time.sleep(0.5)
            
            values = node_CtlCenter_globals.dicModbusShakeLevel[mbid_tmp]
            if len(values) > 0:
              mean_value = round(sum(values) / len(values),3)  # 평균 계산
              min_value = min(values)  # 최소값
              max_value = max(values)  # 최대값            
              if isRealMachine:
                SendInfoHTTP(f'SHAKE_MBID:{mbid_tmp},평균:{mean_value},최소:{min_value},최대:{max_value}')
              node_CtlCenter_globals.dicModbusShakeLevel[mbid_tmp].clear()
            if node_CtlCenter_globals.dicServoPos.get(mbid_tmp) is None:
              node_CtlCenter_globals.dicServoPos[mbid_tmp] = [0,0]
            node_CtlCenter_globals.dicServoPos[mbid_tmp][1] = cur_pos
            ClearDistanceBox()
            curTargetT,curTargetN = GetCurrentTargetTable()
            # 모터 구동 완료 ACK 수신 - 활성모터 리스트에서 제거
            
            removeRunningMotorsBLB(mbid_tmp)
            if mbid_tmp == (str)(ModbusID.ROTATE_MAIN_540.value):
                if node_CtlCenter_globals.robot.get_current_state() == Robot_Status.cali_mainRotate:
                   node_CtlCenter_globals.robot.trigger_complete_calibration_mainRotate() 
            
            if mbid_tmp == (str)(ModbusID.ROTATE_SERVE_360.value):
                if node_CtlCenter_globals.robot.get_current_state() == Robot_Status.cali_tray:
                   node_CtlCenter_globals.robot.trigger_complete_calibration_tray() 
                
                if GetTiltStatus() == TRAY_TILT_STATUS.TiltDiagonal:
                    avg_values = calculate_average_values(GetArucoMarkerDict())
                    print_average_values(avg_values)

            # # 수평모터 구동이 완료된 경우 다음목적지로 이동해야 한다.
            # if mbid_tmp == (str)(ModbusID.MOTOR_H.value):
            #     # 다음 목적지가 없으면 (node_target == 0) 운행 종료
            #     # 다음목적지가 있으면 현재 노드를 node_target 으로 변경하고 node_target = 0 처리
            #     if node_CtlCenter_globals.table_target != 0:
            #         node_CtlCenter_globals.node_current = (
            #             node_CtlCenter_globals.table_target
            #         )
            #         node_CtlCenter_globals.table_target = 0
            # rospy.loginfo(
            #     f"모터번호{mbid_tmp}/구동시간 : {finishTime_local}, curr_table,node:{curTable}:{curNode}"
            # )
            if mbid_tmp == (str)(ModbusID.TELE_SERV_MAIN.value):
                ClearDistanceV()
                if not isFileExist(node_CtlCenter_globals.strFileDistanceArm) and isStopped and len(node_CtlCenter_globals.lsDistanceArmDicArr) > 0:# and GetTiltStatus() == TRAY_TILT_STATUS.TiltFace:
                    df = pd.DataFrame(node_CtlCenter_globals.lsDistanceArmDicArr)
                    df_unique = df.drop_duplicates(subset=DISTANCE_V.distanceLD.name, keep='last')
                    rospy.loginfo(df_unique)
                    # DataFrame을 텍스트 파일로 저장
                    df_unique.to_csv(node_CtlCenter_globals.strFileDistanceArm, index=False, sep=sDivTab) 
                    node_CtlCenter_globals.lsDistanceArmDicArr.clear()
                    #print('V모터 구동 완료')
            
            isFileArucoExist = isFileExist(node_CtlCenter_globals.strFileAruco)
            if mbid_tmp == (str)(ModbusID.MOTOR_H.value):       
              if len(node_CtlCenter_globals.lsHistory_motorH) > 0:
                startPos = node_CtlCenter_globals.dicServoPos[mbid_tmp][0]
                endPos = node_CtlCenter_globals.dicServoPos[mbid_tmp][1]
                dicMotorH = node_CtlCenter_globals.lsHistory_motorH[-1]
                start_node = dicMotorH[SeqMapField.START_NODE.name]
                distanceTmp = dicMotorH[SeqMapField.DISTANCE.name]
                end_node = dicMotorH[SeqMapField.END_NODE.name]
                started_node_mm = pulseH_to_distance(startPos)
                linkKey = GetLinkKey(start_node,end_node)
                #if not IsEnableSvrPath():
                SetCurrentNode(end_node)
                dicLinkInfo = {TableInfo.LINK_ID.name:linkKey,SeqMapField.START_NODE.name:startPos,SeqMapField.END_NODE.name:endPos}
                if node_CtlCenter_globals.dfLinkPosInfo.empty or linkKey not in node_CtlCenter_globals.dfLinkPosInfo[SeqMapField.START_NODE.name]:
                  node_CtlCenter_globals.dfLinkPosInfo=pd.concat([node_CtlCenter_globals.dfLinkPosInfo, pd.DataFrame([dicLinkInfo])], ignore_index=True)
                rospy.loginfo(node_CtlCenter_globals.dfLinkPosInfo)
                
              if isScanMode():
                if node_CtlCenter_globals.ScanInfo.get(linkKey) is None:
                  node_CtlCenter_globals.ScanInfo[linkKey] = []
                if len(GetArucoMarkerDict()) > 0:
                    combined_list = list(chain(*GetArucoMarkerDict().values()))
                    node_CtlCenter_globals.ScanInfo[linkKey].extend(combined_list)
                
                #경로길이가 제대로 정의되지 않은 구간을 탐색한 경우 해당 링크(노드와 노드)의 길이를 저장.
                distancePulse = abs(endPos-startPos)
                distance_mm = pulseH_to_distance(distancePulse)
                #node_CtlCenter_globals.lsHistory_motorH.append(dicInfo_local_org)
                rospy.loginfo(node_CtlCenter_globals.lsHistory_motorH)
                if abs(int(distanceTmp)) < 10:
                  updateShortCutInfo(start_node,end_node,distance_mm)
                
                #node_CtlCenter_globals.dfLinkPosInfo
                
                # #ScanMode 에서 주행중 아르코 마커가 발견된경우!
                # if len(GetArucoMarkerDict()) > 0:
                #     #노드 정보 업데이트 (새로 발견된 아르코마커 정보를 노드에 추가)
                #     #노드간 길이가 10cm 이내인 경우 같은 노드로 통합
                #     lsNodeScannedPulse = []
                #     dictArucoAll = copy.deepcopy(GetArucoMarkerDict())
                #     for curTable, lsArucoTmp in dictArucoAll.items():
                #         avg_values = calculate_average_values(GetArucoMarkerDict())
                #         dicCurTableScanResult = avg_values.get(curTable, {})
                #         node_mm = int(dicCurTableScanResult[BLB_LOCATION.DISTANCE_FROM_HOME.name]) - started_node_mm
                #         node_Z = int(dicCurTableScanResult[ARUCO_RESULT_FIELD.Z.name])
                #         #Z가 너무 크면 아르코 마커 에러로 간주
                #         if node_Z > 3000:
                #             continue
                #         lsNodeScannedPulse.append(node_mm)
                #     #graph = [(start_node, end_node, distance_mm)]
                #     if len(lsNodeScannedPulse) > 0:
                #         updated_graph = update_graph_with_new_nodes(start_node, end_node, distance_mm, lsNodeScannedPulse, max(node_CtlCenter_globals.graph.keys())+1)
                #         print(calculate_average_values(GetArucoMarkerDict()))
                #         for edge in updated_graph:
                #             stTmp = edge[0]
                #             endTmp = edge[1]
                #             distTmp = edge[2]
                #             updateShortCutInfo(stTmp,endTmp,distTmp,False)
                #             lsJC = node_CtlCenter_globals.StateInfo.keys()
                #             isStartJC = stTmp in lsJC
                #             isEndJC = endTmp in lsJC
                #             isStartNodeJC = start_node in lsJC
                #             isEndNodeJC = end_node in lsJC
                #             node_id = start_node
                #             old_config = end_node
                #             new_config = endTmp
                #             if isEndJC:
                #                 node_id = end_node
                #                 old_config = start_node
                #                 new_config = stTmp
                #                 update_node_config(node_CtlCenter_globals.strFileCross, node_id,old_config, new_config )
                            
                #             if isStartJC:
                #                 update_node_config(node_CtlCenter_globals.strFileCross, node_id,old_config, new_config )

                #         updateShortCutInfo(start_node,end_node,0, True)
              # else: #스캔모드가 아닌 경우
              #   if isFinishedMotor()
              if not isFileArucoExist and isStopped and len(node_CtlCenter_globals.lsArucoDicArr) > 0 and GetTiltStatus() == TRAY_TILT_STATUS.TiltDiagonal:
                df = pd.DataFrame(node_CtlCenter_globals.lsArucoDicArr)
                df_unique = df.drop_duplicates(subset=MonitoringField.CUR_POS.name, keep='last')
                rospy.loginfo(df_unique)
                # DataFrame을 텍스트 파일로 저장
                df_unique.to_csv(node_CtlCenter_globals.strFileAruco, index=False, sep=sDivTab) 
                node_CtlCenter_globals.lsArucoDicArr.clear()
                #print('V모터 구동 완료')
            
            if mbid_tmp == (str)(ModbusID.MOTOR_V.value):
                messageTTS= TTSMessage.REMAIN_30S.value
                LightWelcome(False)
                ClearDistanceV()
                if not isFileExist(node_CtlCenter_globals.strFileDistanceV) and isStopped and len(node_CtlCenter_globals.lsDistanceVDicArr) > 0 and GetTiltStatus() == TRAY_TILT_STATUS.TiltDown:
                    #V모터 캘리브레이션 데이터가 비어있으면 저장한다.
                    #V모터 캘리브레이션은 파일로 저장한 후 로드하고 없으면 수기 캘리로 생성한다.
                    #캘리브레이션 파일은 결국 DISTANCE_V 를 필드를 원소로 갖는 DF이다.
                    df = pd.DataFrame(node_CtlCenter_globals.lsDistanceVDicArr)
                    df_unique = df.drop_duplicates(subset=DISTANCE_V.distanceLD.name, keep='last')
                    rospy.loginfo(df_unique)
                    # DataFrame을 텍스트 파일로 저장
                    df_unique.to_csv(node_CtlCenter_globals.strFileDistanceV, index=False, sep=sDivEmart) 
                    node_CtlCenter_globals.lsDistanceVDicArr.clear()
                    #print('V모터 구동 완료')
                    
                #curTargetT,curTargetN = GetCurrentTargetTable()
                #curTargetTable,curTarNode = GetTargetTableNode()
                dfReceived = GetDF(curTargetT)
                if dfReceived is not None:
                    dicFirst = dfReceived.iloc[0]                
                    tasktype_current = int(dicFirst[APIBLB_FIELDS_TASK.tasktype.name])
                    if tasktype_current == APIBLB_TASKTYPE.CashPay.value:
                        messageTTS= TTSMessage.PAY_30S.value
                    elif tasktype_current == APIBLB_TASKTYPE.CollectingEmptyPlattes.value:
                        messageTTS= TTSMessage.RETURN_30S.value
                CamControl(False)
                isLiftTrayDown = isLiftTrayDownFinished()
                node_current = GetCurrentNode()
                if isLiftTrayDown and node_current is not None:
                    #if int(node_current) != node_KITCHEN_STATION:
                    # 리프트 모터 하강구동이 완료된 경우 30초내로 꺼내가라는 방송을 한다.
                    #if not isScanOn:
                    DoorOpen()
                    targetTable =GetCurrentTargetTable()
                    dicTagretTableInfo = getTableServingInfo(targetTable)
                    target_marker = dicTagretTableInfo.get(TableInfo.MARKER_VALUE.name,-1)
                    if is_equal(targetTable,target_marker):
                        bReturn,strResult=SaveTableInfo(targetTable)
                        SendInfoHTTP(f'{targetTable} 번 테이블정보 저장 :{bReturn} - {strResult}')
                    
                    if curTargetN != node_KITCHEN_STATION and curTargetN != node_CHARGING_STATION and isLiftTrayDown and IsEnableSvrPath():
                        SetWaitConfirmFlag(True,AlarmCodeList.WAITING_USER)
                        nowTime = getDateTime()
                        # 경과 시간 리스트 (초 단위)
                        time_deltas = [5, 10, 15, 20, 25,30,35,40,45,50,55,60]
                        time_deltas = [5, 60]
                        time_deltas = [3, 6]
                        # 딕셔너리 생성
                        time_dict = {
                            (nowTime + timedelta(seconds=delta)): ("이제 곧 닫힙니다." if delta == time_deltas[-1] else f"{time_deltas[-1]-delta}초 후 닫힙니다.")
                            for delta in time_deltas
                        }
                        #tts 서비스 가용하면 음식을 꺼내가라는 안내방송 진행
                        ttsResult = TTSAndroid(messageTTS,1)
                        if ttsResult:
                            node_CtlCenter_globals.dicTTS.clear()
                            sorted_time_dict = OrderedDict(sorted(time_dict.items()))
                            node_CtlCenter_globals.dicTTS.update(sorted_time_dict)
                        
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        SendAlarmHTTP(e,True,node_CtlCenter_globals.BLB_ANDROID_IP)
        # SendFeedback(e)

def callBackLidarDistance(data,topic_name='' ):
    # TODO : 응답메세지 발행하는 것도 만들기.
    """_summary_
    data example :
        data: "boxCenter:0.522,-0.001,0.088`width:0.15`height: 0.42`depth: 0.15`distance:0.53"
    """
    try:
        recvData = f'{data.data}{sDivEmart}{ARUCO_RESULT_FIELD.LASTSEEN.name}{sDivFieldColon}{getDateTime().timestamp()}'
        recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivEmart)
        objWidth = try_parse_float(recvDataMap.get(LIDAR_DISTANCE_PARAMS.width.name))
        if  objWidth > 0.14:
        #print(recvData)
            AppendDistanceBox(recvDataMap)
        #
        # isLiftOnDown = isLiftLanding()
        # isMotorH_Moving = isActivatedMotor(ModbusID.MOTOR_H.value)
        # detected_distance = 1000
        # if isLiftOnDown or isMotorH_Moving and isTimeExceeded(GetLidarDistanceBoxTimeStamp(),2000):
        #     cur_rpm = round(abs(getMotorSpdDirection(ModbusID.MOTOR_H.value)) * 0.9)
        #     cur_spd= calculate_speed_fromRPM(cur_rpm)
        #     boxCenter = recvDataMap.get(LIDAR_DISTANCE_PARAMS.boxCenter, None)
        #     width = recvDataMap.get(LIDAR_DISTANCE_PARAMS.width, None)
        #     height = recvDataMap.get(LIDAR_DISTANCE_PARAMS.height, None)
        #     depth = recvDataMap.get(LIDAR_DISTANCE_PARAMS.depth, None)
        #     detected_distance = recvDataMap.get(LIDAR_DISTANCE_PARAMS.distance, None)
        #     targetPOS_H = node_CtlCenter_globals.dicTargetPos.get(str(ModbusID.MOTOR_H.value),MIN_INT)
        #     targetPOS_V = node_CtlCenter_globals.dicTargetPos.get(str(ModbusID.MOTOR_V.value),MIN_INT)
            
        #     #log_all_frames(recvData)
        #     cmdpos_lift,cur_pos_h =GetPosServo(ModbusID.MOTOR_H)
        #     #pot_cur_lift,not_cur_lift ,cmdpos_lift,cur_pos_lift =GetPotNotCurPosServo(ModbusID.MOTOR_V)
        #     targetpules_left = abs(targetPOS_H-cur_pos_h)
        #     target_mm_left=pulseH_to_distance(targetpules_left)
        #     tiltStutus = GetTiltStatus()
        #     #거리 및 인식된 높이를 추출해낸다. (distance, height)
            
        #     if tiltStutus == TRAY_TILT_STATUS.TiltFace:   
        #         logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name} : {node_CtlCenter_globals.activated_motors}"
        #         rospy.loginfo(logmsg)
        #         if target_mm_left > 1000 and cur_rpm > DEFAULT_SPD_SLOW:
        #             rospy.loginfo(f'감속-목표까지거리:{target_mm_left},현재속도:{cur_rpm},감지된물체정보:{recvDataMap}')
        #             dicSpd = getMotorSpeedDic(ModbusID.MOTOR_H.value, True, DEFAULT_SPD_SLOW, ACC_MOVE_H,DECC_MOVE_H)
        #             SendCMD_Device([dicSpd])
        #             #주행일때 - 1~2m 남았는데 속도가 RPM 보다 빠르면 속도를 변경한다.
        #             #UpdateLidarDistanceBoxTimeStamp()
        #         elif target_mm_left <= 1000 and cur_rpm > 100:
        #             #주행일때 - 1m 이하로 남았는데 계속 장애물이 감지되면 속도를 1로 만든다. (거의 멈춤)
        #             dicSpd = getMotorSpeedDic(ModbusID.MOTOR_H.value, True, ALMOST_ZEROINT, ACC_MOVE_H,DECC_MOVE_H)
        #             SendCMD_Device([dicSpd])
        #         UpdateLidarDistanceBoxTimeStamp()    
        #     elif tiltStutus == TRAY_TILT_STATUS.TiltDetectingV:
        #         StopMotor(ModbusID.MOTOR_V.value)
        #         UpdateLidarDistanceBoxTimeStamp()
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        SendAlarmHTTP(e,True,node_CtlCenter_globals.BLB_ANDROID_IP)
        # SendFeedback(e)


def callbackBLB_CMD(data,topic_name=''):
    """_summary_
        /BLB_CMD 토픽, 즉 UI, 혹은 MQTT 등에서 날라오는 명령어를 모니터링
        data: "{\"ID\": \"Bumblebee1\", \"TRAY_A\": \"2\", \"TRAY_B\": \"1\", \"LEVEL\": \"0\",\
          \ \"STATE\": \"CONFIRM\", \"TIME\": \"20240207_141000\"}"

    Args:
        data (_type_): _description_
    """
    try:
        node_CtlCenter_globals.dicTopicCallbackTimeStamp[topic_name] = getDateTime()
        recvDataMap = {}
        recvData = (str)(data.data)
        logmsg = f"{recvData} from {topic_name} - {sys._getframe(1).f_code.co_name}"
        #log_all_frames(logmsg)
        rospy.loginfo(logmsg)
        # recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivEmart)
        # 커맨드 형태가 JSON 인 경우 (범블비 UI에서 날아오는 경우)
        # 와 구분자로 된 데이터 파싱하여 dict 개체 생성
        if is_json(recvData):
            recvDataMap = json.loads(recvData)
        else:
            recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivEmart)
        if APIBLB_FIELDS_ACTION.chainlist.name in recvDataMap.keys():
            lsDicArray=recvDataMap[APIBLB_FIELDS_ACTION.chainlist.name]
            node_CtlCenter_globals.dfTaskChainInfo = GetDFTaskChain(lsDicArray)  
            return          
        elif APIBLB_FIELDS_ACTION.target.name in recvDataMap.keys():
          #target = 주행시작하라는 명령어
          lsDicArray=recvDataMap[APIBLB_FIELDS_ACTION.target.name]
          SetDFTableInfo(lsDicArray)
          return 

        # 메인회전, 트레이 회전 명령어.
        TABLE = recvDataMap.get(BLB_CMD_CUSTOM.TABLE.name, None)
        if TABLE is not None:
          gotoNode(GetMachineStr(), TABLE, False)
          return
        # 메인회전, 트레이 회전 명령어.
        ROTATE = recvDataMap.get(BLB_CMD_CUSTOM.ROTATE.name, "")
        # STATE 값에 따라 처리하는 기능들이 달라진다.
        OBS = recvDataMap.get(BLB_CMD_CUSTOM.OBS.name, "")
        # STATE 값에 따라 처리하는 기능들이 달라진다.
        PC = recvDataMap.get(BLB_CMD_CUSTOM.PC.name, "")
        # STATE 값에 따라 처리하는 기능들이 달라진다.
        PROFILE = recvDataMap.get(BLB_CMD_CUSTOM.PROFILE.name, "")
        # STATE 값에 따라 처리하는 기능들이 달라진다.
        STATE = recvDataMap.get(BLB_CMD.STATE.name, "")
        # 트레이의 레벨을 정하는 필드지만 LAB TEST 에서는 사용하지 않는다
        LEVEL = recvDataMap.get(BLB_CMD.LEVEL.name, "")
        
        #포인트 클라우드 테스트 블록
        if PC != "":
            fn = '/root/catkin_ws/angle_pc2.dat'
            if isFileExist(fn):            
              with open(fn, 'rb') as f:
                dicSafety = pickle.load(f)
                node_CtlCenter_globals.dicSafety.clear()
                node_CtlCenter_globals.dicSafety.update(dicSafety)
            
            targetPos = round(mapRange(int(PC),0,360,0,10000))
            chkData = get_closest_value(node_CtlCenter_globals.dicSafety,targetPos)
            #chkData = node_CtlCenter_globals.dicSafety.get(targetPos, None)
            if chkData is not None:
              node_CtlCenter_globals.merged_pc2 = chkData
            else:
              cmd_pos, cur_pos= GetPosServo(ModbusID.ROTATE_TESTER)
              node_CtlCenter_globals.dicSafety[cur_pos] = node_CtlCenter_globals.dicSafety[-1]
              if len(node_CtlCenter_globals.dicSafety) >= 5:
                with open(fn, 'wb') as f:
                  pickle.dump(node_CtlCenter_globals.dicSafety, f)          
                dictWallMap = copy.deepcopy(node_CtlCenter_globals.dicSafety)
                node_CtlCenter_globals.dicSafety.clear()
                angle_pc2_dict = {}
                for enc, pc2list in dictWallMap.items():
                    if enc == -1:
                      continue
                    targetAngle = round(mapRange(enc,0,10000,0,360))
                    angle_pc2_dict[targetAngle] = pc2list
                
                # Define the radius from the rotation center to the lidar (in meters)
                radius = 0.01  # 1 cm
                rospy.loginfo(angle_pc2_dict.keys())
                node_CtlCenter_globals.merged_pc2 = merge_point_clouds(angle_pc2_dict, radius)
                                   
        #라이다 장애물 테스트 기능 테스트 블록
        if OBS != "":
            target_dimensions = (1, 0.05, 0.05)            
            paramArmControl = OBS.split(sep=sDivItemComma)
            lsModbusRequests = []
            x = float(paramArmControl[0])
            y = float(paramArmControl[1])
            z = float(paramArmControl[2])
            target_position = (x, y, z)

            pcl_data = pointcloud2_to_pcl(node_CtlCenter_globals.merged_pc2)
            if is_area_clear(pcl_data, target_position, target_dimensions):
                rospy.loginfo("Area is clear for the robot arm to extend.")
            else:
                rospy.loginfo("Area is not clear. Obstruction detected.")

        # INIT - 명령어
        # QBI 터치패널 특성상 UI가 완전히 로딩 된 후 터치패널 초기화 명령어를 내려야 한다.
        # QBI UI가 로딩되면 INIT 값을 가진 명령어를 보내고 이를 처리하는 부분
        # 처리후 함수 종료 return
        if STATE.find('PROFILE') >= 0:
            PROFILE = STATE.split(sDivFieldColon)[-1]
        elif STATE.find(HOME_TABLE) >= 0:
            HomeCall()
            return
        elif STATE.find(BLB_CMD_STATUS.INIT.name) >= 0:
            AppendSendStatus(BLB_STATUS_FIELD.INIT)
            #SendStatus(BLB_STATUS_FIELD.INIT)
            node_CtlCenter_globals.timestamp_touchinit = getDateTime()
            node_CtlCenter_globals.bInit = True
            return
        #MQTT 등 외부에서 dic 메세지를 수신하는 경우 아래 ESTOP 이 호출된다.
        elif STATE.find(BLB_CMD_STATUS.ESTOP.name) >= 0:
            StopEmergency(ALM_User.USER_ESTOP.value, False)            
        elif STATE.find(BLB_CMD_STATUS.STOP_SMOOTH.name) >= 0:
            StopMotor(ModbusID.MOTOR_H.value, DECC_MOVE_H)
        elif STATE.find(BLB_CMD_STATUS.RESUME.name) >= 0:
            #1.대기상태가 아니면 무시한다.
            #2.listBLB와 tableist 확인.
            # 아니면 curTarget 에서 현재 테이블 꺼내서 paused 인 행을 찾아 재구성
            #지시정보를 교체한 뒤 pause 를 해제한다.
            ResumeState()
            return
        #CancelJob  
        elif STATE.find(BLB_CMD_STATUS.RELOAD_SVR.name) >= 0:
            ReloadSvrTaskList()
            return
        elif STATE.find(BLB_CMD_STATUS.CANCEL.name) >= 0:
            CancelJob()
            return
        elif STATE.find(BLB_CMD_STATUS.PAUSE.name) >= 0:
            SetPauseState()
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
        chkValidOrder = recvDataMap.get(BLB_CMD.STATE.name, None)
        if chkValidOrder is not None:
        #if CheckAllKeysExist(BLB_CMD, recvDataMap):
            STATE = recvDataMap.get(BLB_CMD.STATE.name, "")
            # QBI UI 에서 확인 메세지를 누른 경우
            if STATE.find(BLB_CMD_STATUS.CONFIRM.name) >= 0:
                SetWaitConfirmFlag(False,AlarmCodeList.JOB_COMPLETED)
            # elif STATE.find(BLB_CMD.MODE.name) >= 0:
            #     InsertTableList(1)
            #     #AppendTableList('P1')
            # 범블비 이동명령어인 경우!
            elif STATE.find(BLB_CMD_STATUS.MOVE.name) >= 0:
                TRAY_A = (int)(recvDataMap.get(BLB_CMD.TRAY_A.name, node_NOT_TABLE))
                #
                if IsEnableSvrPath():
                  rackID,empty_cells = GetRackID_Empty()
                  # lsR1=GetTaskChain(APIBLB_FIELDS_TASK.trayrack.name, 'R1')
                  # lsR2=GetTaskChain(APIBLB_FIELDS_TASK.trayrack.name, 'R2')
                  # rackID = 'R1'
                  # if len(lsR1) > len(lsR2):
                  #   rackID = 'R2'
                  API_AppendOrderTable(TRAY_A,rackID.name, APIBLB_TASKTYPE.ServingTask.value)
                else:
                  SetWaitConfirmFlag(False,AlarmCodeList.OK)
                  TRAY_B = (int)(recvDataMap.get(BLB_CMD.TRAY_B.name, node_NOT_TABLE))
                  nodeOnlyMode = (int)(recvDataMap.get(BLB_CMD.MODE.name, 0))
                  AppendTableList(TRAY_A)
                  if TRAY_B != node_NOT_TABLE:
                    AppendTableList(TRAY_B)
                  if isTrue(nodeOnlyMode):
                      for moveOnlyNode in GetTableList():
                          node_id = GetNodeFromTable(moveOnlyNode)
                          node_CtlCenter_globals.lsNoLiftDownNodes.append(node_id)
                          
                # ID = recvDataMap.get(BLB_CMD.ID.name, "")
                # TRAY_B = (int)(recvDataMap.get(BLB_CMD.TRAY_B.name, node_NOT_TABLE))
                # LEVEL = (int)(recvDataMap.get(BLB_CMD.LEVEL.name, node_NOT_TABLE))
                # TIME = recvDataMap.get(BLB_CMD.TIME.name, "")
                # MODE = recvDataMap.get(BLB_CMD.MODE.name, BLB_CMD_MODE.FAST.name)
                rospy.loginfo(
                    f"From {GetCurrentNode()} to {TRAY_A},"
                    f" Cur_listTable:{GetTableList()}"
            )

        # # TODO : 응답메세지 발행하는 것도 만들기.
        if ROTATE != "":  # 사용자 커스텀 명령어 GetCustomFileControl 활용
            paramArmControl = ROTATE.split(sep=sDivItemComma)
            isMainRotate = isTrue(paramArmControl[0])
            angle_rotate = strToRoundedInt(paramArmControl[1]) % 360
            dicRotate = {}
            if isMainRotate:
                dicRotate = GetDicRotateMotorMain(angle_rotate)
            else:
                dicRotate = GetDicRotateMotorTray(angle_rotate)
            
            lsRotateRequests = getListedDic(dicRotate)
            if len(lsRotateRequests) > 0:
                SetWaitConfirmFlag(False,AlarmCodeList.OK)
                node_CtlCenter_globals.listBLB.clear()
                node_CtlCenter_globals.listBLB.append(lsRotateRequests)
                #node_CtlCenter_globals.cmdIdx = 0
                rospy.loginfo(lsRotateRequests)                           
        
        if PROFILE != "":  # 사용자 커스텀 명령어 GetCustomFileControl 활용
            lsModbusRequests = BLB_CMD_Profile(PROFILE)
            if isinstance(lsModbusRequests,list) and len(lsModbusRequests) > 0:
              node_CtlCenter_globals.listBLB.clear()
              node_CtlCenter_globals.listBLB.extend(lsModbusRequests)
              #node_CtlCenter_globals.cmdIdx = 0
              rospy.loginfo(lsModbusRequests)
              #SetWaitConfirmFlag(True)
              #SendMsgToMQTT(pub_topic2mqtt,MQTT_TOPIC_VALUE.TTS.value,f"범블비 동작을 시작합니다.")
            else:
              rospy.loginfo(lsModbusRequests)  
        else:
            rospy.loginfo(recvDataMap)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        SendAlarmHTTP(e,True,node_CtlCenter_globals.BLB_ANDROID_IP)
        # SendFeedback(e)

def callbackModbus(data,topic_name):
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
        if isinstance(node_CtlCenter_globals.dic_485ex.get(mbid), dict):
            node_CtlCenter_globals.dic_485ex[mbid].update(recvDataMap)
        else:
            node_CtlCenter_globals.dic_485ex[mbid] = recvDataMap

        #node_CtlCenter_globals.dicModbusShakeLevel[mbid] = node_CtlCenter_globals.dicModbusShakeLevel.get(mbid, 0) + 1
        #shake_level = try_parse_float(node_CtlCenter_globals.dicARD_CARRIER.get(CARRIER_STATUS.SHAKE_TRAY.name,0))
        shake_level = try_parse_float(node_CtlCenter_globals.dicARD_CARRIER.get(DataKey.MOVE_LEVEL.name,0))
        if node_CtlCenter_globals.dicModbusShakeLevel.get(mbid) is None:
          node_CtlCenter_globals.dicModbusShakeLevel[mbid] = []
        node_CtlCenter_globals.dicModbusShakeLevel[mbid].append(shake_level)
        #if mbid == str(ModbusID.BAL_ARM1.value):
        node_CtlCenter_globals.dicTopicCallbackTimeStamp[topic_name] = getDateTime()
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(f'ERR callbackModbus:{message}')
        SendAlarmHTTP(e,True,node_CtlCenter_globals.BLB_ANDROID_IP)
        #prtMsg(message)
        # SendFeedback(e)


def callbackARD_CARRIER(data,topic_name=''):
    """_summary_
        아두이노에서 ARD_CARRIER 토픽으로 JSON 메세지를 발행한걸 수신하여 관련 전역변수에 업데이트 한다.
    Args: data : 
        "{\"\\nI_TRAY_2_BOTTOM\": \"0\", \"I_TRAY_2_HOME\": \"0\", \"I_TRAY_2_TOP\": \"0\"\
        , \"I_DOOR_1_BOTTOM\": \"0\", \"I_DOOR_1_HOME\": \"0\", \"I_DOOR_1_TOP\": \"0\"\
        , \"I_LIMIT_BOTTOM\": \"0\", \"O_V12_NC\": \"0\", \"O_V5_NC\": \"0\", \"LOAD\":
        \\ \"0.73\", \"GOR_SVN\": \"148.71,-25.84,0.00\", \"GAV_SVN\": \"-0.00,0.06,-0.02\"\
        , \"GLA_SVN\": \"5.38,5.77,-9.50\", \"GAV_TUN\": \"0.08\", \"TEMPN\": \"33.04\"}"

    """
    node_CtlCenter_globals.dicTopicCallbackTimeStamp[topic_name] = getDateTime()
    try:
        recvData = data.data
        if is_valid_python_dict_string(recvData):
            recvDataMap = ast.literal_eval(recvData)
        elif is_json(recvData):
            recvDataMap = json.loads(recvData)
        else:
            recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        # recvDataMap = json.loads(recvData)
        cleaned_data = {}
        for key, value in recvDataMap.items():
            cleaned_key = re.sub(r'[^\x20-\x7E]', '', key)  # Only keep ASCII characters (from space to tilde)
            cleaned_data[cleaned_key] = value
        #GOR_SVN = cleaned_data.get(TRAY_ARD_Field.GOR_SVN.name, None)
        current_angle_z = cleaned_data.get(DataKey.Angle_Z.name)
        android_ip = cleaned_data.get(DataKey.AndroidIP.name)
        angle_y = cleaned_data.get(DataKey.Angle_Y.name)
        if current_angle_z is not None and angle_y is not None:
          height_diff = calculate_relative_height_difference(node_CtlCenter_globals.last_detect_3d,90-angle_y)
          if height_diff is not None:
            #node_CtlCenter_globals.last_detect_status=is_wall_or_obstacle(node_CtlCenter_globals.last_detect_3d)
            #node_CtlCenter_globals.last_detect_status=detect_environment(node_CtlCenter_globals.last_detect_3d)
            #print(f"장애물 내 상대적인 높이 차이: {height_diff:.2f} m")
            node_CtlCenter_globals.last_detect_3d = None
            cleaned_data[MAIN_STATUS.DETECTED_HEIGHT.name] = round(height_diff,3)
          else:
            cleaned_data[MAIN_STATUS.DETECTED_HEIGHT.name] = -1
            node_CtlCenter_globals.last_detect_status = None
          if android_ip != node_CtlCenter_globals.BLB_ANDROID_IP:
            node_CtlCenter_globals.BLB_ANDROID_IP = android_ip
        
        #if GOR_SVN is not None:
            #GOR_SVNarr = GOR_SVN.split(sDivItemComma)  # orientation , roll,pitch,yaw
            #current_angle_z = float(GOR_SVNarr[2])
            #current_angle_z = cleaned_data.get(DataKey.Angle_Z.name)
            #if current_angle_z is not None:
            blb_status = node_CtlCenter_globals.robot.get_current_state()
            lastdistance = 0
            if blb_status == Robot_Status.onNoding:
                cmd_pos, cur_pos = GetPosServo(ModbusID.MOTOR_H)
                distanceNew = pulseH_to_distance(cur_pos)/1000.0
                if len(node_CtlCenter_globals.trajectory) > 0:
                  node_id, posX,posY, old_angle_z,lastdistance = node_CtlCenter_globals.trajectory[-1]
                  distanceDiff = distanceNew - lastdistance
                  #if abs(distanceDiff) > 0.00001:
                  theta = math.radians(90-current_angle_z)
                  node_CtlCenter_globals.position.x += round(distanceDiff * math.cos(theta),3)
                  node_CtlCenter_globals.position.y += round(distanceDiff * math.sin(theta),3)
                  node_CtlCenter_globals.node_id += 1
                  node_CtlCenter_globals.trajectory.append((node_CtlCenter_globals.node_id, node_CtlCenter_globals.position.x, node_CtlCenter_globals.position.y,current_angle_z,distanceNew))
                  sMsg = f"New Node Created:ID={node_CtlCenter_globals.node_id},X={node_CtlCenter_globals.position.x},Y={node_CtlCenter_globals.position.y},angle_z={current_angle_z},distance={distanceNew}"
                  rospy.loginfo(sMsg)
                #SendAlarmHTTP(sMsg,True,node_CtlCenter_globals.BLB_ANDROID_IP)
        
        node_CtlCenter_globals.dicARD_CARRIER.update(cleaned_data)
        load1 = node_CtlCenter_globals.dicARD_CARRIER.get(CARRIER_STATUS.LOAD1.name,-1)
        load2 = node_CtlCenter_globals.dicARD_CARRIER.get(CARRIER_STATUS.LOAD2.name,-1)
        SetRackIDStatus(APIBLB_FIELDS_STATUS.R1.name,load1)
        SetRackIDStatus(APIBLB_FIELDS_STATUS.R2.name,load2)
        if node_CtlCenter_globals.runFromLaunch:
            logmsg = f"{recvData} from {topic_name},{sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
            rospy.loginfo(logmsg)
    except Exception as e:
        message = traceback.format_exc()
        rospy.logdebug(message)
        SendAlarmHTTP(e,True,node_CtlCenter_globals.BLB_ANDROID_IP)
        # SendFeedback(e)


def callback_CROSS_INFO(data,topic_name=''):
    """_summary_
        트레이에서 ARUCO_RESULT 토픽으로 JSON 메세지를 발행한걸 수신하여 관련 전역변수에 업데이트 한다.
    Args: 
    data: "{\"DIFF_X\": 16.59, \"DIFF_Y\": 23.56, \"ANGLE\": 162.89, \"CAM_ID\": 0, \"MARKER_VALUE\"\
  : 10, \"X\": 0.2423, \"Y\": 0.2155, \"Z\": 3.3798}"
    """
    recvDatalist = []
    try:
        node_CtlCenter_globals.dicTopicCallbackTimeStamp[topic_name] = getDateTime()
        recvData = data.data.replace("'", '"')
        if is_valid_python_dict_string(recvData):
            recvDataMap2 = ast.literal_eval(recvData)
        elif is_json(recvData):
            recvDataMap2 = json.loads(recvData)
        else:
            recvDataMap2 = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        
        if isinstance(recvDataMap2,dict):
            recvDatalist.append(recvDataMap2)
            node_CtlCenter_globals.dic_CROSSINFO.update(recvDataMap2)
        else:
            recvDatalist.extend(recvDataMap2)
        
    except Exception as e:
        message = traceback.format_exc()
        rospy.logdebug(message)
        SendAlarmHTTP(e,True,node_CtlCenter_globals.BLB_ANDROID_IP)
                
def callback_ARUCO_RESULT(data,topic_name=''):
    """_summary_
        트레이에서 ARUCO_RESULT 토픽으로 JSON 메세지를 발행한걸 수신하여 관련 전역변수에 업데이트 한다.
    Args: 
    data: "{\"DIFF_X\": 16.59, \"DIFF_Y\": 23.56, \"ANGLE\": 162.89, \"CAM_ID\": 0, \"MARKER_VALUE\"\
  : 10, \"X\": 0.2423, \"Y\": 0.2155, \"Z\": 3.3798}"
    """
    recvDatalist = []
    try:
        node_CtlCenter_globals.dicTopicCallbackTimeStamp[topic_name] = getDateTime()
        recvData = data.data.replace("'", '"')
        if is_valid_python_dict_string(recvData):
            recvDataMap2 = ast.literal_eval(recvData)
        elif is_json(recvData):
            recvDataMap2 = json.loads(recvData)
        else:
            recvDataMap2 = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        
        if isinstance(recvDataMap2,dict):
            recvDatalist.append(recvDataMap2)
        else:
            recvDatalist.extend(recvDataMap2)
        current_angle_z_float = node_CtlCenter_globals.dicARD_CARRIER.get(DataKey.Angle_Z.name, None)
        if current_angle_z_float is None:
          return
        current_angle_z = round(current_angle_z_float)            
        for recvDataMap3 in recvDatalist:
            recvDataMap =recvDataMap3
            #recvDataMap = rotate_marker(recvDataMap3,current_angle_z)
            # recvData = data.data
            # recvDataMap = json.loads(recvData)
            OnScanMarker = recvDataMap.get(ARUCO_RESULT_FIELD.IS_MARKERSCAN.name, None)
            if OnScanMarker is not None:
                node_CtlCenter_globals.bOnScanMarker = OnScanMarker

            cam_id = recvDataMap.get(ARUCO_RESULT_FIELD.CAM_ID.name, None)
            if cam_id == None:
                return
            
            angle_y = node_CtlCenter_globals.dicARD_CARRIER.get(DataKey.Angle_Y.name, -1)
            marker_value = recvDataMap.get(ARUCO_RESULT_FIELD.MARKER_VALUE.name, None)
            marker_x = float(recvDataMap.get(ARUCO_RESULT_FIELD.X.name, 0.0))
            marker_y = float(recvDataMap.get(ARUCO_RESULT_FIELD.Y.name, 0.0))
            marker_z = float(recvDataMap.get(ARUCO_RESULT_FIELD.Z.name, 0.0))
            # print((ref_dict))
            # print((recvDataMap))
            #recvDataMap['Y'] = marker_y * -1
            result = compute_movement_and_rotation_from_dict(ref_dict, recvDataMap)
            #result = compute_final_position_and_angle(ref_dict, recvDataMap)
            # print(result)
            result.update(recvDataMap)
            #node_CtlCenter_globals.dicARUCO_last.update(result)
            #node_CtlCenter_globals.dicARUCO_last.update(recvDataMap)
            # marker_x = round(float(recvDataMap.get(ARUCO_RESULT_FIELD.X.name, 0.0)),3)
            # marker_y = round(float(recvDataMap.get(ARUCO_RESULT_FIELD.Y.name, 0.0)*-1),3)
            # marker_z = round(float(recvDataMap.get(ARUCO_RESULT_FIELD.Z.name, 0.0)),3)
            # 예제 테스트 (pitch = 30도 = π/6 라디안)
            pitch_angle = np.radians(angle_y)  # 30도를 라디안으로 변환
            x_new, y_new, z_new = transform_marker_coordinates(marker_x, marker_y, marker_z, pitch_angle)
            # print(f"angle:{angle_y},기존:{marker_x},{marker_y},{marker_z}->변환x'={x_new:.3f}, y'={y_new:.3f}, z'={z_new:.3f}")
            
            
            if node_CtlCenter_globals.robot.get_current_state() == Robot_Status.onNoding and current_angle_z is not None:
                #cmd_pos, cur_pos = GetPosServo(ModbusID.MOTOR_H)
                #node_id, posX,posY, lastdistance = node_CtlCenter_globals.trajectory[-1]
                #distance_meter = pulseH_to_distance(cur_pos)/1000.0
                # theta = math.radians(90-current_angle_z)
                # node_CtlCenter_globals.position.x = round(distance_meter * math.cos(theta),3)
                # node_CtlCenter_globals.position.y = round(distance_meter * math.sin(theta),3)
                # sMsg = f"Curpos={cur_pos},cur_Meter={distance_meter},nodeID={node_CtlCenter_globals.node_id}, X={node_CtlCenter_globals.position.x}, Y={node_CtlCenter_globals.position.y}, AngleZ={current_angle_z}"
                #dictTableInfo = update_raw_data(marker_value, marker_x, marker_y, marker_z, node_CtlCenter_globals.trajectory)
                dictTableInfo = update_raw_data(marker_value, x_new, y_new, z_new, node_CtlCenter_globals.trajectory)
                node_CtlCenter_globals.table_positions.append(dictTableInfo)
                rospy.loginfo(dictTableInfo)
                #SendAlarmHTTP(sMsg,True,node_CtlCenter_globals.BLB_ANDROID_IP)
            else:
                recvDataMap[ARUCO_RESULT_FIELD.LASTSEEN.name] = getDateTime().timestamp()
                sPos = GetItemsFromModbusTable(ModbusID.MOTOR_H,MonitoringField.CUR_POS)
                recvDataMap[MonitoringField.CUR_POS.name] = sPos
                recvDataMap[MonitoringField.CUR_SPD.name] = GetItemsFromModbusTable(ModbusID.MOTOR_H,MonitoringField.CUR_SPD)
                curDistanceSrvTele, curAngle,cur_angle_360  = GetCurrentPosDistanceAngle()
                recvDataMap[BLB_LOCATION.DISTANCE_FROM_HOME.name] = pulseH_to_distance(sPos)
                recvDataMap[BLB_LOCATION.DISTANCE_ARMEXTENDED.name] = curDistanceSrvTele
                recvDataMap[BLB_LOCATION.ANGLE_540.name] = curAngle
                recvDataMap[BLB_LOCATION.ANGLE_360.name] = cur_angle_360
                recvDataMap[ARUCO_RESULT_FIELD.XX.name] = x_new
                recvDataMap[ARUCO_RESULT_FIELD.YY.name] = y_new
                recvDataMap[ARUCO_RESULT_FIELD.ZZ.name] = z_new

                node_CtlCenter_globals.lsArucoDicArr.append(recvDataMap)       
                #AppendArucoTable(marker_value,recvDataMap)
                if marker_value in node_CtlCenter_globals.dicARUCO_Result:
                    node_CtlCenter_globals.dicARUCO_Result[marker_value].append(recvDataMap)
                else:
                    lsMap = [ recvDataMap ]
                    node_CtlCenter_globals.dicARUCO_Result[marker_value] = lsMap
        
        # 배포모드로 실행시 로깅한다.
        # if not node_CtlCenter_globals.runFromLaunch:
        #     logmsg = f"{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
        #     rospy.loginfo(logmsg)
    except Exception as e:
        message = traceback.format_exc()
        rospy.logerr(message)
        print(message)
        SendAlarmHTTP(e,True,node_CtlCenter_globals.BLB_ANDROID_IP)
        
def callback_lidar_obstacle(data,topic_name=''):
    recvDatalist = []
    try:
        recvData = data.data.replace("'", '"')
        if is_valid_python_dict_string(recvData):
            recvDataMap2 = ast.literal_eval(recvData)
        elif is_json(recvData):
            recvDataMap2 = json.loads(recvData)
        else:
            recvDataMap2 = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)        
        add_new_obstacle_data(recvDataMap2)
    except Exception as e:
        message = traceback.format_exc()
        rospy.logerr(message)
        print(message)
        SendAlarmHTTP(e,True,node_CtlCenter_globals.BLB_ANDROID_IP)

def callbackRFID_DF(data,topic_name=''):
    """
    트레이에서 RFID_DF 토픽으로 JSON 메세지를 발행한걸 수신하여 관련 전역변수에 업데이트 한다."""
    try:
        curtime = getDateTime()
        node_CtlCenter_globals.dicTopicCallbackTimeStamp[topic_name] = curtime
        recvData = data.data
        recvDataMap = json.loads(recvData)
        node_CtlCenter_globals.dfEPCView = pd.DataFrame(recvDataMap)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        SendAlarmHTTP(message,True,node_CtlCenter_globals.BLB_ANDROID_IP)
        
def callbackRFID(data,topic_name=''):
    """
    주행모드 사전 설정되어있어야 함. global 변수명 기재.
    1. 수동운전 - 자동멈춤
    2. 수동운전 - 엔코더 변경
    3. 자동운전 - 엔코더 변경
    * 교차로 태그
    *
    """
    #return
    AutoStop = False
    try:
        sEPCKey = RFID_RESULT.EPC.name
        sRSSIKey = RFID_RESULT.RSSI.name
        sPOS_Key = MonitoringField.CUR_POS.name
        sSPD_Key = MonitoringField.CUR_SPD.name
        curtime = getDateTime()
        node_CtlCenter_globals.dicTopicCallbackTimeStamp[topic_name] = curtime
        recvData = data.data
        if is_json(recvData):
            recvDataMap = json.loads(recvData)
        else:
            recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        
        sEPC = recvDataMap.get(sEPCKey,RailNodeInfo.NOTAG.name)
        sRSSI = try_parse_int(recvDataMap.get(sRSSIKey,-1))
        sPOS = try_parse_int(recvDataMap.get(sPOS_Key,-1))
        sSPD = try_parse_int(recvDataMap.get(sSPD_Key,-1))
        if len(sEPC) < 24:
            node_CtlCenter_globals.dicInv_last.update(recvDataMap)
            return
        else:
            node_CtlCenter_globals.dicEPC_last.update(recvDataMap)
        return
        dicSpdSlowH = getMotorSpeedDic(ModbusID.MOTOR_H.value, True, DEFAULT_RPM_SLOW, ACC_MOVE_H,DECC_MOVE_H)
        dicSpdFastH = getMotorSpeedDic(ModbusID.MOTOR_H.value, True, SPD_MOVE_H, ACC_MOVE_H,DECC_MOVE_H)            
        #if isinstance(recvDataMapList, list):
        #for recvDataMap in recvDataMapList:
        
        if abs(sRSSI) > 16:
          return
        
        if sSPD == 0:
          return
      
        curNode = node_CtlCenter_globals.EPCNodeInfo.get(sEPC, -1)        
        lastSeenEPC = node_CtlCenter_globals.dicRFIDTmp.get(sEPC, DATETIME_OLD)
        node_CtlCenter_globals.dicRFIDTmp[sEPC] = curtime
        node_CtlCenter_globals.dicEPC_last[sEPC]=sSPD
        #return
        #rospy.loginfo(f'curtime:{curtime},sEPC:{sEPC},sPOS:{sPOS}')
        SendInfoHTTP(f"curNode:{curNode},RFID:{sEPC},RSSI:{sRSSI},POS:{sPOS},SPD:{sSPD}")
        if sEPC != isTimeExceeded(lastSeenEPC,DECC_MOVE_H):
            df = GetDF()
            isMoving = isActivatedMotor(ModbusID.MOTOR_H.value)
            #rospy.loginfo(f'isMoving:{isMoving}')
            #if True:
            #dicMoveTeleSrv = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value, True, 0 ,DEFAULT_RPM_SLOW ,  ACC_ST,DECC_ST)
            if len(node_CtlCenter_globals.lsHistory_motorH) > 0:
              dicMotorH = node_CtlCenter_globals.lsHistory_motorH[-1]
              endNode = dicMotorH.get(SeqMapField.END_NODE.name)
              crossNode=find_key_by_nested_value(node_CtlCenter_globals.StateInfo, curNode)
            #   if endNode in node_CtlCenter_globals.StateInfo.keys():
            #       lsCross = node_CtlCenter_globals.StateInfo[endNode]
            #       if curNode in lsCross:
            #           #분기기 인접노드인 경우 속도줄임 + 텔레전개
            #           SendCMD_Device([dicSpdSlowH,dicMoveTeleSrv])
            #           TTSAndroid('전개')
                            
              if crossNode is not None:
                  #목적지는 분기기가 아니지만 인접노드인 경우는 분기기 구간을 감속해서 지나가야 한다.
                  lsCross = node_CtlCenter_globals.StateInfo[crossNode]
                  if curNode not in lsCross:
                    curNodeIdx = df[df[APIBLB_FIELDS_NAVI.endnode] == curNode]
                    crossNodeIdx = df[df[APIBLB_FIELDS_NAVI.endnode] == crossNode]
                    if curNodeIdx.index[0] < crossNodeIdx.index[0]:
                        #분기기에 진입하고 있는 중이면 속도를 줄인다
                        TTSAndroid('감속')
                        SendCMD_Device([dicSpdSlowH])
                    else:
                        TTSAndroid('원복.')
                        #분기기에서 벗어나면 속도를 빠르게.
                        SendCMD_Device([dicSpdFastH])
              #endNode = node_KITCHEN_STATION
              endNodeRFID = node_CtlCenter_globals.EPCNodeInfo.get(sEPC)              
              #분기기에 인접한 노드리스트를 추출
              #방금 인식된 EPC 가 분기기와 인접한 노드인지 확인한다
              #rospy.loginfo(f'endNode:{endNode},endNodeRFID:{endNodeRFID}')
              if endNode == endNodeRFID and isMoving:
                stopdecc = EMERGENCY_DECC
                if abs(int(sSPD)) < 100:
                    stopdecc = ACC_DECC_SMOOTH
                StopMotor(ModbusID.MOTOR_H.value, stopdecc)
                SendInfoHTTP(f"Stopped at curnode:{curNode}, endNode:{endNode} {sPOS},SPD:{sSPD},EPC:{sEPC},RSSI:{sRSSI},DECC:{stopdecc}")
                RFIDControl(False)
                return
            else: #조그운전인 경우 AUTOSTOP
                return
            return
            dicEPCInfo = GetEPCInfo(sEPC)
            rospy.loginfo(dicEPCInfo)
            spd_h = int(GetItemsFromModbusTable(ModbusID.MOTOR_H,MonitoringField.CUR_SPD))
            TAG_DIRECTION = int(dicEPCInfo[EPCINFO_FIELDS.TAG_DIRECTION.name])
            TAG_TYPE = dicEPCInfo[EPCINFO_FIELDS.TAG_TYPE.name]
            LINK_ID = dicEPCInfo[EPCINFO_FIELDS.LINK_ID.name]
            TAG_X = dicEPCInfo[EPCINFO_FIELDS.TAG_X.name]
            TAG_Y = dicEPCInfo[EPCINFO_FIELDS.TAG_Y.name]
            lsCmd = []
            if TAG_TYPE == EPCINFO_TAG_TYPE.D.name:
              if (spd_h > 0 and TAG_DIRECTION == EPCINFO_TAG_DIRECTION.POSITIVE.value) or (spd_h < 0 and TAG_DIRECTION == EPCINFO_TAG_DIRECTION.NEGATIVE.value):
                lsCmd.append(getMotorStopDic(ModbusID.MOTOR_H.value,EMERGENCY_DECC))
                UpdateXY(TAG_X,TAG_Y)
            elif (spd_h > 0 and TAG_DIRECTION == EPCINFO_TAG_DIRECTION.POSITIVE.value) or (spd_h < 0 and TAG_DIRECTION == EPCINFO_TAG_DIRECTION.NEGATIVE.value):
              lsCmd.append(getMotorSpeedDic(ModbusID.MOTOR_H.value, True, DEFAULT_RPM_NORMAL, ACC_MOVE_H,DECC_MOVE_H))
            elif (spd_h < 0 and TAG_DIRECTION == EPCINFO_TAG_DIRECTION.POSITIVE.value) or (spd_h > 0 and TAG_DIRECTION == EPCINFO_TAG_DIRECTION.NEGATIVE.value):
              lsCmd.append(getMotorSpeedDic(ModbusID.MOTOR_H.value, True, SPD_MOVE_H, ACC_MOVE_H,DECC_MOVE_H))
            elif (TAG_DIRECTION == EPCINFO_TAG_DIRECTION.BOTH.value):
              if LINK_ID == endNode:  #주행 목적지가 endNode 가 아닌 경우!
                UpdateXY(TAG_X,TAG_Y)
                SendCMD_Device([getMotorStopDic(ModbusID.MOTOR_H.value,EMERGENCY_DECC)])
                cmdPosH,curPosH = GetPosServo(ModbusID.MOTOR_H)
                while abs(curPosH) > roundPulse:
                  time.sleep(MODBUS_WRITE_DELAY)
                  SendCMD_Device([getMotorHomeDic(ModbusID.MOTOR_H.value)])
                  cmdPosH,curPosH = GetPosServo(ModbusID.MOTOR_H)
                  rospy.loginfo(curPosH)
                
              if node_CtlCenter_globals.lastCrossEPC != sEPC:
                node_CtlCenter_globals.lastCrossEPC = sEPC
            # elif (abs(spd_h) > 500 and TAG_DIRECTION == EPCINFO_TAG_DIRECTION.BOTH.value):
            #   dicSpd = getMotorStopDic(ModbusID.MOTOR_H.value,DECC_MOVE_H)
            # elif (abs(spd_h) <=500 and TAG_DIRECTION == EPCINFO_TAG_DIRECTION.BOTH.value):
            #   dicSpd = getMotorStopDic(ModbusID.MOTOR_H.value,EMERGENCY_DECC)            
            # if not isMoving:
            #   UpdateXY(TAG_X,TAG_Y)
            if len(lsCmd) > 0 and isMoving:
              if len(lsCmd) == 1:
                dicTmp = lsCmd[0]
                print(dicTmp)
                if dicTmp[MotorWMOVEParams.CMD.name] == MotorCmdField.WSTOP.name and TAG_TYPE == EPCINFO_TAG_TYPE.D.name:
                  StopEmergency(ALM_error.DEADEND_DETECTED.value)
                else:
                  SendCMD_Device(lsCmd)
              # else:
              #   for i in range(len(lsCmd)):
              #     SendCMD_Device([lsCmd[i]])
              #     time.sleep(MODBUS_WRITE_DELAY*3)
              rospy.loginfo(lsCmd)
    # else:
          
        return
            # if sEPC == epcNot:
            #     tmpData = getStr_FromFile(filePath_CMD_RECONNECT)
            #     df_fromSPGMAP.join(str2frame(tmpData,'\t'))
            #
            # el
            # if lastEPC != sEPC:
            # if True:
            #     differenceEPC = 0
            #     chkStatusH, chkStatusV = getMultiEx(dic_485, "TARG")
            #     savedEPC = dicRFID.get(sEPC, "")
            #     # if lastEPC != sEPC and chkStatusH:
            #     #     rospy.loginfo(f'RFID Recv : {recvData},{sEPC} at Pos : {curLoc}')
            #     if mapping:
            #         dicRFID[curLoc] = sEPC
            #         if lastEPC != sEPC:
            #             rospy.loginfo(f"RFID Mapping : {sEPC} at {curLoc}")

            #     # if lastEPC != sEPC and is_digit(curLoc) and chkStatusH != strREADY:
            #     if chkStatusH != strREADY:  # 수평모터 동작중
            #         mapSPD = try_parse_int(dicSPDMAP.get(sEPC, ""))
            #         if LastActionCmd == dirCaption_Backward:  # 복귀방향
            #             if mapSPD > 0:
            #                 if not SpdDown:
            #                     rospy.loginfo(
            #                         f"Slow Tag Detected from Mapping : {sEPC} spd : {mapSPD} at {curLoc}"
            #                     )
            #                     # ChargeNestEnable(False)
            #                     motorMove(0, mapSPD, dirCaption_R, False, None, None)
            #                     SpdDown = True
            #             else:
            #                 if not SpdBoost:
            #                     rospy.loginfo(
            #                         f"Fastest Tag Detected from Mapping : {sEPC} at {curLoc}"
            #                     )
            #                     motorMove(0, 100, dirCaption_R, False, None, None)
            #                     SpdBoost = True

            #     if (
            #         lastEPC != sEPC
            #         and is_digit(curLoc)
            #         and chkStatusH != strREADY
            #         and savedEPC != ""
            #     ):
            #         # rospy.loginfo(f'RFID Detected : {sEPC} - {curLoc}')
            #         # differenceEPC = abs(savedEPC) -
            #         curLocABS = abs(int(curLoc))
            #         curLocEPC = abs(int(savedEPC))
            #         differenceEPC = abs(curLocEPC - curLocABS)
            #         differenceMM = differenceEPC / param_HEncoderPulse
            #         rospy.loginfo(
            #             f"RFID Recv with {LastActionCmd} : {recvData},{sEPC} at Pos : {curLoc} : saved {savedEPC}, difference : {differenceEPC}({differenceMM : 0.2f} mm)"
            #         )
            #     elif savedEPC != "" and lastEPC != sEPC:
            #         rospy.loginfo(
            #             f"Invalid RFID  : {sEPC} with savedEPC : {savedEPC}, curLoc : {curLoc}"
            #         )
            #     lastEPC = sEPC
            #     if AutoStop and sEPC is not None:
            #         rospy.loginfo(f"Stop by {sEPC}")
            #         motorStop(drvCaption_H, True)
            # SendFeedback(recvData)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        SendAlarmHTTP(message,True,node_CtlCenter_globals.BLB_ANDROID_IP)
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


def callbackTopic2mqtt(data,topic_name=''):
    """RECEIVE_MQTT 에 수신된 메세지를 파싱하여 범블비를 제어한다
    node_MQTT 노드에서 MQTT 를 수신하여 RECEIVE_MQTT 로 발행하고 그 메세지를 여기서 수신하여 제어함

    Args: 
    웹에서 수신 
    data: "{\"TOPIC\": \"BLB/CALL\", \"PAYLOAD\": \"98\"}"
    
    main 루틴에서 수신 (크로스 브릿지 및 리프트 아두이노 참고용)
    data: "{\"TOPIC\": \"BLB/CROSS_CMD/set\", \"PAYLOAD\": \"{\\\"99\\\": -1, \\\"2\\\": -1,
    \\ \\\"1\\\": -1}\"}"
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

        data_out = {}
        devID = recvDataMap.get('DEV',None)
        if devID is not None:
            sTOPIC_MQTT = f'{devID}_{devID}'
            sPAYLOAD = recvData

        # 유효하지 않은 메세지면 함수 종료
        if sPAYLOAD == None or sTOPIC_MQTT == None:
            return

        # MQTT 토픽별 처리루틴
        if isinstance(sPAYLOAD,dict):
            node_CtlCenter_globals.stateDic.update(sPAYLOAD)
        elif is_json(sPAYLOAD):
            # 그외의 경우에
            data_out = json.loads(sPAYLOAD)
            data_out[CALLBELL_FIELD.TIMESTAMP.name] = getCurrentTime(spliter="")
            nodeID = sTOPIC_MQTT.split("_")[-1]
            if sTOPIC_MQTT.find(CALLBELL_FIELD.CALLBELL.name) > 0:
                #'BLB/mcu_relay_100' : 교차로에서 발행하는 메세지
                #'BLB/mcu_relay_CALLBELL_1' : 호출벨에서 온 메세지인 경우 토픽명에 CALLBELL 이란 단어가 있음.
                if data_out.get(CALLBELL_FIELD.BTN_RED.value, None) == 1:
                    if node_CtlCenter_globals.runFromLaunch:
                        TTSAndroid(f"{GetKoreanFromNumber(nodeID)}번 테이블에서 직원을 호출하셨습니다.")
                        # SendMsgToMQTT(
                        #     pub_topic2mqtt,
                        #     MQTT_TOPIC_VALUE.TTS.value,
                        #     f"{GetKoreanFromNumber(nodeID)}번 테이블에서 직원을 호출하셨습니다.",
                        # )
                        # SendMsgToMQTT(mqtt_topic_TTS,f'{nodeID}번 테이블에서 직원을 호출하셨습니다.')
                elif data_out.get(CALLBELL_FIELD.BTN_BLUE.value, None) == 1:
                    if data_out.get(CALLBELL_FIELD.CALL_STATE.value, None) == 1:
                        if node_CtlCenter_globals.runFromLaunch:
                            # SendMsgToMQTT(
                            #     pub_topic2mqtt,
                            #     MQTT_TOPIC_VALUE.TTS.value,
                            #     f"테이블 {GetKoreanFromNumber(nodeID)}번에서 로봇 호출을 취소하셨습니다.",
                            #)
                            TTSAndroid(f"테이블 {GetKoreanFromNumber(nodeID)}번에서 로봇 호출을 취소하셨습니다.")
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
        SendAlarmHTTP(e,True,node_CtlCenter_globals.BLB_ANDROID_IP)

for item in ModbusID:
    subTopicName = f'{TopicName.MB_.name}{item.value}'
    rospy.Subscriber(subTopicName, String, callbackModbus, queue_size=ROS_TOPIC_QUEUE_SIZE,callback_args=subTopicName)
    node_CtlCenter_globals.numSubTopics += 1
    rospy.loginfo(f'모드버스 구독번호 {node_CtlCenter_globals.numSubTopics}:{subTopicName}')
    #rospy.loginfo(f'모드버스 구독번호 {node_CtlCenter_globals.numSubTopics}:{subTopicName}')

x, y, z = 0.5, 0.0, 0.0  # Example target position, modify as needed
dim_x,dim_y,dim_z = 1, 0.1, 0.1

# 전역 변수로 포인트 클라우드의 총 포인트 수를 저장합니다.
original_point_count = None
# vis = o3d.visualization.VisualizerWithKeyCallback()
pcd = o3d.geometry.PointCloud()

  
def point_cloud_callback(msg2):
  mbidTestMotor = 3
  isMoving = isActivatedMotor(mbidTestMotor)
  if not isMoving:
    return
  
  #recvDataMapLift = node_CtlCenter_globals.dic_485ex.get(mbidTestMotor, None)  
  #cmd_pos, cur_pos = GetPosServo(ModbusID.ROTATE_TESTER)
  # cloud = pc2.read_points(msg2, field_names=("x", "y", "z"), skip_nans=True)
  # points = list(cloud)
#   node_CtlCenter_globals.dicSafety[cur_pos] = remove_points_near_origin(msg2,0.2)
  
# def point_cloud_callback2(msg):
#     global lastTimeStamp, original_point_count 
#     if not isTimeExceeded(lastTimeStamp, 1000):
#         return
#     lastTimeStamp = getDateTime()
#     try:
#         if tf_buffer:
#             try:
#                 # Transform listener
#                 transform = tf_buffer.lookup_transform("laser_frame", msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
#                 # Rotate the pointcloud
#                 msg2 = tf2_sensor_msgs.do_transform_cloud(msg, transform)
#             except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
#                 rospy.logerr(f"Transform error: {e}")
#                 msg2 = msg
#         else:
#             msg2 = msg

#         cloud = pc2.read_points(msg2, field_names=("x", "y", "z"), skip_nans=True)
        
#         # Convert to list of points
#         points = list(cloud)
        
#         # 원래 포인트 수를 설정합니다.
#         if original_point_count is None:
#             original_point_count = len(points)
        
#         # 현재 포인트 수가 원래 포인트 수의 50% 이하인지 확인합니다.
#         current_point_count = len(points)
#         if current_point_count <= 0.5 * original_point_count:
#             rospy.logwarn("Point cloud density is less than 50% of the original. Possible obstacle too close.")

#         rotated_points = rotate_point_cloud(points, 'x', 0)

#         # Publish the transformed PointCloud2 message
#         transformed_pc2 = pc2.create_cloud_xyz32(msg2.header, rotated_points)
#         transformed_cloud_pub.publish(transformed_pc2)
        
#         # Convert to Numpy array
#         np_points = np.array(rotated_points, dtype=np.float32)

#         # Convert to PCL PointCloud
#         pcl_data = pcl.PointCloud()
#         pcl_data.from_array(np_points)

#         # Define the target area
#         target_position = (0.5, 0.0, 0.0)  # 50cm 앞을 중심으로 설정
#         target_dimensions = (0.05, 0.05, 1.0)  # 5cm x 5cm x 1m 영역

#         if is_area_clear(pcl_data, target_position, target_dimensions):
#             rospy.loginfo("Area is clear for the robot arm to extend.")
#         else:
#             rospy.loginfo("Area is not clear. Obstruction detected.")
#             # Publish the obstacles point cloud
#             obstacles_pc2 = pc2.create_cloud_xyz32(msg2.header, np_points)
#             obstacles_cloud_pub.publish(obstacles_pc2)

#     except Exception as e:
#         rospy.logerr(f"Unexpected error: {e}")
#         SendAlarmHTTP(e,True,node_CtlCenter_globals.BLB_ANDROID_IP)

def callback_detect_3D(data):
    node_CtlCenter_globals.last_detect_3d = data

        
# TopicName과 Callback 함수를 정의한 사전
node_CtlCenter_globals.dictTopicToSubscribe = {
    #TopicName.CROSS_INFO.name: callbackCROSS,
    # TopicName.RFID.name: callbackRFID,
    # TopicName.RFID_DF.name: callbackRFID_DF,
    TopicName.ARD_CARRIER.name: callbackARD_CARRIER,
    TopicName.ANDROID.name: callbackARD_CARRIER,
    TopicName.ACK.name: callbackAck,
    TopicName.BLB_CMD.name : callbackBLB_CMD,
    TopicName.BMS.name : callbackModbus,
    TopicName.RECEIVE_MQTT.name : callbackTopic2mqtt,
    TopicName.ARUCO_RESULT.name : callback_ARUCO_RESULT,
    TopicName.CROSS_INFO.name : callback_CROSS_INFO,
    TopicName.SMARTPLUG_INFO.name : callback_CROSS_INFO,    
    TopicName.LIDAR_OBSTACLE.name : callback_lidar_obstacle
    # ,
    # '/bumblebee/distance' : callBackLidarDistance,
    # '/detect_cropped_distance' : callBackLidarDistanceOnly
}
for topic, callback in node_CtlCenter_globals.dictTopicToSubscribe.items():
    rospy.Subscriber(topic, String, callback, queue_size=ROS_TOPIC_QUEUE_SIZE,callback_args=topic)

#rospy.Subscriber(TopicName.detect_3D.name, PointCloud2, callback_detect_3D)
#rospy.Subscriber(TopicName.BLB_TRAY_IMU.name, Imu, imu_callback)
# rospy.Subscriber(TopicName.RFID.name, String, callbackRFID, queue_size=ROS_TOPIC_QUEUE_SIZE)
# rospy.Subscriber(TopicName.ARD_CARRIER.name, String, callbackARD_CARRIER, queue_size=ROS_TOPIC_QUEUE_SIZE)
# rospy.Subscriber(TopicName.ACK.name, String, callbackAck, queue_size=ROS_TOPIC_QUEUE_SIZE)
# rospy.Subscriber(TopicName.BLB_CMD.name, String, callbackBLB_CMD, queue_size=ROS_TOPIC_QUEUE_SIZE)
# rospy.Subscriber(TopicName.RECEIVE_MQTT.name, String, callbackTopic2mqtt, queue_size=ROS_TOPIC_QUEUE_SIZE)
# rospy.Subscriber(TopicName.ARUCO_RESULT.name, String, callback_ARUCO_RESULT, queue_size=ROS_TOPIC_QUEUE_SIZE)
#rospy.Subscriber(TopicName.BLB_CMD.name, String, callbackBLB_CMD, queue_size=ROS_TOPIC_QUEUE_SIZE,callback_args=TopicName.BLB_CMD.name)  # UI에서 명령어 수신
print(os.path.splitext(os.path.basename(__file__))[0],getDateTime())