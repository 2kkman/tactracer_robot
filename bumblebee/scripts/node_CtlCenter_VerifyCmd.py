#!/usr/bin/env python3
from node_CtlCenter_processes import *

def RunListBlbMotorsEx(listBLB):
    lsMotorOperationNew = []
    isReady_Start = False
    isReady_End = False
    runningMotors = getRunningMotorsBLB()
    isTimePassed = isTimeExceeded(GetLastCmdTimeStamp(), MODBUS_EXECUTE_DELAY_ms)
    if len(runningMotors) > 0 or not isTimePassed:
        return APIBLB_ACTION_REPLY.E105
    if len(listBLB) == 0:
        return APIBLB_ACTION_REPLY.E104
    node_CtlCenter_globals.lock.acquire()
    dicInfo_local = listBLB[0]
    node_CtlCenter_globals.lock.release()
    if isinstance(dicInfo_local, dict) and GetWaitConfirmFlag():  # 주행모드
        return APIBLB_ACTION_REPLY.E110
    nextAction = {}
    curTargetTable,curTargetNode = GetCurrentTargetTable()
    curNode = GetCurrentNode()
    #curTargetTable,curTarNode = GetTargetTableNode()
    df = GetDF(curTargetTable)    
    tiltStutus = GetTiltStatus()
    dicTagretTableInfoCurrent = getTableServingInfo(curTargetTable)
    target540 = dicTagretTableInfoCurrent.get(TableInfo.SERVING_ANGLE.name)
    targetH = dicTagretTableInfoCurrent.get(TableInfo.MOVE_DISTANCE.name)
    isTeaching540 = True if target540 == StateBranchValue.ERROR.value and targetH != StateBranchValue.ERROR.value else False
    targetServingDistance = dicTagretTableInfoCurrent.get(TableInfo.SERVING_DISTANCE.name, StateBranchValue.ERROR.value)
    isTeachingServingDistance = True if targetServingDistance == StateBranchValue.ERROR.value else False
    pot_telesrv,not_telesrv,cmdpos_srv,cur_pos_srv =GetPotNotCurPosServo(ModbusID.TELE_SERV_MAIN)
    pot_6,not_6,cmdpos_6,cur_pos_6 =GetPotNotCurPosServo(ModbusID.MOTOR_V)
    pot_31,not_31,cmdpos_31,cur_pos_31 =GetPotNotCurPosServo(ModbusID.ROTATE_SERVE_360)
    pot_13,not_13,cmdpos_13,cur_pos_13 =GetPotNotCurPosServo(ModbusID.BAL_ARM1)
    potRotate540_cmd, notRotate540_cur, posRotate540_cmd,posRotate540_cur= GetPotNotCurPosServo(ModbusID.ROTATE_MAIN_540)    
    potRotate540, notRotate540, poscmdRotate540,poscurRotate540= GetPotNotCurPosServo(ModbusID.ROTATE_MAIN_540)
    dicInfo_local_org = copy.deepcopy(dicInfo_local)
    #get_tasmota_info
    #현재 뻗은 암 길이 / 메인회전각도 / 트레이각도
    cmd_posH, cur_posH = GetPosServo(ModbusID.MOTOR_H)
    DI_POT,DI_NOT,DI_HOME,SI_POT = GetPotNotHomeStatus(ModbusID.MOTOR_H)  
    DI_POT_540,DI_NOT_540,DI_HOME_540,SI_POT_540 = GetPotNotHomeStatus(ModbusID.ROTATE_MAIN_540)  
    DI_POT_360,DI_NOT_360,DI_HOME_360,SI_POT_360 = GetPotNotHomeStatus(ModbusID.ROTATE_SERVE_360)  
    cur_pos_mm = pulseH_to_distance(cur_posH)
    curDistanceSrvTele, curAngle_540,cur_angle_360  = GetCurrentPosDistanceAngle()
    #dicInfo_local = listBLB[0]
    stateCharger = isChargerPlugOn()
    onScan = isScanTableMode(curTargetTable)
    dicTagretTableInfo = getTableServingInfo(curTargetTable)
    infoLIFT_Height = try_parse_int(dicTagretTableInfo.get(TableInfo.MARKER_VALUE.name), 0)
    finalScan = not is_equal(infoLIFT_Height,curTargetTable)
    dicAruco = {}
    lsAruco = filter_recent_data(ARUCO_RESULT_FIELD.LASTSEEN.name,GetArucoMarkerInfo(),0.2)
    angle_y = node_CtlCenter_globals.dicARD_CARRIER.get(DataKey.Angle_Y.name)
    if len(lsAruco) > 0:
        dicAruco = lsAruco[0]
        #lsDF = GetNewRotateArmList(dicAruco)
    
    
    if isinstance(dicInfo_local, dict):  # 주행모드
        mbid = dicInfo_local.get(MotorWMOVEParams.MBID.name)
        sPOS = dicInfo_local.get(MotorWMOVEParams.POS.name)            
        sPOS_R = dicInfo_local.get(APIBLB_FIELDS_TASK.distance_total.name, MIN_INT)
        sDir = dicInfo_local.get(SeqMapField.DIRECTION.name)
        sPOS_R = dicInfo_local[SeqMapField.DISTANCE.name]         
        start_node = dicInfo_local.get(SeqMapField.START_NODE.name)
        end_node = dicInfo_local.get(SeqMapField.END_NODE.name)
        bIsAllMotorFolded = isReadyToMoveH_and_540()
        if end_node == GetCurrentNode() or end_node == start_node:
            listBLB.pop(0)
            # UpdateLastCmdTimeStamp()
            # UpdateLastBalanceTimeStamp()                
            return APIBLB_ACTION_REPLY.E108

        if not bIsAllMotorFolded:
            DoorClose()
            lsLiftUp = GetLiftControl(True)
            lsBLBTmp = copy.deepcopy(listBLB)
            listBLB[:] = lsLiftUp + lsBLBTmp            
            return APIBLB_ACTION_REPLY.E106
        else:
            if sPOS is None:
                donCare = -1
                #서버에서 받아온 정보가 있는 경우
                if dicInfo_local.get(APIBLB_FIELDS_TASK.detailcode_list.name) is None:
                  rospy.loginfo(dicInfo_local)
                else:
                  lsCheckPoints = calculate_checkpoints(dicInfo_local,cur_posH)
                  if lsCheckPoints is not None:
                    lsDetailcode = list(str(dicInfo_local[APIBLB_FIELDS_TASK.detailcode_list.name]).split(sDivItemComma))
                    iCntRecords = len(lsCheckPoints)
                    if iCntRecords == len(lsDetailcode):
                        key_pos = APIBLB_FIELDS_TASK.POS_ABS.name
                        node_CtlCenter_globals.dicTargetPosFeedBack.clear()
                        # 'POS_ABS' 필드가 없으면 생성하고, 기본값을 0으로 설정
                        if df is None or df.empty:
                          rospy.loginfo(f'{curTargetTable} DF is None')
                          print(node_CtlCenter_globals.dic_DF)
                        else:
                          if key_pos not in df.columns:
                              df[key_pos] = 0
                          for i in range(iCntRecords):
                              keyCheckPoint = lsCheckPoints[i]
                              valueDetailCode = lsDetailcode[i]
                              df.loc[df[APIBLB_FIELDS_TASK.detailcode.name].astype(int) == int(valueDetailCode), key_pos] = keyCheckPoint
                              node_CtlCenter_globals.dicTargetPosFeedBack[keyCheckPoint]=int(valueDetailCode)
                          SendInfoHTTP(f'cur_posH={cur_posH},{str(node_CtlCenter_globals.dicTargetPosFeedBack)}')
                          fill_pos_abs(df)
                          result = dict(zip(df[APIBLB_FIELDS_TASK.endnode.name],df[key_pos]))
                          node_CtlCenter_globals.dicPOS_ABS.update(result)
                          SendInfoHTTP(str(node_CtlCenter_globals.dicPOS_ABS))
                          PrintDF(df)
                          rospy.loginfo(node_CtlCenter_globals.dicTargetPosFeedBack)
                        
                curX,curY = GetLocXY()
                rospy.loginfo(f'현재Pulse:{cur_posH},펄스거리:{cur_pos_mm},현지점:{curX},{curY},중간노드좌표:{node_CtlCenter_globals.dicTargetPosFeedBack},남은테이블:{node_CtlCenter_globals.listTable}')
                
                iSpdH = round(SPD_MOVE_H*SPEED_RATE_H)
                iPOS_R = try_parse_int(sPOS_R)
                if iPOS_R == 0:
                  rospy.loginfo(dicInfo_local)
                rospy.loginfo('CheckPoint')
                
                if sDir is None:
                  iPOS_R = (distance_to_pulseH(iPOS_R))
                elif is_equal(sPOS,MIN_INT):
                  iPOS_R = abs(distance_to_pulseH(iPOS_R))
                  if is_equal(sDir,'S') or is_equal(sDir ,'W'):
                      iPOS_R = -iPOS_R
                else:
                  iPOS_R = distance_to_pulseH(iPOS_R)
                rospy.loginfo('CheckPointH1')
                iTargetPulse = iPOS_R + cur_posH
                cs = dicInfo_local.get(SeqMapField.CROSS_STATUS.name)
                bCrossCheck = True
                strStatusInfo = ''
                rospy.loginfo('CheckPointH2')
                if cs is not None and isinstance(cs, dict):
                  rospy.loginfo(cs)
                  for cross_node,cross_status in cs.items():
                    setNodeStateEx(cross_node, cross_status)
                    iCurStartState = getNodeState(cross_node)
                    strStatusInfo += f'{cross_node}:{cross_status}->{iCurStartState},'
                    if cross_status != iCurStartState:
                      bCrossCheck = False
                #if False:   #임시로 분기기는 무조건 True 라고 가정하자.
                if bCrossCheck == False and isRealMachine:
                    if NODE_CROSS == GetCurrentNode():
                        SetWaitCrossFlag(True)
                        UpdateLastCmdTimeStamp()
                        message=f'남은테이블:{node_CtlCenter_globals.listTable},분기기 상황:{strStatusInfo}'
                        PrintStatusInfoEverySec()
                        rospy.loginfo_throttle(20, message)       
                        return APIBLB_ACTION_REPLY.E107
                    else:
                        StopEmergency(ALM_User.CROSS_STATUS_INVALID.value)
                        return APIBLB_ACTION_REPLY.E107
                #데모에서는 일단 이렇게
                dicInfo_local = getMotorMoveDic(ModbusID.MOTOR_H.value, True, iTargetPulse, iSpdH, ACC_MOVE_H,DECC_MOVE_H)
                SetWaitCrossFlag(False)

            node_CtlCenter_globals.dicTargetPos[str(ModbusID.MOTOR_H.value)] = iTargetPulse
            diffH_pulse = abs(cur_posH - iTargetPulse)
            node_CtlCenter_globals.lastSendDic_H = dicInfo_local
            #dicRotateDirection = getMainRotateDicByDirection(dicInfo_local)
            dicRotateDirection = getMainRotateDicByNode(end_node)
            rospy.loginfo('CheckPointH3')
            if CheckMotorOrderValid(dicInfo_local):
                rospy.loginfo('CheckPointH4')
                #if CheckMotorOrderValid(dicRotateDirection):
                #curDistanceSrvTele, curAngle_540,cur_angle_360  = GetCurrentPosDistanceAngle()
                if CheckMotorOrderValid(dicRotateDirection):
                    #if not ((curAngle_540 == 0 or curAngle_540 == 180) and diffH_pulse < MOVE_H_SAMPLE_PULSE/3):
                        target540_pos = dicRotateDirection.get(MotorWMOVEParams.POS.name,MIN_INT)
                        lsDicRotateDirection = [dicRotateDirection]
                        listBLB.insert(0,lsDicRotateDirection)
                        return APIBLB_ACTION_REPLY.E108
                TiltFace()
                endnode_current = dicInfo_local_org.get(SeqMapField.END_NODE.name)
                pulseTarget= GetNodePos_fromNode_ID(endnode_current)
                dicInfo_local[MotorWMOVEParams.POS.name] = pulseTarget
                rospy.loginfo(dicInfo_local)
                if dicInfo_local_org.get(SeqMapField.END_NODE.name) is not None:
                    node_CtlCenter_globals.lsHistory_motorH.append(dicInfo_local_org)
                if isRealMachine:
                    bReturn,strResult=API_MoveH(pulseTarget,dicInfo_local.get(MotorWMOVEParams.SPD.name),endnode_current)
                    rospy.loginfo(f'현재펄스:{cur_posH}, 타겟펄스:{pulseTarget}')
                    bReturn,strResult=GetResultMessageFromJsonStr(strResult)
                    rospy.loginfo(f'{bReturn},{strResult}')
                    if not isTrue(bReturn):
                        node_CtlCenter_globals.listBLB.insert(0,dicInfo_local_org)
                        #SetWaitConfirmFlag(True, AlarmCodeList.JOB_PAUSE)
                        return APIBLB_ACTION_REPLY.E110
                else:
                    SendCMD_DeviceService([dicInfo_local])
                rospy.loginfo(f"Moving H motor : {node_CtlCenter_globals.nStart}->{node_CtlCenter_globals.nTarget}({listBLB.pop(0)})")
            else:
                rospy.loginfo(f"Skipped H motor : {node_CtlCenter_globals.nStart}->{node_CtlCenter_globals.nTarget}({listBLB.pop(0)})")

            ClearArucoTable()
            UpdateLastCmdTimeStamp()
            PrintStatusInfoEverySec(1)
            return APIBLB_ACTION_REPLY.R101
    elif isinstance(dicInfo_local, list):
        #cmd_pos11,cur_pos11=GetPosServo(ModbusID.TELE_SERV_MAIN)
        #현재 노드가 NONE 타입이 아니라면 반드시 마지막 H 모터의 
        #종료코드는 IsPOT == 1 이어야 위치결정이 완료된것임. 수정할것
        #현재노드가 NONE 타입 이라면 한번에 멈출 예정.
        #가장 인접한 노드에서 move 명령어를 한번 더 내보낸다.
        curNode_Type = node_CtlCenter_globals.dicLast_POSITION_INFO[RFID_RESULT.EPC.name]
        curNode_diPot = node_CtlCenter_globals.dicLast_POSITION_INFO[MonitoringField.DI_POT.name]
        if curNode_Type.find(strNOTAG) >= 0 and is_equal(curNode_diPot,0) and isRealMachine:
            return APIBLB_ACTION_REPLY.E112
        
        filtered_data = [item for item in dicInfo_local if item]
        if len(filtered_data) > 0:
            #print(dicInfo_local)
            #GetWaitConfirmFlag 가 활성화 되어 있는 경우 암을 펼치거나 리프트다운을 하지 않는다.
            dfReceived = pd.DataFrame(filtered_data) 
            try:
                dfReceived[MotorWMOVEParams.POS.name] = dfReceived[MotorWMOVEParams.POS.name].astype(int)
                dfReceived[MotorWMOVEParams.MBID.name] = dfReceived[MotorWMOVEParams.MBID.name].astype(int)
                lsLiftDown= dfReceived[(dfReceived[MotorWMOVEParams.POS.name] > 0) & (dfReceived[MotorWMOVEParams.MBID.name] == ModbusID.MOTOR_V.value)].tail(1).to_dict(orient='records')
                lsExpandArm= dfReceived[(dfReceived[MotorWMOVEParams.POS.name] > 0) & (dfReceived[MotorWMOVEParams.MBID.name] == ModbusID.TELE_SERV_MAIN.value)].tail(1).to_dict(orient='records')
                if len(lsLiftDown) > 0 or len(lsExpandArm) > 0:
                    if GetWaitConfirmFlag():
                        return
            except Exception as e:
                #단 POS 와 MBID가 없는 명령어인 경우 그냥 내보낸다.
                SendCMD_Device(dicInfo_local)
                listBLB.pop(0)
        
        # 루프를 돌면서 원소를 직접 수정할 수 있게 한다.
        isArmControl = False
        isLiftControl = False
        distance_target = 0
        isRotateMainControl = False
        isRotateTrayControl = False
        adjustrate = node_CtlCenter_globals.adjustrate
        valuesInlist = len(dicInfo_local)
        for i in range(valuesInlist):
            dicArray = dicInfo_local[i]
            if not isinstance(dicArray,dict):
                rospy.logerr(dicArray)
                continue
            sMBID = dicArray.get(MotorWMOVEParams.MBID.name,None)
            sPOS = dicArray.get(MotorWMOVEParams.POS.name)
            if sMBID == None:
                print(dicArray)
                continue
            if sPOS is None:
                print(dicArray)
                continue
            iMBID = int(sMBID)
            sMBIDInstance = ModbusID.from_value(iMBID)
            iPOS = int(sPOS)
            sSPD = dicArray[MotorWMOVEParams.SPD.name]
            iSPD = int(sSPD)
            iSPDSlow = round(iSPD /3)
            # if iMBID == ModbusID.MOTOR_V.value:
            #     isLiftControl = True
            #     break
            # elif iMBID == ModbusID.ROTATE_SERVE_360.value:
            #     isRotateTrayControl = True
            #     potRotate360_cmd, notRotate360_cur, posRotate360_cmd,posRotate360_cur= GetPotNotCurPosServo(ModbusID.ROTATE_SERVE_360)
            #     distance_target = pulse_to_angle(iPOS, potRotate360_cmd, MAX_ANGLE_TRAY)%360
            #     break
            # elif iMBID == ModbusID.ROTATE_MAIN_540.value:
            #     isRotateMainControl = True
            #     distance_target = pulse_to_angle(iPOS, potRotate540_cmd, MAX_ANGLE_BLBBODY)%360
            #     break
            # el
            if iMBID == ModbusID.TELE_SERV_MAIN.value:
                isArmControl = True
                distance_target = GetTargetLengthMMServingArm(iPOS,0)
                if iPOS > cur_pos_srv and onScan:
                    adjustrate = 0.3
                    TiltArucoScan()
                    CamControl(True)
                    MotorBalanceControlEx.onCaliT = True
                    dicArray[MotorWMOVEParams.POS.name] = (pot_telesrv - roundPulse*10)
                break
            # #테이블 탐색 모드에서는 속도 줄인다.
            # if iPOS > cur_pos_srv and (valuesInlist > 1) and (onScan or finalScan):
            #     dicArray[MotorWMOVEParams.SPD.name] = iSPDSlow
            # elif (valuesInlist == 1) and iMBID == ModbusID.TELE_SERV_MAIN.value and finalScan and iPOS > cur_pos_srv:
            #     dicArray[MotorWMOVEParams.SPD.name] = iSPDSlow
                
            sACC = dicArray[MotorWMOVEParams.ACC.name]
            iACC = int(sACC)
            sDECC = dicArray[MotorWMOVEParams.DECC.name]
            iDECC = int(sDECC)
            cmd_pos,cur_pos = GetPosServo(sMBIDInstance)
            targetStroke = abs(cur_pos - iPOS)
            targetRoundsToGo = targetStroke / roundPulse
            floatTime = calculate_rpm_time_accdesc(targetRoundsToGo,iSPD,iACC,iDECC)
            dicArray[MotorWMOVEParams.TIME.name] = floatTime

            #메인회전 모터 제어
            if iMBID == ModbusID.ROTATE_MAIN_540.value:# and abs(iPOS) > roundPulse:
                bIsAllMotorFolded540 = isReadyToMoveH_and_540()                
                # if onScan and dicAruco:
                #     lsDF = GetNewRotateArmList(dicAruco)
                #     if lsDF:
                #         rospy.loginfo(json.dumps(dicAruco, indent=4))
                #         node_CtlCenter_globals.listBLB.clear()
                #         node_CtlCenter_globals.listBLB.extend(lsDF)
                #         return APIBLB_ACTION_REPLY.E108
                    
                if not bIsAllMotorFolded540 and iSPD > MAINROTATE_RPM_SLOWEST:
                    # if abs(cur_pos_13) < roundPulse and abs(cur_pos_6) < roundPulse :
                    #     dicMoveTeleSrv = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value, True, not_telesrv,DEFAULT_RPM_SLOW ,ACC_ST,DECC_ST)
                    #     node_CtlCenter_globals.dicTargetPos.clear()
                    #     SendCMD_Device([dicMoveTeleSrv])
                    #     node_CtlCenter_globals.listBLB.insert(0,dicInfo_local_org)
                    #     return APIBLB_ACTION_REPLY.E108
                    # else:
                    #     lsLiftUp = GetLiftControl(True)
                    #     lsBLBTmp = copy.deepcopy(listBLB)
                    DoorClose()
                    lsLiftUp = GetLiftControl(True)
                    lsBLBTmp = copy.deepcopy(listBLB)
                    listBLB[:] = lsLiftUp + listBLB
                    return APIBLB_ACTION_REPLY.E106
                
                sPOS = str(iPOS)  
                lenBLB = len(listBLB)
                lsCurListBLB = []

                if len(listBLB) > 2 and node_CtlCenter_globals.aruco_try == 0:   
                    updatedTable = []
                    arucoInfo = []
                    
                    # if len(GetArucoMarkerDict()) > 0:
                    #     #티칭모드에서 아르코 마커가 인식되었을때
                    #     if isTeaching540:
                    #       avg_values = calculate_average_values(GetArucoMarkerDict())
                    #       dicCurTableScanResult = avg_values.get(curTargetTable, {})
                    #       rospy.loginfo(dicCurTableScanResult)                          
                    #       new540 = (540-dicCurTableScanResult[BLB_LOCATION.ANGLE_360.name])%360
                    #       #updateTableServingInfo(curTargetTable,None,new540,None,None)
                    #       newDic540 = GetDicRotateMotorMain(new540)
                    #       dicInfo_local[i].update(newDic540)
                    #       rospy.loginfo(f'스캔완료 : {dicInfo_local[i]}')
                    #       return APIBLB_ACTION_REPLY.E105
                    #     elif len(lsAruco) > 0:
                    #       #전개전 아르코 마커가 인식되었을때
                    #       dicAruco = lsAruco[0]
                    #       rospy.loginfo(json.dumps(dicAruco, indent=4))                          
                    #     #   result = compute_movement_and_rotation_from_dict(ref_dict, dicAruco)
                    #     #   ROTATE_ANGLE = result[APRIL_RESULT.ROTATE_ANGLE.name:]
                    #     #   dISTANCE_mm = result[APRIL_RESULT.DISTANCE_mm.name:]
                    #     #   mARGIN_mm = result[APRIL_RESULT.MARGIN_mm.name:]
                    #       dISTANCE_mm, ROTATE_ANGLE= compute_position(ref_dict, dicAruco)
                    #       angle_cal = (curAngle_540 + ROTATE_ANGLE) % 360
                    #       distance = curDistanceSrvTele + dISTANCE_mm 
                          
                    #       #newPosX,newPosY = CalculatePosXYFromAruco(dicAruco,(cur/Angle_540+270)%360,-2,5)
                    #       newPosX,newPosY = calculate_coordinates(newPosX,newPosY)
                    #       #distance, angle_cal = calculate_distance_and_angle(newPosX, newPosY)    
                    #       rospy.loginfo(f'PosX,PosY:{newPosX},{newPosY},길이:{distance},각도:{angle_cal}')
                    #       lsNewArmRotate = GetStrArmExtendMain(distance,angle_cal,False)
                    #       listBLB[1] = lsNewArmRotate
                    #       dicNewRotate = GetDicRotateMotorMain(angle_cal)
                    #       rospy.loginfo(f'기존메인회전타켓:{dicInfo_local[0][MotorWMOVEParams.POS.name]},마커타겟:{dicNewRotate[MotorWMOVEParams.POS.name]}')
                    #       dicInfo_local[i] = dicNewRotate
                    # elif isTeaching540: #인식된 아르코마커가 없지만 각도 티칭 모드인 경우
                    #   lsTeach540 = GetCameraRotateCmds()  #각도치팅명령어
                    #   listBLB[:] = lsTeach540 + listBLB
                    #   return APIBLB_ACTION_REPLY.E105
            sCMD = dicArray[MotorWMOVEParams.CMD.name]
            sMODE = dicArray[MotorWMOVEParams.MODE.name]
            sSPD = dicArray[MotorWMOVEParams.SPD.name]
            sACC = dicArray[MotorWMOVEParams.ACC.name]
            sDECC = dicArray[MotorWMOVEParams.DECC.name]
            # if iMBID == ModbusID.TELE_SERV_MAIN.value:
            # #if iMBID == ModbusID.TELE_SERV_INNER.value:
            #   if isTeaching540:
            #     if iPOS == INNERSTEP_PULSE_TRAYMOVE:
            #       TiltDiagonal()
            #       ClearArucoTable()
            #     else:
            #       TiltTrayCenter()
            
            #리프팅 모터 제어정보 사전 점검
            if iMBID == ModbusID.MOTOR_V.value and CheckMotorOrderValid(dicArray):
                if iPOS !=0 and tiltStutus == TRAY_TILT_STATUS.TiltTableObstacleScan and isRealMachine:    #하강전 라이다 스캔 결과 확인후 내려간다.
                    imgPath = capture_frame_from_mjpeg()
                    lsObstacleInfo = get_obstacle_data(1)
                    descendable_distance = node_CtlCenter_globals.DefaultGndDistance
                    isObstaclePresent = len(lsObstacleInfo)
                    rospy.loginfo(json.dumps(lsObstacleInfo, indent=4))
                    bins_points = 0
                    isCoolTimePassed = True
                    if isObstaclePresent:
                        descendable_distance = lsObstacleInfo[-1].get(OBSTACLE_INFO.OBSTACLE_DISTANCE.name)
                        bins_points = lsObstacleInfo[-1].get(OBSTACLE_INFO.OBSTACLE_POINTS.name)
                        isCoolTimePassed = TTSAndroid(TTSMessage.REQUEST_TABLECLEAR.value, 5)
                    # if imgPath is not None and angle_y is not None and isCoolTimePassed:
                    #     save_image_with_lidar_data(imgPath,descendable_distance,angle_y,bins_points)
                    if isObstaclePresent:
                        return APIBLB_ACTION_REPLY.E111

                currentWeight1,currentWeight2,currentWeightTotal = getLoadWeight()

                if currentWeightTotal >= WEIGHT_LOADCELL_LIMITGRAM*10000:
                    SendMsgToMQTT(pub_topic2mqtt,MQTT_TOPIC_VALUE.BLB_ALARM.value,ALM_User.TRAY_WEIGHT_LIMIT.value)
                    SetWaitConfirmFlag(True,ALM_User.TRAY_WEIGHT_LIMIT)
                    return APIBLB_ACTION_REPLY.E109
                else:
                    if iPOS < roundPulse:
                        DoorClose()
                    if dicAruco:    #아르코마커가 인식되면 로그.
                        # #TODO : 아르코마커가 인식되었습니다TTS. + 음성 메세지 클래스 정의할 것.
                        # TTSAndroid(TTSMessage.ARUCO_FOUND_OK.value)                        
                        resultDiff,diff_X,diff_Y= compare_dicts(dicAruco, ref_dict, CAM_LOCATION_MARGIN_OK)
                        rospy.loginfo(format_vars(resultDiff,diff_X,diff_Y))
                        #rospy.loginfo(json.dumps(dicAruco, indent=4))
                        #GetNewRotateArmList(dicAruco)
                        # if diff_Y < 0.1:
                        # #if resultDiff:
                        #     TTSAndroid('테이블 위치파악 성공')
                        # else:
                        #     GetNewRotateArmList(dicAruco)
                        #     node_CtlCenter_globals.aruco_try += 1
                        #     node_CtlCenter_globals.dicARUCO_last.clear()
                        #     lsDF = GetNewRotateArmList(dicAruco,False)
                        #     if lsDF and node_CtlCenter_globals.aruco_try < 3:
                        #         TTSAndroid(f'{node_CtlCenter_globals.aruco_try} 번째 시도')
                        #         node_CtlCenter_globals.listBLB.clear()
                        #         node_CtlCenter_globals.listBLB.extend(lsDF)                                
                        #         return APIBLB_ACTION_REPLY.E102
                        #     else:
                        #         node_CtlCenter_globals.listBLB.clear()
                        #         node_CtlCenter_globals.aruco_try = 0
                        #         # dicInfo_local[i][MotorWMOVEParams.POS.name] = 100000
                        #         # TTSAndroid('위치파악실패')
                        # else:
                            #SetWaitConfirmFlag(True,AlarmCodeList.WAITING_USER)
            #elif iMBID == ModbusID.ROTATE_SERVE_360.value and CheckMotorOrderValid(dicArray) and GetTiltStatus() == TRAY_TILT_STATUS.TiltTrayCenter:
            elif iMBID == ModbusID.ROTATE_SERVE_360.value:
                if not dicAruco:
                    TiltTableObstacleScan()
                if CheckMotorOrderValid(dicArray):
                    #트레이 모터를 움직일때 현재 서빙부의 길이를 구한 후 200mm 이상이지 않으면 알람.
                    # if curDistanceSrvTele < 200 and isRealMachine:
                    #     rospy.loginfo(dicArray)
                    #     StopEmergency(ALM_User.TRAY360_SAFETY.value)
                    #     return APIBLB_ACTION_REPLY.E102
                    #전개후 아르코 마커가 인식되었을때 세부조절과 각도 튜닝 들어감.
                    tray_angle = pulse_to_angle_sse(iPOS)
                    if abs(tray_angle) > 0:
                        if dicAruco:
                            marker_value = dicAruco.get(ARUCO_RESULT_FIELD.MARKER_VALUE.name)
                            marker_angle = round(dicAruco.get(ARUCO_RESULT_FIELD.ANGLE.name))
                            dicTagretTableInfo = getTableServingInfo(curTargetTable)
                            target_marker = dicTagretTableInfo.get(TableInfo.MARKER_VALUE.name,-1)
                            if True:
                            #if is_equal(marker_value,target_marker):
                                angle_new = (180+ marker_angle)%360
                                #GetNewRotateArmList(dicAruco)
                                # #TODO : 아르코마커가 인식되었습니다TTS. + 음성 메세지 클래스 정의할 것.
                                #TTSAndroid(TTSMessage.ARUCO_FOUND_OK.value)
                                resultDiff , diff_X,diff_Y= compare_dicts(dicAruco, ref_dict, CAM_LOCATION_MARGIN_OK)
                                #dicAruco = lsAruco[0]
                                rospy.loginfo(json.dumps(dicAruco, indent=4))
                                dicNewTray = GetDicRotateMotorTray(angle_new)
                                dicInfo_local[i].update(dicNewTray)
                                infoMsg = f'이전 각도:{tray_angle},마커로 바뀐각도:{angle_new},마커정보:{json.dumps(dicAruco, indent=4)}'
                                SendInfoHTTP(f'이전 각도:{tray_angle},마커로 바뀐각도:{angle_new}')
                                dic_newNodeInfo2 = {}
                                dic_newNodeInfo2[TableInfo.TABLE_ID.name] = curTargetTable
                                dic_newNodeInfo2[TableInfo.NODE_ID.name] = curNode
                                dic_newNodeInfo2[TableInfo.SERVING_DISTANCE.name] = curDistanceSrvTele
                                dic_newNodeInfo2[TableInfo.SERVING_ANGLE.name] = curAngle_540
                                dic_newNodeInfo2[TableInfo.MARKER_ANGLE.name] = angle_new
                                dic_newNodeInfo2[TableInfo.HEIGHT_LIFT.name] = 880000   #나중에 라이다 하강거리로 대체
                                dic_newNodeInfo2[TableInfo.MARKER_VALUE.name] = curTargetTable
                                
                                add_or_update_row(strFileTableNodeEx,dic_newNodeInfo2, sDivTab,TableInfo.TABLE_ID.name)
                                
                                #df.to_csv(strFileTableNodeEx, index=False, sep=sDivTab)
                                df = pd.read_csv(strFileTableNodeEx, dtype={TableInfo.NODE_ID.name: int}, sep=sDivTab)
                                filtered = df[(df[TableInfo.NODE_ID.name] == curNode) & (df[TableInfo.MARKER_VALUE.name] < 0)]
                                lsDictNotScaned = filtered.to_dict(orient='records')
                                if len(lsDictNotScaned)>0:
                                    dicNext = lsDictNotScaned[0]
                                    nextTable = dicNext[TableInfo.TABLE_ID.name]
                                    SetWaitConfirmFlag(True,AlarmCodeList.WAITING_USER)
                                    InsertTableList(nextTable)
                                    TTSAndroid(f'{curTargetTable}번 테이블정보 저장 후 {nextTable}번 테이블 스캔을 시작합니다.')
                                else:
                                    TTSAndroid(f'{curTargetTable}번 테이블 위치정보 및 현재 노드 스캔 완료')    
                                rospy.loginfo(infoMsg)
                            else:
                                infoMsg = f'마커값이 다릅니다 : {marker_value,target_marker}'
                                rospy.loginfo(infoMsg)
                        else:
                            infoMsg = '아르코마커가 인식되지 않았습니다.'
                            rospy.loginfo(infoMsg)
        lsFinalCmd = []
        lsFinalCmdEx = []        
        node_CtlCenter_globals.lock.acquire()
        node_CtlCenter_globals.dicTargetPos.clear()
        node_CtlCenter_globals.dicTargetPos.update(create_mbid_dict(dicInfo_local, MotorWMOVEParams.POS.name))
        rospy.loginfo(f'dicTargetPos:{node_CtlCenter_globals.dicTargetPos}')
        UpdateLastBalanceTimeStamp()
        dfReceivedNew = pd.DataFrame(dicInfo_local)
        rospy.loginfo(dfReceivedNew)
        node_CtlCenter_globals.status_bal = STATUS_BALANCING.READY                 
        node_CtlCenter_globals.lock.release()
        dfBackUp = None
        bResult = True
        bStrMsg = AlarmCodeList.OK.name
        if isLiftControl:
            bResult, bStrMsg = API_SendCMD_Device(dicInfo_local)
            #dfBackUp = listBLB.pop(0)
            bResult, bStrMsg = GetResultMessageFromJsonStr(bStrMsg)
            sMsg = f'리프트컨트롤:{bResult},{bStrMsg}'
            rospy.loginfo(sMsg)
        elif isArmControl:
            if dfReceivedNew[MotorWMOVEParams.MBID.name].astype(str).isin([str(ModbusID.ROTATE_SERVE_360.value)]).any():
                sPosTray = get_last_value_for_key(dfReceivedNew, MotorWMOVEParams.MBID.name, str(ModbusID.ROTATE_SERVE_360.value),MotorWMOVEParams.POS.name)
                sPosAngle = pulse_to_angle_sse(int(sPosTray),pot_31)
                bResult, bStrMsg = API_MoveArms(distance_target, adjustrate,sPosAngle)
            else:
                bResult, bStrMsg = API_MoveArms(distance_target, adjustrate)
            #dfBackUp = listBLB.pop(0)
            bResult, bStrMsg = GetResultMessageFromJsonStr(bStrMsg)
            sMsg = f'암컨트롤:{bResult},{bStrMsg}'
            rospy.loginfo(sMsg)
        elif isRotateMainControl:
            bResult, bStrMsg = API_MoveMainRotate(distance_target, adjustrate)
            #dfBackUp = listBLB.pop(0)
            bResult, bStrMsg = GetResultMessageFromJsonStr(bStrMsg)
            sMsg = f'메인회전컨트롤:{bResult},{bStrMsg}'
            rospy.loginfo(sMsg)
        elif isRotateTrayControl:
            bResult, bStrMsg = API_MoveMainTray(distance_target, adjustrate)
            #dfBackUp = listBLB.pop(0)
            bResult, bStrMsg = GetResultMessageFromJsonStr(bStrMsg)
            sMsg = f'트레이회전컨트롤:{bResult},{bStrMsg}'
            rospy.loginfo(sMsg)
        elif len(listBLB) > 0:
            dfBackUp = listBLB.pop(0)
            lsFinalCmd.extend(dfBackUp)
        else:
            sMsg = 'listBLB 스레드 점검필요'
            rospy.loginfo(sMsg)
            bResult = False
            SendAlarmHTTP(sMsg,True,node_CtlCenter_globals.BLB_ANDROID_IP) 
        if len(lsFinalCmd) ==0 and not isTrue(bResult):
            if bStrMsg is None or isExceptionSSE(bStrMsg):
                #SendAlarmHTTP(ALM_User.SSE_CONNETION_ERROR.value,True)    
                StopEmergency(ALM_User.SSE_CONNETION_ERROR.value, True,False)
                return APIBLB_ACTION_REPLY.E500
            else:
                #SetWaitConfirmFlag(True,AlarmCodeList.WAITING_USER)
                DoorClose()
                lsLiftUp = GetLiftControl(True)
                lsBLBTmp = copy.deepcopy(listBLB)
                node_CtlCenter_globals.listBLB[:] = lsLiftUp + lsBLBTmp            
                return APIBLB_ACTION_REPLY.E104
        if len(lsFinalCmd) ==0 and isTrue(bResult):
            dfBackUp = listBLB.pop(0)
            rospy.loginfo(dfBackUp)
        # #isArmControl = False
        # for dicCtlTmp in dicInfo_local:
        #     sPos = dicCtlTmp.get(MotorWMOVEParams.POS.name, MIN_INT)
        #     mbid = dicCtlTmp.get(MotorWMOVEParams.MBID.name, None)
        #     target_pulse = int(sPos)
        #     if isArmControl or isRotateMainControl or isRotateTrayControl:
        #         continue
            
        #     if mbid == str(ModbusID.ROTATE_MAIN_540.value):
        #         targetCW = target_pulse +potRotate540
        #         targetCCW = target_pulse -potRotate540
        #         arr =[target_pulse,targetCW,targetCCW]
        #         iPOSBack = min(arr, key=lambda x: abs(x - poscurRotate540))
        #         iPOSList = sorted(arr, key=lambda x: abs(x - poscurRotate540))[:2]
        #         diff_1 = abs(iPOSList[0] - poscurRotate540)
        #         diff_2 = abs(iPOSList[1] - poscurRotate540)
        #         if abs(diff_1 - diff_2) < roundPulse:
        #             iPOSBack = min(iPOSList, key=abs)                
        #         strPos = ','.join(map(str, arr))
                
        #         SendInfoHTTP(f'MBID:{mbid},현재펄스:{poscurRotate540},최단회전 펄스 선택:{strPos} = ({iPOSBack})')
        #         dicCtlTmp[MotorWMOVEParams.POS.name] = iPOSBack            
            
        #     if mbid == str(ModbusID.ROTATE_SERVE_360.value):
        #         potRotate360, notRotate360, poscmdRotate360,poscurRotate360= GetPotNotCurPosServo(ModbusID.ROTATE_SERVE_360)
        #         target_pulse = int(sPos)
        #         targetCW = target_pulse +potRotate360
        #         targetCCW = target_pulse -potRotate360
        #         arr =[target_pulse,targetCW,targetCCW]
        #         iPOSBack = min(arr, key=lambda x: abs(x - poscurRotate360))
        #         strPos = ','.join(map(str, arr))
        #         SendInfoHTTP(f'MBID:{mbid},현재펄스:{poscurRotate360},최단회전 펄스 선택:{strPos} = ({iPOSBack})')
        #         dicCtlTmp[MotorWMOVEParams.POS.name] = iPOSBack
            
        #     # if mbid == str(ModbusID.TELE_SERV_MAIN.value):
        #     #     if int(sPos) > 0 and len(dicInfo_local) > 1 and cur_pos_srv < -roundPulse:
        #     #         rpmSrv = round((DEFAULT_RPM_SLOW/2)) if onScan else DEFAULT_RPM_SLOW
        #     #         dicMoveTeleSrv = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value,True,0, rpmSrv,ACC_ST,DECC_ST)                    
        #     #         node_CtlCenter_globals.dicTargetPos.clear()
        #     #         if iPOS > cur_pos_srv:
        #     #         #if onScan and iPOS > cur_pos_srv:
        #     #             TiltArucoScan()
        #     #             CamControl(True)
                        
        #     #         SendCMD_Device([dicMoveTeleSrv])
        #     #         node_CtlCenter_globals.listBLB.insert(0,dicInfo_local_org)
        #     #         return APIBLB_ACTION_REPLY.E108
        #     #     elif int(sPos) == 0 and (cur_pos_srv-not_telesrv) > roundPulse and cur_pos_srv < roundPulse and len(dicInfo_local) > 1:
        #     #         dicMoveTeleSrv = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value, True, not_telesrv,DEFAULT_RPM_SLOW , ACC_ST,DECC_ST)
        #     #         SendCMD_Device([dicMoveTeleSrv])
        #     #         return APIBLB_ACTION_REPLY.E108

        #     # elif mbid == str(ModbusID.TELE_SERV_MAIN.value) and sPos == '0' and len(dicInfo_local) > 1:
        #     #     #isArmControl = True
        #     #     lsFinalCmd.clear()
        #     #     lsFinalCmd= GetStrArmExtendMain(0,0,True)
        #     #     break

        #     # if len(lsAruco) > 0 and onScan:
        #     #     dicAruco = lsAruco[0]
        #     #     lsDF = GetNewRotateArmList(dicAruco)
        #     #     if lsDF:
        #     #         # #TODO : 아르코마커가 인식되었습니다TTS. + 음성 메세지 클래스 정의할 것.
        #     #         TTSAndroid(TTSMessage.ARUCO_FOUND_OK.value)
        #     #         resultDiff,diff_X,diff_Y= compare_dicts(dicAruco, ref_dict, CAM_LOCATION_MARGIN_OK)
        #     #         node_CtlCenter_globals.listBLB.clear()
        #     #         node_CtlCenter_globals.listBLB.extend(lsDF)
        #     #         return APIBLB_ACTION_REPLY.E108
            
        #     lsFinalCmd.append(dicCtlTmp)
            
        for dicCtlTmp2 in lsFinalCmd:
            #mbid = dicCtlTmp2[MotorWMOVEParams.MBID.name]
            mbid = dicCtlTmp2.get(MotorWMOVEParams.MBID.name)
            sPOS = dicCtlTmp2.get(MotorWMOVEParams.POS.name)
            if mbid is None:
                print(dicCtlTmp2)
                continue
            if sPOS is None:
                print(dicArray)
                continue
            
            mbidInstance = ModbusID.from_value(mbid)
            target_pulse = int(dicCtlTmp2[MotorWMOVEParams.POS.name])
            rotateRPM = int(dicCtlTmp2[MotorWMOVEParams.SPD.name])
            #dicCtlTmp2[MotorWMOVEParams.TIME.name] = GetTimeFromRPM(mbidInstance, target_pulse, rotateRPM)            
            if CheckMotorOrderValid(dicCtlTmp2):                
                dicCtlTmp2[MotorWMOVEParams.TIME.name] = GetRPMFromTimeAccDecc(dicCtlTmp2)
                # if mbid == str(ModbusID.TELE_SERV_MAIN.value):
                #     if (onScan or finalScan) and iPOS > cur_pos_srv:
                #     #if target_pulse != 0 and :
                #         #모니터 감지각으로 틸팅 변경
                #         #TiltDetectingMonitor()
                #         TiltArucoScan()
                #         CamControl(True)
                    # if dicInfo_local_org is not None and target_pulse == 0 and abs(node_CtlCenter_globals.SERVING_ARM_BALANCE_PULSE - cur_pos11) > roundPulse and node_CtlCenter_globals.SERVING_ARM_BALANCE_PULSE < cur_pos11:
                    #     dicMoveH = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value,True,node_CtlCenter_globals.SERVING_ARM_BALANCE_PULSE, DEFAULT_RPM_SLOW,ACC_DECC_SMOOTH,ACC_DECC_SMOOTH)
                    #     node_CtlCenter_globals.dicTargetPos.clear()
                    #     SendCMD_Device([dicMoveH])
                    #     node_CtlCenter_globals.listBLB.insert(0,dicInfo_local_org)
                    #     return APIBLB_ACTION_REPLY.E108

                    # if dicInfo_local_org is not None and target_pulse > node_CtlCenter_globals.SERVING_ARM_EXPAND_PULSE and abs(node_CtlCenter_globals.SERVING_ARM_EXPAND_PULSE - cur_pos11) > roundPulse:
                    #     dicTeleExpand = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value,True,node_CtlCenter_globals.SERVING_ARM_EXPAND_PULSE, DEFAULT_RPM_SLOW,ACC_DECC_SMOOTH,ACC_DECC_SMOOTH)
                    #     node_CtlCenter_globals.dicTargetPos.clear()
                    #     SendCMD_Device([dicTeleExpand])
                    #     node_CtlCenter_globals.listBLB.insert(0,dicInfo_local_org)
                    #     return APIBLB_ACTION_REPLY.E108

                #최종 필터링 이후 추가해야할 동작 확인
                if mbid == str(ModbusID.MOTOR_V.value):                    
                    if target_pulse ==0:    #상승시
                        TiltServFinish()
                    else:#target_pulse != 0:   #하강시
                        #모니터 감지각으로 틸팅 변경
                        CamControl(False)
                        TiltTableObstacleScan()
                        if int(curTargetNode) == node_KITCHEN_STATION:
                            SetWaitConfirmFlag(False,AlarmCodeList.JOB_COMPLETED)
                            node_CtlCenter_globals.dicPOS_ABS.clear()
                        
                node_CtlCenter_globals.dicTargetPos[mbid] = dicCtlTmp2[MotorWMOVEParams.POS.name]                        
                lsFinalCmdEx.append(dicCtlTmp2)
            else:
                rospy.loginfo(f"Skipped motor : {dicCtlTmp2}")

        if len(lsFinalCmdEx) > 0:
            dfPrev = pd.DataFrame(lsFinalCmdEx)
            PrintDF(dfPrev)
            manager = MotorCommandManager(lsFinalCmdEx)
            manager.remove_cmd(MotorWMOVEParams.SPD.name, str(DEFAULT_RPM_MIN))
            lsFinalCmdEx.clear()
            lsFinalCmdEx = manager.get_all_commands()
            dfLog = pd.DataFrame(lsFinalCmdEx)
            rospy.loginfo(f"Moving {len(lsFinalCmdEx)} motors,curTable:{curTargetTable},{dfLog}")
            lsNextCmd = []
            # print(node_CtlCenter_globals.listBLB)
            # lsFinalMbid = ""
            # nextCmdMbid = ""
            #메인회전 모터 필터링 - 다음 명령어도 메인회전이거나 다음 명령어가 H 인 경우 스킵한다.
            # if len(node_CtlCenter_globals.listBLB) > 0:
            #     lsDF = []
            #     lsNextCmd = node_CtlCenter_globals.listBLB[0]
            #     isNextDict = isinstance(lsNextCmd, dict)
            #     if isNextDict:
            #         lsDF.append(lsNextCmd)
            #     else:
            #         lsDF.extend(lsNextCmd)
            #     dfNext = pd.DataFrame(lsDF)
            #     dfNext.dropna(how='all', inplace=True)
            #     rospy.loginfo(dfNext)
            #     contains_nan = dfNext.isnull().values.any()
            #     if contains_nan:
            #       rospy.loginfo(dfNext)
            #     if len(lsFinalCmdEx) == 1:
            #         lsCurCmd = lsFinalCmdEx[0]
            #         curMBID = lsCurCmd.get(MotorWMOVEParams.MBID.name, "")
            #         if curMBID == str(ModbusID.ROTATE_MAIN_540.value):
            #             if len(lsNextCmd) == 1:
            #                 nextMBID = lsNextCmd[0].get(MotorWMOVEParams.MBID.name, "")
            #                 if curMBID == nextMBID:
            #                     rospy.loginfo('Skipped by dup 540 cmd')
            #                     return APIBLB_ACTION_REPLY.E105
            #df = pd.DataFrame(lsFinalCmdEx)
            #SendCMD_Device(lsFinalCmdEx)
            # marginTime = df[df['MBID'] == '9']['TIME'].values[0]
            # row_dict = df[df['MBID'].astype(str) == '9'].to_dict(orient='records')[0]
            # #GetRPMFromTimeAccDecc(row_dict)
            # #marginTime =  node_CtlCenter_globals.dicTargetPos.get(ModbusID.TELE_BALANCE.from_value) 
            # if marginTime is None and node_CtlCenter_globals.dicTargetPos.get(ModbusID.TELE_SERV_MAIN.from_value) is not None:                
            #     df_updated = update_motor_spd_by_time(df, target_mbid='11', ref_mbid='13', cur_pos=cur_pos11, marginTime=marginTime)
            #     lsFinalDF = df_updated.to_dict(orient="records")
            #     SendCMD_Device(lsFinalDF)
            # else:
            #     SendCMD_Device(lsFinalCmdEx)
            df_final = pd.DataFrame(lsFinalCmdEx)
            unique_mbid = df_final[MotorWMOVEParams.MBID.name].unique().tolist()
            if len(unique_mbid) == 1:
                mbid_current = unique_mbid[0]
                dicCurrent = lsFinalCmdEx[0]
                iPos = int(dicCurrent[MotorWMOVEParams.POS.name])
                iSpd = int(dicCurrent[MotorWMOVEParams.SPD.name])
                if is_equal(mbid_current, ModbusID.ROTATE_MAIN_540.value):
                    if iSpd == MAINROTATE_RPM_SLOWEST:
                        bResult, bStrMsg = SendCMD_Device(lsFinalCmdEx)
                    else:
                        cur_angle_540 = pulse_to_angle(iPos, potRotate540_cmd, MAX_ANGLE_TRAY)%360
                        bResult, bStrMsg = API_MoveMainRotate(cur_angle_540, adjustrate)
                    sMsg = f'메인회전컨트롤:{bResult},{bStrMsg}'
                    rospy.loginfo(sMsg)
                elif is_equal(mbid_current, ModbusID.ROTATE_SERVE_360.value):
                    cur_angle_360 = pulse_to_angle(iPos, pot_31, MAX_ANGLE_TRAY)%360
                    bResult, bStrMsg = API_MoveMainTray(cur_angle_360, adjustrate)
                    sMsg = f'트레이회전컨트롤:{bResult},{bStrMsg}'
                    rospy.loginfo(sMsg)
                else:
                    SendCMD_Device(lsFinalCmdEx)
            else:
                SendCMD_Device(lsFinalCmdEx)                    
        UpdateLastCmdTimeStamp()
        UpdateLastBalanceTimeStamp()
        # if len(listBLB) == 0 and not isScanMode():
        #     SetWaitConfirmFlag(True,AlarmCodeList.WAITING_USER)
        # else:
        #   print(listBLB)
    
    return APIBLB_ACTION_REPLY.R101

print(os.path.splitext(os.path.basename(__file__))[0],getDateTime())