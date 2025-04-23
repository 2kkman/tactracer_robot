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
    #curTargetTable,curTarNode = GetTargetTableNode()
    df = GetDF(curTargetTable)    
      
    dicTagretTableInfoCurrent = getTableServingInfo(curTargetTable)
    target540 = dicTagretTableInfoCurrent.get(TableInfo.SERVING_ANGLE.name)
    targetH = dicTagretTableInfoCurrent.get(TableInfo.MOVE_DISTANCE.name)
    isTeaching540 = True if target540 == StateBranchValue.ERROR.value and targetH != StateBranchValue.ERROR.value else False
    targetServingDistance = dicTagretTableInfoCurrent.get(TableInfo.SERVING_DISTANCE.name, StateBranchValue.ERROR.value)
    isTeachingServingDistance = True if targetServingDistance == StateBranchValue.ERROR.value else False
    pot_telesrv,not_telesrv,cmdpos_srv,cur_pos_srv =GetPotNotCurPosServo(ModbusID.TELE_SERV_MAIN)
    pot_6,not_6,cmdpos_6,cur_pos_6 =GetPotNotCurPosServo(ModbusID.MOTOR_V)
    pot_13,not_13,cmdpos_13,cur_pos_13 =GetPotNotCurPosServo(ModbusID.BAL_ARM1)
    dicInfo_local_org = copy.deepcopy(dicInfo_local)
    #get_tasmota_info
    #현재 뻗은 암 길이 / 메인회전각도 / 트레이각도
    cmd_posH, cur_posH = GetPosServo(ModbusID.MOTOR_H)
    DI_POT,DI_NOT,DI_HOME,SI_POT = GetPotNotHomeStatus(ModbusID.MOTOR_H)  
    cur_pos_mm = pulseH_to_distance(cur_posH)
    curDistanceSrvTele, curAngle_540,cur_angle_360  = GetCurrentPosDistanceAngle()
    #dicInfo_local = listBLB[0]
    stateCharger = isChargerPlugOn()
    onScan = isScanTableMode(curTargetTable)
    dicAruco = {}
    lsAruco = filter_recent_data(ARUCO_RESULT_FIELD.LASTSEEN.name,GetArucoMarkerInfo(),0.2)
    if len(lsAruco) > 0 and onScan:
        dicAruco = lsAruco[0]
        lsDF = GetNewRotateArmList(dicAruco)
    
    if isinstance(dicInfo_local, dict):  # 주행모드
        bIsAllMotorFolded = isReadyToMoveH_and_540()
        start_node = dicInfo_local.get(SeqMapField.START_NODE.name)
        start_node_str = str(start_node)
        end_node = dicInfo_local.get(SeqMapField.END_NODE.name)
        if end_node == GetCurrentNode() or end_node == start_node:
            listBLB.pop(0)
            UpdateLastCmdTimeStamp()
            UpdateLastBalanceTimeStamp()                
            return APIBLB_ACTION_REPLY.E108

        if not bIsAllMotorFolded:
            if abs(cur_pos_13) < roundPulse and abs(cur_pos_6) < roundPulse :
                dicMoveTeleSrv = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value, True, not_telesrv,DEFAULT_RPM_SLOW ,  ACC_ST, DECC_ST)
                node_CtlCenter_globals.dicTargetPos.clear()
                SendCMD_Device([dicMoveTeleSrv])
                #node_CtlCenter_globals.listBLB.insert(0,dicInfo_local_org)
                return APIBLB_ACTION_REPLY.E108
            else:
                lsLiftUp = GetLiftControl(True)
                lsBLBTmp = copy.deepcopy(listBLB)
                listBLB[:] = lsLiftUp + lsBLBTmp
            # listBLB.clear()
            # listBLB.extend(lsLiftUp + lsBLBTmp)
            return APIBLB_ACTION_REPLY.E106
        else:
            # #연속으로 주행해야 할때는 한개의 커맨드로 합친다.
            # if len(node_CtlCenter_globals.listBLB) > 1 and isinstance(node_CtlCenter_globals.listBLB[1], dict):
            #     sendbuf = getListedDic(node_CtlCenter_globals.listBLB[1])
            rospy.loginfo(dicInfo_local)
            sPOS = dicInfo_local.get(MotorWMOVEParams.POS.name, MIN_INT)
            iTargetPulse = int(sPOS)
            
            # 출발지점 혹은 도착지점이 충전소인 경우 움직이기 전 충전기를 OFF
            if stateCharger and (is_equal(end_node, node_KITCHEN_STATION) or is_equal(start_node, node_KITCHEN_STATION)):
                SetChargerPlug(False)
                
            # 서버에서 온 시작지점과 현재 위치값이 맞지 않은 경우 히스토리에 기록된 시작지점의 pulse 값으로 로봇 이동 후 출발
            # RFID 활성화 되면 필요 없음
            # if start_node_str in node_CtlCenter_globals.dicPOS_ABS:
            #     start_node_pos = node_CtlCenter_globals.dicPOS_ABS[start_node_str]
            #     if dicInfo_local_org is not None and abs(cur_posH-start_node_pos) > roundPulse:
            #         dicMoveH = getMotorMoveDic(ModbusID.MOTOR_H.value,True,start_node_pos, SPD_MOVE_H,ACC_MOVE_H,DECC_MOVE_H)
            #         node_CtlCenter_globals.dicTargetPos.clear()
            #         SendCMD_Device([dicMoveH])
            #         #node_CtlCenter_globals.listBLB.insert(0,dicInfo_local_org)
            #         return APIBLB_ACTION_REPLY.E108

            #분기기가 목적지인 경우 감속구간을 늘려서 진입한다
            #RFID 태그와는 별개로 동작한다.
            deccH = DECC_MOVE_H
            if str(end_node) in node_CtlCenter_globals.stateDic:
                deccH =5000
            
            if sPOS == MIN_INT: #상대길이 경로
                donCare = -1
                #서버에서 받아온 정보가 있는 경우
                if dicInfo_local.get(APIBLB_FIELDS_TASK.detailcode_list.name) is None:
                  rospy.loginfo(dicInfo_local)
                else:
                  # end_node = dicInfo_local[APIBLB_FIELDS_TASK.endnode.name]
                  # start_node = dicInfo_local.get(APIBLB_FIELDS_TASK.startnode.name)                  
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
                sPOS_R = dicInfo_local.get(APIBLB_FIELDS_TASK.distance_total.name, MIN_INT)
                sDir = dicInfo_local.get(SeqMapField.DIRECTION.name)
                sPOS_R = dicInfo_local[SeqMapField.DISTANCE.name] 
                # if sPOS_R == MIN_INT:
                # sDir = dicInfo_local.get(APIBLB_FIELDS_TASK.direction.name)
                # if not IsEnableSvrPath():
                #   start_node = dicInfo_local[SeqMapField.START_NODE.name]
                #   end_node = dicInfo_local[SeqMapField.END_NODE.name]
                iPOS_R = try_parse_int(sPOS_R)
                if iPOS_R == 0:
                  rospy.loginfo(dicInfo_local)
                #iPOS_R = int(sPOS_R)
                rospy.loginfo('CheckPoint')
                # if isScanMode():
                #   #탐색 모드, 진행 길이는 100미터로 설정. 수기로 정지신호가 들어오거나 RFID가 인식되면 멈춤.
                #   #RFID 태그가 인식되지 않으면 데드엔드에서 위험하므로 RFID 가 정상인지 확인 필요.
                #   #카메라 돌리고 틸팅하는 명령어도 여기서 구현
                #   if abs(iPOS_R) < 10:
                #     isTeaching540 = False
                #     newDistance = getLinkDistance(start_node,end_node)
                #     if newDistance > 1:
                #         iPOS_R = newDistance
                #         iSpdH = SPD_MOVE_H * SPEED_RATE_H
                #     else:
                #         iPOS_R = iPOS_R * 100000
                #         iSpdH = DEFAULT_RPM_SLOW
                #   # else:
                #   #   #스캔모드가 아닌 경우 현재 좌표값을 확인해서 이동거리를 정한다.
                #   #   targetX,targetY = GetLocNodeID(end_node)
                #   #   if sDir == 'N':
                #   #     iPOS_R = targetY - curY
                #   #   if sDir == 'S':
                #   #     iPOS_R = targetY - curY
                # #   startnode_cur = cur_posH
                # #   endnode_cur = cur_posH+distance_to_pulseH(iPOS_R)
                # #   lk = GetLinkKey(start_node,end_node)
                # #   dicLinkInfo = {TableInfo.LINK_ID.name:lk,
                # #   SeqMapField.START_NODE.name:startnode_cur,
                # #   SeqMapField.END_NODE.name:endnode_cur}
                # #   if node_CtlCenter_globals.dfLinkPosInfo.empty or lk not in node_CtlCenter_globals.dfLinkPosInfo[TableInfo.LINK_ID.name].values:
                # #     node_CtlCenter_globals.dfLinkPosInfo=pd.concat([node_CtlCenter_globals.dfLinkPosInfo, pd.DataFrame([dicLinkInfo])], ignore_index=True)
                # #targetX,targetY = GetLocNodeID(end_node)
                # #iPOS_R = distance_to_pulseH(targetX)
                if sDir is None:
                  iPOS_R = (distance_to_pulseH(iPOS_R))
                elif is_equal(sPOS,MIN_INT):
                  iPOS_R = abs(distance_to_pulseH(iPOS_R))
                  if is_equal(sDir,'S') or is_equal(sDir ,'W'):
                      iPOS_R = -iPOS_R
                else:
                  iPOS_R = distance_to_pulseH(iPOS_R)
                rospy.loginfo('CheckPoint')
                #iTargetPulse = iPOS_R
                iTargetPulse = iPOS_R + cur_posH
                # if iTargetPulse < -roundPulse or iTargetPulse > 3260000:
                #   #print(iTargetPulse)
                #   iTargetPulse = 3260000
                cs = dicInfo_local.get(SeqMapField.CROSS_STATUS.name)
                #cs = node_CtlCenter_globals.stateDic
                bCrossCheck = True
                strStatusInfo = ''
                rospy.loginfo('CheckPoint')
                if cs is not None and isinstance(cs, dict):
                  rospy.loginfo(cs)
                  for cross_node,cross_status in cs.items():
                    # if int(cross_node) != 10:
                    #     continue
                    # if is_equal(cross_status,0):
                    #     cross_status = 1
                    # else:
                    #     cross_status = 0
                    # #cross_status = 1 if cross_status == 0 else 0
                    setNodeStateEx(cross_node, cross_status)
                    iCurStartState = getNodeState(cross_node)
                    strStatusInfo += f'{cross_node}:{cross_status}->{iCurStartState},'
                    if cross_status != iCurStartState:
                      bCrossCheck = False
                #if False:   #임시로 분기기는 무조건 True 라고 가정하자.
                if bCrossCheck == False and isRealMachine:
                #rospy.loginfo('CheckPoint')
                #if bCrossCheck == False:
                    SetWaitCrossFlag(True)
                    UpdateLastCmdTimeStamp()
                    message=f'남은테이블:{node_CtlCenter_globals.listTable},분기기 상황:{strStatusInfo}'
                    PrintStatusInfoEverySec()
                    rospy.loginfo_throttle(20, message)       
                    return APIBLB_ACTION_REPLY.E107
                #dicNodeMove = getMotorMoveDic(ModbusID.MOTOR_H.value, True, infoMOVE_DISTANCE, DEFAULT_SPD_FAST, ACC_MOVE_H, DECC_MOVE_H)
                #데모에서는 일단 이렇게
                dicInfo_local = getMotorMoveDic(ModbusID.MOTOR_H.value, True, iTargetPulse, iSpdH, ACC_MOVE_H,deccH)
                SetWaitCrossFlag(False)
                #print(dicInfo_local)

            #rospy.loginfo('CheckPoint')
            node_CtlCenter_globals.dicTargetPos[str(ModbusID.MOTOR_H.value)] = iTargetPulse
            node_CtlCenter_globals.lastSendDic_H = dicInfo_local
            dicRotateDirection = getMainRotateDicByDirection(dicInfo_local)
            rospy.loginfo('CheckPoint')
            if CheckMotorOrderValid(dicInfo_local):
                rospy.loginfo('CheckPoint')
                if CheckMotorOrderValid(dicRotateDirection):
                    rospy.loginfo('CheckPoint')
                    listBLB.insert(0,[dicRotateDirection])
                    return APIBLB_ACTION_REPLY.E108                
                # if CheckMotorOrderValid(dicServExpand):
                #     listBLB.insert(0,[dicServExpand])
                #     return APIBLB_ACTION_REPLY.E108                
                rospy.loginfo('CheckPoint')
                TiltFace()
                if dicInfo_local_org.get(SeqMapField.END_NODE.name) is not None:
                  node_CtlCenter_globals.lsHistory_motorH.append(dicInfo_local_org)
                #print(node_CtlCenter_globals.listTable)
                rospy.loginfo('CheckPoint')
                if False:
                #if isScanMode():
                    isScanOK = isScanCompleted()
                    if isScanOK:# and curNode == node__STATION:
                      #TODO : 이제까지 나온 결과를 저장한다. CROSS,SHORTCUT, TABLENODE_EX 를 한번에 업데이트 한다.
                      #DF 를 로드한다.(없으면 빈 DF 생성)
                        with open(strJsonGraphInfo, "w") as file:
                            json.dump(node_CtlCenter_globals.graph, file)    

                        node_CtlCenter_globals.dfLinkPosInfo.to_csv(node_CtlCenter_globals.strJsonLinkPosInfo, index=False, sep=sDivTab) 
                        # with open(strJsonLinkPosInfo, "w") as file:
                        #     json.dump(node_CtlCenter_globals.dfLinkPosInfo, file)    

                        with open(strJsonScanInfo, "w") as file:
                            json.dump(node_CtlCenter_globals.ScanInfo, file)                            
                        #SaveTableNodeInfo()
                    else:
                      rospy.loginfo('CheckPoint')
                      SendCMD_Device([dicInfo_local])
                else:
                    rospy.loginfo(dicInfo_local)
                    endnode_current = dicInfo_local_org.get(SeqMapField.END_NODE.name)
                    pulseTarget= GetNodePos_fromNode_ID(endnode_current)
                    #bReturn,strResult=MoveH_MotorRFID(dicInfo_local.get(MotorWMOVEParams.POS.name),dicInfo_local.get(MotorWMOVEParams.SPD.name),dicInfo_local_org.get(SeqMapField.END_NODE.name))
                    rospy.loginfo(f'현재펄스:{cur_posH}, 타겟펄스:{pulseTarget}')
                    if isRealMachine:
                        bReturn,strResult=MoveH_MotorRFID(pulseTarget,dicInfo_local.get(MotorWMOVEParams.SPD.name),endnode_current)
                        rospy.loginfo(f'{bReturn},{strResult}')
                    else:
                        SendCMD_Device([dicInfo_local])                            
                    # rospy.loginfo('CheckPoint')
                    # RFIDControl(True)
                rospy.loginfo(f"Moving H motor : {node_CtlCenter_globals.nStart}->{node_CtlCenter_globals.nTarget}({listBLB.pop(0)})")
            else:
                rospy.loginfo(f"Skipped H motor : {node_CtlCenter_globals.nStart}->{node_CtlCenter_globals.nTarget}({listBLB.pop(0)})")

            ClearArucoTable()
            UpdateLastCmdTimeStamp()
            PrintStatusInfoEverySec(1)
            return APIBLB_ACTION_REPLY.R101
    elif isinstance(dicInfo_local, list):
        #cmd_pos11,cur_pos11=GetPosServo(ModbusID.TELE_SERV_MAIN)
        if GetRFIDInventoryStatus():
            rospy.loginfo(f"RFID is on, suspend motor action : {dicInfo_local}")
            return APIBLB_ACTION_REPLY.E104
                
        
        filtered_data = [item for item in dicInfo_local if item]
        if len(filtered_data) > 0:
            #print(dicInfo_local)
            dfReceived = pd.DataFrame(filtered_data) 
            #정수형 데이터는 int 로 변환한다
            dfReceived[MotorWMOVEParams.POS.name] = dfReceived[MotorWMOVEParams.POS.name].astype(int)
            dfReceived[MotorWMOVEParams.MBID.name] = dfReceived[MotorWMOVEParams.MBID.name].astype(int)
            lsLiftDown= dfReceived[(dfReceived[MotorWMOVEParams.POS.name] > 0) & (dfReceived[MotorWMOVEParams.MBID.name] == ModbusID.MOTOR_V.value)].tail(1).to_dict(orient='records')
            lsExpandArm= dfReceived[(dfReceived[MotorWMOVEParams.POS.name] > 0) & (dfReceived[MotorWMOVEParams.MBID.name] == ModbusID.TELE_SERV_MAIN.value)].tail(1).to_dict(orient='records')
            if len(lsLiftDown) > 0 or len(lsExpandArm) > 0:
                if GetWaitConfirmFlag():
                    return
        
        # 루프를 돌면서 원소를 직접 수정할 수 있게 한다.
        valuesInlist = len(dicInfo_local)
        for i in range(valuesInlist):
            dicArray = dicInfo_local[i]
            sMBID = dicArray.get(MotorWMOVEParams.MBID.name,None)
            if sMBID == None:
                print(dicArray)
                continue
            iMBID = int(sMBID)
            sMBIDInstance = ModbusID.from_value(iMBID)
            sPOS = dicArray[MotorWMOVEParams.POS.name]
            iPOS = int(sPOS)
            sSPD = dicArray[MotorWMOVEParams.SPD.name]
            iSPD = int(sSPD)
            iSPDSlow = round(iSPD /3)
            #테이블 탐색 모드에서는 속도 줄인다.
            if iPOS > 0 and (valuesInlist == 3) and onScan:
                dicArray[MotorWMOVEParams.SPD.name] = iSPDSlow
            elif (valuesInlist == 1) and iMBID == ModbusID.TELE_SERV_MAIN.value:
                dicArray[MotorWMOVEParams.SPD.name] = iSPDSlow
                
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
                potRotate540, notRotate540, poscmdRotate540,poscurRotate540= GetPotNotCurPosServo(ModbusID.ROTATE_MAIN_540)
                # if onScan and dicAruco:
                #     lsDF = GetNewRotateArmList(dicAruco)
                #     if lsDF:
                #         rospy.loginfo(json.dumps(dicAruco, indent=4))
                #         node_CtlCenter_globals.listBLB.clear()
                #         node_CtlCenter_globals.listBLB.extend(lsDF)
                #         return APIBLB_ACTION_REPLY.E108
                    
                if not bIsAllMotorFolded540 and iSPD > MAINROTATE_RPM_SLOWEST:
                    if abs(cur_pos_13) < roundPulse and abs(cur_pos_6) < roundPulse :
                        dicMoveTeleSrv = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value, True, not_telesrv,DEFAULT_RPM_SLOW ,ACC_ST,DECC_ST)
                        node_CtlCenter_globals.dicTargetPos.clear()
                        SendCMD_Device([dicMoveTeleSrv])
                        node_CtlCenter_globals.listBLB.insert(0,dicInfo_local_org)
                        return APIBLB_ACTION_REPLY.E108
                    else:
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
            
            #리프팅 모터 제어
            if iMBID == ModbusID.MOTOR_V.value and CheckMotorOrderValid(dicArray):
                if sMBID == str(ModbusID.MOTOR_V.value):
                    currentWeight1,currentWeight2,currentWeightTotal = getLoadWeight()
                    if currentWeightTotal >= WEIGHT_LOADCELL_LIMITGRAM*10000:
                        SendMsgToMQTT(pub_topic2mqtt,MQTT_TOPIC_VALUE.BLB_ALARM.value,ALM_User.TRAY_WEIGHT_LIMIT.value)
                        SetWaitConfirmFlag(True,ALM_User.TRAY_WEIGHT_LIMIT)
                        return APIBLB_ACTION_REPLY.E109
                    else:
                        if iPOS < roundPulse:
                            DoorClose()
                    
                    if dicAruco:
                        # #TODO : 아르코마커가 인식되었습니다TTS. + 음성 메세지 클래스 정의할 것.
                        # TTSAndroid(TTSMessage.ARUCO_FOUND_OK.value)
                        resultDiff,diff_X,diff_Y= compare_dicts(dicAruco, ref_dict, CAM_LOCATION_MARGIN_OK)
                        rospy.loginfo(format_vars(resultDiff,diff_X,diff_Y))
                        #rospy.loginfo(json.dumps(dicAruco, indent=4))
                        GetNewRotateArmList(dicAruco)
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
            elif iMBID == ModbusID.ROTATE_SERVE_360.value and CheckMotorOrderValid(dicArray):
                #트레이 모터를 움직일때 현재 서빙부의 길이를 구한 후 200mm 이상이지 않으면 알람.
                if curDistanceSrvTele <= 50 and isRealMachine:
                    rospy.loginfo(dicArray)
                    StopEmergency(ALM_User.TRAY360_SAFETY.value)
                    return APIBLB_ACTION_REPLY.E102
                #전개후 아르코 마커가 인식되었을때 세부조절과 각도 튜닝 들어감.
                tray_angle = GetRotateTrayAngleFromPulse(iPOS)
                if abs(tray_angle) > 0:
                    if dicAruco:
                        GetNewRotateArmList(dicAruco)
                        # #TODO : 아르코마커가 인식되었습니다TTS. + 음성 메세지 클래스 정의할 것.
                        # TTSAndroid(TTSMessage.ARUCO_FOUND_OK.value)
                        resultDiff , diff_X,diff_Y= compare_dicts(dicAruco, ref_dict, CAM_LOCATION_MARGIN_OK)
                        #dicAruco = lsAruco[0]
                        rospy.loginfo(json.dumps(dicAruco, indent=4))
                        
                        # marker_X = dicAruco[ARUCO_RESULT_FIELD.X.name] / MARKER_X_RATE
                        # marker_Y = dicAruco[ARUCO_RESULT_FIELD.Y.name] / MARKER_Y_RATE
                        # #marker_X = dicAruco[ARUCO_RESULT_FIELD.XX.name] / MARKER_X_RATE
                        # #marker_Y = dicAruco[ARUCO_RESULT_FIELD.YY.name] / MARKER_Y_RATE
                        # marker_angle = (dicAruco[ARUCO_RESULT_FIELD.ANGLE.name]+360)%360
                        # #P1 - 범블비 원점 (회전중심)
                        # #P2 - 현재 카메라의 센터 좌표.
                        # #P3 - 아르코마커의 좌표
                        # P2X = CAM_OFFSET_TOP_X
                        # P2Y = CAM_OFFSET_TOP_Y
                        # P1X = P2X
                        # P1Y = P2Y+curDistanceSrvTele
                        # P3X = marker_X - P2X
                        # P3Y = marker_Y - P2Y
                        # #삼각형이 이루어지지 않는 경우 서빙암거리만 조절하는 예외처리 루틴 추가.
                        # side_a, distance_new_mm,side_c,angle_diff_to_minus, angle_B_deg, angle_C_deg = calculate_triangle(P1X,P1Y,P2X,P2Y,P3X,P3Y)
                        # curAngle_540_new = curAngle_540 + angle_diff_to_minus if P3X-P1X > 0 else curAngle_540 - angle_diff_to_minus
                        # rospy.loginfo(f'P1:{P1X},{P1Y},P2:{P2X},{P2Y},P3:{P3X},{P3Y},보정암길이:{distance_new_mm},보정각도:{curAngle_540_new}')
                        # lsArmBeTunedFromOrigin = GetStrArmExtendMain(distance_new_mm,curAngle_540_new,False)
                        # lsArmBeTunedFromCurrent = GetStrArmExtendMain(distance_new_mm,curAngle_540_new,True)
                        # lsArmBeFold = GetStrArmExtendMain(0,0,True)
                        # dicRotateNewVerySlow = GetDicRotateMotorMain(curAngle_540_new,MAINROTATE_RPM_SLOWEST,False)
                        # dicRotateNewNormal = GetDicRotateMotorMain(curAngle_540_new)
                        # rotateTime = dicRotateNewVerySlow.get(MotorWMOVEParams.TIME.name, MIN_INT)
                        # #is_within_deviationX = abs(CAM_OFFSET_TOP_X - marker_X) <= CAM_LOCATION_MARGIN_OK
                        # #is_within_deviationY = abs(CAM_OFFSET_TOP_Y - marker_Y) <= CAM_LOCATION_MARGIN_OK
                        # diff_X = abs(CAM_OFFSET_TOP_X - marker_X)
                        # diff_Y = abs(CAM_OFFSET_TOP_Y - marker_Y)
                        # diff_arm = abs(distance_new_mm-curDistanceSrvTele)
                        # rospy.loginfo(f'이동해야할 각도/시간:{-angle_diff_to_minus}:{rotateTime},이동해야할암길이:{diff_arm}mm,X오차:{diff_X},Y오차:{diff_Y}')
                        # #아르코마커가 정위치에 감지되어 바로 내리면 되는 경우
                        # if (diff_X <= CAM_LOCATION_MARGIN_OK and diff_Y <= CAM_LOCATION_MARGIN_OK) or node_CtlCenter_globals.aruco_try == 3:
                        #     rospy.loginfo(json.dumps(dicAruco, indent=4))
                        #     if (diff_X <= CAM_LOCATION_MARGIN_OK and diff_Y <= CAM_LOCATION_MARGIN_OK):
                        #         TTSAndroid(TTSMessage.ARUCO_CORRECTED.value)
                        #         #제대로 찾아냈으면 테이블 정보를 업데이트 한다.
                        #         updateTableServingInfo(curTargetTable,curDistanceSrvTele,curAngle_540,marker_angle,cur_posH)
                        #     else:
                        #         TTSAndroid(TTSMessage.ARUCO_NOT_CORRECTED.value)
                        #     node_CtlCenter_globals.aruco_try = 0

                        # #아르코마커가 약간 벗어나 있어 내리기 전 약간의 위치보정이 필요한 경우
                        # #아주 느린 속도로 메인회전을 돌리면서 암길이를 조절한다.
                        # elif rotateTime < 5 and diff_arm < 100:
                        # #elif rotateTime < 10 and diff_arm < 300:
                        # #elif diff_X <= CAM_LOCATION_MARGIN_FINE and diff_Y <= CAM_LOCATION_MARGIN_FINE:
                        #     #lsMotorOperationNew.insert(0,dicRotateNew)
                        #     #node_CtlCenter_globals.listBLB.pop(0)
                        #     if distance_new_mm > curDistanceSrvTele:
                        #         lsMotorOperationNew.append([dicRotateNewVerySlow])
                        #         lsMotorOperationNew.append(lsArmBeTunedFromCurrent)
                        #     else:
                        #         lsMotorOperationNew.append(lsArmBeTunedFromCurrent)                                
                        #         lsMotorOperationNew.append([dicRotateNewVerySlow])
                        #     listBLB[:] = lsMotorOperationNew + listBLB
                        #     # node_CtlCenter_globals.listBLB.insert(0, [GetDicRotateMotorMain(curAngle_540_new,MAINROTATE_SPD_SLOWEST,False)])
                        #     node_CtlCenter_globals.aruco_try += 1
                        #     TTSAndroid(TTSMessage.ARUCO_FINE_TUNING.value)
                        #     return APIBLB_ACTION_REPLY.E105
                        # else:
                        #     #lsArmFoldControl = GetStrArmExtendMain(0,0,False)
                        #     #node_CtlCenter_globals.listBLB.pop(0)
                        #     lsMotorOperationNew.append(lsArmBeFold)
                        #     lsMotorOperationNew.append([dicRotateNewNormal])
                        #     lsMotorOperationNew.append(lsArmBeTunedFromOrigin)
                        #     #node_CtlCenter_globals.listBLB.insert(0,lsMotorOperationNew)
                        #     listBLB[:] = lsMotorOperationNew + listBLB
                            
                        #     # node_CtlCenter_globals.listBLB.insert(0,[GetDicRotateMotorMain(curAngle_540_new)])
                        #     # node_CtlCenter_globals.listBLB.insert(0,GetStrArmExtendMain(0,0,True))
                        #     node_CtlCenter_globals.aruco_try += 1
                        #     TTSAndroid(TTSMessage.ARUCO_FOLD_TUNING.value)
                        #     return APIBLB_ACTION_REPLY.E105
                        
                        # x2 = 152
                        # y2=-2278
                        # angle_rotate_gap = GetAngleMargin(DIFF_X,DIFF_Y,x2,y2)
                        # angle_rotate_abs = abs(angle_rotate_gap)
                        #print(f'메인 회전 조금 더 돌아가야하는 각도 : {angle_rotate_gap}')
                        #아래 동작부는 정확한 움직임이 구현될 때까지 주석처리. 
                        # if  angle_rotate_abs > 10 and angle_rotate_abs < 30:
                        #     potRotate540, notRotate540, poscmdRotate540,poscurRotate540= GetPotNotCurPosServo(ModbusID.ROTATE_MAIN_540)
                        #     target_pulse = angle_rotate_abs*(abs(notRotate540)+potRotate540)/MAX_ANGLE_BLBBODY
                        #     dicTuning = getMotorMoveDic(ModbusID.ROTATE_MAIN_540.value, False, -target_pulse,20,ACC_DECC_SMOOTH,ACC_DECC_SMOOTH*2)
                        #     dicInfo_local.append(dicTuning)
                        #angle_new = int(dicAruco[ARUCO_RESULT_FIELD.ANGLE.name])-angle_rotate_gap
                        angle_new = (180+ int(dicAruco[ARUCO_RESULT_FIELD.ANGLE.name]))%360
                        dicNewTray = GetDicRotateMotorTray(angle_new)
                        dicInfo_local[i].update(dicNewTray)
                        infoMsg = f'이전 각도:{tray_angle},마커로 바뀐각도:{angle_new},마커정보:{json.dumps(dicAruco, indent=4)}'
                        SendInfoHTTP(f'이전 각도:{tray_angle},마커로 바뀐각도:{angle_new}')
                        rospy.loginfo(infoMsg)
                        
                        #print(f'기존메인회전타켓:{dicInfo_local[0][MotorWMOVEParams.POS.name]},마커타겟:{dicNewRotate[MotorWMOVEParams.POS.name]}')
                    # else:
                    #     #TTSQbi(TTSMessage.ARUCO_FOUND_FAIL.value)
                    #     TTSQbi(TTSMessage.REMAIN_30S.value)
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
        if len(listBLB) > 0:
            dfBackUp = listBLB.pop(0)
        else:
            sMsg = 'listBLB 스레드 점검필요'
            rospy.loginfo(sMsg)
            SendAlarmHTTP(sMsg,True,node_CtlCenter_globals.BLB_ANDROID_IP)

        lsFinalCmd = []
        lsFinalCmdEx = []
        #isArmControl = False
        for dicCtlTmp in dicInfo_local:
            sPos = dicCtlTmp.get(MotorWMOVEParams.POS.name, None)
            mbid = dicCtlTmp.get(MotorWMOVEParams.MBID.name, None)
            
            if mbid == str(ModbusID.ROTATE_MAIN_540.value):
                target_pulse = int(sPos)
                targetCW = target_pulse +potRotate540
                targetCCW = target_pulse -potRotate540
                arr =[target_pulse,targetCW,targetCCW]
                iPOSBack = min(arr, key=lambda x: abs(x - poscurRotate540))
                iPOSList = sorted(arr, key=lambda x: abs(x - poscurRotate540))[:2]
                diff_1 = abs(iPOSList[0] - poscurRotate540)
                diff_2 = abs(iPOSList[1] - poscurRotate540)
                if abs(diff_1 - diff_2) < roundPulse:
                    iPOSBack = min(iPOSList, key=abs)                
                strPos = ','.join(map(str, arr))
                
                SendInfoHTTP(f'MBID:{mbid},현재펄스:{poscurRotate540},최단회전 펄스 선택:{strPos} = ({iPOSBack})')
                dicCtlTmp[MotorWMOVEParams.POS.name] = iPOSBack            
            
            if mbid == str(ModbusID.ROTATE_SERVE_360.value):
                potRotate360, notRotate360, poscmdRotate360,poscurRotate360= GetPotNotCurPosServo(ModbusID.ROTATE_SERVE_360)
                target_pulse = int(sPos)
                targetCW = target_pulse +potRotate360
                targetCCW = target_pulse -potRotate360
                arr =[target_pulse,targetCW,targetCCW]
                iPOSBack = min(arr, key=lambda x: abs(x - poscurRotate360))
                strPos = ','.join(map(str, arr))
                SendInfoHTTP(f'MBID:{mbid},현재펄스:{poscurRotate360},최단회전 펄스 선택:{strPos} = ({iPOSBack})')
                dicCtlTmp[MotorWMOVEParams.POS.name] = iPOSBack
            
            if mbid == str(ModbusID.TELE_SERV_MAIN.value):
                if int(sPos) > 0 and len(dicInfo_local) > 1 and cur_pos_srv < -roundPulse:
                    rpmSrv = round((DEFAULT_RPM_SLOW/2)) if onScan else DEFAULT_RPM_SLOW
                    dicMoveTeleSrv = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value,True,0, rpmSrv,ACC_ST,DECC_ST)                    
                    node_CtlCenter_globals.dicTargetPos.clear()
                    SendCMD_Device([dicMoveTeleSrv])
                    node_CtlCenter_globals.listBLB.insert(0,dicInfo_local_org)
                    return APIBLB_ACTION_REPLY.E108
                elif int(sPos) == 0 and (cur_pos_srv-not_telesrv) > roundPulse and cur_pos_srv < roundPulse and len(dicInfo_local) > 1:
                    dicMoveTeleSrv = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value, True, not_telesrv,DEFAULT_RPM_SLOW , ACC_ST,DECC_ST)
                    SendCMD_Device([dicMoveTeleSrv])
                    return APIBLB_ACTION_REPLY.E108

            elif mbid == str(ModbusID.TELE_SERV_MAIN.value) and sPos == '0' and len(dicInfo_local) > 1:
                #isArmControl = True
                lsFinalCmd.clear()
                lsFinalCmd= GetStrArmExtendMain(0,0,True)
                break

            # if len(lsAruco) > 0 and onScan:
            #     dicAruco = lsAruco[0]
            #     lsDF = GetNewRotateArmList(dicAruco)
            #     if lsDF:
            #         # #TODO : 아르코마커가 인식되었습니다TTS. + 음성 메세지 클래스 정의할 것.
            #         TTSAndroid(TTSMessage.ARUCO_FOUND_OK.value)
            #         resultDiff,diff_X,diff_Y= compare_dicts(dicAruco, ref_dict, CAM_LOCATION_MARGIN_OK)
            #         node_CtlCenter_globals.listBLB.clear()
            #         node_CtlCenter_globals.listBLB.extend(lsDF)
            #         return APIBLB_ACTION_REPLY.E108
            
            lsFinalCmd.append(dicCtlTmp)
            
        for dicCtlTmp2 in lsFinalCmd:
            #mbid = dicCtlTmp2[MotorWMOVEParams.MBID.name]
            mbid = dicCtlTmp2.get(MotorWMOVEParams.MBID.name)
            if mbid is None:
                print(dicCtlTmp2)
                continue
            mbidInstance = ModbusID.from_value(mbid)
            target_pulse = int(dicCtlTmp2[MotorWMOVEParams.POS.name])
            rotateRPM = int(dicCtlTmp2[MotorWMOVEParams.SPD.name])
            #dicCtlTmp2[MotorWMOVEParams.TIME.name] = GetTimeFromRPM(mbidInstance, target_pulse, rotateRPM)            
            if CheckMotorOrderValid(dicCtlTmp2):                
                dicCtlTmp2[MotorWMOVEParams.TIME.name] = GetRPMFromTimeAccDecc(dicCtlTmp2)
                if mbid == str(ModbusID.TELE_SERV_MAIN.value):
                    if onScan:
                    #if target_pulse != 0 and :
                        #모니터 감지각으로 틸팅 변경
                        #TiltDetectingMonitor()
                        TiltDown()  #데모에서는 아래로.
                        #CamControl(True)
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

                if mbid == str(ModbusID.MOTOR_V.value) and target_pulse != 0:
                    #모니터 감지각으로 틸팅 변경
                    CamControl(False)
                    #TiltMaxUp()
                    #TiltDetectingMonitor()
                    #TiltDown()  #데모에서는 아래로.
                    if int(curTargetNode) == node_KITCHEN_STATION:
                        #dicCtlTmp2[MotorWMOVEParams.POS.name] = 1131241
                        SetWaitConfirmFlag(False,AlarmCodeList.JOB_COMPLETED)
                        # if abs(cur_posH) > roundPulse:
                        #   dicLoc = getMotorHomeDic(ModbusID.MOTOR_H.value)
                        #   SendCMD_Device([dicLoc])
                          #SendAlarmHTTP(f'포지션을 0 으로 보정합니다:{cur_posH}',True,node_CtlCenter_globals.BLB_ANDROID_IP)
                        node_CtlCenter_globals.dicPOS_ABS.clear()
                          #SendInfoHTTP()
                    # elif int(curNode) == 4 or int(curNode) == 5:
                    #     dicCtlTmp2[MotorWMOVEParams.POS.name] = 1290000
                        
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
            #추후 전개 전 트레이가 안착되어있는지 확인하는 세이프티 코드 추가할 것.
            #compressed_output = json.dumps(lsFinalCmd, separators=(',', '\n'))
            dfLog = pd.DataFrame(lsFinalCmdEx)
            rospy.loginfo(f"Moving {len(lsFinalCmdEx)} motors,curTable:{curTargetTable},{dfLog}")

            lsNextCmd = []
            # print(node_CtlCenter_globals.listBLB)
            # lsFinalMbid = ""
            # nextCmdMbid = ""
            #메인회전 모터 필터링 - 다음 명령어도 메인회전이거나 다음 명령어가 H 인 경우 스킵한다.
            if len(node_CtlCenter_globals.listBLB) > 0:
                lsDF = []
                lsNextCmd = node_CtlCenter_globals.listBLB[0]
                isNextDict = isinstance(lsNextCmd, dict)
                if isNextDict:
                    lsDF.append(lsNextCmd)
                else:
                    lsDF.extend(lsNextCmd)
                dfNext = pd.DataFrame(lsDF)
                dfNext.dropna(how='all', inplace=True)
                rospy.loginfo(dfNext)
                contains_nan = dfNext.isnull().values.any()
                if contains_nan:
                  rospy.loginfo(dfNext)
                if len(lsFinalCmdEx) == 1:
                    lsCurCmd = lsFinalCmdEx[0]
                    curMBID = lsCurCmd.get(MotorWMOVEParams.MBID.name, "")
                    if curMBID == str(ModbusID.ROTATE_MAIN_540.value):
                        if len(lsNextCmd) == 1:
                            nextMBID = lsNextCmd[0].get(MotorWMOVEParams.MBID.name, "")
                            if curMBID == nextMBID:
                                rospy.loginfo('Skipped by dup 540 cmd')
                                return APIBLB_ACTION_REPLY.E105
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
            SendCMD_Device(lsFinalCmdEx)
        UpdateLastCmdTimeStamp()
        UpdateLastBalanceTimeStamp()
        # if len(listBLB) == 0 and not isScanMode():
        #     SetWaitConfirmFlag(True,AlarmCodeList.WAITING_USER)
        # else:
        #   print(listBLB)
    
    return APIBLB_ACTION_REPLY.R101