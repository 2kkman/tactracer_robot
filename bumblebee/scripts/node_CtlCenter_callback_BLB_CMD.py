#!/usr/bin/env python3
from node_CtlCenter_func_control import *

def BLB_CMD_Profile(PROFILE):
    lsMotorOperation = []
    lsModbusRequests = []    
    
    recvDataMap = {}
    if is_json(PROFILE):
        recvDataMap = json.loads(PROFILE)
    rospy.loginfo(recvDataMap)    
    # if APIBLB_FIELDS_ACTION.target.name in recvDataMap.keys():
    #   #target = 주행시작하라는 명령어
    #   lsDicArray=recvDataMap[APIBLB_FIELDS_ACTION.target.name]
    #   df = pd.DataFrame(lsDicArray)
    #   #df_manager = DataFrameManager(df)
    #   dicTmp = df.iloc[-1].get()
    #   endnodeStr = dicTmp[APIBLB_FIELDS_TASK.endnode.name]
    #   nodeID = try_parse_int(endnodeStr, MIN_INT)
    #   if nodeID == MIN_INT:
    #     nodeID = endnodeStr[1:]
      
    #   print(dicTmp)
    #   gotoNode(device_ID, nodeID, True)
    
    if isFileExist(PROFILE):
        lsMotorOperation = GetCustomFileControl(PROFILE)
    elif PROFILE.find(APIBLB_FIELDS_ACTION.resume.name) >= 0:
        #SetWaitConfirmFlag(False,AlarmCodeList.OK)
        return ResumeState()
    elif PROFILE.find(APIBLB_FIELDS_ACTION.pause.name) >= 0:
        #SetWaitConfirmFlag(True,AlarmCodeList.WAITING_USER)
        df, bAPIVal = SetPauseState()
        return bAPIVal
    elif PROFILE.find(APIBLB_FIELDS_ACTION.cancel.name) >= 0 or PROFILE.find(APIBLB_FIELDS_ACTION.home.name) >= 0:
      CancelJob()
    # elif PROFILE.find(APIBLB_FIELDS_ACTION.home.name) >= 0:
    #     #StopEmergency(ALM_User.USER_ESTOP.value)
    #     curTargetTable,curTarNode = GetTargetTableNode()
    #     dfReceived = GetDF(curTargetTable)
    #     API_robot_navigation_info(dfReceived,APIBLB_STATUS_TASK.Canceled)
    #     node_CtlCenter_globals.listBLB.clear()
    #     node_CtlCenter_globals.listTable.clear()
    #     StopAllMotors(DECC_MOVE_H)
    #     SetWaitConfirmFlag(False,AlarmCodeList.OK)
    #     return APIBLB_ACTION_REPLY.R101
    else:
        #문자열 자체로 들어오는 경우 2000`1000`90
        paramArmControl = PROFILE.split(sep=sDivItemComma)
        DEFAULT_STRING = 'NONE'
        paramStr1 = list_get(paramArmControl, 0, DEFAULT_STRING)
        paramStr2 = list_get(paramArmControl, 1, DEFAULT_STRING)
        paramStr3 = list_get(paramArmControl, 2, DEFAULT_STRING)
        paramInt1 = try_parse_int(paramStr1, MIN_INT)
        paramInt2 = try_parse_int(paramStr2, MIN_INT)
        paramInt3 = try_parse_int(paramStr3, MIN_INT)
        lenSrvInner,cur_angle_540 ,cur_angle_360 = GetCurrentPosDistanceAngle()
        # paramInt2 = strToRoundedInt(paramArmControl[1])
        # paramInt3 = strToRoundedInt(paramArmControl[2])
        #paramInt1 = strToRoundedInt(next((item for item in paramArmControl if item), DEFAULT_STRING))
        # if len(paramArmControl) == 1:
        #     endNode = paramInt1
        #     startNode = GetCurrentNode()
        #     listSeqMapOptimized,listSeqMapOrg = getSeqMap(startNode,endNode)
        #     SetWaitConfirmFlag(False, AlarmCodeList.OK)
        #     # print(f'최적화된 경로 : {listSeqMapOptimized}')
        #     # print(f'원본 경로 : {listSeqMapOrg}')
        #     lsModbusRequests.extend(listSeqMapOptimized)

        if len(paramArmControl) == 2:
            if paramInt2 == MIN_INT:
                if paramStr2 == 'S':
                    tableList = paramStr1.split(sep=sDivSlash)
                    lenTables = len(tableList)
                    if  lenTables> 0:
                        #SetWaitConfirmFlag(True, AlarmCodeList.WAITING_USER)
                        ClearArucoTable()
                        AppendTableList(tableList)
                        #TTSAndroid(f'확인버튼을 누르시면 시작합니다.')
                if paramStr2 == 'T':
                    endNode = paramInt1
                    startNode = GetCurrentNode()
                    listSeqMapOptimized,listSeqMapOrg = getSeqMap(startNode,endNode)
                    SetWaitConfirmFlag(False, AlarmCodeList.OK)
                    # print(f'최적화된 경로 : {listSeqMapOptimized}')
                    # print(f'원본 경로 : {listSeqMapOrg}')
                    lsModbusRequests.extend(listSeqMapOptimized)
                
            elif paramInt2 == BLD_PROFILE_CMD.TrayHome.value:
                lsModbusRequests.extend(GetLiftControl(True))
                lsModbusRequests.append(getListedDic(dicServFold))
            else:   #제어HTTP - 마커1 버튼
                dicAruco = {}
                lsAruco = filter_recent_data(ARUCO_RESULT_FIELD.LASTSEEN.name,GetArucoMarkerInfo(),1)
                if lsAruco:
                    dicAruco.update(lsAruco[0])
                    lsDF=calculate_robot_translation2(dicAruco)
                    # node_CtlCenter_globals.dicARUCO_last.clear()
                    # lsDF = GetNewRotateArmList(dicAruco)
                    #return
                    #rospy.loginfo(json.dumps(lsDF, indent=4))
                    #print(lsDF)
                    lsModbusRequests.extend(lsDF)
                else:
                    print('Marker not found.')
                #lsModbusRequests.extend(lsDF)

        if len(paramArmControl) == 3:
            SetWaitConfirmFlag(False, AlarmCodeList.OK)
            
            lsRotate = []
            if abs(paramInt3) == 1: #인풋이 길이와 각도로 들어온 경우
                #x,y = calculate_coordinates(paramInt1,paramInt2)
                #print(f'변환된 좌표 : X:{x},Y:{y}')
                lsMotorOperation = GetStrArmExtendMain(paramInt1,paramInt2,True)
                #lsRotate.append(GetDicRotateMotorMain(paramInt2))
            else:   #x,y 좌표 그대로 들어온 경우
                distanceServingTeleTotal, angle_degrees = calculate_distance_and_angle(paramInt1, paramInt2)    
                angle_degrees = (360-angle_degrees)
                print(f'길이:{distanceServingTeleTotal},각도:{angle_degrees}')
                lsMotorOperation = GetStrArmExtendMain(paramInt1,paramInt2,True)
                #lsRotate.append(GetDicRotateMotorMain(angle_degrees))
            if paramInt2 == 0:
                lsModbusRequests.append(lsMotorOperation)                    
                if paramInt3 >= 0 and len(lsRotate) > 0:
                    lsModbusRequests.append(lsRotate)
            else:                
                if paramInt3 >= 0:
                    lsModbusRequests.append(lsRotate)
                lsModbusRequests.append(lsMotorOperation)                    

        elif paramInt1 == BLD_PROFILE_CMD.ESTOP.value:
            StopEmergency(ALM_User.USER_ESTOP.value)
        elif paramInt1 == BLD_PROFILE_CMD.balDataClear.value:
            node_CtlCenter_globals.dicWeightBal.clear()
            rospy.loginfo(f'Record Cleared')
        elif paramInt1 == BLD_PROFILE_CMD.balDataSave.value:
            saveDic_ToFile(node_CtlCenter_globals.dicWeightBal, strFileWeightBal, None, True)
            lenDic = len(node_CtlCenter_globals.dicWeightBal)
            if lenDic > 0:
                rospy.loginfo(f'{lenDic} Records Saved as {strFileWeightBal}')
                print(node_CtlCenter_globals.dicWeightBal)
            else:
                rospy.loginfo(f'No record to save. Discarded.')                        
        elif paramInt1 == BLD_PROFILE_CMD.balDataRecord.value:
            cmdpos_arm1, curpos_arm1 = GetPosServo(ModbusID.BAL_ARM1)
            cmdpos_tele, curpos_tele = GetPosServo(ModbusID.TELE_BALANCE)
            total_balPos = curpos_tele+curpos_arm1
            currentWeight1,currentWeight2,currentWeightTotal = getLoadWeight()
            if currentWeightTotal < 0:
                rospy.loginfo(f'트레이 무게 데이터가 수신되지 않았습니다.')  
            else:
                node_CtlCenter_globals.dicWeightBal[currentWeightTotal] = total_balPos
                rospy.loginfo(f'Added weight {currentWeightTotal} -> Pos:{total_balPos}')
        elif paramInt1 == BLD_PROFILE_CMD.balLiftDown.value:
            lsLiftDown = GetListLiftDown()
            lsModbusRequests.append(lsLiftDown)
        elif paramInt1 == BLD_PROFILE_CMD.balLiftUp.value:
            lsLiftDown = GetListLiftUp()
            lsModbusRequests.append(lsLiftDown)
        elif paramInt1 == BLD_PROFILE_CMD.balSrvArm100mm.value:
            #x,y = calculate_coordinates(lenSrvInner+paramInt1,cur_angle_540)
            #lsMotorOperation = GetStrArmExtendMain(x,y,True)
            lsMotorOperation = GetStrArmExtendMain(110,0,True)
            lsModbusRequests.append(lsMotorOperation)
        elif paramInt1 == BLD_PROFILE_CMD.balSrvArmZERO.value:
            lsMotorOperation = GetStrArmExtendMain(0,cur_angle_540,True)
            lsModbusRequests.append(lsMotorOperation)
        elif paramInt1 == BLD_PROFILE_CMD.bal540_45CW.value:
            lsRotate540_down = getListedDic(GetDicRotateMotorMain(cur_angle_540+45))
            lsModbusRequests.append(lsRotate540_down)
        elif paramInt1 == BLD_PROFILE_CMD.bal540_ZERO.value:
            lsRotate540_down = getListedDic(GetDicRotateMotorMain(0))
            lsModbusRequests.append(lsRotate540_down)
        elif paramInt1 == BLD_PROFILE_CMD.balTray_45CW.value:
            lsRotate540_down = getListedDic(GetDicRotateMotorTray(cur_angle_360+45))
            lsModbusRequests.append(lsRotate540_down)
        elif paramInt1 == BLD_PROFILE_CMD.balTray_ZERO.value:
            lsRotate540_down = getListedDic(GetDicRotateMotorTray(0))
            lsModbusRequests.append(lsRotate540_down)
        elif paramInt1 == BLD_PROFILE_CMD.SRV_EXPAND.value:
            lsRotate540_down = getListedDic(dicServExpand)
            lsModbusRequests.append(lsRotate540_down)
        elif paramInt1 == BLD_PROFILE_CMD.SRV_FOLD.value:
            lsRotate540_down = getListedDic(dicServFold)
            lsModbusRequests.append(lsRotate540_down)
        elif paramInt1 == BLD_PROFILE_CMD.BACK_HOME.value:
            lsRotate540_down = getListedDic(dicBackHome)
            lsModbusRequests.append(lsRotate540_down)
        elif paramInt1 == BLD_PROFILE_CMD.SAVE_POS.value:
            SaveCurrentPos()
        elif paramInt1 == BLD_PROFILE_CMD.SCANTABLE.value:
            TiltDetectingMonitor()
        elif paramInt1 == BLD_PROFILE_CMD.CALI_TRAY.value:
            callResult = StartCaliTray()
            rospy.loginfo(f'트레이 캘리브레이션 요청결과 : {callResult}')
        elif paramInt1 == BLD_PROFILE_CMD.TrayHome.value:
            #서빙 텔레스코픽 트레이 최소화
            lsModbusRequests.extend(GetLiftControl(True))
            lsModbusRequests.append(getListedDic(dicServFold))
        elif paramInt1 == BLD_PROFILE_CMD.CALI_MAINROTATE.value:
            callResult = StartCaliMainRotate()
            rospy.loginfo(f'메인회전 캘리브레이션 요청결과 : {callResult}')
        elif paramInt1 == BLD_PROFILE_CMD.MOVE_MOTOR_H.value:
            if isReadyToMoveH_and_540():
              distNew = distance_to_pulseH(paramInt2)
              # SendCMD_Device([getMotorMoveDic(
              #         ModbusID.MOTOR_H.value, False, distNew, DEFAULT_RPM_SLOW, ACC_MOVE_H, DECC_MOVE_H
              #     )])
              lsModbusRequests.append([getMotorMoveDic(
                      ModbusID.MOTOR_H.value, True, distNew, SPD_MOVE_H, ACC_MOVE_H, DECC_MOVE_H
                  )])
            #lsModbusRequests.clear()
        elif paramInt1 == BLD_PROFILE_CMD.TiltMaxUp.value:
            TiltServFinish()
        elif paramInt1 == BLD_PROFILE_CMD.TiltDown.value:
            TiltArucoScan()
        elif paramInt1 == BLD_PROFILE_CMD.TiltFace.value:
            TiltFace()
        elif paramInt1 == BLD_PROFILE_CMD.TiltDiagonal.value:
            TiltDiagonal()
        elif paramInt1 == BLD_PROFILE_CMD.DoorUp.value:
            DoorOpen()
        elif paramInt1 == BLD_PROFILE_CMD.DoorStop.value:
            DoorStop()
        elif paramInt1 == BLD_PROFILE_CMD.DoorDown.value:
            DoorClose()
        else:
            if paramInt1 == MIN_INT:
              cmdStr = paramArmControl[0]
              paramStr = paramArmControl[1]
              if cmdStr == ServiceBLB.ROTATE_360.name:
                lsRotate540_down = getListedDic(GetDicRotateMotorTray(paramInt2))
                lsModbusRequests.append(lsRotate540_down)
              elif cmdStr == ServiceBLB.ROTATE_540.name:
                lsRotate540_down = getListedDic(GetDicRotateMotorMain(paramInt2))
                lsModbusRequests.append(lsRotate540_down)
              elif cmdStr == ServiceBLB.TTS_ITX.name:
                TTSItx(paramStr)
              elif cmdStr == ServiceBLB.TTS_QBI.name:
                TTSAndroid(paramStr)
            else:
              if len(lsModbusRequests) == 0:
                GetTuningArms(paramInt1)
    if len(lsModbusRequests) == 0:
      return APIBLB_ACTION_REPLY.R101
    else:
      return lsModbusRequests

print(os.path.splitext(os.path.basename(__file__))[0],getDateTime())