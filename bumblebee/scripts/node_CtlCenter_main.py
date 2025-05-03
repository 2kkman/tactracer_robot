#!/usr/bin/env python3
# import node_CtlCenter_globals
# from node_CtlCenter_import import *
# from tactracer_robot.bumblebee.scripts.node_CtlCenter_func_logic import *
#node_CtlCenter_globals.listBLB = GetLiftControlUp()

from node_CtlCenter_VerifyCmd import *
node_CtlCenter_globals.listBLB.clear()
#SendAlarmHTTP(ALM_User.JOB_SUSPENDED.value,True,node_CtlCenter_globals.BLB_ANDROID_IP)
#API_SendAlarm(ALM_User.ROBOT_JUST_INITIALIZED)
#print(pulseH_to_distance(3065957))
def get_available_services():
    """
    현재 활성화된 ROS 서비스 목록을 반환하는 함수.

    Returns:
        list: 활성화된 서비스 이름 리스트
    """
    master = rospy.get_master()  # ROS 마스터와 통신
    try:
        code, msg, services = master.getSystemState()  # 시스템 상태 가져오기
        if code == 1:  # 성공적으로 데이터를 가져옴
            # 서비스 리스트는 세 번째 반환값의 두 번째 요소에 있음
            #service_list = [srv[0] for srv in services]
            #print(services)
            return services[2]
        else:
            rospy.logwarn(f"Failed to get services: {msg}")
            return []
    except Exception as e:
        rospy.logerr(f"Error while fetching services: {e}")
        return []

#getNodeDirection(20)
# hostName = get_hostname()
# print(hostName)
bInitOK = False

#print(get_connected_monitors())
# dicCurNode = getTableServingInfo(4)
# print(dicCurNode)

# with open(node_CtlCenter_globals.strFileTableNode, 'w') as file:
#     json.dump(dicCurNode, file)
#TTSQbi(TTSMessage.ARUCO_CORRECTED.value)

class CtlCenter():
    def __init__(self):
        self.CMD_service = rospy.Service(ServiceBLB.CTL_BLB.name, utilboxData, self.CTL_BLB) #nodeFrom:str, nodeTo:str, crossID:str
        
    def CTL_BLB(self, req):
        message = req.message
        bReturnAPI = APIBLB_ACTION_REPLY.R101
        #if message != APIBLB_FIELDS_ACTION.resume.name:
        # if message != APIBLB_FIELDS_ACTION.cancel.name:
        #   rospy.loginfo(f'{message} from {sys._getframe(0).f_code.co_name}')
        try:
            lsModbus = BLB_CMD_Profile(message)
            if isinstance(lsModbus, list) and len(lsModbus) > 0:
              bReturnAPI = RunListBlbMotorsEx(lsModbus)
            else:
              bReturnAPI = lsModbus
        except Exception as e:
            bReturnAPI = APIBLB_ACTION_REPLY.E500
            rospy.loginfo(traceback.format_exc())
        #BLB Core 에 메세지 응답 코드는 통일해서 보낸다. (R101, E500 등)
        #dictReturn = {APIBLB_ACTION_REPLY.code.name :bReturnAPI.name,APIBLB_ACTION_REPLY.message.name: bReturnAPI.value}        
        if bReturnAPI is None:
          returnMsg = f'{message} 의 리턴값이 없습니다.'
          SendInfoHTTP(returnMsg)
        else:
          dictReturn = {APIBLB_ACTION_REPLY.code.name :APIBLB_ACTION_REPLY.R101.name,APIBLB_ACTION_REPLY.message.name: bReturnAPI.value}
          returnMsg = json.dumps(dictReturn)
        return utilboxDataResponse(True if bReturnAPI == APIBLB_ACTION_REPLY.R101 else False, returnMsg)

# print(isLinkHorizon('34'))

# #시작을 역주행 ( - ) 로 하고 싶으면 아래 세줄을 주석처리한다.
# dicInfo_tmp = getMotorMoveDic(ModbusID.MOTOR_H.value, False, -1000, DEFAULT_SPD_MAX, DEFAULT_ACC,DEFAULT_ACC)
# node_CtlCenter_globals.lastSendDic_H.update(dicInfo_tmp)
# print(getSeqMap(5, node_KITCHEN_STATION))
#updateShortCutInfo(1,4,0)
# SetCurrentTable()
# SetCurrentTable('H1')
#TTSQbi(TTSMessage.ALARM_BATTERY.value,True,1)
#print(calculate_deceleration_factor(0.4))
# JSON 파일로 저장
# with open(strJsonGraphInfo, "w") as file:
#     json.dump(node_CtlCenter_globals.graph, file)    

# with open(strJsonScanInfo, "r") as file:
#     node_CtlCenter_globals.ScanInfo = json.load(file)

# dicLinkInfo = {TableInfo.LINK_ID.name:'14',
#                   SeqMapField.START_NODE.name:0,
#                   SeqMapField.END_NODE.name:distance_to_pulseH(2535)}
# node_CtlCenter_globals.dfLinkPosInfo=pd.concat([node_CtlCenter_globals.dfLinkPosInfo, pd.DataFrame([dicLinkInfo])], ignore_index=True)

# print(find_dead_end_nodes_from_file(node_CtlCenter_globals.strFileShortCut))
#TTSItx('서비스를 시작합니다',True,1)

# with open(strJsonScanInfo, "r") as file:
#     node_CtlCenter_globals.ScanInfo = json.load(file)
# # with open(strJsonScanInfo, "r") as file:
#     # node_CtlCenter_globals.ScanInfo = json.load(file)
# node_CtlCenter_globals.dfLinkPosInfo = pd.read_csv(strJsonLinkPosInfo, delimiter=sDivTab)
# g = Graph(node_CtlCenter_globals.graph)
# g.visualize_graph()

# SaveTableNodeInfo()
# a = Graph(node_CtlCenter_globals.graph)
# a.visualize_graph()
#GetStraightLinks()
# LightTrayCell(0,1000000,0)
# LightTrayCell(1,1000000,0)
#LightWelcome(True)
# df_updated = update_node_bindings(node_CtlCenter_globals.strCSV_TableLoc,node_CtlCenter_globals.strCSV_NodeLoc)
# print(df_updated)
AppendTableHistory(GetTableTarget())
ReloadSvrTaskList()
LightWelcome(False)
CamControl(False)
#API_call_http(BLB_ANDROID_IP,BLB_ANDROID_PORT,"control",f'vol=70')
#API_call(BLB_ANDROID_IP,BLB_ANDROID_PORT,"","control",f'vol=70')
if __name__ == "__main__":
    if IsEnableSvrPath():
      SetCurrentTable()
      API_SetCurrentNode()
    rospy.init_node(node_name, anonymous=False)
    CtlCenter()
    rospy.loginfo(f"{node_name} Started")     
    #runFromLaunch = float(rospy.get_param(f"~{ROS_PARAMS.lidar_gnd_limit.name}", default=0.56))    
    bReturn_ANDROID,strResult_ANDROID=API_call_Android(node_CtlCenter_globals.BLB_ANDROID_IP,BLB_ANDROID_PORT,f'svrip={IP_MASTER}')
    if not bReturn_ANDROID:
      rospy.loginfo(f"안드로이드 통신에러:{node_CtlCenter_globals.BLB_ANDROID_IP}:{BLB_ANDROID_PORT}")
    else:
      rospy.loginfo(f"안드로이드 통신성공:{strResult_ANDROID}")

    if isRealMachine:
      # dynamic_reconfigure 클라이언트 생성
      try:
        node_CtlCenter_globals.dynamic_reconfigure_client = Client(node_CtlCenter_globals.dynamic_reconfigure_clientName, timeout=1)
      except Exception as e:
        rospy.loginfo(f"dynamic_reconfigure_client error : {e}")
      SetLidarCrop(LidarCropProfile.SERVING_ARM)      
      #bReturn_CROSS,strResult_CROSS=API_call_Android(BLB_CROSS_IP_DEFAULT,HTTP_COMMON_PORT,f'svrip={IP_MASTER}')
      bReturn_CROSS,strResult_CROSS=API_call_http(BLB_CROSS_IP_DEFAULT, HTTP_COMMON_PORT, 'control', f'svrip={IP_MASTER}')
      if not bReturn_CROSS:
        rospy.loginfo(f"분기기 통신에러:{node_CtlCenter_globals.BLB_CROSS_IP_DEFAULT}:{HTTP_COMMON_PORT}")
      else:
        rospy.loginfo(f"분기기 통신성공:{strResult_CROSS}")

      # bReturn_RFID,strResult_RFID = RFIDControl(False)
      # if not bReturn_RFID:
      #   sErrRFID = f"RFID 통신에러:{node_CtlCenter_globals.BLB_RFID_IP}:{HTTP_COMMON_PORT}"
      #   TTSAndroid(sErrRFID,1)
      #   rospy.loginfo(sErrRFID)
      # else:
      #   rospy.loginfo(f"RFID 통신성공:{bReturn_RFID}")
      #   time.sleep(MODBUS_EXCEPTION_DELAY)
      #   bReturn_RFID,strResult_RFID = RFIDPwr(2000)
      #   time.sleep(MODBUS_EXCEPTION_DELAY)
      #   bReturn_RFID,strResult_RFID = RFIDControl(True)


       #프리징 되는 경우 체크
      # if not bReturn_CHARGE:
      #   rospy.loginfo(f"충전 스테이션 통신에러:{node_CtlCenter_globals.BLB_CHARGE_IP}")
      # else:
      #   rospy.loginfo(f"충전 스테이션 통신성공:{strResult_CHARGE}")
     
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    # 현재 생성된 토픽 리스트를 모아 토픽명 배열을 만든다.
    lsTmp = rospy.get_published_topics()
    #rospy.loginfo(lsTmp)
    service540 = rospy.Service(ServiceBLB.ROTATE_540.value, Kill, Rotate540)
    service360 = rospy.Service(ServiceBLB.ROTATE_360.value, Kill, Rotate360)
    cur_ros_param = getROS_Param(LIDAR_CROP_PARAMS.range_max_x.name)
    for lsCur in lsTmp:
        topic_name = (str)(lsCur[0])
        topic_type = (str)(lsCur[1])

        if topic_type.find("std_msgs/String") >= 0:
            node_CtlCenter_globals.lsTopicList.append(topic_name)
    # lsTopicList 토픽명 배열 완성!
    #rospy.loginfo(node_CtlCenter_globals.lsTopicList)
    lsServiceRaw = get_available_services()
    for lsServiceSet in lsServiceRaw:
      node_CtlCenter_globals.lsServices.append(lsServiceSet[0])  
    #rospy.loginfo(node_CtlCenter_globals.lsServices)
    waitCnt = 0
    while(not isInitMotorsAll()):
        waitCnt += 1
        rospy.loginfo(f'Wait {waitCnt}s for modbus_IF')
        time.sleep(1)
    DI_POT,DI_NOT,DI_HOME,SI_POT = GetPotNotHomeStatus(ModbusID.MOTOR_H)
    cmdpos_H, curpos_H = GetPosServo(ModbusID.MOTOR_H)
    if not isRealMachine:
        fieldvalue = f'q={BLD_PROFILE_CMD.WLOC_NOT.value}'
        print(API_call_http(GetMasterIP(), HTTP_COMMON_PORT, 'CMD_DEVICE', fieldvalue))
        # pot_int,not_int =GetPotNotServo(ModbusID.TELE_SERV_MAIN)
        # SendCMD_Device([getMotorLocationSetDic(ModbusID.TELE_SERV_MAIN.value, not_int)])
    
    if isTrue(DI_POT):
      dicLoc = getMotorLocationSetDic(ModbusID.MOTOR_H.value, 0)
      SendCMD_Device([dicLoc])
      TTSAndroid('현재 충전소에 있습니다.',1)
      SetCurrentNode(node_KITCHEN_STATION)
    # else:
    #   #RFID로 위치 확인 isRealMachine 일때만
    #   #RFID_DF 에 값이 들어올때까지 기다릴것.
    #   for i in range(0, 3):
    #       try:
    #         sEPC = node_CtlCenter_globals.dicEPC_last.get(RFID_RESULT.EPC.name)
    #         #sEPC = node_CtlCenter_globals.dicEPC_last.get(TableInfo.NODE_ID.name)
    #         if sEPC is None:
    #           raise ValueError(f"RFID 데이터가 없습니다.")
    #         else:
    #           break
    #       except Exception as e:  
    #         time.sleep(1)
    #       # if node_CtlCenter_globals.dfEPCView.empty:
    #       #     time.sleep(1)
    #       # else:
    #       #     break
    #   if len(node_CtlCenter_globals.dicEPC_last) == 0:
    #     dicCurNodeInfo=GetCurrentNodeDicFromPulsePos(node_CtlCenter_globals.dfEPCTotal,curpos_H)
    #     if dicCurNodeInfo:
    #       node_current = dicCurNodeInfo.get(TableInfo.NODE_ID.name)
    #       SetCurrentNode(node_current)
    #       TTSAndroid(f'{node_current}번 노드가 현재 위치 입니다',1)
    #   else:
    #     bReturn_RFID,strResult_RFID = RFIDControl(False)
    #     time.sleep(MODBUS_EXCEPTION_DELAY)
    #     bReturn_RFID,strResult_RFID = RFIDPwr(1000)
    #     dicEPCView = node_CtlCenter_globals.dicEPC_last
    #     ts_current = dicEPCView[RFID_RESULT.TIMESTAMP.name]
    #     sEPCCurrent = dicEPCView[MAPFIELD.EPC.name]
    #     dicEPCPos = GetEPC_Pos_Info(sEPCCurrent)
    #     node_current = dicEPCPos.get(TableInfo.NODE_ID.name)
    #     ts_passed = (getDateTime() - datetime.fromtimestamp(ts_current)).total_seconds()
    #     #if ts_passed <= 1:
    #     SetCurrentNode(node_current)
    #     #cur_pos=GetEPC_Loc_Master(sEPCCurrent)
    #     cur_pos=GetNodePos_fromEPC(sEPCCurrent)
    #     iLoc = try_parse_int(cur_pos,MIN_INT)
    #     TTSAndroid('RFID 위치감지 성공.',1)
    #     if iLoc != MIN_INT:
    #       sCUR_POS = iLoc
    #       dicLoc = getMotorLocationSetDic(ModbusID.MOTOR_H.value, sCUR_POS)
    #       SendCMD_Device([dicLoc])
        
        # #전체 DF 를 확인할 것
        # #dicEPCStatus=GetEPCDict(TableInfo.NODE_ID,RFID_RESULT.TIMESTAMP.name)
        # print(node_CtlCenter_globals.dfEPCView)
        # dicEPCStatus = df_to_dict(node_CtlCenter_globals.dfEPCView,TableInfo.NODE_ID.name,RFID_RESULT.TIMESTAMP.name)
        # # EPC 길이가 5 초과인 행만 필터링
        # filtered = node_CtlCenter_globals.dfEPCView[
        #     node_CtlCenter_globals.dfEPCView[MAPFIELD.EPC.name].astype(str).str.len() > 5
        # ]
        # filtered['TIMESTAMP'] = filtered['TIMESTAMP'].astype(float)
        # # TIMESTAMP가 가장 큰 (최신) 행 가져오기
        # if not filtered.empty and 'TIMESTAMP' in filtered.columns:
        #     latest_row = filtered.loc[filtered['TIMESTAMP'].idxmax()].to_dict()
        # else:
        #     latest_row = None  # 혹은 {} 로 반환 가능
        
        # if latest_row is None:
        #     sErrRFID = f"RFID 데이터 에러"
        #     TTSAndroid(sErrRFID,True,1)
        #     rospy.loginfo(sErrRFID)
        # else:            
        #   dicEPCView = latest_row
        #   ts_current = dicEPCView[RFID_RESULT.TIMESTAMP.name]
        #   sEPCCurrent = dicEPCView[MAPFIELD.EPC.name]
        #   node_current = int(dicEPCView[TableInfo.NODE_ID.name])
        #   ts_passed = (getDateTime() - datetime.fromtimestamp(ts_current)).total_seconds()
        #   if ts_passed <= 1:
        #     SetCurrentNode(node_current)
        #     cur_pos=GetEPC_Loc_Master(sEPCCurrent)
        #     iLoc = try_parse_int(cur_pos,MIN_INT)
        #     if iLoc != MIN_INT:
        #       sCUR_POS = iLoc
        #       dicLoc = getMotorLocationSetDic(ModbusID.MOTOR_H.value, sCUR_POS)
        #       SendCMD_Device([dicLoc])
        #   else:
        #     sErrRFID = f"RFID 데이터가 갱신되지 않습니다."
        #     TTSAndroid(sErrRFID,True,1)
        #     rospy.loginfo(sErrRFID)
      
    #모터 위치 정보를 로드하여 반영한다.
    #LoadCurrentPos()
    if IsEnableSvrPath():
      TiltServFinish()
#    print(GetDicRotateMotorTray(270))        
    #print(GetStrArmExtendMain(0,0,0))
    #print(GetDicRotateMotorTray(90))        
    #print(GetLiftControl(True))
    # print(node_CtlCenter_globals.dic_485ex)
    # print(GetDicRotateMotorTray(172))

    #TODO : 모든 모터 정지
    if isRealMachine:
      StopAllMotors()
      #RFIDControl(False)
    curNode = GetCurrentNode()
    cmdpos_H, curpos_H = GetPosServo(ModbusID.MOTOR_H)
    #node_CtlCenter_globals.dicPOS_ABS[str(curNode)] = curpos_H
    # dictmp = getSpeedTableInfo(6,0.6)
    # ACC_LIFT_UP = dictmp['ACC_CCW']
    # print(ACC_LIFT_UP)
    # #setNodeStateEx('101', '0')
    #print(getSeqMap(98, 110))
    bSkip = False
    #TiltDetectingMonitor()
    #TiltDiagonal()
    #TiltFace()
    #이부분을 개조해서 샤누이사 서비스와 내 서비스가 다 살아있는지 확인하고 대기하는 코드 추가
    #TODO : 
    
    # testDic = API_callBLB()
    # print(testDic)
    iCnt5s = 0
    
    
    if IsEnableSvrPath():
      API_SetCurrentNode(nodeID=curNode)
    else:
      UpdateXY_nodeInfo(curNode)
    
    locX,locY=GetLocNodeID(curNode)
    
    if not isRealMachine:
      DoorOpen()
    #locX,locY=GetLocXY()
    # if locX == 0:
    #   dicLoc = getMotorLocationSetDic(ModbusID.MOTOR_H.value, distance_to_pulseH(locY))
    # else:
    #   dicLoc = getMotorLocationSetDic(ModbusID.MOTOR_H.value, distance_to_pulseH(locX))
    # SendCMD_Device([dicLoc])    
    
    # #현재 좌표 확인 및 현 노드에 따른 엔코더값 보정.
    # for nodeTmp, connections in node_CtlCenter_globals.StateInfo.items():
    #   #newDistance = getLinkDistance(nodeTmp,curNode)
    #   #newDirection = getNodeDirection(10,curNode)
    #   listSeqMapOptimized,listSeqMapOrg = getSeqMap(nodeTmp,curNode)
    #   if len(listSeqMapOptimized) == 1:
    #     print(listSeqMapOptimized)
    #     dicSeqInfo = listSeqMapOptimized[0]
    #     newDistance = abs(dicSeqInfo.get(SeqMapField.DISTANCE.name))
    #     nodeDirection = dicSeqInfo.get(SeqMapField.DIRECTION.name)
    #     curPos = 0
    #     #분기기의 위치
    #     locX,locY=GetLocNodeID(nodeTmp)
    #     if nodeDirection == 'N':
    #       locY += newDistance
    #       curPos = locY
    #     elif nodeDirection == 'S':
    #       locY -= newDistance
    #       curPos = locY
    #     elif nodeDirection == 'E':
    #       locX += newDistance        
    #       curPos = locX
    #     elif nodeDirection == 'W':
    #       locX -= newDistance
    #       curPos = locX
    #     else:
    #       print('위치 초기화 에러!')
          
        # dicLoc = getMotorLocationSetDic(ModbusID.MOTOR_H.value, distance_to_pulseH(curPos))
        # SendCMD_Device([dicLoc])
      #print(newDistance,newDirection)
    # data_list = node_CtlCenter_globals.dfDistanceV.to_dict(orient="records")
    # # 리스트를 JSON 문자열로 변환
    # json_string = json.dumps(data_list)
    # pub_DF.publish(json_string)
    while not rospy.is_shutdown():
        try:
            dtNow = getDateTime() 
            # 이전 루틴과 현재시간 사이의 길이
            td_local = dtNow - node_CtlCenter_globals.lastMainLoopTimeStamp
            # listBLB = []  # 경로 지시정보
            # listTable = []  # 순차서빙 테이블 리스트
            # listTable 에 따라 구체적으로 어떤 모터를 어떻게 움직여야 하는지 정의하는 listBLB 가 생성된다.
            lnListBLB = 0
            node_CtlCenter_globals.cntLoop += 1
            #InitCali()
            #ScanWall()
            MotorBalanceControlEx(bSkip)
            #모터 알람 체크 하는 부분 추가
            waitFlag = GetWaitConfirmFlag()
            if not waitFlag:
              GenerateServingTableList()
            RunListBlbMotorsEx(node_CtlCenter_globals.listBLB)
            
            isLiftTrayDown = isLiftTrayDownFinished()
            #curTable,curNode = GetCurrentTable()
            if td_local.total_seconds() > 1:
                iCnt5s += 1
                CheckMotorAlarms()
                # 딕셔너리를 순회하면서 시간을 체크
                if len(node_CtlCenter_globals.dicTTS) > 0 and not isActivatedMotor(ModbusID.MOTOR_V.value):
                    if waitFlag:
                        for scheduled_time, message in list(node_CtlCenter_globals.dicTTS.items()):
                            if dtNow >= scheduled_time:
                                TTSAndroid(message,True,1)
                                #print(f"{current_time}: {message}")
                                node_CtlCenter_globals.dicTTS.pop(scheduled_time, None)
                                #del node_CtlCenter_globals.dicTTS[scheduled_time]
                                if len(node_CtlCenter_globals.dicTTS) == 0:
                                    if IsOrderEmpty():                                      
                                      #오더가 없을시 홈으로 복귀한다.
                                      InsertTableList(HOME_TABLE)
                                      #API_SetOrderHome()
                                    #else:
                                    SetWaitConfirmFlag(False,AlarmCodeList.JOB_COMPLETED)
                                elif len(node_CtlCenter_globals.dicTTS) < 3:
                                  LightTrayCell(TraySector.Cell1.value,LightBlink.Fast.value,LightColor.RED.value)
                                  time.sleep(MODBUS_WRITE_DELAY)
                                  LightTrayCell(TraySector.Cell2.value,LightBlink.Fast.value,LightColor.RED.value)
                                break
                    else:
                        node_CtlCenter_globals.dicTTS.clear()
                # elif waitFlag and isLiftTrayDown and not isActivatedMotor(ModbusID.MOTOR_V.value) and curNode != node_CHARGING_STATION and curNode != node_KITCHEN_STATION:
                #     rospy.loginfo(f'시간이 경과하여 서빙을 취소합니다.')                
                #상태정보 표시하는 함수 추가
                #PrintStatusInfoEverySec(1)
                PrintStatusInfoEverySec(0.97)
                if node_CtlCenter_globals.lastSendStatusList:
                  if isTimeExceeded(node_CtlCenter_globals.lastSendStatus,node_CtlCenter_globals.lastSendStatusDelaySec*1000):
                    node_CtlCenter_globals.lastSendStatus = getDateTime()
                    SendStatus(node_CtlCenter_globals.lastSendStatusList.pop())
                else:
                  blbStatus = getBLBStatus()
                  SendStatus(blbStatus)
                node_CtlCenter_globals.lastMainLoopTimeStamp = getDateTime()
                #배터리 저전압 등 체크는 n초마다 한번씩
                if iCnt5s % 10 == 0:
                  CheckETCActions()
                #TODO 동적 밸런싱 기능 - 중량에 따른 밸런싱암 길이 조절
                # if GetWaitConfirmFlag():z
                #   KeepArmBalancing()
            #bSkip = ControlArco()
        except Exception as e:
            bReturn_ANDROID = False
            msg = traceback.format_exc()
            rospy.loginfo(e)
            SendAlarmHTTP(msg,False)
            #rospy.signal_shutdown(e)
        # rate.sleep()
    rospy.spin()

rospy.loginfo(os.path.splitext(os.path.basename(__file__))[0])