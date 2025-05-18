#!/usr/bin/env python3

from node_CtlCenter_VerifyCmd import *
node_CtlCenter_globals.listBLB.clear()

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

bInitOK = False
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

AppendTableHistory(GetTableTarget())
ReloadSvrTaskList()
LightWelcome(False)

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
      print(TTSServer('스마트폰 통신을 점검해주세요.'))
      raise ValueError(f"안드로이드 통신에러:{node_CtlCenter_globals.BLB_ANDROID_IP}:{BLB_ANDROID_PORT}")
    else:
      rospy.loginfo(f"안드로이드 통신성공:{strResult_ANDROID}")
    if isRealMachine:
      # for p in psutil.process_iter(['name']):
      #   if 'mpv' in p.info['name']:
      #       p.cpu_percent(interval=None)      
      # dynamic_reconfigure 클라이언트 생성
      try:
        node_CtlCenter_globals.dynamic_reconfigure_client = Client(node_CtlCenter_globals.dynamic_reconfigure_clientName, timeout=1)
      except Exception as e:
        rospy.loginfo(f"dynamic_reconfigure_client error : {e}")
      SetLidarCrop(LidarCropProfile.SERVING_ARM)      
      #bReturn_CROSS,strResult_CROSS=API_call_Android(BLB_CROSS_IP_DEFAULT,HTTP_COMMON_PORT,f'svrip={IP_MASTER}')
      node_CtlCenter_globals.bReturn_CROSS,strResult_CROSS=API_call_http(BLB_CROSS_IP_DEFAULT, HTTP_COMMON_PORT, 'control', f'svrip={IP_MASTER}')
      if node_CtlCenter_globals.bReturn_CROSS:
        rospy.loginfo(f"분기기 통신성공:{strResult_CROSS}")
      else:
        rospy.loginfo(f"분기기 통신에러:{node_CtlCenter_globals.BLB_CROSS_IP_DEFAULT}:{HTTP_COMMON_PORT}")        
        print(TTSServer('분기기 통신을 점검해주세요.'))
        raise ValueError(f"분기기 통신에러:{node_CtlCenter_globals.BLB_CROSS_IP_DEFAULT}:{HTTP_COMMON_PORT}")        

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
    while(curpos_H == MIN_INT):
        cmdpos_H, curpos_H = GetPosServo(ModbusID.MOTOR_H)
        rospy.loginfo(f'Checking motor position: {curpos_H}')
        if curpos_H != MIN_INT:
            break
        time.sleep(0.1)
    #개발기에서는 항상 모든 모터 펄스를 0 으로 초기화 한다.
    if not isRealMachine:
        fieldvalue = f'q={BLD_PROFILE_CMD.WLOC_NOT.value}'
        print(API_call_http(GetMasterIP(), HTTP_COMMON_PORT, 'CMD_DEVICE', fieldvalue))
        # pot_int,not_int =GetPotNotServo(ModbusID.TELE_SERV_MAIN)
        # SendCMD_Device([getMotorLocationSetDic(ModbusID.TELE_SERV_MAIN.value, not_int)])
    
    # if isTrue(DI_POT):
    #   dicLoc = getMotorLocationSetDic(ModbusID.MOTOR_H.value, 0)
    #   SendCMD_Device([dicLoc])
    #   TTSAndroid('현재 충전소에 있습니다.',1)
    #   SetCurrentNode(node_KITCHEN_STATION)
    # else:
    dicCurNodeInfo=GetNodeDicFromPos(node_CtlCenter_globals.dfNodeInfo,curpos_H)
    curNodeID_fromPulse = dicCurNodeInfo.get(TableInfo.NODE_ID.name)
    curNode_type = str(dicCurNodeInfo.get(RFID_RESULT.EPC.name))
    diff_pos = dicCurNodeInfo.get(MotorWMOVEParams.DIFF_POS.name)
    curNode_pos = int(dicCurNodeInfo.get(posStr))
    CamControl(False)
    if abs(diff_pos) > roundPulse*10:
      StopEmergency(ALM_User.NODE_NOT_FOUND.value)
    else:
      rospy.loginfo(dicCurNodeInfo)
      TTSAndroid(f'현재 {curNodeID_fromPulse}번 노드에 있습니다.')
      SetCurrentNode(curNodeID_fromPulse)
    TiltServFinish()

    #TODO : 모든 모터 정지
    if isRealMachine:
      StopAllMotors()
    curNode = GetCurrentNode()
    cmdpos_H, curpos_H = GetPosServo(ModbusID.MOTOR_H)
    bSkip = False
    #이부분을 개조해서 샤누이사 서비스와 내 서비스가 다 살아있는지 확인하고 대기하는 코드 추가
    #TODO : 
    iCnt5s = 0
    
    if IsEnableSvrPath():
      API_SetCurrentNode(nodeID=curNode)
    else:
      UpdateXY_nodeInfo(curNode)
    
    locX,locY=GetLocNodeID(curNode)
    
    if not isRealMachine:
      DoorClose()

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
                                TTSAndroid(message,1)
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