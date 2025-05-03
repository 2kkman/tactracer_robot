#!/usr/bin/env python3
from node_CtlCenter_import import *
import node_CtlCenter_globals


def CamControl(enable):
    # onScan = isScanTableMode(GetTableTarget())
    # if onScan:
    #     return
    tagscan = node_CtlCenter_globals.dicARD_CARRIER.get(DataKey.TAGSCAN.name,0)
    isDetectingTag = isTrue(tagscan)
    if isDetectingTag == enable:
      return False, f'cam status:{enable}'
    ttsMsg = 1 if enable else 0
    log_all_frames(f"Trying to start Marker Scan : {enable}")
    return API_call_Android(node_CtlCenter_globals.BLB_ANDROID_IP,BLB_ANDROID_PORT,f'tag={ttsMsg}')
    #rospy.loginfo(f"Trying to start Marker Scan : {enable}")
    camSet = service_setbool_client(ServiceBLB.MarkerScan.value, enable, SetBool)
    rospy.loginfo(f"Start Marker Scan Result : {camSet}")
    return camSet

def ArucoLastRecordSetX(aruco_lastDiff):
  node_CtlCenter_globals.aruco_lastDiffX = aruco_lastDiff

def ArucoLastRecordSetY(aruco_lastDiff):
  node_CtlCenter_globals.aruco_lastDiffY = aruco_lastDiff

def ArucoLastRecordClear():
  node_CtlCenter_globals.aruco_lastDiffX = aruco_lastDiff_Default
  node_CtlCenter_globals.aruco_lastDiffY = aruco_lastDiff_Default

# 새로운 데이터가 들어올 때마다 append
def add_new_obstacle_data(new_data):
    node_CtlCenter_globals.obstacle_history.append(new_data)

def get_obstacle_data(seconds=1.0):
    now = time.time()
    return [data for data in node_CtlCenter_globals.obstacle_history if now - data[MonitoringField.LASTSEEN.name] <= seconds]
  
lastTTS = getDateTime()
def GetRFIDInventoryStatus():
  invStatus = node_CtlCenter_globals.dicInv_last.get(RFID_RESULT.inventoryMode.name)
  return isTrue(invStatus)
       
#@log_arguments
def GetDFTaskChain(lsDicArray):
  try:
    if (type(lsDicArray) != list):
      lsDicArray = json.loads(lsDicArray)
    if len(lsDicArray) == 0:
      return pd.DataFrame()
    df_chainlist = pd.DataFrame(lsDicArray)
    df_chainlist = df_chainlist[pd.to_numeric(df_chainlist[APIBLB_FIELDS_TASK.tasktype.name], errors='coerce').notnull()]
    #필요한 필드는 정수형으로 변환.
    lsIntergerColumns = [APIBLB_FIELDS_TASK.taskid.name, APIBLB_FIELDS_TASK.taskrunok.name, APIBLB_FIELDS_TASK.tasktype]
    if len(df_chainlist) > 0:
      data_list = df_chainlist.to_dict(orient="records")
      json_string = json.dumps(data_list)
      if rospy.core.is_initialized():
        pub_JOBLIST.publish(json_string)
      
      for keytmp in lsIntergerColumns:
        if keytmp in df_chainlist.columns:
          df_chainlist[keytmp] = df_chainlist[keytmp].astype(float).astype(int)
    df_chainlist.to_csv(node_CtlCenter_globals.strCSV_TaskChain, index=False, sep=sDivTab)
    #print(df_chainlist)
    return df_chainlist.drop(df_chainlist[df_chainlist[APIBLB_FIELDS_TASK.taskrunok.name] == 0].index)          
  except Exception as e:
    rospy.loginfo(e)
    sMsg = traceback.format_exc()
    SendAlarmHTTP(sMsg,True,node_CtlCenter_globals.BLB_ANDROID_IP)
    print(lsDicArray)
    return pd.DataFrame()
  
def GetHeightDiffTray():
  angle_y = node_CtlCenter_globals.dicARD_CARRIER.get(DataKey.Angle_Y.name)
  if angle_y is not None:
    height_diff = calculate_relative_height_difference(node_CtlCenter_globals.last_detect_3d,90-angle_y)
    if height_diff is not None:
      #print(f"장애물 내 상대적인 높이 차이: {height_diff:.2f} m")
      node_CtlCenter_globals.last_detect_3d = None

def GetCrossInfo(strKey):
  return node_CtlCenter_globals.dic_CROSSINFO.get(strKey, None)

def SetCrossInfo(strKey, strValue): 
  node_CtlCenter_globals.dic_CROSSINFO[strKey] = strValue

def isChargeSensorOn():
  chargerSensorState = GetCrossInfo(SMARTPLUG_INFO.GPI1_CHARGE.name)
  return isTrue(chargerSensorState)

def isChargerPlugOn():
  chargerPlugState = GetCrossInfo(SMARTPLUG_INFO.CHARGERPLUG_STATE.name)
  return isTrue(chargerPlugState)

def GetLastTableHistory():
  return node_CtlCenter_globals.table_history[-1]

def AppendTableHistory(table:str):
  node_CtlCenter_globals.table_history.append(table)
    
def GetEPCDict(key1, key2):
  return df_to_dict(node_CtlCenter_globals.dfEPCTotal, key1, key2)

def GetEPC_Pos_Info(epcValue):
  #lastEPC = get_last_field_value(node_CtlCenter_globals.dfEPCTotal, MAPFIELD.EPC.name)
  print(node_CtlCenter_globals.dfEPCTotal)
  result_dict = node_CtlCenter_globals.dfEPCTotal[node_CtlCenter_globals.dfEPCTotal[MAPFIELD.EPC.name]==str(epcValue)].tail(1).to_dict(orient='records')
  if len(result_dict) == 0:
    return {}
  else:
    return result_dict[0]

def UpdateEPC_Pos_Info(dicEPC):
  insert_or_update_row_to_df(node_CtlCenter_globals.dfEPCTotal, dicEPC, MonitoringField.CUR_POS.name)

def GetEPCInfo(epcValue): #당분간 사용 안함
  if node_CtlCenter_globals.dfEPCInfo.empty:
    node_CtlCenter_globals.dfEPCInfo = pd.read_csv(node_CtlCenter_globals.strFileEPC_Info, sep=sDivTab)
  result_dict = node_CtlCenter_globals.dfEPCInfo[node_CtlCenter_globals.dfEPCInfo[EPCINFO_FIELDS.TAG_ID.name]==str(epcValue)].tail(1).to_dict(orient='records')
  #log_all_frames(result_dict)
  if len(result_dict) == 0:
    return {}
  else:
    return result_dict[0]
  #return None if len(result_dict) == 0 else result_dict[0].get(APIBLB_FIELDS_INFO.direction.name)

def GetStraightLinks():
  nodes = find_dead_end_nodes_from_file(node_CtlCenter_globals.strFileShortCut)
  edges = list(itertools.combinations(nodes, 2))
  dicLinkInfo = {}
  for edge in edges:
    startNode = edge[0]
    endNode = edge[1]
    linkKey = GetLinkKey(startNode,endNode)
    listSeqMapOptimized,listSeqMapOrg = getSeqMap(startNode,endNode)
    if len(listSeqMapOptimized) == 1:
      dicLinkInfo[linkKey] = listSeqMapOrg
  print(dicLinkInfo)
  return dicLinkInfo

def AppendSendStatus(SendStatus : BLB_STATUS_FIELD):
  node_CtlCenter_globals.lastSendStatusList.insert(0, SendStatus)

def isLinkHorizon(linkKey):
    result = []
    for key, values in node_CtlCenter_globals.StateInfo.items():
        for value in values:
          result.append(f"{key}{value}")
    if Counter(linkKey) == Counter(result[0]) or Counter(linkKey) == Counter(result[1]):
      return False
    return True

def getLinkDistance(startNode, endNode):
    with open(strFileShortCut, 'r') as file:
        for line in file:
            # 각 줄을 공백 기준으로 나누어 노드1, 노드2, 거리로 분리
            parts = line.strip().split(sDivTab)
            if len(parts) != 3:
                continue  # 각 줄은 노드1, 노드2, 거리 형태여야 함
            
            node1, node2, distance = int(parts[0]), int(parts[1]), int(parts[2])
            if node1 == startNode and node2 == endNode:
              return distance
            if node2 == startNode and node1 == endNode:
              return distance
    return -1

def getNodeDirection(startNode,endNode=None):
  bResult,recvDataMap=API_robot_node_info()
  dfReceived = pd.DataFrame(recvDataMap)
  if endNode is None:
    result_dict = dfReceived[dfReceived[APIBLB_FIELDS_INFO.start.name]==int(startNode)].tail(1).to_dict(orient='records')
  else:
    result_dict = dfReceived[(dfReceived[APIBLB_FIELDS_INFO.start.name]==int(startNode)) & (dfReceived[APIBLB_FIELDS_INFO.end.name]==int(endNode))].tail(1).to_dict(orient='records')
  #result_dict = dfReceived[dfReceived[APIBLB_FIELDS_INFO.end.name]==nodeID].tail(1).to_dict(orient='records')
  #print(dfReceived)
  #print(result_dict)
  return None if len(result_dict) == 0 else result_dict[0].get(APIBLB_FIELDS_INFO.direction.name)

def getLinkDirection(linkKey):
    result = []
    for key, values in node_CtlCenter_globals.StateInfo.items():
        for value in values:
          result.append(f"{key}{value}")
    print(linkKey)
    if linkKey == result[0]:
      return 'N'
    if linkKey == result[1]:
      return 'S'
    if linkKey == result[2]:
      return 'E'
    if linkKey == result[3]:
      return 'W'
    if Counter(linkKey) == Counter(result[0]):
      return 'S'
    if Counter(linkKey) == Counter(result[1]):
      return 'N'
    if Counter(linkKey) == Counter(result[2]):
      return 'W'
    if Counter(linkKey) == Counter(result[3]):
      return 'E'
    return None


def getSimplePath2(lsReturn, bScanMode = False):
    if len(lsReturn) == 1:
        startDic2 = lsReturn.pop(0)
        curDicStartNode = startDic2[SeqMapField.START_NODE.name]
        curDicEndNode = startDic2[SeqMapField.END_NODE.name]
        
        linkIDFirst = f'{curDicStartNode}_{curDicEndNode}'
        if linkIDFirst in node_CtlCenter_globals.node_seq:
          startDic2[SeqMapField.DIRECTION.name] = 'E'
          startDic2[APIBLB_FIELDS_INFO.direction.name] = 'E'
        else:
          startDic2[SeqMapField.DIRECTION.name] = 'W'
          startDic2[APIBLB_FIELDS_INFO.direction.name] = 'W'
        lsReturn.append(startDic2)
        rospy.loginfo(lsReturn)
        return lsReturn

    dicDirectionInfo = {}
    for startDic2 in lsReturn:
      curDicStartNode = startDic2[SeqMapField.START_NODE.name]
      curDicEndNode = startDic2[SeqMapField.END_NODE.name]
      linkIDFirst = f'{curDicStartNode}_{curDicEndNode}'
      strDirection = 'S'
      if linkIDFirst in node_CtlCenter_globals.node_seq:
          strDirection = 'N'

      dicDirectionInfo[curDicStartNode] = strDirection
      
    
    lsFinal2 = []
    lsDirection = []
    dicCrossState = {}
    startDic2 = lsReturn.pop(0)
    curDicStartNode = startDic2[SeqMapField.START_NODE.name]
    startX, startY = GetLocNodeID(curDicStartNode)
    curDicEndNode = startDic2[SeqMapField.END_NODE.name]
    endX, endY = GetLocNodeID(curDicEndNode)
    direction, distance = calculate_direction_and_distance(startX, startY, endX, endY)
    cardinal_direction = get_cardinal_direction(direction)
    linkID = f'{curDicStartNode}{curDicEndNode}'
    linkIDFirst = f'{curDicStartNode}_{curDicEndNode}'
    #dirCur = getLinkDirection(linkID)
    #if dirCur is not None:
    if linkIDFirst in node_CtlCenter_globals.node_seq:
      lsDirection.append('E')
    else:
      lsDirection.append('W')
    
    startDic2StartStatus = startDic2[SeqMapField.START_STATUS.name]
    startDic2EndStatus = startDic2[SeqMapField.END_STATUS.name]
    dicCrossState[curDicStartNode] = startDic2StartStatus
    dicCrossState[curDicEndNode] = startDic2EndStatus
    
    while len(lsReturn) > 0:
        curDic = lsReturn.pop(0)
        curDicStartNode = curDic[SeqMapField.START_NODE.name]
        curDicStartStatus = curDic[SeqMapField.START_STATUS.name]
        curDicEndNode = curDic[SeqMapField.END_NODE.name]
        curDicEndStatus = curDic[SeqMapField.END_STATUS.name]
        dicCurNodeState = {}
        dicCurNodeState[curDicStartNode] = curDicStartStatus
        dicCurNodeState[curDicEndNode] = curDicEndStatus
        bSplit = False
        linkID = f'{curDicStartNode}{curDicEndNode}'
        # dirCur = getLinkDirection(linkID)
        # if dirCur is not None:
        #   lsDirection.append(dirCur)
        
        for nodeTmp, nodeStatus in dicCurNodeState.items():
            # print(nodeTmp, nodeStatus)
            # print(dicCrossState)
            paramedStatus = dicCrossState.get(nodeTmp, -1)
            if paramedStatus >= 0 and (paramedStatus != nodeStatus or bScanMode):
                bSplit = True
                startDic2[SeqMapField.CROSS_STATUS.name] = {
                    k: v for k, v in dicCrossState.items() if v != -1
                }
                # if dirCur is not None:
                #   startDic2[SeqMapField.DIRECTION.name] = dirCur


            if paramedStatus == -1:
                dicCrossState[nodeTmp] = nodeStatus

        if bSplit:
            linkIDFirst = f'{curDicStartNode}_{curDicEndNode}'
            lsFinal2.append(startDic2.copy())
            dicCrossState.clear()
            curDic[SeqMapField.CROSS_STATUS.name] = {
                k: v for k, v in dicCurNodeState.items() if v != -1
            }
            dicCrossState = dicCurNodeState
            startDic2 = curDic.copy()
            # lsFinal2.append(curDic)
        else:
            if len(startDic2) > 0:
                startDic2[SeqMapField.DISTANCE.name] += curDic[
                    SeqMapField.DISTANCE.name
                ]
                startDic2[SeqMapField.END_NODE.name] = curDic[SeqMapField.END_NODE.name]
            else:
                startDic2 = curDic
        
        if linkIDFirst in node_CtlCenter_globals.node_seq:
          lsDirection.append('E')
        else:
          lsDirection.append('W')
                
        if len(lsReturn) == 0:
            if len(dicCrossState) > 0:
                startDic2[SeqMapField.CROSS_STATUS.name] = {
                    k: v for k, v in dicCrossState.items() if v != -1
                }
            lsFinal2.append(startDic2.copy())
            dicCrossState.clear()
    #lsDirection = remove_consecutive_duplicates(lsDirection)
    for i in range(len(lsFinal2)):      
      startDic2 = lsFinal2[i]
      curDicStartNode = startDic2[SeqMapField.START_NODE.name]
      strDir = dicDirectionInfo[curDicStartNode]
      lsFinal2[i][SeqMapField.DIRECTION.name] = strDir
      lsFinal2[i][APIBLB_FIELDS_INFO.direction.name] = strDir
      
      # curDicEndNode = startDic2[SeqMapField.END_NODE.name]
      
      # linkIDFirst = f'{curDicStartNode}_{curDicEndNode}'
      # if linkIDFirst in node_CtlCenter_globals.node_seq:
      #   lsFinal2[i][SeqMapField.DIRECTION.name] = 'N'
      #   lsFinal2[i][APIBLB_FIELDS_INFO.direction.name] = 'N'
      # else:
      #   lsFinal2[i][SeqMapField.DIRECTION.name] = 'S'
      #   lsFinal2[i][APIBLB_FIELDS_INFO.direction.name] = 'S'
      
      # if len(lsDirection) == len(lsFinal2):
        
      #   lsFinal2[i][SeqMapField.DIRECTION.name]=lsDirection[i]
      #   lsFinal2[i]['direction']=lsDirection[i]
      # else:
      #   lsFinal2[i][SeqMapField.DIRECTION.name] = cardinal_direction
        # CROSS_STATUS 값 확인 및 반환
        # cross_status_value = list(lsFinal2[0]['CROSS_STATUS'].values())[0]  # dict의 첫 번째 값 가져오기
        # result = bool(cross_status_value)  # 1이면 True, 0이면 False로 변환
        # if result:
        #   lsFinal2[i][SeqMapField.DIRECTION.name] = 'E' if lsFinal2[i][SeqMapField.DISTANCE.name] > 0 else 'W'
        # else:
        #   lsFinal2[i][SeqMapField.DIRECTION.name] = 'N' if lsFinal2[i][SeqMapField.DISTANCE.name] > 0 else 'S'
    #   dicPathTmp = lsFinal2[i]
    #   startnode_tmp = dicPathTmp[SeqMapField.START_NODE.name]
    #   endnode_tmp = dicPathTmp[SeqMapField.END_NODE.name]
    #   linkID_tmp = getLinkDirection(f'{startnode_tmp}{endnode_tmp}')
    #   dicPathTmp[SeqMapField.DIRECTION.name] = linkID_tmp
      
    #print(f"2차 필터링 : {lsFinal2}")
    return lsFinal2

def LoadMapRefresh():
  node_CtlCenter_globals.graph, node_CtlCenter_globals.bgraphOK ,node_CtlCenter_globals.lsTotalMap,node_CtlCenter_globals.node_location,node_CtlCenter_globals.node_seq = node_CtlCenter_globals.LoadMap()

def GetRackID_Empty():
  count_zero = sum(1 for x in node_CtlCenter_globals.lsRackStatus if int(x) < WEIGHT_OCCUPIED)
  if count_zero == 0:
    return MIN_INT,0
  first_index_zero = next((i for i, val in enumerate(node_CtlCenter_globals.lsRackStatus) if int(val) < WEIGHT_OCCUPIED), None)
  #first_index_zero = node_CtlCenter_globals.lsRackStatus.index(0) if 0 in node_CtlCenter_globals.lsRackStatus else None
  return RackID.from_name_or_value(first_index_zero,False),count_zero

def SetRackIDStatus(idx, val):
  try:
    if type(idx) == str:
      rackIDInstance = RackID.from_name_or_value(idx,True)
      idxNew = rackIDInstance.value
    node_CtlCenter_globals.lsRackStatus[idxNew] = val
  except Exception as e:    
    message = traceback.format_exc()
    rospy.loginfo(message)

def GetRackIDStatus(idx):
  if type(idx) == str:
    idx = RackID.from_name_or_value(idx,True).value
  return node_CtlCenter_globals.lsRackStatus[idx]

def IsEnableSvrPath():
    lsReturn = node_CtlCenter_globals.enableSvrPath
    return lsReturn

def ReloadSvrTaskList():
  if IsEnableSvrPath():
    bReturn,strResult = API_GetTaskList()
    if bReturn:
      df = GetDFTaskChain(strResult)
      print(df)
      if not df.empty:
        df = df[pd.to_numeric(df[APIBLB_FIELDS_TASK.tasktype.name], errors='coerce').notnull()]
        #df = df[pd.to_numeric(df[APIBLB_FIELDS_TASK.tasktype.name], errors="coerce").notna()]
        node_CtlCenter_globals.dfTaskChainInfo = df
        PrintDF(node_CtlCenter_globals.dfTaskChainInfo)
        #df.to_csv(strCSV_TaskChain, index=False, sep=sDivTab) 
        #print(node_CtlCenter_globals.dfTaskChainInfo)              
  

def GetLocNodeID(nodeID):
    strnodeID = round(try_parse_int(nodeID))
    try:
      lsReturn = node_CtlCenter_globals.node_location[strnodeID]
      return lsReturn[0],lsReturn[1]
    except:
      return 0,0

def GetRangeV(value=None, column_input="", column_output=None, df = node_CtlCenter_globals.dfDistanceV):
    return GetRange(value, column_input, column_output, df)
# def GetRangeV(pulseV=None, column_input=DISTANCE_V.pulseV.name, column_output=DISTANCE_V.distanceLD.name):
#     df = node_CtlCenter_globals.dfDistanceV

#     # 컬럼 존재 여부 확인
#     if column_input not in df.columns or column_output not in df.columns:
#         return -1

#     if pulseV is None:
#         return df[column_output].max()

#     # pulseV가 데이터의 범위를 벗어나면 경계값 반환
#     min_pulse, max_pulse = df[column_input].min(), df[column_input].max()
#     if pulseV <= min_pulse:
#         return df[column_output].iloc[0]
#     if pulseV >= max_pulse:
#         return df[column_output].iloc[-1]

#     # 가장 가까운 두 개의 점을 찾기
#     lower_idx = df[column_input].searchsorted(pulseV, side='right') - 1
#     upper_idx = lower_idx + 1

#     # 데이터 범위 확인
#     if upper_idx >= len(df):
#         return df[column_output].iloc[lower_idx]

#     x0, y0 = df[column_input].iloc[lower_idx], df[column_output].iloc[lower_idx]
#     x1, y1 = df[column_input].iloc[upper_idx], df[column_output].iloc[upper_idx]

#     # 선형 보간 공식 적용
#     interpolated_value = y0 + (y1 - y0) * (pulseV - x0) / (x1 - x0)

#     return interpolated_value

# def GetRangeV(pulseV=None, column_input=DISTANCE_V.pulseV.name, column_output=DISTANCE_V.distanceLD.name):
#     dfTmp = node_CtlCenter_globals.dfDistanceV

#     # 키가 존재하지 않으면 -1 반환
#     if column_input not in dfTmp.columns or column_output not in dfTmp.columns:
#         return -1

#     if pulseV is None:
#         return dfTmp[column_output].max()

#     closest_value_idx = (dfTmp[column_input] - pulseV).abs().idxmin()
#     return dfTmp.loc[closest_value_idx, column_output]

def GetMaxV():
    return GetRangeV()

def GetLiftCurPositionDown(cur_pos_lift = None):
    if cur_pos_lift == None:
        pot_cur_lift,not_cur_lift ,cmdpos_lift,cur_pos_lift =GetPotNotCurPosServo(ModbusID.MOTOR_V)
    maxV = GetMaxV()    #리프트 NOT 지점에서의 지면까지의 거리
    cur_distance = GetRangeV(cur_pos_lift,DISTANCE_V.pulseV.name,None, node_CtlCenter_globals.dfDistanceV)
    return round(maxV - cur_distance,3)

def GetLiftTargetPulseAruco(aruco_Z = None):
    try:
      if aruco_Z == None:
          lsAruco = filter_recent_data(ARUCO_RESULT_FIELD.LASTSEEN.name,GetArucoMarkerInfo(None),0.5)
          if len(lsAruco) > 0:
              #TODO : 여기서 KEY 에러 나는거 수정해보자
              aruco_Z = lsAruco[-1][ARUCO_RESULT_FIELD.Z.name]
          else:
              return -2
      targetPulse = GetRangeV(aruco_Z,DISTANCE_V.aruco_Z.name,DISTANCE_V.pulseV.name,node_CtlCenter_globals.dfDistanceV)
      return targetPulse
    except Exception as e:
      message = traceback.format_exc()
      rospy.loginfo(message)
      print(aruco_Z)

def GetLiftCurPositionAruco(aruco_Z = None):
    #카메라 높이 추정 수치 보정 마진 = 5~10cm 이지만 안전하게 5cm
    cam_margin = 0.025
    targetP = GetLiftTargetPulseAruco(aruco_Z)
    target_tmp = GetRangeV(targetP)
    target_height =round(GetMaxV() - target_tmp - cam_margin,3)
    targetPulse = GetRangeV(target_height,DISTANCE_V.distanceLD.name,DISTANCE_V.pulseV.name)
    return target_height, targetPulse
    
def IsOrderEmpty():
  condition_A = len(node_CtlCenter_globals.listBLB)
  condition_B =len(GetTableList())
  condition_C =len(getRunningMotorsBLB())
  if IsEnableSvrPath():
    dicTask = GetTaskChainHead(APIBLB_FIELDS_TASK.taskrunok.name, 1, False)
    condition_B =len(dicTask)
  if condition_A == 0 and  condition_B== 0 and condition_C == 0:
    TTSAndroid('작업이 없습니다.')
    return True
  return False

def SetDynamicConfigROS(new_params : dict):
    return
    if node_CtlCenter_globals.dynamic_reconfigure_client is not None:
        node_CtlCenter_globals.dynamic_reconfigure_client.update_configuration(new_params)

def GetDynamicConfigROS():
    if node_CtlCenter_globals.dynamic_reconfigure_client is not None:
        # 파라미터 값을 가져와서 수정
        params = node_CtlCenter_globals.dynamic_reconfigure_client.get_configuration()
        #rospy.loginfo("Current Parameters: {0}".format(params))
        return params
    return {}
  
def SetLidarCrop(pf : LidarCropProfile):
  return
  if not isRealMachine:
    return
  log_all_frames()
  dicSetDetectingMonitorMode = node_CtlCenter_globals.dicLidarCropProfile[pf]
  SetDynamicConfigROS(dicSetDetectingMonitorMode)

def GetWaitCrossFlag():
  return node_CtlCenter_globals.waitCross

def SetWaitCrossFlag(flag_bool):
  node_CtlCenter_globals.waitCross = flag_bool

def SetCurrentNode(flag_bool):
  result = try_parse_int(flag_bool,MIN_INT)
  sMsg = log_all_frames(flag_bool)
  rospy.loginfo(sMsg)
  if result == MIN_INT:
    SendAlarmHTTP(sMsg,True,node_CtlCenter_globals.BLB_ANDROID_IP)
  else:
    node_CtlCenter_globals.node_current = result
    
  sEPC = get_key_by_value(node_CtlCenter_globals.EPCNodeInfo, result)
  
  if not isRealMachine and sEPC is not None:
    dictEPC = { MAPFIELD.EPC.name : sEPC }
    API_public_strtopic(f'/{TopicName.RFID.name}', dictEPC)
  if result != MIN_INT and sEPC is not None:
    SendInfoHTTP(sMsg + sEPC)

def GetCurrentNode():
  return node_CtlCenter_globals.node_current

def SetTableTarget(tableNo):
  sMsg = log_all_frames(logmsg=tableNo,max_frames=5)
  rospy.loginfo(sMsg)
  if tableNo == MIN_INT:
    return
  
  if node_CtlCenter_globals.table_target != tableNo:
    df = GetDF(node_CtlCenter_globals.table_target)
    if df is None:
      rospy.loginfo(f'Table - {node_CtlCenter_globals.table_target} not Found.')
    else:
      PrintDF(df)
  
  node_CtlCenter_globals.table_target = tableNo
  SendInfoHTTP(sMsg)

def GetTableTarget():
  return node_CtlCenter_globals.table_target

def GetDictFromEnum(objEnum):
    return {objEnum.value :objEnum.name}

def GetWaitConfirmFlag():
  return node_CtlCenter_globals.flag_WaitConfirm

def SetWaitConfirmFlag(flag_bool, reason):
    if flag_bool:
      node_CtlCenter_globals.flag_WaitConfirm =flag_bool
      
    lsCurTable,curNode = GetCurrentTableNode()
    table_target = GetTableTarget()
    dfReceived = GetDF(table_target)
    if dfReceived is not None or reason != AlarmCodeList.JOB_COMPLETED:
      if GetWaitConfirmFlag() == flag_bool and flag_bool:
        return

    if dfReceived is None and IsEnableSvrPath() and GetWaitConfirmFlag() == flag_bool:
      return
      
    sMsg = f"reason={reason},tabletarget={table_target},curnode:{curNode},curTable:{lsCurTable},Flag:{flag_bool},{sys._getframe(0).f_code.co_name}:{sys._getframe(1).f_code.co_name}:{sys._getframe(2).f_code.co_name}"
    rospy.loginfo(sMsg)
    STATUS_TASK=APIBLB_STATUS_TASK.Completed
    if reason == AlarmCodeList.JOB_COMPLETED:
      tableValue = try_parse_int(table_target)
      table_targetNew = table_target
      if tableValue > 0:
        try:
          table_targetNew = f'T{table_target}'
          dicTaskInfo = GetTaskChainHead(APIBLB_FIELDS_TASK.workname.name, table_targetNew, True)
          trayrack = dicTaskInfo.get(APIBLB_FIELDS_TASK.trayrack.name)
          tasktype = dicTaskInfo.get(APIBLB_FIELDS_TASK.tasktype.name)
          
          if trayrack is not None and tasktype == str(APIBLB_TASKTYPE.ServingTask.value):
            trayRackID = RackID.from_name_or_value(trayrack,True)
            trayidx = trayRackID.value
            tray_weight = int(GetRackIDStatus(trayidx))
            if tray_weight > WEIGHT_OCCUPIED:
              STATUS_TASK = APIBLB_STATUS_TASK.NONE 
        except Exception as e:
          print(e) 
    #log_all_frames(flag_bool)
    reasonDict = reason
    if type(reason) != dict:
        reasonDict = GetDictFromEnum(reason)
        
    node_CtlCenter_globals.dict_WaitReason = reasonDict
    if not flag_bool and len(node_CtlCenter_globals.dicTTS) > 0:
        node_CtlCenter_globals.dicTTS.clear()
    
    #if table_target == curTable and not flag_bool and IsEnableSvrPath() and reason == AlarmCodeList.JOB_COMPLETED:
    if not flag_bool and IsEnableSvrPath() and reason == AlarmCodeList.JOB_COMPLETED:
      #dfReceived = GetDF(curTable)
      if dfReceived is not None:
        # dfReceived[APIBLB_FIELDS_TASK.workstatus.name] = APIBLB_STATUS_TASK.Completed.value
        # dfReceived[APIBLB_FIELDS_TASK.orderstatus.name] = APIBLB_STATUS_TASK.Completed.value
        dfReceived[APIBLB_FIELDS_TASK.workstatus.name] = STATUS_TASK.value
        dfReceived[APIBLB_FIELDS_TASK.orderstatus.name] = STATUS_TASK.value
        PrintDF(dfReceived)
        dicLast = dfReceived.iloc[-1]
        
        taskid_current = int(dicLast[APIBLB_FIELDS_TASK.taskid.name])
        tableid_current = dicLast[APIBLB_FIELDS_TASK.endnode.name]
        table_target_tts = table_target if try_parse_int(table_target, MIN_INT) == MIN_INT else f'T{table_target}'
        if table_target in lsCurTable:
          if STATUS_TASK == APIBLB_STATUS_TASK.Completed:
            TTSAndroid(f'{table_target_tts}완료')
          else:
            TTSAndroid(f'{table_target_tts}실패')
          #time.sleep(1)
        else:
          TTSAndroid(f'현재 테이블 {lsCurTable}, 목적지는 {table_target_tts}입니다')
        data_list = dfReceived.to_dict(orient="records")
        json_string = json.dumps(data_list)
        pub_DF.publish(json_string)
        node_CtlCenter_globals.lock.acquire()
        AppendTableHistory(tableid_current)        
        resultAPI, nodeReturn = API_robot_navigation_info(dfReceived,STATUS_TASK)
        node_CtlCenter_globals.dfLast = copy.deepcopy(dfReceived)
        print(SetTaskCompleted(taskid_current))
        #RemoveDF(curTable)
        RemoveDF()        
        if nodeReturn is None:
            TTSAndroid('리턴 노드값이 정상적이지 않습니다')
            SendAlarmHTTP(str(resultAPI), False, node_CtlCenter_globals.BLB_ANDROID_IP)
            rospy.loginfo(resultAPI)   
        else:
            SetCurrentNode(nodeReturn)
        node_CtlCenter_globals.lock.release()
      else:
        TTSAndroid('테이블 값이 없습니다')
        SendInfoHTTP(sMsg)
    node_CtlCenter_globals.flag_WaitConfirm =flag_bool
        

def SetWaitConfirmStart(flag_bool, reason,delaySeconds):
    log_all_frames(reason)
    # delaySeconds초 대기
    time.sleep(delaySeconds)
    SetWaitConfirmFlag(flag_bool,reason)

def GetTorqueData(mbid):
    tMax = node_CtlCenter_globals.dicTorqueMax.get(mbid,-1)
    tAve = node_CtlCenter_globals.dicTorqueAve.get(mbid,-1)
    oMax = node_CtlCenter_globals.dicOvrMax.get(mbid,-1)
    oAve = node_CtlCenter_globals.dicOvrAve.get(mbid,-1)
    return tMax,tAve,oMax,oAve

def SetTorqueData(mbid, tMax,tAve, oMax,oAve):
    iMax = int(tMax)
    iAve = int(tAve)
    if iMax < 0 or iAve < 0:
        return
    node_CtlCenter_globals.dicTorqueMax[mbid] = tMax
    node_CtlCenter_globals.dicTorqueAve[mbid] = tAve
    node_CtlCenter_globals.dicOvrMax[mbid] = oMax
    node_CtlCenter_globals.dicOvrAve[mbid] = oAve

def GetCurrentTargetTable():
  table_target = GetTableTarget()
  
  node_target=GetNodeFromTable(table_target)
  return table_target,node_target
  # if len(node_CtlCenter_globals.lsHistory_motorH) > 0:
  #   dicMotorH = node_CtlCenter_globals.lsHistory_motorH[-1]
  #   # start_node = dicMotorH[SeqMapField.START_NODE.name]
  #   # distanceTmp = dicMotorH[SeqMapField.DISTANCE.name]
  #   end_node = dicMotorH[SeqMapField.END_NODE.name]
  #   return GetTableFromNode(end_node), end_node
  # else:
  #   #return HOME_TABLE,node_KITCHEN_STATION
  #   return HOME_TABLE,node_KITCHEN_STATION
  
def GetCurrentTableNode():
    node_current = GetCurrentNode()
    if node_current is None:
      print('Node error')
    end_node = node_current
    return GetTableFromNode(end_node), end_node
  
def GetTargetTableNode():
    if len(node_CtlCenter_globals.lsNodeHistory) > 0:
      toReturn = node_CtlCenter_globals.lsNodeHistory[-1]
      return toReturn[0],toReturn[1]
    else:
      return -1,-1
    
def IsSuspendedJob():
  curTargetTable,curTarNode = GetCurrentTargetTable()
  endnodeStr= GetTableIDFromNumber(curTargetTable)
  dfReceived = GetDF(curTargetTable)
  if dfReceived is None or dfReceived.empty:
    dicTaskInfo = GetTaskChainHead(APIBLB_FIELDS_TASK.workname.name, endnodeStr, False)
    taskStatus = dicTaskInfo.get(APIBLB_FIELDS_TASK.taskrunok.name,0)
    if taskStatus == 2:
      return False
    return GetWaitConfirmFlag()
  lsPaused = dfReceived[dfReceived[APIBLB_FIELDS_NAVI.workstatus.name] == APIBLB_STATUS_TASK.Paused.value].tail(1).to_dict(orient='records')
  if len(lsPaused) > 0:
      return True
  return False
  

# def GetTableFromNode(curNodeSt):
#     if curNodeSt is not None:
#       curNodeint = int(curNodeSt)
#       file_path = node_CtlCenter_globals.strFileTableNodeEx
#       if isFileExist(file_path):
#         df_manager = DataFrameManager(file_path)
#         if curNodeint is not None:
#             curTableList = df_manager.filter_by_key(TableInfo.NODE_ID.name, curNodeint)
#             if len(curTableList) == 0:
#                 return None
#             else:
#                 curTable = curTableList.iloc[0][TableInfo.TABLE_ID.name]
#                 return curTable
#     return None

def SetCurrentTable(curTable=None, curNode=None):
    #isOK = True
    file_path = strFileTableNodeEx
    if not isFileExist(file_path):
      if len(node_CtlCenter_globals.lsNodeHistory) == 0:
        node_CtlCenter_globals.lsNodeHistory.append([node_KITCHEN_STATION,node_KITCHEN_STATION])
      return None
    
    df_manager = DataFrameManager(file_path)
    if curTable is None and curNode is None:
        curNode = GetCurrentNode()

    if curTable is not None:
        curNodeList = df_manager.filter_by_key(TableInfo.TABLE_ID.name, str(curTable))
        if len(curNodeList) == 0:
            return None
        curNode = int(curNodeList.iloc[0][TableInfo.NODE_ID.name])
    elif curNode is not None:
        curTableList = df_manager.filter_by_key(TableInfo.NODE_ID.name, curNode)
        if len(curTableList) == 0:
            return None
        else:
          print(curTableList)
        curTable = curTableList.iloc[0][TableInfo.TABLE_ID.name]
    #print(type(curNodeList))
    #print(curNodeList.columns)
    #curNode = int(curNodeList.iloc[0][TableInfo.NODE_ID.name])
    #print(type(curNode))
    if len(node_CtlCenter_globals.lsNodeHistory) == 0 or node_CtlCenter_globals.lsNodeHistory[-1] != [curTable,curNode]:
      node_CtlCenter_globals.lsNodeHistory.append([curTable,curNode])
    curX,curY = GetLocNodeID(curNode)
    #UpdateXY(curX,curY)
    return curNode
    
def GetTiltStatus():
  angle = node_CtlCenter_globals.dicARD_CARRIER.get(DataKey.Angle_Y.name, None)
  if angle is None:
    return node_CtlCenter_globals.tiltStatus
  # 모든 상태 값의 리스트
  tilt_values = np.array([status.value for status in TRAY_TILT_STATUS])
  
  # 가장 가까운 값 찾기
  closest_idx = (np.abs(tilt_values - angle)).argmin()
  
  # 해당 인덱스의 Enum 멤버 반환
  rs = list(TRAY_TILT_STATUS)[closest_idx]  
  return rs

def GetArucoMarkerInfo(tableNo=None):
    tableNo = try_parse_int(tableNo)
    if tableNo == 0:
      tableNo = None
    arucoInfo = node_CtlCenter_globals.dicARUCO_Result.get(tableNo, [])
    if tableNo is None:
    # dicARUCO_Result의 모든 값들을 병합한 리스트로 반환
        arucoInfo = sum(node_CtlCenter_globals.dicARUCO_Result.values(), [])
    return arucoInfo

def GetArucoMarkerDict():
    return node_CtlCenter_globals.dicARUCO_Result

def ClearArucoTable():
    #rospy.loginfo(f"{sys._getframe(1).f_code.co_name}:{sys._getframe(2).f_code.co_name}")
    log_all_frames()
    node_CtlCenter_globals.dicARUCO_Result.clear()

def AppendArucoTable(tableNo, reason):
    #rospy.loginfo(f"{sys._getframe(1).f_code.co_name}:{sys._getframe(2).f_code.co_name}")
    log_all_frames()
    if node_CtlCenter_globals.dicARUCO_Result.get(tableNo) is None:
      node_CtlCenter_globals.dicARUCO_Result[tableNo] = []
    
    if type(reason) != list:
      node_CtlCenter_globals.dicARUCO_Result[tableNo].append(reason)
    else:
      node_CtlCenter_globals.dicARUCO_Result[tableNo].extend(reason)
      
    node_CtlCenter_globals.dicARUCO_Result.clear()


def GetLastCmdTimeStamp():
  return node_CtlCenter_globals.lastCmdTimeStamp

def UpdateLastCmdTimeStamp():
  node_CtlCenter_globals.lastCmdTimeStamp = getDateTime()

def GetFirstnodeFromDF(df):
  return GetIdxnodeFromDF(df,0,APIBLB_FIELDS_TASK.startnode)

def GetEndnodeFromDF(df):
  return GetIdxnodeFromDF(df,-1,APIBLB_FIELDS_TASK.endnode)

def GetIdxnodeFromDF(df, id, fieldName : APIBLB_FIELDS_TASK):
  # df에 존재하는 컬럼만 필터링
  columns_to_keep = [field.name for field in APIBLB_FIELDS_TASK if field.name in df.columns]
  # 필터링된 컬럼만으로 DataFrame을 선택
  df = df[columns_to_keep]
  #df_manager = DataFrameManager(df)
  dicTmp = df.iloc[id]
  endnodeStr = dicTmp[fieldName.name]
  return endnodeStr
  
def GetTableIDFromStr(endnodeStr):
    nodeID = try_parse_int(endnodeStr, MIN_INT)
    if endnodeStr.startswith('H'):
      return endnodeStr
    if nodeID == MIN_INT:
      nodeID = endnodeStr[1:]  
      return nodeID  
    else:
      return None
  
def GetTableIDFromNumber(curTargetTable):
    tableNo = try_parse_int(curTargetTable,MIN_INT)
    if tableNo == MIN_INT:
        tableNo =curTargetTable
    else:
        tableNo = f'T{curTargetTable}'
    return tableNo


def SetDF(tableID, dfTmp):
  #if len(GetTableList()) == 0 or GetTableList()[-1] != tableID:
    #AppendTableList(tableID)
    node_CtlCenter_globals.dic_DF[str(tableID)] = dfTmp
  
def GetDF(tableID=None)-> Optional[pd.DataFrame]:
  if tableID is None:
    tableID = GetTableTarget()
  return node_CtlCenter_globals.dic_DF.get(str(tableID))

def RemoveDF(tableID=None):
  SendInfoHTTP(log_all_frames())
  if tableID == None:
    node_CtlCenter_globals.dic_DF.clear()  
  else:
    node_CtlCenter_globals.dic_DF.pop(str(tableID))

def GetTaskChain(taskKey, val,filtering):
  df = node_CtlCenter_globals.dfTaskChainInfo
  try:
      dfFiltered = df[df[taskKey] == val]
      if filtering:
      #if taskKey != APIBLB_FIELDS_TASK.taskrunok:
        dfFiltered = dfFiltered.drop(dfFiltered[dfFiltered[APIBLB_FIELDS_TASK.taskrunok.name] != 1].index)                
  except:
      # 'key' 열이 없으면 빈 DataFrame 반환
      dfFiltered = pd.DataFrame()
  # print(dfFiltered)  
  # dfFiltered = df[df[taskKey] == val]
  return dfFiltered.to_dict(orient='records')

def SetTaskCompleted(taskid, value=2):
  sMsg = log_all_frames(logmsg=taskid,max_frames=5)
  SendInfoHTTP(sMsg)
  if node_CtlCenter_globals.robot.get_current_state() == Robot_Status.paused:
    node_CtlCenter_globals.robot.trigger_resume_serving()
    
  if node_CtlCenter_globals.robot.get_current_state() == Robot_Status.onServing:
    node_CtlCenter_globals.robot.trigger_complete_serving()
  else:
    SendInfoHTTP(f'State:{node_CtlCenter_globals.robot.get_current_state()},{sMsg}')
    TTSAndroid("서빙상태 정보가 맞지 않습니다.")
  itaskid = int(taskid)
  try:
    # 필요한 열이 존재하는지 먼저 확인
    required_columns = [APIBLB_FIELDS_TASK.taskid.name, APIBLB_FIELDS_TASK.taskrunok.name]
    missing_columns = [col for col in required_columns if col not in node_CtlCenter_globals.dfTaskChainInfo.columns]

    PrintDF(node_CtlCenter_globals.dfTaskChainInfo)
    if missing_columns:
        SendInfoHTTP(f"필요한 열이 없습니다: {missing_columns}")
        return node_CtlCenter_globals.dfTaskChainInfo
    node_CtlCenter_globals.dfTaskChainInfo.loc[node_CtlCenter_globals.dfTaskChainInfo[APIBLB_FIELDS_TASK.taskid.name] == itaskid, APIBLB_FIELDS_TASK.taskrunok.name] = value
    # tableName = node_CtlCenter_globals.dfTaskChainInfo.loc[node_CtlCenter_globals.dfTaskChainInfo[APIBLB_FIELDS_TASK.workname.name]]
    # 특정_workname = 'T1'  # 찾고자 하는 workname 값
    # tableName = node_CtlCenter_globals.dfTaskChainInfo.loc[node_CtlCenter_globals.dfTaskChainInfo[APIBLB_FIELDS_TASK.workname.name] == 특정_workname]
    #curNode = GetNodeFromTable(tableName)
    
    # node_CtlCenter_globals.node_current = curNode
    data_list = node_CtlCenter_globals.dfTaskChainInfo.to_dict(orient="records")
    
    # 리스트를 JSON 문자열로 변환
    json_string = json.dumps(data_list)
    pub_JOBLIST.publish(json_string)    
    PrintDF(node_CtlCenter_globals.dfTaskChainInfo)
  except Exception as e:
    message = traceback.format_exc()
    SendAlarmHTTP(message,True,node_CtlCenter_globals.BLB_ANDROID_IP)
    #return node_CtlCenter_globals.dfTaskChainInfo
  return node_CtlCenter_globals.dfTaskChainInfo

def SetTaskRackStatus(taskid, rackID):
  itaskid = int(taskid)
  try:
    node_CtlCenter_globals.dfTaskChainInfo.loc[node_CtlCenter_globals.dfTaskChainInfo[APIBLB_FIELDS_TASK.taskid.name] == itaskid, APIBLB_FIELDS_TASK.trayrack.name] = rackID
    PrintDF(node_CtlCenter_globals.dfTaskChainInfo)
  except Exception as e:
    print(e)
    return

def GetTaskChainHead(taskKey, val, filtering):
  lsDF = GetTaskChain(taskKey, val,filtering)
  if lsDF is not None and len(lsDF) > 0:
    return lsDF[0]
  return {}

# def GetTaskTrayIdx(taskid):
#   dicTask = GetTaskChainHead(APIBLB_FIELDS_TASK.taskid.name,taskid)
#   return dicTask.get(APIBLB_FIELDS_TASK.trayrack.name)

#node_CtlCenter_globals.dfTaskChainInfo = pd.read_csv(node_CtlCenter_globals.strCSV_TaskChain, delimiter=sDivTab)
# SetTaskCompleted(3)
# print(GetTaskChainHead(APIBLB_FIELDS_TASK.taskrunok.name, 1))
# SetTaskCompleted(29)
# print(GetTaskChainHead(APIBLB_FIELDS_TASK.taskrunok.name, 1))

# df_chainlist.to_csv(node_CtlCenter_globals.strCSV_TaskChain, index=False, sep=sDivTab)

def PopTablelist(idx=0):
  return str(node_CtlCenter_globals.listTable.pop(idx))

def GetTablelist(idx=0):
  log_all_frames(idx)
  if len(node_CtlCenter_globals.listTable) > idx:
    return str(node_CtlCenter_globals.listTable[idx])
  TTSAndroid('테이블 인덱스 에러.')
  return MIN_INT

def InsertTableList(tableTarget_local,idx=0):
    rospy.loginfo(f'{sys._getframe(0).f_code.co_name}:{sys._getframe(1).f_code.co_name}:{tableTarget_local}:{idx}')
    tableID = str(tableTarget_local)
    #tableID = try_parse_int(tableTarget_local)
    if tableID not in node_CtlCenter_globals.listTable:
      node_CtlCenter_globals.listTable.insert(idx,tableID)
    else:
      rospy.loginfo(log_all_frames(tableTarget_local))
    #node_CtlCenter_globals.table_target = tableTarget_local
    table_target_tts = tableTarget_local if try_parse_int(tableTarget_local, MIN_INT) == MIN_INT else f'T{tableTarget_local}'
    TTSAndroid(f'{table_target_tts} 시작.')
    SetTableTarget(GetTablelist(0))
    
def AppendTableList(tableTarget_local):
    SendInfoHTTP(log_all_frames(tableTarget_local))
    if type(tableTarget_local) == list:
      node_CtlCenter_globals.listTable.extend(tableTarget_local)
    else:
      node_CtlCenter_globals.listTable.append(tableTarget_local)

def ClearTableList():
  node_CtlCenter_globals.listTable.clear()

def GetTableList():
    listTable = [str(item) for item in node_CtlCenter_globals.listTable]
    return listTable
   
def AppendDistanceV(ls_distanceV):
    #log_all_frames(distanceBoxStr)
    node_CtlCenter_globals.lsDistanceV.append(ls_distanceV)
    
def GetDistanceV():
    if len(node_CtlCenter_globals.lsDistanceV) == 0:
        return []
    return node_CtlCenter_globals.lsDistanceV[-1] 
    
def ClearDistanceV():
    node_CtlCenter_globals.lsDistanceV.clear()    

def AppendDistanceBox(distanceBoxStr):
    #log_all_frames(distanceBoxStr)
    node_CtlCenter_globals.lsDistanceBox.append(distanceBoxStr)
    
def GetDistanceBox():
    if len(node_CtlCenter_globals.lsDistanceBox) == 0:
        return {}
    return node_CtlCenter_globals.lsDistanceBox[-1]
    
def ClearDistanceBox():
    node_CtlCenter_globals.lsDistanceBox.clear()    

#장애물 감지 및 속도 제어에 쓰이는 함수
def SetSuspend(isSuspended):
    #log_all_frames(distanceBoxStr)
    node_CtlCenter_globals.isSuspended = isSuspended
    
def IsSuspended():
    return node_CtlCenter_globals.isSuspended
    
def UpdateLastBalanceTimeStamp():
  log_all_frames()
  node_CtlCenter_globals.lastCmdBalanceStamp = getDateTime()

def GetLastBalanceTimeStamp():
  return node_CtlCenter_globals.lastCmdBalanceStamp

def UpdateLidarDistanceBoxTimeStamp():
  node_CtlCenter_globals.lastUpdatedLidar = getDateTime()

def GetLidarDistanceBoxTimeStamp():
  return node_CtlCenter_globals.lastUpdatedLidar

def UpdateXY(x,y):
  log_all_frames(f'{x}_{y}')
  node_CtlCenter_globals.lsXYLoc =[x, y]

def UpdateXY_nodeInfo(node_ID=None):
  log_all_frames(node_ID)
  if node_ID == None:
    curTable,node_ID = GetCurrentTargetTable()
  curX,curY = GetLocNodeID(node_ID)
  UpdateXY(curX,curY)
  return curX,curY
  
def GetLocXY():
  if len(node_CtlCenter_globals.lastCrossEPC) >= 24:
    sEPC = node_CtlCenter_globals.lastCrossEPC
    dicEPCInfo = GetEPCInfo(sEPC)
    cmdpos_H, curpos_H = GetPosServo(ModbusID.MOTOR_H)
    return pulseH_to_distance(curpos_H), dicEPCInfo[EPCINFO_FIELDS.TAG_Y.name]
  return node_CtlCenter_globals.lsXYLoc[0],node_CtlCenter_globals.lsXYLoc[1]

#node_CtlCenter_globals.dicTargetPosFeedBack

def GetCurrentJob(tableID = None, isFirstJob = None):
  lsCurrentRow = []
  if tableID == None:
    tableID = GetTableTarget()
  dfReceived = GetDF(tableID)
  if dfReceived is None or dfReceived.empty:
    return {}
  
  if isFirstJob is None:
    lsCurrentRow = dfReceived.loc[dfReceived[APIBLB_FIELDS_TASK.workstatus.name] == APIBLB_STATUS_TASK.Running.value].tail(1).to_dict(orient='records')
  elif isFirstJob:
    lsCurrentRow = [dfReceived.iloc[0]]
  else:
    lsCurrentRow = [dfReceived.iloc[-1]]
  #print(lsCurrentRow)
  if len(lsCurrentRow) > 0:
    return lsCurrentRow[0]
  else:
    return {}
    
  
  



def prtMsg(sendbuf):
    return
    rospy.loginfo(sendbuf)
    # if runFromLaunch:
    #     rospy.loginfo(sendbuf)
    # else:
    #     print(sendbuf)

def SendKeepAlive(sendbuf):
    if pub_ka is not None:
        pub_ka.publish(sendbuf)
        prtMsg(sendbuf)

def GetItemsFromModbusTable(mbid :ModbusID ,itemName : MonitoringField):
  str_mbid_motor = str(mbid.value)
  dicTmp = node_CtlCenter_globals.dic_485ex.get(str_mbid_motor, {})
  return dicTmp.get(itemName.name, MIN_INT)

def GetDestPoint(target_pulse, mbid:ModbusID):
    pot_pos = GetItemsFromModbusTable(mbid,MonitoringField.POT_POS)
    not_pos = GetItemsFromModbusTable(mbid,MonitoringField.NOT_POS)
    if target_pulse == not_pos:
        return not_pos-PULSE_POTNOT_MARGIN
    if target_pulse == pot_pos:
        return pot_pos+PULSE_POTNOT_MARGIN
    return target_pulse

def GetPosServo(mbid:ModbusID):
  cmd_pos = GetItemsFromModbusTable(mbid,MonitoringField.CMD_POS)
  cur_pos = GetItemsFromModbusTable(mbid,MonitoringField.CUR_POS)
  return int(cmd_pos),int(cur_pos)

def GetTimeFromRPM(mbid : ModbusID,target_pulse_str,rotateRPM):
  target_pulse = try_parse_int(target_pulse_str)
  cmd_pos,cur_pos=GetPosServo(mbid)
  diffRPM = round(abs(cur_pos-target_pulse)/roundPulse)
  return calculate_rpm_time(diffRPM, rotateRPM)
    
def GetRPMFromTime(mbid : ModbusID,target_pulse_str,totaltime_sec):
  target_pulse = try_parse_int(target_pulse_str)
  cmd_pos,cur_pos=GetPosServo(mbid)
  diffRPM = round(abs(cur_pos-target_pulse)/roundPulse)
  return calculate_targetRPM_fromtime(diffRPM, totaltime_sec)
    
def GetRPMFromTimeAccDecc(list_local):
    lsTotal = [[]]
    rpm_time = 0
    listDep =  get_list_depth(list_local)
    if isinstance(list_local, dict):
        lsTotal.append([list_local])
    elif isinstance(list_local, list):
        if listDep == 1:
            lsTotal.append(list_local)
        elif listDep == 2:
            lsTotal.extend(list_local)
        else:
            return -2
    else:
        return -1
    timeReturn = 0
    for listTmp in lsTotal:
        if len(listTmp) == 0:
            continue
        if isinstance(listTmp, dict):
            listTmp = [listTmp]
        maxTime = 0
        for dicCtlTmp in listTmp:
            if len(dicCtlTmp) == 0:
                continue            
            if not isinstance(dicCtlTmp, dict):
                print(dicCtlTmp)
            dicCtlTmp: dict 
            sPos = dicCtlTmp.get(MotorWMOVEParams.POS.name, None)
            sSpd = dicCtlTmp.get(MotorWMOVEParams.SPD.name, None)
            sACC = dicCtlTmp.get(MotorWMOVEParams.ACC.name, None)
            sDECC = dicCtlTmp.get(MotorWMOVEParams.DECC.name, None)
            mbid = dicCtlTmp.get(MotorWMOVEParams.MBID.name, None)
            target_pulse = try_parse_int(sPos)
            acc_tmp = try_parse_int(sACC)
            decc_tmp = try_parse_int(sDECC)
            rpm_tmp = try_parse_int(sSpd)
            mbid_instance = ModbusID.from_value(mbid)
            cmd_pos,cur_pos=GetPosServo(mbid_instance)
            diffRPM = round(abs(cur_pos-target_pulse)/roundPulse)
            if rpm_tmp > 0 :
                rpm_time = calculate_rpm_time_accdesc(diffRPM,rpm_tmp,acc_tmp,decc_tmp)
                maxTime = max(maxTime,rpm_time)
                timeReturn += maxTime
    return round(timeReturn,1)


def GetPotNotServo(mbid:ModbusID):
  pot_pos = GetItemsFromModbusTable(mbid,MonitoringField.POT_POS)
  not_pos = GetItemsFromModbusTable(mbid,MonitoringField.NOT_POS)
  return int(pot_pos),int(not_pos)


def GetPotNotHomeStatus(mbid:ModbusID):
  DI_POT = GetItemsFromModbusTable(mbid,MonitoringField.DI_POT)
  DI_NOT = GetItemsFromModbusTable(mbid,MonitoringField.DI_NOT)
  DI_HOME = GetItemsFromModbusTable(mbid,MonitoringField.DI_HOME)
  SI_POT = GetItemsFromModbusTable(mbid,MonitoringField.SI_POT)
  return isTrue(DI_POT),isTrue(DI_NOT),isTrue(DI_HOME),SI_POT

def GetPotNotCurPosServo(mbid:ModbusID):
  pot_int,not_int = GetPotNotServo(mbid)
  cmdpos_int, curpos_int = GetPosServo(mbid)
  return pot_int,not_int,cmdpos_int,curpos_int
  #return max(pot_int,0),max(not_int,0),max(cmdpos_int,0), max(curpos_int,0)

def getPrevDistance():
  # # f"{MotorWMOVEParams.MBID.name}{sDivFieldColon}{mbid}{sDivItemComma}"
  # cmdPrev = node_CtlCenter_globals.lastSendDic_H
  # dicPrevSendbuf = getDic_strArr(cmdPrev, sDivFieldColon, sDivItemComma)
  #dicPrevSendbuf = node_CtlCenter_globals.lastSendDic_H.get(MotorWMOVEParams.POS.name, MIN_INT)
  dicPrevSendbuf = node_CtlCenter_globals.lastSendDic_H.get(MotorWMOVEParams.POS.name, 1)
  return getSignFromInt(dicPrevSendbuf)

def isActivatedMotor(mbid_any):
  # mbid = int(mbid_any)
  # lsReturn = getRunningMotorsBLB()
  # return mbid in lsReturn
  return str(mbid_any) in node_CtlCenter_globals.activated_motors

def getSpeedTableInfo(MBID,k,adjustRate=1):
  return node_CtlCenter_globals.getSpeedTableInfo(MBID,k,adjustRate)

def isScanMode():
  lsRestTables = GetTableList()
  for curTable in lsRestTables:
    dicTagretTableInfoCurrent = getTableServingInfo(curTable)
    if len(dicTagretTableInfoCurrent) == 0:
      return True
  return False    
  
def isScanCompleted():
  lsRestTables = GetTableList()
  #lsRestTables = [4,5,6,7]
  for curTable in lsRestTables:
    combined_list = list(chain(*node_CtlCenter_globals.ScanInfo.values()))
    dicArucoTable = groupFromList(combined_list,ARUCO_RESULT_FIELD.MARKER_VALUE.name)
    lsTableMarkers = dicArucoTable.get(curTable, [])
    if len(lsTableMarkers) == 0:
      return False
  return True
  
def updateShortCutInfo(node1, node2, distance, bReload = True):
    filename=node_CtlCenter_globals.strFileShortCut
    data = pd.read_csv(filename, sep=" ", header=None, names=["Node1", "Node2", "Distance"])
    data[["Node1", "Node2"]] = data[["Node1", "Node2"]].apply(sorted, axis=1, result_type="expand")
    # 노드 간 연결을 일관성 있게 정렬합니다. (작은 노드가 Node1으로 오도록)
    if node1 > node2:
        node1, node2 = node2, node1
    
    # 기존 행이 있는지 확인합니다.
    condition = ((data["Node1"] == node1) & (data["Node2"] == node2))
    if distance <= 0:
        # distance가 0 이하인 경우 해당 노드 쌍을 삭제합니다.
        if condition.any():
            data = data[~condition]
            rospy.loginfo(f"노드 {node1}-{node2} 연결이 삭제되었습니다.")
        else:
            rospy.loginfo(f"노드 {node1}-{node2} 연결 정보가 존재하지 않습니다.")
    else:    
      if condition.any():
          # 기존 행이 있을 경우 거리 값을 업데이트합니다.
          data.loc[condition, "Distance"] = distance
      else:
          # 기존 행이 없을 경우 새로운 행을 추가합니다.
          new_row = pd.DataFrame({"Node1": [node1], "Node2": [node2], "Distance": [distance]})
          data = pd.concat([data, new_row], ignore_index=True)
    
    # 업데이트된 데이터를 파일에 저장합니다.
    data.to_csv(filename, sep=" ", index=False, header=False)
    rospy.loginfo(f"노드 {node1}-{node2}-{distance}의 정보가 성공적으로 업데이트되었습니다.")
    if bReload:
      LoadMapRefresh()

def updateTableServingInfo(tableNo, new_serving_distance=None, new_serving_angle=None, new_marker_angle=None, new_move_distance=None):
    """
    테이블 정보를 업데이트하는 함수. None 파라미터는 기존 값을 유지합니다.

    Parameters:
    tableNo (int): 업데이트할 테이블 번호.
    new_serving_distance (int or None): 새로운 서빙 거리 (None일 경우 기존 값 유지).
    new_serving_angle (int or None): 새로운 서빙 각도 (None일 경우 기존 값 유지).
    new_marker_angle (int or None): 새로운 마커 각도 (None일 경우 기존 값 유지).
    new_move_distance (int or None): 새로운 원점에서 노드까지 거리 (None일 경우 기존 값 유지).
    """
    # 파일을 읽어 리스트로 변환
    file_list = getLines_FromFile(node_CtlCenter_globals.strFileTableNode)
    tableNoInt = int(tableNo)
    updated_lines = []
    updated = False

    for line in file_list:
        checkIDX = line.find("#")
        if checkIDX >= 0 or len(line) < 2:
            updated_lines.append(line)
            continue
        
        splitTmp = line.split(" ")
        if len(splitTmp) >= 6:
            tableID_tmp = int(splitTmp[0])
            
            if tableNoInt == tableID_tmp:
                # 기존 정보에서 None이 아닌 값만 업데이트
                current_serving_distance = splitTmp[2]
                current_serving_angle = splitTmp[3]
                current_marker_angle = splitTmp[4]
                current_move_distance = splitTmp[5]
                
                splitTmp[2] = str(new_serving_distance) if new_serving_distance is not None else current_serving_distance
                splitTmp[3] = str(new_serving_angle) if new_serving_angle is not None else current_serving_angle
                splitTmp[4] = str(new_marker_angle) if new_marker_angle is not None else current_marker_angle
                splitTmp[5] = str(new_move_distance) if new_move_distance is not None else current_move_distance
                
                # 수정된 정보를 다시 하나의 문자열로 합쳐 저장
                updated_line = " ".join(splitTmp)
                updated_lines.append(updated_line)
                updated = True
            else:
                updated_lines.append(line)
        else:
            updated_lines.append(line)

    if not updated:
        rospy.loginfo(f"테이블 번호 {tableNoInt}에 대한 정보를 찾을 수 없습니다.")
        return

    # 파일에 수정된 내용을 다시 저장
    with open(node_CtlCenter_globals.strFileTableNode, 'w') as file:
        file.write("\n".join(updated_lines))

    rospy.loginfo(f"테이블 번호 {tableNoInt}의 정보가 성공적으로 업데이트되었습니다.")

def getMotorStatus(mbid:ModbusID):
  # cmd_pos,cur_pos = GetPosServo(mbid)
  # cmd_spd = int(GetItemsFromModbusTable(mbid,MonitoringField.CMD_SPD))
  cur_spd = int(GetItemsFromModbusTable(mbid,MonitoringField.CUR_SPD))
  cur_pos = int(GetItemsFromModbusTable(mbid,MonitoringField.CUR_POS))
  
  if cur_spd == MIN_INT:
    return STATUS_MOTOR.UNKNOWN 
  cur_spd_abs = abs(cur_spd)
  #mbid_str = str(mbid.value)
  if not isActivatedMotor(mbid.value):
    return STATUS_MOTOR.STOPPED
#   elif isFinishedMotor(mbid):
#     return STATUS_MOTOR.STOPPED
  elif cur_spd_abs >= 0 and cur_spd_abs < 20:
    if cur_spd >= 0:
      return STATUS_MOTOR.PENDING_CW
    else:
      return STATUS_MOTOR.PENDING_CCW
  elif cur_spd_abs > 10:
    if cur_spd > 0:
      return STATUS_MOTOR.RUNNING_CW
    else:
      return STATUS_MOTOR.RUNNING_CCW
  return STATUS_MOTOR.UNKNOWN

def getBLBMotorStatus():
    dic_AlmCDTable = {}
    dic_AlmNMTable = {}
    lsAlarmMBID = []
    for mbid, dictModbus in node_CtlCenter_globals.dic_485ex.items():
        cur_cd = int(dictModbus.get(MonitoringField.ALM_CD.name, -1))
        cur_nm = dictModbus.get(MonitoringField.ALM_NM.name, "N/A")
        mbidInt = int(mbid)
        dic_AlmCDTable[mbidInt] = cur_cd
        dic_AlmNMTable[mbidInt] = cur_nm
        if cur_cd != 0:
          lsAlarmMBID.append(mbidInt)
    return lsAlarmMBID,dic_AlmCDTable,dic_AlmNMTable

def isLiftTrayDownFinished():
    mbid_instance = ModbusID.MOTOR_V
    pot_cur,not_cur,cmdpos,cur_pos =GetPotNotCurPosServo(mbid_instance)
    openTargetPos = round(pot_cur * DOOROPEN_PERCENT)
    if cur_pos > openTargetPos:    
    # if abs(pot_cur-cur_pos) < PULSES_PER_ROUND * 2 and isFinishedMotor(mbid_instance):
    #if getBLBStatus() == BLB_STATUS_FIELD.DOOR_MOVING:
        return True
    else:
        return False

def isInitMotorsAll():
    motors = len(node_CtlCenter_globals.dic_485ex)
    initMotors = count_elements(node_CtlCenter_globals.lsTopicList, '/MB')
    return motors+1 >= initMotors

def isReadyToMoveH_and_540(modbusClass = None):
    lsCheckList = [ModbusID.BAL_ARM1,ModbusID.BAL_ARM2,ModbusID.MOTOR_V,ModbusID.TELE_SERV_MAIN, ModbusID.ROTATE_SERVE_360]
    #lsCheckList = [ModbusID.BAL_ARM1,ModbusID.BAL_ARM2,ModbusID.TELE_BALANCE,ModbusID.MOTOR_V,ModbusID.TELE_SERV_MAIN]
    if modbusClass is not None: #예외처리
      lsCheckList.append(modbusClass)
    return isReadyToMoveMotor(lsCheckList)
    try:
        for mbid, dicTmp in node_CtlCenter_globals.dic_485ex.items():
            mbid_instance = ModbusID.from_value(mbid)
            pot_cur,not_cur,cmdpos,cur_pos =GetPotNotCurPosServo(mbid_instance)
            if mbid == (str)(ModbusID.ROTATE_MAIN_540.value) or mbid == (str)(ModbusID.ROTATE_SERVE_360.value) or mbid == (str)(ModbusID.MOTOR_H.value):
              continue
            DI_POT,DI_NOT,DI_HOME,SI_POT = GetPotNotHomeStatus(mbid_instance)
            if abs(not_cur-cur_pos) > roundPulse and not DI_NOT:
                return False
        return True
    except Exception as e:
        rospy.loginfo(e)
        return False  

def isReadyToMoveMotor(lsCheckList):
    try:
        for mbid, dicTmp in node_CtlCenter_globals.dic_485ex.items():
            mbid_instance = ModbusID.from_value(mbid)
            if mbid_instance not in lsCheckList:
                continue
            if isActivatedMotor(mbid):
                return False            
            pot_cur,not_cur,cmdpos,cur_pos =GetPotNotCurPosServo(mbid_instance)
            DI_POT,DI_NOT,DI_HOME,SI_POT = GetPotNotHomeStatus(mbid_instance)
            #if abs(cur_pos) > roundPulse and not DI_NOT:
            if abs(not_cur-cur_pos) > roundPulse and not DI_NOT:
                return False
        return True
    except Exception as e:
        rospy.loginfo(e)
        sMsg = traceback.format_exc()
        SendAlarmHTTP(sMsg,True,node_CtlCenter_globals.BLB_ANDROID_IP)        
        return False  

    
def isFinishedMotor(mbid:ModbusID, posMargin = PULSES_PER_ROUND*2 ):
    if isActivatedMotor(mbid.value):
        cmd_pos,cur_pos = GetPosServo(mbid)
        target_cur = int(node_CtlCenter_globals.dicTargetPos.get(str(mbid.value), MIN_INT))
        return abs(target_cur - cur_pos) < posMargin
    else:
        return True

def getRunningMotorsBLB():
  return node_CtlCenter_globals.activated_motors

def removeRunningMotorsBLB(mbid):
  if mbid in node_CtlCenter_globals.activated_motors:
    node_CtlCenter_globals.activated_motors.remove(mbid)
    return True
  return False

def appendRunningMotorsBLB(mbid):
  if mbid not in node_CtlCenter_globals.activated_motors:
    node_CtlCenter_globals.activated_motors.append(mbid)  

def getMotorSpdDirection(mbid):
    # CW 는 양수, CCW 는 음수, 정지상태면 0 을 리턴한다    
    curDic = node_CtlCenter_globals.dic_485ex.get(str(mbid),{})
    rtValyue = curDic.get(MonitoringField.CUR_SPD.name, MIN_INT)
    return int(rtValyue)
  
def isISVMotor(mbid):
    curDic = node_CtlCenter_globals.dic_485ex.get(str(mbid),{})
    rtValyue = int(curDic.get(MonitoringField.TOQ_LIMIT.name, MIN_INT))
    return True if rtValyue >= 100 else False

def isLiftLanding():
  rpmV = try_parse_int(getMotorSpdDirection(ModbusID.MOTOR_V.value), 0)
  if rpmV > 10:
    return True
  return False

def movePrepareBLB(nodeFrom: int, nodeTo: int):
    # def movePrepareBLB(nodeFrom: int, nodeTo: int, isForward: bool, inputIdx: int):
    """1개의 노드를 이동한다. 노드 이동 전 분기점에 문제 없는지 검사한다.
    연결되어있는지 여부는 검사하지 않는다. (호출전 별도 검사)
    1. nodeFrom 을 기반으로

    Args:
        nodeFrom (int): 현재 노드
        nodeTo (int): 이동할 노드
        isForward (bool): 진행방향이 Forward 인지 여부,
    """
    """
  0. 노드정보 확인. StateInfo - dict 개체 활용
  1. 만일 둘다 테이블이고 서로 연결되어있다면 문제가 없다.
  2. 둘중 하나 이상이 분기기라면 연결상태로 만듬.
  
  현재출발지가 테이블이면 그냥 가면 됨.
  분기기나 엘베인 경우
  1. 출발지 노드의 진입점 배열에 출발지 ID가 있는지 확인
  2. 당연히 있을거고, (없다면 익셉션) - 상태값이 0인지 1인지 확인 후 SetState 로 출발 노드 세팅
  """
    """ 
  둘이 연결되어있다고 가정.

  """
    scmList = [nodeFrom, nodeTo]  # 시작노드와 목적지 노드
    nodeReadyList = [0, 0]
    nodeSetResult = [False, False]
    nodeSetResult2 = [-1, -1]
    iCnt = 0

    for (
        scmCur
    ) in scmList:  # 시작노드와 목적지 노드를 체크하므로 이 루프는 무조건 2회 돈다.
        scmCurValue = node_CtlCenter_globals.StateInfo.get(scmCur, None)
        if (
            scmCurValue == None
        ):  # 테이블이나 분기기, 엑세스 도어처럼 특수 노드가 아닌 경우는 그냥 출발.
            nodeSetResult[iCnt] = True
            nodeReadyList[iCnt] = -1
        else:
            setStateKey = scmCur
            if iCnt == 1:
                setStateKey = scmList[0]
            else:
                setStateKey = scmList[1]

            if setStateKey in scmCurValue:
                # nodeReadyList[iCnt] = (int)(scmCurValue.index(setStateKey)/2)
                nodeReadyList[iCnt] = scmCurValue.index(setStateKey)
            if setStateKey < 0:
                nodeSetResult[iCnt] = False
                # 익셉션 발생시켜야 함.
            else:
                bIsOpen = nodeReadyList[iCnt] in node_CtlCenter_globals.nodeStateOpen
                nodeSetResult[iCnt] = setStateBranch(scmCur, bIsOpen)

                # 진입방향과 출구방향이 같으면 역방향, 그 외는 정방향으로 나가야 됨
                nodeSetResult2[iCnt] = 0 if bIsOpen else 1
        # directionCheckCnt += nodeReadyList[iCnt]
        iCnt += 1
    result = False in nodeSetResult

    if result:
        # 노드 세팅이 잘못되어 있는 경우. 어떻게 알람낼지 생각해보자.
        rospy.loginfo("ERROR")
    # else:
    #     # print(f"경로설정 성공! 이동합니다.{scmList},포워딩:{isForward} : 진출 {nodeReadyList[0] } ,진입 {nodeReadyList[1] }")
    #     rospy.loginfo(
    #         f"경로설정 성공! 이동합니다.{scmList}, 시작:{nodeFrom}:{nodeReadyList[0]}, 도착:{nodeTo}:{nodeReadyList[1]}"
    #     )
    # return not result,nodeReadyList[1]
    return not result, nodeSetResult2[0], nodeSetResult2[1]

    """
  iPos = 0 일때 Open, iPos = 1일때 Close 상태
  bStraightPath = False 라는건 백워드로 가야한다는 뜻.
  """

#@log_arguments
def getSeqMap(startNode: int, endNode: int) -> list:
    """노드의 시작지점과 끝 지점을 받아 제어시퀀스를 리턴한다

    Args:
        startNode (int): 시작노드
        endNode (int): 도착노드

    Returns:
        _type_: 딕셔너리의 배열
    """
    log_all_frames()
    curStart = int(startNode)
    curEnd = int(endNode)
    lsPath, pathCost = getPathBLB(node_CtlCenter_globals.graph, curStart, curEnd)
    lsPathRev, pathCostRev = getPathBLB(node_CtlCenter_globals.graph, curEnd, curStart)
    # lspath :[1, 2], pathCost:[1400000, 1000000] - 거쳐야 하는 노드 번호 및 각 노드간 거리
    # rospy.loginfo(f"경로정보:{lsPath}, 구간별거리:{pathCost}")
    # rospy.loginfo(f"역방향정보:{lsPathRev}, 역방향 구간별거리:{pathCostRev}")
    # rospy.loginfo(f"마지막 주행 정보:{node_CtlCenter_globals.lastPath}")
    #lastPath 와 lsPath 의 공통원소가 2개 이상이면 왔던방향으로 되돌아가고 그렇지 않으면 왔던 방향으로 진행한다.

    numberOfPath = len(lsPath) - 1  # 구간 개수
    listSeqMapOrg = []
    listSeqMapOptimized = []
    signDistance, distancePrev = getPrevDistance()
    firstStep = signDistance * pathCost[0]
    lastNodeStart = -1
    # if lastNode == curTarget:
    common_elements = set(node_CtlCenter_globals.lastPath) & set(lsPath)
    #if len(node_CtlCenter_globals.lastPath) > 1 and len(lsPath) > 1:
    if len(common_elements) >= 2:
        # if lsPath[1] == node_CtlCenter_globals.lastPath[-2]:
            firstStep *= -1
    # 각 구간을
    for idx in range(numberOfPath):
        curTarget = lsPath[idx + 1]
        # if lastNode == curTarget:
        #     moveForward = not moveForward
        # scmCurValue = StateInfo.get(scmCur, None)
        # if scmCurValue == None: #테이블이나 분기기, 엑세스 도어처럼 특수 노드가 아닌 경우는 그냥 출발.
        #   nodeSetResult[iCnt] = True
        #   nodeReadyList[iCnt] = -1
        resultMove, nodeSetResult_IN, nodeSetResult_OUT = movePrepareBLB(
            curStart, curTarget
        )
        # print(f'{curStart} -> {curTarget} : {resultMove}, 진입인덱스 : {inputIDX}' )
        # rospy.loginfo(
        #     f"{resultMove}, 출발지&제어:{curStart}/{nodeSetResult_IN}, 도착지제어:{curTarget}/{nodeSetResult_OUT}, 엔코더:{pathCost[idx]}"
        # )
        if resultMove:
            dicMap = {}
            distanceCur = pathCost[idx]
            if firstStep == 0:
                firstStep = distanceCur
            scmCurValue = node_CtlCenter_globals.StateInfo.get(curStart, None)
            signTmp, distanceTmp = getSignFromInt(firstStep)
            if idx == 0 and abs(firstStep) > 0:
                distanceCur = firstStep
            elif scmCurValue is not None:
                if (
                    curStart in node_CtlCenter_globals.StateInfo
                    and curTarget in scmCurValue
                ):
                    if lastNodeStart in scmCurValue:
                        nodeEnt = scmCurValue.index(lastNodeStart) % 2
                        nodeExit = scmCurValue.index(curTarget) % 2
                        if nodeEnt == nodeExit:
                            distanceCur = distanceCur * signTmp * -1
                            firstStep = distanceCur
                        else:
                            distanceCur = distanceCur * signTmp
                    else:
                        SendAlarmHTTP('여기에 어떻게 들어옴?22')
                else:
                    SendAlarmHTTP('여기에 어떻게 들어옴?11')
            else:
                distanceCur = distanceCur * signTmp

            # if not isForward:
            #     distanceCur = distanceCur * -1
            dicMap[SeqMapField.START_NODE.name] = curStart
            dicMap[SeqMapField.START_STATUS.name] = nodeSetResult_IN
            dicMap[SeqMapField.END_NODE.name] = curTarget
            dicMap[SeqMapField.END_STATUS.name] = nodeSetResult_OUT
            lastNodeStart = curStart
            lastNodeEnd = curTarget
            # dicMap[SeqMapField.DIRECTION.name] = isForward
            dicMap[SeqMapField.DISTANCE.name] = distanceCur
            # sDir = 'E' #증가
            # if distanceCur < 0:
            #   sDir = 'W'
            # dicMap[APIBLB_FIELDS_TASK.direction.name] = sDir
            listSeqMapOrg.append(dicMap)
            lastNode = curStart  # 이전 노드를 기억해둔다
            curStart = curTarget  # 이동 완료한 노드가 새 출발점이 된다
        else:
            sErrMsg = "ERROR"
            raise Exception(sErrMsg)  # 예외를 발생시킴

    listSeqMapOrg_B = copy.deepcopy(listSeqMapOrg)
    node_CtlCenter_globals.lastPath = lsPath
    rospy.loginfo(f"원래경로:{listSeqMapOrg_B}")
    listSeqMapOptimized = getSimplePath2(listSeqMapOrg_B, isScanMode())
    rospy.loginfo(f"최적화경로:{listSeqMapOptimized}")
    #최적화 경로와 원래경로를 모두 리턴
    return listSeqMapOptimized,listSeqMapOrg


def getNodeState(nodeID):
    nodeIDint = int(try_parse_float(nodeID))
    nodeIDStr = str(nodeIDint)
    scmCurValue = node_CtlCenter_globals.StateInfo.get(nodeIDint, None)
    if (
        scmCurValue == None
    ):  # 테이블이나 분기기, 엑세스 도어처럼 특수 노드가 아닌 경우는 -1 리턴
        return -1
    
    #여기는 키가 str 로 저장됨
    nodeInfoDic = node_CtlCenter_globals.stateDic.get(nodeIDStr, None)
    if nodeInfoDic == None:
        return -2  # 정보가 들어온 것이 없음. MQTT + ROS 통신에 문제가 있음

    # nodeState = nodeInfoDic.get("STATE", None)
    # if nodeState == None:
    #     return (
    #         -3
    #     )  # 데이터는 들어왔지만 상태값을 나타내는 필드가 없음. 프로그램 로직 문제.
    #return nodeState
    return nodeInfoDic


def setNodeStateEx(nodeID, statusVal):
    istatusVal = int(statusVal)
    if istatusVal >= 0:
      node_CtlCenter_globals.StateSet[int(nodeID)] = istatusVal
    # 여기에 현재 노드 추가


def service_setbool_client(serviceName, enable, serviceType):
    bResult = None
    if isServiceExist(serviceName) == False:
        log_all_frames(f"Service not found : {serviceName}")
        return False
    # rospy.wait_for_service(serviceName, 2)
    dtNow = getDateTime()
    serviceNameID = None
    if enable == None:
        serviceNameID = f"{serviceNameID}"
    else:
        serviceNameID = f"{serviceNameID}{enable}"
    serviceLastTimeStamp = node_CtlCenter_globals.dicServiceTimeStamp.get(serviceNameID)
    node_CtlCenter_globals.dicServiceTimeStamp[serviceNameID] = dtNow
    if serviceLastTimeStamp != None and not isTimeExceeded(serviceLastTimeStamp, 1000):
        return False
    try:
        setbool_proxy = rospy.ServiceProxy(serviceName, serviceType)
        sos = None
        # print(type(serviceType))
        if enable == None:
            if serviceType == Trigger:
                sos = TriggerRequest()
            else:
                sos = EmptyRequest()
            bResult = setbool_proxy(sos)
            return bResult

        responseResult = setbool_proxy(enable)
        # print(responseResult)
        #rospy.loginfo(f"Service({serviceType}) called : {serviceName} - {enable}")
        log_all_frames()
        return responseResult
    except Exception as e:
        sMsg = traceback.format_exc()
        SendAlarmHTTP(sMsg,True,node_CtlCenter_globals.BLB_ANDROID_IP)        
        rospy.loginfo(e)
        return False  
    
def SetDFTableInfo(lsDicArray):
  if (type(lsDicArray) != list):
    lsDicArray = json.loads(lsDicArray)
  df = pd.DataFrame(lsDicArray)
  
  #필요한 필드는 정수형으로 변환.
  lsIntergerColumns = [key_workstatus, key_detailcode, key_distance]
  if len(df) > 0:
    for keytmp in lsIntergerColumns:
      if keytmp in df.columns:
        df[keytmp] = df[keytmp].astype(float).astype(int)

    PrintDF(df)
    firstnodeStr = GetFirstnodeFromDF(df)
    firstnodeStr = try_parse_int(firstnodeStr, firstnodeStr)
    #node_CtlCenter_globals.lsNodeHistory.append([firstTable,firstnodeStr])
    #node_CtlCenter_globals.node_current = firstnodeStr
    endnodeStr = GetEndnodeFromDF(df)
    nodeIDTarget = GetTableIDFromStr(endnodeStr)
    if nodeIDTarget is not None:
      if IsEnableSvrPath():
        dicTaskInfo = GetTaskChainHead(APIBLB_FIELDS_TASK.workname.name, endnodeStr, True)
        tableTarget = GetEndnodeFromDF(df)
        taskid = dicTaskInfo.get(APIBLB_FIELDS_TASK.taskid.name,0)
        if tableTarget != HOME_TABLE and taskid == 0:
          node_CtlCenter_globals.dfTaskChainInfo
        df[APIBLB_FIELDS_TASK.taskid.name] = taskid
        PrintDF(df)
        SetDF(nodeIDTarget,df)
        #AppendTableList(tableTarget)
        # if IsOrderEmpty():
        #   InsertTableList(nodeIDTarget)
          #SetWaitConfirmFlag(False,AlarmCodeList.OK)
      else:
        rospy.loginfo(f'서버에서 지시정보가 수신되었지만 현재 단독모드 : {df}')  
    else:
      rospy.loginfo(f'수신된 지시정보 파싱 에러 : {df}')
  else:
    rospy.loginfo(f'Empty DF : {df}')  
print(os.path.splitext(os.path.basename(__file__))[0],getDateTime())