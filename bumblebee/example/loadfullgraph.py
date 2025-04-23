from UtilBLB import *
strPNG_NodeInfoTest = f"{dirPath}/node_csv_Test.png"
strFileCrossTest = f"{dirPath}/CROSS_순환.txt"
strFileShortCutTest = f"{dirPath}/SHORTCUT_순환.txt"
# strFileCrossTest = f"{dirPath}/CROSS.txt"
# strFileShortCutTest = f"{dirPath}/SHORTCUT.txt"

def LoadFullGraph2(dictMap, dictCross,strCSV_NodeInfo,strCSV_RrailInfo):
    dfResult = pd.DataFrame()                    
    dfRail = pd.DataFrame()                    
    node_xy = {} #nodeID 가 키, 값은 [x,y] 형태로 들어옴.
    bidirectional_list = []
    node_coordinates = {}
    lsDic_nodeInfo = []
    def calculate_coordinates():
        def dfs(nodeStr, x, y, parent=None, isRecur=True):
            node = int(nodeStr)
            if node in node_coordinates:
                return
            node_coordinates[node] = (x, y)
            for neighbor, distance in dictMap[node]:
                if neighbor == parent:
                    continue
                direction = ''
                if node in dictCross:
                    index = dictCross[node].index(neighbor) if neighbor in dictCross[node] else -1
                    if index == 0:  # North
                        new_x, new_y = x, y + distance
                        direction='N'
                    elif index == 1:  # South
                        new_x, new_y = x, y - distance
                        direction='S'
                    elif index == 2:  # East
                        new_x, new_y = x + distance, y
                        direction='E'
                    elif index == 3:  # West
                        new_x, new_y = x - distance, y
                        direction='W'
                    else:
                        dx = neighbor - node
                        new_x, new_y = x + dx * distance, y
                        direction='X'
                else:
                    direction='Y'
                    dx = neighbor - node
                    new_x, new_y = x + dx * distance, y
                
                if direction != '':
                  dicTmp =  {
                      APIBLB_FIELDS_INFO.start.name: node,
                      APIBLB_FIELDS_INFO.end.name: neighbor,
                      APIBLB_FIELDS_INFO.distance.name:distance,
                      APIBLB_FIELDS_INFO.speed.name: "500",
                      APIBLB_FIELDS_INFO.direction.name: direction,
                      APIBLB_FIELDS_INFO.nodetype.name: GetNodeType(node,neighbor,dictCross),
                      APIBLB_FIELDS_INFO.railtype.name: "N",
                      APIBLB_FIELDS_INFO.st_xval.name: x,
                      APIBLB_FIELDS_INFO.st_yval.name: y,
                      APIBLB_FIELDS_INFO.et_xval.name: new_x,
                      APIBLB_FIELDS_INFO.et_yval.name: new_y
                    }
                  lsDic_nodeInfo.append(dicTmp)
                  log_all_frames(f'neighbor:{neighbor},new_x:{new_x},new_y:{new_y},node:{node}:direction={direction}')
                if isRecur:
                  dfs(neighbor, new_x, new_y, node, isRecur)

        lsCross = list(dictCross.keys())
        dfs(lsCross[0], 0, 0, parent=None,isRecur=False)
        lsDicCounterNode = []
        for dicBase in lsDic_nodeInfo:
          lsDicCounterNode.append(getNodeMirrored(dicBase,dictCross))
        lsDic_nodeInfo.extend(lsDicCounterNode)
        df = pd.DataFrame(lsDic_nodeInfo)
        while(True):
          df_set = set(df[APIBLB_FIELDS_INFO.start.name]).union(set(df[APIBLB_FIELDS_INFO.end.name]))
          missing_values = [value for value in dictMap.keys() if value not in df_set]
          if len(missing_values) == 0:
              break
          
          #print(df_set)
          result_set = df_set.difference(dictCross.keys())
          result_set = df_set
          #print(result_set)
          # 시간 측정 시작
          start_time = time.time()
          for nodeToCheck in result_set:
              lsNestedNodesCurrent = dictMap[nodeToCheck]
              for nodeInfo in lsNestedNodesCurrent:
                nodeNested = nodeInfo[0]
                conditionToUpdate = (df[APIBLB_FIELDS_INFO.start.name] == nodeToCheck) & (df[APIBLB_FIELDS_INFO.end.name] == nodeNested)
                matching_Add2 = df.loc[conditionToUpdate].to_dict(orient="records")
                #print(df)
                if len(matching_Add2) > 0:
                    continue
                lsRecord = df.loc[df[APIBLB_FIELDS_INFO.end.name] ==  nodeToCheck].to_dict(orient="records")
                dicRecord = lsRecord[0]
                print(matching_Add2)
                startNode = int(dicRecord[APIBLB_FIELDS_INFO.start.name])
                endNode = int(dicRecord[APIBLB_FIELDS_INFO.end.name])
                node_distance = nodeInfo[1]
                dicNested = getNodeNested(nodeNested,node_distance,dicRecord,dictCross,dfRail)
                # if nodeNested in dictCross:
                #     dfs(nodeNested, dicNested['et_xval'], dicNested['et_yval'], parent=None,isRecur=False)
                dicMirror = getNodeMirrored(dicNested,dictCross)
                #if len(matching_Add2) == 0:
                df = pd.concat([df, pd.DataFrame([dicNested,dicMirror])], ignore_index=True)
                print(df)
          # 시간 측정 종료
          end_time = time.time()
          print(f"for nodeToCheck in result_set 시간: {end_time - start_time:.6f} 초")
                    
        pd.set_option('display.max_rows', None) 
        print(df)
        return df                
             
    crossroad_nodes = set(dictCross.keys())
    map_nodes = set(dictMap.keys())

    if not crossroad_nodes.issubset(map_nodes):
        raise MismatchError("Some crossroad nodes in dictCross are missing from dictMap")

    for start, connections in dictMap.items():
        if len(connections) == 0:
            raise DisconnectedNodeError(f"Node {start} is disconnected in dictMap")

    dfResult = calculate_coordinates()
    sorted_df = dfResult.sort_values(by=APIBLB_FIELDS_INFO.start.name, ascending=True)
    sorted_df.to_csv(strCSV_NodeInfo,index=False, sep=sDivTab)
    
    bidirectional_list = sorted_df.to_dict(orient="records")
    node_xy = getNodeLocationInfo(bidirectional_list)
    for dicTmp in bidirectional_list:
            #         APIBLB_FIELDS_INFO.st_xval.name: str(end_x),
            # APIBLB_FIELDS_INFO.st_yval.name: str(end_y),
        startTmp = dicTmp[APIBLB_FIELDS_INFO.start.name]
        startX = dicTmp[APIBLB_FIELDS_INFO.st_xval.name]
        startY = dicTmp[APIBLB_FIELDS_INFO.st_yval.name]
        node_xy[startTmp] = [startX,startY]
    
    return bidirectional_list,node_xy

StateInfo: Dict[str, list] = {}
file_list = getLines_FromFile(strFileCrossTest)
for i in file_list:
    checkIDX = i.find("#")
    if checkIDX >= 0 or len(i) < 2:
        continue
    splitTmp = i.split(sDivTab)
    if len(splitTmp) > 4:
        nodeID_tmp = (int)(splitTmp[0])
        stateNodeCur = [
            (int)(splitTmp[1]),
            (int)(splitTmp[2]),
            (int)(splitTmp[3]),
            (int)(splitTmp[4]),
        ]
        StateInfo[nodeID_tmp] = stateNodeCur
    else:
        rospy.loginfo(f"잘못된 형식의 분기기 정보! {i} - 알람에 추가할 것")
graph, bgraphOK,node_seq = LoadGraph(strFileShortCutTest)
if not bgraphOK:
    rospy.loginfo(
        f"맵 무결성 검사 실패, 도달할 수 없는 노드가 있습니다 - {graph}- 알람에 추가할 것"
    )
strCSV_NodeInfoTest = f"{dirPath}/node_csv_test.txt"
lsReturn,node_xy = LoadFullGraph2(graph, StateInfo,strCSV_NodeInfoTest,strCSV_RrailInfo)
df = pd.DataFrame(lsReturn)
# 데이터 추출
x_values = df[APIBLB_FIELDS_INFO.st_xval.name]
y_values = df[APIBLB_FIELDS_INFO.st_yval.name]
node_labels = df[APIBLB_FIELDS_INFO.start.name]

# 플롯 생성
plt.figure(figsize=(10, 8))
plt.scatter(x_values, y_values, c='blue', label='Node', zorder=1)

# 각 노드에 라벨 추가
for x, y, label in zip(x_values, y_values, node_labels):
    plt.text(x, y, str(label), fontsize=6, ha='right', va='bottom', zorder=3)

resultJS = []
print(node_xy)
try:
    # Simulating with the uploaded file
    file_path = strFileTableNodeEx
    df_manager = DataFrameManager(file_path)
    # Using the new transform_to_custom_dict_list method
    custom_dict_list = df_manager.transform_to_custom_dict_list()
    #custom_df = pd.DataFrame(custom_dict_list)
    #tmpReturn = DataFrameManager.sort_by_keys(None, custom_df, "tableno", "end", True,True)
    #print(tmpReturn.to_dict(orient='records'))
    lsX = []
    lsY = []
    for dicTable in custom_dict_list:
        end = int(dicTable[APIBLB_FIELDS_INFO.end.name])
        tableName = dicTable[APIBLB_FIELDS_INFO.tableno.name]
        node_x = node_xy[end]
        tb_xval = dicTable[APIBLB_FIELDS_INFO.tb_xval.name] + int(node_x[0])
        tb_yval = dicTable[APIBLB_FIELDS_INFO.tb_yval.name] + int(node_x[1])
        dicTable[APIBLB_FIELDS_INFO.tb_xval.name] = tb_xval
        dicTable[APIBLB_FIELDS_INFO.tb_yval.name] = tb_yval
        lsX.append(tb_xval)
        lsY.append(tb_yval)
        resultJS.append(dicTable)
        plt.text(tb_xval, tb_yval,tableName , fontsize=10, ha='right', va='bottom', zorder=3)                
    plt.scatter(lsX, lsY, c='red', label='Table', zorder=2)

except Exception as e:
    print(traceback.format_exc())

# 축과 배경 설정
plt.axhline(0, color='black', linewidth=0.5, zorder=1)
plt.axvline(0, color='black', linewidth=0.5, zorder=1)
plt.grid(color='gray', linestyle='--', linewidth=0.5, zorder=0)
plt.title('Node&Table Location')
plt.xlabel('Loc X')
plt.ylabel('Loc Y')
# Loc X와 Loc Y 축 범위를 동일하게 설정
min_val = min(min(lsX), min(lsY), min(x_values), min(y_values))
max_val = max(max(lsX), max(lsY), max(x_values), max(y_values))
# Loc X와 Loc Y 축 범위를 동일하게 설정하며, 마진 추가
margin = (max_val - min_val) * 0.1  # 전체 범위의 10%를 마진으로 추가
plt.xlim(min_val - margin, max_val + margin)
plt.ylim(min_val - margin, max_val + margin)        
# plt.xlim(min_val, max_val)
# plt.ylim(min_val, max_val)        
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')

if isFileExist(strPNG_NodeInfoTest):
    os.remove(strPNG_NodeInfoTest)
try:
    plt.savefig(strPNG_NodeInfoTest)
except Exception as e:
    print(e)          
plt.close()        
