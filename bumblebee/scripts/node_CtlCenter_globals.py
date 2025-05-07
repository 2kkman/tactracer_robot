#!/usr/bin/env python3
from node_CtlCenter_import import *
bInitOK = False
bReturn_CROSS = False
#현재 위치한 노드ID
node_current = node_KITCHEN_STATION
node_seq = []
table_history = []
# node_current = 1
table_target = HOME_CHARGE
# node_direction = True

param_IMU_show = True
param_ARD_show = False
enableDummyArduino = False

#샤누이사 서버랑 연동할때 True
enableSvrPath = False
#enableSvrPath = True

stateDic = {}
dicTorqueAve = {}
dicTorqueMax = {}
dicOvrAve = {}
dicOvrMax = {}
dicPOS_ABS = {}

dicARD_CARRIER = {}
dicARUCO_Result = {}
dicARUCO_last = {}
lsARUCO_History = []
last_detect_3d = None
last_detect_status = None
lsHistory_motorH = []
dicModbusShakeLevel = {}
dicTopicCallbackTimeStamp = {}
dictTopicToSubscribe = {}
lock = threading.Lock()
lastSpdArm1TimeStamp = getDateTime()
lastCmdBalanceStamp = getDateTime()
lastMainLoopTimeStamp = getDateTime()
lastCmdTimeStamp = getDateTime()
lastUpdatedLidar = getDateTime()
# 최근 100개 데이터를 저장할 deque 생성
obstacle_history = deque(maxlen=10)
lastSendStatus = getDateTime()
lastSendStatusDelaySec = 3
lastSendStatusList = []

aruco_lastDiffX = aruco_lastDiff_Default
aruco_lastDiffY = aruco_lastDiff_Default

lsXYLoc = [0,0]
tiltStatus = TRAY_TILT_STATUS.TiltDiagonal

aruco_try = 0
cntLoop = 0
#cmdIdx = 0
# dirPrev = True
dicServiceTimeStamp = {}
IMU_roll = 0
IMU_pitch = 0
moveForward = True  # 우선 정방향으로 세팅 , 실전에서는 현재 라이다가 바라보고 있는 방향으로 결정된다.
bOnScanMarker = False
"""
CROSS.txt 예제 (교차로 정보)
100 -1 2 3 91

#2023-09-05에 업데이트
#100번 크로스 상태0 일때 시작점은 2과 , 끝점은 3과 연결된 상태.
#100번 크로스 상태1 일때 시작점은 끊김(-1) , 끝점은 91와 연결된 상태.
"""

# 테이블을 제외한 분기기, 리프트, 엑세스 도어의 nodeID 가 등록된 변수
StateInfo: Dict[str, list] = {}

#테이블 스캔시 스캔정보 임시저장.
#노드의 시작+끝 값이 KEY, 아르코마커 dict 의 배열이 Value 가 됨.
#아르코마커
ScanInfo: Dict[str, list] = {}
dfLinkPosInfo = pd.DataFrame()
dfLast = pd.DataFrame()
dfTaskChainInfo = pd.DataFrame()
dfEPCInfo = pd.DataFrame()
dfEPCTotal = pd.read_csv(strFileEPC_total, delimiter=sDivTab)
dfEPCView = pd.DataFrame()
lsRackStatus = [0,0]
# 분기기 메타정보 100 -1 2 3 91 를 읽어들여서 100 이 키, -1 2 3 91 번호가 배열
# 100 -1 2 3 91 => 100번 리프트에서 상태 0 일때는 진입쪽은 없고 진출은 2번으로 연결됨,
#                   상태 1일때는 진입은 3번과, 진출은 91번과 연결되어있음.
# 200 2 91 3 -1 => 200번 리프트에서 상태 0 일때는 진입쪽와 진출은 91번으로 연결됨,
#                   상태 1일때는 진입은 3번과, 진출은 끊어져있음 -1

# 분기기 제어명령, nodeID - 상태값 쌍으로 설정 원하는 분기기 값을 ROSTOPIC -> SEND_MQTT 로 발행한다.
# 모든 분기기 nodeMCU 가 이 값을 수신하며 nodeID 에 해당하는 nodeMCU 는 분기기 상태를 이 변수에 따라 제어한다.
StateSet: Dict[int, int] = {}

# portArd_M = "/dev/ttCARD_M"  # 아두이노 시리얼 포트
# # portArd_M = '/dev/ttyACM0' # 아두이노 메가 정품 시리얼 포트
# serArd_M = None
# listPortArd = [portArd_M]
# listArdInstance = [serArd_M]
# baudArd = 115200  # 아두이노 통신속도
# lineArdData = []
dic_485ex = {}  # 모터 모니터링 데이터
dic_CROSSINFO = {}
dic_BMS = {}
dic_DF = {}  # 테이블 - DataFrame 지시정보
activated_motors = []  # 현재 모니터링 중인 모터
listBLB = []  # 경로 지시정보
listTable = []  # 순차서빙 테이블 리스트
lsSlowDevices = ["1", "2"]
last_imu_time = getDateTime()
lsNoLiftDownNodes = []
angle_z = 0

status_bal = STATUS_BALANCING.EXTEND_FINISHED
flag_liftdown = False
flag_liftup = False
is_lifted = False
is_docked = False
flag_WaitConfirm = False
dict_WaitReason = {AlarmCodeList.OK.value:AlarmCodeList.OK.name}

seq = 0
lastSendDic_H = {}
waitCross = False
nStart = ""
nTarget = ""
nEncoder = 0
numSubTopics = 0
multi_line_string = ""  #1초 단위로 콘솔에 메세지를 출력하기 위한 변수.
lastCrossEPC = ""  #1초 단위로 콘솔에 메세지를 출력하기 위한 변수.

# nStartStatus = ""
# nEndStatus = ""
# curStartState = None
# curTargetState = None
dicCrossState = {}
dicTargetPos = {}
dicServoPos = {}    #모터 구동전, 구동후 위치값을 저장. 모터 구동마다 갱신됨.dicServoPos[key_motorH][0] = 시작점, [1] = 종료점
dicTargetPosFeedBack = {}
dicNestedNodes: Dict[int, int] = {}

lsDistanceBox = []
isSuspended = False
lsDistanceV = []
lsDistanceVDicArr = []  #리프트 높이 캘리브레이션 용도
lsDistanceArmDicArr = []  #서빙암 모터 캘리브레이션 용도
dfDistanceV = pd.DataFrame()
dfDistanceArm = pd.DataFrame()
dfAruco = pd.DataFrame()
lsArucoDicArr = []  #아르코마커 탐색용도

distanceServingTeleTotal = -1
#타겟 필요한지 생각 좀 해보자
dicTTS = {}
#dicCmdPos = {}
lsServices = []
lsTopicList = []
curBLB_Status = BLB_STATUS_FIELD.CHARGING
curOP_Status = BLB_OP_FIELD.MANUAL

flag_req_doorOpen = False
flag_req_doorClose = False

timestamp_touchinit = getDateTime()
bInit = False

# 전체 지도 맵을 읽어들인다.
""" 예제
99 98 2200
98 1 1800
1 2 4200
2 3 2400
3 4 5000
4 5 5000
"""
dynamic_reconfigure_client = None
dynamic_reconfigure_clientName = '/bumblebee_reconfigure_3D'

BLB_ANDROID_IP = BLB_ANDROID_IP_DEFAULT
SERVING_ARM_FOLD_CONSTANT = 0
SERVING_ARM_EXPAND_CONSTANT = 0
SERVING_ARM_BALANCE_PULSE = 530000
SERVING_ARM_EXPAND_PULSE = 200000
DefaultGndDistance = 0.56

if not isRealMachine:
    BLB_ANDROID_IP = '172.30.1.22'

if isFileExist(strFileDistanceArm):
    dfDistanceArm = pd.read_csv(strFileDistanceArm, delimiter=sDivTab)
if isFileExist(strFileDistanceV):
    dfDistanceV = pd.read_csv(strFileDistanceV, delimiter=sDivEmart)
if isFileExist(strFileAruco):
    dfAruco = pd.read_csv(strFileAruco, delimiter=sDivTab)

nodeStateOpen = [0, 1]  # 인덱스가 0,1 일때는 Open 상태, 2,3 일땐 Close

def LoadMap():
    # graph.keys() 가 전체 노드리스트가 된다
    file_list = getLines_FromFile(strFileCross)
    lsInitShortCut = []
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
            break
        else:
            sMsg = f"잘못된 형식의 분기기 정보! {i}"
            rospy.loginfo(sMsg)    
            SendAlarmHTTP(sMsg,True,BLB_ANDROID_IP)
    
    if not isFileExist(strFileShortCut):
        for jcID, lsNodes in StateInfo.items():
            for nodeID in lsNodes:
                if nodeID > 0:
                    lsInitShortCut.append(f'{nodeID} {jcID} 1\n')
        if len(lsInitShortCut) > 0:
            # 파일에 업데이트된 내용 쓰기
            with open(strFileShortCut, 'w') as file:
                for line in lsInitShortCut:
                    file.write(line)
            
    
    graph, bgraphOK,node_seq = LoadGraph(strFileShortCut)
    
    if not bgraphOK:
        rospy.loginfo(
            f"맵 무결성 검사 실패, 도달할 수 없는 노드가 있습니다 - {graph}- 알람에 추가할 것"
        )

    for crossID, nestedNodes in StateInfo.items():
        graphValue = graph.get(crossID, None)
        if graphValue == None:
            rospy.loginfo("분기기 정보가 전체 맵에 없음! - 알람에 추가할 것")
            continue
        # 분기기 정보는 graph 에 명시되어있으며 분기기 정보에 연결되었다고 명시된 노드들과 graph 의 인접 노드
        # 정보는 일치해야 한다.
        graphValueFilteredRaw = [
            x for x in graphValue if x != -1
        ]  # -1 은 연결포인트가 없음을 의미하며 비교에서 제외한다.
        nestedNodesFiltered = [x for x in nestedNodes if x != -1]

        keys, values = zip(*graphValueFilteredRaw)
        is_included = set(list(keys)).issubset(set(nestedNodesFiltered))
        if not is_included:
            rospy.loginfo(
                f"분기기 무결성 이상!, 분기기번호 : {crossID}:{keys}-{nestedNodes} - 알람에 추가할 것"
            )

    totalNodes = graph.keys()
    totalNodesCnt = len(totalNodes)
    lsTotalMap,node_location = LoadFullGraph2(graph, StateInfo,strCSV_NodeInfo,strCSV_RrailInfo)
    print(node_location)

    # 특수노드가 몇개인지 조사.(현재로는 부엌, 충전소만 정의됨)
    numberOfSpecialNodes = sum(
        is_value_in_enum(node, NODE_SPECIAL_VALUE) for node in totalNodes
    )
    print(f"특수노드:{numberOfSpecialNodes}개")
    numberOfCrossNodes = len(StateInfo)
    numberOfOrdinaryNodes = totalNodesCnt - numberOfCrossNodes - numberOfSpecialNodes
    print(f"테이블노드:{numberOfOrdinaryNodes}개")
    print(f"분기기:{numberOfCrossNodes}개, 총 노드수 {totalNodesCnt} : 모든노드 정보 : {totalNodes}")
    return graph, bgraphOK ,lsTotalMap,node_location,node_seq
graph, bgraphOK ,lsTotalMap,node_location,node_seq = LoadMap()

#lsNoLiftDownNodes.extend(StateInfo.keys())
#lsNoLiftDownNodes.extend([10])
#lsNoLiftDownNodes.append(node_CHARGING_STATION)

lastPath = []
dicSTROKE={}
#dicSTROKE[ModbusID.TELE_SERV_INNER] = STROKE_INNER
dicSTROKE[ModbusID.TELE_SERV_MAIN] = STROKE_SERVE_TELE
dicSTROKE[ModbusID.BAL_ARM1] = LENGTH_ARM1
dicSTROKE[ModbusID.BAL_ARM2] = LENGTH_ARM2
#dicSTROKE[ModbusID.TELE_BALANCE] = STROKE_BAL_TELE
dicSafety = {}
dicWeightBal = {}
dicWeightBalTmp = LoadJsonFile(strFileWeightBal)

#EPCNodeInfo = LoadJsonFile(strFileEPC_node)
EPCNodeInfo = GetEPCNodeInfoDic()
if dicWeightBalTmp is not None:
#Key Value 를 모두 정수로 변경한다
  dicWeightBal = numeric_dict = {int(k): int(v) for k, v in dicWeightBalTmp.items()}
merged_pc2 = PointCloud2()
dicRFIDTmp = {}
dicEPC_last = {}
dicInv_last = {}
lsNodeHistory = []
spdMode = BLB_CMD_MODE.FAST
node_id = 0
position = Pose2D(x=0.0, y=0.0)
trajectory = []
table_positions = []
robot= Robot()
#getattr(robot, Robot_Event.start_calibration_tray.name)()
#robot.trigger_start_calibration_tray()
#print(robot.get_current_state())
#print(dicWeightBal)
print(os.path.splitext(os.path.basename(__file__))[0],getDateTime())