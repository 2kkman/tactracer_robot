import roslibpy
from Util import *
from UtilAlarm import *
from UtilTTS import *
from std_srvs.srv import *
from rospy_tutorials.srv import *
from turtlesim.srv import *
from tta_blb.srv import *
from std_msgs.msg import Header, String
from scipy.spatial import *
import ros_numpy
minGYRO = -90
maxGYRO = 45

NODE_KITCHEN = 1
NODE_CROSS = 10
NODES_SPECIAL = [NODE_KITCHEN,NODE_CROSS]

HTTP_COMMON_PORT=6001
WEIGHT_OCCUPIED = 200
HOME_TABLE='H1'
HOME_CHARGE='P1'
DATETIME_OLD = datetime(2013, 2, 18, 23, 0, 30)
ACC_DECC_SMOOTH = 800
ACC_DECC_LONG = 3000
CAM_OFFSET_TOP_X = 26
CAM_OFFSET_TOP_Y = -85

class IPList(Enum):
    BLB_SVR_IP = '172.30.1.4'
    BLB_RFID_IP = '172.30.1.29'
    BLB_ANDROID_IP = '172.30.1.22'
    BLB_CHARGE_IP = '172.30.1.36'
    BLB_CHARGERPLUG_IP = '172.30.1.26'
    BLB_CROSS_IP = '172.30.1.16'
    BLB_CROSSPLUG_IP = '172.30.1.25'
    BLB_SVR_PORT = '4041'
    
def getConfigPath(host=None) -> str:
    include_path = GetUbutuParam(UbuntuEnv.SCR_DIR.name)
    if host is None:
        host = GetUbutuParam(UbuntuEnv.CONFIG.name)
    config_dir = "common"
    if host == UbuntuEnv.ITX.name:
        config_dir = "carrier_itx"
    elif host == UbuntuEnv.QBI.name:
        config_dir = "tray_qbi"
    strFilePath = f"{include_path}/../bumblebee/config/{config_dir}"
    return strFilePath
# Get base directory path
dirPath = getConfigPath(UbuntuEnv.ITX.name)

# Main configuration files
csvPathNodes = f'{dirPath}/node_info.csv'
csvPathalarm = f'{dirPath}/history_alarm.csv'
csvPathInfo = f'{dirPath}/history_info.csv'
strFileCross = f"{dirPath}/CROSS.txt"
strFileTableNode = f"{dirPath}/DF_EPC_TABLE_TEMP.txt"
strFileTableNodeEx = f"{dirPath}/DF_EPC_TABLE.txt"
strFileShortCut = f"{dirPath}/SHORTCUT.txt"
strFileShortCutTemp = f"{dirPath}/SHORTCUT_Temp.txt"
machineName = 'ITX' if get_hostname().find(UbuntuEnv.ITX.name) >= 0 else 'DEV'
filePath_IPSetup = f"{dirPath}/IPSetup_{machineName}.txt"
try:
    with open(filePath_IPSetup, "r") as f:
        ip_dict = json.load(f)
except Exception as e:
    ip_dict = {item.name: item.value for item in IPList}

# Distance related files
strFileDistanceV = f"{dirPath}/DISTANCE_V.txt"
strFileDistanceArm = f"{dirPath}/DISTANCE_Arm.txt"
strFileDistanceBal = f"{dirPath}/DISTANCE_Balance.txt"

# Node and rail information
strCSV_NodeLoc = f"{dirPath}/node_Loc.txt"
strCSV_TableLoc = f"{dirPath}/table_Loc.txt"
strCSV_RrailInfo = f"{dirPath}/node_Rrail.txt"
strCSV_NodeInfo = f"{dirPath}/node_csv.txt"
strPNG_NodeInfo = f"{dirPath}/node_csv.png"
strPNG_NodeLoc = f"{dirPath}/node_Loc.png"

# Task and chain information
strCSV_TaskChain = f"{dirPath}/TaskChain.txt"

# Temporary and processing files
strJsonScanInfo = f"{dirPath}/temp_scan.txt"
strJsonGraphInfo = f"{dirPath}/temp_graph.txt"
strJsonLinkPosInfo = f"{dirPath}/temp_link.txt"
strMixedDF = f"{dirPath}/temp_Mixed.txt"
strRecvDF = f"{dirPath}/temp_recv.txt"
strGetSeq = f"{dirPath}/temp_getseq.txt"

# Reference and configuration tables
strFileAruco = f"{dirPath}/Table_aruco.txt"
strFileEPC_Info = f"{dirPath}/EPC_INFO.txt" #사용하지 않는다
strFileEPC_node = f"{dirPath}/epcNode.txt"
strFileEPC_total = f"{dirPath}/DF_EPC_NODE.txt"
strFileEPC_scan = f"{dirPath}/DF_EPC_SCAN.txt"
strFileTableSpd = f"{dirPath}/DF_SPEEDTABLE_ITX.txt"
strFileWeightBal = f"{dirPath}/WEIGHTBAL.txt"
strFileLastPos = f"{dirPath}/LAST_POSITION.txt"

BLB_SVR_IP_DEFAULT=ip_dict[IPList.BLB_SVR_IP.name]
BLB_RFID_IP=ip_dict[IPList.BLB_RFID_IP.name]
BLB_ANDROID_IP_DEFAULT=ip_dict[IPList.BLB_ANDROID_IP.name]
BLB_CHARGE_IP = ip_dict[IPList.BLB_CHARGE_IP.name]
BLB_CHARGERPLUG_IP_DEFAULT=ip_dict[IPList.BLB_CHARGERPLUG_IP.name]
BLB_CROSS_IP_DEFAULT=ip_dict[IPList.BLB_CROSS_IP.name]
BLB_CROSSPLUG_IP_DEFAULT=ip_dict[IPList.BLB_CROSSPLUG_IP.name]
BLB_SVR_PORT_DEFAULT=4041

WEIGHT_ISNOT_EMPTY = 100
CAM_LOCATION_MARGIN_OK = 0.02
CAM_LOCATION_MARGIN_FINE = 0.1
PATH_RECORDING = '/root/Downloads/'
MOVE_H_SAMPLE_PULSE = 1200000
MOVE_H_SAMPLE_MM = 2820
MARKER_X_RATE = 100*(0.899221 - 0.0065129)/60.0
MARKER_Y_RATE = 100*(0.545958 + 0.340437)/60.0

MODBUS_WRITE_DELAY = 0.001
MODBUS_READ_DELAY = 0.002
MODBUS_EXCEPTION_DELAY = 0.01
MODBUS_EXECUTE_DELAY_ms = 500
PULSE_POTNOT_MARGIN = 500

ROS_TOPIC_QUEUE_SIZE = 100
LIDAR_MIN_Y = -0.23
LIDAR_MAX_Y = 0.27
SPD_TRAY_MARKER_SCAN = 700

MAX_ANGLE_BLBBODY = 360
MAX_ANGLE_TRAY = 360
SPEED_RATE_H = 1.0
SPEED_RATE_ARM = 0.6
DEFAULT_RPM_MIN = 2
DEFAULT_RPM_SLOW = 300
DEFAULT_RPM_SLOWER = 100
MAINROTATE_RPM_SLOWEST = 20
DEFAULT_RPM_MID = 500
DEFAULT_RPM_NORMAL = 1000
DEFAULT_RPM_FAST = 1500
DEFAULT_SPD_MAX = 2000
DEFAULT_SPD_LIMIT = 3000
DEFAULT_SPD_BAL2EXT = 500
DEFAULT_SPD_BAL1EXT = 150
DEFAULT_SPD_BAL2RETURN = 300
DEFAULT_SPD_BAL1RETURN = 500
ALARM_RPM_LIMIT = 100000 #현재 2관절 기준으로 1관절과 매칭하기 위한 RPM 속도가 이것보다 높은 경우 알람발생.
DOOROPEN_PERCENT = 0.6
TILT_TIMING = 50000
ALMOST_ZEROINT = 1

DEFAULT_ACC = 500
DEFAULT_DECC = 500
EMERGENCY_DECC = 50
TORQUE_LIMIT_DEFAULT = 300

LENGTH_ARM1 = 800
LENGTH_ARM2 = 800
STROKE_SERVE_TELE = 1035
#STROKE_SERVE_TELE = 1700
STROKE_SERV_DEFAULT = 560
STROKE_INNER = 1
STROKE_SERVE_TOTAL = STROKE_SERVE_TELE+STROKE_INNER
STROKE_BAL_EXTEND = LENGTH_ARM1+LENGTH_ARM2
STROKE_BAL_TELE = 500
STROKE_BAL_TOTAL = STROKE_BAL_EXTEND+STROKE_BAL_TELE
PULSES_PER_ROUND = 10000
INNERSTEP_PULSE_TRAYMOVE = 110000
WEIGHT_BALARM_GRAM = 50000
WEIGHT_SERVARM_GRAM = 54500
WEIGHT_LOADCELL_LIMITGRAM = 15000
RSSI_FILTER = 28

IP_MASTER = GetMasterIP()
#EPCNodeInfo = LoadJsonFile(strFileEPC_node)
isRealMachine = get_hostname().find(UbuntuEnv.ITX.name) >= 0

# 상수 정의
STROKE_MAX = 345000
STROKE_MM = 1700
STROKE_MIN = -200000

SERVING_ANGLE_MAX = 649800
MARKER_ANGLE_MAX = 833146
ANGLE_FULL = 360

class DataKey(Enum):
    SSID = auto()
    BSSID = auto()
    LinkSpeed = auto()
    RSSI = auto()
    Brightness = auto()
    longitude = auto()
    latitude = auto()
    LocationAddress = auto()
    ISSCREEN_ON = auto()
    SVRIP = auto()
    INTERVAL_DATA = auto()
    TAGSCAN = auto()
    LIGHT = auto()
    ACTIVATED_CAM = auto()
    Fx = auto()
    Fy = auto()
    IMG_WIDTH = auto()
    IMG_HEIGHT = auto()
    proximity = auto()
    lightlux = auto()
    AndroidIP = auto()
    Angle_X = auto()
    Angle_Y = auto()
    Angle_Z = auto()
    CPU_USAGE = auto()
    MEMORY_USAGE = auto()
    MEMORY_USED = auto()
    MEMORY_TOTAL = auto()
    NETWORK_TX = auto()
    NETWORK_RX = auto()
    TEMPERATURE_DEGREE = auto()
    MOVE_LEVEL = auto()

    @classmethod
    def from_string(cls, key: str) -> Optional['DataKey']:
        try:
            return cls[key]
        except KeyError:
            return None
        
class APIBLB_STATUS_TASK(Enum):
    NONE = 0
    Ideal = 1
    Started = 2
    Running = 3
    Paused = 4
    Failed = 5
    Completed = 6
    Canceled = 7
    NotFound = 404

'''
1- "Ideal",2-Started,3-Running,4-target_reached,5- Tray_moving_down,
6-Tray_Rack_opening,7 Food_Served_Confirmed,8-Tray_Moving_Up,
9-Tray_mounted_to_bumblebee,10-Canceled,11-Paused,12-Telescope_Expanding,
13-Telescope_Shrinking,14-Main_Rotation 
'''
class APIBLB_STATUS(Enum):
    Charging = 0
    Ideal = 1
    Started = 2
    Running = 3
    target_reached = 4
    Tray_moving_down = 5
    Tray_Rack_opening = 6
    Food_Served_Confirmed = 7
    Tray_Moving_Up = 8
    Tray_mounted_to_bumblebee = 9
    Canceled = 10
    Paused = 11
    Telescope_Expanding = 12
    Telescope_Shrinking = 13
    Main_Rotation = 14
    Wait = 15
    
class BLB_STATUS_FIELD(Enum):
    Started = 2
    N_A = auto()  #not determined
    ROTATING_MAIN = auto()  #14-MAIN ROTATION
    IDEAL = auto()  #1 - IDEAL
    CONFIRM = auto()  # 사용자의 터치를 기다리는 상태
    MOVING = auto()  # 레일주행중 3- running
    LIFTING_DOWN = auto()  # 도킹 다운 5- tray down
    LIFTING_UP = auto()  # 리프팅 업 8- tray up
    DOOR_MOVING = auto()  # 도어 움직이는중
    READY = auto()  # 이동 명령어는 수신하였지만 UI입력 대기중
    WAITING = auto()  # 분기기 제어완료 대기중
    WAITING_CROSS = auto()  # 분기기 제어완료 대기중
    OBSTACLE_DETECTED = auto()  #장애물 감지
    HOMING = auto()  # 작업 완료 후 복귀중
    CHARGING = auto()  # 충전중
    INIT = auto()  # 초기화 요청
    EXPANDING = auto()  # 암 전개 12 - telescope expanding
    FOLDING = auto()  # 암 수축 13 - telescope shringking
    TRAY_RACK_OPENED = auto()   #6-서빙이 완료된 후 멈춰 있는 상태 (문이 열린 상태)
    PAUSED = auto() #11-Paused
    CANCELLED = auto()  #10-Canceled

class CALLBELL_FIELD(Enum):
    BTN_RED = "I1"
    BTN_BLUE = "I2"
    BTN_MAGNET = "I3"
    CALL_STATE = "O1"
    CALLBELL = auto()
    ID = auto()  # 호출벨ID (보통 테이블ID)
    IP = auto()  # 호출벨IP주소
    MAC = auto()  # 호출벨 맥어드레스
    TIMESTAMP = auto()

class BLB_OP_FIELD(Enum):
    MANUAL = auto()  # 수동운전중
    MANUAL_AUTOSTOP = auto()  # 수동운전 + RFID태그 감지시 자동정지.
    MANUAL_UPDATE = auto()  # 수동운전 + RFID태그 감지시 위치값 업데이트
    MANUAL_UPDATE_AUTOSTOP = (
        auto()
    )  # 수동운전 + RFID태그 감지시 자동으로 멈추며 위치값 업데이트
    ON_BATCH = (
        auto()
    )  # 자동운전, 기존태그 감지시 위치값 업데이트 + 새로운 태그 감지시 태그 등록

class MAIN_STATUS(Enum):  # 컨트롤 센터에서 발행하는 범블비 상태 메세지
    DETECTED_HEIGHT=auto()


class CARRIER_STATUS(Enum):  # 컨트롤 센터에서 발행하는 범블비 상태 메세지
    I_TRAY_2_BOTTOM = auto()
    I_TRAY_2_HOME = auto()
    I_TRAY_2_TOP = auto()
    I_DOOR_1_BOTTOM = auto()
    I_DOOR_1_HOME = auto()
    I_DOOR_1_TOP = auto()
    I_DOOR_2_BOTTOM = auto()
    I_DOOR_2_HOME = auto()
    I_DOOR_2_TOP = auto()
    I_LIMIT_BOTTOM = auto()
    I_LIMIT_TOP = auto()
    LOAD1 = auto()
    LOAD2 = auto()
    GOR_SVN = auto()
    GAV_SVN = auto()
    GLA_SVN = auto()
    GAV_TUN = auto()
    TEMPN = auto()
    O_V5_NC = auto()
    O_V12_NC = auto()
    L0 = auto() #LED 상태 정보
    L1 = auto()
    TILT_ANGLE = auto()
    SHAKE_TRAY = auto()

class LED_STATUS(Enum):  # 현재 트레이 도어 상태 메세지
    COLOR_CODE = auto()
    BLINK_TIME = auto()

class TRAYDOOR_STATUS(Enum):  # 현재 트레이 도어 상태 메세지
    OPENED = auto()
    CLOSED = auto()
    MOVING = auto()
    DOORALARM = auto()


class BLB_STATUS(Enum):  # 캐리어 상태 메세지
    ID = auto()
    STATUS = auto()
    NODE_CURRENT = auto()
    TABLE_CURRENT = auto()
    NODE_TARGET = auto()
    # @classmethod
    # def from_value(cls, value):
    #     # 만약 value가 문자열이라면 정수로 변환 시도
    #     if isinstance(value, str):
    #         try:
    #             value = int(value)
    #         except ValueError:
    #             raise ValueError(f"Invalid literal for int() with base 10: '{value}'")
                
    #     for member in cls:
    #         if member.value == value:
    #             return member
    #     raise ValueError(f"No member found for value: {value}")

class BLB_CMD(Enum):
    """
    Enum class to define various Bumblebee command parameters.

    Attributes:
        ID: Device ID
        TRAY_A: First serving table number
        TRAY_B: Second serving table number
        LEVEL: Tray open/close height
        STATE: Device status parameter
        TIME: Command ID (timestamp)
        MODE: Bumblebee's speed mode definition

    # ID:BumbleBee1`TRAY_A:4`TRAY_B:93`LEVEL:-3`STATE:move1`TIME:0 - 첫번째 테이블 4, 두번째 테이블 93 으로 서빙 지시
    # ID:BumbleBee1`TRAY_A:4`TRAY_B:93`LEVEL:-3`STATE:CONFIRM`TIME:0 - 사용자가 제품을 꺼내고 확인버튼을 누름
    """
    ID = auto()  # Device ID
    TRAY_A = auto()  # 첫번째로 서빙할 테이블 번호
    TRAY_B = auto()  # 2번째로 서빙할 테이블 번호
    LEVEL = auto()  # 트레이 개폐 높이
    STATE = auto()  # Device Status Param
    TIME = auto()  # 명령어ID (timestamp)
    MODE = auto()  # 범블비 동작 속도 모드 정의



class BLB_CMD_MODE(Enum):
    """
    Enum class to define Bumblebee's speed modes.
    
    Attributes:
        SLOW: Low speed mode
        NORMAL: Medium speed mode
        FAST: High speed mode
    """
    SLOWEST = 0.1   # 초저속
    SLOW = 0.33   # 저속
    NORMAL = 0.66   # 중속
    FAST = 1   # 고속

class BLB_CMD_CUSTOM(Enum):
    PROFILE = auto()  # Config 폴더에 있는 모터 지시정보 (Json) 파일을 읽어 동작시킨다
    OBS = auto()  # 장애물 탐지 테스트 명령어
    PC = auto() #포인트 클라우드 임의 생성 명령어
    ROTATE = auto() #회전 테스트 명령어.
    TABLE = auto() #테이블 이동 명령어.


class BLB_CMD_STATUS(Enum):
    MOVE = auto()
    CONFIRM = auto()
    INIT = auto()
    EVENT = auto()
    MOVEARM = auto()
    ESTOP = auto()
    STOP_SMOOTH = auto()
    PAUSE = auto()
    CANCEL = auto()
    RESUME = auto()
    RELOAD_SVR = auto()

class TRAY_ARD_Field(Enum):
    """
    Enum class to define message fields for Arduino communication.

    Attributes:
        GLA_SVN: Linear acceleration
        GAV_SVN: Angular velocity
        TEMPN: MPU9250 temperature
        GAV_TUN: Sum of absolute values of 3-axis acceleration
        GOR_SVN: Orientation (roll, pitch, yaw)
        TEMPAHT_N: AHT20 temperature
        HUMIDAHT_N: AHT20 humidity
        PARTICLEN: Fine dust levels (RED/IR/GREEN)
        RESULTN: Command processing result
    """
    GLA_SVN = auto()  # linear_acceleration
    GAV_SVN = auto()  # angular_velocity
    TEMPN = auto()  # mpu9250 온도
    GAV_TUN = auto()  # 가속도 3축 합산 절대값
    GOR_SVN = auto()  # orientation -> roll, pitch, yaw
    TEMPAHT_N = auto()  # AHT20 온도
    HUMIDAHT_N = auto()  # AHT20 습도
    PARTICLEN = auto()  # 미세먼지 (RED/IR/GREEN)
    RESULTN = auto()  # 명령어처리결과

class BLB_UI_EVENTS(Enum):
    up_pressed = auto()
    up_released = auto()
    down_released = auto()
    down_pressed = auto()
    level_reset = auto()
    A_pressed = auto()
    B_pressed = auto()
    A_released = auto()
    B_released = auto()


class EventLidar(
    Enum
):  # data: "boxCenter: 0.714,1.189,-0.000 width: 0.37 height: 0.16 depth: 0.00"
    boxcenter = (
        auto()
    )  #:(클러스터링박스좌표m단위)width,height,0 (2D라이다니까 depth 값은 무조건 0 고정)
    width = auto()  # 클러스터링박스 너비
    height = auto()  # 클러스터링박스 높이
    depth = auto()  # 클러스터링박스 깊이
    distance = auto()

class LidarCropProfile(Enum):
  MOTOR_H = auto()
  MOTOR_V = auto()
  SERVING_ARM = auto()
  CHECK_GROUND = auto()
  

class EndPoints(Enum):
    JOG = auto()  # RFID조그 운전
    alarm = auto()  # 알람정보 수신
    info = auto()  # info 정보 수신 (디버그용)
    control = auto()  # KEEP_ALIVE 메세지 발행 토픽
    ARD = auto()  # 아두이노 제어

class TopicName(Enum):
    CMD_DEVICE = auto()  # 모드버스 제어 커맨드 토픽.
    KEEPALIVE = auto()  # KEEP_ALIVE 메세지 발행 토픽
    MB_ = auto()  # 모드버스 장비별 모니터링 메세지
    ACK = auto()  # 모터 구동 ACK 메세지
    BLB_STATUS = auto() # 제어센터에서 발행하는 커맨드 처리 현황. (map string 으로 전달)    
    BLB_STATUS_HTTPS = auto() # HTTPS 에서 발행하는 커맨드 처리 현황 (임과장에게 전달되는 맵)
    RFID = auto()  # KLM900 RFID 리더에 읽힌 태그값
    RFID_DF = auto()  # RFID 태그값과 위치 및 시간 정보 매핑
    BLB_CMD = auto()  # 범블비 UI에서 발행하는 커맨드. (map string)
    SEND_MQTT = auto()  # MQTT 메세지를 ROS상에서 발행하기 위한 토픽.
    RECEIVE_MQTT = auto()  # ROS 및 HOME ASSISTANT 에서 발행된 메세지는 여기로 들어감.
    ARUCO_RESULT = auto()  # node_CamBase 에서 발행하는 QR 파싱 결과.
    TOPIC_LIST = auto()  # 현재 발행되고 있는 토픽 리스트
    ARD_CARRIER = auto()  # 캐리어 아두이노
    HISTORY_ALARM = auto()  # 알람히스토리
    HISTORY_INFO = auto()  # 인포히스토리
    IMU = auto()  # 캐리어에 장착된 자이로
    BMS = auto()  # BMS
    BLB_TRAY_IMU = auto()  # 트레이에 장착된 자이로
    BLB_TRAY_TEMPC = auto()  # 트레이에 장착된 온도센서
    BLB_TRAY_HUMID = auto()  # 트레이에 장착된 습도센서
    TABLE_ORDER_STATUS = auto()  # 테이블 오더 상태 (테이블별 장바구니 상태)
    TABLE_ORDER_REQ = auto()  # 테이블별 오더요청(장바구니에 상품추가)
    SD = auto()
    BLB_STATUS_MONITOR = auto()  # 범블비 상태 모니터링 정보
    BLB_STATUS_MONITOR_BMS_NTP = auto()  # 범블비 BMS 및 온도 모니터링 정보
    BLB_STATUS_MONITOR_BOOT = auto()  # 범블비 부팅상태 모니터링 정보
    RESONANT_TRAY = auto()  # 트레이 공진 정보
    OBS = auto()  #노드주변 방해물 정보
    BLB_JC = auto()  #범블비 분기기에서 발행하는 상태 정보
    BLB_JC_CMD = auto()  #범블비 분기기 제어명령어 송신 토픽
    ANDROID = auto()
    MOTOR_POS = auto()
    ALARM_STATUS = auto()
    detect_3D = auto()
    ROS_CONFIG = auto()
    JOB_DF = auto()
    JOB_LIST = auto()
    JOB_PATH = auto()
    CROSS_INFO = auto()
    SMARTPLUG_INFO = auto()

class SMARTPLUG_INFO(Enum):
    GPI1_CHARGE = auto()
    CHARGERPLUG_STATE = auto()
    SET_CHARGERPLUG = auto()
    SET_CROSSPLUG = auto()
    
class SeqMapField(Enum):
    START_NODE = auto()  # 출발지 노드
    CROSS_STATUS = (
        auto()
    )  # 출발/도착 노드를 구분하지 않고 출발 전 일괄제어 하는 경우에 사용.
    START_STATUS = auto()  # 출발전 현재 위치의 제어기(크로스, 리프트 등) 세팅값.
    END_NODE = auto()  # 도착지 노드
    END_STATUS = auto()  # 도착지 진입을 위해 해당 제어기가 세팅되어야 할 값
    DIRECTION = (
        auto()
    )  # True 전진 False 후진. (보통 기존에 왔던 노드를 다시 돌아가는 경우에는 후진.)
    DISTANCE = auto()  # 대략적인 주행 거리(엔코더값)

class TableInfo(Enum):
    LINK_ID = auto()  # 이 테이블이 어느구간에 속해있는지 연동된 링크 아이디.
    TABLE_ID = auto()
    NODE_ID = auto()  #추후 LINK_ID 로 대체하고 사라질 예정
    SERVING_ANGLE = auto()
    MARKER_ANGLE = auto()
    SERVING_DISTANCE = auto()
    MOVE_DISTANCE = auto()
    HEIGHT_LIFT = auto()
    MARKER_VALUE = auto()

class ARUCO_RESULT_FIELD(Enum):
    IS_MARKERSCAN = auto()  # 마커 스캔 동작 여부
    IS_IMGPUBLISH = auto()  # 카메라 이미지 발행 여부
    CAM_ID = auto()  # 캠인덱스 번호 (/dev/video?)
    DIFF_Y = auto()  # 마커중심이 센터와 Y축으로 차이나는 비율
    DIFF_X = auto()  # 마커중심이 센터와 X축으로 차이나는 비율
    X = auto()  # 3D거리-X 마커와 카메라가 좌우로 떨어진 거리 (렌즈에서 우측으로 가면 +)
    Y = auto()  # 3D거리-Y 마커와 카메라가 상하로 떨어진 거리. (렌즈에서 위로 가면 +)
    Z = auto()  # 3D거리-Z 마커와 카메라와 직선으로 떨어진거리
    XX = auto()  # 3D거리-X 마커와 카메라가 좌우로 떨어진 거리 (렌즈에서 우측으로 가면 +)
    YY = auto()  # 3D거리-Y 마커와 카메라가 상하로 떨어진 거리. (렌즈에서 위로 가면 +)
    ZZ = auto()  # 3D거리-Z 마커와 카메라와 직선으로 떨어진거리
    MARKER_VALUE = auto()  # 마커값 (0~255)
    ANGLE = auto()  # 마커의 회전각도
    LASTSEEN = auto()   #아르코마커 수신시각 (마스터에서 기록)

class EPCINFO_FIELDS(Enum):
    TAG_ID = auto()
    TAG_TYPE = auto()
    TAG_DIRECTION = auto()
    LINK_ID = auto()
    TAG_X = auto()
    TAG_Y = auto()

class EPCINFO_TAG_DIRECTION(Enum):
    POSITIVE = 1
    BOTH = 0
    NEGATIVE = -1

class EPCINFO_TAG_TYPE(Enum):
    D = auto()
    S = auto()
    P = auto()

class SPEEDTABLE_FIELDS(Enum):
    ACC_CW = auto()
    DECC_CW = auto()
    ACC_CCW = auto()
    DECC_CCW = auto()
    SPD = auto()
    
class APIBLB_REPLY_YESNO(Enum):
    yes = auto()
    no = auto()
    Y = auto()
    N = auto()

class DISTANCE_V(Enum):
    pulseV = auto()
    distanceLD = auto()
    distanceSTD = auto()
    timestampLD = auto()
    aruco_Z = auto()

reply_yes = APIBLB_REPLY_YESNO.yes.name
reply_no = APIBLB_REPLY_YESNO.no.name
reply_Y = APIBLB_REPLY_YESNO.Y.name
reply_N = APIBLB_REPLY_YESNO.N.name

class APIBLB_FIELDS_ACTION(Enum):
    up = auto()
    down = auto()
    half = auto()
    full=auto()
    fold=auto()
    open=auto()
    close=auto()
    pause=auto()
    resume=auto()
    cancel=auto()
    home=auto()
    target = auto()
    forward = auto()
    backward = auto()
    chainlist = auto()

class APIBLB_FIELDS_NAVI(Enum):
    taskid   = auto()
    robotips   = auto()
    current_taskmst   = auto()
    current_taskdtl   = auto()
    start_station   = auto()
    current_station   = auto()
    endnode   = auto()
    orderstatus   = auto()
    simulayoutcd=auto()
    #orderstatus=auto()
    workstatus=auto()
    initialstation = auto()
    finalstation   = auto()

class APIBLB_TASKTYPE(Enum):
  HomeEmergencyCall = 0
  Parking = 1
  ServingTask=2
  CashPay=3
  CollectingEmptyPlattes=4
  ReturnHomeAfteReachTask=5
    
class LightColor(Enum):
  OFF = 0
  RED = 1
  GREEN=2
  BLUE=3
  WHITE=4
  ORANGE=5
  
class TraySector(Enum):
  Cell1 = 0
  Cell2 = 1

class LightBlink(Enum):
  Solid = 0
  Normal = 1000
  Fast = 200
    
class APIBLB_FIELDS_ALARM(Enum):
    ipaddresss   = auto()
    alarmcodes   = auto()
    alarmmsgs   = auto()
    commentss = auto()

class APIBLB_FIELDS_TASK(Enum):
    taskids   = auto()
    taskid   = auto()
    robotips   = auto()
    mastercode   = auto()
    detailcode   = auto()
    detailcode_list   = auto()
    direction   = auto()
    distance   = auto()
    distance_total   = auto()
    distance_list   = auto()
    #simulayoutcd   = auto()
    startnode   = auto()
    endnode = auto()
    endnode_list   = auto()
    ordertype   = auto()
    orderstatus   = auto()
    workstatus   = auto()
    #comments   = auto()
    nodetype = auto()
    POS_ABS = auto()
    tasktype = auto()
    tasktypes = auto()
    updateduser = auto()
    taskrunok=auto()
    trayrack=auto()
    trayracks=auto()
    workname=auto()
    worknames=auto()
    prioritys = auto()
    emergencys = auto()
    sortorders = auto()
    commentss = auto()
    UpdatedBys = auto()
    SPstatus = auto()
    rackcode = auto()
    extraserviceruns=auto()
    
key_distance = APIBLB_FIELDS_TASK.distance.name    
key_detailcode = APIBLB_FIELDS_TASK.detailcode.name    
key_workstatus = APIBLB_FIELDS_TASK.workstatus.name
key_startnode= APIBLB_FIELDS_TASK.startnode.name
key_endnode= APIBLB_FIELDS_TASK.endnode.name
    
class MAP_ARUCO(Enum):
    ANGLE   = auto()
    MARKER_VALUE   = auto()
    CUR_POS   = auto()    
    DISTANCE_FROM_HOME   = auto()    
    
    

class APIBLB_RAILTYPE(Enum):
    LS = auto() #Logshape R-rail
    ES = auto() #Exponetial shape R-rail
    N = auto()  #Normal Rail

class APIBLB_NODETYPE(Enum):
    H = auto() #HOME
    P = auto() #PARKING
    R = auto()  #ROTATION (JUNCTION)
    N = auto()  #Normal

#APIBLB_NODETYPE.H.name = APIBLB_NODETYPE.H.name
#APIBLB_NODETYPE.P.name = APIBLB_NODETYPE.P.name
#APIBLB_NODETYPE.R.name = APIBLB_NODETYPE.R.name
#APIBLB_NODETYPE.N.name = APIBLB_NODETYPE.N.name
RailType_LS = APIBLB_RAILTYPE.LS.name
RailType_ES = APIBLB_RAILTYPE.ES.name
RailType_NORMAL = APIBLB_RAILTYPE.N.name
    
class APIBLB_FIELDS_INFO(Enum):
    angle = auto()
    angle_marker = auto()
    x = auto()
    y = auto()
    vx = auto()
    vy = auto()
    w = auto()
    start = auto()
    end = auto()
    distance = auto()
    speed = auto()
    direction = auto()
    nodetype = auto()
    railtype = auto()
    rotation = auto()
    tableno = auto()
    time = auto()
    st_xval = auto()
    st_yval = auto()
    et_xval = auto()
    et_yval = auto()
    tb_xval = auto()
    tb_yval = auto()
    
class APIBLB_FIELDS_STATUS(Enum):
    onlineStatus = auto()
    robot_status_code = auto()
    robot_status = auto()
    R1 = auto()
    R2 = auto()
    robot_charge_node = auto()
    charging = auto()
    auto_charging = auto()
    manual_charging = auto()
    battery_level = auto()
    battery_temp = auto()
    voltage = auto()
    watt = auto()
    error_code = auto()
    alarm_msg = auto()
    df = auto()
    
class APIBLB_METHODS_GET(Enum):
    robot_online_status = auto()
    robot_loc_info = auto()
    robot_speed_inf = auto()
    robot_status= auto()
    Robot_trayload_status = auto()
    robot_battery_status = auto()
    robot_node_info = auto()
    robot_table_info = auto()
    robot_alarm_status = auto()
    robot_navigation_info = auto()
    robot_TaskChain_ListRequest = auto()
    robot_TrackRackUpdate_action = auto()
    robot_serviceRunokno_action = auto()

class APIBLB_METHODS_POST(Enum):
    robot_task_action = auto()
    robot_move_action = auto()
    robot_expand_action = auto()
    robot_activity_action = auto()
    robot_clearalarm_action = auto()
    robot_CurrTask_info  = auto()
    robot_Current_node_info = auto()
    robot_TaskChain_action = auto()
    robot_topic_publish = auto()

# Function with added error handling for mismatches between dictMap and dictCross, and disconnected nodes
class GraphError(Exception):
    pass

class MismatchError(GraphError):
    def __init__(self, message="Mismatch between dictMap and dictCross"):
        self.message = message
        super().__init__(self.message)

class DisconnectedNodeError(GraphError):
    def __init__(self, message="There are disconnected nodes"):
        self.message = message
        super().__init__(self.message)


def generate_table_info(new_csv_path: str, table_info_path: str):
    # CSV 로드
    df = pd.read_csv(new_csv_path, sep='\t')

    # T0 → 0 형식으로 TABLE_ID 정수화
    df[TableInfo.TABLE_ID.name] = df[TableInfo.TABLE_ID.name].str.replace("T", "", regex=False).astype(int)

    # SERVING_DISTANCE 계산
    #df["SERVING_DISTANCE"] = ((df["MB_11"] + abs(STROKE_MIN)) * STROKE_MM / (STROKE_MAX - STROKE_MIN)).round().astype(int)
    print(df["MB_11"])
    #df["SERVING_DISTANCE"] = ((df["MB_11"] - STROKE_MIN) * STROKE_MM / (STROKE_MAX - STROKE_MIN)).round().astype(int)
    df[TableInfo.SERVING_DISTANCE.name] = ((df["MB_11"] + 200000) * STROKE_MM / (STROKE_MAX - STROKE_MIN)).round().astype(int)

    # SERVING_ANGLE 계산
    df[TableInfo.SERVING_ANGLE.name] = (df["MB_27"] * ANGLE_FULL / SERVING_ANGLE_MAX).round().astype(int)

    # MARKER_ANGLE 계산
    df[TableInfo.MARKER_ANGLE.name] = (df["MB_31"] * ANGLE_FULL / MARKER_ANGLE_MAX).round().astype(int)

    # LIFT_V 계산
    df[TableInfo.HEIGHT_LIFT.name] = df["MB_6"]

    # LIFT_V 계산
    df[TableInfo.MARKER_VALUE.name] = -1

    # 최종 테이블 구성
    table_info_df = df[[TableInfo.TABLE_ID.name, TableInfo.NODE_ID.name, TableInfo.SERVING_DISTANCE.name, TableInfo.SERVING_ANGLE.name,TableInfo.MARKER_ANGLE.name,TableInfo.HEIGHT_LIFT.name,TableInfo.MARKER_VALUE.name]]

    # 저장
    table_info_df.to_csv(table_info_path, index=False, sep='\t')
    print(f"테이블 정보가 '{table_info_path}' 경로에 저장되었습니다.")

def generate_node_graph_from_csv(csv_path: str, save_path: str):
    # 원본 데이터 불러오기
    full_df = pd.read_csv(csv_path, sep='\t')

    # MB_15 가 0 또는 NaN인 행은 기본값 10으로 대체
    full_df["MB_15"] = full_df["MB_15"].fillna(0)
    full_df.loc[full_df["MB_15"] == 0, "MB_15"] = 10

    # MB_15 기준 정렬
    df = full_df.copy()
    df = df.sort_values("MB_15").reset_index(drop=True)

    # 노드 정보 초기화
    current_node = 10  # 시작 노드
    next_node = 11     # 11번부터 새 노드 시작
    current_pulse = df.loc[0, "MB_15"]
    node_map = {df.loc[0, TableInfo.TABLE_ID.name]: current_node}

    graph = []

    for idx in range(1, len(df)):
        row = df.loc[idx]
        table_id = row[TableInfo.TABLE_ID.name]
        pulse = row["MB_15"]

        delta_pulse = abs(pulse - current_pulse)

        if delta_pulse <= 10000:
            node_map[table_id] = current_node
        else:
            distance_mm = round(delta_pulse * MOVE_H_SAMPLE_MM / MOVE_H_SAMPLE_PULSE)
            graph.append(f"{current_node}\t{next_node}\t{distance_mm}")
            current_node = next_node
            current_pulse = pulse
            node_map[table_id] = current_node
            next_node += 1

    # 그래프 저장
    with open(save_path, 'w', encoding='utf-8') as f:
        f.write("\n".join(graph))

    print(f"노드 그래프를 '{save_path}' 경로에 저장했습니다.")

    # NODE_ID 컬럼 추가
    full_df[TableInfo.NODE_ID.name] = full_df[TableInfo.TABLE_ID.name].map(node_map).fillna("").astype(str)

    # NODE_ID 추가된 CSV 저장
    base, ext = os.path.splitext(csv_path)
    new_csv_path = f"{base}_with_node{ext}"
    #new_csv_path = csv_path
    full_df.to_csv(new_csv_path, sep='\t', index=False)

    print(f"'NODE_ID' 컬럼이 추가된 파일을 '{new_csv_path}' 경로에 저장했습니다.")
    return new_csv_path


def GetDirectionFromAngle(iSERVING_ANGLE):
  if 46 <= iSERVING_ANGLE <= 135:
      direction = 'N'
  elif 136 <= iSERVING_ANGLE <= 225:
      direction = 'W'
  elif 226 <= iSERVING_ANGLE <= 315:
      direction = 'S'
  else:
      direction = 'E'
  return direction


def detect_environment(pc2_msg, config=None):
    """
    PointCloud2 메시지를 분석하여 현재 환경(벽/장애물/열린 공간) 탐지
    
    Parameters:
    - pc2_msg: ROS PointCloud2 메시지
    - config: 탐지 설정 딕셔너리 (선택적)
    
    Returns:
    - 환경 정보 딕셔너리
    """
    # 기본 설정값
    default_config = {
        'distance_threshold': 0.1,   # 포인트 근접성 임계값 (미터)
        'max_distance': 5.0,         # 최대 스캔 거리 (미터)
        'fov_angle': 45,             # 시야각 (도)
        'wall_density_threshold': 0.3,  # 벽으로 판단하는 밀도 임계값
        'obstacle_density_threshold': 0.6  # 장애물로 판단하는 밀도 임계값
    }
    
    # 사용자 설정과 기본 설정 병합
    config = {**default_config, **(config or {})}
    
    # 결과 딕셔너리 초기화
    result = {
        'status': 'unknown',
        'type': None,
        'center_point': None,
        'avg_distance': None,
        'point_density': None
    }
    
    try:
        # PointCloud2를 numpy 배열로 변환
        pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(pc2_msg)
        points = np.array([[p['x'], p['y'], p['z']] for p in pc_array if np.isfinite(p['x'])])
        
        # 포인트가 없으면 종료
        if len(points) == 0:
            return result
        
        # 전방 시야각 내 포인트 필터링
        distances = np.linalg.norm(points, axis=1)
        cos_threshold = np.cos(np.radians(config['fov_angle']))
        
        # 전방 포인트 선택 (각도 및 거리 제한)
        forward_mask = (points[:, 0] / distances > cos_threshold) & (distances < config['max_distance'])
        front_points = points[forward_mask]
        
        # 포인트가 충분하지 않으면 종료
        if len(front_points) < 10:
            return result
        
        # KD-트리를 사용한 밀도 계산
        tree = KDTree(front_points)
        
        # 각 포인트의 이웃 수 계산
        neighbor_counts = [len(tree.query_ball_point(p, config['distance_threshold'])) for p in front_points]
        point_density = np.mean(neighbor_counts) / len(front_points)
        avg_distance = np.mean(front_points[:, 0])
        
        # 중심점 계산
        center_point = np.mean(front_points, axis=0)
        
        # 벽 판단 로직
        if (point_density < config['wall_density_threshold'] and 
            avg_distance > 1.0 and avg_distance < 5.0):
            result['status'] = 'clear'
            result['type'] = 'wall'
        
        # 장애물 판단 로직
        elif (point_density > config['obstacle_density_threshold'] and 
              avg_distance < 1.0):
            result['status'] = 'blocked'
            result['type'] = 'obstacle'
        
        # 결과 업데이트
        result['center_point'] = center_point
        result['avg_distance'] = avg_distance
        result['point_density'] = point_density
        
    except Exception as e:
        rospy.logerr(f"환경 탐지 중 오류 발생: {e}")
    
    return result
  
def is_wall_or_obstacle(pc2_msg, distance_threshold=1.0, density_threshold=1.5):
    dicResult = {"LD_STATUS": "UNKNOWN"}

    # 1. PointCloud2 -> numpy 배열 변환
    pc_np = ros_numpy.point_cloud2.pointcloud2_to_array(pc2_msg)
    points = np.array([[p['x'], p['y'], p['z']] for p in pc_np if np.isfinite(p['x'])])

    # 2. 시야 내 점 필터링 (정면 기준)
    forward_vector = np.array([1, 0, 0])  # 항상 정면(x축 방향)을 바라봄
    distances = np.linalg.norm(points, axis=1)

    # x축(전방) 기준으로 일정 각도(25도) 이내의 점 필터링
    fov_mask = (points[:, 0] / distances) > 0.9  # cos(25°) ≈ 0.9
    front_points = points[fov_mask & (distances < 5.0)]  # 최대 5m 범위

    if len(front_points) == 0:
        return dicResult

    # 3. 벽과 장애물 구분
    tree = KDTree(front_points)
    mean_distance = np.mean(front_points[:, 0])  # x축 거리 평균
    density = np.mean([len(tree.query_ball_point(p, distance_threshold)) for p in front_points]) / len(front_points)

    # 대표 좌표 계산 (중심 좌표)
    center_point = np.mean(front_points, axis=0)

    if mean_distance > 0.5 and density < density_threshold:
        dicResult["LD_STATUS"] = "wall"
    elif mean_distance < 0.5 and density > density_threshold:
        dicResult["LD_STATUS"] = "obstacle"

    dicResult["X"], dicResult["Y"], dicResult["Z"] = center_point

    return dicResult

def get_cardinal_direction(angle):
    if 0 <= angle < 45 or 315 <= angle < 360:
        return "E"
    elif 45 <= angle < 135:
        return "N"
    elif 135 <= angle < 225:
        return "W"
    elif 225 <= angle < 315:
        return "S"
      
def calculate_direction_and_distance(x1, y1, x2, y2):
    # 방향 계산 (각도: 북쪽 기준 시계 방향)
    delta_x = x2 - x1
    delta_y = y2 - y1
    angle = math.degrees(math.atan2(delta_y, delta_x))  # atan2로 각도 계산
    
    # 방향을 북쪽(0도) 기준으로 변환
    if angle < 0:
        angle += 360

    # 거리 계산 (피타고라스 정리)
    distance = math.sqrt(delta_x**2 + delta_y**2)
    
    return angle, distance

# # 예제 좌표
# x1, y1 = 0, 0
# x2, y2 = 3, 4

# # 함수 호출
# direction, distance = calculate_direction_and_distance(x1, y1, x2, y2)

# print(f"방향: {direction:.2f}°")  # 각도 (소수점 2자리)
# print(f"거리: {distance:.2f}")   # 거리 (소수점 2자리)

def API_callBLB(svName = APIBLB_METHODS_GET.robot_battery_status.name,field_Value =None):
    return API_call(svrIP=IP_MASTER, port=9000, baseDir='',serviceName=svName,fieldValue = field_Value)

def API_robot_battery_status():
  bResult,recvData = API_callBLB(svName = APIBLB_METHODS_GET.robot_battery_status.name,field_Value =None)
  recvDataMap = {}
  if bResult and is_json(recvData):
    recvDataMap = json.loads(recvData)
  return bResult,recvDataMap

def API_robot_table_info():
  bResult,recvData = API_callBLB(svName = APIBLB_METHODS_GET.robot_table_info.name,field_Value =None)
  recvDataMap = {}
  if bResult and is_json(recvData):
    recvDataMap = json.loads(recvData)
  return bResult,recvDataMap

def API_robot_node_info():
  bResult,recvData = API_callBLB(svName = APIBLB_METHODS_GET.robot_node_info.name,field_Value =None)
  recvDataMap = {}
  if bResult and is_json(recvData):
    recvDataMap = json.loads(recvData)
  return bResult,recvDataMap

def API_call(svrIP=BLB_SVR_IP_DEFAULT, port=BLB_SVR_PORT_DEFAULT,baseDir = 'api/BeeCoreAPI',serviceName=APIBLB_METHODS_POST.robot_Current_node_info.name, fieldValue=None):
    strResult = ''
    #f'{APIBLB_FIELDS_TASK.taskid.name}=1&{APIBLB_FIELDS_TASK.robotips.name}={IP_MASTER}&current_station=-1'
    # URL 설정, station_id를 파라미터로 추가
    if fieldValue is None:
      url = f"https://{svrIP}:{port}/{baseDir}/{serviceName}"
    else:
      if baseDir == "":
        url = f"https://{svrIP}:{port}/{serviceName}?{fieldValue}"
      else:
        url = f"https://{svrIP}:{port}/{baseDir}/{serviceName}/?{fieldValue}"
    bReturn = False
    # if url.find('robot_alarm_status') > 1:
    #   log_all_frames(url)
    # if svrIP == GetMasterIP():
    #   log_all_frames(url)
    try:
        # GET 요청 전송
        response = requests.get(url, verify=False)  # verify=False는 SSL 인증을 무시하기 위한 설정 (자체 인증서일 때 사용)
        
        # 상태 코드가 200 (정상 응답)일 경우
        if response.status_code == 200:
            # 응답 내용 출력
            #print("Response:", )
            strResult = response.text
            bReturn = True
        else:
            strResult=(f"err url:{url}. code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        strResult=(f"An error occurred: {e}")
    #master_uri = GetUbutuParam(UbuntuEnv.ROS_MASTER_URI.name)
    #IP_MASTER = extract_hostname_from_uri(master_uri)
    sMsg = f'{url}->{bReturn},{strResult}'
    
    if svrIP == IP_MASTER:
        logger_local.info(sMsg)
        #logger_local.pretty(url=url,result=bReturn,response=strResult)
    elif port == HTTP_COMMON_PORT:
        #logger_android.info(sMsg)
        logger_android.pretty(url=url,result=bReturn,response=strResult)
    else:
        logger_svr.pretty(url=url,result=bReturn,response=strResult)
    
    # if svrIP != IP_MASTER and port != BLB_ANDROID_PORT_DEFAULT:
    #     log_all_frames(url)
    #     sMsg = f'{url}->{bReturn},{strResult}'
    #rospy.loginfo(sMsg)        
        #SendInfoHTTP(sMsg)
    return bReturn,strResult

def API_call_Android(BLB_ANDROID_IP,BLB_ANDROID_PORT,fieldValue=None):
    bReturn,strResult = API_call(BLB_ANDROID_IP,BLB_ANDROID_PORT,"",EndPoints.control.name,fieldValue)
    time.sleep(MODBUS_EXCEPTION_DELAY)
    SendInfoHTTP(strResult)
    return bReturn,strResult


def API_call_http(svrIP=BLB_SVR_IP_DEFAULT, port=9000,serviceName=APIBLB_METHODS_POST.robot_Current_node_info.name, fieldValue=None):
    strResult = ''
    #f'{APIBLB_FIELDS_TASK.taskid.name}=1&{APIBLB_FIELDS_TASK.robotips.name}={IP_MASTER}&current_station=-1'
    # URL 설정, station_id를 파라미터로 추가
    if fieldValue is None:
      url = f"http://{svrIP}:{port}/{serviceName}"
    else:
      url = f"http://{svrIP}:{port}/{serviceName}?{fieldValue}"
    bReturn = False
    # if url.find('robot_alarm_status') > 1:
    #   log_all_frames(url)
    # if svrIP == GetMasterIP():
    #   log_all_frames(url)
    try:
        # GET 요청 전송
        response = requests.get(url, verify=False)  # verify=False는 SSL 인증을 무시하기 위한 설정 (자체 인증서일 때 사용)
        
        # 상태 코드가 200 (정상 응답)일 경우
        if response.status_code == 200:
            # 응답 내용 출력
            #print("Response:", )
            strResult = response.text
            bReturn = True
        else:
            strResult=(f"err url:{url}. code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        strResult=(f"An error occurred: {e}")
    #master_uri = GetUbutuParam(UbuntuEnv.ROS_MASTER_URI.name)
    #IP_MASTER = extract_hostname_from_uri(master_uri)
    if svrIP != IP_MASTER:
        log_all_frames(url)
        print(strResult)        
    return bReturn,strResult


#분기기 제어
def API_CROSS_set(ipaddress, statusCross):
    fieldvalue ='CMD=WMOVE&MODE=1&POS=517560&SPD=500&ACC=3000&DECC=3000'    
    if isTrue(statusCross):
        fieldvalue = 'CMD=WMOVE&MODE=1&POS=0&SPD=500&ACC=3000&DECC=3000'        
    return API_call_http(BLB_CROSS_IP_DEFAULT, HTTP_COMMON_PORT, 'cross', fieldvalue)

def API_CROSS_stop():
    fieldvalue ='CMD=STOP&DECC=800'    
    return API_call_http(BLB_CROSS_IP_DEFAULT, HTTP_COMMON_PORT, 'cross', fieldvalue)

def API_CHARGER_Init():
    fieldvalue = f'ip={GetMasterIP()}'
    return API_call_http(BLB_CHARGE_IP, HTTP_COMMON_PORT, 'control', fieldvalue)

def SendAlarmHTTP(alarmMsg, tts=True, TTS_IP=BLB_ANDROID_IP_DEFAULT):
    rtMsg = log_all_frames(alarmMsg)
    rs = API_call_http(IP_MASTER,HTTP_COMMON_PORT,EndPoints.alarm.name, f'{getCurrentTime(spliter=sDivFieldColon, includeDate=True)}={rtMsg}')  
    if tts:
        API_call_Android(TTS_IP,HTTP_COMMON_PORT,"tts=10,10,알람이 발생하였습니다.")
    return rs

def logSSE_error(msg):
    logger_api.info(log_all_frames(msg))
    SendAlarmHTTP(msg,True,BLB_ANDROID_IP_DEFAULT)

infoMsg = ''
def SendInfoHTTP(strResult):
    global infoMsg
    infoMsg2=strResult.replace(sDivFieldColon,sDivSemiCol).replace('&','+')
    if infoMsg != infoMsg2:
        infoMsg = infoMsg2
        return API_call_http(IP_MASTER,HTTP_COMMON_PORT,EndPoints.info.name, f'{getCurrentTime(spliter=sDivFieldColon, includeDate=True)}={infoMsg2}')  

robot_navi_reply_timestamp = DATETIME_OLD
robot_navi_recv_lastValue = ""
robot_navi_lasttask = ""

def API_robot_navigation_info(df=pd.DataFrame(),STATUS_TASK=APIBLB_STATUS_TASK.Running):
  global robot_navi_reply_timestamp
  global robot_navi_recv_lastValue 
  global robot_navi_lasttask 

  if len(df) > 0:
    dictReplyTable = {field.name: "" for field in APIBLB_FIELDS_NAVI}
    df_started = df[df[APIBLB_FIELDS_TASK.workstatus.name] != APIBLB_STATUS_TASK.Ideal.value].tail(1)
    PrintDF(df_started)
    dicLast = df.iloc[-1]
    dicFirst = df.iloc[0]
    taskID = dicFirst[APIBLB_FIELDS_TASK.taskid.name]
    #robotip = dicFirst[APIBLB_FIELDS_TASK.robotips.name]
    current_taskmst = dicFirst[APIBLB_FIELDS_TASK.mastercode.name]
    initialstation = dicFirst[APIBLB_FIELDS_TASK.startnode.name]
    start_station=initialstation
    simulayoutcd = dicFirst[APIBLB_FIELDS_NAVI.simulayoutcd.name]    
    current_node = start_station
    current_node = dicFirst[APIBLB_FIELDS_TASK.startnode.name]
    current_task = dicFirst[APIBLB_FIELDS_TASK.detailcode.name]
    endnode = dicLast[APIBLB_FIELDS_TASK.endnode.name]
    if df_started is not None and len(df_started) > 0:
        #columns_to_keep = [field.name for field in APIBLB_FIELDS_TASK if field.name in df_started.columns]
        #print(df[columns_to_keep])
        current_node = df_started.iloc[0][APIBLB_FIELDS_TASK.startnode.name]
        start_station = current_node
        current_task = df_started.iloc[0][APIBLB_FIELDS_TASK.detailcode.name]
        endnode = df_started.iloc[0][APIBLB_FIELDS_TASK.endnode.name]
    else:
        rospy.loginfo('조건에 맞는 행이 없음.')
        PrintDF(df_started)
    
    mst_status = dicFirst[APIBLB_FIELDS_TASK.workstatus.name]
    if mst_status == APIBLB_STATUS_TASK.Completed.value:
        mst_status = STATUS_TASK.value
        
    #final_statoin = dicLast[APIBLB_FIELDS_TASK.endnode.name]
    final_station = dicLast[APIBLB_FIELDS_TASK.startnode.name]
    
    task_status = dicLast[APIBLB_FIELDS_TASK.workstatus.name]
    if task_status == APIBLB_STATUS_TASK.Completed.value:
        mst_status = APIBLB_STATUS_TASK.Completed.value
        current_node = dicLast[APIBLB_FIELDS_TASK.startnode.name]
        current_task = dicLast[APIBLB_FIELDS_TASK.detailcode.name]
        start_station = current_node
    elif task_status == APIBLB_STATUS_TASK.NONE.value:
        mst_status = APIBLB_STATUS_TASK.Ideal.value
        task_status=mst_status
        current_node = dicLast[APIBLB_FIELDS_TASK.startnode.name]
        current_task = dicLast[APIBLB_FIELDS_TASK.detailcode.name]
        start_station = current_node
        simulayoutcd = '08' #샤누이사 하드 코딩
    else:
        if df_started is not None and len(df_started) > 0:
            task_status = STATUS_TASK.value
        else:
            task_status = APIBLB_STATUS_TASK.Ideal.value
        #dictDF[current_taskmst] = df.to_json(orient='records')
    
    dictReplyTable[APIBLB_FIELDS_TASK.taskid.name] = taskID
    dictReplyTable[APIBLB_FIELDS_TASK.robotips.name] = IP_MASTER
    dictReplyTable[APIBLB_FIELDS_NAVI.current_taskmst.name] = current_taskmst
    dictReplyTable[APIBLB_FIELDS_NAVI.current_taskdtl.name] = current_task
    dictReplyTable[APIBLB_FIELDS_NAVI.start_station.name] = start_station
    dictReplyTable[APIBLB_FIELDS_NAVI.simulayoutcd.name] = simulayoutcd
    dictReplyTable[APIBLB_FIELDS_NAVI.endnode.name] = endnode
    dictReplyTable[APIBLB_FIELDS_NAVI.current_station.name] = current_node
    dictReplyTable[APIBLB_FIELDS_NAVI.orderstatus.name] = mst_status
    dictReplyTable[APIBLB_FIELDS_NAVI.workstatus.name] = task_status
    if STATUS_TASK==APIBLB_STATUS_TASK.Paused or  STATUS_TASK==APIBLB_STATUS_TASK.Canceled:
      dictReplyTable[APIBLB_FIELDS_NAVI.orderstatus.name] = task_status
      
    dictReplyTable[APIBLB_FIELDS_NAVI.initialstation.name] = initialstation
    dictReplyTable[APIBLB_FIELDS_NAVI.finalstation.name] = final_station
    # isKeepTimeOver = isTimeExceeded(robot_navi_reply_timestamp,2000)
    # if task_status == APIBLB_STATUS_TASK.Started.value:
    #     if robot_navi_lasttask  == current_task:
    #         if isKeepTimeOver:
    #             task_status = APIBLB_STATUS_TASK.Running.value
    #             dictReplyTable[APIBLB_FIELDS_NAVI.workstatus.name] = task_status
    #             if dictReplyTable[APIBLB_FIELDS_NAVI.orderstatus.name] == APIBLB_STATUS_TASK.Started.value:
    #                 dictReplyTable[APIBLB_FIELDS_NAVI.orderstatus.name] = task_status
    #     else:
    #         robot_navi_reply_timestamp = getDateTime()
    robot_navi_lasttask = current_task
    robot_navi_recv_lastValue = task_status
    dictReplyTable2 = {APIBLB_FIELDS_TASK.taskid.name:taskID,
                    APIBLB_FIELDS_TASK.robotips.name:IP_MASTER,
                    APIBLB_FIELDS_NAVI.current_station.name:current_node
                  }
    API_call(serviceName=APIBLB_METHODS_POST.robot_Current_node_info.name,fieldValue=urllib.parse.urlencode(dictReplyTable2))
    return API_call(svrIP=BLB_SVR_IP_DEFAULT,serviceName=APIBLB_METHODS_GET.robot_navigation_info.name, fieldValue=urllib.parse.urlencode(dictReplyTable)), current_node

def API_public_strtopic(topic, dictParam):
    dictParam['topic'] = topic
    json_str = json.dumps(dictParam)
    encoded = urllib.parse.quote(json_str)
    #dictData = {'data': encoded}
    strParam = f'data={encoded}'
    return API_callBLB(svName = APIBLB_METHODS_POST.robot_topic_publish.name,field_Value =strParam)

def API_GetTaskList():
  dictReplyTable = {
    APIBLB_FIELDS_TASK.robotips.name : IP_MASTER
  }
  return API_call(svrIP=BLB_SVR_IP_DEFAULT,serviceName=APIBLB_METHODS_GET.robot_TaskChain_ListRequest.name, fieldValue=urllib.parse.urlencode(dictReplyTable))

def API_SetRackID(taskid=0,tasktype=3,rackID='R1'):
    dictParam = {APIBLB_FIELDS_TASK.taskids.name:taskid,
                 APIBLB_FIELDS_TASK.robotips.name:IP_MASTER,
                 APIBLB_FIELDS_TASK.tasktype.name:tasktype,
                 APIBLB_FIELDS_TASK.rackcode.name:rackID,
    }
    paramStr=urllib.parse.urlencode(dictParam)
    log_all_frames(paramStr)
    return API_call(svrIP=BLB_SVR_IP_DEFAULT,serviceName=APIBLB_METHODS_GET.robot_TrackRackUpdate_action.name, fieldValue=paramStr)

def API_SetOrderTable(tableID=HOME_TABLE,tasktype=APIBLB_TASKTYPE.ServingTask.value, taskid=0):
    tableIDStr = tableID
    tableIDNum = try_parse_int(tableID, MIN_INT)
    if tableIDNum != MIN_INT:
      tableIDStr = f'T{tableID}'
    dictParam = {APIBLB_FIELDS_TASK.taskid.name:taskid,
                 APIBLB_FIELDS_TASK.robotips.name:IP_MASTER,
                 APIBLB_FIELDS_TASK.tasktype.name:tasktype,
                 APIBLB_FIELDS_ACTION.target.name:tableIDStr,
                 APIBLB_FIELDS_TASK.updateduser.name:11
    }
    paramStr=urllib.parse.urlencode(dictParam)
    #paramStr = f'{APIBLB_FIELDS_TASK.taskid.name}={tasktype}&{APIBLB_FIELDS_TASK.robotips.name}={IP_MASTER}&target={tableIDStr}&tasktype={tasktype}&updateduser=11'
    #log_all_frames(paramStr)
    return API_call(svrIP=BLB_SVR_IP_DEFAULT,serviceName=APIBLB_METHODS_POST.robot_task_action.name, fieldValue=paramStr)

def API_AppendOrderTable(tableID,rackID,tasktype):
    tableIDStr = tableID
    tableIDNum = try_parse_int(tableID, MIN_INT)
    if tableIDNum != MIN_INT:
      tableIDStr = f'T{tableID}'
    rackIDNum = try_parse_int(rackID, MIN_INT)
    if rackIDNum != MIN_INT:
      rackID = f'R{rackID}'
    dictParam = {APIBLB_FIELDS_TASK.taskids.name:0,
                 APIBLB_FIELDS_TASK.worknames.name:tableIDStr,
                 APIBLB_FIELDS_TASK.trayracks.name:rackID,
                 APIBLB_FIELDS_TASK.tasktypes.name:tasktype,
                 APIBLB_FIELDS_TASK.prioritys.name:1,
                 APIBLB_FIELDS_TASK.emergencys.name:1,
                 APIBLB_FIELDS_TASK.sortorders.name:1,
                 APIBLB_FIELDS_TASK.commentss.name: "''",
                 APIBLB_FIELDS_TASK.UpdatedBys.name:11,
                 APIBLB_FIELDS_TASK.SPstatus.name:3,
    }
    paramStr=urllib.parse.urlencode(dictParam)
    #paramStr = f'{APIBLB_FIELDS_TASK.taskid.name}={tasktype}&{APIBLB_FIELDS_TASK.robotips.name}={IP_MASTER}&target={tableIDStr}&tasktype={tasktype}&updateduser=11'
    log_all_frames(paramStr)
    return API_call(svrIP=BLB_SVR_IP_DEFAULT,serviceName=APIBLB_METHODS_POST.robot_TaskChain_action.name, fieldValue=paramStr)

def API_SetOrderHome():
    return API_SetOrderTable(tableID=HOME_TABLE,tasktype=APIBLB_TASKTYPE.ReturnHomeAfteReachTask.value,taskid = 0)

def API_SetCurrentNode(nodeID=-1 ,taskid = 0):
  dictReplyTable = {APIBLB_FIELDS_TASK.taskid.name:taskid,
                    APIBLB_FIELDS_TASK.robotips.name:IP_MASTER,
                    APIBLB_FIELDS_NAVI.current_station.name:nodeID
                  }
  fieldValue=urllib.parse.urlencode(dictReplyTable)
  #fieldStr = f'taskid=1&current_station={nodeID}'
  return API_call(fieldValue=fieldValue)

def API_SendAlarm(AlarmInstance):
  if type(AlarmInstance) == ALM_User:
    alarm_class = 'Info'
  elif type(AlarmInstance) == ALM_Fatal:
    alarm_class = 'Error'
  else:
    alarm_class = 'Warning'
  alarm_msg_split = AlarmInstance.value.split(sDivFieldColon)
  alarm_code = alarm_msg_split[0]
  alarm_msg = alarm_msg_split[1].strip()
  dictReplyTable = {APIBLB_FIELDS_ALARM.ipaddresss.name:IP_MASTER,
                    APIBLB_FIELDS_ALARM.alarmcodes.name:f'{alarm_class[0]}{alarm_code[-4:]}',
                    APIBLB_FIELDS_ALARM.alarmmsgs.name:alarm_msg,
                    APIBLB_FIELDS_ALARM.commentss.name:alarm_class,                    
                  }
  fieldValue=urllib.parse.urlencode(dictReplyTable)
  return API_call(svrIP=BLB_SVR_IP_DEFAULT,serviceName=APIBLB_METHODS_GET.robot_alarm_status.name, fieldValue=fieldValue)

def calculate_distance(graph: Dict[int, List[Tuple[int, int]]], start: int, end: int) -> int:
    """
    두 노드 간 최단 거리를 계산하는 함수 (다익스트라 알고리즘 사용)
    :param graph: 그래프 데이터 (예: {1: [(2, 2500), (3, 5000)], ...})
    :param start: 출발 노드 번호
    :param end: 도착 노드 번호
    :return: 최단 거리 값 또는 None (경로가 없는 경우)
    """
    # 우선순위 큐와 거리 초기화
    queue = [(0, start)]  # (누적 거리, 현재 노드)
    distances = {node: float('inf') for node in graph}
    distances[start] = 0

    # 다익스트라 알고리즘
    while queue:
        current_distance, current_node = heapq.heappop(queue)

        # 이미 처리된 노드 무시
        if current_distance > distances[current_node]:
            continue

        # 인접 노드 확인
        for neighbor, weight in graph.get(current_node, []):
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(queue, (distance, neighbor))

    # 결과 반환
    return distances[end] if distances[end] != float('inf') else None
  
# graphTmp를 DataFrame으로 변환하는 함수
def graph_to_dataframe(graph):
    edges = set()  # 중복된 간선 제거를 위한 집합
    for node, neighbors in graph.items():
        for neighbor, cost in neighbors:
            # 중복 간선 방지: 작은 노드 번호가 먼저 오도록 튜플 생성
            edge = tuple(sorted((node, neighbor)))
            edges.add((edge[0], edge[1], cost))
    
    # DataFrame 생성
    df = pd.DataFrame(edges, columns=["node1", "node2", "distance"])
    return df
  
def ConvertInt64toInt_DF(df):
  return df.astype({col: 'int' for col in df.select_dtypes(include=['int64']).columns})

def ConvertInt64toInt_Dict(data:dict):
  return {key: (int(value) if isinstance(value, np.int64) else value) for key, value in data.items()}

def get_adjacency_changes(oldGraph, newGraph):
    """
    두 그래프의 인접 노드 변경사항을 비교합니다.
    
    Args:
        oldGraph (dict): 변경 전의 그래프
        newGraph (dict): 변경 후의 그래프

    Returns:
        dict: 노드별로 변경된 인접 노드 정보
              {
                  노드: {
                      "added": [추가된 인접 노드],
                      "removed": [제거된 인접 노드]
                  }
              }
    """
    changes = {}

    # 모든 노드에 대해 비교
    all_nodes = set(oldGraph.keys()).union(newGraph.keys())
    for node in all_nodes:
        # 이전 그래프의 인접 노드
        old_neighbors = set(neighbor for neighbor, _ in oldGraph.get(node, []))
        # 새로운 그래프의 인접 노드
        new_neighbors = set(neighbor for neighbor, _ in newGraph.get(node, []))

        # 추가된 인접 노드와 제거된 인접 노드 확인
        added = new_neighbors - old_neighbors
        removed = old_neighbors - new_neighbors

        # 변경사항이 있을 경우 저장
        if added or removed:
            changes[node] = [list(added),list(removed)]
    return changes

# CRC 계산 함수 (Modbus RTU에서 사용)
def calc_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 0x0001) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

# Modbus 요청 패킷 생성 (슬레이브 주소, 기능 코드, 시작 주소, 레지스터 수)
def create_modbus_request(slave_addr, function_code, start_addr, register_count):
    request = struct.pack('>B', slave_addr) + struct.pack('>B', function_code) + struct.pack('>H', start_addr) + struct.pack('>H', register_count)
    crc = calc_crc(request)
    crc_bytes = struct.pack('<H', crc)  # CRC는 리틀 엔디안으로 패킹
    return request + crc_bytes

def parse_modbus_response(response):
    if len(response) < 5:
        raise ValueError("응답 길이가 너무 짧습니다.")
    
    slave_addr, function_code, byte_count = struct.unpack('>BBB', response[:3])
    
    # 레지스터 값은 2바이트씩 나누어져 있음
    register_values = []
    for i in range(byte_count // 2):
        register_value = struct.unpack('>H', response[3 + i*2 : 5 + i*2])[0]
        register_values.append(register_value)
    
    # CRC는 마지막 2바이트
    crc_received = struct.unpack('<H', response[-2:])[0]
    
    # CRC 계산해서 비교
    if calc_crc(response[:-2]) != crc_received:
        raise ValueError("CRC 불일치")
    
    return slave_addr, function_code, register_values

def distance_to_pulseH(distance_mm):
    """
    Converts the given distance in millimeters to the corresponding motor pulse value.

    Parameters:
    distance_mm (int): The distance in millimeters.

    Returns:
    int: The calculated motor pulse value.
    """

    # 비례식을 이용하여 펄스를 계산
    pulse_value = (distance_mm / MOVE_H_SAMPLE_MM) * MOVE_H_SAMPLE_PULSE
    return round(pulse_value)

def pulseH_to_distance(current_pulseStr):
    """
    Converts the given motor pulse value to the corresponding distance in millimeters.

    Parameters:
    current_pulse (int): The current motor pulse value.

    Returns:
    int: The calculated distance in millimeters.
    """
    
    current_pulse = try_parse_int(current_pulseStr)
    # 비례식을 이용하여 거리를 계산
    distance = (current_pulse / MOVE_H_SAMPLE_PULSE) * MOVE_H_SAMPLE_MM
    return round(distance)

# print(pulseH_to_distance(1310000))
# print(pulseH_to_distance(3060000))
# print(pulseH_to_distance(3060000))

def calculate_speed_fromRPM(rpmStr):
    rpm = try_parse_float(rpmStr)
    # 1분에 몇 바퀴 도는지 주어진 rpm에서 초당 바퀴수로 변환
    revolutions_per_second = rpm / 60.0
    
    # 120 바퀴당 2820mm 전진한다는 조건을 활용
    distance_per_revolution = roundPulse* MOVE_H_SAMPLE_MM / MOVE_H_SAMPLE_PULSE  # 한 바퀴당 전진 거리(mm)
    
    # 초당 이동 거리 계산 (mm/s)
    speed_mm_per_second = revolutions_per_second * distance_per_revolution
    
    return round(speed_mm_per_second)

def GetAngleMargin(x1,y1,x2,y2):
    # 벡터 정의
    v1 = np.array([x1, y1])
    v2 = np.array([x2, y2])

    # 벡터의 크기 계산
    v1_magnitude = np.linalg.norm(v1)
    v2_magnitude = np.linalg.norm(v2)

    # 벡터의 내적 계산
    dot_product = np.dot(v1, v2)

    # 각도 계산 (라디안 단위)
    angle_radians = np.arccos(dot_product / (v1_magnitude * v2_magnitude))

    # 각도를 도 단위로 변환
    angle_degrees = np.degrees(angle_radians)

    return round(angle_degrees)


def limit_RPM_value(value):
    if abs(value) > DEFAULT_SPD_LIMIT:
        value = DEFAULT_SPD_LIMIT if value > 0 else -DEFAULT_SPD_LIMIT
    return value  

def is_name_in_enum(name, enumClass):
    return any(name == item.name for item in enumClass)


def is_value_in_enum(value, enumClass):
    return any(value == item.value for item in enumClass)


# def getRunningMotors(dic_485poll):
#     returnData = []
#     modbusIDs = list(dic_485poll.keys())  # Get a list of the dictionary keys
#     for modbusID in modbusIDs:
#         dicModbus = dic_485poll[modbusID]
#         #print(f'Checking Modbus ID{modbusID}')
#         #modbusIDint = int(modbusID)
#         # if ModbusID.BAL_ARM1.value == modbusIDint:
#         #     print(dicModbus)
#         st_CMD_FINISH = isTrue(dicModbus.get(MonitoringField.ST_CMD_FINISH.name, None))
#         st_RUNNING = isTrue(dicModbus.get(MonitoringField.ST_RUNNING.name, None))
#         isRunning = st_RUNNING
#         if not isRunning:
#             continue
#         if not st_CMD_FINISH:
#             returnData.append(int(modbusID))
    
#     return returnData


# def getRunningMotors(dic_485poll):
#     returnData = []
#     for modbusID, dicModbus in dic_485poll.items():
#         #print(f'Checking Modbus ID{modbusID}')
#         #modbusIDint = int(modbusID)
#         # if ModbusID.BAL_ARM1.value == modbusIDint:
#         #     print(dicModbus)
#         st_CMD_FINISH = isTrue(dicModbus.get(MonitoringField.ST_CMD_FINISH.name, None))
#         st_RUNNING = isTrue(dicModbus.get(MonitoringField.ST_RUNNING.name, None))
#         isRunning = st_RUNNING
#         if not isRunning:
#             continue
#         if not st_CMD_FINISH:
#             returnData.append(int(modbusID))
    
#     return returnData

class APIBLB_ACTION_REPLY(Enum):
    code = auto()
    message = auto()
    R101 = '완료되었습니다.'
    R102 = '이미 완료된 업무입니다.'    #완료된 작업을 Pause 하려했을때
    R103 = '일시중지 된 작업이 없습니다.'   #완료된 작업을 Resume 하려 했을때
    #R101 = 'Action Done Successfully'
    E500 = 'Internal Server error.'
    E101 = 'Not enough space to open tray door'
    E102 = 'Not enough space to rotate tray'
    E103 = 'BLB Stopped cause some motor alarms detected'   #알람이 감지된 상태에서 모터 제어 명령어를 보내면 응답
    E104 = 'Checking parameter values consistency were invalid.'   #알람이 감지된 상태에서 모터 제어 명령어를 보내면 응답
    E105 = 'BLB is already on the operation. try again later.'   #알람이 감지된 상태에서 모터 제어 명령어를 보내면 응답
    E106 = 'BLB is not able to move carrier or rotate before all arms folded.'   #알람이 감지된 상태에서 모터 제어 명령어를 보내면 응답
    E107 = 'BLB is not able to move carrier before set rail road switch.'   #알람이 감지된 상태에서 모터 제어 명령어를 보내면 응답
    E108 = 'BLB heading side is invalid to move carrier.'   #알람이 감지된 상태에서 모터 제어 명령어를 보내면 응답
    E109 = 'Overload error,too much foods in the tray.'   #알람이 감지된 상태에서 모터 제어 명령어를 보내면 응답
    E110 = 'MotorH is paused.'   #주행중지 모드 (관절 및 리프트만 동작)

class MonitoringField_EX(Enum):
    def __init__(
        self,
        value,
        description_value_Negative,
        description_value_zero,
        description_value_Positive,
    ):
        self._value_ = value
        self.description_value_Negative = description_value_Negative
        self.description_value_zero = description_value_zero
        self.description_value_Positive = description_value_Positive

    ALM_CD = ("알람명", "통신불가", "정상", "알람발생")
    ST_CMD_FINISH = ("상태", "알수없음", "동작중", "정지")
    ST_ENABLE = ("모터초기화", "알수없음", "이상", "완료")

    STATE = ("상태", "동작중", "오픈", "클로즈")

    def GetEnumValue(strCurrent):
        for item in MonitoringField_EX:
            if item.name == strCurrent:
                return (
                    item.value,
                    item.description_value_Negative,
                    item.description_value_zero,
                    item.description_value_Positive,
                )
        return None, None, None, None  # 해당 value를 가진 멤버가 없을 경우 None 반환


class ModbusID_EX(Enum):
    def __init__(self, value, caption, description):
        self._value_ = value
        self.caption = caption
        self.description = description

    MOTOR_H = (14, "주행모터", "주행용 - 400W -> 6020 to ISV-6040")
    ROTATE_540 = (26, "캐리어회전", "돌림회전 뻗을방향 -> 2323 to ISV2-6020")
    BAL_ARM1_1 = (12, "밸런스관절1-1", "밸런스 1관절 첫번째 모터 (6040)")
    BAL_ARM1_2 = (11, "밸런스관절1-2", "밸런스 1관절 두번째 모터 (6040)")
    BAL_ARM2 = (
        15,
        "밸런스 2관절",
        "밸런스 2관절 (2313, 1706/6020 모터는 60분내 교체 가능)",
    )
    TELE_STRAIGHT_1 = (15, "서빙텔레스코픽1", "서빙텔레스코픽(뻗는부분 1차)")
    TELE_STRAIGHT_2 = (15, "서빙텔레스코픽2", "서빙텔레스코픽(뻗는부분 2차)")
    ROTATE_360 = (31, "트레이회전", "1706")
    MOTOR_V = (6, "견인모터", "견인모터 (400W)")
    NTC = (2, "NTC", "온도계, 9600")
    BMS = (1, "BMS", "QUCC 9600")

    def GetEnumValue(strCurrent):
        for item in ModbusID_EX:
            if item.value == strCurrent:
                return {
                    "value": item.value,
                    "caption": item.caption,
                    "description": item.description,
                }
        return None  # 해당 value를 가진 멤버가 없을 경우 None 반환


class MonitoringField(Enum):
    BUS_VOLTAGE = auto()
    CMD_SPD = auto()
    ALM_CD = auto()
    ALM_NM = auto()
    CMD_POS = auto()
    CUR_POS = auto()
    CUR_SPD = auto()
    LASTSTART = auto()
    LAST_STARTED_POS = auto()
    LAST_TARGET_POS = auto()
    CUR_CURRENT = auto()
    CUR_TORQUE = auto()
    ST_CMD_FINISH = auto()  # 정상적으로 종료하면 이 부분이 True
    ST_RUNNING = auto()  # 모터가 돌고 있지 않으면 이 부분이 True
    LASTSEEN = auto()
    ST_ENABLE = auto()
    DI_NOT = auto()  # 모터가 CCW 로 회전할때 여기에 걸리면 무조건 멈춘다.
    DI_HOME = auto()
    DI_ESTOP = auto()
    DI_POT = auto()  # 모터가 CW 로 회전할때 여기에 걸리면 무조건 멈춘다.
    POT_POS = auto()  # CALI 로 결정된 POT와 NOT
    NOT_POS = auto()
    IS_CCW = auto()
    POT_ALLOCATION = auto()
    OVER_LOAD = auto()
    TOQ_LIMIT = auto()
    INERTIA_RATIO = auto()
    AUTO_GAIN = auto()
    SI_POT = auto()


class MonitoringField_BMS(Enum):
    Charger = auto()  # 전체전압
    Voltage = auto()  # 전체전압
    CurCadc = auto()  # 현재사용전류 (단위 나중에 기입하자)
    Tmax = auto()  # 제일 높은 셀 온도
    Tmin = auto()  # 제일 낮은 셀 온도
    Vmax = auto()  # 제일 높은 셀 전압
    Vmin = auto()  # 제일 낮은 셀 전압
    RSOC = auto()  # 배터리 잔량 퍼센티지
    WATT = auto() #소모전류와트

class BLD_PROFILE_CMD(Enum):
    balDataRecord = -2  # 범블비밸런스데이터레코드 추가
    balDataSave = -3  # 범블비밸런스데이터저장
    balDataClear = -1  # 범블비밸런스데이터클리어
    bal540_45CW = 2  # 메인회전45도돌리기
    bal540_ZERO = 3  # 메인회전원점복귀
    balTray_45CW = 4  # Tray45도돌리기
    balTray_ZERO = 5  # Tray원점복귀
    balLiftUp = 6  # 리프트원점(상승)
    balLiftDown = 7  # 리프트하강
    balSrvArm100mm = 8  # 서빙암100mm전개
    balSrvArmZERO = 9  # 서빙암제로
    TiltDown = 10
    TiltFace = 11
    TiltMaxUp = 12
    TiltDiagonal = 13
    DoorUp = 14
    DoorStop = 15
    DoorDown = 16
    SCANTABLE = 17
    MOVE_MOTOR_H = 18
    SAVE_POS = 19
    CALI_TRAY = 20
    CALI_MAINROTATE = 21
    SRV_EXPAND = 22
    SRV_FOLD = 23
    BACK_HOME = 24
    ESTOP = 25
    FOLD_ALL = 26
    EXPAND_ALL = 27
    FOLD_ARM = 28
    EXPAND_ARM = 29
    ALM_C = 30
    WLOC_NOT = 31
    TrayHome = 32
    MOTORSTOP = 33
    
class ModbusID(Enum):
    MOTOR_H = 15  # 주행 (6040)
    MOTOR_V = 6  # 견인 (6040)
    BAL_ARM1 = 13  # 밸런스 1관절 (2323)
    ROTATE_MAIN_540 = 27  # 메인회전540 (2323)
    BAL_ARM2 = 10  # 밸런스 2관절(2축) (6020)
    TELE_BALANCE = 9  # 밸런싱 텔레스코픽(2313)
    TELE_SERV_MAIN = 11  # 서빙텔레스코픽(뻗는부분 메인) (6020)
    #TELE_SERV_INNER = 28  # 서빙텔레스코픽(뻗는부분 200mm 까지만) (1708)
    ROTATE_SERVE_360 = 31  # 트레이 자체미세회전360 (1706)
    # ROTATE_TESTER = 3
    # NTC = 2
    # BMS = 1
    @classmethod
    def from_value(cls, value):
        # 만약 value가 문자열이라면 정수로 변환 시도
        if isinstance(value, str):
            try:
                value = int(value)
            except ValueError:
                raise ValueError(f"Invalid literal for int() with base 10: '{value}'")
                
        for member in cls:
            if member.value == value:
                return member
        raise ValueError(f"No member found for value: {value}")

class RackID(Enum):
    R1 = 0
    R2 = 1
    @classmethod
    def from_name_or_value(cls, nm, isName):
        for member in cls:
            if isName:
              if (str)(member.name) == str(nm):
                  return member
            else:
              if (str)(member.value) == str(nm):
                  return member              
        raise ValueError(f"No member found for name: {nm}")

#중량을 key, arm1 모터와 밸런스텔레스코픽의 합계의 Pulse 를 value 로 갖는 
#dict 에서 무게에 대한 밸런스부 pulse 값을 추정한다
def get_balanceArmPulse(weight, dicWeightBalData):
    keys = sorted(dicWeightBalData.keys(), reverse=True)
    
    # 입력 값이 데이터의 최소값보다 작을 경우
    if weight <= keys[-1]:
        return dicWeightBalData[keys[-1]]
    
    # 입력 값이 데이터의 최대값보다 클 경우
    if weight >= keys[0]:
        return dicWeightBalData[keys[0]]

    # 선형 보간법을 통한 토크 계산
    for i in range(len(keys) - 1):
        if keys[i] >= weight >= keys[i + 1]:
            x1, y1 = keys[i], dicWeightBalData[keys[i]]
            x2, y2 = keys[i + 1], dicWeightBalData[keys[i + 1]]
            return round(y1 + (weight - x1) * (y2 - y1) / (x2 - x1))

#임시코드 나중에 31번 지우자
list_ArmControlMotors = [str(ModbusID.BAL_ARM1.value),str(ModbusID.BAL_ARM2.value),str(ModbusID.TELE_BALANCE.value),str(ModbusID.TELE_SERV_MAIN.value),str(ModbusID.MOTOR_V.value),str(ModbusID.MOTOR_H.value),str(ModbusID.ROTATE_SERVE_360.value),str(ModbusID.ROTATE_MAIN_540.value)]

def get_modbus_objects_from_topics(topics):
    modbus_objects = []
    for topic in topics:
        if topic.startswith('/MB_'):
            try:
                value = int(topic.split('_')[1])
                modbus_objects.append(ModbusID(value))
            except (ValueError, KeyError):
                # Handle the case where the value is not an integer or not a valid ModbusID
                continue
    return modbus_objects

class StateBranchValue(Enum):
    ERROR = -1
    MOVING = -2
    CLOSE = 1
    OPEN = 0

    def GetEnumValue(strCurrent):
        for item in StateBranchValue:
            if item.value == strCurrent:
                return item.name
        return None  # 해당 value를 가진 멤버가 없을 경우 None 반환


class StateBranch:
    STATUS = {
        200: StateBranchValue.CLOSE,
        100: StateBranchValue.OPEN,
    }


class MQTT_FIELD(Enum):
    # 노드에 전송할 스트링
    PAYLOAD = auto()
    TOPIC = auto()  # MQTT토픽주소
    DIVSTR_ITEM = auto()  # 아이템별 구분자. 보통 콤마
    DIVSTR_FIELD = auto()  # KEY-VALUE 구분자. 보통 콜론:


class MQTT_TOPIC_VALUE(Enum):
    BLB_ALARM = "BLB/alarm/"
    BLB_CALL = "BLB/CALL"
    IMU = "BLB/IMU"  # NODEMCU 에 자이로를 장착하여 이 MQTT 노드에 보내면 ROS1 IMU 포맷으로 변환하여 퍼블리시 한다
    TTS = "/home/member"  # HomeAssistant TTS 요청 메세지를 보내는 MQTT 노드
    MSG_CROSS_REQUEST = "BLB/CROSS_CMD/set"


class NODE_SPECIAL_VALUE(Enum):
    CHARGING_STATION = 2  # 충전소 노드 : 99
    KITCHEN_STATION = 1  # 부엌 노드 (음식제공) : 98
    NOT_TABLE = 0  # 테이블이 아닌곳은 0

    def GetEnumValue(strCurrent):
        sCurrent = str(strCurrent)
        for s in NODE_SPECIAL_VALUE:
            sCurrentTmp = str(s)
            if sCurrent.endswith(sCurrentTmp):
                return s.value
        return -1

def filter_recent_data(tsKey, data_list, time_threshold=1.0):
    current_time = time.time()
    filtered_data = [data for data in data_list if current_time - data[tsKey] <= time_threshold]
    sorted_data = sorted(filtered_data, key=lambda x: x[tsKey], reverse=True)
    return sorted_data

def filter_and_sort_by_lastseen(data: Dict[int, List[Dict[str, Any]]], time_threshold: timedelta) -> List[Dict[str, Any]]:
    current_time = getDateTime()
    filtered_data = [
        item
        for marker_list in data.values()
        for item in marker_list
        if current_time - datetime.fromtimestamp(item[ARUCO_RESULT_FIELD.LASTSEEN.name]) <= time_threshold
    ]
    return sorted(filtered_data, key=lambda x: x[ARUCO_RESULT_FIELD.LASTSEEN.name], reverse=True)
def rotate_pixel_coordinates(x, y, angle_degrees):
    # 각도를 라디안으로 변환
    angle_radians = math.radians(angle_degrees)
    
    # 회전 변환 행렬 적용
    x_corrected = x * math.cos(angle_radians) + y * math.sin(angle_radians)
    y_corrected = -x * math.sin(angle_radians) + y * math.cos(angle_radians)
    
    # 정수로 변환 (필요에 따라 정수 좌표로 변환)
    x_corrected = int(round(x_corrected))
    y_corrected = int(round(y_corrected))
    
    return x_corrected, y_corrected

# # 예시 사용
# x, y = 597, -195
# angle = 90  # 예: 시계방향으로 90도 회전
# x_corrected, y_corrected = rotate_pixel_coordinates(x, y, angle)
# print(f"{angle}도 회전 후 정위치 좌표: ({x_corrected}, {y_corrected})")


def CalculateDistanceAngleFromArucoTop(dicAruco):
    DIFF_X = float(dicAruco[ARUCO_RESULT_FIELD.DIFF_X.name]) * 100
    DIFF_Y = float(dicAruco[ARUCO_RESULT_FIELD.DIFF_Y.name]) * 100
    DIFF_Y_NEW = DIFF_Y + 903
    CAM_ID = dicAruco[ARUCO_RESULT_FIELD.CAM_ID.name]   
    distanceDiscard, angle_real = calculate_distance_and_angle(DIFF_X, DIFF_Y_NEW)    
    offSetX = 70 if CAM_ID == 0 else -70
    X = dicAruco[ARUCO_RESULT_FIELD.X.name] + offSetX
    Y = dicAruco[ARUCO_RESULT_FIELD.Y.name]
    distanceReal, angle_Discard = calculate_distance_and_angle(X, Y)    
    return round(distanceReal*1.754385964912281),(angle_real+90)%360
    

def CalculatePosXYFromAruco(dicAruco,angle_degrees = 90,offSetX=70,offSetY=130):    
    X = dicAruco[ARUCO_RESULT_FIELD.X.name]
    Y = dicAruco[ARUCO_RESULT_FIELD.Y.name]
    # CAM_ID = dicAruco[ARUCO_RESULT_FIELD.CAM_ID.name]
    # offSetX = 70 if CAM_ID == 0 else -70
    # offSetY = 130  #카메라에서부터 메인회전 중심까지의 거리

    # ratioX = 140.0/118.0
    # ratioY = 600.0/660.0
    # distX = X * ratioX
    # distY = Y * ratioY
    # print(distX,distY)
    # posX = distX + offSetX
    # posY = distY + offSetY
    posX = X + offSetX
    posY = Y + offSetY
    x_corrected, y_corrected=rotate_pixel_coordinates(posX,posY,angle_degrees)
    # posX = round(distX + offSetX)
    # posY = round(distY + offSetY)
    log_all_frames(x_corrected,y_corrected)
    return x_corrected,y_corrected
    #return posX,posY
        
class POLL_COMMON(Enum):  # 캐리어 상태 메세지
    START = auto()
    RATE = auto()
    LEN = auto()


class MOTOR_CALI_STATUS(Enum):  # 캘리브레이션 상태
    X_CANCELLED = -3
    Z_NOT_COMPLETED = -1
    A_REQUESTED = 0
    B_JOGCW_REQUESTED = 1
    C_JOGCCW_REQUESTED = 2
    D_CALI_COMPLETED = 3


class MotorCmdField(Enum):
    WSETUP2 = auto()  # 모니터링 데이터 매핑셋업 임시
    WSETUP = auto()  # 모니터링 데이터 매핑셋업
    WINIT = auto()  # 모터 초기화 명령
    WINITREVERSE = auto()  # 모터 초기화 명령+POT/NOT가 뒤바뀌는 경우
    WLIMIT = auto()  # 모터 SW POT 및 NOT 지정
    WLIMIT_OFF = auto()  # SW POT NOT 해제
    WOFF = auto()  # 모터 브레이크 해제 (DISABLE 모드)
    WMOVE = auto()  # 모터 회전 명령
    WCALI = auto()  # 모터 캘리브레이션 명령
    WSTOP = auto()  # 모터 중지
    WALM_C = auto()  # 모터 알람 클리어
    WZERO = auto()  # 현재 위치를 엔코더값 0 으로 초기화
    WLOC = auto()  # 현재 위치값을 지정한 파라미터로 임의설정
    WSPD = auto()  # 모터 속도 명령
    WTORQUE = auto()  # 토크 제한 설정
    WGAIN = auto()  # 모터 게인
    WRATIO = auto()  # 모터 관성비
    WPN_OFF = auto()  # POT NOT 활성화 설정
    WP_ON = auto()  #`P 정방향에 반응
    WN_ON = auto() #`0x409:0x26,0x40B:0x0`N 역방향에 반응
    


class MotorWMOVEParams(Enum):
    MBID = auto()
    CMD = auto()
    MODE = auto()
    POS = auto()
    SPD = auto()
    ACC = auto()
    DECC = auto()
    TIME = auto()
    TQL = auto()  #토크리밋
    POT = auto()  #SW POT
    NOT = auto()  #SW NOT

class MotorCommandManager:
    def __init__(self, data):
        self.data = {item[MotorWMOVEParams.MBID.name]: item for item in data}
    
    # def get(self, mbid, key):
    #     """MBID와 키를 사용하여 값을 조회합니다."""
    #     if mbid in self.data and key in self.data[mbid]:
    #         return self.data[mbid][key]
    #     return None
    def get(self, mbid, key):
        """MBID 또는 MBID 리스트와 키를 사용하여 값을 조회합니다."""
        if isinstance(mbid, list):  # MBID가 리스트일 경우
            return [self.data[mb][key] if mb in self.data and key in self.data[mb] else None for mb in mbid]
        else:  # MBID가 단일 값일 경우
            if mbid in self.data and key in self.data[mbid]:
                return self.data[mbid][key]
            return None

    def update(self, mbid, key, value):
        """MBID, 키, 값을 사용하여 데이터를 업데이트합니다."""
        if mbid in self.data:
            self.data[mbid][key] = value
            return True
        return False
    
    def add_command(self, command):
        """새로운 명령을 추가합니다."""
        if 'MBID' in command:
            self.data[command[MotorWMOVEParams.MBID.name]] = command
            return True
        return False
      
    def remove_cmd(self, key, value):
        """특정 키에 해당하는 값이 일치하는 명령을 제거합니다."""
        to_remove = [mbid for mbid, cmd in self.data.items() if key in cmd and cmd[key] == value]
        for mbid in to_remove:
            del self.data[mbid]
        return to_remove
          
    def remove_command(self, mbid):
        """MBID 또는 MBID 리스트를 사용하여 명령을 제거합니다."""
        if isinstance(mbid, list):  # MBID가 리스트일 경우
            removed = []
            for mb in mbid:
                if mb in self.data:
                    del self.data[mb]
                    removed.append(mb)
            return removed
        else:  # MBID가 단일 값일 경우
            if mbid in self.data:
                del self.data[mbid]
                return [mbid]
        return []
    
    def get_all_commands(self):
        """모든 명령을 리스트 형태로 반환합니다."""
        return list(self.data.values())
    
    def get_command_count(self):
        """총 명령 수를 반환합니다."""
        return len(self.data)
         
class MonitorPC(Enum):
    DISK_ID = auto()
    PARTITION_TYPE = auto()
    DISK_TOTAL_SIZE = auto()
    DISK_USED_SIZE = auto()
    DISK_USAGE_RATE = auto()
    CPU_USAGE = auto()
    CPU_TEMP = auto()
    MEMORY_TOTAL_SIZE = auto()
    MEMORY_USED_SIZE = auto()
    MEMORY_USED_PERCENT = auto()
    RX = auto()
    TX = auto()
    NUM_OF_DISPLAYS = auto()

class MonitorROS(Enum):
    master_uri = auto()
    slave_uri = auto()
    master_hostname = auto()
    slave_hostname = auto()
    ros_start_time = auto()

class CameraMode(Enum):
    WIDE_LOW = 'camid=2,1024,768'
    WIDE_FHD = 'camid=2,1280,960'
    WIDE_HIGH = 'camid=2,1920,1440'
    MAIN_LOW = 'camid=0,640,480'
    MAIN_FHD = 'camid=0,1920,1440'
    MAIN_HIGH = 'camid=0,2304,1728'
    FRONT_LOW = 'camid=1,640,480'
    FRONT_FHD = 'camid=1,1920,1440'
    FRONT_HIGH = 'camid=1,2304,1728'

class TRAY_TILT_STATUS(Enum):
    TiltDetectingMonitor = 90
    TiltDown = 0   #수직으로 밑을 봤을때
    TiltFace = -90    #똑바로 봤을때
    TiltMaxUp = 45
    TiltDiagonal = -45   #테이블 스캔모드 (Angle_Y : -45)
    TiltTrayCenter = 30

def TiltingARD(tiltStatus : TRAY_TILT_STATUS, smoothdelay=10):
    targetServo = mapRange(tiltStatus.value, minGYRO,maxGYRO,0,180)
    params = f"S:{smoothdelay},{round(targetServo)}"
    msg = f"q={params}"
    return API_call_http(IP_MASTER,HTTP_COMMON_PORT,EndPoints.ARD.name, msg)  

class TRAY_ARD_Field(Enum):
    GLA_SVN = auto()  # linear_acceleration
    GAV_SVN = auto()  # angular_velocity
    TEMPN = auto()  # mpu9250 온도
    GAV_TUN = auto()  # 가속도 3축 합산 절대값
    GOR_SVN = auto()  # orientation -> roll,pitch,yaw
    TEMPAHT_N = auto()  # AHT20 온도
    HUMIDAHT_N = auto()  # AHT20 습도
    PARTICLEN = auto()  # 미세먼지 (RED/IR/GREEN)
    RESULTN = auto()  # 명령어처리결과

class LIDAR_CROP_PARAMS(Enum):
    range_max_x = auto()  # 물체까지의 최대 감지 거리
    range_min_y = auto()  # 감지너비 좌측
    range_max_y = auto()  # 감지너비 우측
    range_min_z = auto()  # 감지너비 좌측
    range_max_z = auto()  # 감지너비 우측
    #Z는 워낙 좁아서 정의할 필요없음. (38도)

class LIDAR_DISTANCE_PARAMS(Enum):
    boxCenter = auto()  # 라이다 센터기준 박스의 센터 좌표 (x,y,z 문자열) - x가 거리, y가 너비, z가 높이
    width = auto()  # 감지너비 좌측
    height = auto()  # 감지너비 우측
    depth = auto()  # 감지너비 우측
    distance = auto()  # 감지너비 우측
    #Z는 워낙 좁아서 정의할 필요없음. (38도)

class ROS_PARAMS(Enum):
    startReal = auto()  # 런치 파일을 통해서 실행시켰는지 여부
    fastModbus = (
        auto()
    )  # itx 에서 modbus 통신 장치 구분 (True : 서보등 고속통신, False : BMS 등 저속통신)


def CheckNodeType(start,dictCross):
    lsCross = dictCross.keys()
    isStartCross = start in lsCross
    isStartHome = start == NODE_SPECIAL_VALUE.KITCHEN_STATION.value
    isStartPark = start == NODE_SPECIAL_VALUE.CHARGING_STATION.value
    isStartNormal = not (isStartCross or isStartPark or isStartHome)
    return isStartCross, isStartHome, isStartPark, isStartNormal

def GetNodeType(start, end,dictCross):
    isStartCross, isStartHome, isStartPark, isStartNormal = CheckNodeType(start,dictCross)
    isEndCross, isEndHome, isEndPark, isEndNormal = CheckNodeType(end,dictCross)
    if isStartCross:
        return APIBLB_NODETYPE.R.name
    if isEndCross:
        if isStartHome:
            return APIBLB_NODETYPE.H.name + APIBLB_NODETYPE.R.name
        if isStartPark:
            return APIBLB_NODETYPE.P.name + APIBLB_NODETYPE.R.name
        return APIBLB_NODETYPE.N.name + APIBLB_NODETYPE.R.name
    if isStartHome:
        return APIBLB_NODETYPE.H.name
    if isStartPark:
        return APIBLB_NODETYPE.P.name
    return APIBLB_NODETYPE.N.name

def getNodeMirrored(dicRecord,dictCross):
    #dicRecord = lsCurrentRow.tail(1).to_dict(orient='records')[0]
    startNew = dicRecord[APIBLB_FIELDS_INFO.end.name]
    endNew = dicRecord[APIBLB_FIELDS_INFO.start.name]
    directionNew = GetCounterDirection(dicRecord[APIBLB_FIELDS_INFO.direction.name])
    if startNew in dictCross:
        directionArr = dictCross[startNew]
        index = directionArr.index(endNew)
        if index == 0:  # North
            directionNew='N'
        elif index == 1:  # South
            directionNew='S'
        elif index == 2:  # East
            directionNew='E'
        elif index == 3:  # West
            directionNew='W'
    old_distance = dicRecord[APIBLB_FIELDS_INFO.distance.name]    
    new_X = dicRecord[APIBLB_FIELDS_INFO.et_xval.name]
    new_Y = dicRecord[APIBLB_FIELDS_INFO.et_yval.name]
    railtype_r = dicRecord[APIBLB_FIELDS_INFO.railtype.name]
    if railtype_r == 'ES1':
        railtype_r = 'LS1'
    elif railtype_r == 'LS1':
        railtype_r = 'ES1'

    dicNew = { APIBLB_FIELDS_INFO.start.name : startNew,
                APIBLB_FIELDS_INFO.end.name : endNew,
                APIBLB_FIELDS_INFO.speed.name : dicRecord[APIBLB_FIELDS_INFO.speed.name],
                APIBLB_FIELDS_INFO.distance.name : old_distance,
                APIBLB_FIELDS_INFO.direction.name : directionNew,
                APIBLB_FIELDS_INFO.nodetype.name : GetNodeType(startNew,endNew,dictCross),
                APIBLB_FIELDS_INFO.railtype.name : railtype_r,
                APIBLB_FIELDS_INFO.st_xval.name : new_X,
                APIBLB_FIELDS_INFO.st_yval.name : new_Y,
                APIBLB_FIELDS_INFO.et_xval.name : dicRecord[APIBLB_FIELDS_INFO.st_xval.name],
                APIBLB_FIELDS_INFO.et_yval.name : dicRecord[APIBLB_FIELDS_INFO.st_yval.name],
    } 
    return dicNew

def fill_pos_abs(df):        
    current_start_idx = 0
    while current_start_idx < len(df):
        # 다음 비제로 값을 찾습니다
        next_idx = current_start_idx + 1
        while next_idx < len(df) and df.iloc[next_idx]['POS_ABS'] == 0:
            next_idx += 1
            
        # 구간의 끝에 도달했거나 마지막 행인 경우 중단
        if next_idx >= len(df):
            break
            
        # 시작과 끝 값을 가져옵니다
        start_pos = df.iloc[current_start_idx]['POS_ABS']
        end_pos = df.iloc[next_idx]['POS_ABS']
        
        # 구간 내 전체 거리를 계산합니다
        total_distance = sum(df.iloc[current_start_idx:next_idx+1][APIBLB_FIELDS_INFO.distance.name])
        
        # 누적 거리를 계산하고 비율에 따라 값을 할당합니다
        cumulative_distance = 0
        for idx in range(current_start_idx + 1, next_idx):
            cumulative_distance += df.iloc[idx-1][APIBLB_FIELDS_INFO.distance.name]
            ratio = cumulative_distance / total_distance
            interpolated_value = start_pos + (end_pos - start_pos) * ratio
            df.at[idx, 'POS_ABS'] = round(interpolated_value)
        
        current_start_idx = next_idx

def getNodeNested(nodeNested,old_distance,dicRecord,dictCross, df):
    # if nodeNested in dictCross:# or endNode in lsCross[1:]:
    #     print(dicRecord)
    # startNode = int(dicRecord[APIBLB_FIELDS_INFO.start.name])
    # endNode = int(dicRecord[APIBLB_FIELDS_INFO.end.name])
    directionNew = dicRecord[APIBLB_FIELDS_INFO.direction.name]
    startNew = dicRecord[APIBLB_FIELDS_INFO.end.name]
    endNew = nodeNested

    if startNew in dictCross or endNew in dictCross:
        print(dicRecord)
        if startNew in dictCross:
            directionArr = dictCross[startNew]
            index = directionArr.index(endNew)
        else:
            directionArr = dictCross[endNew]
            index = directionArr.index(startNew)                        
        
        if index == 0:  # North
            direction='N'
        elif index == 1:  # South
            direction='S'
        elif index == 2:  # East
            direction='E'
        elif index == 3:  # West
            direction='W'
        if endNew in dictCross:
            direction = GetCounterDirection(direction)
        if directionNew != direction:
            directionNew = direction
    
    old_X = dicRecord[APIBLB_FIELDS_INFO.et_xval.name]
    old_Y=dicRecord[APIBLB_FIELDS_INFO.et_yval.name]
    new_X, new_Y = GetNewPos(old_X,old_Y, (directionNew), old_distance)
    railtype_r = 'N'
    # 조건에 맞는 행을 가져와 dict로 변환
    if df.empty:
        result = []  # 빈 리스트 반환
    else:
        result = df[
            ((df[APIBLB_FIELDS_INFO.start.name] == startNew) & (df[APIBLB_FIELDS_INFO.end.name] == endNew)) |
            ((df[APIBLB_FIELDS_INFO.start.name] == endNew) & (df[APIBLB_FIELDS_INFO.end.name] == startNew))
        ].to_dict(orient='records')
    if len(result) > 0:
        dictR_Info = result[0]
        start_r = dictR_Info.get(APIBLB_FIELDS_INFO.start.name)
        end_r = dictR_Info.get(APIBLB_FIELDS_INFO.end.name)
        distance_r = dictR_Info.get(APIBLB_FIELDS_INFO.distance.name)
        angle_r = dictR_Info.get(APIBLB_FIELDS_INFO.angle.name)
        railtype_r = dictR_Info.get(APIBLB_FIELDS_INFO.railtype.name)
        if start_r == endNew:
            angle_r = (angle_r+180)%360
            if railtype_r == 'ES1':
                railtype_r = 'LS1'
            else:
                railtype_r = 'ES1'
        new_X, new_Y = calculate_coordinates(old_distance,angle_r,old_X,old_Y)
        directionNew = GetDirectionFromAngle(angle_r)
        
    
    #old_distance = dicRecord[APIBLB_FIELDS_INFO.distance.name]
    dicNew = { APIBLB_FIELDS_INFO.start.name : startNew,
                APIBLB_FIELDS_INFO.end.name : endNew,
                APIBLB_FIELDS_INFO.speed.name : dicRecord[APIBLB_FIELDS_INFO.speed.name],
                APIBLB_FIELDS_INFO.distance.name : old_distance,
                APIBLB_FIELDS_INFO.direction.name : directionNew,
                APIBLB_FIELDS_INFO.nodetype.name : GetNodeType(startNew,endNew,dictCross),
                APIBLB_FIELDS_INFO.railtype.name : railtype_r,
                APIBLB_FIELDS_INFO.et_xval.name : new_X,
                APIBLB_FIELDS_INFO.et_yval.name : new_Y,
                APIBLB_FIELDS_INFO.st_xval.name : dicRecord[APIBLB_FIELDS_INFO.et_xval.name],
                APIBLB_FIELDS_INFO.st_yval.name : dicRecord[APIBLB_FIELDS_INFO.et_yval.name],
          }
    return dicNew
    

def getStateBranch(nodeID: int):
    stateNode = StateBranch.STATUS.get(nodeID, StateBranchValue.ERROR)
    return stateNode


def setStateBranch(nodeID: int, stateBranch: bool):
    setValue = StateBranchValue.OPEN if isTrue(stateBranch) else StateBranchValue.CLOSE

    logmsg = f"{nodeID},{setValue} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
    log_all_frames(logmsg)
    stateNode = StateBranch.STATUS.get(nodeID, StateBranchValue.ERROR)
    # if stateNode == StateBranchValue.ERROR:
    #     return False
    StateBranch.STATUS[nodeID] = setValue
    log_all_frames(StateBranch.STATUS[nodeID])
    return True


def bfs(graph, start):
    visited = set()
    queue = deque([start])

    while queue:
        node = queue.popleft()
        if node not in visited:
            visited.add(node)
            queue.extend([n for n, _ in graph.get(node, [])])

    return visited

# Refined function to ensure proper cascading direction propagation for general nodes
def enhanced_graph_to_bidirectional_list_refined(dictMap, dictCross):
    direction_map = {0: "N", 1: "S", 2: "E", 3: "W"}  # Mapping index to direction
    reverse_direction_map = {"N": "S", "S": "N", "E": "W", "W": "E"}  # Reverse direction map
    bidirectional_list = []
    processed_pairs = set()  # To track processed start-end pairs and avoid duplication

    # Helper function to find direction for crossroad nodes
    def find_direction_from_crossroad(start, end):
        if start in dictCross:
            if end in dictCross[start]:
                index = dictCross[start].index(end)
                return direction_map.get(index, "")
        return ""

    # Step 1: Process crossroad nodes first
    def process_crossroad_node(start, connections):
        for end, distance in connections:
            if (start, end) not in processed_pairs and (end, start) not in processed_pairs:
                direction = find_direction_from_crossroad(start, end)
                reverse_direction = reverse_direction_map.get(direction, "")
                
                bidirectional_list.append({
                    "start": str(start),
                    "end": str(end),
                    "distance": str(distance),
                    "speed": "500",
                    "direction": direction,
                    "rotation_yesno": "Y"
                })
                bidirectional_list.append({
                    "start": str(end),
                    "end": str(start),
                    "distance": str(distance),
                    "speed": "500",
                    "direction": reverse_direction,
                    "rotation_yesno": "Y"
                })

                processed_pairs.add((start, end))
                processed_pairs.add((end, start))

    # Step 2: Process general nodes connected to crossroads
    def process_general_node(start, connections):
        for end, distance in connections:
            if (start, end) not in processed_pairs and (end, start) not in processed_pairs:
                # Cascading direction: use the first connected node's direction
                for neighbor, _ in connections:
                    if (start, neighbor) in processed_pairs:
                        previous_direction = next(item["direction"] for item in bidirectional_list if item["start"] == str(start) and item["end"] == str(neighbor))
                        direction = reverse_direction_map.get(previous_direction, "")
                        break
                else:
                    direction = "E"  # Fallback in case no neighbor is processed yet
                
                reverse_direction = reverse_direction_map.get(direction, "")

                bidirectional_list.append({
                    "start": str(start),
                    "end": str(end),
                    "distance": str(distance),
                    "speed": "500",
                    "direction": direction,
                    "rotation_yesno": "N"
                })
                bidirectional_list.append({
                    "start": str(end),
                    "end": str(start),
                    "distance": str(distance),
                    "speed": "500",
                    "direction": reverse_direction,
                    "rotation_yesno": "N"
                })

                processed_pairs.add((start, end))
                processed_pairs.add((end, start))

    # First process crossroad nodes
    for crossroad in dictCross.keys():
        if crossroad in dictMap:
            process_crossroad_node(crossroad, dictMap[crossroad])

    # Then process general nodes
    for start, connections in dictMap.items():
        if start not in dictCross:
            process_general_node(start, connections)

    return bidirectional_list

def LoadGraph(strFilePath: str) -> dict:
    """LoadGraph 아래와 같은 텍스트를 읽어들여 경로정보를 생성한다.
    또한 모든 노드에 도달할 수 있는지를 체크하고 끊어진 노드가 있으면 False 를 리턴한다.

    1 200 5
    200 3 10
    2 100 10
    3 5 12
    100 5 10

    Args:
        strFilePath (str): _description_

    Returns:
        dict: _description_
        {"start":"1","end":"2","distance":"5000","speed":"500","direction":"S","nodetype":"HR",
        "st_xval":"5","st_yval":"4","et_xval":"6","end_yval":"8","rail_type":"LS/ES/N"}        
    """
    file_list = getLines_FromFile(strFilePath)
    node_seq = []
    graphTmp: Dict[int, int] = {}
    for i in file_list:
        splitI = i.split(sDivTab)
        n_a, n_b, cost = map(int, splitI)
        strA = n_a
        strB = n_b
        if graphTmp.get(strA, None) is None:
            graphTmp[strA] = []
        if graphTmp.get(strB, None) is None:
            graphTmp[strB] = []

        graphTmp[strA].append((strB, cost))
        graphTmp[strB].append((strA, cost))
        node_seq.append(f'{strA}_{strB}')
        
    node_all = graphTmp.keys()
    bMapVerified = True
    for nodeCurrent in node_all:
        reachable_nodes = bfs(graphTmp, nodeCurrent)
        if Counter(reachable_nodes) == Counter(node_all):
            pass
        else:
            log_all_frames(f"{nodeCurrent} is False : {nodeCurrent}-{reachable_nodes}")
            bMapVerified = False
            break
    #print(type(graphTmp), graphTmp)
    return graphTmp, bMapVerified,node_seq

def getNodeLocationInfo(bidirectional_list):
    node_xy = {}
    for dicTmp in bidirectional_list:
            # APIBLB_FIELDS_INFO.st_xval.name: str(end_x),
            # APIBLB_FIELDS_INFO.st_yval.name: str(end_y),
        startTmp = dicTmp[APIBLB_FIELDS_INFO.start.name]
        startX = dicTmp[APIBLB_FIELDS_INFO.st_xval.name]
        startY = dicTmp[APIBLB_FIELDS_INFO.st_yval.name]
        node_xy[startTmp] = [startX,startY]
        if startTmp == 3:
            log_all_frames(node_xy)
    return node_xy   

def LoadFullGraph2(dictMap, dictCross,strCSV_NodeInfo,strCSV_RrailInfo):
    try:
        dfRail = pd.read_csv(strCSV_RrailInfo, sep=sDivTab)
    except FileNotFoundError:
        # 파일이 없으면 빈 DataFrame 생성
        dfRail = pd.DataFrame()                    
    try:
        dfResult = pd.read_csv(strCSV_NodeInfo, sep=sDivTab)
        return dfResult, getNodeLocationInfo(dfResult.to_dict(orient='records'))
    except FileNotFoundError:
        # 파일이 없으면 빈 DataFrame 생성
        dfResult = pd.DataFrame()    
                        
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
          
          result_set = df_set
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
                dicMirror = getNodeMirrored(dicNested,dictCross)
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

def LoadFullGraph(dictMap, dictCross,strCSV_NodeInfo,strCSV_RrailInfo):
    try:
        dfRail = pd.read_csv(strCSV_RrailInfo, sep=sDivTab)
    except FileNotFoundError:
        # 파일이 없으면 빈 DataFrame 생성
        dfRail = pd.DataFrame()                    
    try:
        dfResult = pd.read_csv(strCSV_NodeInfo, sep=sDivTab)
        return dfResult, getNodeLocationInfo(dfResult.to_dict(orient='records'))
    except FileNotFoundError:
        # 파일이 없으면 빈 DataFrame 생성
        dfResult = pd.DataFrame()                    
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
        #dfGraph=graph_to_dataframe(dictMap)
        dfs(lsCross[0], 0, 0, parent=None,isRecur=False)
        lsCross2 = lsCross[1:]
        for i in range(len(lsCross2)):
          crossNodeID = lsCross2[i]
          distanceTmp = calculate_distance(dictMap, lsCross[0],crossNodeID)
          dfs(crossNodeID, 0, distanceTmp,parent=None,isRecur=False)  # Start from node 2 at (0,0)
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
          
          result_set = df_set.difference(dictCross.keys())
          # 시간 측정 시작
          start_time = time.time()
          for nodeToCheck in result_set:
              lsNestedNodesCurrent = dictMap[nodeToCheck]
              for nodeInfo in lsNestedNodesCurrent:
                nodeNested = nodeInfo[0]
                conditionToUpdate = (df[APIBLB_FIELDS_INFO.start.name] == nodeToCheck) & (df[APIBLB_FIELDS_INFO.end.name] == nodeNested)
                matching_Add2 = df.loc[conditionToUpdate].to_dict(orient="records")
                if len(matching_Add2) > 0:
                    continue
                lsRecord = df.loc[df[APIBLB_FIELDS_INFO.end.name] ==  nodeToCheck].to_dict(orient="records")
                dicRecord = lsRecord[0]
                #print(matching_Add2)
                #print(dicRecord)
                node_distance = nodeInfo[1]
                dicNested = getNodeNested(nodeNested,node_distance,dicRecord,dictCross,dfRail)
                dicMirror = getNodeMirrored(dicNested,dictCross)
                #if len(matching_Add2) == 0:
                df = pd.concat([df, pd.DataFrame([dicNested,dicMirror])], ignore_index=True)
                    #print(df)                
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

def getCostPath(graph: dict, nodeStart, nodeEnd):
    costList = graph[nodeStart]
    for costTuple in costList:
        if costTuple[0] == nodeEnd:
            return costTuple[1]
    return -1


def dijkstra(graph: dict, node):
    """최단 경로 전체정보를 구하는데 필요한 lead_time 과 node_from 값을 구한다.
    예제 : lead_time, node_from = dijkstra(graph, "A")
    Args:
        graph = {
            "A": [("B", 1), ("C", 4), ("D", 5)],
            "B": [("A", 1), ("D", 2)],
            "C": [("A", 4), ("D", 4), ("E", 3)],
            "D": [("A", 5), ("B", 2), ("C", 4), ("F", 3)],
            "E": [("C", 3), ("F", 2)],
            "F": [("D", 3), ("E", 2)]
        }
        node (_type_): 시작노드 지정

    Returns:
        lead_time = {'1': None, '2': '1', '3': '2', '4': '2', '5': '4'} #node 를 중심으로 한 경로가중치값
        node_from = {'1': 0, '2': 5, '3': 15, '4': 15, '5': 25} #각 노드까지의 최단거리.
    """
    # 소요시간을 기록할 사전과 현재 노드의 이전 노드를 기록할 사전을 초기화
    lead_time = {node: math.inf for node in graph}
    node_from = {node: None for node in graph}

    # 출발 노드(node)의 소요 시간을 0으로 만든다.
    lead_time[node] = 0
    visited = set()

    # (소요 시간, 노드)의 자료를 넣는 최소 힙을 만든다.
    # 그리고 출발 노드를 힙에 넣는다.
    heap = []
    heapq.heappush(heap, (0, node))

    # 힙이 빌 때까지 반복한다.
    while heap:
        # 힙에서 자료를 꺼내서 시간과 노드를 변수에 저장한다.
        prev_time, u = heapq.heappop(heap)

        # 이미 방문한 노드이면 다음 노드를 꺼내고, 그렇지 않으면 방문처리한다.
        if u in visited:
            continue
        visited.add(u)

        # 현재 노드에 인접한 노드와 가중치(소요 시간)를 가져와서 반복한다.
        for v, weight in graph[u]:
            # 인접한 노드로 가는 소요 시간을 계산하여 기본 값보다 작으면,
            # 시간 정보를 갱신하고, 이전 노드를 기록한다. 그리고 힙에 넣는다.
            if (new_time := prev_time + weight) < lead_time[v]:
                lead_time[v] = new_time
                node_from[v] = u
                heapq.heappush(heap, (lead_time[v], v))
    # 소요 시간과 이전 노드를 기록한 사전을 반환한다.
    return lead_time, node_from


def shortest_path(graph: dict, node_from, lead_time, end):
    returnPath = []
    returnCosts = []
    path = ""
    node = end
    while node_from[node]:
        returnPath.insert(0, node)
        if len(returnPath) >= 2:
            curCost = getCostPath(graph, returnPath[0], returnPath[1])
            returnCosts.insert(0, curCost)
        path = " → " + str(node) + path
        node = node_from[node]
    lead_time_end = str(lead_time[end])
    curCost = getCostPath(graph, returnPath[0], node)
    returnCosts.insert(0, curCost)
    # print(f"{str(node) + path} (cost = {lead_time_end})")
    # print(f"{str(node) + path} (cost = {returnCosts})")
    # returnPath.append(end)
    # return returnPath,lead_time_end
    return returnPath, returnCosts


def getPathBLB(graph, start, endPoint):
    lead_time, node_from = dijkstra(graph, start)
    lsResult, cost = shortest_path(graph, node_from, lead_time, endPoint)
    lsResult.insert(0, start)
    return lsResult, cost


def getSignFromInt(distancePrevStr: str):
    distancePrev = int(distancePrevStr)
    signint = 1 if distancePrev > 0 else -1
    return signint, distancePrev


def getMotorMoveString(mbid, isMode: bool, pos, spd, acc, decc):
    modeVal = (
        ServoParam.MOVE_TYPE_ABS.value if isMode else ServoParam.MOVE_TYPE_REL.value
    )
    sCmd = (
        f"{MotorWMOVEParams.MBID.name}{sDivFieldColon}{mbid}{sDivItemComma}"
        f"{MotorWMOVEParams.CMD.name}{sDivFieldColon}{MotorCmdField.WMOVE.name}{sDivItemComma}"
        f"{MotorWMOVEParams.MODE.name}{sDivFieldColon}{modeVal}{sDivItemComma}"
        f"{MotorWMOVEParams.POS.name}{sDivFieldColon}{int(pos)}{sDivItemComma}"
        f"{MotorWMOVEParams.SPD.name}{sDivFieldColon}{int(spd)}{sDivItemComma}"
        f"{MotorWMOVEParams.ACC.name}{sDivFieldColon}{int(acc)}{sDivItemComma}"
        f"{MotorWMOVEParams.DECC.name}{sDivFieldColon}{int(decc)}"
    )
    return sCmd

def getMotorMoveDic(mbid, isMode: bool, pos, spd, acc, decc):
    strVal = getMotorMoveString(mbid, isMode, pos, round(spd), round(acc), round(decc))
    return getDic_strArr(strVal, sDivFieldColon, sDivItemComma)

def getMotorSpeedString(mbid, isMode: bool, spd, acc, decc):
    sCmd = (
        f"{MotorWMOVEParams.MBID.name}{sDivFieldColon}{mbid}{sDivItemComma}"
        f"{MotorWMOVEParams.CMD.name}{sDivFieldColon}{MotorCmdField.WSPD.name}{sDivItemComma}"
        f"{MotorWMOVEParams.SPD.name}{sDivFieldColon}{spd}{sDivItemComma}"
        f"{MotorWMOVEParams.ACC.name}{sDivFieldColon}{acc}{sDivItemComma}"
        f"{MotorWMOVEParams.DECC.name}{sDivFieldColon}{decc}"
    )
    return sCmd

def getMotorSpeedDic(mbid, isMode: bool, spd, acc, decc):
    strVal = getMotorSpeedString(mbid, isMode, round(spd), round(acc), round(decc))
    return getDic_strArr(strVal, sDivFieldColon, sDivItemComma)

def getMotorDefaultString(mbid, mtr):
    sCmd = (
        f"{MotorWMOVEParams.MBID.name}{sDivFieldColon}{mbid}{sDivItemComma}"
        f"{MotorWMOVEParams.CMD.name}{sDivFieldColon}{mtr}{sDivItemComma}"
    )
    return sCmd

def getMotorDefaultDic(mbid, mtr):
    strVal = getMotorDefaultString(mbid, mtr)
    return getDic_strArr(strVal, sDivFieldColon, sDivItemComma)

def getMotorWN_ONDic(mbid = ModbusID.MOTOR_H.value):
    return getMotorDefaultDic(mbid, MotorCmdField.WN_ON.name)

def getMotorWP_ONDic(mbid = ModbusID.MOTOR_H.value):
    return getMotorDefaultDic(mbid, MotorCmdField.WP_ON.name)

def getMotorWPN_OFFDic(mbid = ModbusID.MOTOR_H.value):
    return getMotorDefaultDic(mbid, MotorCmdField.WPN_OFF.name)

def getMotorStopString(mbid,decc=EMERGENCY_DECC):
    sCmd = (
        f"{MotorWMOVEParams.MBID.name}{sDivFieldColon}{mbid}{sDivItemComma}"
        f"{MotorWMOVEParams.CMD.name}{sDivFieldColon}{MotorCmdField.WSTOP.name}{sDivItemComma}"
        f"{MotorWMOVEParams.DECC.name}{sDivFieldColon}{decc}"
    )
    return sCmd

def getMotorLocationSetString(mbid,positionPulse=EMERGENCY_DECC):
    sCmd = (
        f"{MotorWMOVEParams.MBID.name}{sDivFieldColon}{mbid}{sDivItemComma}"
        f"{MotorWMOVEParams.CMD.name}{sDivFieldColon}{MotorCmdField.WLOC.name}{sDivItemComma}"
        f"{MotorWMOVEParams.POS.name}{sDivFieldColon}{positionPulse}"
    )
    return sCmd

def getMotorTorqueString(mbid,tql=TORQUE_LIMIT_DEFAULT):
    sCmd = (
        f"{MotorWMOVEParams.MBID.name}{sDivFieldColon}{mbid}{sDivItemComma}"
        f"{MotorWMOVEParams.CMD.name}{sDivFieldColon}{MotorCmdField.WTORQUE.name}{sDivItemComma}"
        f"{MotorWMOVEParams.TQL.name}{sDivFieldColon}{tql}"
    )
    return sCmd

def getMotorSetPOTNOTString(mbid,not_pos=0,pot_pos = 0):
    sCmd = (
        f"{MotorWMOVEParams.MBID.name}{sDivFieldColon}{mbid}{sDivItemComma}"
        f"{MotorWMOVEParams.CMD.name}{sDivFieldColon}{MotorCmdField.WLIMIT.name}{sDivItemComma}"
        f"{MotorWMOVEParams.POT.name}{sDivFieldColon}{pot_pos}{sDivItemComma}"
        f"{MotorWMOVEParams.NOT.name}{sDivFieldColon}{not_pos}"
    )
    return sCmd

def getMotorTorqueDic(mbid,tql=EMERGENCY_DECC):
    strVal = getMotorTorqueString(mbid,tql)
    return getDic_strArr(strVal, sDivFieldColon, sDivItemComma)

def getMotorLocationSetDic(mbid,position_pulse):
    strVal = getMotorLocationSetString(mbid,position_pulse)
    return getDic_strArr(strVal, sDivFieldColon, sDivItemComma)

def getMotorStopDic(mbid,decc=EMERGENCY_DECC):
    strVal = getMotorStopString(mbid,decc)
    return getDic_strArr(strVal, sDivFieldColon, sDivItemComma)

def getMotorHomeString(mbid):
    sCmd = (
        f"{MotorWMOVEParams.MBID.name}{sDivFieldColon}{mbid}{sDivItemComma}"
        f"{MotorWMOVEParams.CMD.name}{sDivFieldColon}{MotorCmdField.WZERO.name}{sDivItemComma}"
    )
    return sCmd

def getMotorHomeDic(mbid):
    strVal = getMotorHomeString(mbid)
    return getDic_strArr(strVal, sDivFieldColon, sDivItemComma)

def getMotorSimpleCmdString(mbid, mtr: MotorCmdField):
    sCmd = (
        f"{MotorWMOVEParams.MBID.name}{sDivFieldColon}{mbid}{sDivItemComma}"
        f"{MotorWMOVEParams.CMD.name}{sDivFieldColon}{mtr.name}{sDivItemComma}"
    )
    return sCmd

def getMotorSimpleCmdDic(mbid, mtr: MotorCmdField):
    strVal = getMotorSimpleCmdString(mbid,mtr)
    return getDic_strArr(strVal, sDivFieldColon, sDivItemComma)




class NodeType(Enum):
    TABLE = auto()
    CROSS = auto()
    LIFT = auto()
    RESERVED = auto()

    def GetNodeType(nodeID: str):
        chk = try_parse_int(nodeID, -1)
        if chk < 0:
            return NodeType.RESERVED
        if chk >= 200:
            return NodeType.CROSS
        if chk >= 100:
            return NodeType.LIFT
        return NodeType.TABLE


class CmdStatus(Enum):
    Alarm = auto()
    Canceled = auto()
    Started = auto()
    Finished = auto()


class ReportFields(Enum):
    DATE = auto()
    TIMESTAMP_START = auto()
    TIMESTAMP_END = auto()
    VOLTAGE_START = auto()
    VOLTAGE_END = auto()
    IMG_TRIED = auto()
    IMG_PASSED = auto()
    IMG_SKIPPED = auto()
    IMG_SKIPPED_LOG = auto()


# class NEST_TOPIC(Enum):
#     scan_alarm = "scan_alarm_util"
#     scheduler = "scheduler"
#     utilIMU = "imu"
#     nestIMU = "/nestImu"
#     scanutil = "/scanutil"
#     SD = "SD"
#     SERVO = "SERVO"
#     RFID = "RFID"
#     BMS = "GOAL"
#     CMD = "CMD"
#     FEEDBACK = "FEEDBACK"
#     KEEPALIVE = "KEEPALIVE"
#     ENCODER = "ENCODER"
#     GOAL = "GOAL"
#     ALARM = "ALARM"
#     UTIL_ARD = "KEEP_UTIL"
#     KEEP_ARD = "KEEP_ARD"

#     def GetEnumValue(strCurrent):
#         sCurrent = str(strCurrent)
#         for s in Config_Main:
#             sCurrentTmp = str(s)
#             if sCurrent.endswith(sCurrentTmp):
#                 return s.value
#         return -1


class LED_Status(Enum):
    # RED
    ON_REDSOLID = ["P:7,248", "B:0"]
    CHARGING = ["P:7,248", "B:0"]
    ERROR_CHARGE = ["P:7,216", "B:1", "L:1500"]
    ERROR_FATAL = ["P:7,248", "B:1", "L:1000"]
    # GREEN
    ON_GREENSOLID = ["P:7,168", "B:0"]
    ON_CHARGEFULL = ["P:7,168", "B:0"]
    ON_OPERATION = ["P:7,136", "B:1", "L:1500"]
    ON_DATA = ["P:7,168", "B:1", "L:1000"]
    # BLUE
    ON_BLUESOLID = ["P:7,88", "B:0"]
    ON_ARDINO = ["P:7,88", "B:1", "L:1000"]
    ON_UTILBOXLOADED = ["P:7,88", "B:0"]
    ON_NODESERVO = ["P:7,56", "B:1", "L:1500"]

    def GetEnumValue(strCurrent):
        sCurrent = str(strCurrent)
        for s in Config_Main:
            sCurrentTmp = str(s)
            if sCurrent.endswith(sCurrentTmp):
                return s.value
        return -1


class CHARGE_Status(Enum):
    DISCHARGING = 2
    CHARGING = 0
    CHARGEFULL = 1
    UNKNOWN = -1

    def GetEnumValue(strCurrent):
        sCurrent = str(strCurrent)
        for s in Config_Main:
            sCurrentTmp = str(s)
            if sCurrent.endswith(sCurrentTmp):
                return s.value
        return -1


class _ServiceList(Enum):
    SDV16 = auto()
    SDV48 = auto()
    SDV24 = auto()
    SDV12 = auto()
    SDV5 = auto()

    SDV16LOCK = auto()
    SDV48LOCK = auto()
    SDV24LOCK = auto()
    SDV12LOCK = auto()
    SDV5LOCK = auto()

    MARKER_SCAN = auto()
    IMG_PUBLISH = auto()
    IMG_SAVE = auto()
    CMDARD = auto()
    CMDSCR = auto()
    CHARGE_ENABLE = auto()
    USB_ENABLE = auto()
    BREAK_ENABLE = auto()
    GPO_SET = auto()
    RFID_INV = auto()
    CAM_ENABLE = auto()
    GOAL = auto()
    ALARM = auto()
    ESTOP = auto()
    SAVE = auto()
    MODBUS_MOVE = auto()
    TRAY540_ROTATE = auto()
    TRAY360_ROTATE = auto()
    REC_CAM = auto()
    MODBUS_STATUS = auto()

    def GetEnumValue(strCurrent):
        sCurrent = str(strCurrent)
        for s in Config_Main:
            sCurrentTmp = str(s)
            if sCurrent.endswith(sCurrentTmp):
                return s.value
        return -1


class ServiceBLB(Enum):
    MarkerScan = f"/{UbuntuEnv.QBI.name}/{_ServiceList.MARKER_SCAN.name}"  # rosservice call /tray/MarkerScan 3 1
    IMG_PUBLISH = f"/{UbuntuEnv.QBI.name}/{_ServiceList.IMG_PUBLISH.name}"  # rosservice call /tray/IMG_PUBLISH True
    IMG_SAVE = f"/{UbuntuEnv.QBI.name}/{_ServiceList.IMG_SAVE.name}"
    SDV48 = f"/{UbuntuEnv.ITX.name}/{_ServiceList.SDV48.name}"
    SDV24 = f"/{UbuntuEnv.ITX.name}/{_ServiceList.SDV24.name}"
    SDV48LOCK = f"/{UbuntuEnv.ITX.name}/{_ServiceList.SDV48LOCK.name}"
    SDV24LOCK = f"/{UbuntuEnv.ITX.name}/{_ServiceList.SDV24LOCK.name}"
    CMDARD_ITX = f"/{UbuntuEnv.ITX.name}/{_ServiceList.CMDARD.name}"
    CMDSCR_ITX = f"/{UbuntuEnv.ITX.name}/{_ServiceList.CMDSCR.name}"
    CMDARD_QBI = f"/{UbuntuEnv.QBI.name}/{_ServiceList.CMDARD.name}"
    REC_CAM_QBI = f"/{UbuntuEnv.QBI.name}/{_ServiceList.REC_CAM.name}"
    CMDSCR_QBI = f"/{UbuntuEnv.QBI.name}/{_ServiceList.CMDSCR.name}"
    CHARGE_ENABLE = f"/{UbuntuEnv.ITX.name}/{_ServiceList.CHARGE_ENABLE.name}"
    CMD_DEVICE = f"/{UbuntuEnv.ITX.name}/{_ServiceList.MODBUS_MOVE.name}"
    CMD_ESTOP = f"/{UbuntuEnv.ITX.name}/{_ServiceList.ESTOP.name}"
    CMD_SAVE = f"/{UbuntuEnv.ITX.name}/{_ServiceList.SAVE.name}"
    CMD_ALMC = f"/{UbuntuEnv.ITX.name}/{_ServiceList.ALARM.name}"
    STATUS_DEVICE = f"/{UbuntuEnv.ITX.name}/{_ServiceList.MODBUS_STATUS.name}"
    # USB_ENABLE = auto()
    # BREAK_ENABLE = auto()
    # GPO_SET = auto()
    RFID_INV = f"/{UbuntuEnv.ITX.name}/{_ServiceList.RFID_INV.name}"
    CAM_ENABLE = f"/{UbuntuEnv.QBI.name}/{_ServiceList.CAM_ENABLE.name}"
    # GOAL = auto()
    ALARM = f"/{UbuntuEnv.QBI.name}/{_ServiceList.ALARM.name}"
    ROTATE_540 = f"/{UbuntuEnv.QBI.name}/{_ServiceList.TRAY540_ROTATE.name}"
    ROTATE_360 = f"/{UbuntuEnv.QBI.name}/{_ServiceList.TRAY360_ROTATE.name}"
    TTS_QBI = auto()
    TTS_ITX = auto()
    CTL_BLB = auto()


class Config_Main(Enum):
    ROS_SPIN_RATE = 0.5
    RFID_PWR = 1500
    lidar_Angle = 30
    lidar_Alarm = 2.0
    MOTOR_ACC_RATE = 150  # 많을수록 급가속 급정거
    bms_hide = True
    lidar_hide = True

    def GetEnumValue(strCurrent):
        sCurrent = str(strCurrent)
        for s in Config_Main:
            sCurrentTmp = str(s)
            if sCurrent.endswith(sCurrentTmp):
                return s.value
        return -1


# class ServoKey(Enum):
#     MOVE_TYPE = 0x6200
#     MOVE_POSITION_H = 0x6201
#     MOVE_POSITION_L = 0x6202
#     MOVE_SPD = 0x6203
#     MOVE_ACC_TIME = 0x6204
#     MOVE_DEC_TIME = 0x6205
#     MOVE_DWELL_TIME = 0x6206
#     MOVE_PATHNUMBER = 0x6207
#     SET_HOME1 = 0x600A
#     SET_HOME2 = 0x6002
#     SERVO_ENABLE = 0x405
#     E_STOP = 0x6002
#     MONITOR_LOC = 0x602A
#     MONITOR_STATUS = 0x6002  # 상태모니터링
#     CLEAR_ALARM = 0x33


class ServoParam(Enum):
    MOVE_TYPE_ABS = 1  # 절대위치
    MOVE_TYPE_REL = 65  # 상대위치
    MOVE_TYPE_SPD = 2  # 속도유지
    MOVE_PATHNUMBER_DEFAULT = 0x10  # PathNumber 설정값
    SET_HOME_HERE = 0x20
    SERVO_ENABLE = 0x405
    HOMING_RPM = 500
    E_STOP_NOW = 0x40
    MONITOR_LOC = 0x602A
    MONITOR_STATUS = 0x6002  # 상태모니터링
    CLEAR_ALARM_NOW = 0x1111

class BLB_LOCATION(Enum):
    """
    범블비(Bumblebee)의 현재 이동 거리 및 전개 위치를 나타내는 필드를 정의한 Enum 클래스.

    속성:
        DISTANCE_FROM_HOME: 범블비가 원점에서부터 이동한 거리
        DISTANCE_ARMEXTENDED: 범블비가 전개한 서빙암의 길이
        ANGLE_540: 범블비의 메인 회전 각도
        ANGLE_360: 범블비의 트레이 회전 각도
    """
    DISTANCE_FROM_HOME = auto()  # 원점에서부터 범블비가 이동한 거리
    DISTANCE_ARMEXTENDED = auto()  # 범블비가 전개한 서빙암 길이
    ANGLE_540 = auto()  # 범블비 메인회전 각도
    ANGLE_360 = auto()  # 범블비 트레이 회전 각도

def calculate_relative_distances(absolute_distances):
    # 절대 거리들을 받아 상대 거리로 변환합니다.
    relative_distances = [absolute_distances[0]]  # 첫 번째 상대 거리는 첫 절대 거리와 동일합니다.
    for i in range(1, len(absolute_distances)):
        relative_distances.append(absolute_distances[i] - absolute_distances[i - 1])
    return relative_distances

def update_graph_with_new_nodes(node1, node2, total_distance, lsNodeScannedmm, start_number):
    """
    기존 두 노드 사이에 새로운 노드들을 추가하고 그래프를 갱신하는 함수
    
    Parameters:
    node1: 시작 노드 번호
    node2: 끝 노드 번호
    total_distance: 전체 구간 거리
    new_node_distances: 시작점으로부터의 절대 거리 리스트
    start_number: 신규 노드 번호 시작값
    
    Returns:
    list: [node1, node2, distance] 형태의 엣지 정보 리스트
    """
    # 거리순으로 정렬
    lsNodeScannedmm.sort()
    
    # 새로운 노드 번호 할당
    new_nodes = [start_number + i for i in range(len(lsNodeScannedmm))]
    
    # 모든 노드의 위치를 순서대로 정렬
    all_nodes = [node1] + new_nodes + [node2]
    all_distances = [0] + lsNodeScannedmm + [total_distance]
    
    # 엣지 정보 생성
    edges = []
    for i in range(len(all_nodes)-1):
        current_node = all_nodes[i]
        next_node = all_nodes[i+1]
        distance = all_distances[i+1] - all_distances[i]
        edges.append([current_node, next_node, distance])
    
    return edges

# def calculate_average_values(data: Dict[int, List[Dict[str, Any]]]) -> Dict[int, Dict[str, float]]:
#     results = {}
    
#     for key, value_list in data.items():
#         angle_360_values = [item[BLB_LOCATION.ANGLE_360.name] for item in value_list if BLB_LOCATION.ANGLE_360.name in item]
#         x_values = [item[ARUCO_RESULT_FIELD.X.name] for item in value_list if ARUCO_RESULT_FIELD.X.name in item]
#         y_values = [item[ARUCO_RESULT_FIELD.Y.name] for item in value_list if ARUCO_RESULT_FIELD.Y.name in item]
        
#         results[key] = {
#             BLB_LOCATION.ANGLE_360.name: round(mean(angle_360_values)) if angle_360_values else None,
#             ARUCO_RESULT_FIELD.X.name: round(mean(x_values)) if x_values else None,
#             ARUCO_RESULT_FIELD.Y.name: round(mean(y_values)) if y_values else None
#         }
    
#     return results

def calculate_average_values(data: Dict[int, List[Dict[str, Any]]]) -> Dict[int, Dict[str, float]]:
    results = {}
    
    for key, value_list in data.items():
        averages = {}
        all_keys = {k for item in value_list for k in item.keys()}  # 각 value_list에 있는 모든 키 수집
        
        for k in all_keys:
            values = []
            for item in value_list:
                if k in item:
                    try:
                        # 숫자형이나 변환 가능한 문자열이면 float로 변환해서 추가
                        values.append(float(item[k]))
                    except (ValueError, TypeError):
                        # 변환 불가능한 경우 무시
                        continue
            
            # 변환 가능한 값들로 평균 계산
            if values:
                averages[k] = round(mean(values), 2)
        
        results[key] = averages
    
    return results

def print_average_values(avg_values: Dict[int, Dict[str, float]]):
    for key, values in avg_values.items():
        print(f"Marker {key}:")
        print(f"  Average ANGLE_360: {values[BLB_LOCATION.ANGLE_360.name]:.2f}" if values[BLB_LOCATION.ANGLE_360.name] is not None else "  Average ANGLE_360: N/A")
        print(f"  Average X: {values[ARUCO_RESULT_FIELD.X.name]:.2f}" if values[ARUCO_RESULT_FIELD.X.name] is not None else "  Average X: N/A")
        print(f"  Average Y: {values[ARUCO_RESULT_FIELD.Y.name]:.2f}" if values[ARUCO_RESULT_FIELD.Y.name] is not None else "  Average Y: N/A")
        print()


# 밸런스암 전개 상태코드
class STATUS_BALANCING(Enum):
  READY = auto()
  EXTEND_STARTED = auto()
  EXTEND_SPD_RAISED = auto()
  EXTEND_FINISHED = auto()
  SHRINK_STARTED = auto()
  SHRINK_SPD_DELAYED = auto()
  SHRINK_FINISHED = auto()

# 밸런스암 전개 상태코드
class STATUS_MOTOR(Enum):
  STOPPED = auto()
  PENDING_CW = auto()
  PENDING_CCW = auto()
  RUNNING_CW = auto()
  RUNNING_CCW = auto()
  UNKNOWN = auto()

# 범블비 맵 필드
class MAPFIELD(Enum):
  EDGES = auto()
  EPC = auto()
  TABLES = auto()

def find_node_rfid(graph, valParam):
  return find_node(graph, MAPFIELD.EPC, valParam)

def find_node(graph, mapfield : MAPFIELD, valParam):
    for node, data in graph.items():
        if data.get(mapfield.name, None) == valParam:
            return node, data
    return None, None

def GetNodeEPCInfoDic():
    try:
        df = pd.read_csv(strFileEPC_total, sep=sDivTab)
        epcnodeinfo = df_to_dict_int_values(df,TableInfo.NODE_ID.name,MAPFIELD.EPC.name)
        return epcnodeinfo
    except Exception as e:
        return {}
  
def GetEPCNodeInfoDic():
    try:
        df = pd.read_csv(strFileEPC_total, sep=sDivTab)
        epcnodeinfo = df_to_dict_int_values(df, MAPFIELD.EPC.name, TableInfo.NODE_ID.name)
        return epcnodeinfo
    except Exception as e:
        return {}
  
def GetNodeID_fromEPC(epc):
    df = pd.read_csv(strFileEPC_total, sep=sDivTab)
    result = get_value_by_key_fromDF(df, MAPFIELD.EPC.name, epc, TableInfo.NODE_ID.name)
    return result

def GetNodePos_fromEPC(epc):
    df = pd.read_csv(strFileEPC_total, sep=sDivTab)
    result = get_value_by_key_fromDF(df, MAPFIELD.EPC.name, epc, MotorWMOVEParams.POS.name)
    return result

def GetNodePos_fromNode_ID(node_id):
    df = pd.read_csv(strFileEPC_total, sep=sDivTab)
    result = get_value_by_key_fromDF(df, TableInfo.NODE_ID.name, node_id, MotorWMOVEParams.POS.name)
    return result

def GetEPC_Loc_Master(epc):
    df = pd.read_csv(strFileEPC_total, sep=sDivTab)
    #epcnodeinfo = df_to_dict_int_values(df, MAPFIELD.EPC.name, MonitoringField.CUR_POS.name)
    result=get_last_value_for_key(df, MAPFIELD.EPC.name, epc,MotorWMOVEParams.POS.name)
    return result
  
# class ServoMonitorLOC(Enum):
#     # ServoMonitorLOC_Addr = 0x602A
#     # ServoMonitorLOC_Items = ['LOC_CMD_1', 'LOC_CMD_2', 'LOC_MOT_1', 'LOC_MOT_2',4,5]
#     ServoMonitorLOC_Addr = 0x6200
#     ServoMonitorLOC_Items = [
#         "PRO_MODE",
#         "LOCCMD_1",
#         "LOCCMD_2",
#         "SPD_CMD",
#         "ACC_TIME",
#         "DEC_TIME",
#     ]
#     ServoMonitorLOC_Len = len(ServoMonitorLOC_Items)


# class ServoMonitorALM(Enum):
#     ALM_CD = 0
#     ALM_STR = "READY"
#     ServoMonitorALM_Addr = 0x0B03
#     ServoMonitorALM_Items = [
#         "ALM",
#         "MOTOR_FACTOR",
#         "DRV_STATE",
#         "ACTU_SPD",
#         "ACT_TORQ",
#         "ACT_CURRENT",
#         "ACTF_SPD",
#         "DC_VOLT",
#         "DRV_TEMPER",
#         12,
#         13,
#         14,
#         "OVERLOAD_RATIO",
#         "REGENLOAD_RATIO",
#         17,
#         18,
#         19,
#         20,
#         21,
#         22,
#         23,
#         24,
#         25,
#         26,
#         27,
#         "LOCMOT_1",
#         "LOCMOT_2",
#         "DEVI_ENC_1",
#         "DEVI_ENC_2",
#         30,
#         31,
#     ]
#     # ServoMonitorALM_Items = ['ALM', 'MOTOR_FACTOR', 'DRV_STATE', 'ACTU_SPD','ACT_TORQ','ACT_CURRENT', 'ACTF_SPD', 'DC_VOLT'
#     # ,'DRV_TEMPER', 12,13,14,'OVERLOAD_RATIO', 'REGENLOAD_RATIO', 'DIN_STATUS', 'DOUT_STATUS', 19, 'LOCCMD_1'
#     # , 'LOCCMD_2','PULSE_1','PULSE_2', 'DEVI_CMD_1','DEVI_CMD_2','LOCENC_1'
#     # , 'LOCENC_2','LOCMOT_1', 'LOCMOT_2','DEVI_ENC_1','DEVI_ENC_2','POS_RORATION_1','POS_RORATION_2']
#     ServoMonitorALM_Len = len(ServoMonitorALM_Items)


# class ServoMonitorStatus(Enum):
#     NAME = "MONITOR_STATUS"
#     STATUS_ADDR = 0x6002
#     ERR01 = 0x01
#     ERR20 = 0x20
#     ERR40 = 0x40
#     READY = 0  # 위치 지정이 완료되고 새 데이터를 수신 할 수 있음을 나타냄 Ready
#     MOVING = 0x100  # 경로가 실행 중임을 표시 Moving
#     FINISHING = 0x200  # 경로가 실행 중임을 표시 Moving

#ip_dict = {item.name: item.value for item in IPList}

class PIN_BCM(Enum):
    FORCELIFT_I = 26
    LOGO_O = 17
    NEST_DOCK_I = 16
    MAIN_DOCK_I = 19
    NEST_CHARGE_O = 22
    MAIN_CHARGE_O = 27
    USB_SHARE_O = 10
    SERVO_PWM = 18
    BREAK_PWM = 13
    BREAK_O = 4
    BREAK_H_O = 20
    BREAK_EVT_H_I = 5
    BREAK_EVT_V_I = 6
    ARD_ENABLE_O = 9
    # ARD_21_I = 5
    # ARD_20_I = 6
    ARD_19_I = 26
    SMOOTH_EVT_H_I = 23
    SMOOTH_EVT_V_I = 24
    ARD_10_I = 25

    def GetEnumValue(strCurrent):
        sCurrent = str(strCurrent)
        for s in PIN_BCM:
            sCurrentTmp = str(s)
            if sCurrent.endswith(sCurrentTmp):
                return s.value
        return -1

def getSpeedTableInfo(MBID, k,adjustRate=1.0,file_path = strFileTableSpd):
    # if tableNo == node_CHARGING_STATION:
    #     tableNo = 'H1'
    MBIDKey = MotorWMOVEParams.MBID.name
    df_manager = DataFrameManager(file_path)
    df = df_manager.filter_by_key(MBIDKey, (MBID))
    #print(df)
    df_adjusted = df.copy()
    #print(df_adjusted)
    df_adjusted.loc[:, df.columns != MBIDKey] = np.round(df_adjusted.loc[:, df.columns != MBIDKey] * adjustRate)
    #dicRt = df_adjusted.to_dict(orient='records')
    dicRt = df_adjusted.to_dict(orient='records')[0]
    #print(dicRt)
    return round(try_parse_float(dicRt[k]))

def getConfigCommon() -> dict:
    filePath_modbusconfig = f"{getConfigPath(UbuntuEnv.COMMON.name)}/deviceID.txt"
    dicConfigTmp = getDic_FromFile(filePath_modbusconfig, sDivEmart)
    return dicConfigTmp


dictCommon = getConfigCommon()
device_ID = dictCommon.get(Config_Common.DEVICE_ID.name, "NONAME")


def SendMsgToMQTT(pub_topic2mqtt, topicTmp, payloadTmp):
    dicSendMqttTopic = {}
    dicSendMqttTopic[MQTT_FIELD.TOPIC.name] = topicTmp
    dicSendMqttTopic[MQTT_FIELD.PAYLOAD.name] = payloadTmp
    data_out = json.dumps(dicSendMqttTopic)
    logmsg = f"Send {data_out} msg from {sys._getframe(0).f_code.co_name}-{sys._getframe(1).f_code.co_name}"
    rospy.loginfo(logmsg)
    pub_topic2mqtt.publish(data_out)


def getSimplePath(lsTest):
    lsFinal = []
    startDic = lsTest.pop(0)
    while len(lsTest) > 0:
        curDic = lsTest.pop(0)
        startDic[SeqMapField.DISTANCE.name] += curDic[SeqMapField.DISTANCE.name]
        startDic[SeqMapField.END_NODE.name] = curDic[SeqMapField.END_NODE.name]

        if curDic[SeqMapField.END_STATUS.name] != -1:
            startDic[SeqMapField.END_STATUS.name] = curDic[SeqMapField.END_STATUS.name]
            lsFinal.append(startDic)
            startDic = lsTest.pop(0)
    lsFinal.append(startDic)

    return lsFinal

#지렛대로부터 서빙지점까지 거리와 트레이 하중을 입력받아 밸런싱 암이 뻗어야 할 거리를 계산합니다.
def calculate_balance_distance(left_weight_kg, right_weight_kg, distance_right_mm):
    # left_weight_kg: 왼쪽 끝의 하중 (킬로그램 단위)
    # right_weight_kg: 오른쪽 끝의 하중 (킬로그램 단위)
    # distance_right_mm: 지렛대로부터 오른쪽 하중까지의 거리 (밀리미터 단위)
    
    # 무게 중심 공식: left_weight * distance_left = right_weight * distance_right
    # distance_left = (right_weight * distance_right) / left_weight    
    distance_left_mm = (right_weight_kg * distance_right_mm) / left_weight_kg    
    return distance_left_mm

#막대기의 길이, 왼쪽 끝의 하중, 오른쪽 끝의 하중을 입력받아 무게 중심을 계산하고, 그 위치를 반환합니다.
def calculate_lever_position_mm(rod_length_mm, left_weight_kg, right_weight_kg):
    # rod_length_mm: 막대기의 길이 (밀리미터 단위)
    # left_weight_kg: 왼쪽 끝의 하중 (킬로그램 단위)
    # right_weight_kg: 오른쪽 끝의 하중 (킬로그램 단위)
    # 무게 중심 계산
    x_c_mm = (left_weight_kg * 0 + right_weight_kg * rod_length_mm) / (left_weight_kg + right_weight_kg)
    return x_c_mm

def pulse_to_angle(pulse_count, total_pulses=10000, max_angle=540):
    if total_pulses == 0:
        return -1
    pulse_count = int(pulse_count)
    total_pulses = int(total_pulses)
    max_angle = int(max_angle)
    
    angle_per_pulse = max_angle / total_pulses  # 각 펄스당 회전 각도
    angle = (pulse_count * angle_per_pulse) % 360  # 360도로 조정
    return round(angle)


def angle_to_pulse(angle, total_pulses=10000, max_angle=540):
    angle = float(angle)
    total_pulses = int(total_pulses)
    max_angle = int(max_angle)
    
    angle_per_pulse = max_angle / total_pulses  # 각 펄스당 회전 각도
    pulse = angle / angle_per_pulse
    return round(pulse)



# 현재 포지션의 펄스와 목표 각도를 입력 받아 최단거리로 움직이는 펄스를 계산하는 함수
def calculate_minimum_pulse(
    current_pulse_str, target_angle, total_pulses_str=10000, max_angle=540
):
    current_pulse = int(current_pulse_str)
    total_pulses = int(total_pulses_str)
    # 현재 포지션의 각도를 계산
    current_angle = pulse_to_angle(current_pulse, total_pulses, max_angle)

    # 목표 각도와 현재 각도 간의 차이를 계산
    angle_difference = (target_angle - current_angle) % 360
    if angle_difference > 180:
        angle_difference -= 360

    # 각도 차이를 펄스로 변환
    pulse_difference = angle_to_pulse(angle_difference, total_pulses, max_angle)

    # 최종 목표 펄스를 계산
    target_pulse = (current_pulse + pulse_difference) % total_pulses
    return target_pulse

# # 예시
# current_pulse = 75
# target_angle = 90
# minimum_pulse = calculate_minimum_pulse(current_pulse, target_angle)
# print(f"Minimum pulse to move from current position to {target_angle} degrees: {minimum_pulse}")


#서보모터 정지상태에서 rpm 1000 까지 리니어하게 끌어올리는데 5초 걸렸어. 그 사이에 몇바퀴를 돌았을까?
def calculate_accdesc_pulse(accdec_time_seconds = 5,final_rpm = 1000):
    initial_rpm = 0    
    # 평균 RPM 계산
    average_rpm = (initial_rpm + final_rpm) / 2
    # 초당 회전수 계산
    revolutions_per_second = average_rpm / 60
    # 총 회전수 계산
    total_revolutions = revolutions_per_second * accdec_time_seconds
    #print(f'RPM1000 도달 회전수 : {total_revolutions}' )
    return total_revolutions

def calculate_rpm_time(round_togo_str, rpm_str):
    round_togo = abs(int(round_togo_str))
    rpm = limit_RPM_value(abs(int(rpm_str)))    
    if round_togo == 0:
        return 0
    if round_togo < 0 or rpm <= 0:
        return -1
    
    # 초당 회전수 계산
    rps = rpm / 60
    # 총 소요 시간 계산
    time_seconds = round_togo / rps
    if time_seconds < 0:
        logmsg = f"Invalid result : {time_seconds}/{round_togo_str}/{rpm_str} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
        rospy.loginfo(logmsg)
        return -1
    return time_seconds


#가감속을 반영하여 pulsecount 회전수를 rpm 속도로 돌을때 몇초 걸리는지 계산한다
def calculate_rpm_time_accdesc(pulsecount, rpm, acctime_ms, dectime_ms):
    #가속과 감속에 해당하는 회전수를 구한다.
    rpm = limit_RPM_value(rpm)
    accMaxRpm = 1000
    if rpm < 1000:
        accMaxRpm = rpm / 2
    accpulse = calculate_accdesc_pulse(acctime_ms/1000,accMaxRpm)
    decpulse = calculate_accdesc_pulse(dectime_ms/1000,accMaxRpm)
    #전체 회전수에서 가감속에 해당하는 회전수는 뺀다.
    pulsecountMain = pulsecount - accpulse - decpulse
    #가감속을 제외하고 회전에 걸리는 시간을 구한다
    rpmMainTimeSecond = calculate_rpm_time(pulsecountMain,rpm)
    #가감속시간까지 합산 후 결과값 리턴
    return rpmMainTimeSecond+(acctime_ms/1000)+(dectime_ms/1000)

# # 예제 호출
# n = 200  # 회전수
# rpm = 1500  # RPM
# time_taken = calculate_time(n, rpm)
# print(f"{n} 바퀴를 {rpm} RPM으로 돌릴 때 소요되는 시간: {time_taken:.2f} 초")

def calculate_targetRPM_fromtime(round_count, totaltime_sec):
    if totaltime_sec == 0:
        return -1
    # 초당 회전수 계산
    rps = round_count / totaltime_sec
    # 분당 회전수 계산
    rpm = rps * 60
    return abs(int(rpm))

def calculate_deceleration_factor(velocity):
    return abs(velocity*2)
    # # 최대 허용 가속도 (m/s²) - 물이 쏟아지지 않는 범위로 설정
    # max_allowable_acceleration = 4.9  # m/s²
    
    # # 필요한 감속 시간 (ms)
    # required_decel_time_ms = (velocity / max_allowable_acceleration) * 1000  # ms

    # # 기존 감속 시간과 비교하여 결과 출력
    # return abs(round(required_decel_time_ms))
  
# calculate_rotations 예시 사용법
# desired_angle = 60
# required_rotations = calculate_rotations(desired_angle)
# print(f"{desired_angle}도에 도달하기 위해 필요한 회전수는 {required_rotations:.2f}회입니다.")
def calculate_rotations(angle_degrees, pulse_midpoint, pulse_pot):
    """
    주어진 각도에 대해 필요한 회전수를 계산하는 함수.

    :param angle_degrees: 원하는 각도 (0도에서 90도 사이의 값)
    :pulse_midpoint: 45도를 지날때 pulse
    :pulse_pot: 끝까지 갈때의 pulse
    :return: 필요한 모터 회전수
    """
    
    if angle_degrees < 0 or angle_degrees > 90:
        raise ValueError("각도는 0도에서 90도 사이여야 합니다.")
    
    if angle_degrees <= 45:
        rotations = (pulse_midpoint / 45) * angle_degrees
    else:
        rotations = pulse_midpoint + (((pulse_pot-pulse_midpoint) / 45) * (angle_degrees - 45))
    
    return int(rotations)
  
def extract_pos_by_mbid(lsCmdArray):
    pos_dict = {}    
    try:
      for element in lsCmdArray:
          if isinstance(element, list):
              for cmd in element:
                  #print(f'Check : {element}')
                  mbid = cmd.get(MotorWMOVEParams.MBID.name)
                  pos = cmd.get(MotorWMOVEParams.POS.name)
                  if mbid and pos:
                      pos_dict[mbid] = pos
          elif isinstance(element, dict):
              mbid = element.get(MotorWMOVEParams.MBID.name)
              pos = element.get(MotorWMOVEParams.POS.name)
              if mbid and pos:
                  pos_dict[mbid] = pos
    except Exception as e:
        print(e,lsCmdArray)                
    return pos_dict

def create_mbid_dict(recvDataMap, fieldName):
    mbid_pos_dict = {}    
    def extract_mbid_pos(data):
        for item in data:
            if isinstance(item, list):
                extract_mbid_pos(item)
            elif isinstance(item, dict):
                mbid = item.get('MBID')
                pos = item.get(fieldName)
                if mbid and pos:
                    mbid_pos_dict[mbid] = pos
    
    extract_mbid_pos(recvDataMap)
    return mbid_pos_dict

def calculate_speed_adjustmentExp(cur_pos_arm1, cur_pos_arm2, not_cur_arm1, pot_cur_arm1, not_cur_arm2, pot_cur_arm2, spd_cur_arm1, spd_cur_arm2, k, time_seconds, pulses_per_round):
    """
    주어진 모터의 현재 위치, 속도, 그리고 시간 간격을 기준으로 새로운 RPM을 계산한다.
    
    Parameters:
    cur_pos_arm1 (float): 모터1의 현재 위치 (펄스)
    cur_pos_arm2 (float): 모터2의 현재 위치 (펄스)
    not_cur_arm1 (float): 모터1의 최소 위치 (펄스)
    pot_cur_arm1 (float): 모터1의 최대 위치 (펄스)
    not_cur_arm2 (float): 모터2의 최소 위치 (펄스)
    pot_cur_arm2 (float): 모터2의 최대 위치 (펄스)
    spd_cur_arm1 (float): 모터1의 현재 RPM
    spd_cur_arm2 (float): 모터2의 현재 RPM
    k (float): 지수 매핑 함수의 계수
    time_seconds (float): 예상 이동 시간 (초)
    
    Returns:
    float: 새로 계산된 모터1의 RPM
    """
    def prtMsg(strInfo, bIsPrint=True):
      if (bIsPrint):
        print(strInfo)
        
    spd_cur_arm1 = limit_RPM_value(spd_cur_arm1)
    spd_cur_arm2 = limit_RPM_value(spd_cur_arm2)
    # rpm_OK 의 부호를 기준으로 rpm_Wrong의 부호를 조정
    if (spd_cur_arm1 > 0 and spd_cur_arm2 < 0) or (spd_cur_arm1 < 0 and spd_cur_arm2 > 0):
        spd_cur_arm2 = -spd_cur_arm2
    prtMsg(f'모터1위치:{cur_pos_arm1},모터2위치:{cur_pos_arm2},RPM1:{spd_cur_arm1},RPM2:{spd_cur_arm2}')

    #print(f'not_cur_arm1:{not_cur_arm1},pot_cur_arm1:{pot_cur_arm1},not_cur_arm2:{not_cur_arm2},pot_cur_arm2:{pot_cur_arm2}')
    # 펄스 계산 비율
    pulse_rate = pulses_per_round / 60  # 1분에 10000 펄스

    # 모터2의 예상 위치 계산
    expected_pos_arm2 = cur_pos_arm2 + (spd_cur_arm2 * pulse_rate * time_seconds)
    prtMsg(f'모터2의 {time_seconds}초 후 예상 위치 계산: {expected_pos_arm2 : .1f}')

    # 모터1의 목표 위치 계산 (모터2의 예상 위치 기반)
    target_tmp_arm1 = mapRange(expected_pos_arm2, not_cur_arm2, pot_cur_arm2, not_cur_arm1, pot_cur_arm1)
    #target_tmp_arm1 = mapRangeExp(expected_pos_arm2, not_cur_arm2, pot_cur_arm2, not_cur_arm1, pot_cur_arm1, k)
    prtMsg(f'모터1의 목표 위치 계산 (모터2의 예상 위치 기반): {target_tmp_arm1 : .1f}')

    # 모터1의 예상 위치
    expected_pos_arm1 = cur_pos_arm1 + spd_cur_arm1 * pulse_rate * time_seconds
    prtMsg(f'모터1의 {time_seconds}초 후 예상 위치: {expected_pos_arm1 : .1f}')
    
    if spd_cur_arm1 > 10 and target_tmp_arm1 < cur_pos_arm1:
      prtMsg(f'모터1의 현재 위치값 {cur_pos_arm1}이 목표치값 {target_tmp_arm1: .1f}을 초과하였음. 속도 감소 spd1:{spd_cur_arm1},spd2:{spd_cur_arm2}')
      return 1
    if spd_cur_arm1 < -10 and target_tmp_arm1 > cur_pos_arm1:
      prtMsg(f'모터1의 현재 위치값 {cur_pos_arm1}이 목표치값 {target_tmp_arm1: .1f}을 초과하였음. 속도 감소! spd1:{spd_cur_arm1},spd2:{spd_cur_arm2}')
      return 1

    # 필요한 총 펄스 계산
    needed_pulses = target_tmp_arm1 - expected_pos_arm1
    prtMsg(f'필요한 총 펄스 계산: {needed_pulses : .1f}')

    # 추가적으로 필요한 RPM 계산
    additional_rpm = needed_pulses / (pulse_rate * time_seconds)
    prtMsg(f'추가적으로 필요한 RPM 계산: {additional_rpm : .1f}')

    # 새로운 RPM 계산
    new_rpmTmp = spd_cur_arm1 + additional_rpm
    if abs(new_rpmTmp) > ALARM_RPM_LIMIT:
        sErrMsg = '1/2관절 밸런싱 실패'
        raise Exception(sErrMsg)  # 예외를 발생시킴
    
    new_rpm = min(abs(round(new_rpmTmp)),DEFAULT_SPD_LIMIT)
    prtMsg(f'새로운 RPM 계산: {new_rpmTmp : .1f}')
    prtMsg(f"속도변경 : {spd_cur_arm1} -> {new_rpm}")
    return new_rpm

def compare_dicts(dict1, dict2, threshold):
    """
    두 딕셔너리의 X, Y 값의 차이가 threshold 이상인지 비교하는 함수

    :param dict1: 첫 번째 딕셔너리
    :param dict2: 두 번째 딕셔너리
    :param threshold: 차이의 기준값
    :return: 차이가 threshold 이상이면 True, 아니면 False
    """
    x_diff_signed = (dict1[ARUCO_RESULT_FIELD.X.name] - dict2[ARUCO_RESULT_FIELD.X.name])
    y_diff_signed = (dict1[ARUCO_RESULT_FIELD.Y.name] - dict2[ARUCO_RESULT_FIELD.Y.name])
    x_diff = abs(x_diff_signed)
    y_diff = abs(y_diff_signed)
    
    return not(x_diff > threshold or y_diff > threshold), x_diff_signed, y_diff_signed


def filter_outliers(data, m=2):
    """
    주어진 데이터에서 x, y, z 값이 튀는(이상치) 원소들을 필터링합니다.
    :param data: 원본 데이터 (리스트 형식)
    :param m: 이상치를 식별하기 위한 표준편차의 배수
    :return: 이상치가 제거된 데이터
    """
    # x, y, z 값을 추출합니다.
    x_values = np.array([d[ARUCO_RESULT_FIELD.X.name] for d in data])
    y_values = np.array([d[ARUCO_RESULT_FIELD.Y.name] for d in data])
    z_values = np.array([d[ARUCO_RESULT_FIELD.Z.name] for d in data])

    # # 평균과 표준편차를 계산합니다.
    # mean_x, std_x = np.mean(x_values), np.std(x_values)
    # mean_y, std_y = np.mean(y_values), np.std(y_values)
    # mean_z, std_z = np.mean(z_values), np.std(z_values)
      
    # # x, y, z 값이 평균에서 m 표준편차 이상 벗어난 원소를 필터링합니다.
    # filtered_data = [d for d in data 
    #                  if abs(d[ARUCO_RESULT_FIELD.X.name] - mean_x) < m * std_x 
    #                  and abs(d[ARUCO_RESULT_FIELD.Y.name] - mean_y) < m * std_y 
    #                  and abs(d[ARUCO_RESULT_FIELD.Z.name] - mean_z) < m * std_z]
    
    # 중앙값을 계산합니다.
    median_x, median_y, median_z = np.median(x_values), np.median(y_values), np.median(z_values)
    
    # 중위 절대 편차를 계산합니다.
    mad_x = np.median(np.abs(x_values - median_x))
    mad_y = np.median(np.abs(y_values - median_y))
    mad_z = np.median(np.abs(z_values - median_z))
    
    # mad_x, mad_y, mad_z가 0일 때 예외 처리
    if mad_x == 0:
        mad_x = ZERO_ALMOST # 작은 값으로 설정하여 0으로 나누는 것을 방지
    if mad_y == 0:
        mad_y = ZERO_ALMOST
    if mad_z == 0:
        mad_z = ZERO_ALMOST
        
    # 이상치를 필터링합니다.
    filtered_data = [d for d in data 
                     if abs(d[ARUCO_RESULT_FIELD.X.name] - median_x) / mad_x < m 
                     and abs(d[ARUCO_RESULT_FIELD.Y.name] - median_y) / mad_y < m 
                     and abs(d[ARUCO_RESULT_FIELD.Z.name] - median_z) / mad_z < m]  
    return filtered_data

# # 예제 데이터
# data = [
#     {"DIFF_X": 5.73, "DIFF_Y": 27.94, "ANGLE": 11.87, "CAM_ID": 2, "MARKER_VALUE": 1, "X": 105.6, "Y": 289.0, "Z": 3916.3},
#     {"DIFF_X": 5.73, "DIFF_Y": 27.94, "ANGLE": 11.87, "CAM_ID": 2, "MARKER_VALUE": 1, "X": 105.0, "Y": 289.9, "Z": 3916.1},
#     # 추가적인 데이터 원소들...
# ]

# # 함수 호출
# filtered_data = filter_outliers(data)
# print(filtered_data)
def pointcloud2_to_pcl(pointcloud2_msg):
    # Convert PointCloud2 message to pcl.PointCloud
    points_list = []
    for point in pc2.read_points(pointcloud2_msg, skip_nans=True):
        points_list.append([point[0], point[1], point[2]])
    
    pcl_data = pcl.PointCloud()
    pcl_data.from_list(points_list)
    
    return pcl_data

def pcl_to_pointcloud2(pcl_cloud):
    # Convert pcl.PointCloud to PointCloud2 message
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'base_link'
    
    points_list = []
    for point in pcl_cloud:
        points_list.append([point[0], point[1], point[2], 0])
        
def is_area_clear(cloud, target_position, target_dimensions,leafLen = 0.01):
    # Extract points within the target area
    x_min = target_position[0] - target_dimensions[0] / 2
    x_max = target_position[0] + target_dimensions[0] / 2
    y_min = target_position[1] - target_dimensions[1] / 2
    y_max = target_position[1] + target_dimensions[1] / 2
    z_min = target_position[2] - target_dimensions[2] / 2
    z_max = target_position[2] + target_dimensions[2] / 2

    # Create a PassThrough filter to filter points within the target area
    passthrough = cloud.make_passthrough_filter()
    passthrough.set_filter_field_name("x")
    passthrough.set_filter_limits(x_min, x_max)
    cloud_filtered = passthrough.filter()

    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name("y")
    passthrough.set_filter_limits(y_min, y_max)
    cloud_filtered = passthrough.filter()

    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name("z")
    passthrough.set_filter_limits(z_min, z_max)
    cloud_filtered = passthrough.filter()

    # Remove noise using a StatisticalOutlierRemoval filter
    # sor = cloud_filtered.make_statistical_outlier_filter()
    # sor.set_mean_k(50)
    # sor.set_std_dev_mul_thresh(1.0)
    # cloud_filtered = sor.filter()
    
    # Apply a density-based filter to remove sparse points (additional noise reduction)
    fil = cloud_filtered.make_voxel_grid_filter()

    fil.set_leaf_size(leafLen, leafLen, leafLen)
    cloud_filtered = fil.filter()

    # Check if the filtered cloud is empty
    if cloud_filtered.size == 0:
        return True

    # If not empty, return the filtered cloud for obstacle publishing
    global filtered_obstacle_cloud
    filtered_obstacle_cloud = cloud_filtered
    return False

def rotate_point(point, angle, radius):
    """
    Rotate a point around the z-axis by the given angle, considering the radius offset.
    
    :param point: The point to rotate (x, y, z).
    :param angle: The angle in radians.
    :param radius: The radius from the rotation center to the lidar.
    :return: Rotated point (x, y, z).
    """
    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)
    x, y, z = point
    
    # Translate point to account for the radius
    x -= radius * cos_theta
    y -= radius * sin_theta

    # Rotate the point around the z-axis
    x_new = x * cos_theta - y * sin_theta
    y_new = x * sin_theta + y * cos_theta

    # Translate point back
    x_new += radius * cos_theta
    y_new += radius * sin_theta

    return x_new, y_new, z

# def merge_point_clouds(angle_pc2_dict, radius):
#     """
#     Merge multiple PointCloud2 messages into a single PointCloud2 message considering the radius offset.
    
#     :param angle_pc2_dict: A dictionary with angles as keys and PointCloud2 messages as values.
#     :param radius: The radius from the rotation center to the lidar.
#     :return: A single merged PointCloud2 message.
#     """
#     all_points = []
#     header = None

#     for angle, pc2_msg in angle_pc2_dict.items():
#         # Convert angle from degrees to radians
#         theta = np.radians(angle)
        
#         # Extract points from the PointCloud2 message
#         points = pc2.read_points(pc2_msg, field_names=("x", "y", "z"), skip_nans=True)
#         rotated_points = [rotate_point(point, theta, radius) for point in points]
        
#         all_points.extend(rotated_points)
        
#         # Save the header from one of the messages (assuming all headers are similar)
#         if header is None:
#             header = pc2_msg.header

#     # Create a new PointCloud2 message from the combined points
#     merged_pc2 = pc2.create_cloud_xyz32(header, all_points)
#     return merged_pc2

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from tf.transformations import quaternion_from_euler

def merge_point_clouds(angle_pc2_dict, radius):
    merged_points = []
    header = None
    for angle, cloud in angle_pc2_dict.items():
        # 각도를 라디안으로 변환
        angle_rad = np.radians(angle)
        header = cloud.header
        
        # 회전 행렬 생성
        rotation_matrix = np.array([
            [np.cos(angle_rad), -np.sin(angle_rad), 0],
            [np.sin(angle_rad), np.cos(angle_rad), 0],
            [0, 0, 1]
        ])

        # 라이다의 위치 계산 (회전 반경 고려)
        lidar_position = np.array([radius * np.cos(angle_rad), radius * np.sin(angle_rad), 0])

        # PointCloud2 메시지에서 포인트 추출
        for point in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            # 포인트를 numpy 배열로 변환
            point_array = np.array(point)
            
            # 라이다 위치를 고려하여 포인트 조정
            adjusted_point = point_array + lidar_position
            
            # 포인트를 회전
            rotated_point = np.dot(rotation_matrix, adjusted_point)
            
            merged_points.append(rotated_point)

    # 새로운 PointCloud2 메시지 생성
    merged_cloud = PointCloud2()
    merged_cloud.header = header
    merged_cloud.header.stamp = rospy.Time.now()  # 타임스탬프 업데이트

    # PointCloud2 필드 설정
    merged_cloud.fields = [
        pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
        pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
        pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
    ]
    merged_cloud.point_step = 12
    merged_cloud.row_step = merged_cloud.point_step * len(merged_points)
    merged_cloud.height = 1
    merged_cloud.width = len(merged_points)
    merged_cloud.is_dense = False

    # 포인트 데이터를 바이트 배열로 변환
    merged_cloud.data = np.asarray(merged_points, np.float32).tostring()

    return merged_cloud

import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def remove_points_near_origin(cloud, distance_threshold=0.1):
    """
    PointCloud2 데이터에서 원점으로부터 지정된 거리 이내의 포인트를 제거합니다.

    :param cloud: 입력 PointCloud2 메시지
    :param distance_threshold: 제거할 포인트의 원점으로부터의 최대 거리 (기본값: 0.1m)
    :return: 필터링된 PointCloud2 메시지
    """
    # PointCloud2 메시지에서 포인트 데이터 추출
    points = np.array(list(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)))

    # 각 포인트의 원점으로부터의 거리 계산
    distances = np.sqrt(np.sum(points**2, axis=1))

    # 거리가 임계값보다 큰 포인트만 선택
    mask = distances > distance_threshold
    filtered_points = points[mask]

    # 필터링된 포인트로 새로운 PointCloud2 메시지 생성
    filtered_cloud = pc2.create_cloud_xyz32(cloud.header, filtered_points)

    return filtered_cloud
  
class DataFrameManager:
    def __init__(self, file_path):
        if isinstance(file_path, pd.DataFrame):
            df = file_path.astype({col: int for col in df.select_dtypes('int64').columns})
            self.df = df
        else:
            dfTmp = pd.read_csv(file_path, sep='\t')
            self.df = dfTmp.astype({col: int for col in dfTmp.select_dtypes('int64').columns})  # Reading the file as a DataFrame
        
    def getdf(self):
      return self.df
    
    # 1. Method to filter rows based on a specific key value
    def filter_by_key(self, key_column, key_value):
        if key_column == None or key_value == None:
          return self.getdf(self)
        
        filtered_df = self.df[self.df[key_column].astype(str) == str(key_value)]
        return filtered_df

    # 2. Method to convert DataFrame (or filtered DataFrame) to a list of dictionaries
    def to_dict_list(self, df=None, orientMethod='records'):
        if df is None:
            df = self.df
        return df.to_dict(orient=orientMethod)

    # 3. Method to remove rows based on a specific key value
    def remove_by_key(self, key_column, key_value):
        self.df = self.df[self.df[key_column] != key_value]

    # 4. Method to update specific key-value pairs in rows that match a certain key
    def update_by_key(self, key_column, key_value, update_column, new_value):
        self.df.loc[self.df[key_column] == key_value, update_column] = new_value

    # 5. Method to transform DataFrame as per the given structure
    def transform_to_custom_dict_list(self):
        dict_list = []
        for _, row in self.df.iterrows():
            tableno = try_parse_int(row[TableInfo.TABLE_ID.name], MIN_INT)
            end = row[TableInfo.NODE_ID.name]
            nodeID = try_parse_int(end, MIN_INT)
            if nodeID == NODE_SPECIAL_VALUE.CHARGING_STATION.value:
              tableno = "P1"
            elif nodeID == NODE_SPECIAL_VALUE.KITCHEN_STATION.value:
              tableno = "H1"            
            # if tableno != MIN_INT:
            #   tableno = f"T{row[TableInfo.TABLE_ID.name]}"
            else:
              tableno = f"T{row[TableInfo.TABLE_ID.name]}"
            
            iSERVING_ANGLE = int(row[TableInfo.SERVING_ANGLE.name])
            distance = row[TableInfo.SERVING_DISTANCE.name]
            marker_angle = row[TableInfo.MARKER_ANGLE.name]
            time = round(10 + (iSERVING_ANGLE/ 5) + (distance / 100))
            
            direction = GetDirectionFromAngle(iSERVING_ANGLE)
                
            x,y = calculate_coordinates(distance,iSERVING_ANGLE)
            #print(f'변환된 좌표 : X:{x},Y:{y}')
            dict_list.append({
                APIBLB_FIELDS_INFO.tableno.name: str(tableno),
                APIBLB_FIELDS_INFO.end.name: str(end),
                APIBLB_FIELDS_INFO.distance.name: str(distance),
                APIBLB_FIELDS_INFO.time.name: str(time),
                APIBLB_FIELDS_INFO.direction.name: direction,
                APIBLB_FIELDS_INFO.angle.name: (iSERVING_ANGLE),
                APIBLB_FIELDS_INFO.tb_xval.name : x,
                APIBLB_FIELDS_INFO.tb_yval.name : y,
                APIBLB_FIELDS_INFO.angle_marker.name : marker_angle,
            })
        
        return dict_list
    
    # New method to sort DataFrame by up to two keys and return the sorted DataFrame
    def sort_by_keys(self, df, key1, key2=None, ascending1=True, ascending2=True):
        """
        Sorts the DataFrame by up to two keys and returns the sorted DataFrame.
        
        Parameters:
        - key1: First column to sort by.
        - key2: Second column to sort by (optional).
        - ascending1: Sort order for the first key (True for ascending, False for descending).
        - ascending2: Sort order for the second key (True for ascending, False for descending).
        
        Returns:
        - A sorted DataFrame.
        """
        if df is None:
            df = self.df
            
        if key2:
            return df.sort_values(by=[key1, key2], ascending=[ascending1, ascending2])
        else:
            return df.sort_values(by=[key1], ascending=[ascending1])

# 인접한 중복 항목만 제거하는 함수 
def remove_adjacent_duplicates(data):
    result = []
    previous_mbid = None  # 이전 항목의 MBID를 저장할 변수

    for entry in data:
        # 각 원소가 리스트일 경우 처리
        if isinstance(entry, list):
            filtered_entry = []
            for item in entry:
                current_mbid = item[MotorWMOVEParams.MBID.name]
                if current_mbid != previous_mbid:
                    filtered_entry.append(item)
                previous_mbid = current_mbid
            if filtered_entry:
                result.append(filtered_entry)
        else:
            # 리스트가 아닌 경우 처리
            current_mbid = entry[MotorWMOVEParams.MBID.name]
            if current_mbid != previous_mbid:
                result.append(entry)
            previous_mbid = current_mbid

    return result
  
def PrintDF(dfReceived):
  if dfReceived is None or dfReceived.empty:
    return
  columns_to_keep = [field.name for field in APIBLB_FIELDS_TASK if field.name in dfReceived.columns]
  log_all_frames(dfReceived[columns_to_keep].to_string())  

    # PrintDF(temp_recv)
    # print(temp_getseq)
    # # 컬럼 타입을 정수로 변환

# def merge_dataframes(temp_recv, temp_getseq):
#     temp_recv = temp_recv.copy()
#     temp_recv[APIBLB_FIELDS_INFO.distance.name] = temp_recv[APIBLB_FIELDS_INFO.distance.name].astype(int)
#     temp_recv[APIBLB_FIELDS_TASK.startnode.name] = temp_recv[APIBLB_FIELDS_TASK.startnode.name].astype(int)
#     temp_recv[APIBLB_FIELDS_TASK.endnode.name] = temp_recv[APIBLB_FIELDS_TASK.endnode.name].astype(int)

#     temp_getseq = temp_getseq.copy()
#     temp_getseq[APIBLB_FIELDS_INFO.distance.name] = temp_getseq[APIBLB_FIELDS_INFO.distance.name].astype(int)
#     temp_getseq[SeqMapField.START_NODE.name] = temp_getseq[SeqMapField.START_NODE.name].astype(int)
#     temp_getseq[SeqMapField.END_NODE.name] = temp_getseq[SeqMapField.END_NODE.name].astype(int)

#     # 결과 DataFrame 초기화
#     result = temp_getseq.copy()
#     result[APIBLB_FIELDS_TASK.detailcode_list.name] = ''
#     result[APIBLB_FIELDS_TASK.distance_list.name] = ''
#     result[APIBLB_FIELDS_TASK.endnode_list.name] = ''
#     result[APIBLB_FIELDS_TASK.taskid.name] = ''
#     result[APIBLB_FIELDS_TASK.tasktype.name] = ''
#     result[APIBLB_FIELDS_TASK.ordertype.name] = ''

#     # temp_getseq의 각 행에 대해 처리
#     for idx, row in result.iterrows():
#         start_node = row[SeqMapField.START_NODE.name]
#         end_node = row[SeqMapField.END_NODE.name]

#         # temp_recv에서 startnode와 endnode가 순서에 관계없이 범위에 있는 행 필터링
#         lower_bound = min(start_node, end_node)
#         upper_bound = max(start_node, end_node)
#         matching_recv = temp_recv.loc[(temp_recv[APIBLB_FIELDS_TASK.startnode.name] >= lower_bound) & (temp_recv[APIBLB_FIELDS_TASK.endnode.name] <= upper_bound)]
        
#         # detailcodes와 endnodes 수집
#         distance_list = ','.join(map(str, matching_recv[APIBLB_FIELDS_INFO.distance.name].tolist()))
#         detailcode_list = ','.join(map(str, matching_recv[APIBLB_FIELDS_TASK.detailcode.name].tolist()))
#         endnode_list = ','.join(map(str, matching_recv[APIBLB_FIELDS_TASK.endnode.name].tolist()))

#         # taskid, tasktype, ordertype 수집 (모든 행에서 동일한 값이라고 가정)
#         taskid = matching_recv[APIBLB_FIELDS_TASK.taskid.name].iloc[0] if not matching_recv.empty else ''
#         tasktype = matching_recv[APIBLB_FIELDS_TASK.tasktype.name].iloc[0] if not matching_recv.empty else ''
#         ordertype = matching_recv[APIBLB_FIELDS_TASK.ordertype.name].iloc[0] if not matching_recv.empty else ''

#         # 결과 DataFrame 갱신
#         result.loc[idx, APIBLB_FIELDS_TASK.distance_list.name] = distance_list
#         result.loc[idx, APIBLB_FIELDS_TASK.detailcode_list.name] = detailcode_list
#         result.loc[idx, APIBLB_FIELDS_TASK.endnode_list.name] = endnode_list
#         result.loc[idx, APIBLB_FIELDS_TASK.taskid.name] = taskid
#         result.loc[idx, APIBLB_FIELDS_TASK.tasktype.name] = tasktype
#         result.loc[idx, APIBLB_FIELDS_TASK.ordertype.name] = ordertype

#     return result

def merge_dataframes(temp_recv, temp_getseq):
    temp_recv = temp_recv.copy()
    temp_recv[APIBLB_FIELDS_INFO.distance.name] = temp_recv[APIBLB_FIELDS_INFO.distance.name].astype(int)
    temp_recv[APIBLB_FIELDS_TASK.startnode.name] = temp_recv[APIBLB_FIELDS_TASK.startnode.name].astype(int)
    temp_recv[APIBLB_FIELDS_TASK.endnode.name] = temp_recv[APIBLB_FIELDS_TASK.endnode.name].astype(int)
    PrintDF(temp_recv)
    
    temp_getseq = temp_getseq.copy()
    temp_getseq[SeqMapField.DISTANCE.name] = temp_getseq[SeqMapField.DISTANCE.name].astype(int)
    temp_getseq[SeqMapField.START_NODE.name] = temp_getseq[SeqMapField.START_NODE.name].astype(int)
    temp_getseq[SeqMapField.END_NODE.name] = temp_getseq[SeqMapField.END_NODE.name].astype(int)
    print(temp_getseq)
    
    # 결과 DataFrame 초기화
    result = temp_getseq.copy()
    result[APIBLB_FIELDS_TASK.detailcode_list.name] = ''
    result[APIBLB_FIELDS_TASK.distance_list.name] = ''
    result[APIBLB_FIELDS_TASK.endnode_list.name] = ''
    result[APIBLB_FIELDS_TASK.taskid.name] = ''
    result[APIBLB_FIELDS_TASK.tasktype.name] = ''
    result[APIBLB_FIELDS_TASK.ordertype.name] = ''
    
    # Build a dictionary to track paths
    node_connections = {}
    for _, row in temp_recv.iterrows():
        start = row[APIBLB_FIELDS_TASK.startnode.name]
        end = row[APIBLB_FIELDS_TASK.endnode.name]
        if start not in node_connections:
            node_connections[start] = []
        node_connections[start].append(row)
    
    # Process each path in temp_getseq
    for idx, row in result.iterrows():
        start_node = row[SeqMapField.START_NODE.name]
        end_node = row[SeqMapField.END_NODE.name]
        DIRECTION = row[SeqMapField.DIRECTION.name]
        # Find path from start_node to end_node
        path_segments = []
        visited = set()
        
        def find_path(current_node, target_node, current_path):
            if current_node == target_node:
                path_segments.extend(current_path)
                return True
                
            if current_node in visited:
                return False
                
            visited.add(current_node)
            
            if current_node not in node_connections:
                return False
                
            for next_segment in node_connections[current_node]:
                next_node = next_segment[APIBLB_FIELDS_TASK.endnode.name]
                if find_path(next_node, target_node, current_path + [next_segment]):
                    return True
            
            return False
        
        # Try to find a path
        find_path(start_node, end_node, [])
        
        if path_segments:
            # Extract information from path segments
            detailcodes = [str(segment[APIBLB_FIELDS_TASK.detailcode.name]) for segment in path_segments]
            distances = [str(segment[APIBLB_FIELDS_INFO.distance.name]) for segment in path_segments]
            endnodes = [str(segment[APIBLB_FIELDS_TASK.endnode.name]) for segment in path_segments]
            direction = [str(segment[APIBLB_FIELDS_INFO.direction.name]) for segment in path_segments]
            
            # Get common attributes (assuming they're the same for all segments)
            taskid = path_segments[0][APIBLB_FIELDS_TASK.taskid.name]
            tasktype = path_segments[0][APIBLB_FIELDS_TASK.tasktype.name]
            ordertype = path_segments[0].get(APIBLB_FIELDS_TASK.orderstatus.name, '')  # Assuming orderstatus is ordertype
            direction = path_segments[0].get(APIBLB_FIELDS_INFO.direction.name, '')
            
            # Update resultDIRECTION
            #result.loc[idx, APIBLB_FIELDS_INFO.direction.name] = ','.join(direction)
            result.loc[idx, APIBLB_FIELDS_INFO.direction.name] = DIRECTION
            result.loc[idx, APIBLB_FIELDS_TASK.detailcode_list.name] = ','.join(detailcodes)
            result.loc[idx, APIBLB_FIELDS_TASK.distance_list.name] = ','.join(distances)
            result.loc[idx, APIBLB_FIELDS_TASK.endnode_list.name] = ','.join(endnodes)
            result.loc[idx, APIBLB_FIELDS_TASK.taskid.name] = taskid
            result.loc[idx, APIBLB_FIELDS_TASK.tasktype.name] = tasktype
            result.loc[idx, APIBLB_FIELDS_TASK.ordertype.name] = ordertype
    print(result)
    return result
  
# Function to process the dataframe according to the request
def process_robot_path(df):
    # Step 1: Sort by detailcode
    df = df.sort_values(APIBLB_FIELDS_TASK.detailcode.name)
    
    # Step 2: Group by direction until it changes and consolidate the groups
    consolidated_rows = []
    current_direction = None
    current_group = []
    
    for i, row in df.iterrows():
        if row[APIBLB_FIELDS_TASK.direction.name] != current_direction:
            if current_group:
                # Consolidate the current group
                consolidated_rows.append(consolidate_group(current_group))
            current_direction = row[APIBLB_FIELDS_TASK.direction.name]
            current_group = [row]
        else:
            current_group.append(row)
    
    # Consolidate the last group
    if current_group:
        consolidated_rows.append(consolidate_group(current_group))
    
    # Step 3: Ignore rows where the endnode is not a number
    result_df = pd.DataFrame(consolidated_rows)
    result_df = result_df[result_df[APIBLB_FIELDS_TASK.endnode.name].apply(lambda x: str(x).isdigit())]
    
    return result_df

def consolidate_group(group):
    startnode = group[0][APIBLB_FIELDS_TASK.startnode.name]
    endnode = group[-1][APIBLB_FIELDS_TASK.endnode.name]
    mastercode = group[-1][APIBLB_FIELDS_TASK.mastercode.name]
    endnode_list = ",".join(map(str, [row[APIBLB_FIELDS_TASK.endnode.name] for row in group]))
    detailcode_list = ",".join(map(str, [row[APIBLB_FIELDS_TASK.detailcode.name] for row in group]))
    distance_list = ",".join(map(str, [row[APIBLB_FIELDS_TASK.distance.name] for row in group]))
    distance_total = sum([try_parse_float(row[APIBLB_FIELDS_TASK.distance.name]) for row in group])
    
    return {
        APIBLB_FIELDS_TASK.startnode.name: startnode,
        APIBLB_FIELDS_TASK.endnode.name: endnode,
        APIBLB_FIELDS_TASK.mastercode.name:mastercode,
        APIBLB_FIELDS_TASK.direction.name: group[0][APIBLB_FIELDS_TASK.direction.name],
        APIBLB_FIELDS_TASK.detailcode_list.name: detailcode_list,
        APIBLB_FIELDS_TASK.endnode_list.name: endnode_list,
        APIBLB_FIELDS_TASK.distance_list.name: distance_list,
        APIBLB_FIELDS_TASK.distance_total.name: round(distance_total)
    }
    
#@log_arguments    
def calculate_checkpoints(data, cur_pos):
    try:
      # Split the distance_list by commas
      distance_list = list(map(int, str(data[APIBLB_FIELDS_TASK.distance_list.name]).split(sDivItemComma)))
      #rospy.loginfo(data)
      # Determine whether to add or subtract based on the DISTANCE sign
      #sDir = data.get(APIBLB_FIELDS_TASK.direction.name)
      sDir = data.get(SeqMapField.DIRECTION.name)
      if sDir == 'N' or sDir == 'E':
        sign = 1# if data[SeqMapField.DISTANCE.name] < 0 else -1
      else:
        sign = -1
      
      # Initialize the checkpoint array with the current position
      checkpoints = []
      
      # Calculate each checkpoint
      for distance in distance_list:
          cur_pos += sign * distance_to_pulseH(distance)
          checkpoints.append(cur_pos)
      
      return checkpoints
    except Exception as e:
        log_all_frames(f'param : {data},{cur_pos}-{traceback.format_exc()}')
  
def calculate_safe_deceleration_distance(initial_speed_mm_s, safety_factor=2.0):
    """
    Calculate the safe deceleration distance for an AGV.
    
    :param initial_speed_mm_s: Initial speed of the AGV in mm/s
    :param safety_factor: A multiplier to add extra safety margin (default 2.0)
    :return: Safe deceleration distance in meters
    """
    # Convert initial speed from mm/s to m/s
    initial_speed_m_s = initial_speed_mm_s / 1000

    # Set deceleration to 1/10 of gravity (adjust as needed)
    deceleration = 9.8 / 10  # m/s^2

    # Calculate the theoretical stopping distance
    stopping_distance = (initial_speed_m_s ** 2) / (2 * deceleration)

    # Apply safety factor
    safe_distance = stopping_distance * safety_factor

    return round(safe_distance, 3)

# import sensor_msgs.point_cloud2 as pc2
# import numpy as np
# from sensor_msgs.msg import PointCloud2

def compute_xyz_means(point_cloud_msg):
    # point_cloud_msg: PointCloud2 메시지
    points = pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
    
    # 리스트로 변환
    points_list = list(points)
    
    # x, y, z 좌표를 각각 분리
    if len(points_list) == 0:
        return None, None, None

    x_vals = [p[0] for p in points_list]
    y_vals = [p[1] for p in points_list]
    z_vals = [p[2] for p in points_list]

    # numpy를 사용해 각각의 평균값 계산
    x_mean = np.mean(x_vals)
    y_mean = np.mean(y_vals)
    z_mean = np.mean(z_vals)

    return x_mean, y_mean, z_mean

# 예시: PointCloud2 메시지에서 평균값을 계산
# point_cloud_msg가 PointCloud2 메시지라고 가정
# x_mean, y_mean, z_mean = compute_xyz_means(point_cloud_msg)
def update_node_config(file_path, node_id, old_config, new_config):
    # 파일 읽기
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    # 파일 내용 업데이트
    updated_lines = []
    for line in lines:
        if line.startswith('#'):
          continue
        parts = line.strip().split(sDivTab)
        # 첫 번째 항목이 node_id와 일치하는지 확인
        if parts and int(parts[0]) == node_id:
            # 설정값에서 old_config을 찾아 new_config으로 변경
            updated_line = sDivTab.join(
                [str(node_id)] + [str(new_config) if int(val) == old_config else val for val in parts[1:]]
            ) + "\n"
        else:
            updated_line = line
        updated_lines.append(updated_line)
    
    if len(updated_lines) > 0:
        # 파일에 업데이트된 내용 쓰기
        with open(file_path, 'w') as file:
            file.writelines(updated_lines)
     
def find_dead_end_nodes_from_file(file_path):
    # 각 노드의 연결 횟수를 저장할 딕셔너리 초기화
    connections = {}

    # 파일 읽기
    with open(file_path, 'r') as file:
        for line in file:
            # 각 줄을 공백 기준으로 나누어 노드1, 노드2, 거리로 분리
            parts = line.strip().split(sDivTab)
            if len(parts) != 3:
                continue  # 각 줄은 노드1, 노드2, 거리 형태여야 함
            
            node1, node2, distance = int(parts[0]), int(parts[1]), int(parts[2])
            
            # 각 노드의 연결 수 계산
            if node1 not in connections:
                connections[node1] = 0
            if node2 not in connections:
                connections[node2] = 0
            connections[node1] += 1
            connections[node2] += 1

    # 연결이 하나뿐인 노드들을 막다른 노드로 간주하여 리스트에 추가
    dead_end_nodes = [node for node, count in connections.items() if count == 1]

    return dead_end_nodes

def groupFromList(data,key):
    grouped_data = {}

    for item in data:
        marker_value = item[key]
        if marker_value not in grouped_data:
            grouped_data[str(marker_value)] = []
        grouped_data[str(marker_value)].append(item)

    return grouped_data
  
def GetLinkKey(start_node, end_node):
  linkKey = f'{start_node}{end_node}'
  if end_node < start_node:
    linkKey = f'{end_node}{start_node}'
  return linkKey
  
class Graph:
    def __init__(self, graphInit=None):
        if graphInit is None:
            self.graph = {}  # 그래프 데이터를 저장할 딕셔너리
        else:
            self.graph = graphInit

    def add_edge(self, node1, node2, distance):
        """노드 간의 간선을 추가"""
        if node1 not in self.graph:
            self.graph[node1] = []
        if node2 not in self.graph:
            self.graph[node2] = []
        
        # 간선 추가 (중복 방지)
        if [node2, distance] not in self.graph[node1]:
            self.graph[node1].append([node2, distance])
        if [node1, distance] not in self.graph[node2]:
            self.graph[node2].append([node1, distance])

    def remove_edge(self, node1, node2):
        """노드 간의 간선을 제거"""
        if node1 in self.graph:
            self.graph[node1] = [edge for edge in self.graph[node1] if edge[0] != node2]
        if node2 in self.graph:
            self.graph[node2] = [edge for edge in self.graph[node2] if edge[0] != node1]

    def find_path(self, start, end):
        """
        두 노드 사이의 경로와 총 거리를 찾음.
        """
        visited = set()
        stack = [(start, [start], 0)]

        while stack:
            current, path, distance = stack.pop()

            if current == end:
                return path, distance

            if current not in visited:
                visited.add(current)
                for neighbor, dist in self.graph[current]:
                    if neighbor not in visited:
                        stack.append((neighbor, path + [neighbor], distance + dist))

        return None, 0
    
    def add_node_between(self, start, end, listNode):
        """
        두 노드 사이에 여러 새로운 노드를 추가합니다.
        listNode: [(노드 ID, 절대 거리)]의 리스트 (절대 거리 기준)
        """
        oldGraph = copy.deepcopy(self.graph)
        
        # 1. 두 노드 사이의 경로와 총 거리 찾기
        path, total_distance = self.find_path(start, end)
        print("Path:", path, "Total Distance:", total_distance)

        if not path:
            raise ValueError(f"No path exists between {start} and {end}")

        # 2. 절대 거리 -> 상대 거리로 변환
        absolute_distances = [0] + [distance for _, distance in listNode] + [total_distance]
        relative_distances = [absolute_distances[i] - absolute_distances[i - 1] for i in range(1, len(absolute_distances))]

        # listNode 업데이트: 절대 거리 -> 상대 거리
        #listNode = [(node_id, rel_distance) for (node_id, _), rel_distance in zip(listNode, relative_distances[1:-1])]
        listNode = [(node_id, rel_distance) for (node_id, _), rel_distance in zip(listNode, relative_distances[1:])]


        # 3. 경로를 따라가며 노드 삽입
        current_distance = 0
        prev_node = path[0]
        oldConfig, newConfig = None, None

        for next_node in path[1:]:
            # 간선 거리 가져오기
            edge_distance = next(
                (dist for neighbor, dist in self.graph[prev_node] if neighbor == next_node),
                None
            )

            if edge_distance is None:
                raise ValueError(f"No edge exists between {prev_node} and {next_node}")

            # 현재 간선에 새로운 노드 삽입
            while listNode and listNode[0][1] <= current_distance + edge_distance:
                # 새 노드 정보 가져오기
                new_node, rel_distance = listNode.pop(0)

                # 기존 간선 수정
                self.remove_edge(prev_node, next_node)
                self.add_edge(prev_node, new_node, rel_distance)
                self.add_edge(new_node, next_node, edge_distance - rel_distance)

                # if next_node in {start, end}:
                #     newConfig = new_node
                #     if prev_node not in [node_id for node_id, _ in listNode]:
                #         oldConfig = prev_node

                # 새 노드를 현재 간선의 시작 노드로 업데이트
                prev_node = new_node
                edge_distance -= rel_distance

            # 간선에 새 노드가 없으면 다음으로 진행
            current_distance += edge_distance
            prev_node = next_node

        # 남은 노드 확인
        if listNode:
            print("Remaining nodes in listNode:", listNode)
            raise ValueError("Not all nodes were added to the graph!")

        #return oldConfig, newConfig
        return get_adjacency_changes(oldGraph,self.graph)

    def add_node_between_old(self, start, end, listNode):
        """
        두 노드 사이에 여러 새로운 노드를 추가합니다.
        listNode: [(노드 ID, 절대 거리)]의 리스트 (절대 거리 기준)
        """
        # 1. 두 노드 사이의 경로와 총 거리 찾기
        path, total_distance = self.find_path(start, end)
        print(path,total_distance)
        if not path:
            raise ValueError(f"No path exists between {start} and {end}")

        # 2. 경로를 따라가며 노드 삽입
        listNodeCopy = []
        for nodeNewID in listNode:
            listNodeCopy.append(nodeNewID[0])
        current_distance = 0
        prev_node = path[0]
        oldConfig = None
        newConfig = None

        for next_node in path[1:]:
            # 간선 거리 가져오기
            edge_distance = next(
                (dist for neighbor, dist in self.graph[prev_node] if neighbor == next_node),
                None
            )

            if edge_distance is None:
                raise ValueError(f"No edge exists between {prev_node} and {next_node}")

            # 현재 간선에 새로운 노드 삽입
            while listNode and current_distance + edge_distance >= listNode[0][1]:
                # 새 노드 정보 가져오기
                new_node, abs_distance = listNode.pop(0)
                rel_distance = abs_distance - current_distance

                # 기존 간선 수정
                self.remove_edge(prev_node, next_node)
                self.add_edge(prev_node, new_node, rel_distance)
                self.add_edge(new_node, next_node, edge_distance - rel_distance)
                if (next_node == start or next_node == end):
                    newConfig = new_node
                    if not prev_node in listNodeCopy:
                        oldConfig = prev_node

                # 새 노드를 현재 간선의 시작 노드로 업데이트
                prev_node = new_node
                edge_distance -= rel_distance

            # 간선에 새 노드가 없으면 다음으로 진행
            current_distance += edge_distance
            prev_node = next_node
        print(path)
        return oldConfig,newConfig
    
    def visualize_graph(self):
        """
        self.graph의 데이터를 시각적으로 보여줍니다.
        - 직각 연결로 노드를 배치.
        - 노드 및 간선 레이블이 겹치지 않도록 조정.
        """
        # NetworkX 그래프 객체 생성
        G = nx.DiGraph()  # 방향 그래프를 사용해 직각 레이아웃 지원

        # self.graph 데이터를 NetworkX에 추가
        for node, edges in self.graph.items():
            for neighbor, weight in edges:
                G.add_edge(node, neighbor, weight=weight)

        # 직각 연결을 위한 그리드 레이아웃
        pos = nx.planar_layout(G)  # 직각 레이아웃에 가까운 배치

        # 그래프 그리기
        plt.figure(figsize=(12, 8))
        nx.draw(
            G,
            pos,
            with_labels=True,
            node_color="skyblue",
            node_size=2000,
            font_size=12,
            font_weight="bold",
            edgecolors="black"
        )

        # 간선 레이블 추가 (가중치 표시)
        edge_labels = nx.get_edge_attributes(G, 'weight')
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=10)

        # 노드 레이블 위치 조정
        for node, (x, y) in pos.items():
            plt.text(x, y + 0.05, str(node), fontsize=10, ha='center', va='center', color='blue')

        # 출력
        plt.title("Graph Visualization with Rectangular Layout", fontsize=16)
        plt.show()     
        
    def remove_node(self, node):
        """노드를 삭제하고 연결된 모든 간선 제거"""
        if node in self.graph:
            for neighbor, _ in self.graph[node]:
                self.graph[neighbor] = [edge for edge in self.graph[neighbor] if edge[0] != node]
            del self.graph[node]

    def to_text_file(self, file_path):
        """그래프를 텍스트 파일로 저장"""
        visited = set()
        lines = []

        for node, edges in self.graph.items():
            for neighbor, distance in edges:
                if (node, neighbor) not in visited and (neighbor, node) not in visited:
                    lines.append(f"{node}{sDivTab}{neighbor}{sDivTab}{distance}")
                    visited.add((node, neighbor))

        with open(file_path, "w") as file:
            file.write("\n".join(lines))

    @classmethod
    def from_text_file(cls, file_path):
        """텍스트 파일에서 그래프를 로드"""
        graph = cls()
        with open(file_path, "r") as file:
            for line in file:
                parts = line.strip().split(sDivTab)
                if len(parts) == 3:
                    node1, node2, distance = int(parts[0]), int(parts[1]), int(parts[2])
                    graph.add_edge(node1, node2, distance)
        return graph

    def __str__(self):
        """그래프를 문자열로 출력"""
        return str(self.graph)
    
    def getGraph(self):
        return self.graph
      
class Robot_Status(Enum):
    idle = auto()
    cali_tray = auto()
    cali_mainRotate = auto()
    onNoding = auto()
    onServing = auto()
    paused = auto()

class Robot_Event(Enum):
    start_calibration_tray = auto()
    complete_calibration_tray = auto()
    start_calibration_mainRotate = auto()
    complete_calibration_mainRotate = auto()
    start_noding = auto()
    complete_noding = auto()
    start_serving = auto()
    complete_serving = auto()
    pause_serving = auto()
    resume_serving = auto()

class Robot:
    def __init__(self):
        self.states = Robot_Status
        self.machine = Machine(model=self, 
                             states=self.states, 
                             initial=Robot_Status.idle)

        self.machine.add_transition(Robot_Event.start_noding.name,
                                  Robot_Status.idle, 
                                  Robot_Status.onNoding)
        self.machine.add_transition(Robot_Event.complete_noding.name,
                                  Robot_Status.onNoding, 
                                  Robot_Status.idle)
        self.machine.add_transition(Robot_Event.start_calibration_mainRotate.name,
                                  Robot_Status.idle, 
                                  Robot_Status.cali_mainRotate)
        self.machine.add_transition(Robot_Event.complete_calibration_mainRotate.name,
                                  Robot_Status.cali_mainRotate, 
                                  Robot_Status.idle)
        self.machine.add_transition(Robot_Event.start_calibration_tray.name,
                                  Robot_Status.idle, 
                                  Robot_Status.cali_tray)
        self.machine.add_transition(Robot_Event.complete_calibration_tray.name,
                                  Robot_Status.cali_tray, 
                                  Robot_Status.idle)
        self.machine.add_transition(Robot_Event.start_serving.name,
                                  Robot_Status.idle, 
                                  Robot_Status.onServing)
        self.machine.add_transition(Robot_Event.complete_serving.name,
                                  Robot_Status.onServing, 
                                  Robot_Status.idle)
        self.machine.add_transition(Robot_Event.pause_serving.name,
                                  Robot_Status.onServing, 
                                  Robot_Status.paused)
        self.machine.add_transition(Robot_Event.resume_serving.name,
                                  Robot_Status.paused, 
                                  Robot_Status.onServing)

    def trigger_event(self, event: Robot_Event) -> None:
        """이벤트를 트리거하는 내부 헬퍼 메서드"""
        getattr(self, event.name)()
        SendInfoHTTP(log_all_frames(event.name))

    def trigger_start_calibration_tray(self) -> None:
        #getattr(self, Robot_Event.start_calibration_tray.name)()        
        """트레이 캘리브레이션 시작"""
        self.trigger_event(Robot_Event.start_calibration_tray)

    def trigger_complete_calibration_tray(self) -> None:
        """트레이 캘리브레이션 완료"""
        self.trigger_event(Robot_Event.complete_calibration_tray)

    def trigger_start_noding(self) -> None:
        #getattr(self, Robot_Event.start_calibration_tray.name)()        
        """노드 생성 시작"""
        self.trigger_event(Robot_Event.start_noding)

    def trigger_complete_noding(self) -> None:
        """노드생성 완료"""
        self.trigger_event(Robot_Event.complete_noding)

    def trigger_start_calibration_mainRotate(self) -> None:
        """메인회전 캘리브레이션 시작"""
        self.trigger_event(Robot_Event.start_calibration_mainRotate)

    def trigger_complete_calibration_mainRotate(self) -> None:
        """메인회전 캘리브레이션 완료"""
        self.trigger_event(Robot_Event.complete_calibration_mainRotate)

    def trigger_start_serving(self) -> None:
        """서빙 시작"""
        self.trigger_event(Robot_Event.start_serving)

    def trigger_complete_serving(self) -> None:
        """서빙 완료"""
        self.trigger_event(Robot_Event.complete_serving)

    def trigger_pause_serving(self) -> None:
        """서빙 일시중지"""
        self.trigger_event(Robot_Event.pause_serving)

    def trigger_resume_serving(self) -> None:
        """서빙 재개"""
        self.trigger_event(Robot_Event.resume_serving)

    def get_current_state(self) -> Robot_Status:
        """현재 상태 반환"""
        return self.state
# robot = Robot()
# robot.start_task1()
# print(robot.state)  # 출력: task1
# robot.complete_task1()
# print(robot.state)  # 출력: idle


def calculate_orientation(ax, ay, az, mx, my, mz):
    """
    가속도계와 자기장 데이터를 사용하여 roll, pitch, yaw 계산
          # # IMU 메시지 생성
          # imu_msg = Imu()
          
          # # 타임스탬프 설정
          # imu_msg.header.stamp = rospy.Time.now()
          # imu_msg.header.frame_id = "imu_link"  # IMU의 프레임 ID 설정

          # # 각속도 (Angular Velocity)
          # imu_msg.angular_velocity.x = data.get("x", 0.0)
          # imu_msg.angular_velocity.y = data.get("y", 0.0)
          # imu_msg.angular_velocity.z = data.get("z", 0.0)

          # # 선형 가속도 (Linear Acceleration)
          # imu_msg.linear_acceleration.x = data.get("ax", 0.0)
          # imu_msg.linear_acceleration.y = data.get("ay", 0.0)
          # imu_msg.linear_acceleration.z = data.get("az", 0.0)

          # # Orientation은 기본적으로 쿼터니언(Quaternion)으로 제공
          # # Orientation 값을 계산할 경우 아래 주석 코드를 활성화하세요.
          # # roll, pitch, yaw 값을 기반으로 Orientation 설정 예시
          # # roll, pitch, yaw = 0.0, 0.0, 0.0  # 필요시 계산
          # # roll, pitch, yaw 계산
          # ax = data.get("ax", 0.0)
          # ay = data.get("ay", 0.0)
          # az = data.get("az", 0.0)
          # mx = data.get("mx", 0.0)
          # my = data.get("my", 0.0)
          # mz = data.get("mz", 0.0)
          # roll, pitch, yaw = calculate_orientation(ax, ay, az, mx, my, mz)
          # print(math.degrees(roll), math.degrees(pitch), math.degrees(yaw) )
                  
          # quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
          # imu_msg.orientation.x = quaternion[0]
          # imu_msg.orientation.y = quaternion[1]
          # imu_msg.orientation.z = quaternion[2]
          # imu_msg.orientation.w = quaternion[3]

          # # Covariance 값 설정 (필요시)
          # imu_msg.angular_velocity_covariance[0] = -1  # 기본값
          # imu_msg.linear_acceleration_covariance[0] = -1

          # # ROS 토픽 발행
          # imu_publisher.publish(imu_msg)
    
    """
    # Roll과 Pitch 계산
    roll = np.arctan2(ay, az)
    pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

    # Yaw 계산 (자기장 데이터 기반)
    mag_x = mx * np.cos(pitch) + my * np.sin(roll) * np.sin(pitch) + mz * np.cos(roll) * np.sin(pitch)
    mag_y = my * np.cos(roll) - mz * np.sin(roll)
    yaw = np.arctan2(-mag_y, mag_x)

    return roll, pitch, yaw
  
def replace_string(input_str: str):
    i1b = CARRIER_STATUS.I_DOOR_1_BOTTOM.name
    i2b = CARRIER_STATUS.I_DOOR_2_BOTTOM.name
    i1t = CARRIER_STATUS.I_DOOR_1_TOP.name
    i2t = CARRIER_STATUS.I_DOOR_2_TOP.name
    v12 = CARRIER_STATUS.O_V12_NC.name

    # ✅ 매핑 딕셔너리 정의
    mapping = {
        "O:2,4": f"{i1b}:0,{i1t}:1,{i2b}:0,{i2t}:1",
        "O:2,0": f"{i1b}:0,{i1t}:1",
        "O:2,1": f"{i2b}:0,{i2t}:1",
        "O:1,4": f"{i1b}:1,{i1t}:0,{i2b}:1,{i2t}:0",
        "O:1,0": f"{i1b}:1,{i1t}:0",
        "O:1,1": f"{i2b}:1,{i2t}:0",
        "V12": v12
    }

    # ✅ 일반적인 매핑 적용
    for k, v in mapping.items():
        if k in input_str:
            input_str = input_str.replace(k, v)
            return input_str  # 매핑이 완료되면 반환

    # ✅ "S:<무시할 숫자>,<실제 값>" 패턴 처리
    match = re.match(r"S:\d+,(\d+)", input_str)  # S: 뒤의 두 번째 숫자 추출
    if match:
        return f"TILT_ANGLE:{match.group(1)}"  # "TILT_ANGLE:10" 같은 변환
    # ✅ "L:<index>,<value1>,<value2>" → "L<index>:<value1>,<value2>" 변환
    match = re.match(r"L:(\d+),(\d+),(\d+)", input_str)
    if match:
        sMsg = f"L{match.group(2)}:{match.group(1)},{match.group(3)}"  # 변환 후 반환
        return sMsg
    return input_str  # 변환되지 않으면 원래 문자열 반환

def save_positions(file_path, trajectory):
    with open(file_path, "w", newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter='\t')
        writer.writerow(["nodeid", "x", "y","AngleZ", "distanceH"])
        writer.writerows(trajectory)
    rospy.loginfo(f"Position data saved to {file_path}")

def update_raw_data(marker_value, marker_x, marker_y, marker_z, trajectory):
    distance = round(math.sqrt(marker_x**2 + marker_y**2),3)
    angle = round(math.degrees(math.atan2(marker_y, marker_x)))
    n_id, n_x, n_y, angle_z,distanceH = trajectory[-1]
    table_x_candidate = round(n_x + distance * math.cos(math.radians(angle)),3)
    table_y_candidate = round(n_y + distance * math.sin(math.radians(angle)),3)
    data = {
        "marker_value": marker_value,
        "distance": distance,   
        "angle": angle,          
        "marker_z": marker_z,
        "angle_z": angle_z,
        "table_x": table_x_candidate,
        "table_y": table_y_candidate,
        "distanceH": distanceH,
        "node_id": n_id
    }
    return data

def save_table_positions(table_positions,filename):
    with open(filename, "w", newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter='\t')
        writer.writerow(["marker_value", "distance", "angle", "marker_z", "angle_z","table_x", "table_y","distanceH","node_id"])
        for marker_value, (distance, angle, marker_z, angle_y, table_x, table_y,distanceH,node_id) in table_positions.items():
            writer.writerow([marker_value, distance, angle, marker_z, angle_y, table_x, table_y,distanceH,node_id])
    rospy.loginfo(f"Table position data saved to {filename}")

def plot_positions(img_pathCurrent, node_file_path="posXY.csv", table_file_path="table_positions.csv"):
    try:
        # Read Node Data
        node_ids, x_values, y_values = [], [], []
        with open(node_file_path, "r") as csvfile:
            reader = csv.reader(csvfile, delimiter='\t')
            next(reader)  # Skip header
            for row in reader:
                node_ids.append(int(row[0]))
                x_values.append(float(row[1]))
                y_values.append(float(row[2]))
        
        # Read Table (Marker) Data
        table_ids, table_x, table_y ,distanceH,node_id= [], [], [],[],[]
        with open(table_file_path, "r") as csvfile:
            reader = csv.reader(csvfile, delimiter='\t')
            next(reader)  # Skip header
            for row in reader:
                table_ids.append((row[0])) # marker_value
                table_x.append(float(row[5]))  # table_x
                table_y.append(float(row[6]))  # table_y
                distanceH.append(row[7])  # distance_H
                node_id.append(row[8])  # node_id
        
        # Plot Nodes
        plt.figure(figsize=(8, 8))
        plt.scatter(x_values, y_values, c='blue', label='Nodes', zorder=2)
        plt.plot(x_values, y_values, linestyle='-', color='blue', alpha=0.6, zorder=1)
        
        # Annotate node IDs
        for i, txt in enumerate(node_ids):
            plt.annotate(txt, (x_values[i], y_values[i]), textcoords="offset points", xytext=(5,5), ha='right', fontsize=5, color='red')
        
        # Plot Tables (Markers) in Red
        plt.scatter(table_x, table_y, c='red', marker='s', label='Tables', zorder=3)
        for i, txt in enumerate(table_ids):
            plt.annotate(f"T{txt}({node_id[i]},{distanceH[i]})", (table_x[i], table_y[i]), textcoords="offset points", xytext=(5,5), ha='right', fontsize=5, color='black')
        
        # Axis settings
        plt.axhline(0, color='black', linewidth=0.5, zorder=1)
        plt.axvline(0, color='black', linewidth=0.5, zorder=1)
        plt.grid(color='gray', linestyle='--', linewidth=0.5, zorder=0)
        plt.title('Node & Table Location')
        plt.xlabel('Loc X')
        plt.ylabel('Loc Y')
        
        # Set equal aspect ratio
        min_val = min(min(x_values), min(y_values), min(table_x, default=0), min(table_y, default=0))
        max_val = max(max(x_values), max(y_values), max(table_x, default=0), max(table_y, default=0))
        margin = (max_val - min_val) * 0.1  # 10% margin
        plt.xlim(min_val - margin, max_val + margin)
        plt.ylim(min_val - margin, max_val + margin)
        
        plt.legend()
        plt.gca().set_aspect('equal', adjustable='box')
        
        # Save as PNG
        plt.savefig(img_pathCurrent)
        plt.close()
        rospy.loginfo(f"Position plot saved as {img_pathCurrent}")
    except Exception as e:
        rospy.logerr(f"Error plotting positions: {e}")

def rotate_marker(marker_data, angle):
    """
    주어진 ArUco 마커 데이터를 주어진 각도만큼 회전합니다.
    
    :param marker_data: dict 형태의 ArUco 마커 정보
    :param angle: 회전 각도 (0~360도)
    :return: 회전된 마커 데이터
    """
    # 각도 변환을 라디안 단위로 변경
    rad = np.radians(angle)

    # 회전 변환 행렬
    rotation_matrix = np.array([
        [np.cos(rad), -np.sin(rad)],
        [np.sin(rad), np.cos(rad)]
    ])

    rotated_data = copy.deepcopy(marker_data)

    # 마커 중심 회전
    x, y = marker_data["X"], marker_data["Y"]
    rotated_x, rotated_y = rotation_matrix @ np.array([x, y])
    rotated_data["X"] = rotated_x
    rotated_data["Y"] = rotated_y

    # 코너 좌표 회전
    for i in range(1, 5):
        cx, cy = marker_data[f"cx{i}"], marker_data[f"cy{i}"]
        rotated_cx, rotated_cy = rotation_matrix @ np.array([cx, cy])
        rotated_data[f"cx{i}"] = rotated_cx
        rotated_data[f"cy{i}"] = rotated_cy

    # 마커의 각도 업데이트
    rotated_data["ANGLE"] = (marker_data["ANGLE"] + angle) % 360

    return rotated_data

def update_node_bindings(table_data_path, node_data_path):
    """
    Updates node bindings based on shortest distance between tables and nodes.
    
    Args:
        table_data_path (str): Path to CSV file containing table data
        node_data_path (str): Path to CSV file containing node data
        
    Returns:
        pd.DataFrame: Updated table data with new node bindings and metrics
    """
    # Read CSV files
    table_df = pd.read_csv(table_data_path, sep='\t')
    node_df = pd.read_csv(node_data_path, sep='\t')
    
    # Function to calculate distance between two points
    def calculate_distance(x1, y1, x2, y2):
        return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    # Function to calculate angle between two points
    def calculate_angle(x1, y1, x2, y2):
        angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
        # Normalize angle to 0-360 range
        return (angle + 360) % 360
    
    # Initialize lists to store updated values
    new_node_ids = []
    new_distances = []
    new_angles = []
    new_distanceHs = []
    #print(table_df)
    # Process each table
    for _, table in table_df.iterrows():
        #print(table)
        table_x = table['table_x']
        table_y = table['table_y']
        
        # Calculate distances to all nodes
        distances = []
        for _, node in node_df.iterrows():
            dist = calculate_distance(table_x, table_y, node['x'], node['y'])
            distances.append((node['nodeid'], dist, node['AngleZ'], node['distanceH']))
        
        # Find closest node
        closest_node = min(distances, key=lambda x: x[1])
        
        # Calculate new angle
        new_angle = calculate_angle(table_x, table_y, 
                                  node_df.loc[node_df['nodeid'] == closest_node[0], 'x'].iloc[0],
                                  node_df.loc[node_df['nodeid'] == closest_node[0], 'y'].iloc[0])
        
        # Store updated values
        new_node_ids.append(int(closest_node[0]))
        new_distances.append(closest_node[1])
        new_angles.append(new_angle)
        new_distanceHs.append(closest_node[3])
    
    # Update the table DataFrame
    table_df['node_id'] = new_node_ids
    table_df[APIBLB_FIELDS_INFO.distance.name] = new_distances
    table_df['angle'] = new_angles
    table_df['distanceH'] = new_distanceHs
    
    return table_df


def service_setbool_client_common(serviceName, enable, serviceType):
    bResult = None
    if isServiceExist(serviceName) == False:
        log_all_frames(f"Service not found : {serviceName}")
        return False

    serviceNameID = None
    if enable == None:
        serviceNameID = f"{serviceNameID}"
    else:
        serviceNameID = f"{serviceNameID}{enable}"

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
        log_all_frames(enable,2)
        return responseResult
    except Exception as e:
        rospy.loginfo(e)
        return False  

def set_tasmota_state(ip, stateBool:bool):
    """
    주어진 Tasmota 플러그의 전원을 on/off 하고, 전력 소비량(Power)을 읽어옵니다.
    
    인자:
        ip (str): Tasmota 플러그의 IP 주소 (예: "172.30.1.71")
        state (str): "on" 또는 "off" (대소문자 구분 없음)
        
    반환:
        boolean : 제어 성공 여부
    """
    # 전원 제어 명령: URL은 "Power On" 또는 "Power Off" 형식
    state = 'ON' if stateBool else 'OFF'
    
    control_url = f"http://{ip}/cm?cmnd=Power%20{state}"
    try:
        ctrl_resp = requests.get(control_url, timeout=5)
        ctrl_resp.raise_for_status()
    except Exception as e:
        print("전원 제어 중 에러 발생:", e)
        return False
    return True

def get_tasmota_state(ip):
    """
    주어진 Tasmota 플러그의 전원 상태를 조회합니다.
    
    인자:
        ip (str): Tasmota 플러그의 IP 주소 (예: "172.30.1.71")
        
    반환:
        str: "ON" 또는 "OFF", 조회 실패 시 None
    """
    url = f"http://{ip}/cm?cmnd=Power"
    try:
        resp = requests.get(url, timeout=5)
        resp.raise_for_status()
        data = resp.json()  # 응답 예시: {"POWER": "ON"}
        return data.get("POWER")
        #return data
    except Exception as e:
        print(f"전원 상태 조회 중 에러 발생:{url},{e}")
        return None
    
def get_tasmota_info(ip):
    # 전력 소비량 읽기: "Status 8" 명령 사용
    status_url = f"http://{ip}/cm?cmnd=Status%208"
    try:
        status_resp = requests.get(status_url, timeout=5)
        status_resp.raise_for_status()
        status_data = status_resp.json()
    except Exception as e:
        print("상태 정보 읽기 중 에러 발생:", e)
        return None
    return status_data["StatusSNS"]["ENERGY"]

    # # JSON 응답에서 전력 소비량(Power) 값 추출 (일반적으로 StatusSNS > ENERGY > Power)
    # try:
    #     power_consumption = status_data["StatusSNS"]["ENERGY"]["Power"]
    # except Exception as e:
    #     print("전력 소비량 데이터 파싱 중 에러 발생:", e)
    #     power_consumption = None

#     # return {"power_state": state.capitalize(), "power_consumption": power_consumption}
# print(get_tasmota_info(BLB_CHARGER_IP_DEFAULT))
# print(get_tasmota_state(BLB_CHARGER_IP_DEFAULT))
# print(set_tasmota_state(BLB_CHARGER_IP_DEFAULT, "on"))

def SetCrossPlug(state : bool):
    return set_tasmota_state(BLB_CROSSPLUG_IP_DEFAULT, state)

def SetChargerPlug(state : bool):
    return set_tasmota_state(BLB_CHARGERPLUG_IP_DEFAULT, state)

def getStatePlug(ip : str):
    return get_tasmota_state(ip)

def getCrossPlugState():
    return getStatePlug(BLB_CROSSPLUG_IP_DEFAULT)

def getChargerPlugState():
    return getStatePlug(BLB_CHARGERPLUG_IP_DEFAULT)

def RFIDControl(enable : bool):
    bReturn,strResult = None,None
    if isRealMachine:
        invValue = 1 if enable else 0
        bReturn,strResult = API_call_http(BLB_RFID_IP,HTTP_COMMON_PORT,"rfid",f'inv={invValue}')
        log_all_frames(strResult)
    return bReturn,strResult

def RFIDPwr(enable : int):
    bReturn,strResult = None,None
    if isRealMachine:
        if is_between(1000,2000,enable):
            bReturn,strResult = API_call_http(BLB_RFID_IP,HTTP_COMMON_PORT,"rfid",f'pwr={enable}')
    return bReturn,strResult

def MoveH_MotorRFID(POS,spd,endnode):
    msg = f"{MotorWMOVEParams.POS.name}={POS}&{MotorWMOVEParams.SPD.name}={spd}&endnode={endnode}"
    rtMsg = log_all_frames(msg)
    SendInfoHTTP(rtMsg)
    rs = API_call_http(IP_MASTER,HTTP_COMMON_PORT,EndPoints.JOG.name, msg)
    return rs

def SetIMUInterval(time_ms):
    calStr = time_ms
    API_call_Android(BLB_ANDROID_IP_DEFAULT,HTTP_COMMON_PORT,f'time={calStr}')

def SetCameraMode(cm : CameraMode):
    #calStr = 'camid=1,640,480'
    calStr = cm.value
    API_call_Android(BLB_ANDROID_IP_DEFAULT,HTTP_COMMON_PORT,calStr)

def TTSAndroid(ttsMsg,isQueueing = True, ttsInterval = 1):
    calStr = f'tts=10,10,{ttsMsg}'
    API_call_Android(BLB_ANDROID_IP_DEFAULT,HTTP_COMMON_PORT,calStr)
    return True

def GetCrossInfo():
    StateInfo: Dict[str, list] = {}
    file_list = getLines_FromFile(strFileCross)
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
            sMsg = f"잘못된 형식의 분기기 정보! {i}"
            rospy.loginfo(sMsg)
            #SendAlarmHTTP(sMsg)
    return StateInfo

def GetNodeFromTable(curTable):
    file_path = strFileTableNodeEx
    df_manager = DataFrameManager(file_path)
    if curTable is not None:
        curNodeList = df_manager.filter_by_key(TableInfo.TABLE_ID.name, str(curTable))
        if len(curNodeList) == 0:
            return None
        curNode = curNodeList.iloc[0][TableInfo.NODE_ID.name]
        return try_parse_int(curNode)
    return None

def GetRange(value=None, column_input="", column_output=None, df = pd.DataFrame()):
    # 컬럼 존재 여부 확인    
    if column_input not in df.columns or column_output not in df.columns:
        return -1

    if value is None:
        return df[column_output].max()

    # column_input 기준으로 정렬 (오름차순)
    df_sorted = df.sort_values(by=column_input, ascending=True).reset_index(drop=True)

    # 값이 데이터의 범위를 벗어나면 경계값 반환
    min_val, max_val = df_sorted[column_input].min(), df_sorted[column_input].max()
    if value <= min_val:
        return df_sorted[column_output].iloc[0]
    if value >= max_val:
        return df_sorted[column_output].iloc[-1]

    # 가장 가까운 두 개의 점을 찾기
    lower_idx = df_sorted[column_input].searchsorted(value, side='right') - 1
    upper_idx = lower_idx + 1

    # 데이터 범위 확인
    if upper_idx >= len(df_sorted):
        return df_sorted[column_output].iloc[lower_idx]

    x0, y0 = df_sorted[column_input].iloc[lower_idx], df_sorted[column_output].iloc[lower_idx]
    x1, y1 = df_sorted[column_input].iloc[upper_idx], df_sorted[column_output].iloc[upper_idx]

    # 선형 보간 공식 적용
    interpolated_value = y0 + (y1 - y0) * (value - x0) / (x1 - x0)

    return interpolated_value

# def GetTargetPulseServingArm(strokeTargetMeter, not_pos=-200000):
#     dfDistanceArm = pd.read_csv(strFileDistanceArm, delimiter=sDivTab)
#     maxArmLD = GetRange(0,DISTANCE_V.pulseV.name,DISTANCE_V.distanceLD.name,dfDistanceArm)
#     minArmPulse = GetRange(maxArmLD,DISTANCE_V.distanceLD.name,DISTANCE_V.pulseV.name,dfDistanceArm)
#     maxArmPulse = GetRange(None,DISTANCE_V.distanceLD.name,DISTANCE_V.pulseV.name,dfDistanceArm)
#     minArmLD = GetRange(maxArmPulse,DISTANCE_V.pulseV.name,DISTANCE_V.distanceLD.name,dfDistanceArm)
#     strokeTotal = maxArmLD - minArmLD   
#     strokeKey = maxArmLD - strokeTargetMeter
#     strokeTargePulse = round(GetRange(strokeKey,DISTANCE_V.distanceLD.name,DISTANCE_V.pulseV.name,dfDistanceArm)) + not_pos
#     #print(minArmLD,minArmPulse,strokeTargePulse)
#     return strokeTargePulse
def GetTargetLengthMMServingArm(strokeTargetPulse, not_pos=-200000):
    dfDistanceArm = pd.read_csv(strFileDistanceArm, delimiter=sDivTab)
    strokeTargetPulse = strokeTargetPulse - not_pos
    strokeTargeLen = GetRange(strokeTargetPulse,DISTANCE_V.pulseV.name,DISTANCE_V.distanceLD.name,dfDistanceArm)
    #print(minArmLD,minArmPulse,strokeTargePulse)
    return round(strokeTargeLen)
# print(GetTargetLengthMMServingArm(250000))

#print(MoveH_MotorRFID(350000,500,7) ) 
def GetTargetPulseServingArm(strokeTargetMM, not_pos=-200000):
    dfDistanceArm = pd.read_csv(strFileDistanceArm, delimiter=sDivTab)
    strokeTargePulse = round(GetRange(strokeTargetMM,DISTANCE_V.distanceLD.name,DISTANCE_V.pulseV.name,dfDistanceArm)) + not_pos
    return strokeTargePulse


# print(GetTargetPulseServingArm(100))
# print(GetTargetPulseServingArm(500))
# print(GetTargetPulseServingArm(1000))
# print(GetTargetPulseServingArm(1500))
# print(GetTargetPulseServingArm(2000))
    
#print(MoveH_MotorRFID(350000,500,7) ) 
def GetTargetPulseBalanceArm(strokeSrvTargetMM):
    dfDistanceArm = pd.read_csv(strFileDistanceBal, delimiter=sDivTab)
    strokeTargePulse = round(GetRange(strokeSrvTargetMM,DISTANCE_V.distanceLD.name,DISTANCE_V.pulseV.name,dfDistanceArm))
    return strokeTargePulse
print(GetTargetPulseBalanceArm(1250))


# def GetTargetLengthServingArm(strokeTargetPulse, not_pos=0):
#     dfDistanceArm = pd.read_csv(strFileDistanceArm, delimiter=sDivTab)
#     maxArmLD = GetRange(0,DISTANCE_V.pulseV.name,DISTANCE_V.distanceLD.name,dfDistanceArm)
#     minArmPulse = GetRange(maxArmLD,DISTANCE_V.distanceLD.name,DISTANCE_V.pulseV.name,dfDistanceArm)
#     maxArmPulse = GetRange(None,DISTANCE_V.distanceLD.name,DISTANCE_V.pulseV.name,dfDistanceArm)
#     minArmLD = GetRange(maxArmPulse,DISTANCE_V.pulseV.name,DISTANCE_V.distanceLD.name,dfDistanceArm)
#     strokeTotal = maxArmLD - minArmLD   
#     strokeKey = strokeTargetPulse - not_pos
#     strokeTargeLen = GetRange(strokeKey,DISTANCE_V.pulseV.name,DISTANCE_V.distanceLD.name,dfDistanceArm)
#     #print(minArmLD,minArmPulse,strokeTargePulse)
#     result = maxArmLD - strokeTargeLen
#     return round(result*1000)

def GetTableFromNode(curNodeSt):
    if curNodeSt is not None:
        curNodeint = int(curNodeSt)
        file_path = strFileTableNodeEx
        if isFileExist(file_path):
            df_manager = DataFrameManager(file_path)
            if curNodeint is not None:
                curTableList = df_manager.filter_by_key(TableInfo.NODE_ID.name, curNodeint)
                if len(curTableList) == 0:
                    return []
                else:
                    # 모든 TABLE_ID 값을 리스트로 추출
                    curTables = curTableList[TableInfo.TABLE_ID.name].tolist()
                    return curTables
    return []

def getTableServingInfo(tableNo):
    file_path = strFileTableNodeEx
    if not isFileExist(file_path):
      return {}
    if str(tableNo).startswith('T'):
      tableNo = tableNo[1:]
    df_manager = DataFrameManager(file_path)
    curNodeList = df_manager.filter_by_key(TableInfo.TABLE_ID.name, str(tableNo))
    if len(curNodeList) == 0:
      return {}
    dicRt = curNodeList.to_dict(orient='records')
    smallest_serving_distance = min(dicRt, key=lambda x: x[TableInfo.SERVING_DISTANCE.name])
    # print(type(dicRt))
    # print(dicRt)
    return smallest_serving_distance

#테이블의 아르코마커 업데이트 여부를 가져온다
def isScanTableMode(tableNo):
    dicTagretTableInfo = getTableServingInfo(tableNo)
    infoLIFT_Height = try_parse_int(dicTagretTableInfo.get(TableInfo.MARKER_VALUE.name), 0)
    return infoLIFT_Height <= 0

def compare_better_marker(dictOld, dictNew, dictSample = ref_dict):
    if not dictOld:
        return True
    if not dictNew:
        return False
    resultDiff1,diff_X1,diff_Y1= compare_dicts(dictOld, dictSample, CAM_LOCATION_MARGIN_OK)
    resultDiff2,diff_X2,diff_Y2= compare_dicts(dictNew, dictSample, CAM_LOCATION_MARGIN_OK)
    return abs(diff_X1) > abs(diff_X2)
    