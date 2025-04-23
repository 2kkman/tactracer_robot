from varname import *
from enum import Enum, auto

'''
구분자 정의 예제 메세지

RESULT:OK`CMDN:U`MSGN:SPG2060N`NESTCHARGE_NC:0`BREAK_V_NC:0`USBHUB_TXRX_NC:0`USBHUB_VCC_NC:0`GOR_SVN:-86.88,-0.27,1.90`GAV_SVN:0.00,-0.00,0.00`GLA_SVN:9.81,-0.05,0.53`GAV_TUN:0.00`TEMPN:30.67`TEMPAHT_N:27.06`HUMIDAHT_N:18.88

` 로 Split 하면 Key-Value 값 추출됨 ['RESULT:OK','CMDN:U' ... ] 
각 값에 대해 : 로 Split 하면 Key 와 Value 를 구할 수 있음
Value 에 여러값이 결합된 경우 , 로 Split (예 : GOR_SVN:-86.88,-0.27,1.90)
'''
class StrParser(Enum):
    sDivEmart1 = "`"
    sDivColon2 = ":"
    sDivComma3 = ","
    sDivSlash = "/"
    sDivSemiColon = ";"

'''
명령 실행 상태 정의 클래스
'''
class CmdStatus(Enum):
    Alarm = auto()
    Canceled = auto()
    Started = auto()
    Finished = auto()
    OnProcess = auto()

'''
범블비 일일 운행 기록 필드
'''
class ReportFields(Enum):
    DATE = auto() #날짜
    TIMESTAMP_START = auto() #작업시작시간
    TIMESTAMP_END = auto()  #작업종료시간
    VOLTAGE_START = auto() #시작 전압
    VOLTAGE_END = auto() #종료전압
    SERVED_COUNT = auto() #서빙횟수
    SERVED_WEIGHT = auto()  #일일 누적 서빙 하중

'''
범블비 서비스 정의 - 업데이트 중
'''
class BLB_CMD(Enum):
    ID =  auto()
    TRAY_A = auto()
    TRAY_B = auto()
    LEVEL = auto()
    STATE = auto()
    TIME = auto()
    
class BLB_STATUS(Enum):
    ID = auto()
    STATUS = auto()
    
class Services(Enum):
    BAR_SET = '/carrier/BARCD'
    IMG_PUBLISH = '/carrier/IMG'
    IMG_SAVE = '/carrier/saveImage'


''' 범블비 토픽명 - 업데이트 중'''
class TOPICS(Enum):
    scan_alarm = 'scan_alarm_util'
    scheduler = 'scheduler'
    utilIMU = 'imu'
    nestIMU = '/nestImu'
    scanutil = '/scanutil'
    SD = 'SD'
    SERVO = 'SERVO'
    RFID = 'RFID'
    BMS = 'GOAL'
    CMD = 'CMD'
    FEEDBACK = 'FEEDBACK'
    KEEPALIVE = 'KEEPALIVE'
    ENCODER = 'ENCODER'
    GOAL = 'GOAL'
    ALARM = 'ALARM'
    UTIL_ARD = 'KEEP_UTIL'
    KEEP_ARD = 'KEEP_ARD'

'''
LED 상태
P, B, L 값으로 구분된다.
L - BLINK 주기 / 0 : Solid , 1000 : 빠른깜빡임, 1500:느린깜빡임
B - BLINK On/OFF
P:7,n - 7번핀의 PWM 값을 조정한다. 248 은 빨강, 168은 녹색, 88은 파랑
'''
class LED_Status(Enum):
    #RED
    ON_REDSOLID  = [ 'P:7,248','B:0']
    CHARGING  = [ 'P:7,248','B:0']
    ERROR_CHARGE  = [ 'P:7,216','B:1','L:1500' ]
    ERROR_FATAL  = [ 'P:7,248','B:1','L:1000' ]
    #GREEN
    ON_GREENSOLID  = [ 'P:7,168','B:0']
    ON_CHARGEFULL  = [ 'P:7,168','B:0']
    ON_OPERATION  = [ 'P:7,136','B:1','L:1500' ]
    ON_DATA  = [ 'P:7,168','B:1','L:1000' ]
    #BLUE
    ON_BLUESOLID  = [ 'P:7,88','B:0']
    ON_ARDINO  = [ 'P:7,88','B:1','L:1000' ]
    ON_UTILBOXLOADED  = [ 'P:7,88','B:0']
    ON_NODESERVO  = [ 'P:7,56','B:1','L:1500' ]


'''
충전상태 정의상수
'''
class CHARGE_Status(Enum):
    DISCHARGING  = 2
    CHARGING = 0
    CHARGEFULL = 1
    UNKNOWN = -1
'''
class ServiceList(Enum):
    LIDAR_START = '/spg_recording/start'
    LIDAR_STOP = '/spg_recording/stop'
    SDV16 = '/nestSD/SDV16'
    SDV24 = '/nestSD/SDV24'
    SDV5 = '/nestSD/SDV5'
    SDV16LOCK = '/nestSD/SDV16LOCK'
    SDV24LOCK = '/nestSD/SDV24LOCK'
    SDV5LOCK = '/nestSD/SDV5LOCK'
    CMDARD = '/nestARD/CMD'
    CMDSCR = '/nestARD/SCR'
    CMDARD_UTIL = '/utilbox/CMD'
    CMDARD_RUNSCRIPT = '/utilbox/SCR'
    NEST_CHARGE_ENABLE = '/nestARD/CHARGE_ENABLE'
    NEST_USB_ENABLE = '/nestARD/USB_ENABLE'
    NEST_BREAK_ENABLE = '/nestARD/BREAK_ENABLE'
    NEST_GPO_SET = '/nestARD/GPO_SET'
    NEST_RFID_INV = '/nestRFID/InvRFID'
    CAM_ENABLE = '/cam/start_capture'
    CAM_L_ENABLE = '/usb_cam_L/start_capture'
    CAM_R_DISABLE = '/usb_cam_R/stop_capture'
    CAM_L_DISABLE = '/usb_cam_L/stop_capture'
    CAM_PREFIX = '/utilbox/saveImage'
    CAM_PREFIX_FOCUS = '/utilbox/saveImageFocus'
    CAM_PREFIX_TEST = '/utilbox/saveImageTest'
    CAM_SAVE_START = '/utilbox/save'
    CAM_SAVE_STOP = '/utilbox/saveStop'
    GOAL = 'GOAL'
    ALARM = 'ALARM'
    def GetEnumValue(strCurrent):
        sCurrent = str(strCurrent)
        for s in Config_Main:
            sCurrentTmp = str(s)
            if sCurrent.endswith(sCurrentTmp):
                return s.value
        return -1
'''