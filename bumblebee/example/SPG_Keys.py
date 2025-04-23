from varname import *
from enum import Enum, auto

class ModbusKeys(Enum):
    CMD = auto()

class BUM_Keys(Enum):
    CMD = auto()
    POS = auto()
    SPD = auto()
    ACC = auto()
    DECC = auto()
    MBID = auto()
    MODE = auto()

class SPG_Keys(Enum):
    LASTSEEN = auto()
    START = auto()
    STOP = auto()
    DESC = auto()
    TYPE = auto()
    REP = auto()
    RFIDPWR = auto()
    HEIGHT = auto()
    SCAN_3D = auto()
    MULTI_CAM = auto()
    #WAVESCAN = auto()
    SPD_H = auto()
    MOVE_ = auto()
    TIER = auto()
    LENGTH = auto()
    ANGLE = auto()
    EPC = auto()
    POS_R = auto()
    POS_L = auto()
    OPMODE = auto()
    REMAINS = auto()
    STATUS = auto()
    ID = auto()
    DEBUG = auto()
    LASTCMD = auto()
    CURRTIME = auto()
    RECVTIME = auto()
    ACC = auto()
    DECC = auto()
    ACT = auto()


'''
2023-02 스파이더 커맨드 프로토콜 최적화 작업으로 새롭게 메세지 정의
메인 테이블에서는 작업해야 할 지점(엔코더)과 프로필ID만 정해줌.
'''
class SPG_Profile(Enum):
    LASTSEEN = auto()
    START = auto()
    PROFILE = auto()
    HEIGHT = auto()
    LENGTH = auto()
    SPD_H = auto()
    SPD_D = auto()
    SPD_U = auto()
    TIER = auto()
    ACC_H = auto()
    DECC_H = auto()
    ACC_V = auto()
    DECC_V = auto()
    ACT_H = auto()
    ACT_D = auto()
    ACT_U = auto()
    COMMENT = auto()

class SPG_CMD(Enum):
    RETURN = auto()
    MAP = auto()
    MAPPING = auto()
    AVOID = auto()
    SKIPV = auto()
    REP = auto()
    ARD_N = auto()
    ARD_U = auto()
    CAM_T = auto()

''' 각 노드에 내리는 명령어 '''
class COMMON_CMD(Enum):
    ID = auto()
    CMD = auto()
    VALUE = auto()

