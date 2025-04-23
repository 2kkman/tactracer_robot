from varname import *
from enum import Enum, auto

class CmdStatus(Enum):
    Alarm = auto()
    Canceled = auto()
    Started = auto()
    Finished = auto()

class ServoKey(Enum):
    MOVE_TYPE=0x6200
    MOVE_POSITION_H=0x6201
    MOVE_POSITION_L=0x6202
    MOVE_SPD=0x6203
    MOVE_ACC_TIME=0x6204
    MOVE_DEC_TIME=0x6205
    MOVE_DWELL_TIME=0x6206
    MOVE_PATHNUMBER=0x6207
    SET_HOME1=0x600A
    SET_HOME2=0x6002
    SERVO_ENABLE=0x405
    E_STOP=0x6002
    MONITOR_LOC=0x602A
    MONITOR_STATUS=0x6002 #상태모니터링
    CLEAR_ALARM=0X33

class ServoParam(Enum):
    MOVE_TYPE_ABS=1 #절대위치
    MOVE_TYPE_REL=65 #상대위치
    MOVE_TYPE_SPD=2 #속도유지
    MOVE_PATHNUMBER_DEFAULT=0x10 #PathNumber 설정값
    SET_HOME_HERE=0x20
    SERVO_ENABLE=0x405
    HOMING_RPM = 500
    E_STOP_NOW=0x40
    MONITOR_LOC=0x602A
    MONITOR_STATUS=0x6002 #상태모니터링
    CLEAR_ALARM_NOW=0x1111
    ServoLimit_Addr = 0x6006


class ServoMonitorLOC(Enum):
    ServoMonitorLOC_Addr = 0x602A
    #ServoMonitorLOC_Items = ['LOC_CMD_1', 'LOC_CMD_2', 'LOC_MOT_1', 'LOC_MOT_2',4,5]
    ServoMonitorLOC_Items = ['PRO_MODE', 'LOCCMD_1', 'LOCCMD_2', 'SPD_CMD','ACC_TIME','DEC_TIME']
    ServoMonitorLOC_Len = len(ServoMonitorLOC_Items)

class ServoMonitorALM(Enum):
    ALM_CD = 0
    ALM_STR = 'READY'
    ServoMonitorALM_Addr = 0x0B03
    ServoMonitorALM_Items = ['ALM', 'MOTOR_FACTOR', 'DRV_STATE', 'ACTU_SPD','ACT_TORQ','ACT_CURRENT', 'ACTF_SPD', 'DC_VOLT'
    ,'DRV_TEMPER', 12,13,14,'OVERLOAD_RATIO', 'REGENLOAD_RATIO', 17, 18, 19, 20
    , 21,22,23, 24,25,26
    , 27,'LOCMOT_1', 'LOCMOT_2','DEVI_ENC_1','DEVI_ENC_2',30,31]
    # ServoMonitorALM_Items = ['ALM', 'MOTOR_FACTOR', 'DRV_STATE', 'ACTU_SPD','ACT_TORQ','ACT_CURRENT', 'ACTF_SPD', 'DC_VOLT'
    # ,'DRV_TEMPER', 12,13,14,'OVERLOAD_RATIO', 'REGENLOAD_RATIO', 'DIN_STATUS', 'DOUT_STATUS', 19, 'LOCCMD_1'
    # , 'LOCCMD_2','PULSE_1','PULSE_2', 'DEVI_CMD_1','DEVI_CMD_2','LOCENC_1'
    # , 'LOCENC_2','LOCMOT_1', 'LOCMOT_2','DEVI_ENC_1','DEVI_ENC_2','POS_RORATION_1','POS_RORATION_2']
    ServoMonitorALM_Len = len(ServoMonitorALM_Items)

class ServoMonitorStatus(Enum):
    NAME = 'MONITOR_STATUS'
    STATUS_ADDR = 0x6002
    ERR01 = 0x01
    ERR20 = 0x20
    ERR40 = 0x40
    READY=0 #위치 지정이 완료되고 새 데이터를 수신 할 수 있음을 나타냄 Ready
    MOVING = 0x100 #경로가 실행 중임을 표시 Moving
    FINISHING = 0x200 #경로가 실행 중임을 표시 Movingsa
