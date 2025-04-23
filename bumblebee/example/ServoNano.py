from varname import *
from enum import Enum, auto

class ServoKey(Enum):
    MOVE_TYPE=0x6200
    MOVE_POSITION_H=0x6000
    MOVE_POSITION_L=0x6001
    MOVE_SPD=0x6202
    MOVE_ACC_TIME=0x6004
    MOVE_DEC_TIME=0x6006
    MOVE_DWELL_TIME=-1
    MOVE_PATHNUMBER=-1
    SET_HOME1=6060
    SET_HOME2=6040
    SERVO_ENABLE=0x405
    E_STOP=6008
    MONITOR_LOC=0x6064
    MONITOR_STATUS=0x6041 #상태모니터링
    CLEAR_ALARM=6008

class ServoParam(Enum):
    MOVE_TYPE_ABS=31+32 #절대위치
    MOVE_TYPE_REL=95+32 #상대위치
    MOVE_TYPE_SPD=-1 #속도유지
    MOVE_PATHNUMBER_DEFAULT=-1
    SET_HOME_HERE1=0x6
    SET_HOME_HERE2='6,7,15,31'
    SERVO_ENABLE=15
    HOMING_RPM = 100
    E_STOP_NOW=27
    E_SPD_CHANGE=63
    E_RESUME = 31
    MONITOR_LOC=0x602A
    MONITOR_STATUS=0x6041 #상태모니터링
    CLEAR_ALARM_NOW=128

class ServoMonitor(Enum):
    #ServoMonitorLOC_Addr = 0x602A
    ServoMonitorStatus_Addrs = [5000,5001,5002,5003,5004,5005,5006,5007]
    ServoMonitorField_StartAddr = 6000
    ServoMonitorField_CtrlAddr = 6008
    ServoMonitorField_SpeedAddr = 6002
    ServoMonitorField_LimitAddr = 6011
    ServoMonitorField_Items = ['LOCCMD_1', 'LOCCMD_2','SPDCMD_1', 'SPDCMD_2', 'ACC_TIME_1','ACC_TIME_2'
    ,'DEC_TIME_1','DEC_TIME_2','CTRL_MODE','SPDTAR_1','SPDTAR_2','CUR_MODE']
    ServoMonitorLOC_Len = len(ServoMonitorField_Items)

# class ServoMonitorALM(Enum):
#     ALM_CD = 0
#     ALM_STR = 'READY'
#     ServoMonitorALM_Addr = 0x6004
#     ServoMonitorALM_Items = ['MAN', 'RES', 'DRV_STATE', 'ACTU_SPD','ACT_TORQ','ACT_CURRENT', 'ACTF_SPD', 'DC_VOLT'
#     ,'DRV_TEMPER', 12,13,14,'OVERLOAD_RATIO', 'REGENLOAD_RATIO', 17, 18, 19, 20
#     , 21,22,23, 24,25,26
#     , 27,'LOCMOT_1', 'LOCMOT_2','DEVI_ENC_1','DEVI_ENC_2',30,31]
#     # ServoMonitorALM_Items = ['ALM', 'MOTOR_FACTOR', 'DRV_STATE', 'ACTU_SPD','ACT_TORQ','ACT_CURRENT', 'ACTF_SPD', 'DC_VOLT'
#     # ,'DRV_TEMPER', 12,13,14,'OVERLOAD_RATIO', 'REGENLOAD_RATIO', 'DIN_STATUS', 'DOUT_STATUS', 19, 'LOCCMD_1'
#     # , 'LOCCMD_2','PULSE_1','PULSE_2', 'DEVI_CMD_1','DEVI_CMD_2','LOCENC_1'
#     # , 'LOCENC_2','LOCMOT_1', 'LOCMOT_2','DEVI_ENC_1','DEVI_ENC_2','POS_RORATION_1','POS_RORATION_2']
#     ServoMonitorALM_Len = len(ServoMonitorALM_Items)

# class ServoMonitorStatus(Enum):
#     NAME = 'MONITOR_STATUS'
#     STATUS_ADDR = 0x6002
#     ERR01 = 0x01
#     ERR20 = 0x20
#     ERR40 = 0x40
#     READY=0 #위치 지정이 완료되고 새 데이터를 수신 할 수 있음을 나타냄 Ready
#     MOVING = 0x100 #경로가 실행 중임을 표시 Moving
#     FINISHING = 0x200 #경로가 실행 중임을 표시 Moving
