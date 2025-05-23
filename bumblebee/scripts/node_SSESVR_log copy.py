#!/usr/bin/env python3
from flask import jsonify
from flask import Flask, render_template, request, render_template_string
from flask_socketio import SocketIO
import rospy
import threading
import time
from std_msgs.msg import String
from enum import Enum, auto
from UtilBLB import *
from flask_cors import CORS

dicWE_ON = getMotorWE_ONDic()
dicWE_OFF = getMotorWE_OFFDic()
#strNOTAG = RailNodeInfo.NOTAG.name

node_virtual_str = RailNodeInfo.NONE.name
isRealMachine = get_hostname().find(UbuntuEnv.ITX.name) >= 0
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')
shared_data = {topic.name: {} for topic in TopicName}
maxAlarmHistory = 500
maxRetryCount = 5
cmdIntervalSec = 3

rospy.init_node('node_API', anonymous=False)
move_publisher = rospy.Publisher(TopicName.CMD_DEVICE.name, String, queue_size=10)
tag_publisher = rospy.Publisher(TopicName.ARUCO_RESULT.name, String, queue_size=10)
and_publisher = rospy.Publisher(TopicName.ANDROID.name, String, queue_size=10)
alarm_publisher = rospy.Publisher(TopicName.HISTORY_ALARM.name, String, queue_size=10)
info_publisher = rospy.Publisher(TopicName.HISTORY_INFO.name, String, queue_size=10)
pub_motorPos = rospy.Publisher(TopicName.MOTOR_POS.name, String, queue_size=1)
pub_alarmstatus = rospy.Publisher(TopicName.ALARM_STATUS.name, String, queue_size=1)
pub_BLB_CMD = rospy.Publisher(TopicName.BLB_CMD.name, String, queue_size=1)
pub_BLB_CROSS = rospy.Publisher(TopicName.CROSS_INFO.name, String, queue_size=1)
pub_SMARTPLUG = rospy.Publisher(TopicName.SMARTPLUG_INFO.name, String, queue_size=1)
pub_RECEIVE_MQTT = rospy.Publisher(TopicName.RECEIVE_MQTT.name, String, queue_size=1)

target_pulse_cw = 50000000
ACC_DECC_MOTOR_H = 5000
ACC_DECC_NORMAL = 3000
findNodeTryPulse = roundPulse

dicServExpand = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value, True, target_pulse_cw, DEFAULT_RPM_SLOW, ACC_DECC_NORMAL,ACC_DECC_NORMAL)
dicServFold = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value, True,-target_pulse_cw, DEFAULT_RPM_SLOW,ACC_DECC_NORMAL,ACC_DECC_NORMAL)    
dic540CW = getMotorMoveDic(ModbusID.ROTATE_MAIN_540.value, True, target_pulse_cw, DEFAULT_RPM_SLOWER,ACC_DECC_NORMAL,ACC_DECC_NORMAL)    
dic540CCW = getMotorMoveDic(ModbusID.ROTATE_MAIN_540.value, True, -target_pulse_cw, DEFAULT_RPM_SLOWER,ACC_DECC_NORMAL,ACC_DECC_NORMAL)    
dic360CW = getMotorMoveDic(ModbusID.ROTATE_SERVE_360.value, True, target_pulse_cw, DEFAULT_RPM_SLOW,ACC_DECC_NORMAL,ACC_DECC_NORMAL)    
dic360CCW = getMotorMoveDic(ModbusID.ROTATE_SERVE_360.value, True, -target_pulse_cw, DEFAULT_RPM_SLOW,ACC_DECC_NORMAL,ACC_DECC_NORMAL)    

dicBackHome = getMotorMoveDic(ModbusID.MOTOR_H.value, True, -target_pulse_cw, DEFAULT_RPM_SLOW,ACC_DECC_NORMAL,ACC_DECC_MOTOR_H)    
dicMotorH = getMotorMoveDic(ModbusID.MOTOR_H.value, True, target_pulse_cw, DEFAULT_RPM_SLOW,ACC_DECC_NORMAL,ACC_DECC_MOTOR_H)    

dicUpHome = getMotorMoveDic(ModbusID.MOTOR_V.value, False, -target_pulse_cw, DEFAULT_RPM_MID,ACC_DECC_SMOOTH,ACC_DECC_SMOOTH)    
dicDownV = getMotorMoveDic(ModbusID.MOTOR_V.value, False, target_pulse_cw, DEFAULT_RPM_MID,ACC_DECC_SMOOTH,ACC_DECC_SMOOTH)    
dicBal1CW = getMotorMoveDic(ModbusID.BAL_ARM1.value, True, target_pulse_cw, 600,ACC_DECC_SMOOTH,ACC_DECC_SMOOTH)    
dicBal2CW = getMotorMoveDic(ModbusID.BAL_ARM2.value, True, target_pulse_cw, 420,ACC_DECC_SMOOTH,ACC_DECC_SMOOTH)    
dicBal1CCW = getMotorMoveDic(ModbusID.BAL_ARM1.value, True, -target_pulse_cw, 600,ACC_DECC_SMOOTH,ACC_DECC_SMOOTH)    
dicBal2CCW = getMotorMoveDic(ModbusID.BAL_ARM2.value, True, -target_pulse_cw, 420,ACC_DECC_SMOOTH,ACC_DECC_SMOOTH)    

dicAllExpand = [getMotorMoveDic(ModbusID.BAL_ARM2.value, True, target_pulse_cw, 1150,750,250),
                getMotorMoveDic(ModbusID.BAL_ARM1.value, True, target_pulse_cw, 1500,250,250),
                getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value, True, 400000, 253,150,2500)]

dicAllFold = [getMotorMoveDic(ModbusID.BAL_ARM2.value, True, 0, 1150,750,250),
                getMotorMoveDic(ModbusID.BAL_ARM1.value, True, 0, 1500,250,250),
                getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value, True, 0, 253,150,2500)]

dicSpdSlow = getMotorSpeedDic(ModbusID.MOTOR_H.value, True, 200, ACC_DECC_SMOOTH,ACC_DECC_MOTOR_H)
dicSpdFast = getMotorSpeedDic(ModbusID.MOTOR_H.value, True, DEFAULT_RPM_NORMAL, ACC_DECC_MOTOR_H,ACC_DECC_MOTOR_H)
        
dicArmExpand = dicAllExpand[:-1]
dicArmFold = dicAllFold[:-1]
dicMotorPos = {}
dicAlarmStatus = {}
dicAlarmCode = {}
startPos = 0
endPos = None
endnode= 0
epcTotalView = pd.DataFrame()
dfNodeInfo = pd.read_csv(strFileEPC_total, sep=sDivTab)
dicPotInfo = getDic_FromFile(filePath_CaliPotConfig)
pot_telesrv = dicPotInfo.get(str(ModbusID.TELE_SERV_MAIN.value))
pot_arm1 = dicPotInfo.get(str(ModbusID.BAL_ARM1.value))
pot_arm2 = dicPotInfo.get(str(ModbusID.BAL_ARM2.value))
pot_540 = dicPotInfo.get(str(ModbusID.ROTATE_MAIN_540.value))
pot_360 = dicPotInfo.get(str(ModbusID.ROTATE_SERVE_360.value))

lastNode = None
lastRSSI = None
lastPos = None
lastAck = ''
isScan = False
isRetry = False
lastcalledAck = DATETIME_OLD
dicLastMotor15 = {}
dicLastMotor11 = {}
dicPulsePos = {}
topicName_ServoPrefix = 'MB_'
topicName_MotorH = f'{topicName_ServoPrefix}{ModbusID.MOTOR_H.value}'
topicName_ServArm = f'{topicName_ServoPrefix}{ModbusID.TELE_SERV_MAIN.value}'
topicName_BAL1 = f'{topicName_ServoPrefix}{ModbusID.BAL_ARM1.value}'
topicName_BAL2 = f'{topicName_ServoPrefix}{ModbusID.BAL_ARM2.value}'
topicName_LiftV = f'{topicName_ServoPrefix}{ModbusID.MOTOR_V.value}'
topicName_RotateTray = f'{topicName_ServoPrefix}{ModbusID.ROTATE_SERVE_360.value}'
topicName_RotateMain = f'{topicName_ServoPrefix}{ModbusID.ROTATE_MAIN_540.value}'
topicName_TOF = f'{topicName_ServoPrefix}{ModbusID.TOF.value}'

lsServoTopics = [topicName_MotorH,topicName_ServArm,topicName_BAL1,topicName_BAL2,topicName_LiftV,topicName_RotateTray,topicName_RotateMain]
#암 속도 조절.
adjustrate = SPEED_RATE_ARM

#6번 리프트 모터
ACC_LIFT_UP = getSpeedTableInfo(ModbusID.MOTOR_V.value,SPEEDTABLE_FIELDS.ACC_CCW.name)
ACC_LIFT_DOWN = getSpeedTableInfo(ModbusID.MOTOR_V.value,SPEEDTABLE_FIELDS.ACC_CW.name)
DECC_LIFT_UP = getSpeedTableInfo(ModbusID.MOTOR_V.value,SPEEDTABLE_FIELDS.DECC_CCW.name)
DECC_LIFT_DOWN = getSpeedTableInfo(ModbusID.MOTOR_V.value,SPEEDTABLE_FIELDS.DECC_CW.name)
SPD_LIFT = getSpeedTableInfo(ModbusID.MOTOR_V.value,SPEEDTABLE_FIELDS.SPD.name)

#10번 2관절 모터
SPD_ARM2 = getSpeedTableInfo(ModbusID.BAL_ARM2.value,SPEEDTABLE_FIELDS.SPD.name,adjustrate)
ACC_ARM2_EXTEND = getSpeedTableInfo(ModbusID.BAL_ARM2.value,SPEEDTABLE_FIELDS.ACC_CW.name,adjustrate)
ACC_ARM2_FOLD = getSpeedTableInfo(ModbusID.BAL_ARM2.value,SPEEDTABLE_FIELDS.ACC_CCW.name,adjustrate)
DECC_ARM2_EXTEND = getSpeedTableInfo(ModbusID.BAL_ARM2.value,SPEEDTABLE_FIELDS.DECC_CW.name,adjustrate)
DECC_ARM2_FOLD = getSpeedTableInfo(ModbusID.BAL_ARM2.value,SPEEDTABLE_FIELDS.DECC_CCW.name,adjustrate)

#11번 서빙 텔레스코픽 모터
ACC_ST_EXTEND = getSpeedTableInfo(ModbusID.TELE_SERV_MAIN.value,SPEEDTABLE_FIELDS.ACC_CW.name)
DECC_ST_EXTEND = getSpeedTableInfo(ModbusID.TELE_SERV_MAIN.value,SPEEDTABLE_FIELDS.DECC_CW.name,adjustrate)
ACC_ST_FOLD = getSpeedTableInfo(ModbusID.TELE_SERV_MAIN.value,SPEEDTABLE_FIELDS.ACC_CCW.name,adjustrate)
DECC_ST_FOLD = getSpeedTableInfo(ModbusID.TELE_SERV_MAIN.value,SPEEDTABLE_FIELDS.DECC_CCW.name)

#13번 1관절 모터
SPD_ARM1 = getSpeedTableInfo(ModbusID.BAL_ARM1.value,SPEEDTABLE_FIELDS.SPD.name,adjustrate)
ACC_ARM1_EXTEND = getSpeedTableInfo(ModbusID.BAL_ARM1.value,SPEEDTABLE_FIELDS.ACC_CW.name,adjustrate)
ACC_ARM1_FOLD = getSpeedTableInfo(ModbusID.BAL_ARM1.value,SPEEDTABLE_FIELDS.ACC_CCW.name,adjustrate)
DECC_ARM1_EXTEND = getSpeedTableInfo(ModbusID.BAL_ARM1.value,SPEEDTABLE_FIELDS.DECC_CW.name,adjustrate)
DECC_ARM1_FOLD = getSpeedTableInfo(ModbusID.BAL_ARM1.value,SPEEDTABLE_FIELDS.DECC_CCW.name,adjustrate)

#15번 주행 모터
ACC_MOVE_H = getSpeedTableInfo(ModbusID.MOTOR_H.value,SPEEDTABLE_FIELDS.ACC_CCW.name,SPEED_RATE_H)
DECC_MOVE_H = getSpeedTableInfo(ModbusID.MOTOR_H.value,SPEEDTABLE_FIELDS.ACC_CW.name,SPEED_RATE_H)
SPD_MOVE_H = getSpeedTableInfo(ModbusID.MOTOR_H.value,SPEEDTABLE_FIELDS.SPD.name,SPEED_RATE_H)

#27번 메인회전 모터
SPD_540 =  getSpeedTableInfo(ModbusID.ROTATE_MAIN_540.value,SPEEDTABLE_FIELDS.SPD.name)
ACC_540 = getSpeedTableInfo(ModbusID.ROTATE_MAIN_540.value,SPEEDTABLE_FIELDS.ACC_CW.name)
DECC_540 = getSpeedTableInfo(ModbusID.ROTATE_MAIN_540.value,SPEEDTABLE_FIELDS.DECC_CW.name)

#31번 트레이 모터
SPD_360 =  getSpeedTableInfo(ModbusID.ROTATE_SERVE_360.value,SPEEDTABLE_FIELDS.SPD.name)
ACC_360_DOWN = getSpeedTableInfo(ModbusID.ROTATE_SERVE_360.value,SPEEDTABLE_FIELDS.ACC_CW.name)
DECC_360_DOWN = getSpeedTableInfo(ModbusID.ROTATE_SERVE_360.value,SPEEDTABLE_FIELDS.DECC_CW.name)
ACC_360_UP = getSpeedTableInfo(ModbusID.ROTATE_SERVE_360.value,SPEEDTABLE_FIELDS.ACC_CCW.name)
DECC_360_UP = getSpeedTableInfo(ModbusID.ROTATE_SERVE_360.value,SPEEDTABLE_FIELDS.DECC_CCW.name)

def GetTofDistance():
    dicTOF = shared_data.get(topicName_TOF,{})
    return dicTOF.get(SeqMapField.DISTANCE.name,-1)

def GetDoorStatus():
    global shared_data    
    dicARD_CARRIER = shared_data.get(TopicName.ARD_CARRIER.name)
    doorStatusClose1 = dicARD_CARRIER.get(
        CARRIER_STATUS.I_DOOR_1_BOTTOM.name, -1
    )
    doorStatusOpen1 = dicARD_CARRIER.get(
        CARRIER_STATUS.I_DOOR_1_TOP.name, -2
    )
    doorStatusClose2 = dicARD_CARRIER.get(
        CARRIER_STATUS.I_DOOR_2_BOTTOM.name, -1
    )
    doorStatusOpen2 = dicARD_CARRIER.get(
        CARRIER_STATUS.I_DOOR_2_TOP.name, -2
    )
    isOpen1 = isTrue(doorStatusOpen1)
    isClose1 = isTrue(doorStatusClose1)
    isOpen2 = isTrue(doorStatusOpen2)
    isClose2 = isTrue(doorStatusClose2)
    resultReturn = TRAYDOOR_STATUS.MOVING
    resultArray = [isClose1,isClose2]
    if (isOpen1 and isClose1) or (isOpen2 and isClose2):
        resultReturn = TRAYDOOR_STATUS.DOORALARM
    elif isOpen1 or isOpen2:
        resultReturn= TRAYDOOR_STATUS.OPENED
    elif isClose1 and isClose2:
        resultReturn= TRAYDOOR_STATUS.CLOSED
    return resultReturn,resultArray

def GetRotateMainAngleFromPulse(target_pulse):
  angle = mapRange(target_pulse,0,pot_540,0,MAX_ANGLE_TRAY)
  return round(angle)%MAX_ANGLE_TRAY

def GetRotateTrayAngleFromPulse(target_pulse):
  angle = mapRange(target_pulse,0,pot_360,0,MAX_ANGLE_TRAY)
  return round(angle)%MAX_ANGLE_TRAY

def GetRotateTrayPulseFromAngle(angleStr):
  cur_360 = int(dicMotorPos[topicName_RotateTray])
  angle = (360+strToRoundedInt(angleStr)) % 360
  target_pulse_org = round(mapRange(angle,0,MAX_ANGLE_TRAY,0,pot_360) )
  target_pulse_cw = target_pulse_org + pot_360
  target_pulse_ccw = target_pulse_org - pot_360
  arr = [target_pulse_org,target_pulse_cw,target_pulse_ccw]
  return find_closest_value(arr,cur_360)

def GetRotateMainPulseFromAngle(angleStr):
  cur_540 = int(dicMotorPos[topicName_RotateMain])
  angle = (360 + strToRoundedInt(angleStr)) % 360
  target_pulse_org = round(mapRange(angle,0,MAX_ANGLE_BLBBODY,0,pot_540))
  target_pulse_cw = target_pulse_org + pot_540
  target_pulse_ccw = target_pulse_org - pot_540
  arr = [target_pulse_org,target_pulse_cw,target_pulse_ccw]
  return find_closest_value(arr,cur_540)

def onScaning():
    dicAndroid = GetAndroidInfoDic()
    angle_y = dicAndroid.get(DataKey.Angle_Y.name)
    onScan = False
    if angle_y is not None and is_between(-3,3,angle_y):
        onScan = True
    return onScan    

def GetControlInfoBalances(target_angle,bUseCurrentPosition=False, spd_rate = 1.0):
    targetPulse_arm1=mapRange(target_angle,0,90,0,pot_arm1)    
    if bUseCurrentPosition:
        cur_arm1 = int(dicMotorPos[topicName_BAL1])
        cur_arm2 =int(dicMotorPos[topicName_BAL2])
    else:
        cur_arm1=0
        cur_arm2=0

    stroke_arm1_signed = targetPulse_arm1 - cur_arm1
    stroke_arm1_abs = abs(stroke_arm1_signed)
    targetPulse_arm2 = min(round(mapRange(targetPulse_arm1,0,pot_arm1,0,pot_arm2)),pot_arm2)
    stroke_arm2_signed = targetPulse_arm2 - cur_arm2
    stroke_arm2_abs = abs(stroke_arm2_signed)
    spd_arm1 = SPD_ARM1
    if onScaning():
        spd_arm1 = round(SPD_ARM1/4)

    #1관절 스트로크 RPM
    rpm_arm1 = round(spd_arm1 * spd_rate)
    round1CountAbs = round(stroke_arm1_abs/roundPulse)
    round2CountAbs = round(stroke_arm2_abs/roundPulse)
    rpm_time_arm1 = calculate_rpm_time(round1CountAbs, rpm_arm1)
    rpm_arm2 = max(calculate_targetRPM_fromtime(round2CountAbs, rpm_time_arm1),DEFAULT_RPM_SLOWER)
    isExpand = stroke_arm1_signed > 0
    accArm2 = ACC_ARM2_FOLD
    deccArm2 = DECC_ARM2_FOLD
    accArm1 = ACC_ARM1_FOLD
    deccArm1 = DECC_ARM1_FOLD
    if isExpand:
        if onScaning():
            accArm2 = round(ACC_ARM2_EXTEND/2)
            deccArm2 = round(DECC_ARM2_EXTEND/2)
            accArm1 = round(ACC_ARM1_EXTEND/2)
            deccArm1 = round(DECC_ARM1_EXTEND/2)
        else:
            accArm2 = ACC_ARM2_EXTEND
            deccArm2 = DECC_ARM2_EXTEND
            accArm1 = ACC_ARM1_EXTEND
            deccArm1 = DECC_ARM1_EXTEND
            
    dicArm1 = getMotorMoveDic(ModbusID.BAL_ARM1.value,True,targetPulse_arm1,rpm_arm1,accArm1,deccArm1)
    dicArm2 = getMotorMoveDic(ModbusID.BAL_ARM2.value,True,targetPulse_arm2,rpm_arm2,accArm2,deccArm2)
    return [dicArm1,dicArm2], rpm_time_arm1,stroke_arm1_signed

def GetControlInfoArms(distanceServingTeleTotal,bUseCurrentPosition = False, spd_rate = 1.0):
    #bUseCurrentPosition 에 따라 RPM 이 달라진다.
    # True 인 경우 현재 포지션 기준으로 RPM 계산.
    # False 인 경우 풀스트로크 기준으로 RPM 계산.
    # STROKE_MAX 상수를 활용하여 밸런스 암 RPM 을 계산하자.
    if bUseCurrentPosition:
        cur_serv = int(dicMotorPos[topicName_ServArm])
    else:
        cur_serv = 0
    
    #서빙암 타겟 펄스    
    targetPulse_serv = GetTargetPulseServingArm(distanceServingTeleTotal, 0)    
    #현재 위치에서 서빙암이 뻗어야 할 스트로크 계산
    stroke_servArm_signed = targetPulse_serv - cur_serv
    stroke_servArm_abs = abs(stroke_servArm_signed)
    isCaliMode = False
    if stroke_servArm_signed < 0 and distanceServingTeleTotal > 100:
        callbackMB_11.last_target_mm = distanceServingTeleTotal
        targetPulse_serv = 0
        distanceServingTeleTotal = 0
        isCaliMode = True
        spd_rate = 0.3
    
    targetAngle_arm1 = calculate_weight_angle(distanceServingTeleTotal,1300,650,90)
    #서빙암 스트로크 RPM
    lsBal, rpm_time_arm1,stroke_arm1_signed= GetControlInfoBalances(targetAngle_arm1,bUseCurrentPosition,spd_rate)
    servArmCountAbs = round(stroke_servArm_abs/roundPulse)
    if abs(stroke_arm1_signed) > roundPulse:
        rpm_servArm = max(calculate_targetRPM_fromtime(servArmCountAbs, rpm_time_arm1),DEFAULT_RPM_SLOWER)
    else:
        rpm_servArm = round(DEFAULT_RPM_SLOW * spd_rate)
    acc_st =ACC_ST_EXTEND
    decc_st = DECC_ST_EXTEND
    if stroke_servArm_signed < 0:
        acc_st =ACC_ST_FOLD
        decc_st = DECC_ST_FOLD
        
    dicSrvArm = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value,True,targetPulse_serv,rpm_servArm,acc_st,decc_st)
    lsBal.append(dicSrvArm)
    rospy.loginfo(json.dumps(lsBal, indent=4))
    return lsBal,rpm_time_arm1

def getChargerPlugStatus():
    global shared_data    
    dicBLB_Status = shared_data.get(TopicName.SMARTPLUG_INFO.name)
    return isTrue(dicBLB_Status.get(SMARTPLUG_INFO.CHARGERPLUG_STATE.name))

def GetAndroidInfoDic():
    global shared_data    
    dicBLB_Status = shared_data.get(TopicName.ANDROID.name)
    return dicBLB_Status

def GetCurrentNode():
    global shared_data    
    dicBLB_Status = shared_data.get(TopicName.BLB_STATUS.name)
    return try_parse_int(dicBLB_Status.get(BLB_STATUS.NODE_CURRENT.name))

def GetMotorPos(topicName):
    cur_pos = try_parse_int(dicMotorPos.get(topicName))
    return cur_pos

def GetMotorHPos():
    return GetMotorPos(topicName_MotorH)

def GetAllMotorPos():
    pos_BAL1 = GetMotorPos(topicName_BAL1)
    pos_BAL2 = GetMotorPos(topicName_BAL2)
    pos_LiftV = GetMotorPos(topicName_LiftV)
    pos_540 = GetMotorPos(topicName_RotateMain)
    pos_360 = GetMotorPos(topicName_RotateTray)
    pos_ServArm = GetMotorPos(topicName_ServArm)
    pos_MotorH = GetMotorHPos()
    return pos_BAL1,pos_BAL2,pos_LiftV,pos_540,pos_360,pos_ServArm,pos_MotorH

def GetAllMotorPosDic():
    filtered_dict = {
    k: v for k, v in dicMotorPos.items()
    if isinstance(k, str) and k.count('_') == 1
    }
    dictPos = {}
    for k,v in filtered_dict.items():
        target_mbid = k.split('_')[1]
        target_pos = try_parse_int(v)
        dictPos[target_mbid] = target_pos    
    return dictPos

def CheckSafetyMotorMove(listReturnTmp):
    #현재 모터 위치 정보를 모은다.
    #회전모터 계열은 각도 정보를 받아온다.
    '''
    0. SSE 를 타고 가는 모든 명령어에 대해 필터링 - 현재 포지션은 dicPos 에서 확인, 필터는 SendCmd_Device 호출전에 수행.
    1. 27은 모두 0 일때 움직일 수 있으며 20rpm 이하일때는 11,13과 10이 벌어져있어도 가능.
    2. 31은 TOF 거리가 200 이상 나와야 회전 가능.
    3. 주행은 27이 0도, 180도 일때만 이동 가능.
    4. 도어는 150000펄스 이후에 가능.    
    '''
    global shared_data    
    strCheckResult = AlarmCodeList.OK.name
    pos_BAL1,pos_BAL2,pos_LiftV,pos_540,pos_360,pos_ServArm,pos_MotorH = GetAllMotorPos()
    dic_tof = shared_data.get(topicName_TOF,{})
    lsCheckArms = [pos_BAL1,pos_BAL2,pos_ServArm]
    distance_tof = dic_tof.get(SeqMapField.DISTANCE.name,-1)
    cur_angle540=abs(GetRotateMainAngleFromPulse(pos_540))
    cur_angle360=GetRotateTrayAngleFromPulse(pos_360)
    isTrayOrigin = True if cur_angle360 == 0 else False
    isLiftOrigin = True if pos_LiftV < roundPulse else False
    bSafetyFalseArms = any(val > roundPulse for val in lsCheckArms)
    df = pd.DataFrame(listReturnTmp)
    ls6 = df.loc[(df[mbidStr] == str(ModbusID.MOTOR_V.value)) & (df[cmdStr] == wmoveStr)].to_dict(orient="records")
    ls11 = df.loc[(df[mbidStr] == str(ModbusID.TELE_SERV_MAIN.value)) & (df[cmdStr] == wmoveStr)].to_dict(orient="records")
    ls31 = df.loc[(df[mbidStr] == str(ModbusID.ROTATE_SERVE_360.value)) & (df[cmdStr] == wmoveStr)].to_dict(orient="records")
    ls27 = df.loc[(df[mbidStr] == str(ModbusID.ROTATE_MAIN_540.value)) & (df[cmdStr] == wmoveStr)].to_dict(orient="records")
    ls15 = df.loc[(df[mbidStr] == str(ModbusID.MOTOR_H.value)) & (df[cmdStr] == wmoveStr)].to_dict(orient="records")
    bSafety = True
    doorStatus,doorArray = GetDoorStatus() #TRAYDOOR_STATUS.OPENED
    
    if len(ls6) > 0:
        #도어가 열린 상태에서는 상승하지 않는다
        dic6 = ls6[0]
        target_pos6 = try_parse_int(dic6.get(posStr),MIN_INT)        
        if abs(target_pos6-pos_LiftV) > roundPulse/10:                
            isUp = target_pos6 < pos_LiftV
            if isUp and target_pos6 < roundPulse * 10 and doorStatus == TRAYDOOR_STATUS.OPENED:
                bSafety = False
                strCheckResult = ALM_User.SAFETY_DOOR_LIFT.value
    if len(ls11) > 0:
        # 암 전개 수축시에는 6번 31번이 원점이어야 함.
        if not isTrayOrigin or not isLiftOrigin:
            bSafety = False
            strCheckResult = ALM_User.SAFETY_ARM.value
    if len(ls31) > 0:
        #31은 TOF 거리가 200 이상 나와야 회전 가능. - 알람코드로 리턴하자.
        #일단 TOF 에서 10만 펄스로 변경함-_-
        dic31 = ls31[0]
        target_pos31 = try_parse_int(dic31.get(posStr),MIN_INT)
        target_angle360=GetRotateTrayAngleFromPulse(target_pos31)
        
        if pos_ServArm < roundPulse*10 and target_angle360 != cur_angle360:
            bSafety = False
            strCheckResult = ALM_User.TRAY360_SAFETY.value
    if len(ls15) > 0:
        #주행은 27이 0도, 180도 일때만 이동 가능. + 모든암이 접혀져있어야 함.
        if not (cur_angle540 == 180 or cur_angle540 == 0):
            bSafety = False
            strCheckResult = ALM_User.SAFETY_MOTORH_2.value
        elif bSafetyFalseArms or not isLiftOrigin or not isTrayOrigin:
            bSafety = False
            strCheckResult = ALM_User.SAFETY_MOTORH_ROTATEMAIN.value
            
    if len(ls27) > 0:
        dic27 = ls27[0]
        spd_27 = try_parse_int(dic27.get(MotorWMOVEParams.SPD.name),MIN_INT)
        #27은 모두 0 일때 움직일 수 있으며 20rpm 이하일때는 11,13과 10이 벌어져있어도 가능.
        if not isLiftOrigin:
            bSafety = False
            strCheckResult = ALM_User.SAFETY_MOTORH_ROTATEMAIN.value
        elif spd_27 > MAINROTATE_RPM_SLOWEST and bSafetyFalseArms:
            bSafety = False
            strCheckResult = ALM_User.SAFETY_MOTORH_ROTATEMAIN.value
    return bSafety,strCheckResult

def GetMotorH_SI_POT():
    cur_pos = shared_data.get(topicName_MotorH,{})
    si_pot = cur_pos.get(sSI_POT,"")
    return si_pot

def GetMotorSensor(topicName = topicName_MotorH):
    curDicMotor = shared_data.get(topicName,{})
    si_pot = curDicMotor.get(sSI_POT,"")
    si_home = curDicMotor.get(sSI_HOME,"")
    di_pot = curDicMotor.get(sDI_POT,"")
    di_not = curDicMotor.get(sDI_NOT,"")
    di_home = curDicMotor.get(sDI_HOME,"")
    di_estop = curDicMotor.get(sDI_ESTOP,"")
    return di_pot,di_not,di_home,di_estop, si_pot,si_home

def GetSpeicificEPCNodeInfo():
    local_epcnodeinfo = [GetNodeFromTable(HOME_CHARGE),GetNodeFromTable(HOME_TABLE)]
    state_keys: list[str] = list(GetCrossInfo().keys())
    local_epcnodeinfo.extend(state_keys)
    return local_epcnodeinfo

def handle_charge(loaded_data = {}):
        for k,v in ip_dict.items():
            if 'PLUG' in k and 'IP' in k and isRealMachine:
                info = get_tasmota_info(v)
                if info is None:
                    continue
                kDevice = k[:-3]
                kDevice = kDevice[4:]
                for k2,v2 in info.items():
                    loaded_data[f'{kDevice}_{k2}'] = v2
                state = get_tasmota_state(v)
                if state is None:
                   return False 
                loaded_data[f'{kDevice}_STATE'] = state
        
        data_out = json.dumps(loaded_data)
        pub_SMARTPLUG.publish(data_out)
        return True
           

def rfidInstanceDefault():
    global lastNode
    global lastRSSI
    global lastPos
    global endnode
    global startPos
    global endPos
    global lastcalledAck
    global isScan
    global isRetry
    global lastAck
    global dicPulsePos
    dicPulsePos.clear()
    lastNode = None
    lastRSSI = None
    lastPos = None
    lastAck = ''
    isScan = False
    isRetry = False
    lastcalledAck =DATETIME_OLD
    endPos = None
    callbackACK.retryCount = 100
    log_all_frames()

def SavePos():
    resultArd = service_setbool_client_common(ServiceBLB.CMD_SAVE.value, "enable", Kill)        
    time.sleep(MODBUS_WRITE_DELAY)        
    return resultArd

def SendAlarmClear(enable):
    resultArd = service_setbool_client_common(ServiceBLB.CMD_ALMC.value, enable, Kill)        
    time.sleep(MODBUS_WRITE_DELAY)        
    return resultArd

def PublishStateMessage(tmpState):
    data_out = {BLB_CMD.STATE.name : tmpState,
                BLB_CMD.ID.name:get_hostname(),
                BLB_CMD.TRAY_A.name : 0,
                BLB_CMD.TRAY_B.name : 0,
                BLB_CMD.LEVEL.name : 0
                }
    sendbuf = json.dumps(data_out)
    pub_BLB_CMD.publish(sendbuf)

def StopMotor(mbid, decc = EMERGENCY_DECC):
    lsStopMotorsCmd = []
    if isinstance(mbid, list):
        for mbidTmp in mbid:
            lsStopMotorsCmd.append(getMotorStopDic(mbidTmp.value, decc))
    else:
        sendInit = getMotorStopDic(mbid, decc)
        lsStopMotorsCmd.append(sendInit)
    SendCMD_Device(lsStopMotorsCmd)
    tof_distance=GetTofDistance()
    log_all_frames(f"Trying to stop Motor {mbid},DECC:{decc} at distance:{tof_distance}")
    
def SendCMDESTOP(enable,isAlarm=True):
    rfidInstanceDefault()
    if isAlarm:
        PublishStateMessage(BLD_PROFILE_CMD.ESTOP.name)
    resultArd = service_setbool_client_common(ServiceBLB.CMD_ESTOP.value, enable, Kill)        
    if isRealMachine and isAlarm:
        API_CROSS_stop()
    time.sleep(MODBUS_WRITE_DELAY)        
    return resultArd

def SendCMDArd(sCmdArd,isSafetyCheck=True):
    resultArd=True
    strMsg = AlarmCodeList.OK.name
    pos_BAL1,pos_BAL2,pos_LiftV,pos_540,pos_360,pos_ServArm,pos_MotorH = GetAllMotorPos()
    splitTmp = sCmdArd.split(sDivFieldColon)
    if len(splitTmp) > 1:
        sCmd = splitTmp[0]
        sParam = splitTmp[1]
        #트레이 세이프티 - 도어(O명령) 상승시(2-상승) 리프트 모터 거리가 10만펄스 이상 확보되지 않으면 
        #도어를 열지 않는다
        if sCmd == 'O' and sParam.startswith('2') and pos_LiftV < roundPulse*10:
            resultArd = False
            strMsg = ALM_User.SAFETY_TRAYDOOR.value
        elif isRealMachine:
            resultArd = service_setbool_client_common(ServiceBLB.CMDARD_QBI.value, sCmdArd, Kill)
        else:
            sCmdArd = replace_string(sCmdArd)
            resultArd = service_setbool_client_common(ServiceBLB.CMDARD_ITX.value, sCmdArd, Kill)        
    else:
        resultArd = False
        strMsg = ALM_User.CMD_FORMAT_INVALID.value
    time.sleep(MODBUS_WRITE_DELAY)        
    return resultArd,strMsg

def SendCMD_Device(sendbuf, cmdIntervalSec=0.01,isCheckSafety=True):
    if not hasattr(SendCMD_Device, "last_cmd_time"):
        SendCMD_Device.last_cmd_time = 0
    if not hasattr(SendCMD_Device, "last_cmd_msg"):
        SendCMD_Device.last_cmd_msg = None
    now = time.time()
    cmdTmp = sendbuf
    if isinstance(cmdTmp, list) and len(cmdTmp) > 0:
        dictPos = GetAllMotorPosDic()
        sendbuf = CheckMotorCmdValid(cmdTmp,dictPos)
        if len(sendbuf) > 0:
            if isCheckSafety:
                bResultSafety,strMsg=CheckSafetyMotorMove(sendbuf)
                if not bResultSafety:
                    rospy.loginfo(strMsg)
                    return False,strMsg
            cmdTmp = json.dumps(sendbuf)
            if cmdTmp == SendCMD_Device.last_cmd_msg:
                # 같은 메시지면 쿨타임 검사
                if now - SendCMD_Device.last_cmd_time < cmdIntervalSec:
                    strMsg = ALM_User.CMD_INTERVAL_DUPLICATED.value
                    rospy.loginfo(strMsg)
                    return False,strMsg
            SendCMD_Device.last_cmd_time = now
            SendCMD_Device.last_cmd_msg = cmdTmp
        else:
            strMsg = ALM_User.ALREADY_FINISHED_CMD_POS.value
            rospy.loginfo(strMsg)
            return True,strMsg
    else:
        strMsg = ALM_User.CMD_FORMAT_INVALID.value
        rospy.loginfo(strMsg)
        return False,strMsg
        
    bExecuteResult = service_setbool_client_common(ServiceBLB.CMD_DEVICE.value, cmdTmp, Kill)
    return bExecuteResult, AlarmCodeList.OK.name

def callbackACK(recvData):
    global lastNode
    global lastRSSI
    global lastPos
    global endnode
    global startPos
    global isScan
    global lastcalledAck
    global epcTotalView
    global endPos
    
    try:
        '''
        print(dfEPCTotal)
        print(recvData)

        rfidpwr inventoryMode      time status RSSI           ip    EPC
        0    1000           off  15:04:40  ALIVE  -51  172.30.1.29  NOTAG
        1744783483.05749:0:15
        rfidpwr inventoryMode      time status RSSI           ip    EPC
        0    1000           off  15:04:40  ALIVE  -51  172.30.1.29  NOTAG
        1744783483.05749:1:15:4:-20:0:0:0:10000
        위에서 inventoryMode가 off 인 행이 있으면 RFID가 꺼진 상태이니 아무것도 하지 않음
        endnode != 0 and inventoryMode == on 이면 인식을 못하고 주행이 끝났음.
        '''
        # Colon 으로 메세지를 쪼개어 파싱한다.
        # idx 0 - timestamp, 1 - 완료여부 , 2 - MBID
        lsResult = recvData.split(sDivFieldColon)
        torque_max = -1
        torque_ave = -1
        ovr_max = -1
        ovr_ave = -1
        last_started_pos = MIN_INT
        last_targeted_pos = MIN_INT
        isHome = 0
        isPot = 0
        last_spd = 0
        mbid_tmp = lsResult[2]  # 모드버스 ID
        mbid_instance = ModbusID.from_value(mbid_tmp)
        flagFinished = lsResult[1]  # 0 이면 미완료, 1 이면 완료
        stopped_pos=GetMotorHPos()
        if not hasattr(callbackACK, "retryCount"):
                callbackACK.retryCount = maxRetryCount
        
        if not isTrue(flagFinished):
            return
        
        if len(lsResult) > 14:
            torque_max = lsResult[3]  # 정방향 최대토크
            torque_mean = lsResult[4]  # 평균토크
            torque_min = lsResult[5]  # 역방향 최대토크
            ovr_max = lsResult[6]  # 최대오버로드
            last_started_pos = int(lsResult[7])  # 운행 시작 지점
            last_targeted_pos = int(lsResult[8])  # 운행 종료 목표 지점
            stopped_pos = int(lsResult[9])  # 현재 지점
            last_spd = int(lsResult[10])  # 정지시점 속도 및 방향
            isHome = lsResult[11]  # HOME에 걸려서 멈췄으면 1
            isPot = lsResult[12]  # POT에 걸려서 멈췄으면 1
            isNot = lsResult[13]  # NOT에 걸려서 멈췄으면 1
            isESTOP = lsResult[14]  # ESTOP에 걸려서 멈췄으면 1
            
        if isTrue(flagFinished):
            # 회전모터 (27,31) 자동 상시 캘리브레이션.
            # 회전모터가 홈센서에 걸려서 멈췄을 경우 0 으로 위치값을 초기화 한다            
            rospy.loginfo(f'{mbid_tmp}-정방향토크:{torque_max},역방향토크:{torque_min},평균토크:{torque_mean},운행시작:{last_started_pos},목표지점:{last_targeted_pos},현지점:{stopped_pos},속도:{last_spd},isHome:{isHome},isNot:{isNot},isPot:{isPot},isESTOP:{isESTOP}')
            if mbid_instance in lsRotateMotors and isTrue(isHome):
                lsCmdCaliHome = [getMotorWHOME_OFFDic(mbid_tmp),getMotorHomeDic(mbid_tmp)]
                bSafetyCheck,strMsg = SendCMD_Device(lsCmdCaliHome)
                rospy.loginfo(f'Cali {mbid_tmp} Result:{bSafetyCheck},{strMsg}')
            #POT 예정 펄스보다 5바퀴 이상 차이나면 모든 모터를 중지하고 알람을 날린다
            elif mbid_instance in lsReleaseMotors:
                if isTrue(isPot) or isTrue(isNot):
                    diffPos = abs(last_targeted_pos - stopped_pos)
                    if diffPos > roundPulse*5:
                        SendCMDESTOP(f'I{ACC_DECC_SMOOTH}',True)
                        rospy.loginfo(f'ESTOP triggered at {mbid_tmp} too much diff pos {diffPos}')
        
        if not is_equal(mbid_tmp,ModbusID.MOTOR_H.value):
            return
        
        dicCurNodeInfo=GetNodeDicFromPos(dfNodeInfo,stopped_pos,isTrue(isPot))        
        curNodeID_fromPulse = dicCurNodeInfo.get(TableInfo.NODE_ID.name)
        curNode_type = str(dicCurNodeInfo.get(RFID_RESULT.EPC.name))
        rospy.loginfo(f'운행완료.지시노드:{endnode},직전노드:{lastNode},현재노드:{curNodeID_fromPulse}:{curNode_type},Retried:{callbackACK.retryCount},POT:{isPot}')
        rospy.loginfo(f'정방토크:{torque_max},역방토크:{torque_min},평균토크:{torque_ave},최대부하:{ovr_max},평균부하:{ovr_ave},시작위치:{last_started_pos},지시위치:{last_targeted_pos}:,정지위치:{stopped_pos},속도:{last_spd}')
        #목적지 노드 타입 확인 - 유효한 EPC이면 RFID노드, NONE 면 가상노드, NOTAG 면 도그 노드.
        #도그 노드, 혹은 특수 노드인데 도달하지 못 했으면 댐핑으로 추가 운행시도.
        if isScan or isTrue(isPot):            
            if curNodeID_fromPulse == endnode:
                endNodePos = GetNodePos_fromNode_ID(endnode)
                dicLoc = getMotorLocationSetDic(ModbusID.MOTOR_H.value, endNodePos)
                SendCMD_Device([dicWE_OFF,dicLoc,dicWE_ON])
                #TTSAndroid('위치오차를 보정합니다',1)
                if endnode == NODE_KITCHEN:
                    bResult = SetChargerPlug(True)
                    if bResult:
                        TTSAndroid('충전을 시작합니다',1)
                    else:
                        TTSAndroid('충전기를 점검해주세요',1)
            rfidInstanceDefault()
            return
        
        if curNode_type.find(strNOTAG) < 0 and curNodeID_fromPulse == endnode:
            rfidInstanceDefault()
            return
        
        diff_pos = stopped_pos - startPos
        findNodePulse = -findNodeTryPulse if diff_pos < 0 else findNodeTryPulse
        if callbackACK.retryCount < maxRetryCount and isRealMachine:
            callbackACK.retryCount += 1
            finalPos = stopped_pos + findNodePulse
            if endPos is not None:
                finalPos = endPos
                callbackACK.retryCount = 100
            dicJOG = getMotorMoveDic(ModbusID.MOTOR_H.value,True,finalPos ,DEFAULT_RPM_SLOW,ACC_DECC_MOTOR_H,ACC_DECC_MOTOR_H*2)
            rospy.loginfo(f'다시탐색{callbackACK.retryCount}:{dicJOG}')
            endPos = None
            SendCMD_Device([dicWE_ON,dicJOG],0.5,False)
        else:
            rfidInstanceDefault()
            di_pot,di_not,di_home,di_estop, si_pot,si_home=GetMotorSensor(topicName_RotateTray)
            print(f'di_pot:{di_pot},di_not:{di_not},si_pot:{si_pot},si_home:{si_home}')
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        SendAlarmHTTP(message,True,BLB_ANDROID_IP_DEFAULT)

def isCrossRailDirection():
    crossRailDirectionPos = 500000
    cross_pos = round(crossRailDirectionPos / 2)
    dic_CROSSINFO = shared_data.get(TopicName.CROSS_INFO.name)
    if dic_CROSSINFO:
        cross_pos = try_parse_int(dic_CROSSINFO.get(MonitoringField.CUR_POS.name))
        if is_between(roundPulse,500000,cross_pos):
            return None
    return False if cross_pos > roundPulse else True

def callbackMB_11(recvDataMap):
    global lastNode
    global lastPos
    global endPos
    global endnode
    global isScan
    global dicLastMotor11
    global dicPulsePos  #도그 신호가 들어올때 엔코더 값을 기록해두고 보정수치로 쓴다
    dicLastMotor11.update(recvDataMap)
    if not hasattr(callbackMB_11, "last_target_mm"):
        callbackMB_11.last_target_mm = -1
    if callbackMB_11.last_target_mm < 0:
        return    
    try:
        sSPD_signed = try_parse_int(recvDataMap.get(sSPD_Key),MIN_INT)
        if sSPD_signed == MIN_INT:
            return
        target_pos = try_parse_int(recvDataMap.get(sLAST_TARGET_POS),MIN_INT)
        if target_pos == MIN_INT:
            return
        if sSPD_signed < 0 and callbackMB_11.last_target_mm > 100:
            tof_distance=GetTofDistance()
            if is_between(callbackMB_11.last_target_mm-10,callbackMB_11.last_target_mm+20,tof_distance):
                rospy.loginfo(f'Stop at TOF : {tof_distance}, Target = {callbackMB_11.last_target_mm}')
                StopMotor(lsReleaseMotors,EMERGENCY_DECC)
                callbackMB_11.last_target_mm = -1
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        SendAlarmHTTP(message,True,BLB_ANDROID_IP_DEFAULT)


def callbackMB_15(recvDataMap):
    global lastNode
    global lastPos
    global endPos
    global endnode
    global isScan
    global dicLastMotor15
    global dicPulsePos  #도그 신호가 들어올때 엔코더 값을 기록해두고 보정수치로 쓴다
    dicLastMotor15.update(recvDataMap)
    if not hasattr(callbackMB_15, "ESTOP_ON"):
            callbackMB_15.ESTOP_ON = BLD_PROFILE_CMD.MOTORSTOP.name
    try:
        isST_CMD_FINISH = recvDataMap.get(sST_CMD_FINISH)
        if isTrue(isST_CMD_FINISH):
            return
        
        sSPD_signed = (recvDataMap.get(sSPD_Key))
        target_pos = (recvDataMap.get(sLAST_TARGET_POS))
        if sSPD_signed is None:
            return
        if target_pos is None:
            return
        sSPD_signed = int(sSPD_signed)
        target_pos = int(target_pos)
        sPOS = int(recvDataMap.get(sPOS_Key))
        di_pot_status = isTrue(recvDataMap.get(sDI_POT,""))
        last_started_pos = int(recvDataMap.get(MonitoringField.LAST_STARTED_POS.name))
        if di_pot_status:
            sLogMsg = f'도그위치:{sPOS},시작위치:{last_started_pos},목표위치:{target_pos},속도:{sSPD_signed}'
            rospy.loginfo(sLogMsg)        
            if callbackMB_15.ESTOP_ON == BLD_PROFILE_CMD.ESTOP.name:
                callbackMB_15.ESTOP_ON = BLD_PROFILE_CMD.BACK_HOME.name
                StopMotor(ModbusID.MOTOR_H.value,ACC_DECC_NORMAL)
                if sSPD_signed > 0:
                    endPos = sPOS - (sSPD_signed * 5)
                else:
                    endPos = sPOS + (sSPD_signed * 5)    # - (sSPD_signed * 20) 
        if abs(target_pos - sPOS) > roundPulse * 50:
            return
        
        si_pot = recvDataMap.get(sSI_POT,"")
        if callbackMB_15.ESTOP_ON == BLD_PROFILE_CMD.MOTORSTOP.name:
            callbackMB_15.ESTOP_ON = BLD_PROFILE_CMD.ESTOP.name            
            rospy.loginfo(f'#EStop On - 목표까지 남은펄스:{target_pos-sPOS},현재속도:{sSPD_signed},현재위치:{sPOS},목표위치:{target_pos},모드:{si_pot}')
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        SendAlarmHTTP(message,True,BLB_ANDROID_IP_DEFAULT)

def callback_factory(topic_name):
    def callback(msg):
        global shared_data
        global lastAck
        try:
            recvData = msg.data
            if is_valid_python_dict_string(recvData):
                recvDataMap = ast.literal_eval(recvData)
            elif is_json(recvData):
                recvDataMap = json.loads(recvData)
            else:
                recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
            # if topic_name == TopicName.RFID.name:
            #     callbackRFID(recvDataMap)
            if topic_name == topicName_MotorH:
                callbackMB_15(recvDataMap)
            
            if topic_name == topicName_ServArm:
                callbackMB_11(recvDataMap)
            
            if topic_name == TopicName.ACK.name:
                if lastAck != recvData:
                    lastAck = recvData
                    callbackACK(recvData)
                    
            # 기존 데이터와 새로운 데이터를 합쳐서 저장
            lsRecvDataMap = []
            if topic_name in shared_data:
                if isinstance(recvDataMap, list):
                    lsRecvDataMap.extend(recvDataMap)
                    #rospy.loginfo(f"{recvDataMap}")
                    shared_data[topic_name] = lsRecvDataMap
                else:
                    shared_data[topic_name] = recvDataMap
                    if MonitoringField.ALM_NM.name in recvDataMap:
                        dicMotorPos[topic_name] = recvDataMap.get(MonitoringField.CUR_POS.name, MIN_INT)
                        dicMotorPos[f'{topic_name}_SPD'] = recvDataMap.get(MonitoringField.CUR_SPD.name, MIN_INT)
                        dicMotorPos[f'{topic_name}_STATE'] = recvDataMap[MonitoringField.ALM_NM.name]
                        alarmCD = recvDataMap[MonitoringField.ALM_CD.name]
                        if not is_equal(alarmCD, AlarmCodeList.OK.value):
                            TTSAndroid('서보모터를 점검해주세요',10)
                        pub_motorPos.publish(json.dumps(dicMotorPos, sort_keys=True))
            else:
                shared_data[topic_name] = recvDataMap  # 최초 생성
            socketio.emit(f"update_{topic_name}", shared_data[topic_name])
        except Exception as e:
            message = traceback.format_exc()
            logmsg = f"{e}:{message} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
            rospy.loginfo(logmsg)
            logSSE_error(traceback.format_exc())
    return callback

subscribers = {}
for topic in TopicName:
    topic_name = topic.name
    subscribers[topic_name] = rospy.Subscriber(f"/{topic_name}", String, callback_factory(topic_name))

for item in ModbusID:
    subTopicName = f'{TopicName.MB_.name}{item.value}'
    subscribers[subTopicName]=rospy.Subscriber(subTopicName, String, callback_factory(subTopicName))
subscribers.pop(TopicName.MB_.name)

@app.route('/CONTROL', methods=['GET'])
def control_data():
    try:
        table_id_delete = request.args.to_dict().get('DELETE_TABLE_ID')
        table_id_add = request.args.to_dict().get(TableInfo.TABLE_ID.name, None)  
        isTableValid = try_parse_int(table_id_add,MIN_INT)      
        if table_id_delete is not None:
            df_existing = pd.read_csv(csvPathNodes, sep=sDivTab)
            df_existing = df_existing[df_existing[TableInfo.TABLE_ID.name] != table_id_delete].reset_index(drop=True)
            df_existing.to_csv(csvPathNodes, sep=sDivTab, index=False)
            return df_existing.to_json(orient="records", force_ascii=False)
        
        elif table_id_add is None:        
            if isFileExist(csvPathalarm):
                os.remove(csvPathalarm)
            if isFileExist(csvPathInfo):
                os.remove(csvPathInfo)
            if isFileExist(strCSV_NodeInfo):
                os.remove(strCSV_NodeInfo)
            if isFileExist(strPNG_NodeInfo):
                os.remove(strPNG_NodeInfo)
            
            shared_data.pop(TopicName.HISTORY_ALARM.name,None)
            shared_data.pop(TopicName.HISTORY_INFO.name,None)
            print(API_robot_node_info())
            print(API_robot_table_info())
            #return {"error": "topicname is required."}, 400
        else:
            if isTableValid == MIN_INT:
                dic_newNodeInfo = {}
                for k, v in dicMotorPos.items():
                    if try_parse_int(v,MIN_INT) != MIN_INT:
                        dic_newNodeInfo.setdefault(k, v)
                #dic_newNodeInfo['tableid'] = topic_name.upper()
                curnode = GetCurrentNode()
                dic_newNodeInfo2 = {TableInfo.TABLE_ID.name: table_id_add.upper(), **dic_newNodeInfo}
                add_or_update_row(csvPathNodes,dic_newNodeInfo2, sDivTab,TableInfo.TABLE_ID.name)
                new_csv_node = generate_node_graph_from_csv(csvPathNodes,strFileShortCutTemp)
                generate_table_info(new_csv_node,strFileTableNode)
            else:
                if isTableValid == 0:
                    df_existing = pd.read_csv(csvPathNodes, sep=sDivTab)
                    return df_existing.to_json(orient="records", force_ascii=False)
                else:
                    new_csv_node = generate_node_graph_from_csv(csvPathNodes,strFileShortCutTemp)
                    generate_table_info(new_csv_node,strFileTableNode)
                    df_existing = pd.read_csv(strFileTableNode, sep=sDivTab)
                    return df_existing.to_html(index=False), 200
    except Exception as e:
        rospy.logerr(f"데이터 처리 오류: {e}")
        logSSE_error(traceback.format_exc())
        return GetErrResponse(e)
  
    return {"message": f"{request.path},{request.args.to_dict()}"}, 200

@app.route(f'/{EndPoints.ARD.name}', methods=['GET'])
def service_ard():
    bResult = True
    bStrMsg = AlarmCodeList.OK.name
    try:
        tmpHalf = request.args.to_dict().get('q', None)
        tiltAngle = try_parse_int(request.args.to_dict().get('tilt', None),MIN_INT)
        if tiltAngle != MIN_INT:   
            ardmsg = round(mapRange(tiltAngle, minGYRO,maxGYRO,0,180))
            if tiltAngle != MIN_INT:
                bResult,bStrMsg=SendCMDArd(f'S:10,{ardmsg}')
        elif tmpHalf is None:
            bResult = False
            bStrMsg = ALM_User.CMD_ISNULL.value
        else:
            bResult,bStrMsg=SendCMDArd(tmpHalf)
    except Exception as e:
        bResult = False
        bStrMsg = str(traceback.format_exc())
        logSSE_error(traceback.format_exc())
        return {bResult:bStrMsg}, 500
    return {bResult:bStrMsg}, 200
  
@app.route('/DATA2', methods=['GET'])
def service_data2():
    resultData = ''
    try:
        topic_name = request.args.to_dict().get('topicname', None)        
        if topic_name is None:
            return {False: ALM_User.TOPICNAME_REQUIRED.value}, 400
        resultData = shared_data.get(topic_name, None)
        if resultData is None:
            return {False: ALM_User.TOPICNAME_WRONG.value}, 400
        resultData = json.dumps(resultData, ensure_ascii=False)

    except Exception as e:
        rospy.logerr(f"데이터 처리 오류: {e}")
        logSSE_error(traceback.format_exc())
        return GetErrResponse(e)
        
    return jsonify(resultData), 200
  
@app.route('/DATA1', methods=['GET'])
def service_dataExport():
    resultData = ''
    try:
        topic_name = request.args.get('topicname', None)        
        if topic_name is None:
            return {"error": "topicname is required."}, 400
        resultData = shared_data.get(topic_name, None)
        if resultData is None:
            return {"error": "wrong topicname."}, 400
        #resultData = json.dumps(resultData, ensure_ascii=False)

    except Exception as e:
        logSSE_error(traceback.format_exc())
        rospy.logerr(f"데이터 처리 오류: {e}")
        return GetErrResponse(e)
  
    return (resultData), 200

@app.route('/cross', methods=['GET'])
def service_junction():
    loaded_data=immutable_multi_dict_to_dict(request.args)
    query_str = request.query_string.decode('utf-8')
    bResult,strMsg = API_CROSS_CMD(query_str)
    return {bResult: strMsg}, 200  

service_last_cmd_time = 0
@app.route('/CROSS', methods=['GET'])
def service_cross():
    global service_last_cmd_time
    try:
        now = time.time()
        loaded_data=immutable_multi_dict_to_dict(request.args)
        data_out = json.dumps(loaded_data)
        pub_BLB_CROSS.publish(data_out)
        cur_pos =  loaded_data.get(MonitoringField.CUR_POS.name, None)
        isPOT = loaded_data.get(MonitoringField.DI_POT.name, None)
        isNOT = loaded_data.get(MonitoringField.DI_NOT.name, None)
        ipaddr = loaded_data.get(CALLBELL_FIELD.IP.name, None)
        devID = GetLastString(ipaddr, ".")
        
        if now - service_last_cmd_time > cmdIntervalSec:
            bSmartPlugAlive = handle_charge()
            if bSmartPlugAlive:
                service_last_cmd_time = now
            else:
                service_last_cmd_time = now + 60

        if cur_pos is not None:
            cur_pos = int(cur_pos)
            status = -1
            if cur_pos < roundPulse:
                status = 1
            if cur_pos > 490000:
                status = 0
            dicCross = {MQTT_FIELD.TOPIC.name : MQTT_TOPIC_VALUE.MSG_CROSS_REQUEST.value,MonitoringField.LASTSEEN.name:getCurrentTime(),
                        MQTT_FIELD.PAYLOAD.name : {devID : status}}
            data_out = json.dumps(dicCross)
            pub_RECEIVE_MQTT.publish(data_out)            
    except Exception as e:
        logSSE_error(traceback.format_exc())
        rospy.logerr(f"데이터 처리 오류: {e}")
        return GetErrResponse(e)
  
    return {"message": f"{request.method},{request.args}"}, 200  

@app.route('/CHARGE', methods=['GET'])
def service_charge():
    try:
        #rospy.loginfo(request.args)
        loaded_data=immutable_multi_dict_to_dict(request.args,'_CHARGE')
        return handle_charge(loaded_data)    
    except Exception as e:
        rospy.logerr(f"데이터 처리 오류: {traceback.format_exc()}")
        logSSE_error(traceback.format_exc())
        return GetErrResponse(e)

@app.route('/JOG', methods=['GET'])
def service_jog():
    global startPos
    global endnode
    global lastNode
    global lastRSSI
    global isScan
    listReturnTmp=[]
    try:
        reqargs = request.args.to_dict()
        endnode_tmp = request.args.get(APIBLB_FIELDS_TASK.endnode.name)
        spd = try_parse_int(request.args.get(MotorWMOVEParams.SPD.name, DEFAULT_RPM_MID),-1)
        isAbsPos = True
        if spd < 0: 
            return {"error": "spd string is not valid."}, 400
        if endnode_tmp is None:
            return {"error": "endnode is required."}, 400
        endnode = try_parse_int(endnode_tmp,MIN_INT)
        if endnode == -1:   #DF를 short_cut.txt 로 변환 저장
            new_csv_node = generate_node_graph_from_csv(csvPathNodes,strFileShortCut)
            return {"OK": "Node Saved"}, 200      
        if endnode == 0:
            dfScan = pd.read_csv(strFileEPC_scan, sep=sDivTab)
            avg_df = dfScan.groupby("NODE_ID", as_index=False)["POS"].mean()
            avg_df["POS"] = avg_df["POS"].round().astype(int)
            last_epc_df = dfScan.groupby("NODE_ID", as_index=False).last()[["NODE_ID", "EPC"]]
            result_df = pd.merge(avg_df, last_epc_df, on="NODE_ID")
            result_df.to_csv(strFileEPC_total, sep=sDivTab, index=False)
            return {"OK": "MapSaved"}, 200      
        if endnode == MIN_INT:
            isScan = True
        else:
            isScan = False
        epcnodeinfo = GetEPCNodeInfoDic()
        sEPC = get_key_by_value(epcnodeinfo, endnode)
        dicPotNot=(dicWE_OFF)        
        si_pot = GetMotorH_SI_POT()
        cur_posH = GetMotorHPos()        
        distancePulseTarget = GetNodePos_fromNode_ID(endnode)
        if distancePulseTarget is None:
            return {"ERR": "endNode not found"}, 400      
        distanceDiffSigned = distancePulseTarget-(cur_posH)
        distanceDiffAbs = abs(distanceDiffSigned)
        if distanceDiffSigned > 0:
            backlashPos = estimate_backlash_error(distanceDiffSigned)
            distancePulseTarget = distancePulseTarget - backlashPos
        dicMotorH = getMotorMoveDic(ModbusID.MOTOR_H.value,isAbsPos,distancePulseTarget,spd,ACC_DECC_MOTOR_H,ACC_DECC_MOTOR_H)
        
        if  distanceDiffAbs < roundPulse / 2 and si_pot != "ESTOP":
            dicPotNot = dicWE_ON
            callbackMB_15.ESTOP_ON = BLD_PROFILE_CMD.ESTOP.name
        else:
            callbackMB_15.ESTOP_ON = BLD_PROFILE_CMD.MOTORSTOP.name
        listReturnTmp.append(dicPotNot)
        listReturnTmp.append(dicMotorH)
        if getChargerPlugStatus():
            SetChargerPlug(False)
            time.sleep(MODBUS_EXCEPTION_DELAY)        
        lastRSSI = None
        startPos = GetMotorHPos()
        sJob = 'JOG'
        if isScan:
            sJob = 'SCAN'
        sMsg = f"{sJob} from {startPos} to :{distancePulseTarget},Node:{endnode},EPC:{sEPC}"
        rospy.loginfo(sMsg)
        time.sleep(MODBUS_EXCEPTION_DELAY*10)
        callbackACK.retryCount = 0
        bResult,bExecuteMsg=SendCMD_Device(listReturnTmp)
        return {bResult: bExecuteMsg}, 200
    except Exception as e:
        SendCMDESTOP(f'I{ACC_DECC_SMOOTH}')
        rospy.logerr(f"Invalid data:{reqargs},{e}")
        logSSE_error(traceback.format_exc())
        return GetErrResponse(e)

@app.route(f'/{ServiceBLB.CMD_DEVICE.name}', methods=['GET'])
def service_cmd_device():
    listReturn = []
    try:
        reqargs = request.args.to_dict()
        param1 = MIN_INT
        param2 = MIN_INT
        posMsg = json.dumps(dicMotorPos, sort_keys=True)
        rospy.loginfo(reqargs)
        crossplug = request.args.get(SMARTPLUG_INFO.SET_CROSSPLUG.name, None)
        chargeplug = request.args.get(SMARTPLUG_INFO.SET_CHARGERPLUG.name, None)
        lightplug = request.args.get(SMARTPLUG_INFO.SET_LIGHTPLUG.name, None)
        filter_rate = try_parse_float(request.args.get(JogControl.FILTER_RATE.name, None),3.0)
        checkSafety = isTrue(request.args.get(JogControl.SAFETY.name, 1))
        spd_rate = try_parse_float(request.args.get(JogControl.SPD_RATE.name, None),1.0)
        control540 = try_parse_int(request.args.get(JogControl.CONTROL_ROTATE_MAIN.name, None),MIN_INT)
        control360 = try_parse_int(request.args.get(JogControl.CONTROL_ROTATE_TRAY.name, None),MIN_INT)
        controlArm3 = try_parse_int(request.args.get(JogControl.CONTROL_3ARMS.name, None),MIN_INT)
        controlArm2 = try_parse_int(request.args.get(JogControl.CONTROL_2ARMS_ANGLE.name, None),MIN_INT)
        qNumber = request.args.get('q', MIN_INT)
        recvData = request.args.get('data')
        recvDF = request.args.get('DF')
        topicData = request.args.get('topicname')
        bResult = True
        bExecuteMsg = AlarmCodeList.OK.name
        pos_BAL1,pos_BAL2,pos_LiftV,pos_540,pos_360,pos_ServArm,pos_MotorH = GetAllMotorPos()
        if recvDF is not None:
            # URL 디코딩 및 JSON 파싱
            decoded = urllib.parse.unquote(recvDF)
            records = json.loads(decoded)
            df = pd.DataFrame(records)
            lsCmd=(df.to_dict(orient='records'))
            bResult,bExecuteMsg=SendCMD_Device(lsCmd,filter_rate,checkSafety)
            reponseCode = 200 if bResult else 400
            return {bResult:bExecuteMsg}, 200
        if control360 != MIN_INT:
            cur_angle360=GetRotateTrayAngleFromPulse(pos_360)
            if control360 == cur_angle360:
                return {bResult:bExecuteMsg}, 200            
            targetPulse_serv = GetRotateTrayPulseFromAngle(control360)
            dic360 = getMotorMoveDic(ModbusID.ROTATE_SERVE_360.value,True,targetPulse_serv,SPD_360,ACC_360_UP,DECC_360_UP)
            lsCmd = [dic360]
            bResult,bExecuteMsg=SendCMD_Device(lsCmd,filter_rate,checkSafety)
            reponseCode = 200 if bResult else 400
            return {bResult:bExecuteMsg}, 200        
        elif control540 != MIN_INT:            
            cur_angle540=abs(GetRotateMainAngleFromPulse(pos_540))
            if control540 == cur_angle540:
                return {bResult:bExecuteMsg}, 200
            targetPulse_serv = GetRotateMainPulseFromAngle(control540)
            dic540 = getMotorMoveDic(ModbusID.ROTATE_MAIN_540.value,True,targetPulse_serv,SPD_540,ACC_540,DECC_540)
            lsCmd = [dic540]
            di_pot,di_not,di_home,di_estop, si_pot,si_home=GetMotorSensor(topicName_RotateMain)
            bResult,bExecuteMsg=SendCMD_Device(lsCmd,filter_rate,checkSafety)
            reponseCode = 200 if bResult else 400
            return {bResult:bExecuteMsg}, 200        
        elif controlArm3 != MIN_INT:
            #controlArm3 값은 길이로 넘어온다.
            targetPulse_serv = GetTargetPulseServingArm(controlArm3, 0)
            #POT기반한 서빙가능한 최대길이 계산
            limit_arm1_mm = GetTargetLengthMMServingArm(pot_telesrv)
            pos_ServArm_mm = GetTargetLengthMMServingArm(pos_ServArm)
            if targetPulse_serv == pos_ServArm_mm:
                return {bResult:bExecuteMsg}, 200
            #서빙암 요청길이가 물리적 최대길이를 넘어가면 실행하지 않는다.
            if targetPulse_serv > pot_telesrv:
                return {False:f'{ALM_User.OUT_OF_SERVING_RANGE.value} over {limit_arm1_mm}mm'}, 202
            else:
                lsCmd,rpm_time = GetControlInfoArms(controlArm3,True,spd_rate)
                bResult,bExecuteMsg=SendCMD_Device(lsCmd,filter_rate,checkSafety)
                reponseCode = 200 if bResult else 400
            return {bResult:bExecuteMsg}, 200
        
        elif controlArm2 != MIN_INT:
            #controlArm2 은 0~90 사이 각도로 넘어온다.
            currentAngle_arm1 =mapRange(pos_BAL1,0,pot_arm1,0,90)
            if currentAngle_arm1 == controlArm2:
                return {bResult:bExecuteMsg}, 200
            lsCmd,rpm_time,stroke_arm1_signed = GetControlInfoBalances(controlArm2,True, spd_rate)
            bResult,bExecuteMsg=SendCMD_Device(lsCmd,filter_rate,checkSafety)
            reponseCode = 200 if bResult else 400
            return {bResult:bExecuteMsg}, 200        
        elif lightplug is not None:
            plug_enable = isTrue(lightplug)
            bResult = SetLightPlug(plug_enable)
            reponseCode = 200 if bResult else 400
            #return {f'Set SetLightPlug to {plug_enable} -> Result': bResult}, reponseCode
            return {bResult:plug_enable}, reponseCode
        elif chargeplug is not None:
            plug_enable = isTrue(chargeplug)
            bResult = SetChargerPlug(plug_enable)
            reponseCode = 200 if bResult else 400
            return {bResult:plug_enable}, reponseCode        
        elif crossplug is not None:
            plug_enable = isTrue(crossplug)
            bResult = SetCrossPlug(plug_enable)
            reponseCode = 200 if bResult else 400
            return {bResult:plug_enable}, reponseCode        
        if topicData is not None:
            responseData = shared_data.get(topicData)
            if topic_name == TopicName.JOB_DF.name:
                print(type(responseData))
            if responseData is None:
                return {False:posMsg}, 400
            else:
                return {f"{reqargs}": json.dumps(responseData, sort_keys=True)}, 200
        
        if qNumber == MIN_INT and recvData is None:
            return {False: posMsg}, 400
        if recvData is None:
            cmdNumber=try_parse_int(qNumber,MIN_INT)
            if cmdNumber == MIN_INT:
                params = qNumber.split(sep=sDivItemComma)
                param1 = params[1]
                qNumber = int(params[0])
            else:
                qNumber = (cmdNumber)
            if qNumber == BLD_PROFILE_CMD.ESTOP.value:
                SendCMDESTOP(f'I{ACC_DECC_SMOOTH}',True)
                return {f"{reqargs}": dicMotorPos}, 200
            elif qNumber == BLD_PROFILE_CMD.MOTORSTOP.value:
                SendCMDESTOP(f'I{ACC_DECC_SMOOTH}',False)
                return {f"{reqargs}": dicMotorPos}, 200
            elif qNumber == BLD_PROFILE_CMD.WLOC_NOT.value:
                #개발기에서는 WLOC_NOT 받으면 모든 모터들 펄스를 0으로 초기화 한다.
                #실행할때마다 모터 위치 0 으로 원복하는 시간을 아끼기 위해서.
                SendCMDESTOP(f'T{ACC_DECC_SMOOTH}',False)
                return {f"{reqargs}": dicMotorPos}, 200
            elif qNumber == BLD_PROFILE_CMD.ALM_C.value:
                SendAlarmClear(f'T{ACC_DECC_SMOOTH}')    
                return {f"{reqargs}": dicMotorPos}, 200
            elif qNumber == BLD_PROFILE_CMD.balLiftDown.value:
                listReturn.append(dicDownV)
            elif qNumber == BLD_PROFILE_CMD.balLiftUp.value:
                listReturn.append(dicUpHome)
            elif qNumber == BLD_PROFILE_CMD.bal540_45CW.value:
                listReturn.append(dic540CW)
            elif qNumber == BLD_PROFILE_CMD.bal540_ZERO.value:
                listReturn.append(dic540CCW)
            elif qNumber == BLD_PROFILE_CMD.balTray_45CW.value:
                listReturn.append(dic360CW)
            elif qNumber == BLD_PROFILE_CMD.balTray_ZERO.value:
                listReturn.append(dic360CCW)
            elif qNumber == BLD_PROFILE_CMD.SRV_EXPAND.value:
                listReturn.append(dicServExpand)
            elif qNumber == BLD_PROFILE_CMD.SRV_FOLD.value:
                listReturn.append(dicServFold)
            elif qNumber == BLD_PROFILE_CMD.MOVE_MOTOR_H.value:
                if int(param1) > 0:
                    listReturn.append(dicMotorH)
                else:
                    listReturn.append(dicBackHome)
            elif qNumber == BLD_PROFILE_CMD.FOLD_ALL.value:
                listReturn.extend(dicAllFold)
            elif qNumber == BLD_PROFILE_CMD.FOLD_ARM.value:
                listReturn.extend(dicArmFold)
            elif qNumber == BLD_PROFILE_CMD.EXPAND_ALL.value:
                listReturn.extend(dicAllExpand)
            elif qNumber == BLD_PROFILE_CMD.EXPAND_ARM.value:
                listReturn.extend(dicArmExpand)
            elif qNumber == BLD_PROFILE_CMD.SAVE_POS.value:
                SavePos()
                return {f"{reqargs}": dicMotorPos}, 200
            #BLB_TABLEVIEW.html 에서 수신하는 API
            #이 API 결과값을 바탕으로 표시되는 테이블 색깔이 정해진다
            elif qNumber == BLD_PROFILE_CMD.GET_TABLEMAP.value:
                dfTableInfo = pd.read_csv(strFileTableNodeEx, sep=sDivTab)
                lsdfTable=dfTableInfo.to_dict(orient='records')
                return lsdfTable,200
        else:
            recvDataTmp = getDic_strArr(recvData.upper(), sDivFieldColon, sDivItemComma)
            PROFILE = recvDataTmp.get(BLB_CMD_CUSTOM.PROFILE.name)
            if PROFILE is None:
                listReturn.append(recvDataTmp)
            else:
                cmdProfile= PROFILE.replace(sDivEmart,sDivItemComma)
                pub_BLB_CMD.publish(recvData.upper())
                return {False: f'Not defined code {qNumber}'}, 202
        if len(listReturn) > 0:
            bResult,bExecuteMsg=SendCMD_Device(listReturn,filter_rate,checkSafety)
        else:
            data_out = f'PROFILE{sDivFieldColon}{qNumber}'
            #data_out = json.dumps(reqargs) 
            pub_BLB_CMD.publish(data_out)
            return {False: f'Not defined code {qNumber}'}, 202
    except Exception as e:
        SendCMDESTOP(f'I{ACC_DECC_SMOOTH}',False)
        rospy.logerr(f"Invalid data:{reqargs},{e}")
        logSSE_error(traceback.format_exc())
        return GetErrResponse(e)
  
    return {bResult: bExecuteMsg}, 200

@app.route(f'/{EndPoints.alarm_table.name}', methods=['GET', 'POST'])
def service_alarmTable():
    bResult = False
    bStrMsg = ""
    try:
        # dict 생성
        alm_dict = {
            val.value.split(sDivFieldColon)[0].strip(): val.value.split(sDivFieldColon)[1].strip()
            for val in ALM_User
        }
        #data_out = json.dumps(alm_dict)
        return alm_dict,200
    except Exception as e:
        rospy.logerr(f"❌ 데이터 처리 오류: {e}")
        logSSE_error(traceback.format_exc())
        return GetErrResponse(e)
  
@app.route(f'/{EndPoints.info.name}', methods=['GET', 'POST'])
@app.route(f'/{EndPoints.alarm.name}', methods=['GET', 'POST'])
def special_endpoint():
    path = request.path.strip('/')  # '/alarm' → 'alarm', '/info' → 'info'
    publisher = alarm_publisher
    pathCSV = csvPathalarm
    if path == EndPoints.info.name:
        publisher = info_publisher
        pathCSV = csvPathInfo
        
    try:
        if request.method == 'GET':
            #recvData = request.data
            recvDataMap = request.args.to_dict()
        else:
            recvDataMap = request.json
            if not recvDataMap:
                return {"error": "No data received"}, 400
        if len(recvDataMap) > 0:
          save_dict_to_csv(pathCSV, recvDataMap,maxAlarmHistory)
        loaded_data = load_csv_to_dict(pathCSV, sort_ascending=False)
        data_out = json.dumps(loaded_data)
        publisher.publish(data_out)
    except Exception as e:
        rospy.logerr(f"❌ 데이터 처리 오류: {e}")
        logSSE_error(traceback.format_exc())
        return GetErrResponse(e)
  
    return {"message": f"{path},{request.method},{request.data}"}, 200
  
@app.route('/<path:path>', methods=['GET','POST'])
def receive_data(path):
    try:
        global shared_data
        if request.method == 'GET':
            data = request.data
        else:
            data = request.json
            if not data:
                return {"error": "No data received"}, 400
            
        if path == "MB":
            move_publisher.publish(data)
        elif path.find('RFID') >= 0:
            print(data)
        elif path == TopicName.ARUCO_RESULT.name:
            df = pd.DataFrame(data)
            df_json = df.to_json(orient="records")  # DataFrame -> JSON 문자열 변환
            tag_publisher.publish(df_json)
        else:
            and_publisher.publish(str(data))
        return {"status": "data received and published to ROS"}, 200
    except Exception as e:
        rospy.logerr(f"❌ 데이터 처리 오류: {e}")
        logSSE_error(traceback.format_exc())
        return GetErrResponse(e)

@app.route("/<topic_name>")
def topic_page(topic_name):
    """각 토픽 데이터를 실시간으로 표시하는 페이지"""
    if topic_name not in shared_data:
        result = ', '.join(subscribers.keys())
        return f"<h1>토픽 {topic_name}은 존재하지 않습니다.{result}</h1>", 404

    # URL에서 line 파라미터 가져오기 (기본값: 10)
    items_per_column = request.args.get('line', default=5, type=int)
    data_pub = shared_data.get(topic_name,{})
    
    if len(data_pub) == 0:
        return f"<h1>토픽 {topic_name}은 데이터가 수신되지 않았습니다.{result}</h1>", 404
    
    if topic_name == TopicName.JOB_DF.name:
        print(data_pub)
    
    if isinstance(shared_data.get(topic_name), dict):
        return render_template("topic_dict.html", topic_name=topic_name, items_per_column=items_per_column)
    
    # 데이터가 리스트라면 DataFrame으로 변환 후 HTML에 전달
    elif isinstance(shared_data.get(topic_name), list):
        df = pd.DataFrame(shared_data[topic_name])  # 리스트 → DataFrame 변환
        df_records = df.to_dict(orient="records")  # DataFrame → JSON 변환
        return render_template("topic_df.html", topic_name=topic_name, data=df_records)

def update_data():
    """주기적으로 데이터를 업데이트하고 클라이언트에 전송"""
    while True:
        try:
            for topic_name, data in shared_data.items():
                if data:
                    socketio.emit(f"update_{topic_name}", data)
            time.sleep(1)
        except Exception as e:  
            message = traceback.format_exc()
            rospy.loginfo(message)

@socketio.on("connect")
def handle_connect():
    try:
        """클라이언트가 접속하면 현재 데이터 전송"""
        print("✅ 클라이언트가 WebSocket에 연결됨")
        lsKeys = list(shared_data.keys())
        for topic_name in lsKeys:
            data = shared_data.get(topic_name)  # get은 thread-safe
            if data:
                socketio.emit(f"update_{topic_name}", data)
    except:
        pass

if __name__ == "__main__":
    try:
        loaded_data2 = load_csv_to_dict(csvPathalarm, sort_ascending=False)
        #data_out2 = json.dumps(loaded_data2)
        shared_data[TopicName.HISTORY_ALARM.name] = loaded_data2
        threading.Thread(target=update_data, daemon=True).start()        
        socketio.run(app, host="0.0.0.0", port=HTTP_COMMON_PORT, debug=False, use_reloader=False, log_output=True,allow_unsafe_werkzeug=True)
    except rospy.ROSInterruptException:
        pass