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
diff_roundPulse = round(roundPulse/2)
# dicPOTNOT_ON = getMotorDefaultDic(ModbusID.MOTOR_H.value,True)
# dicPOTNOT_OFF = getMotorDefaultDic(ModbusID.MOTOR_H.value,False)
dicWE_ON = getMotorWE_ONDic()
dicWE_OFF = getMotorWE_OFFDic()
#strNOTAG = RailNodeInfo.NOTAG.name
node_virtual_str = RailNodeInfo.NONE.name
isRealMachine = get_hostname().find(UbuntuEnv.ITX.name) >= 0
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')
shared_data = {topic.name: {} for topic in TopicName}
maxAlarmHistory = 500   #최대 웹 알람/로그 저장 갯수
maxRetryCount = 10
cmdIntervalSec = 3

delayTimeMin = 3
# 설정
CHECK_INTERVAL = 1  #백그라운드에서 1초마다 값을 체크
ALERT_THRESHOLD_SECONDS = 5 #해당 값이 이 시간만큼 변하지 않으면 알람 발생
ALARM_SUPPRESS_SECONDS = 10 #같은 알람은 이 시간 내엔 다시 발생하지 않음
last_alarm_times = {} # 상태 추적

move_publisher = rospy.Publisher(TopicName.CMD_DEVICE.name, String, queue_size=10)
tag_publisher = rospy.Publisher(TopicName.ARUCO_RESULT.name, String, queue_size=10)
and_publisher = rospy.Publisher(TopicName.ANDROID.name, String, queue_size=10)
alarm_publisher = rospy.Publisher(TopicName.HISTORY_ALARM.name, String, queue_size=10)
info_publisher = rospy.Publisher(TopicName.HISTORY_INFO.name, String, queue_size=10)
pub_motorPos = rospy.Publisher(TopicName.MOTOR_POS.name, String, queue_size=1)
pub_alarmstatus = rospy.Publisher(TopicName.ALARM_STATUS.name, String, queue_size=1)
pub_BLB_CMD = rospy.Publisher(TopicName.BLB_CMD.name, String, queue_size=1)
pub_BLB_CROSS = rospy.Publisher(TopicName.CROSS_INFO.name, String, queue_size=1)
pub_BLB_POS = rospy.Publisher(TopicName.POSITION_INFO.name, String, queue_size=1)
pub_SMARTPLUG = rospy.Publisher(TopicName.SMARTPLUG_INFO.name, String, queue_size=1)
pub_RECEIVE_MQTT = rospy.Publisher(TopicName.RECEIVE_MQTT.name, String, queue_size=1)
#pub_RFID_DF = rospy.Publisher(TopicName.RFID_DF.name, String, queue_size=1)
target_pulse_cw = 50000000
ACC_DECC_MOTOR_H = 5000
ACC_DECC_NORMAL = 3000
findNodeTryPulse = roundPulse
#target_pulse_ccw = -target_pulse_cw

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

dicSpdSlow = getMotorSpeedDic(ModbusID.MOTOR_H.value, True, DEFAULT_RPM_MID, ACC_DECC_SMOOTH,ACC_DECC_MOTOR_H)
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
dicPotInfo = getDic_FromFile(filePath_CaliPotConfig)
pot_telesrv = dicPotInfo.get(str(ModbusID.TELE_SERV_MAIN.value))
pot_arm1 = dicPotInfo.get(str(ModbusID.BAL_ARM1.value))
pot_arm2 = dicPotInfo.get(str(ModbusID.BAL_ARM2.value))
pot_540 = dicPotInfo.get(str(ModbusID.ROTATE_MAIN_540.value))
pot_360 = dicPotInfo.get(str(ModbusID.ROTATE_SERVE_360.value))

#epcTarget = None
lastNode = None
lastRSSI = None
lastPos = None
lastAck = ''
isScan = False
isRetry = False
lastcalledAck = DATETIME_OLD
dicLastMotor15 = {}
dicLastMotor11 = {}
dicLastAruco = {}
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
adjustrate_h = SPEED_RATE_H
if not isRealMachine:
    adjustrate = 0.7
    adjustrate_h = 1
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
ACC_MOVE_H = getSpeedTableInfo(ModbusID.MOTOR_H.value,SPEEDTABLE_FIELDS.ACC_CCW.name,adjustrate_h)
DECC_MOVE_H = getSpeedTableInfo(ModbusID.MOTOR_H.value,SPEEDTABLE_FIELDS.ACC_CW.name,adjustrate_h)
SPD_MOVE_H = getSpeedTableInfo(ModbusID.MOTOR_H.value,SPEEDTABLE_FIELDS.SPD.name,adjustrate_h)

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
    pos_BAL1,pos_BAL2,pos_LiftV,pos_540,pos_360,pos_ServArm,pos_MotorH = GetAllMotorPos()
    pos_ServArm_mm = GetTargetLengthMMServingArm(pos_ServArm)    
    distance_tof = dicTOF.get(SeqMapField.DISTANCE.name,-1) if isRealMachine else pos_ServArm_mm
    return distance_tof

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
    # else:
    #     return TRAYDOOR_STATUS.MOVING
    return resultReturn,resultArray

def GetRotateMainAngleFromPulse(target_pulse):
#   angle = mapRange(target_pulse,0,pot_540,0,MAX_ANGLE_TRAY)
#   return round(angle)%MAX_ANGLE_TRAY
    return pulse_to_angle_sse(target_pulse, pot_540)    

def GetRotateTrayAngleFromPulse(target_pulse):
#   angle = mapRange(target_pulse,0,pot_360,0,MAX_ANGLE_TRAY)
#   return round(angle)%MAX_ANGLE_TRAY
    return pulse_to_angle_sse(target_pulse, pot_360)

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
    rpm_arm2 = max(calculate_targetRPM_fromtime(round2CountAbs, rpm_time_arm1),MAINROTATE_RPM_SLOWEST)
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
            if target_angle > 70:
                deccArm2 = ACC_DECC_MOTOR_H
                deccArm1 = ACC_DECC_MOTOR_H

    dicArm1 = getMotorMoveDic(ModbusID.BAL_ARM1.value,True,targetPulse_arm1,rpm_arm1,accArm1,deccArm1)
    dicArm2 = getMotorMoveDic(ModbusID.BAL_ARM2.value,True,targetPulse_arm2,rpm_arm2,accArm2,deccArm2)
    return [dicArm1,dicArm2], rpm_time_arm1,stroke_arm1_signed

def GetControlInfoArms(distanceServingTeleTotal,bUseCurrentPosition = False, spd_rate = 1.0):
    #bUseCurrentPosition 에 따라 RPM 이 달라진다.
    # True 인 경우 현재 포지션 기준으로 RPM 계산.
    # False 인 경우 풀스트로크 기준으로 RPM 계산.
    # STROKE_MAX 상수를 활용하여 밸런스 암 RPM 을 계산하자.
    # lsArm = []
    # currentWeightTotal = 0
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
        spd_rate = 0.25
    
    targetAngle_arm1 = calculate_weight_angle(distanceServingTeleTotal,1300,650,90)
    #target_pulse=mapRange(controlArm2,0,90,0,pot_arm1)
    #targetPulse_arm1 = min(round(mapRange(targetPulse_serv,0,STROKE_MAX,0,pot_arm1)),pot_arm1)
    #서빙암 스트로크 RPM
    lsBal, rpm_time_arm1,stroke_arm1_signed= GetControlInfoBalances(targetAngle_arm1,bUseCurrentPosition,spd_rate)
    servArmCountAbs = round(stroke_servArm_abs/roundPulse)
    if abs(stroke_arm1_signed) > roundPulse:
        rpm_servArm = max(calculate_targetRPM_fromtime(servArmCountAbs, rpm_time_arm1),DEFAULT_RPM_SLOWER)
    else:
        rpm_servArm = round(DEFAULT_RPM_SLOW * spd_rate)
        # if onScaning() or isCaliMode:
        #     rpm_servArm = DEFAULT_RPM_SLOWER
    # 원본 값 보정
    #rpm_servArm = max(DEFAULT_RPM_SLOWER, min(rpm_servArm, DEFAULT_RPM_SLOW))
    #rpm_servArm = max(DEFAULT_RPM_SLOWER, min(rpm_servArm, DEFAULT_RPM_SLOW))
    acc_st =ACC_ST_EXTEND
    decc_st = DECC_ST_EXTEND
    if stroke_servArm_signed < 0:
        acc_st =ACC_ST_FOLD
        decc_st = DECC_ST_FOLD

    dicSrvArm = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value,True,targetPulse_serv,rpm_servArm,acc_st,decc_st)
    lsBal.append(dicSrvArm)
    #rospy.loginfo()
    logSSE_info(json.dumps(lsBal, indent=4))
    return lsBal,rpm_time_arm1

# def getRFIDInvStatus():
#     global shared_data    
#     dicBLB_Status = shared_data.get(TopicName.RFID.name)
#     return isTrue(dicBLB_Status.get(RFID_RESULT.inventoryMode.name))

# def getRFIDInvStatus():
#     global epcTotalView
#     global sInv_Key
#     global sEPCKey
#     global sDivTab
#     try:
#         epcViewInfo = df_to_dict(epcTotalView, sEPCKey, sInv_Key)
#         for epc,invStatus in epcViewInfo.items():
#             if not isTrue(invStatus):
#                 return False
#         return True
#     except Exception as e:
#         return False
#         message = traceback.format_exc()
#         rospy.loginfo(message)
#         SendAlarmHTTP(message,True,BLB_ANDROID_IP_DEFAULT)
#     return False

def getChargerPlugStatus():
    global shared_data    
    dicBLB_Status = shared_data.get(TopicName.SMARTPLUG_INFO.name)
    return isTrue(dicBLB_Status.get(SMARTPLUG_INFO.CHARGERPLUG_STATE.name))

def GetAndroidInfoDic():
    global shared_data    
    dicBLB_Status = shared_data.get(TopicName.ANDROID.name)
    return dicBLB_Status

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
    #dicBLB_Status = shared_data.get(TopicName.SMARTPLUG_INFO.name)
    pos_BAL1,pos_BAL2,pos_LiftV,pos_540,pos_360,pos_ServArm,pos_MotorH = GetAllMotorPos()
    dic_tof = shared_data.get(topicName_TOF,{})
    lsCheckArms = [pos_BAL1,pos_BAL2,pos_ServArm]
    distance_tof = GetTofDistance()
    cur_angle540=abs(GetRotateMainAngleFromPulse(pos_540))
    cur_angle360=GetRotateTrayAngleFromPulse(pos_360)
    isTrayOrigin = True if cur_angle360 == 0 else False
    isLiftOrigin = True if pos_LiftV < roundPulse else False
    bSafetyFalseArms = any(val > roundPulse for val in lsCheckArms)
    df = pd.DataFrame(listReturnTmp)
    #print(listReturnTmp)
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
        #if distance_tof < 200:
        dic31 = ls31[0]
        target_pos31 = try_parse_int(dic31.get(posStr),MIN_INT)
        target_angle360=GetRotateTrayAngleFromPulse(target_pos31)
        
        if distance_tof < 150 and target_angle360 != cur_angle360:
        #if pos_ServArm < roundPulse*6 and target_angle360 != cur_angle360:
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

# def GetMotorPot():
#     di_pot = try_parse_int(dicLastMotor15.get(sDI_POT))
#     return di_pot

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
    
    loaded_data[MonitoringField.LASTSEEN.name]= getDateTime().timestamp()
    data_out = json.dumps(loaded_data)
    pub_SMARTPLUG.publish(data_out)
    return True
           

def rfidInstanceDefault():
    #global epcTarget
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
    #epcTarget = None
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
    # try:
    #     dicBLB_STATUS= shared_data.get(TopicName.BLB_STATUS.name)
    #     lastSeenBLBCMD = dicBLB_STATUS.get(MonitoringField.LASTSEEN.name)
    #     lastseenV_datetime = datetime.fromtimestamp(lastSeenBLBCMD)
    # except Exception as e:  
    #     # message = traceback.format_exc()
    #     # rospy.loginfo(message)
    #     lastseenV_datetime = DATETIME_OLD
    #pub_BLB_CMD.publish(BLD_PROFILE_CMD.ESTOP.value)
    # if isTimeExceeded(lastseenV_datetime, 10):
    #     resultArd = service_setbool_client_common(ServiceBLB.CMD_ESTOP.value, enable, Kill)
    # else:
    #     pub_BLB_CMD.publish(BLD_PROFILE_CMD.ESTOP.value)
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
    #rospy.loginfo(sCmdArd)
    time.sleep(MODBUS_WRITE_DELAY)        
    return resultArd,strMsg

def SendCMD_Device(sendbuf, cmdIntervalSec=0.01,isCheckSafety=True, delayTime = 0):
    if not hasattr(SendCMD_Device, "last_cmd_time"):
        SendCMD_Device.last_cmd_time = 0
    if not hasattr(SendCMD_Device, "last_cmd_msg"):
        SendCMD_Device.last_cmd_msg = None
        
    if delayTime > 0:
        time.sleep(delayTime)
        
    now = time.time()
    cmdTmp = sendbuf
    if isinstance(cmdTmp, list) and len(cmdTmp) > 0:
        dictPos = GetAllMotorPosDic()
        sendbuf = CheckMotorCmdValid(cmdTmp,dictPos)
        if len(sendbuf) > 0:
            if isCheckSafety:
                bResultSafety,strMsg=CheckSafetyMotorMove(sendbuf)
                if not bResultSafety:
                    logSSE_info(strMsg)
                    return False,strMsg
            cmdTmp = json.dumps(sendbuf)
            if cmdTmp == SendCMD_Device.last_cmd_msg:
                # 같은 메시지면 쿨타임 검사
                if now - SendCMD_Device.last_cmd_time < cmdIntervalSec:
                    strMsg = ALM_User.CMD_INTERVAL_DUPLICATED.value
                    logSSE_info(strMsg)
                    return False,strMsg
            SendCMD_Device.last_cmd_time = now
            SendCMD_Device.last_cmd_msg = cmdTmp
        else:
            strMsg = ALM_User.ALREADY_FINISHED_CMD_POS.value
            logSSE_info(strMsg)
            return True,strMsg
    else:
        strMsg = ALM_User.CMD_FORMAT_INVALID.value
        logSSE_info(strMsg)
        return False,strMsg
        
    #log_all_frames(sendbuf)
    #rfidInstanceDefault()
    bExecuteResult = service_setbool_client_common(ServiceBLB.CMD_DEVICE.value, cmdTmp, Kill)
    return bExecuteResult, AlarmCodeList.OK.name

def callbackACK(recvData):
    #global epcTarget
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
        if len(lsResult) <3:
            return
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
            #callbackACK.retryCount = 0
            return
        
        #lastcalledAck = getDateTime()
        if len(lsResult) > 14:
            #print(lsResult)
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
            logSSE_info(f'{mbid_tmp}-정방향토크:{torque_max},역방향토크:{torque_min},평균토크:{torque_mean},운행시작:{last_started_pos},목표지점:{last_targeted_pos},현지점:{stopped_pos},속도:{last_spd},isHome:{isHome},isNot:{isNot},isPot:{isPot},isESTOP:{isESTOP}')
            # if mbid_instance.value in lsRotateMotors and isTrue(isHome):
            #     lsCmdCaliHome = [getMotorWHOME_OFFDic(mbid_tmp),getMotorHomeDic(mbid_tmp)]
            #     bSafetyCheck,strMsg = SendCMD_Device(lsCmdCaliHome)
            #     logSSE_info(f'Cali {mbid_tmp} Result:{bSafetyCheck},{strMsg}')
            # #POT 예정 펄스보다 5바퀴 이상 차이나면 모든 모터를 중지하고 알람을 날린다
            # el
            if mbid_instance in lsReleaseMotors and not onScaning():
                if isTrue(isPot) or isTrue(isNot):
                    diffPos = abs(last_targeted_pos - stopped_pos)
                    if diffPos > roundPulse*5:
                        SendCMDESTOP(f'I{ACC_DECC_SMOOTH}',True)
                        logSSE_info(f'ESTOP triggered at {mbid_tmp} too much diff pos {diffPos}')
            elif is_equal(mbid_tmp,ModbusID.ROTATE_MAIN_540.value) or is_equal(mbid_tmp,ModbusID.ROTATE_SERVE_360.value):
                if isTrue(isHome):
                    dicLoc = getMotorHomeDic(mbid_tmp)
                    SendCMD_Device([getMotorWHOME_OFFDic(mbid_tmp),dicLoc])                
                return
        
        if not is_equal(mbid_tmp,ModbusID.MOTOR_H.value):
            return
        
        # if not isTimeExceeded(lastcalledAck, 200):
        #     return
        # last_inv = dfEPCTotal.iloc[-1][sInv_Key] if not dfEPCTotal.empty else None
        # last_EPC = dfEPCTotal.iloc[-1][sEPCKey] if not dfEPCTotal.empty else None        
        # print(last_inv)
        #if lastNode
        #cur_pos = try_parse_int(dicMotorPos.get('MB_15'))
        #현재 노드 추정
        # lsCurNode = find_nearest_pos(dfNodeInfo,stopped_pos,1)
        # dicCurNodeInfo = lsCurNode[0]
        dicCurNodeInfo=GetNodeDicFromPos(dfNodeInfo,stopped_pos,isTrue(isPot))        
        #print(lsCurNode)
        curNodeID_fromPulse = dicCurNodeInfo.get(TableInfo.NODE_ID.name)
        curNode_type = str(dicCurNodeInfo.get(RFID_RESULT.EPC.name))
        logSSE_info(f'운행완료.지시노드:{endnode},직전노드:{lastNode},현재노드:{curNodeID_fromPulse}:{curNode_type},Retried:{callbackACK.retryCount},POT:{isPot}')
        logSSE_info(f'정방토크:{torque_max},역방토크:{torque_min},평균토크:{torque_ave},최대부하:{ovr_max},평균부하:{ovr_ave},시작위치:{last_started_pos},지시위치:{last_targeted_pos}:,정지위치:{stopped_pos},속도:{last_spd}')
        #목적지 노드 타입 확인 - 유효한 EPC이면 RFID노드, NONE 면 가상노드, NOTAG 면 도그 노드.
        
        
        #도그 노드, 혹은 특수 노드인데 도달하지 못 했으면 댐핑으로 추가 운행시도.
        if isScan or isTrue(isPot):
            # dfEPCTotal.to_csv(strFileEPC_total, sep=sDivTab, index=False)
            # isScan = False
            # print(strFileEPC_total)
            # rospy.loginfo(f'Scan End :{cur_pos}')
            # dfEPCTotal.drop(dfEPCTotal.index, inplace=True) 
            #RFIDControl(False)
            
            if curNodeID_fromPulse == endnode:
            #if curNodeID_fromPulse in NODES_SPECIAL and curNodeID_fromPulse == endnode:
                endNodePos = GetNodePos_fromNode_ID(endnode)
                dicLoc = getMotorLocationSetDic(ModbusID.MOTOR_H.value, endNodePos)
                SendCMD_Device([dicLoc])
                #SendCMD_Device([dicWE_OFF,dicLoc,dicWE_ON])
                #TTSAndroid('위치오차를 보정합니다',1)
                if endnode == NODE_KITCHEN:
                    bResult = SetChargerPlug(True)
                    if bResult:
                        TTSAndroid('충전을 시작합니다',1)
                    else:
                        TTSAndroid('충전기를 점검해주세요',1)
            # else:
            #     SendCMD_Device([dicWE_OFF])
                
            #RFIDControl(False)
            rfidInstanceDefault()
            return
        
        if curNode_type.find(strNOTAG) < 0 and curNodeID_fromPulse == endnode:
        #if (curNode_type == 'NONE' or len(curNode_type) == 24) and curNodeID_fromPulse == endnode:
            #RFIDControl(False)
            rfidInstanceDefault()
            return
        
        #rospy.loginfo(f'torque_max:{torque_max},torque_ave:{torque_ave},ovr_max:{ovr_max},ovr_ave:{ovr_ave},last_started_pos:{last_started_pos},last_targeted_pos:{last_targeted_pos}:isNot:{isNot},isPot:{isPot},stopped:{stopped_pos},cur_posH:{cur_pos},lastSPD:{last_spd}')
        # if not getRFIDInvStatus():
        #     return
        
        # if endnode == lastNode:
        # #if lastRSSI is None:
        #     return
        
        # # if epcTarget is None:
        # #     return
        
        # if isTrue(GetMotorPot()):
        #     return
        
        # dfEPCTotal = pd.read_csv(strFileEPC_total, sep=sDivTab)    
        # epcViewInfo = df_to_dict(epcTotalView, sEPCKey, sInv_Key)
        # dicEPCNode = GetEPCNodeInfoDic()
        # isInvOff = False
        # for epc,invStatus in epcViewInfo.items():
        #     if isTrue(invStatus) and len(epc) > 5:
        #         curNode = dicEPCNode.get(epc)
        #         if curNode == endnode:
        #             rospy.loginfo(f'OK curnode:{curNode},epc:{epc} invStatus:{invStatus}')
        #             RFIDControl(False)
        #             return
        #     elif not isTrue(invStatus):
        #         isInvOff = True
                
        # dicSmartPlug = shared_data.get(TopicName.SMARTPLUG_INFO.name)
        # chargeSensorinfo = dicSmartPlug.get(SMARTPLUG_INFO.GPI1_CHARGE.name)
        # chargerInfo = dicSmartPlug.get(SMARTPLUG_INFO.CHARGERPLUG_STATE.name)
        # if isTrue(chargeSensorinfo):
        #     if isTrue(chargerInfo):
        #         SetChargerPlug(True)
        #     return
        
        #SendAlarmHTTP(f'RFID 인식이 안됨! {epcTarget} - {lastNode}',True,BLB_ANDROID_IP_DEFAULT)
        diff_pos = stopped_pos - startPos
        #diff_pos = cur_pos - startPos
        #diff_pos_abs = abs(diff_pos)
        
        findNodePulse = -findNodeTryPulse if diff_pos < 0 else findNodeTryPulse
        if callbackACK.retryCount < maxRetryCount and isRealMachine:
        #if diff_pos_abs > roundPulse and :
            callbackACK.retryCount += 1
            finalPos = stopped_pos + findNodePulse
            if endPos is not None:
                finalPos = endPos
                callbackACK.retryCount = 100
            dicJOG = getMotorMoveDic(ModbusID.MOTOR_H.value,True,finalPos ,DEFAULT_RPM_SLOW,ACC_DECC_MOTOR_H,ACC_DECC_MOTOR_H*2)
            logSSE_info(f'다시탐색{callbackACK.retryCount}:{dicJOG}')
            endPos = None
            dicPotNot = getMotorWP_ONDic()  if finalPos > stopped_pos else getMotorWN_ONDic()
            SendCMD_Device([dicPotNot,dicJOG],0.5,False)
            #SendCMD_Device([dicWE_ON,dicJOG],0.5,False)
        else:
            # SendCMDESTOP(f'I{ACC_DECC_MOTOR_H}', False)
            # message = '노드 위치 계산 에러입니다.'
            # SendAlarmHTTP(message,True,BLB_ANDROID_IP_DEFAULT)
            rfidInstanceDefault()
            di_pot,di_not,di_home,di_estop, si_pot,si_home=GetMotorSensor(topicName_RotateTray)
            print(f'di_pot:{di_pot},di_not:{di_not},si_pot:{si_pot},si_home:{si_home}')
    except Exception as e:
        message = traceback.format_exc()
        logSSE_error(message)
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
    # if not hasattr(callbackMB_15, "retryCount"):
    #         callbackMB_15.retryCount = 0        
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
        # sSPD_abs = abs(sSPD_signed)
        # sPOS = try_parse_int(recvDataMap.get(sPOS_Key))
        # si_pot = recvDataMap.get(sSI_POT,"")
        # di_pot_status = recvDataMap.get(sDI_POT,"")
        # last_started_pos = try_parse_int(recvDataMap.get(MonitoringField.LAST_STARTED_POS.name))        
        if sSPD_signed < 0 and callbackMB_11.last_target_mm > 100:
            tof_distance=GetTofDistance()
            if is_between(callbackMB_11.last_target_mm-10,callbackMB_11.last_target_mm+20,tof_distance):
                logSSE_info(f'Stop at TOF : {tof_distance}, Target = {callbackMB_11.last_target_mm}')
                StopMotor(lsReleaseMotors,EMERGENCY_DECC)
                callbackMB_11.last_target_mm = -1
    except Exception as e:
        message = traceback.format_exc()
        logSSE_error(message)
        SendAlarmHTTP(message,True,BLB_ANDROID_IP_DEFAULT)

# def callbackMB_Rotate(recvDataMap):
#     global lastNode
#     global lastPos
#     global endPos
#     global endnode
#     global isScan
#     global dicLastMotor11
#     global dicPulsePos  #도그 신호가 들어올때 엔코더 값을 기록해두고 보정수치로 쓴다
#     dicLastMotor11.update(recvDataMap)
#     try:
#         isST_CMD_FINISH = recvDataMap.get(sST_CMD_FINISH)
#         if not isTrue(isST_CMD_FINISH):
#             return
        
#         sSPD_signed = (recvDataMap.get(sSPD_Key))
#         target_pos = (recvDataMap.get(sLAST_TARGET_POS))
#         if sSPD_signed is None:
#             return
#         if target_pos is None:
#             return
#         sSPD_signed = int(sSPD_signed)
#         target_pos = int(target_pos)
#         #isCrossRailRunOK = isCrossRailDirection()
#         #sSPD_abs = abs(sSPD_signed)
#         sPOS = int(recvDataMap.get(sPOS_Key))
#         di_pot_status = isTrue(recvDataMap.get(sDI_POT,""))
        
#         # sSPD_abs = abs(sSPD_signed)
#         # sPOS = try_parse_int(recvDataMap.get(sPOS_Key))
#         # si_pot = recvDataMap.get(sSI_POT,"")
#         # di_pot_status = recvDataMap.get(sDI_POT,"")
#         # last_started_pos = try_parse_int(recvDataMap.get(MonitoringField.LAST_STARTED_POS.name))        
#         if sSPD_signed < 0 and callbackMB_11.last_target_mm > 100:
#             tof_distance=GetTofDistance()
#             if is_between(callbackMB_11.last_target_mm-10,callbackMB_11.last_target_mm+20,tof_distance):
#                 rospy.loginfo(f'Stop at TOF : {tof_distance}, Target = {callbackMB_11.last_target_mm}')
#                 StopMotor(lsReleaseMotors,EMERGENCY_DECC)
#                 callbackMB_11.last_target_mm = -1
#     except Exception as e:
#         message = traceback.format_exc()
#         rospy.loginfo(message)
#         SendAlarmHTTP(message,True,BLB_ANDROID_IP_DEFAULT)


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
        #isCrossRailRunOK = isCrossRailDirection()
        sSPD_abs = abs(sSPD_signed)
        sPOS = int(recvDataMap.get(sPOS_Key))
        di_pot_status = isTrue(recvDataMap.get(sDI_POT,""))
        last_started_pos = int(recvDataMap.get(MonitoringField.LAST_STARTED_POS.name))
        if di_pot_status:
            sLogMsg = f'도그위치:{sPOS},시작위치:{last_started_pos},목표위치:{target_pos},속도:{sSPD_signed}'
            logSSE_info(sLogMsg)        
            if callbackMB_15.ESTOP_ON == BLD_PROFILE_CMD.ESTOP.name:
                callbackMB_15.ESTOP_ON = BLD_PROFILE_CMD.BACK_HOME.name
                StopMotor(ModbusID.MOTOR_H.value,ACC_DECC_NORMAL)
                if sSPD_signed > 0:
                    endPos = sPOS - (sSPD_signed * 5)
                else:
                    endPos = sPOS + (sSPD_signed * 5)    # - (sSPD_signed * 20) 
                # sLogMsg = f'도그위치:{sPOS},시작위치:{last_started_pos},목표위치:{target_pos},속도:{sSPD_signed}'
                # rospy.loginfo(sLogMsg)
            else:
                dicCurNodeInfo=GetNodeDicFromPos(dfNodeInfo,sPOS,di_pot_status)
                if not dicCurNodeInfo:
                    return
                endnode_pos = int(recvDataMap.get(MonitoringField.LAST_TARGET_POS.name))
                dicEndNodeInfo=GetNodeDicFromPos(dfNodeInfo,endnode_pos)
                endNodeID_fromPulse = dicEndNodeInfo.get(TableInfo.NODE_ID.name)
                curNodeID_fromPulse = dicCurNodeInfo.get(TableInfo.NODE_ID.name)
                dicCurNodeInfo['END_NODE'] = endnode
                logSSE_info(dicCurNodeInfo)        
                #rospy.loginfo(sLogMsg)        
                
                if endNodeID_fromPulse == curNodeID_fromPulse:
                    return
                curNode_type = str(dicCurNodeInfo.get(RFID_RESULT.EPC.name))
                #curNode_pos = int(dicCurNodeInfo.get(posStr))
                lastNode = curNodeID_fromPulse
                
                # dicSpdControl = {}
                # if curNode_type.find(RailNodeInfo.R_END.name) >= 0:
                #     if sSPD_signed > 0 and sSPD_abs > 100:
                #         dicSpdControl.update(dicSpdFast)
                #     if sSPD_signed < 0 and sSPD_abs > 100:
                #         dicSpdControl.update(dicSpdSlow)
                # elif curNode_type.find(RailNodeInfo.R_START.name) >= 0:
                #     if sSPD_signed < 0:
                #         dicSpdControl.update(dicSpdFast)
                #     else:
                #         dicSpdControl.update(dicSpdSlow)                
                # if dicSpdControl and not is_equal(endnode,0):
                #     SendCMD_Device([dicSpdControl])

        if abs(target_pos - sPOS) > roundPulse * 10:
            return
        
        si_pot = recvDataMap.get(sSI_POT,"")
        #     sEPC = notagstr
        #     sRSSI = 0
        #     sPOS = try_parse_int(recvDataMap.get(sPOS_Key))
        #     sSPD = try_parse_int(recvDataMap.get(sSPD_Key))
        #     sInv = recvDataMap.get(sInv_Key,notagstr)
        #     dicNode[sEPCKey] = sEPC
        #     dicNode[sPOS_Key] = sPOS
        #     dicNode[sInv_Key] = sInv
        #     dicNode[sRSSIKey] = sRSSI
        #     dicNode[sSPD_Key] = sSPD
        #     callbackRFID(dicNode)
        #현재 노드 추정. - endNode 와 오차 비교하여 출력.
        # lsCurNode = find_nearest_pos(dfNodeInfo,sPOS,1,0)
        # dicCurNodeInfo = lsCurNode[0]
        # dicCurNodeInfo=GetNodeDicFromPos(dfNodeInfo,sPOS)
        # if not dicCurNodeInfo:
        #     return
        
        # curNodeID_fromPulse = dicCurNodeInfo.get(TableInfo.NODE_ID.name)
        # curNode_type = str(dicCurNodeInfo.get(RFID_RESULT.EPC.name))
        # curNode_pos = int(dicCurNodeInfo.get(posStr))
        # lastNode = curNodeID_fromPulse
        # endNodePos = GetNodePos_fromNode_ID(endnode)
        # # lsEndNode = find_nearest_pos(dfNodeInfo,endNodePos,1,0)
        # # dicEndNodeInfo = lsEndNode[0]
        # dicEndNodeInfo=GetNodeDicFromPos(dfNodeInfo,endNodePos)        
        # endNode_type = str(dicEndNodeInfo.get(RFID_RESULT.EPC.name))
        # curNodePosMaster = GetNodePos_fromNode_ID(curNodeID_fromPulse)
        # if isTrue(di_pot_status):
        #     dicStartNodeInfo=GetNodeDicFromPos(dfNodeInfo,last_started_pos)
        #     startNodeID_fromPulse = dicStartNodeInfo.get(TableInfo.NODE_ID.name)
        #     if dicPulsePos.get(curNodeID_fromPulse) is None:
        #         dicPulsePos[curNodeID_fromPulse] = sPOS
        #         if not is_equal(startNodeID_fromPulse,curNodeID_fromPulse):
        #             if sSPD_signed > 0:
        #                 endPos = sPOS - roundPulse
        #             else:
        #                 endPos = sPOS + roundPulse
        #             sLogMsg = f'!도그감지:{curNodeID_fromPulse},목표노드:{endnode}.모드:{si_pot},현재위치마스터:{curNodePosMaster},현재위치실측:{sPOS},엔코더오차:{curNodePosMaster-sPOS}'    
        #             rospy.loginfo(sLogMsg)
        #         # if is_equal(curNodeID_fromPulse,endnode):
        #         #     StopMotor(ModbusID.MOTOR_H.value)
        #         #SendInfoHTTP(sLogMsg)
        #         #TTSAndroid('도그감지',1)

        # dicSpdControl = {}
        # if curNode_type.find(RailNodeInfo.R_END.name) >= 0:
        #     if sSPD_signed > 0 and sSPD_abs > 100:
        #         dicSpdControl.update(dicSpdFast)
        #     if sSPD_signed < 0 and sSPD_abs > 100:
        #         dicSpdControl.update(dicSpdSlow)
        # elif curNode_type.find(RailNodeInfo.R_START.name) >= 0:
        #     if sSPD_signed < 0:
        #         dicSpdControl.update(dicSpdFast)
        #     else:
        #         dicSpdControl.update(dicSpdSlow)
        # if abs(curNode_pos - sPOS) < roundPulse/5 or isTrue(di_pot_status):
        #     #rospy.loginfo(f'@현재노드:{curNodeID_fromPulse},현재위치마스터:{curNodePosMaster},현재위치실측:{sPOS},엔코더오차:{curNodePosMaster-sPOS},목표위치:{target_pos},노드속성:{endNode_type},모드:{si_pot},현재속도:{sSPD_signed}')
        #     if dicSpdControl:
        #         SendCMD_Device([dicSpdControl])
                
        #현재 노드와 목적 노드가 같으면 별다른 조치 없음. CallbackAck 에서 처리.
        #다른 경우 내 노드가 목적지 노드와 인접한 노드인지 확인해야 함.
        #node_near = 

        listReturnTmp = []
        if callbackMB_15.ESTOP_ON == BLD_PROFILE_CMD.MOTORSTOP.name:
            callbackMB_15.ESTOP_ON = BLD_PROFILE_CMD.ESTOP.name            
            #listReturnTmp.append(dicSpdSlow)
            #listReturnTmp.append(dicWE_ON)
            # potnotControlDic = getMotorWP_ONDic() if sSPD_signed > 0 else getMotorWN_ONDic()
            # listReturnTmp.append(potnotControlDic)
            logSSE_info(f'#EStop On - 목표까지 남은펄스:{target_pos-sPOS},현재속도:{sSPD_signed},현재위치:{sPOS},목표위치:{target_pos},모드:{si_pot}')
            #SendCMD_Device(listReturnTmp)

        # #목적지가 충전소면 NOT활성화.        
        # # dicSpd = getMotorSpeedDic(ModbusID.MOTOR_H.value, True, DEFAULT_RPM_SLOW, ACC_DECC_LONG,ACC_DECC_LONG)
        # # potnotControlDic = getMotorWP_ONDic() if sSPD_signed > 0 else getMotorWN_ONDic()
        
        # #목적지가 충전소 일땐 NOT 활성
        # # if endnode == NODE_KITCHEN and si_pot != "NOT":
        # #     listReturnTmp.append(getMotorWN_ONDic())
        # #     rospy.loginfo('#목적지가 충전소 일땐 NOT 활성')
        # # #목적지가 분기기이고,충전소에서 출발하는 경우는 POT 활성
        # # elif endnode == NODE_CROSS:
        # #     if not isCrossRailRunOK:
        # #         if si_pot != "POT" and sSPD_signed > 0:
        # #             listReturnTmp.append(potnotControlDic)
        # #             rospy.loginfo('#목적지가 분기기이고,충전소에서 출발하는 경우는 POT 활성')
        # # else:   #그 외에는 움직이는 방향에 따라 좌우됨.
        # #그외에는 모두 댐핑이며 추후 구현.
        # #목적지까지 10만 펄스 이내가 아니면 리턴.
        # #if not is_within_range_signed(target_pos,sPOS, roundPulse*10):
        # if endNode_type.find(notagstr) < 0:
        #     return
        # #rospy.loginfo(f'#엔코더오차:{curNodePosMaster-sPOS},현재속도:{sSPD_signed},현재위치:{sPOS},목표위치:{target_pos},모드:{si_pot},노드속성:{endNode_type}')
        # if callbackMB_15.ESTOP_ON != BLD_PROFILE_CMD.ESTOP.name:
        # #if si_pot != BLD_PROFILE_CMD.ESTOP.name and abs(sSPD_signed) > 10 and callbackMB_15.ESTOP_ON != BLD_PROFILE_CMD.ESTOP.name:
        #     #dicInfo_local = getMotorMoveDic(ModbusID.MOTOR_H.value, True, endNodePos, round(SPD_MOVE_H*SPEED_RATE_H*0.5), ACC_MOVE_H,DECC_MOVE_H)
        #     listReturnTmp.append(dicSpdSlow)
        #     listReturnTmp.append(dicWE_ON)
        #     callbackMB_15.ESTOP_ON = BLD_PROFILE_CMD.ESTOP.name
        #     #listReturnTmp.append(dicInfo_local)
        #     rospy.loginfo(f'#EStop On - 목표까지 남은펄스:{curNodePosMaster-sPOS},현재속도:{sSPD_signed},현재위치:{sPOS},목표위치:{target_pos},모드:{si_pot},노드속성:{endNode_type}')
        # # if si_pot != "POT" and sSPD_signed > 0 and abs(sSPD_signed) > 10:
        # #     listReturnTmp.append(getMotorWP_ONDic())
        # #     rospy.loginfo(f'#POT On - 목표까지 남은펄스:{curNodePosMaster-sPOS},현재속도:{sSPD_signed},현재위치:{sPOS},목표위치:{target_pos},모드:{si_pot},노드속성:{endNode_type}')
        # # elif si_pot != "NOT" and sSPD_signed < 0 and abs(sSPD_signed) > 10:
        # #     listReturnTmp.append(getMotorWN_ONDic())
        # #     rospy.loginfo(f'#NOT On - 목표까지 남은펄스:{curNodePosMaster-sPOS},현재속도:{sSPD_signed},현재위치:{sPOS},목표위치:{target_pos},모드:{si_pot},노드속성:{endNode_type}')
        # if len(listReturnTmp) > 0:
        #     SendCMD_Device(listReturnTmp)
            #lastcalledAck = getDateTime()
        #목적펄스까지 
    except Exception as e:
        message = traceback.format_exc()
        logSSE_error(message)
        SendAlarmHTTP(message,True,BLB_ANDROID_IP_DEFAULT)

# def callbackRFID(recvDataMap):
#     #global epcTarget
#     global lastNode
#     global lastRSSI
#     global lastPos
#     global endnode
#     global isScan
#     global epcTotalView
#     global dfNodeInfo
#     needSave = False
#     #RFID 부분은 없앰
#     #현재 들어온 포지션으로 노드 계산
#     #계산된 노드가 목표노드와 인접한 노드인지 확인
#     #목표노드가 상대노드(도그근처에 가상으로 만든 노드)면 TARGET_POS 를 현재 포지션과 인접한 노드 마스터 정보의
#     #포지션과 비교하여 엔코더 오차 보정하여 보정명령어 송출.
#     #진짜 노드인 경우는 감속
#     try:
#         sEPC = recvDataMap.get(sEPCKey)
#         sRSSI = abs(try_parse_int(recvDataMap.get(sRSSIKey,0)))
#         sPOS = try_parse_int(recvDataMap.get(sPOS_Key))
#         sSPD = (try_parse_int(recvDataMap.get(sSPD_Key)))
#         sInv = recvDataMap.get(sInv_Key,notagstr)
#         sSPDAbs = abs(sSPD)
#         epcnodeinfo = GetEPCNodeInfoDic()
#         curNode = epcnodeinfo.get(sEPC)
#         if sEPC is None:
#             if isScan:
#                 return
#             sEPC = sInv
#             sRSSI = 0
#             sSPDAbs = 0
#             recvDataMap[sEPCKey] = notagstr
#         else:
#             recvDataMap[sInv_Key] = 'on'
#         if sRSSI > RSSI_FILTER:
#             return

#         if recvDataMap.get(sALIVE_Key) is not None:
#             #스마트 플러그값 폴링
#             handle_charge()
#             return
        
#         #dfEPCTotal = pd.read_csv(strFileEPC_total, sep=sDivTab)        
#         if isScan:
#             dfScan = pd.read_csv(strFileEPC_scan, sep=sDivTab)
#             epcnodeinfo2 = df_to_dict_int_values(dfScan, MAPFIELD.EPC.name, TableInfo.NODE_ID.name)
#             result = get_missing_or_next(list(epcnodeinfo2.values()))
#             if epcnodeinfo2.get(sEPC) is not None:
#                 result=epcnodeinfo2.get(sEPC)
            
#             #기준이 되는 노드는 업데이트 하지 않는다.
#             if result in NODES_SPECIAL:
#                 return
            
#             dicEPCNodeInfo = {TableInfo.NODE_ID.name : int(result) ,MotorWMOVEParams.POS.name : sPOS,MAPFIELD.EPC.name: sEPC}
#             dfScan = insert_or_update_row_to_csv(strFileEPC_scan, sDivTab, dicEPCNodeInfo,MotorWMOVEParams.POS.name)
#             print(dfScan)
#             # df_json = df.to_json(orient="records")  # DataFrame -> JSON 문자열 변환
#             # pub_RFID_DF.publish(df_json)
#             return                    
        
#         if sRSSI != 0:
#             epc_column = MAPFIELD.EPC.name  # 예: 'EPC'

#             if epc_column in epcTotalView.columns:
#                 filtered_df = epcTotalView[epcTotalView[epc_column] == str(sEPC)]
#                 if not filtered_df.empty:
#                     result_dict = filtered_df.tail(1).to_dict(orient='records')[0]
#                 else:
#                     result_dict = {}
#             else:
#                 result_dict = {}  # EPC 컬럼 자체가 없음

#             #df = insert_or_update_row_to_df(epcTotalView, recvDataMap, MonitoringField.CUR_POS.name)
#             df = insert_or_update_row_to_df(epcTotalView, recvDataMap, sEPCKey)
#             df_json = df.to_json(orient="records")  # DataFrame -> JSON 문자열 변환
#             pub_RFID_DF.publish(df_json)
            
#             long_values = df[df[sEPCKey].astype(str).str.len() >= 6][sEPCKey].tolist()
#             if len(long_values) > 1:   
#                 epcTotalView.drop(epcTotalView.index, inplace=True)

#             # if not is_equal(result_dict.get(MAPFIELD.EPC.name), sEPC):
#             #     epcTotalView.drop(epcTotalView.index, inplace=True)
        
#             if curNode is None:
#                 result = get_missing_or_next(list(epcnodeinfo.values()))
#                 epcnodeinfo[sEPC] = curNode = endNode = result
#                 #curNode = endNode = result
#                 #epcTarget = sEPC
#                 endNode = curNode
#                 saveDic_ToFile(epcnodeinfo,strFileEPC_node, sDivTab,True)
#                 rospy.loginfo(f'New EPC : {sEPC},Node:{curNode}, POS:{sPOS}')                
#                 needSave = True
#             recvDataMap[TableInfo.NODE_ID.name] = curNode
#         elif sSPDAbs == 0 and not isScan:
#             df = insert_or_update_row_to_df(epcTotalView, recvDataMap, MAPFIELD.EPC.name)
#             df_json = df.to_json(orient="records")  # DataFrame -> JSON 문자열 변환
#             pub_RFID_DF.publish(df_json)
#             return        
#         if isScan:
#         #if epcTarget is None or isScan:
#             return
        
#         # lastNode = curNode        
#         # endNode = epcnodeinfo.get(epcTarget)
#         # if curNode == endNode:
#         #     if lastRSSI is None:
#         #         lastRSSI = sRSSI
#         #         lastPos = sPOS
#         #         rospy.loginfo(f'lastRSSI init : {lastRSSI},POS:{lastPos},SPD:{sSPD}')
#         #         return
            
#         #     if sRSSI <= lastRSSI:
#         #         lastRSSI = sRSSI
#         #         lastPos = sPOS
#         #         rospy.loginfo(f'lastRSSI updated : {lastRSSI},POS:{lastPos},SPD:{sSPD}')
#         #         return
            
#         #     if sSPD > 0:
#         #         finalPos = lastPos+(sSPD*10*3)            
#         #     else:
#         #         finalPos = lastPos+(sSPD*10*4)            
            
#         #     dicJOG = getMotorMoveDic(ModbusID.MOTOR_H.value,True,finalPos ,sSPDAbs,ACC_DECC_MOTOR_H,ACC_DECC_MOTOR_H)
#         #     dicEPCNodeInfo = {TableInfo.NODE_ID.name : lastNode,MotorWMOVEParams.POS.name : finalPos,MAPFIELD.EPC.name: sEPC}
#         #     # if needSave:
#         #     #     #df = insert_row_to_csv(strFileEPC_total, sDivTab, dicEPCNodeInfo)
#         #     #     df = insert_or_update_row_to_csv(strFileEPC_total, sDivTab, dicEPCNodeInfo, MAPFIELD.EPC.name)            
#         #     #     print(df)
#         #     #SendCMDESTOP(f'I{sSPD}')
#         #     SendCMD_Device([dicJOG])
#         #     epcTarget = None
#         #     lastRSSI = None
#         #     lastPos = None
#         #     time.sleep(MODBUS_EXCEPTION_DELAY)
#         #     RFIDControl(False)
#         #     log_all_frames(f"Stop at {endNode},RFID:{sEPC},RSSI:{sRSSI},finalPos:{finalPos},SPD:{sSPD}")
#     except Exception as e:
#         message = traceback.format_exc()
#         rospy.loginfo(message)
#         SendAlarmHTTP(message,True,BLB_ANDROID_IP_DEFAULT)

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

            # if topic_name == topicName_RotateMain or topic_name == topicName_RotateTray:
            #     callbackMB_Rotate(recvDataMap)
            
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
                        if rospy.core.is_initialized():
                            pub_motorPos.publish(json.dumps(dicMotorPos, sort_keys=True))
            else:
                shared_data[topic_name] = recvDataMap  # 최초 생성
            socketio.emit(f"update_{topic_name}", shared_data[topic_name])
        except Exception as e:
            message = traceback.format_exc()
            logmsg = f"{e}:{message} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
            logSSE_info(logmsg)
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

            # SCR_DIR 환경변수에서 경로 읽기
            scr_dir = os.getenv(UbuntuEnv.SCR_DIR.name)

            if scr_dir and os.path.isdir(scr_dir):
                log_files = glob.glob(os.path.join(scr_dir, '*.log'))
                for file_path in log_files:
                    try:
                        os.remove(file_path)
                        print(f"Deleted: {file_path}")
                    except Exception as e:
                        print(f"Failed to delete {file_path}: {e}")
            else:
                print("SCR_DIR is not set or not a valid directory.")
            
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
                dic_newNodeInfo2 = {TableInfo.TABLE_ID.name: table_id_add.upper(), **dic_newNodeInfo}
                logSSE_info(dic_newNodeInfo2)
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
  
    return shared_data, 200

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
    try:
        client_ip = request.remote_addr
        topic_name = request.args.get('topicname', None)
        rospy.loginfo_throttle(10, f"[DATA1] 요청 수신: IP={client_ip}, topicname={topic_name}")

        if topic_name is None:
            rospy.loginfo_throttle(10, "[DATA1] 누락된 파라미터: topicname")
            return {"error": "topicname is required."}, 400

        resultData = shared_data.get(topic_name, None)
        if resultData is None:
            rospy.loginfo_throttle(10, f"[DATA1] 존재하지 않는 topicname 요청: {topic_name}")
            return {"error": "wrong topicname."}, 400

        rospy.loginfo_throttle(10, f"[DATA1] 응답 완료: topicname={topic_name}")
        return resultData, 200

    except Exception as e:
        error_trace = traceback.format_exc()
        logSSE_error(error_trace)
        rospy.logerr(f"[DATA1] 데이터 처리 오류: {e}")
        return GetErrResponse(e)
  
# @app.route('/DATA1', methods=['GET'])
# def service_dataExport():
#     resultData = ''
#     try:
#         topic_name = request.args.get('topicname', None)        
#         if topic_name is None:
#             return {"error": "topicname is required."}, 400
#         resultData = shared_data.get(topic_name, None)
#         if resultData is None:
#             return {"error": "wrong topicname."}, 400
#     except Exception as e:
#         logSSE_error(traceback.format_exc())
#         rospy.logerr(f"데이터 처리 오류: {e}")
#         return GetErrResponse(e)
#     return (resultData), 200

#임과장이 호출하는 엔드포인트. 받은 query_str 그대로 분기기 ESP32 에 포워딩.
@app.route('/cross', methods=['GET'])
def service_junction():
    query_str = request.query_string.decode('utf-8')
    bResult,strMsg = API_CROSS_CMD(query_str)
    return {bResult: strMsg}, 200  

service_last_cmd_time = 0
@app.route('/CROSS', methods=['GET'])
def service_cross():
    global service_last_cmd_time
    try:
        now = time.time()
        recvJunctionStatus=immutable_multi_dict_to_dict(request.args)
        recvJunctionStatus[MonitoringField.LASTSEEN.name]=getDateTime().timestamp()
        data_out = json.dumps(recvJunctionStatus)
        pos_BAL1,pos_BAL2,pos_LiftV,pos_540,pos_360,pos_ServArm,pos_MotorH = GetAllMotorPos()
        dic_tof = shared_data.get(topicName_TOF,{})
        #lsCheckArms = [pos_BAL1,pos_BAL2,pos_ServArm]
        pos_ServArm_mm = GetTargetLengthMMServingArm(pos_ServArm)
        distance_tof = GetTofDistance()
        cur_angle540=abs(GetRotateMainAngleFromPulse(pos_540))
        cur_angle360=GetRotateTrayAngleFromPulse(pos_360)
        cur_pos_mm = pulseH_to_distance(pos_MotorH)
        currentAngle_arm1 =round(mapRange(pos_BAL1,0,pot_arm1,0,90))
        di_pot = dicLastMotor15.get(sDI_POT)
        dicPostion=GetNodeDicFromPos(dfNodeInfo,pos_MotorH,isTrue(di_pot))
        node_id = dicPostion.get(TableInfo.NODE_ID.name)
        table_id = find_most_similar_table_id(node_id,distance_tof,cur_angle540)
        dicPostion[TableInfo.TABLE_ID.name]=table_id
        dicPostion[MonitoringField.DI_POT.name]=di_pot
        dicPostion[TableInfo.SERVING_ANGLE.name]=cur_angle540
        dicPostion[TableInfo.MARKER_ANGLE.name]=cur_angle360
        dicPostion['cur_pos_mm']=cur_pos_mm
        dicPostion[TableInfo.SERVING_DISTANCE.name]=distance_tof
        dicPostion['currentAngle_arm1']=currentAngle_arm1
        dicPostion['onScaning']=onScaning()
        
        data_out2 = json.dumps(dicPostion)
        pub_BLB_POS.publish(data_out2)

        pub_BLB_CROSS.publish(data_out)
        cur_pos =  recvJunctionStatus.get(MonitoringField.CUR_POS.name, None)
        ipaddr = recvJunctionStatus.get(CALLBELL_FIELD.IP.name, None)
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
            dicCross = {MQTT_FIELD.TOPIC.name : MQTT_TOPIC_VALUE.MSG_CROSS_REQUEST.value,MonitoringField.LASTSEEN.name:getDateTime().timestamp(),
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
  
    return {"message": f"{request.method},{loaded_data}"}, 200


@app.route('/JOG', methods=['GET'])
def service_jog():
    global startPos
    global endnode
    #global epcTarget
    global lastNode
    global lastRSSI
    global isScan
    listReturnTmp=[]
    try:
        reqargs = request.args.to_dict()
        endnode_tmp = request.args.get(APIBLB_FIELDS_TASK.endnode.name)
        distance_tmp = request.args.get(APIBLB_FIELDS_TASK.distance.name)
        pulse_tmp = request.args.get(posStr)
        spd = try_parse_int(request.args.get(MotorWMOVEParams.SPD.name, DEFAULT_RPM_MID),-1)
        isAbsPos = True
        if spd < 0: 
            return {"error": "spd string is not valid."}, 400
        
        if endnode_tmp is None:
            return {"error": "endnode is required."}, 400
        
        # dic_CROSSINFO = shared_data.get(TopicName.CROSS_INFO.name)
        # if dic_CROSSINFO:
        #     chargeSensor = dic_CROSSINFO.get(SMARTPLUG_INFO.GPI1_CHARGE.name)
        #     if isTrue(chargeSensor):
        #         lastNode = 1
        
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
        # if is_equal(lastNode ,endnode):
        #     return {"error": "endnode is current node"}, 400
        epcnodeinfo = GetEPCNodeInfoDic()
        sEPC = get_key_by_value(epcnodeinfo, endnode)
        #dicPotNot=(dicWE_OFF)
        
        si_pot = GetMotorH_SI_POT()
        cur_posH = GetMotorHPos()        
        # if sEPC is None:
        #     if isScan:
        #         distancePulse=try_parse_int(pulse_tmp)
        #     else:
        #         dfScan = pd.read_csv(strFileEPC_scan, sep=sDivTab)
        #         avg_df = dfScan.groupby("NODE_ID", as_index=False)["POS"].mean()
        #         avg_df["POS"] = avg_df["POS"].round().astype(int)
        #         last_epc_df = dfScan.groupby("NODE_ID", as_index=False).last()[["NODE_ID", "EPC"]]
        #         result_df = pd.merge(avg_df, last_epc_df, on="NODE_ID")
        #         result_df.to_csv(strFileEPC_total, sep=sDivTab, index=False)
        #         return {"OK": "MapSaved"}, 200
        # else:
        #     epcTarget = sEPC
        #     #distancePulse = GetEPC_Loc_Master(sEPC)
        #     distancePulse=GetNodePos_fromEPC(sEPC)
        
        # if distance_tmp is None:
        #     distancePulse=pulse_tmp
        #     isAbsPos = True
        # else:
        #     distance = try_parse_int(distance_tmp,MIN_INT)
        #     distancePulse = distance_to_pulseH(distance)
        
        # SendCMDESTOP(f'I{ACC_DECC_MOTOR_H}')
        # time.sleep(MODBUS_EXCEPTION_DELAY)
        #distancePulse = distance_to_pulseH(distance)
        # lsSpecialNodes = GetSpeicificEPCNodeInfo()
        # if endnode in lsSpecialNodes:
        #     listReturnTmp.append(dicPOTNOT_ON)
        # else:
        #     listReturnTmp.append(dicPOTNOT_OFF)
        
        #isCrossRailRunOK = isCrossRailDirection()        
        
        distancePulseTarget = GetNodePos_fromNode_ID(endnode)
        if not isRealMachine:
            bResult,bExecuteMsg=SendCMD_Device([getMotorLocationSetDic(ModbusID.MOTOR_H.value,distancePulseTarget)])
            return {bResult: bExecuteMsg}, 200
        
        if distancePulseTarget is None:
            return {"ERR": "endNode not found"}, 400      
        distanceDiffSigned = distancePulseTarget-(cur_posH)
        distanceDiffAbs = abs(distanceDiffSigned)
        if distanceDiffSigned > 0:
            backlashPos = estimate_backlash_error(distanceDiffSigned)
            distancePulseTarget = distancePulseTarget - backlashPos
        dicMotorH = getMotorMoveDic(ModbusID.MOTOR_H.value,isAbsPos,distancePulseTarget,spd,ACC_DECC_MOTOR_H,ACC_DECC_MOTOR_H)
        
        if  distanceDiffAbs < roundPulse / 2 and si_pot != "ESTOP":
            dicPotNot = getMotorWP_ONDic()  if distancePulseTarget > cur_posH else getMotorWN_ONDic()
            listReturnTmp.append(dicPotNot)
            #dicPotNot = dicWE_ON
            callbackMB_15.ESTOP_ON = BLD_PROFILE_CMD.ESTOP.name
        else:
            listReturnTmp.append(getMotorWPN_OFFDic())
            callbackMB_15.ESTOP_ON = BLD_PROFILE_CMD.MOTORSTOP.name
        
        listReturnTmp.append(dicMotorH)
        if getChargerPlugStatus():
            SetChargerPlug(False)
            time.sleep(MODBUS_EXCEPTION_DELAY)        
        # if not getRFIDInvStatus():
        #     RFIDControl(True)
        #     time.sleep(MODBUS_EXCEPTION_DELAY)
        lastRSSI = None
        #startPos = try_parse_int(dicMotorPos.get('MB_15'),MIN_INT)
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

def RotateMotor360(control360,filter_rate,checkSafety, delayTime = 0):
    if delayTime > 0:
        delayTime = min(delayTime, delayTimeMin)
        rospy.loginfo(f"Delay {delayTime} sec")
        time.sleep(delayTime)
    bResult=True
    bExecuteMsg = AlarmCodeList.OK.name
    pos_BAL1,pos_BAL2,pos_LiftV,pos_540,pos_360,pos_ServArm,pos_MotorH = GetAllMotorPos()
    cur_angle360=GetRotateTrayAngleFromPulse(pos_360)
    spd360 = SPD_360
    if control360 == cur_angle360:
        return bResult,bExecuteMsg        
    di_pot,di_not,di_home,di_estop, si_pot,si_home=GetMotorSensor(topicName_RotateTray)
    if control360 >=0:
        targetPulse_serv = GetRotateTrayPulseFromAngle(control360)
        # diff_serv = (targetPulse_serv - pos_360)
        # if control360 == 0 and isRealMachine:
        #     if diff_serv > 0:
        #         targetPulse_serv += diff_roundPulse
        #     else:
        #         targetPulse_serv -= diff_roundPulse
    else:
        spd360=DEFAULT_RPM_SLOW
        if len(dicLastAruco)>0:
            marker_angle = round(dicLastAruco.get(ARUCO_RESULT_FIELD.ANGLE.name))
            angle_new = (180+ marker_angle)%360
            targetPulse_serv = GetRotateTrayPulseFromAngle(angle_new)
            #infoMsg = f'이전 각도:{cur_angle360},마커로 바뀐각도:{angle_new},마커정보:{json.dumps(dicLastAruco, indent=4)}'
            infoMsg = f'이전 각도:{cur_angle360},마커로 바뀐각도:{angle_new}'
            control360 = angle_new
            rospy.loginfo(infoMsg)
        elif isTrue(di_home):
            return bResult,ALM_User.CALI_ALREADY_DONE.value
        else:
            targetPulse_serv = pos_360 - pot_360
    dic360 = getMotorMoveDic(ModbusID.ROTATE_SERVE_360.value,True,targetPulse_serv,spd360,ACC_360_UP,DECC_360_UP)
    #print(dic540)
    lsCmd = [dic360]
    
    # if isTrue(di_home) and si_home == BLD_PROFILE_CMD.ESTOP.name:
    #     lsCmd.insert(0,getMotorWHOME_OFFDic(ModbusID.ROTATE_SERVE_360.value))
    # elif control360 < 0 or (control360 == 0 and not isTrue(di_home) and si_home != BLD_PROFILE_CMD.ESTOP.name):
    #     lsCmd.append(getMotorWHOME_ONDic(ModbusID.ROTATE_SERVE_360.value))
    bResult,bExecuteMsg=SendCMD_Device(lsCmd,filter_rate,checkSafety)
    #reponseCode = 200 if bResult else 400
    return bResult,bExecuteMsg

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
        checkSafety = isTrue(request.args.get(JogControl.SAFETY.name, 0))
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
                if control360 != MIN_INT:
                    bResult,bExecuteMsg=RotateMotor360(control360,filter_rate,checkSafety, max(rpm_time/2,delayTimeMin))
                reponseCode = 200 if bResult else 400
                #return {f'Set SetLightPlug to {controlArm3} -> Result': bResult}, reponseCode
            return {bResult:bExecuteMsg}, reponseCode
        elif control360 != MIN_INT:
            bResult,bExecuteMsg=RotateMotor360(control360,filter_rate,checkSafety)
            reponseCode = 200 if bResult else 400
            return {bResult:bExecuteMsg}, reponseCode    
        elif control540 != MIN_INT:            
            cur_angle540=abs(GetRotateMainAngleFromPulse(pos_540))
            spd540=SPD_540
            if control540 == cur_angle540:
                return {bResult:bExecuteMsg}, 200
            di_pot,di_not,di_home,di_estop, si_pot,si_home=GetMotorSensor(topicName_RotateMain)
            if control540 >=0:
                targetPulse_serv = GetRotateMainPulseFromAngle(control540)
                diff_serv = (targetPulse_serv - pos_540)
                # if control540 == 0 and isRealMachine:
                #     if diff_serv > 0:
                #         targetPulse_serv += diff_roundPulse
                #     else:
                #         targetPulse_serv -= diff_roundPulse
            else:
                if isTrue(di_home):
                    return {bResult:ALM_User.CALI_ALREADY_DONE.value}, 200                
                targetPulse_serv = pos_540 - pot_540
                spd540=60
            dic540 = getMotorMoveDic(ModbusID.ROTATE_MAIN_540.value,True,targetPulse_serv,spd540,ACC_540,DECC_540)
            #print(dic540)
            lsCmd = [dic540]
            if si_home == BLD_PROFILE_CMD.ESTOP.name:
                lsCmd.insert(0,getMotorWHOME_OFFDic(ModbusID.ROTATE_MAIN_540.value))
            elif control540 < 0:
            #elif control540 < 0 or (control540 == 0 and not isTrue(di_home) and si_home != BLD_PROFILE_CMD.ESTOP.name):
                lsCmd.append(getMotorWHOME_ONDic(ModbusID.ROTATE_MAIN_540.value))
            bResult,bExecuteMsg=SendCMD_Device(lsCmd,filter_rate,checkSafety)
            reponseCode = 200 if bResult else 400
            #return {f'Set SetLightPlug to {controlArm3} -> Result': bResult}, reponseCode
            return {bResult:bExecuteMsg}, 200
        
        elif controlArm2 != MIN_INT:
            #controlArm2 은 0~90 사이 각도로 넘어온다.
            currentAngle_arm1 =mapRange(pos_BAL1,0,pot_arm1,0,90)
            if currentAngle_arm1 == controlArm2:
                return {bResult:bExecuteMsg}, 200
            lsCmd,rpm_time,stroke_arm1_signed = GetControlInfoBalances(controlArm2,True, spd_rate)
            #print(lsCmd)
            bResult,bExecuteMsg=SendCMD_Device(lsCmd,filter_rate,checkSafety)
            reponseCode = 200 if bResult else 400
            #return {f'Set SetLightPlug to {controlArm3} -> Result': bResult}, reponseCode
            return {bResult:bExecuteMsg}, reponseCode
        
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
            #return {f'Set SetChargePlug to {plug_enable} -> Result': bResult}, reponseCode
            return {bResult:plug_enable}, reponseCode
        
        elif crossplug is not None:
            plug_enable = isTrue(crossplug)
            bResult = SetCrossPlug(plug_enable)
            reponseCode = 200 if bResult else 400
            #return {f'Set SetCrossPlug to {plug_enable} -> Result ': bResult}, reponseCode
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
            #return {"error": "No data received"}, 400
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
                #return {"message": f"{request.method},{qNumber}"}, 200
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
                #data_out = json.dumps(lsdfTable) 
                return lsdfTable,200
            elif qNumber == BLD_PROFILE_CMD.GET_NODEMAP.value:
                dfTableInfo = pd.read_csv(strFileEPC_total, sep=sDivTab)
                lsdfTable=dfTableInfo.to_dict(orient='records')
                #data_out = json.dumps(lsdfTable) 
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
        # if len(loaded_data) == 0:
        #     loaded_data['0'] = 'No Records'
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
        global dicLastAruco
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
            if len(data) > 0:
                dicData = data[0]
                marker_angle = round(dicData.get(ARUCO_RESULT_FIELD.ANGLE.name))
                if marker_angle is None:
                    angle_new = -1
                else:
                    angle_new = (180+ marker_angle)%360
                dicData[ARUCO_RESULT_FIELD.ANGLE_CALI.name] = angle_new
                dicLastAruco.update(dicData)
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
    
    # if topic_name == TopicName.RFID.name:
    #     print(data_pub)
    
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
    
def publish_loop():
    time.sleep(5) 
    while True:
        try:
            args_dict = {
                "RSSI": "-47",
                "TIME_JC": "19:44:52",
                "IP": "172.30.1.10",
                "BUS_VOLTAGE": "240",
                "DI_NA": "0",
                "DI_07": "0",
                "DI_06": "0",
                "DI_HOME": "0",
                "DI_ESTOP": "0",
                "DI_NOT": "0",
                "DI_POT": "0",
                "DI_01": "1",
                "CMD_SPD": "500",
                "ST_NA": "0",
                "ST_HOME_FINISH": "1",
                "ST_PATH_FINISH": "1",
                "ST_03": "0",
                "ST_RUNNING": "0",
                "ST_ENABLE": "1",
                "ST_FAULTY": "0",
                "ALM_CD": "0",
                "CMD_POS": "501870",
                "CUR_POS": "501865",
                "CUR_SPD": "-1"
            }

            # 자기 자신에게 GET 요청 보내기
            response = requests.get(f"http://localhost:{HTTP_COMMON_PORT}/CROSS", params=args_dict)
            #print("CROSS response:", response.status_code, response.json())
        except Exception as e:
            print("Error calling /CROSS:", e)
        time.sleep(2)

def parse_timestamp(value):
    if isinstance(value, str):
        try:
            value = float(value)
        except ValueError:
            return None        
    if isinstance(value, float):
        return datetime.fromtimestamp(value)
    return None

def find_lastseen(data):
    if isinstance(data, dict):
        for key, value in data.items():
            if key.upper() == "LASTSEEN":
                ts = parse_timestamp(value)
                if ts:
                    return ts
            elif isinstance(value, dict):
                result = find_lastseen(value)
                if result:
                    return result
    return None

def monitor_dict():
    global last_alarm_times
    while True:
        current_time = datetime.now()

        for device, data in shared_data.items():
            lastseen = find_lastseen(data)

            if lastseen:
                time_diff = (current_time - lastseen).total_seconds()
                last_alarm = last_alarm_times.get(device, datetime.min)
                suppress_elapsed = (current_time - last_alarm).total_seconds()

                if time_diff > ALERT_THRESHOLD_SECONDS and suppress_elapsed > ALARM_SUPPRESS_SECONDS:
                    print(f"⚠️  {device} has not been seen for {int(time_diff)} seconds.")
                    last_alarm_times[device] = current_time
                else:
                    print(f"✅ {device} is active. Last seen: {lastseen}. Time difference: {int(time_diff)} seconds.")

        time.sleep(CHECK_INTERVAL)

def start_background_thread():
    # thread_monitor = threading.Thread(target=monitor_dict)
    # thread_monitor.daemon = True  # Flask 종료 시 함께 종료
    # thread_monitor.start()

    if not isRealMachine:
        thread = threading.Thread(target=publish_loop)
        thread.daemon = True  # Flask 종료 시 함께 종료
        thread.start()

if __name__ == "__main__":
    try:
        rospy.init_node('node_API', anonymous=False)
        rospy.loginfo('node_API started')

        loaded_data2 = load_csv_to_dict(csvPathalarm, sort_ascending=False)
        shared_data[TopicName.HISTORY_ALARM.name] = loaded_data2

        threading.Thread(target=update_data, daemon=True).start()
        start_background_thread()

        # Flask + SocketIO run
        socketio.run(app, host="0.0.0.0", port=HTTP_COMMON_PORT, debug=False, use_reloader=False, log_output=True, allow_unsafe_werkzeug=True)

    except rospy.ROSInterruptException:
        rospy.logwarn("node_API 종료됨.")
    
# if __name__ == "__main__":
#     try:
#         rospy.init_node('node_API', anonymous=False)
#         rospy.loginfo('node_API started')
#         loaded_data2 = load_csv_to_dict(csvPathalarm, sort_ascending=False)
#         #data_out2 = json.dumps(loaded_data2)
#         shared_data[TopicName.HISTORY_ALARM.name] = loaded_data2
#         threading.Thread(target=update_data, daemon=True).start()        
#         start_background_thread()
#         socketio.run(app, host="0.0.0.0", port=HTTP_COMMON_PORT, debug=False, use_reloader=False, log_output=True,allow_unsafe_werkzeug=True)
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass