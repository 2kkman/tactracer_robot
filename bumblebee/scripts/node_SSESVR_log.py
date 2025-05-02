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
sEPCKey = RFID_RESULT.EPC.name
sRSSIKey = RFID_RESULT.RSSI.name
sPOS_Key = MonitoringField.CUR_POS.name
sLAST_TARGET_POS = MonitoringField.LAST_TARGET_POS.name
sST_CMD_FINISH = MonitoringField.ST_CMD_FINISH.name
sSPD_Key = MonitoringField.CUR_SPD.name
sSI_POT =MonitoringField.SI_POT.name
sDI_POT =MonitoringField.DI_POT.name
sInv_Key = RFID_RESULT.inventoryMode.name
sALIVE_Key = RFID_RESULT.status.name
# dicPOTNOT_ON = getMotorDefaultDic(ModbusID.MOTOR_H.value,True)
# dicPOTNOT_OFF = getMotorDefaultDic(ModbusID.MOTOR_H.value,False)
dicWE_ON = getMotorWE_ONDic()
dicWE_OFF = getMotorWE_OFFDic()
notagstr = RailNodeInfo.NOTAG.name
node_virtual_str = RailNodeInfo.NONE.name
isRealMachine = get_hostname().find(UbuntuEnv.ITX.name) >= 0
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')
shared_data = {topic.name: {} for topic in TopicName}
maxAlarmHistory = 500
maxRetryCount = 3
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
pub_RFID_DF = rospy.Publisher(TopicName.RFID_DF.name, String, queue_size=1)
target_pulse_cw = 50000000
ACC_DECC_MOTOR_H = 5000
ACC_DECC_NORMAL = 3000
findNodeTryPulse = 10000
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

dicSpdSlow = getMotorSpeedDic(ModbusID.MOTOR_H.value, True, DEFAULT_RPM_MID, ACC_DECC_MOTOR_H,ACC_DECC_MOTOR_H)
dicSpdFast = getMotorSpeedDic(ModbusID.MOTOR_H.value, True, DEFAULT_RPM_NORMAL, ACC_DECC_MOTOR_H,ACC_DECC_MOTOR_H)
        
dicArmExpand = dicAllExpand[:-1]
dicArmFold = dicAllFold[:-1]
dicMotorPos = {}
dicAlarmStatus = {}
dicAlarmCode = {}
startPos = 0
endnode= 0
epcTotalView = pd.DataFrame()
dfNodeInfo = pd.read_csv(strFileEPC_total, sep=sDivTab)
dicPotInfo = getDic_FromFile(filePath_CaliPotConfig)
pot_telesrv = dicPotInfo.get(str(ModbusID.TELE_SERV_MAIN.value))
pot_arm1 = dicPotInfo.get(str(ModbusID.BAL_ARM1.value))
pot_arm2 = dicPotInfo.get(str(ModbusID.BAL_ARM2.value))

#epcTarget = None
lastNode = None
lastRSSI = None
lastPos = None
lastAck = ''
isScan = False
isRetry = False
lastcalledAck = DATETIME_OLD
dicLastMotor15 = {}
dicPulsePos = {}
topicName_ServoPrefix = 'MB_'
topicName_MotorH = f'{topicName_ServoPrefix}{ModbusID.MOTOR_H.value}'
topicName_ServArm = f'{topicName_ServoPrefix}{ModbusID.TELE_SERV_MAIN.value}'
topicName_BAL1 = f'{topicName_ServoPrefix}{ModbusID.BAL_ARM1.value}'
topicName_BAL2 = f'{topicName_ServoPrefix}{ModbusID.BAL_ARM2.value}'
topicName_LiftV = f'{topicName_ServoPrefix}{ModbusID.MOTOR_V.value}'
topicName_RotateTray = f'{topicName_ServoPrefix}{ModbusID.ROTATE_SERVE_360.value}'
topicName_RotateMain = f'{topicName_ServoPrefix}{ModbusID.ROTATE_MAIN_540.value}'

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
SPD_EXTEND_ARM2 = getSpeedTableInfo(ModbusID.BAL_ARM2.value,SPEEDTABLE_FIELDS.SPD.name,adjustrate)
ACC_ARM2_EXTEND = getSpeedTableInfo(ModbusID.BAL_ARM2.value,SPEEDTABLE_FIELDS.ACC_CW.name,adjustrate)
ACC_ARM2_FOLD = getSpeedTableInfo(ModbusID.BAL_ARM2.value,SPEEDTABLE_FIELDS.ACC_CCW.name,adjustrate)
DECC_ARM2 = getSpeedTableInfo(ModbusID.BAL_ARM2.value,SPEEDTABLE_FIELDS.DECC_CW.name,adjustrate)

#11번 서빙 텔레스코픽 모터
ACC_ST = getSpeedTableInfo(ModbusID.TELE_SERV_MAIN.value,SPEEDTABLE_FIELDS.ACC_CW.name,adjustrate)
DECC_ST = getSpeedTableInfo(ModbusID.TELE_SERV_MAIN.value,SPEEDTABLE_FIELDS.DECC_CW.name,adjustrate)

#13번 1관절 모터
SPD_ARM1 = getSpeedTableInfo(ModbusID.BAL_ARM1.value,SPEEDTABLE_FIELDS.SPD.name,adjustrate)
ACC_ARM1 = getSpeedTableInfo(ModbusID.BAL_ARM1.value,SPEEDTABLE_FIELDS.ACC_CW.name,adjustrate)
DECC_ARM1 = getSpeedTableInfo(ModbusID.BAL_ARM1.value,SPEEDTABLE_FIELDS.DECC_CW.name,adjustrate)

#15번 주행 모터
ACC_MOVE_H = getSpeedTableInfo(ModbusID.MOTOR_H.value,SPEEDTABLE_FIELDS.ACC_CCW.name,SPEED_RATE_H)
DECC_MOVE_H = getSpeedTableInfo(ModbusID.MOTOR_H.value,SPEEDTABLE_FIELDS.ACC_CW.name,SPEED_RATE_H)
SPD_MOVE_H = getSpeedTableInfo(ModbusID.MOTOR_H.value,SPEEDTABLE_FIELDS.SPD.name,SPEED_RATE_H)

#27번 메인회전 모터
SPD_540 =  getSpeedTableInfo(ModbusID.ROTATE_MAIN_540.value,SPEEDTABLE_FIELDS.SPD.name)
ACC_540 = getSpeedTableInfo(ModbusID.ROTATE_MAIN_540.value,SPEEDTABLE_FIELDS.ACC_CW.name)
DECC_540 = getSpeedTableInfo(ModbusID.ROTATE_MAIN_540.value,SPEEDTABLE_FIELDS.DECC_CW.name)

#9번 밸런싱 텔레스코픽 모터
SPD_BALTELE = getSpeedTableInfo(ModbusID.TELE_BALANCE.value,SPEEDTABLE_FIELDS.SPD.name,adjustrate)
ACC_BT = getSpeedTableInfo(ModbusID.TELE_BALANCE.value,SPEEDTABLE_FIELDS.ACC_CW.name,adjustrate)
DECC_BT = getSpeedTableInfo(ModbusID.TELE_BALANCE.value,SPEEDTABLE_FIELDS.DECC_CW.name,adjustrate)

#31번 트레이 모터
SPD_360 =  getSpeedTableInfo(ModbusID.ROTATE_SERVE_360.value,SPEEDTABLE_FIELDS.SPD.name)
ACC_360_DOWN = getSpeedTableInfo(ModbusID.ROTATE_SERVE_360.value,SPEEDTABLE_FIELDS.ACC_CW.name)
DECC_360_DOWN = getSpeedTableInfo(ModbusID.ROTATE_SERVE_360.value,SPEEDTABLE_FIELDS.DECC_CW.name)
ACC_360_UP = getSpeedTableInfo(ModbusID.ROTATE_SERVE_360.value,SPEEDTABLE_FIELDS.ACC_CCW.name)
DECC_360_UP = getSpeedTableInfo(ModbusID.ROTATE_SERVE_360.value,SPEEDTABLE_FIELDS.DECC_CCW.name)

def GetControlInfoBalances(targetPulse_arm1,bUseCurrentPosition=False, spd_rate = 1.0):
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
    #서빙암 스트로크 RPM
    rpm_arm1 = round(SPD_ARM1 * spd_rate)
    round1CountAbs = round(stroke_arm1_abs/roundPulse)
    round2CountAbs = round(stroke_arm2_abs/roundPulse)
    rpm_time_arm1 = calculate_rpm_time(round1CountAbs, rpm_arm1)
    rpm_arm2 = calculate_targetRPM_fromtime(round2CountAbs, rpm_time_arm1)
    isExpand = stroke_arm1_signed > 0
    accArm2 = ACC_ARM2_EXTEND if isExpand else ACC_ARM2_FOLD
    dicArm1 = getMotorMoveDic(ModbusID.BAL_ARM1.value,True,targetPulse_arm1,rpm_arm1,ACC_ARM1,DECC_ARM1)
    dicArm2 = getMotorMoveDic(ModbusID.BAL_ARM2.value,True,targetPulse_arm2,rpm_arm2,accArm2,DECC_ARM2)
    return [dicArm1,dicArm2], rpm_time_arm1

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
        
    targetPulse_serv = GetTargetPulseServingArm(distanceServingTeleTotal, 0)
    stroke_servArm_signed = targetPulse_serv - cur_serv
    stroke_servArm_abs = abs(stroke_servArm_signed)
    targetPulse_arm1 = min(round(mapRange(targetPulse_serv,0,STROKE_MAX,0,pot_arm1)),pot_arm1)
    #서빙암 스트로크 RPM
    lsBal, rpm_time_arm1= GetControlInfoBalances(targetPulse_arm1,bUseCurrentPosition,spd_rate)
    servArmCountAbs = round(stroke_servArm_abs/roundPulse)    
    rpm_servArm = calculate_targetRPM_fromtime(servArmCountAbs, rpm_time_arm1)
    # 원본 값 보정
    #rpm_servArm = max(DEFAULT_RPM_SLOWER, min(rpm_servArm, DEFAULT_RPM_SLOW))
    #rpm_servArm = max(DEFAULT_RPM_SLOWER, min(rpm_servArm, DEFAULT_RPM_SLOW))
    dicSrvArm = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value,True,targetPulse_serv,rpm_servArm,ACC_ST,DECC_ST)
    lsBal.append(dicSrvArm)
    return lsBal

def getRFIDInvStatus():
    global shared_data    
    dicBLB_Status = shared_data.get(TopicName.RFID.name)
    return isTrue(dicBLB_Status.get(RFID_RESULT.inventoryMode.name))

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

def GetCurrentNode():
    global shared_data    
    dicBLB_Status = shared_data.get(TopicName.BLB_STATUS.name)
    return try_parse_int(dicBLB_Status.get(BLB_STATUS.NODE_CURRENT.name))

def GetMotorHPos():
    cur_pos = try_parse_int(dicMotorPos.get(topicName_MotorH))
    return cur_pos

def GetMotorH_SI_POT():
    cur_pos = try_parse_int(shared_data.get(topicName_MotorH),{})
    si_pot = cur_pos.get(sSI_POT,"")
    return si_pot

def GetMotorPot():
    di_pot = try_parse_int(dicLastMotor15.get(sDI_POT))
    return di_pot

def GetSpeicificEPCNodeInfo():
    local_epcnodeinfo = [GetNodeFromTable(HOME_CHARGE),GetNodeFromTable(HOME_TABLE)]
    state_keys: list[str] = list(GetCrossInfo().keys())
    local_epcnodeinfo.extend(state_keys)
    return local_epcnodeinfo

def handle_charge(loaded_data = {}):
        for k,v in ip_dict.items():
            if 'PLUG' in k and 'IP' in k:
                info = get_tasmota_info(v)
                if info is None:
                    continue
                kDevice = k[:-3]
                kDevice = kDevice[4:]
                for k2,v2 in info.items():
                    loaded_data[f'{kDevice}_{k2}'] = v2
                state = get_tasmota_state(v)
                loaded_data[f'{kDevice}_STATE'] = state
        
        data_out = json.dumps(loaded_data)
        pub_SMARTPLUG.publish(data_out)     
           

def rfidInstanceDefault():
    #global epcTarget
    global lastNode
    global lastRSSI
    global lastPos
    global endnode
    global startPos
    global lastcalledAck
    global isScan
    global isRetry
    global lastAck
    #epcTarget = None
    lastNode = None
    lastRSSI = None
    lastPos = None
    lastAck = ''
    isScan = False
    isRetry = False
    lastcalledAck =DATETIME_OLD
    callbackACK.retryCount = 100

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

def SendCMDESTOP(enable,isAlarm=True):
    rfidInstanceDefault()
    if isAlarm:
        PublishStateMessage(BLD_PROFILE_CMD.ESTOP.name)
    resultArd = service_setbool_client_common(ServiceBLB.CMD_ESTOP.value, enable, Kill)        
    if isRealMachine:
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

def SendCMDArd(enable):
    #isRealMachine = get_hostname().find(UbuntuEnv.ITX.name) >= 0
    if isRealMachine:
        resultArd = service_setbool_client_common(ServiceBLB.CMDARD_QBI.value, enable, Kill)
    else:
        enable = replace_string(enable)
        resultArd = service_setbool_client_common(ServiceBLB.CMDARD_ITX.value, enable, Kill)        
    time.sleep(MODBUS_WRITE_DELAY)        
    return resultArd

def SendCMD_Device(sendbuf, cmdIntervalSec=5):
    if not hasattr(SendCMD_Device, "last_cmd_time"):
        SendCMD_Device.last_cmd_time = 0
    if not hasattr(SendCMD_Device, "last_cmd_msg"):
        SendCMD_Device.last_cmd_msg = None
    now = time.time()
    cmdTmp = sendbuf
    if isinstance(sendbuf, list):
        if len(sendbuf) > 0:
            cmdTmp = json.dumps(sendbuf)
            if cmdTmp == SendCMD_Device.last_cmd_msg:
                # 같은 메시지면 쿨타임 검사
                if now - SendCMD_Device.last_cmd_time < cmdIntervalSec:
                    return False    
            SendCMD_Device.last_cmd_time = now
            SendCMD_Device.last_cmd_msg = cmdTmp
        else:
            return False
    #log_all_frames(sendbuf)
    rfidInstanceDefault()
    return service_setbool_client_common(ServiceBLB.CMD_DEVICE.value, cmdTmp, Kill)

lsRotateMotors = [str(ModbusID.ROTATE_MAIN_540.value), str(ModbusID.ROTATE_SERVE_360.value)]
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
        flagFinished = lsResult[1]  # 0 이면 미완료, 1 이면 완료
        stopped_pos=GetMotorHPos()
        if not hasattr(callbackACK, "retryCount"):
                callbackACK.retryCount = maxRetryCount
        
        if not isTrue(flagFinished):
            #callbackACK.retryCount = 0
            return
        
        #lastcalledAck = getDateTime()
        if len(lsResult) >= 13:
            #print(lsResult)
            torque_max = lsResult[3]  # 최대토크
            torque_ave = lsResult[4]  # 평균토크
            ovr_max = lsResult[5]  # 최대오버로드
            ovr_ave = lsResult[6]  # 평균오버로드
            last_started_pos = int(lsResult[7])  # 운행 시작 지점
            last_targeted_pos = int(lsResult[8])  # 운행 종료 목표 지점
            stopped_pos = int(lsResult[9])  # 현재 지점
            last_spd = int(lsResult[10])  # 정지시점 속도 및 방향
            isHome = lsResult[11]  # HOME에 걸려서 멈췄으면 1
            isPot = lsResult[12]  # POT에 걸려서 멈췄으면 1
            
        # 회전모터 (27,31) 자동 상시 캘리브레이션.
        # 회전모터가 홈센서에 걸려서 멈췄을 경우 0 으로 위치값을 초기화 한다
        if isTrue(flagFinished) and isTrue(isHome):
            if mbid_tmp in lsRotateMotors:
                lsCmdCaliHome = [getMotorWHOME_OFFDic(mbid_tmp),getMotorHomeDic(mbid_tmp)]
                SendCMD_Device(lsCmdCaliHome)
        
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
        dicCurNodeInfo=GetCurrentNodeDicFromPulsePos(dfNodeInfo,stopped_pos)        
        #print(lsCurNode)
        curNodeID_fromPulse = dicCurNodeInfo.get(TableInfo.NODE_ID.name)
        curNode_type = str(dicCurNodeInfo.get(RFID_RESULT.EPC.name))
        rospy.loginfo(f'운행완료.지시노드:{endnode},직전노드:{lastNode},현재노드:{curNodeID_fromPulse}:{curNode_type}')
        rospy.loginfo(f'최대토크:{torque_max},평균토크:{torque_ave},최대부하:{ovr_max},평균부하:{ovr_ave},시작위치:{last_started_pos},지시위치:{last_targeted_pos}:,정지위치:{stopped_pos},속도:{last_spd},POT:{isPot}')
        #목적지 노드 타입 확인 - 유효한 EPC이면 RFID노드, NONE 면 가상노드, NOTAG 면 도그 노드.
        
        
        #도그 노드, 혹은 특수 노드인데 도달하지 못 했으면 댐핑으로 추가 운행시도.
        if isScan or isTrue(isPot):
            # dfEPCTotal.to_csv(strFileEPC_total, sep=sDivTab, index=False)
            # isScan = False
            # print(strFileEPC_total)
            # rospy.loginfo(f'Scan End :{cur_pos}')
            # dfEPCTotal.drop(dfEPCTotal.index, inplace=True) 
            #RFIDControl(False)
            if curNodeID_fromPulse in NODES_SPECIAL and curNodeID_fromPulse == endnode:
                endNodePos = GetNodePos_fromNode_ID(endnode)
                dicLoc = getMotorLocationSetDic(ModbusID.MOTOR_H.value, endNodePos)
                SendCMD_Device([dicWE_OFF,dicLoc,dicWE_ON])
                #TTSAndroid('위치오차를 보정합니다',1)
                
            RFIDControl(False)
            rfidInstanceDefault()
            return
        
        if curNode_type.find(notagstr) < 0 and curNodeID_fromPulse == endnode:
        #if (curNode_type == 'NONE' or len(curNode_type) == 24) and curNodeID_fromPulse == endnode:
            RFIDControl(False)
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
            dicJOG = getMotorMoveDic(ModbusID.MOTOR_H.value,True,finalPos ,DEFAULT_RPM_SLOWER,ACC_DECC_MOTOR_H,ACC_DECC_MOTOR_H)
            rospy.loginfo(f'다시탐색{callbackACK.retryCount}:{dicJOG}')
            SendCMD_Device([dicJOG])
        else:
            # SendCMDESTOP(f'I{ACC_DECC_MOTOR_H}', False)
            # message = '노드 위치 계산 에러입니다.'
            # SendAlarmHTTP(message,True,BLB_ANDROID_IP_DEFAULT)
            rfidInstanceDefault()
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

def callbackMB_15(recvDataMap):
    global lastNode
    global lastPos
    global endnode
    global isScan
    global lastcalledAck
    global dicLastMotor15
    global dicPulsePos  #도그 신호가 들어올때 엔코더 값을 기록해두고 보정수치로 쓴다
    dicLastMotor15.update(recvDataMap)
    # if not hasattr(callbackMB_15, "retryCount"):
    #         callbackMB_15.retryCount = 0        
    
    try:
        if not isTimeExceeded(lastcalledAck, MODBUS_EXECUTE_DELAY_ms):
            return
        
        isST_CMD_FINISH = recvDataMap.get(sST_CMD_FINISH)
        if isTrue(isST_CMD_FINISH):
            return
        
        target_pos = try_parse_int(recvDataMap.get(sLAST_TARGET_POS))
        if target_pos == MIN_INT:
            return

        isCrossRailRunOK = isCrossRailDirection()
        sSPD_signed = try_parse_int(recvDataMap.get(sSPD_Key))
        sSPD_abs = abs(sSPD_signed)
        sPOS = try_parse_int(recvDataMap.get(sPOS_Key))
        si_pot = recvDataMap.get(sSI_POT,"")
        di_pot_status = recvDataMap.get(sDI_POT,"")
        
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
        dicCurNodeInfo=GetCurrentNodeDicFromPulsePos(dfNodeInfo,sPOS)
        if not dicCurNodeInfo:
            return
        
        curNodeID_fromPulse = dicCurNodeInfo.get(TableInfo.NODE_ID.name)
        curNode_type = str(dicCurNodeInfo.get(RFID_RESULT.EPC.name))
        curNode_pos = int(dicCurNodeInfo.get(MotorWMOVEParams.POS.name))
        lastNode = curNodeID_fromPulse
        endNodePos = GetNodePos_fromNode_ID(endnode)
        # lsEndNode = find_nearest_pos(dfNodeInfo,endNodePos,1,0)
        # dicEndNodeInfo = lsEndNode[0]
        dicEndNodeInfo=GetCurrentNodeDicFromPulsePos(dfNodeInfo,endNodePos)        
        endNode_type = str(dicEndNodeInfo.get(RFID_RESULT.EPC.name))
        curNodePosMaster = GetNodePos_fromNode_ID(curNodeID_fromPulse)
        if isTrue(di_pot_status):
            dicPulsePos[curNodeID_fromPulse] = sPOS
            rospy.loginfo(f'!도그감지:{curNodeID_fromPulse},현재위치마스터:{curNodePosMaster},현재위치실측:{sPOS},엔코더오차:{curNodePosMaster-sPOS}')

        dicSpdControl = {}
        if curNode_type.find(RailNodeInfo.R_END.name) >= 0:
            if sSPD_signed > 0 and sSPD_abs > 100:
                dicSpdControl.update(dicSpdFast)
            if sSPD_signed < 0 and sSPD_abs > 100:
                dicSpdControl.update(dicSpdSlow)
        elif curNode_type.find(RailNodeInfo.R_START.name) >= 0:
            if sSPD_signed < 0:
                dicSpdControl.update(dicSpdFast)
            else:
                dicSpdControl.update(dicSpdSlow)
        if abs(curNode_pos - sPOS) < roundPulse/2 or isTrue(di_pot_status):
            rospy.loginfo(f'@현재노드:{curNodeID_fromPulse},현재위치마스터:{curNodePosMaster},현재위치실측:{sPOS},엔코더오차:{curNodePosMaster-sPOS},목표위치:{target_pos},노드속성:{endNode_type},모드:{si_pot},현재속도:{sSPD_signed}')
            if dicSpdControl:
                SendCMD_Device([dicSpdControl])
                
        #현재 노드와 목적 노드가 같으면 별다른 조치 없음. CallbackAck 에서 처리.
        #다른 경우 내 노드가 목적지 노드와 인접한 노드인지 확인해야 함.
        #node_near = 

        listReturnTmp = []
        #목적지가 충전소면 NOT활성화.        
        # dicSpd = getMotorSpeedDic(ModbusID.MOTOR_H.value, True, DEFAULT_RPM_SLOW, ACC_DECC_LONG,ACC_DECC_LONG)
        # potnotControlDic = getMotorWP_ONDic() if sSPD_signed > 0 else getMotorWN_ONDic()
        
        #목적지가 충전소 일땐 NOT 활성
        # if endnode == NODE_KITCHEN and si_pot != "NOT":
        #     listReturnTmp.append(getMotorWN_ONDic())
        #     rospy.loginfo('#목적지가 충전소 일땐 NOT 활성')
        # #목적지가 분기기이고,충전소에서 출발하는 경우는 POT 활성
        # elif endnode == NODE_CROSS:
        #     if not isCrossRailRunOK:
        #         if si_pot != "POT" and sSPD_signed > 0:
        #             listReturnTmp.append(potnotControlDic)
        #             rospy.loginfo('#목적지가 분기기이고,충전소에서 출발하는 경우는 POT 활성')
        # else:   #그 외에는 움직이는 방향에 따라 좌우됨.
        #그외에는 모두 댐핑이며 추후 구현.
#목적지까지 10만 펄스 이내가 아니면 리턴.
        if not is_within_range(target_pos,sPOS, roundPulse*8):
            return
        if endNode_type.find(notagstr) < 0:
            return
        rospy.loginfo(f'#엔코더오차:{curNodePosMaster-sPOS},현재속도:{sSPD_signed},현재위치:{sPOS},목표위치:{target_pos},모드:{si_pot},노드속성:{endNode_type}')
        if si_pot != "ESTOP" and abs(sSPD_signed) > 10:
            listReturnTmp.append(dicWE_ON)
            rospy.loginfo('ESTOP Enabled')
        # if si_pot != "POT" and sSPD_signed > 0 and abs(sSPD_signed) > 10:
        #     listReturnTmp.append(getMotorWP_ONDic())
        #     rospy.loginfo('POT ON')
        # elif si_pot != "NOT" and sSPD_signed < 0 and abs(sSPD_signed) > 10:
        #     listReturnTmp.append(getMotorWN_ONDic())
        #     rospy.loginfo('NOT ON')
        if len(listReturnTmp) > 0:
            SendCMD_Device(listReturnTmp)
            lastcalledAck = getDateTime()
        #목적펄스까지 
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        SendAlarmHTTP(message,True,BLB_ANDROID_IP_DEFAULT)

def callbackRFID(recvDataMap):
    #global epcTarget
    global lastNode
    global lastRSSI
    global lastPos
    global endnode
    global isScan
    global epcTotalView
    global dfNodeInfo
    needSave = False
    #RFID 부분은 없앰
    #현재 들어온 포지션으로 노드 계산
    #계산된 노드가 목표노드와 인접한 노드인지 확인
    #목표노드가 상대노드(도그근처에 가상으로 만든 노드)면 TARGET_POS 를 현재 포지션과 인접한 노드 마스터 정보의
    #포지션과 비교하여 엔코더 오차 보정하여 보정명령어 송출.
    #진짜 노드인 경우는 감속
    try:
        sEPC = recvDataMap.get(sEPCKey)
        sRSSI = abs(try_parse_int(recvDataMap.get(sRSSIKey,0)))
        sPOS = try_parse_int(recvDataMap.get(sPOS_Key))
        sSPD = (try_parse_int(recvDataMap.get(sSPD_Key)))
        sInv = recvDataMap.get(sInv_Key,notagstr)
        sSPDAbs = abs(sSPD)
        epcnodeinfo = GetEPCNodeInfoDic()
        curNode = epcnodeinfo.get(sEPC)
        if sEPC is None:
            if isScan:
                return
            sEPC = sInv
            sRSSI = 0
            sSPDAbs = 0
            recvDataMap[sEPCKey] = notagstr
        else:
            recvDataMap[sInv_Key] = 'on'
        if sRSSI > RSSI_FILTER:
            return

        if recvDataMap.get(sALIVE_Key) is not None:
            #스마트 플러그값 폴링
            handle_charge()
            return
        
        #dfEPCTotal = pd.read_csv(strFileEPC_total, sep=sDivTab)        
        if isScan:
            dfScan = pd.read_csv(strFileEPC_scan, sep=sDivTab)
            epcnodeinfo2 = df_to_dict_int_values(dfScan, MAPFIELD.EPC.name, TableInfo.NODE_ID.name)
            result = get_missing_or_next(list(epcnodeinfo2.values()))
            if epcnodeinfo2.get(sEPC) is not None:
                result=epcnodeinfo2.get(sEPC)
            
            #기준이 되는 노드는 업데이트 하지 않는다.
            if result in NODES_SPECIAL:
                return
            
            dicEPCNodeInfo = {TableInfo.NODE_ID.name : int(result) ,MotorWMOVEParams.POS.name : sPOS,MAPFIELD.EPC.name: sEPC}
            dfScan = insert_or_update_row_to_csv(strFileEPC_scan, sDivTab, dicEPCNodeInfo,MotorWMOVEParams.POS.name)
            print(dfScan)
            # df_json = df.to_json(orient="records")  # DataFrame -> JSON 문자열 변환
            # pub_RFID_DF.publish(df_json)
            return                    
        
        if sRSSI != 0:
            epc_column = MAPFIELD.EPC.name  # 예: 'EPC'

            if epc_column in epcTotalView.columns:
                filtered_df = epcTotalView[epcTotalView[epc_column] == str(sEPC)]
                if not filtered_df.empty:
                    result_dict = filtered_df.tail(1).to_dict(orient='records')[0]
                else:
                    result_dict = {}
            else:
                result_dict = {}  # EPC 컬럼 자체가 없음

            #df = insert_or_update_row_to_df(epcTotalView, recvDataMap, MonitoringField.CUR_POS.name)
            df = insert_or_update_row_to_df(epcTotalView, recvDataMap, sEPCKey)
            df_json = df.to_json(orient="records")  # DataFrame -> JSON 문자열 변환
            pub_RFID_DF.publish(df_json)
            
            long_values = df[df[sEPCKey].astype(str).str.len() >= 6][sEPCKey].tolist()
            if len(long_values) > 1:   
                epcTotalView.drop(epcTotalView.index, inplace=True)

            # if not is_equal(result_dict.get(MAPFIELD.EPC.name), sEPC):
            #     epcTotalView.drop(epcTotalView.index, inplace=True)
        
            if curNode is None:
                result = get_missing_or_next(list(epcnodeinfo.values()))
                epcnodeinfo[sEPC] = curNode = endNode = result
                #curNode = endNode = result
                #epcTarget = sEPC
                endNode = curNode
                saveDic_ToFile(epcnodeinfo,strFileEPC_node, sDivTab,True)
                rospy.loginfo(f'New EPC : {sEPC},Node:{curNode}, POS:{sPOS}')                
                needSave = True
            recvDataMap[TableInfo.NODE_ID.name] = curNode
        elif sSPDAbs == 0 and not isScan:
            df = insert_or_update_row_to_df(epcTotalView, recvDataMap, MAPFIELD.EPC.name)
            df_json = df.to_json(orient="records")  # DataFrame -> JSON 문자열 변환
            pub_RFID_DF.publish(df_json)
            return        
        if isScan:
        #if epcTarget is None or isScan:
            return
        
        # lastNode = curNode        
        # endNode = epcnodeinfo.get(epcTarget)
        # if curNode == endNode:
        #     if lastRSSI is None:
        #         lastRSSI = sRSSI
        #         lastPos = sPOS
        #         rospy.loginfo(f'lastRSSI init : {lastRSSI},POS:{lastPos},SPD:{sSPD}')
        #         return
            
        #     if sRSSI <= lastRSSI:
        #         lastRSSI = sRSSI
        #         lastPos = sPOS
        #         rospy.loginfo(f'lastRSSI updated : {lastRSSI},POS:{lastPos},SPD:{sSPD}')
        #         return
            
        #     if sSPD > 0:
        #         finalPos = lastPos+(sSPD*10*3)            
        #     else:
        #         finalPos = lastPos+(sSPD*10*4)            
            
        #     dicJOG = getMotorMoveDic(ModbusID.MOTOR_H.value,True,finalPos ,sSPDAbs,ACC_DECC_MOTOR_H,ACC_DECC_MOTOR_H)
        #     dicEPCNodeInfo = {TableInfo.NODE_ID.name : lastNode,MotorWMOVEParams.POS.name : finalPos,MAPFIELD.EPC.name: sEPC}
        #     # if needSave:
        #     #     #df = insert_row_to_csv(strFileEPC_total, sDivTab, dicEPCNodeInfo)
        #     #     df = insert_or_update_row_to_csv(strFileEPC_total, sDivTab, dicEPCNodeInfo, MAPFIELD.EPC.name)            
        #     #     print(df)
        #     #SendCMDESTOP(f'I{sSPD}')
        #     SendCMD_Device([dicJOG])
        #     epcTarget = None
        #     lastRSSI = None
        #     lastPos = None
        #     time.sleep(MODBUS_EXCEPTION_DELAY)
        #     RFIDControl(False)
        #     log_all_frames(f"Stop at {endNode},RFID:{sEPC},RSSI:{sRSSI},finalPos:{finalPos},SPD:{sSPD}")
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

            if topic_name == TopicName.RFID.name:
                callbackRFID(recvDataMap)
            
            if topic_name == topicName_MotorH:
                callbackMB_15(recvDataMap)
            
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
                        dicMotorPos[f'{topic_name}_STATE'] = recvDataMap[MonitoringField.ALM_NM.name]
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
        table_id_delete = request.args.get('DELETE_TABLE_ID')
        table_id_add = request.args.get(TableInfo.TABLE_ID.name, None)  
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
        return {"error": str(e)}, 500
  
    return {"message": f"{request.path},{immutable_multi_dict_to_dict(request.args)}"}, 200

@app.route(f'/{EndPoints.ARD.name}', methods=['GET'])
def service_ard():
    try:
        tmpHalf = request.args.get('q', None)
        tiltAngle = try_parse_int(request.args.get('tilt', None),MIN_INT)
        if tiltAngle != MIN_INT:   
            ardmsg = round(mapRange(tiltAngle, minGYRO,maxGYRO,0,180))
            if tiltAngle != MIN_INT:
                SendCMDArd(f'S:10,{ardmsg}')
        elif tmpHalf is None:
            return {"error": "No data received"}, 400
        else:
            SendCMDArd(tmpHalf)
    except Exception as e:
        logSSE_error(traceback.format_exc())
        rospy.logerr(f"데이터 처리 오류: {e}")
        return {"error": str(e)}, 500
  
    return {"message": f"{request.method},{request.args}"}, 200  
  
@app.route('/DATA2', methods=['GET'])
def service_data():
    resultData = ''
    try:
        topic_name = request.args.get('topicname', None)        
        if topic_name is None:
            return {"error": "topicname is required."}, 400
        resultData = shared_data.get(topic_name, None)
        if resultData is None:
            return {"error": "wrong topicname."}, 400
        resultData = json.dumps(resultData, ensure_ascii=False)

    except Exception as e:
        rospy.logerr(f"데이터 처리 오류: {e}")
        logSSE_error(traceback.format_exc())
        return {"error": str(e)}, 500
        
    return jsonify(resultData), 200
  
@app.route('/DATA1', methods=['GET'])
def service_data2():
    resultData = ''
    try:
        topic_name = request.args.get('topicname', None)        
        if topic_name is None:
            return {"error": "topicname is required."}, 400
        resultData = shared_data.get(topic_name, None)
        if resultData is None:
            return {"error": "wrong topicname."}, 400
        resultData = json.dumps(resultData, ensure_ascii=False)

    except Exception as e:
        logSSE_error(traceback.format_exc())
        rospy.logerr(f"데이터 처리 오류: {e}")
        return {"error": str(e)}, 500
  
    return (resultData), 200
  
@app.route('/CROSS', methods=['GET'])
def service_ard2():
    try:
        #rospy.loginfo(request.args)
        loaded_data=immutable_multi_dict_to_dict(request.args)
        data_out = json.dumps(loaded_data)
        pub_BLB_CROSS.publish(data_out)
        cur_pos =  loaded_data.get(MonitoringField.CUR_POS.name, None)
        isPOT = loaded_data.get(MonitoringField.DI_POT.name, None)
        isNOT = loaded_data.get(MonitoringField.DI_NOT.name, None)
        ipaddr = loaded_data.get(CALLBELL_FIELD.IP.name, None)
        devID = GetLastString(ipaddr, ".")
        if cur_pos is not None:
            cur_pos = int(cur_pos)
            status = -1
            if cur_pos < roundPulse:
                status = 1
            if cur_pos > 490000:
                status = 0
            # if isTrue(isNOT) or cur_pos < roundPulse:
            #     status = 0
            # if isTrue(isPOT) or cur_pos > 490000:
            #     status = 1
            dicCross = {MQTT_FIELD.TOPIC.name : MQTT_TOPIC_VALUE.MSG_CROSS_REQUEST.value,MonitoringField.LASTSEEN.name:getCurrentTime(),
                        MQTT_FIELD.PAYLOAD.name : {devID : status}}
            data_out = json.dumps(dicCross)
            pub_RECEIVE_MQTT.publish(data_out)            
            #TopicName.RECEIVE_MQTT.name
        
    except Exception as e:
        logSSE_error(traceback.format_exc())
        rospy.logerr(f"데이터 처리 오류: {e}")
        return {"error": str(e)}, 500
  
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
        return {"error": str(e)}, 500
  
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
        pulse_tmp = request.args.get(MotorWMOVEParams.POS.name)
        spd = try_parse_int(request.args.get(MotorWMOVEParams.SPD.name, DEFAULT_RPM_MID),-1)
        isAbsPos = True
        if spd < 0: 
            return {"error": "spd string is not valid."}, 400
        
        # if distance_tmp is None and pulse_tmp is None:
        #     return {"error": "distance and pulse is required."}, 400
        
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
        dicPotNot=(dicWE_OFF)
        
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
        if distancePulseTarget is None:
            return {"ERR": "endNode not found"}, 400      
          
        dicMotorH = getMotorMoveDic(ModbusID.MOTOR_H.value,isAbsPos,distancePulseTarget,spd,ACC_DECC_MOTOR_H,ACC_DECC_MOTOR_H)
        distanceDiffSigned = distancePulseTarget-(cur_posH)
        distanceDiffAbs = abs(distanceDiffSigned)
        
        if  distanceDiffAbs < roundPulse / 2 and si_pot != "ESTOP":
            #dicPotNot = getMotorWP_ONDic()  if distancePulseTarget > cur_posH else getMotorWN_ONDic()
            dicPotNot = dicWE_ON
        listReturnTmp.append(dicPotNot)
        listReturnTmp.append(dicMotorH)
        if getChargerPlugStatus():
            SetChargerPlug(False)
            time.sleep(MODBUS_EXCEPTION_DELAY)        
        if not getRFIDInvStatus():
            RFIDControl(True)
            time.sleep(MODBUS_EXCEPTION_DELAY)
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
        SendCMD_Device(listReturnTmp)
        return {"OK": sMsg}, 200
    except Exception as e:
        SendCMDESTOP(f'I{ACC_DECC_SMOOTH}')
        rospy.logerr(f"Invalid data:{reqargs},{e}")
        logSSE_error(traceback.format_exc())
        return {str(e): dicMotorPos }, 500

@app.route(f'/{ServiceBLB.CMD_DEVICE.name}', methods=['GET'])
def service_cmd():
    listReturn = []
    try:
        reqargs = request.args.to_dict()
        param1 = MIN_INT
        param2 = MIN_INT
        posMsg = json.dumps(dicMotorPos, sort_keys=True)
        crossplug = request.args.get(SMARTPLUG_INFO.SET_CROSSPLUG.name, None)
        chargeplug = request.args.get(SMARTPLUG_INFO.SET_CHARGERPLUG.name, None)
        lightplug = request.args.get(SMARTPLUG_INFO.SET_LIGHTPLUG.name, None)
        spd_rate = try_parse_float(request.args.get(JogControl.SPD_RATE.name, None),1.0)
        controlArm3 = try_parse_int(request.args.get(JogControl.CONTROL_3ARMS.name, None),MIN_INT)
        controlArm2 = try_parse_int(request.args.get(JogControl.CONTROL_2ARMS_ANGLE.name, None),MIN_INT)
        qNumber = request.args.get('q', MIN_INT)
        recvData = request.args.get('data')
        topicData = request.args.get('topicname')
        
        if controlArm3 != MIN_INT:
            lsCmd = GetControlInfoArms(controlArm3,True,spd_rate)
            print(lsCmd)
            bResult=SendCMD_Device(lsCmd)
            reponseCode = 200 if bResult else 400
            #return {f'Set SetLightPlug to {controlArm3} -> Result': bResult}, reponseCode
            return {f"{bResult}": json.dumps(lsCmd, sort_keys=True)}, 200
        
        if controlArm2 != MIN_INT:
            target_pulse=mapRange(controlArm2,0,90,0,pot_arm1)
            lsCmd,rpm_time = GetControlInfoBalances(target_pulse,True, spd_rate)
            print(lsCmd)
            bResult=SendCMD_Device(lsCmd)
            reponseCode = 200 if bResult else 400
            #return {f'Set SetLightPlug to {controlArm3} -> Result': bResult}, reponseCode
            return {bResult:rpm_time}, 200
        
        if lightplug is not None:
            plug_enable = isTrue(lightplug)
            bResult = SetLightPlug(plug_enable)
            reponseCode = 200 if bResult else 400
            return {f'Set SetLightPlug to {plug_enable} -> Result': bResult}, reponseCode
        
        elif chargeplug is not None:
            plug_enable = isTrue(chargeplug)
            bResult = SetChargerPlug(plug_enable)
            reponseCode = 200 if bResult else 400
            return {f'Set SetChargePlug to {plug_enable} -> Result': bResult}, reponseCode
        
        elif crossplug is not None:
            plug_enable = isTrue(crossplug)
            bResult = SetCrossPlug(plug_enable)
            reponseCode = 200 if bResult else 400
            return {f'Set SetCrossPlug to {plug_enable} -> Result ': bResult}, reponseCode
        
        if topicData is not None:
            responseData = shared_data.get(topicData)
            if topic_name == TopicName.JOB_DF.name:
                print(type(responseData))
            if responseData is None:
                return {f"Topic not found({topicData})": posMsg}, 400    
            else:
                return {f"{reqargs}": json.dumps(responseData, sort_keys=True)}, 200
        
        if qNumber == MIN_INT and recvData is None:
            #return {"error": "No data received"}, 400
            return {"No data received": posMsg}, 400
        if recvData is None:
            cmdNumber=try_parse_int(qNumber,MIN_INT)
            if cmdNumber == MIN_INT:
                params = qNumber.split(sep=sDivItemComma)
                param1 = params[1]
                qNumber = int(params[0])
            else:
                qNumber = (cmdNumber)
            if qNumber == BLD_PROFILE_CMD.ESTOP.value:
                SendCMDESTOP(f'I{ACC_DECC_SMOOTH}',False)
                return {f"{reqargs}": dicMotorPos}, 200
            elif qNumber == BLD_PROFILE_CMD.MOTORSTOP.value:
                SendCMDESTOP(f'I{ACC_DECC_SMOOTH}')
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
                    # 홈센서에 NOT 을 달아서 충전독 센서가 필요 없어짐
                    # dicSmartPlug = shared_data.get(TopicName.SMARTPLUG_INFO.name)
                    # chargeSensorinfo = dicSmartPlug.get(SMARTPLUG_INFO.GPI1_CHARGE.name)
                    # if isTrue(chargeSensorinfo):
                    #     return {f"err: Jog Back is now allowed at Charging Station": dicMotorPos}, 400
                    # else:
                    #     listReturn.append(dicBackHome)
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
            elif qNumber == BLD_PROFILE_CMD.GET_TABLEMAP.value:
                dfTableInfo = pd.read_csv(strFileTableNodeEx, sep=sDivTab)
                lsdfTable=dfTableInfo.to_dict(orient='records')
                data_out = json.dumps(lsdfTable) 
                return data_out,200
        else:
            recvDataTmp = getDic_strArr(recvData.upper(), sDivFieldColon, sDivItemComma)
            PROFILE = recvDataTmp.get(BLB_CMD_CUSTOM.PROFILE.name)
            if PROFILE is None:
                listReturn.append(recvDataTmp)
            else:
                cmdProfile= PROFILE.replace(sDivEmart,sDivItemComma)
                pub_BLB_CMD.publish(recvData.upper())
                return {f"Not defined code {qNumber}": dicMotorPos}, 202
        if len(listReturn) > 0:
            SendCMD_Device(listReturn)
        else:
            data_out = f'PROFILE{sDivFieldColon}{qNumber}'
            #data_out = json.dumps(reqargs) 
            pub_BLB_CMD.publish(data_out)
            return {f"Not defined code {qNumber}": dicMotorPos}, 202
    except Exception as e:
        SendCMDESTOP(f'I{ACC_DECC_SMOOTH}',False)
        rospy.logerr(f"Invalid data:{reqargs},{e}")
        logSSE_error(traceback.format_exc())
        return {str(e): dicMotorPos }, 500
  
    return {f"{reqargs}": dicMotorPos}, 200
  
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
        return {"error": str(e)}, 500
  
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
        return {"error": str(e)}, 500

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

if __name__ == "__main__":
    try:
        loaded_data2 = load_csv_to_dict(csvPathalarm, sort_ascending=False)
        #data_out2 = json.dumps(loaded_data2)
        shared_data[TopicName.HISTORY_ALARM.name] = loaded_data2
        threading.Thread(target=update_data, daemon=True).start()        
        socketio.run(app, host="0.0.0.0", port=HTTP_COMMON_PORT, debug=False, use_reloader=False, log_output=True,allow_unsafe_werkzeug=True)
    except rospy.ROSInterruptException:
        pass
    
    