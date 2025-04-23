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
sSPD_Key = MonitoringField.CUR_SPD.name
sInv_Key = RFID_RESULT.inventoryMode.name
dicPOTNOT_ON = getMotorDefaultDic(ModbusID.MOTOR_H.value,True)
dicPOTNOT_OFF = getMotorDefaultDic(ModbusID.MOTOR_H.value,False)

isRealMachine = get_hostname().find(UbuntuEnv.ITX.name) >= 0
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')
shared_data = {topic.name: {} for topic in TopicName}
maxAlarmHistory = 500
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

dicArmExpand = dicAllExpand[:-1]
dicArmFold = dicAllFold[:-1]
dicMotorPos = {}
dicAlarmStatus = {}
dicAlarmCode = {}
startPos = 0
endnode= 0
epcTotalView = pd.DataFrame()
epcTarget = None
lastNode = None
lastRSSI = None
lastPos = None
lastAck = ''
isScan = False
isRetry = False
lastcalledAck = DATETIME_OLD

def GetSpeicificEPCNodeInfo():
    local_epcnodeinfo = [GetNodeFromTable(HOME_CHARGE),GetNodeFromTable(HOME_TABLE)]
    state_keys: list[str] = list(GetCrossInfo().keys())
    local_epcnodeinfo.extend(state_keys)
    return local_epcnodeinfo


def rfidInstanceDefault():
    global epcTarget
    global lastNode
    global lastRSSI
    global lastPos
    global endnode
    global startPos
    global lastcalledAck
    global isScan
    global isRetry
    global lastAck
    epcTarget = None
    lastNode = None
    lastRSSI = None
    lastPos = None
    lastAck = ''
    isScan = False
    isRetry = False
    lastcalledAck =DATETIME_OLD
    

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

def SendCMD_Device(sendbuf):
    cmdTmp = sendbuf
    if isinstance(sendbuf, list):
        if len(sendbuf) > 0:
            cmdTmp = json.dumps(sendbuf)
        else:
            return False
    return service_setbool_client_common(ServiceBLB.CMD_DEVICE.value, cmdTmp, Kill)

def getRFIDInvStatus():
    global epcTotalView
    global sInv_Key
    global sEPCKey
    global sDivTab
    try:
        epcViewInfo = df_to_dict(epcTotalView, sEPCKey, sInv_Key)
        for epc,invStatus in epcViewInfo.items():
            if not isTrue(invStatus):
                return False
        return True
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        SendAlarmHTTP(message,True,BLB_ANDROID_IP_DEFAULT)
    return False

def callbackACK(recvData):
    global epcTarget
    global lastNode
    global lastRSSI
    global lastPos
    global endnode
    global startPos
    notagstr = 'NOTAG'
    global isScan
    global lastcalledAck
    global isRetry
    global epcTotalView
    rfidMissPulse = 5000
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

        mbid_tmp = lsResult[2]  # 모드버스 ID
        flag = lsResult[1]  # 0 이면 미완료, 1 이면 완료
        if not is_equal(mbid_tmp,ModbusID.MOTOR_H.value):
            return
        
        if not isTrue(flag):
            isRetry = 0
            return

        #lastcalledAck = getDateTime()
        if len(lsResult) >= 9:
            #print(lsResult)
            torque_max = lsResult[3]  # 최대토크
            torque_ave = lsResult[4]  # 평균토크
            ovr_max = lsResult[5]  # 최대오버로드
            ovr_ave = lsResult[6]  # 평균오버로드
            last_started_pos = int(lsResult[7])  # 최대오버로드
            last_targeted_pos = int(lsResult[8])  # 평균오버로드
            rospy.loginfo(f'torque_max:{torque_max},torque_ave:{torque_ave},ovr_max:{ovr_max},ovr_ave:{ovr_ave},last_started_pos:{last_started_pos},last_targeted_pos:{last_targeted_pos}')
        

        # if not isTimeExceeded(lastcalledAck, 200):
        #     return
        
        # last_inv = dfEPCTotal.iloc[-1][sInv_Key] if not dfEPCTotal.empty else None
        # last_EPC = dfEPCTotal.iloc[-1][sEPCKey] if not dfEPCTotal.empty else None        
        # print(last_inv)
        #if lastNode
        cur_pos = try_parse_int(dicMotorPos.get('MB_15'))

        if isScan:
            # dfEPCTotal.to_csv(strFileEPC_total, sep=sDivTab, index=False)
            # isScan = False
            # print(strFileEPC_total)
            # rospy.loginfo(f'Scan End :{cur_pos}')
            # dfEPCTotal.drop(dfEPCTotal.index, inplace=True) 
            RFIDControl(False)
            return
        
        if endnode == lastNode:
        #if lastRSSI is None:
            return
        
        if epcTarget is None:
            return
        dfEPCTotal = pd.read_csv(strFileEPC_total, sep=sDivTab)    
        epcViewInfo = df_to_dict(epcTotalView, sEPCKey, sInv_Key)
        dicEPCNode = GetEPCNodeInfoDic()
        isInvOff = False
        for epc,invStatus in epcViewInfo.items():
            if isTrue(invStatus) and len(epc) > 5:
                curNode = dicEPCNode.get(epc)
                if curNode == endnode:
                    rospy.loginfo(f'OK curnode:{curNode},epc:{epc} invStatus:{invStatus}')
                    RFIDControl(False)
                    return
            elif not isTrue(invStatus):
                isInvOff = True
                
        dicSmartPlug = shared_data.get(TopicName.SMARTPLUG_INFO.name)
        chargeSensorinfo = dicSmartPlug.get(SMARTPLUG_INFO.GPI1_CHARGE.name)
        chargerInfo = dicSmartPlug.get(SMARTPLUG_INFO.CHARGERPLUG_STATE.name)
        if isTrue(chargeSensorinfo):
            if isTrue(chargerInfo):
                SetChargerPlug(True)
            return
        
        #SendAlarmHTTP(f'RFID 인식이 안됨! {epcTarget} - {lastNode}',True,BLB_ANDROID_IP_DEFAULT)
        diff_pos = cur_pos - startPos
        diff_pos_abs = abs(diff_pos)
        if diff_pos < 0:
            rfidMissPulse = -rfidMissPulse
        if isRetry < 4:
        #if diff_pos_abs > roundPulse and :
            isRetry += 1
            finalPos = cur_pos + rfidMissPulse
            dicJOG = getMotorMoveDic(ModbusID.MOTOR_H.value,True,finalPos ,DEFAULT_RPM_SLOW,ACC_DECC_MOTOR_H,ACC_DECC_MOTOR_H)
            if isInvOff:
                RFIDControl(True)
            rospy.loginfo(f'다시탐색:{dicJOG}')
            SendCMD_Device([dicJOG])
        else:
            SendCMDESTOP(f'I{ACC_DECC_MOTOR_H}', False)
            message = 'RFID 데이터가 맞지 않습니다.'
            SendAlarmHTTP(message,True,BLB_ANDROID_IP_DEFAULT)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)
        SendAlarmHTTP(message,True,BLB_ANDROID_IP_DEFAULT)


def callbackRFID(recvDataMap):
    global epcTarget
    global lastNode
    global lastRSSI
    global lastPos
    global endnode
    global isScan
    global epcTotalView
    notagstr = 'NOTAG'
    needSave = False
    try:
        sEPC = recvDataMap.get(sEPCKey)
        sRSSI = abs(try_parse_int(recvDataMap.get(sRSSIKey,0)))
        sPOS = try_parse_int(recvDataMap.get(sPOS_Key,-1))
        sSPD = (try_parse_int(recvDataMap.get(sSPD_Key,-1)))
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
        #dfEPCTotal = pd.read_csv(strFileEPC_total, sep=sDivTab)        
        if isScan:
            dfScan = pd.read_csv(strFileEPC_scan, sep=sDivTab)
            epcnodeinfo2 = df_to_dict_int_values(dfScan, MAPFIELD.EPC.name, TableInfo.NODE_ID.name)
            result = get_missing_or_next(list(epcnodeinfo2.values()))
            if epcnodeinfo2.get(sEPC) is not None:
                result=epcnodeinfo2.get(sEPC)
            
            #기준이 되는 노드는 업데이트 하지 않는다.
            if result == 1 or result == 10:
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
                epcTarget = sEPC
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
        # elif not isScan:

        if epcTarget is None or isScan:
            return

        lastNode = curNode        
        endNode = epcnodeinfo.get(epcTarget)
        if curNode == endNode:
            if lastRSSI is None:
                lastRSSI = sRSSI
                lastPos = sPOS
                rospy.loginfo(f'lastRSSI init : {lastRSSI},POS:{lastPos},SPD:{sSPD}')
                return
            
            if sRSSI <= lastRSSI:
                lastRSSI = sRSSI
                lastPos = sPOS
                rospy.loginfo(f'lastRSSI updated : {lastRSSI},POS:{lastPos},SPD:{sSPD}')
                return
            
            if sSPD > 0:
                finalPos = lastPos+(sSPD*10*3)            
            else:
                finalPos = lastPos+(sSPD*10*4)            
            
            dicJOG = getMotorMoveDic(ModbusID.MOTOR_H.value,True,finalPos ,sSPDAbs,ACC_DECC_MOTOR_H,ACC_DECC_MOTOR_H)
            dicEPCNodeInfo = {TableInfo.NODE_ID.name : lastNode,MotorWMOVEParams.POS.name : finalPos,MAPFIELD.EPC.name: sEPC}
            # if needSave:
            #     #df = insert_row_to_csv(strFileEPC_total, sDivTab, dicEPCNodeInfo)
            #     df = insert_or_update_row_to_csv(strFileEPC_total, sDivTab, dicEPCNodeInfo, MAPFIELD.EPC.name)            
            #     print(df)
            #SendCMDESTOP(f'I{sSPD}')
            SendCMD_Device([dicJOG])
            epcTarget = None
            lastRSSI = None
            lastPos = None
            time.sleep(MODBUS_EXCEPTION_DELAY)
            RFIDControl(False)
            log_all_frames(f"Stop at {endNode},RFID:{sEPC},RSSI:{sRSSI},finalPos:{finalPos},SPD:{sSPD}")
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
    except Exception as e:
        rospy.logerr(f"데이터 처리 오류: {traceback.format_exc()}")
        logSSE_error(traceback.format_exc())
        return {"error": str(e)}, 500
  
    return {"message": f"{request.method},{loaded_data}"}, 200


@app.route('/JOG', methods=['GET'])
def service_jog():
    global startPos
    global endnode
    global epcTarget
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
        
        dic_CROSSINFO = shared_data.get(TopicName.CROSS_INFO.name)
        if dic_CROSSINFO:
            chargeSensor = dic_CROSSINFO.get(SMARTPLUG_INFO.GPI1_CHARGE.name)
            if isTrue(chargeSensor):
                lastNode = 1
        
        endnode = try_parse_int(endnode_tmp,MIN_INT)
        if endnode == MIN_INT:
            isScan = True
        else:
            isScan = False
        
        if is_equal(lastNode ,endnode):
            return {"error": "endnode is current node"}, 400
        
        epcnodeinfo = GetEPCNodeInfoDic()
        sEPC = get_key_by_value(epcnodeinfo, endnode)
        if sEPC is None:
            if isScan:
                distancePulse=pulse_tmp
            else:
                dfScan = pd.read_csv(strFileEPC_scan, sep=sDivTab)
                avg_df = dfScan.groupby("NODE_ID", as_index=False)["POS"].mean()
                avg_df["POS"] = avg_df["POS"].round().astype(int)                
                last_epc_df = dfScan.groupby("NODE_ID", as_index=False).last()[["NODE_ID", "EPC"]]
                result_df = pd.merge(avg_df, last_epc_df, on="NODE_ID")
                result_df.to_csv(strFileEPC_total, sep=sDivTab, index=False)
                return {"OK": "MapSaved"}, 200
        else:
            epcTarget = sEPC
            #distancePulse = GetEPC_Loc_Master(sEPC)
            distancePulse=GetNodePos_fromEPC(sEPC)
        
        # if distance_tmp is None:
        #     distancePulse=pulse_tmp
        #     isAbsPos = True
        # else:
        #     distance = try_parse_int(distance_tmp,MIN_INT)
        #     distancePulse = distance_to_pulseH(distance)
        
        # SendCMDESTOP(f'I{ACC_DECC_MOTOR_H}')
        # time.sleep(MODBUS_EXCEPTION_DELAY)
        #distancePulse = distance_to_pulseH(distance)
        lsSpecialNodes = GetSpeicificEPCNodeInfo()
        if endnode in lsSpecialNodes:
            listReturnTmp.append(dicPOTNOT_ON)
        else:
            listReturnTmp.append(dicPOTNOT_OFF)
        
        dicMotorH = getMotorMoveDic(ModbusID.MOTOR_H.value,isAbsPos,distancePulse,spd,ACC_DECC_MOTOR_H,ACC_DECC_MOTOR_H)
        listReturnTmp.append(dicMotorH)
        SetChargerPlug(False)
        time.sleep(MODBUS_EXCEPTION_DELAY)        
        RFIDControl(True)
        time.sleep(MODBUS_EXCEPTION_DELAY)
        lastRSSI = None
        SendCMD_Device(listReturnTmp)
        startPos = try_parse_int(dicMotorPos.get('MB_15'),MIN_INT)
        sJob = 'JOG'
        if isScan:
            sJob = 'SCAN'
        sMsg = f"{sJob} start from {startPos} to :{endnode},{sEPC}"
        rospy.loginfo(sMsg)
        time.sleep(MODBUS_EXCEPTION_DELAY)
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
        qNumber = request.args.get('q', MIN_INT)
        recvData = request.args.get('data')
        topicData = request.args.get('topicname')
        
        if chargeplug is not None:
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
        for topic_name, data in shared_data.items():
            if data:
                socketio.emit(f"update_{topic_name}", data)
        time.sleep(1)

@socketio.on("connect")
def handle_connect():
    """클라이언트가 접속하면 현재 데이터 전송"""
    print("✅ 클라이언트가 WebSocket에 연결됨")
    for topic_name, data in shared_data.items():
        if data:
            socketio.emit(f"update_{topic_name}", data)

if __name__ == "__main__":
    try:
        loaded_data2 = load_csv_to_dict(csvPathalarm, sort_ascending=False)
        #data_out2 = json.dumps(loaded_data2)
        shared_data[TopicName.HISTORY_ALARM.name] = loaded_data2
        threading.Thread(target=update_data, daemon=True).start()
        socketio.run(app, host="0.0.0.0", port=HTTP_COMMON_PORT, debug=False, use_reloader=False, log_output=True,allow_unsafe_werkzeug=True)
    except rospy.ROSInterruptException:
        pass