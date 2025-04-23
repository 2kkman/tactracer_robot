#!/usr/bin/env python3
import traceback
import serial
from UtilGPIO import NEST_TOPIC, StrParser
import rospy
from Util import *
from std_msgs.msg import Header
from std_msgs.msg import String
from SPG_Keys import *
from UtilGPIO import *
#/opt/ros/noetic/share/std_srvs/srv/Empty.srv , SetBool.srv, Trigger.srv
from std_srvs.srv import *

lastUpdate = 0
line = [] #라인 단위로 데이터 가져올 리스트 변수
#port = '/dev/ttCRFID' # 시리얼 포트
#port = '/dev/ttyUSB5' # 시리얼 포트
port = '/dev/ttyS0' # 시리얼 포트
baud = 115200 # 시리얼 보드레이트(통신속도)
InvThread = False   # 인벤토리 쓰레드 변수
param_RFIDPWR = 1200
param_RFID_Rate = 10 #스캔 주기도 조절할 수 있어야 함. (1초에 3번)

publish_topic_name = NEST_TOPIC.RFID.value
nodeName = 'spg_nest_rfid'
#publish_topic_name = 'GOAL' #테스트용 변수
serRFID = serial.Serial(port, baud, timeout=None)
rospy.init_node(nodeName, anonymous = False, log_level=rospy.INFO)
param_InvRFID = rospy.get_param("~param_InvRFID", default=False)
print(param_InvRFID)
InvThread = param_InvRFID
btarrayInitRegion = bytearray([0xBB,0x00,0x07,0x00,0x01,0x06,0x0E,0x7E])
btarrayInitFHSS = bytearray([0xBB,0x00,0xAD,0x00,0x01,0xFF,0x7E,0x7E])
#serRFID.write(btarrayInitRegion) #RFID REGION 한국으로 초기화.
serRFID.write(btarrayInitFHSS) #RFID FHSs 초기화. 직전 초기화 명령어에 텀을 두고 보낸다. (그래서 이제서야 튀어나오는거야)
strparam_InvRFID = nameof(param_InvRFID)

def callbackCmd(data):
    try:
        recvData = str(data.data).upper()
        dicCMD = getCMDDictFromStr(recvData)
        rospy.loginfo(f'Recv data ({dicCMD}) : {recvData}')
        if dicCMD == None:
            rospy.loginfo(f'Recv data is invalid!')
            return

        sID = dicCMD[COMMON_CMD.ID.name]
        sCMD = dicCMD[COMMON_CMD.CMD.name]
        sVALUE = dicCMD[COMMON_CMD.VALUE.name]

        if sCMD == 'I':
            if sVALUE == '1':
                RFIDControl(True)
            else:
                RFIDControl(False)

    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)

rospy.Subscriber(NEST_TOPIC.CMD.value, String, callbackCmd)

#데이터 처리할 함수
def parsing_data(data):
    try:
        global lastUpdate
        lastUpdate = lastUpdate +1
        tmp = ''.join(data)
        sEPC = ''
        sTID = 'NONE'
        sEPC_TID = tmp[12:]
        sPC = sEPC_TID[0:2]
        sPCRaw = int('0x'+sPC,16)
        sPCBin = bin(sPCRaw)[1:5]
        sEPC_Len = int('0'+sPCBin,2) * 4
        sEPC = sEPC_TID[4:sEPC_Len+4]
        tmp = ''.join(data)
        if tmp.startswith('BB0222001'):
            return f'EPC:{sEPC},TID:{sTID},SEQ:{lastUpdate}'
        elif tmp.startswith('BB0139001'):
            print(tmp)
            sEPC_TID = tmp[12:]
            sPC = sEPC_TID[0:2]
            sPCRaw = int('0x'+sPC,16)
            sPCBin = bin(sPCRaw)[1:5]
            sEPC_Len = int('0'+sPCBin,2) * 4
            sEPC = sEPC_TID[4:sEPC_Len+4]
            sTID = sEPC_TID[4+sEPC_Len:-4]
            print(sEPC, sTID)
            return f'EPC:{sEPC},TID:{sTID},SEQ:{lastUpdate}'
        else:
            #print('NOTAG')
            return 'NOTAG'
    except Exception as e:
            print(traceback.format_exc())
            return 'NOTAG'
            #print (e)
#RFID명령어 체크썸 계산 함수
def getCRC_RFID(data):
    crcTmp = 0
    for d in data:
      #c = str(d)
      #crcTmp += int(c,16)
      crcTmp += d
    result =crcTmp%256
    #print(result)
    return result

def getRFCmd(rfpwr):
    #btarrayInitPWR = bytearray([0xBB,0x00,0xB6,0x00,0x02,0x07,0xD0,0x8F,0x7E])
    btarrayPWRHead = bytearray([0x00,0xB6,0x00,0x02])
    btarrayPWRHead.extend(rfpwr.to_bytes(2, byteorder='big', signed=True))
    btarrayPWRHead.append(getCRC_RFID(btarrayPWRHead))
    btarrayPWRHead.append(0x7E)
    btarrayPWRHead.insert(0, 0xBB)
    print(btarrayPWRHead)
    return btarrayPWRHead


def RFIDPwr(start):
    global serRFID
    RFpower = int(start)
    btarray = getRFCmd(RFpower)
    serRFID.write(btarray)
    rospy.loginfo(f'RFID PWR:{start}')
    btarray = bytearray([0xBB,0x00,0xB7,0x00,0x00,0xB7,0x7E]) #현재 RF세팅값을 읽는 명령.
    serRFID.write(btarray)

def RFIDControl(start):
    global serRFID
    global InvThread
    global param_InvRFID
    if start is True:
        serRFID.write(bytearray([0xBB,0x00,0x22,0x00,0x00,0x22,0x7E]))
    InvThread = param_InvRFID = start
    rospy.set_param(strparam_InvRFID, InvThread)
    rospy.loginfo(f'RFID:{start}')

class RFID_KLM900():
    def __init__(self):
        self.BREAK_ENABLE_service = rospy.Service(ServiceList.NEST_RFID_INV.value, SetBool, self.InvRFID_SET)
        rospy.loginfo('Nest RFID Started')

    def InvRFID_SET(self, req):
        reqbool = req.data
        RFIDControl(reqbool)
        return SetBoolResponse(True, f'RFID:{reqbool}')

if __name__ == "__main__":
    pub = rospy.Publisher(publish_topic_name, String, queue_size=1)
    rate = rospy.Rate(param_RFID_Rate)
    RFIDPwr(param_RFIDPWR)
    RFID_KLM900()

    while not rospy.is_shutdown():
        # strparam_RFIDPWR = nameof(param_RFIDPWR)
        # if rospy.has_param(strparam_RFIDPWR):
        #     param_RFIDPWR_Tmp = rospy.get_param(strparam_RFIDPWR)
        #     if param_RFIDPWR != param_RFIDPWR_Tmp:
        #         RFIDPwr(param_RFIDPWR_Tmp)
        #         param_RFIDPWR = param_RFIDPWR_Tmp
        #         rospy.loginfo(f'{strparam_RFIDPWR}:{param_RFIDPWR}')
        # else:
        #     rospy.set_param(strparam_RFIDPWR, param_RFIDPWR)

        # if rospy.has_param(strparam_InvRFID):
        #     param_InvRFID = rospy.get_param(strparam_InvRFID)
        #     if param_InvRFID is not InvThread:
        #         InvThread = param_InvRFID
        #         rospy.loginfo(f'{strparam_InvRFID}:{InvThread}')
        #         #rosparam set param_InvRFID True
        # else:
        #     rospy.set_param(strparam_InvRFID, InvThread)

        # strparam_RFID_Rate = nameof(param_RFID_Rate)
        # if rospy.has_param(strparam_RFID_Rate): #스캔 주기도 조절할 수 있어야 함.
        #     Tmp = rospy.get_param(strparam_RFID_Rate)
        #     if Tmp != param_RFID_Rate:
        #         param_RFID_Rate = Tmp
        #         rate = rospy.Rate(param_RFID_Rate)
        #         rospy.loginfo(f'{strparam_InvRFID}:{InvThread}')
        # else:
        #     rospy.set_param(strparam_RFID_Rate, param_RFID_Rate)

        dataRead = 0
        if InvThread:
            serRFID.write(bytearray([0xBB,0x00,0x22,0x00,0x00,0x22,0x7E]))
            bufCnt = serRFID.inWaiting()
            for c in serRFID.read(size=bufCnt):
                    line.append(format(c,'x').rjust(2,'0').upper())
                    dataRead = dataRead + 1
                    if c == 126: #라인의 끝을 만나면..
                        hello_str = parsing_data(line)
                        if len(hello_str) >10:
                            pub.publish(hello_str)
                            rospy.loginfo(hello_str)
                        # else:
                        #     rospy.logdebug(hello_str)

                        #line 변수 초기화
                        del line[:]
        else:
            del line[:]
        rate.sleep()
