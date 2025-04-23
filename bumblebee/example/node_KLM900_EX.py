#!/usr/bin/env python3
import traceback
import time
import serial
import rospy

from Util import *
from std_msgs.msg import Header
from std_msgs.msg import String
from UtilBLB import *
from std_srvs.srv import *

btarray = bytearray(
    [0xBB, 0x00, 0xB7, 0x00, 0x00, 0xB7, 0x7E]
)  # 현재 RF세팅값을 읽는 명령.

lastUpdate = 0
line = []  # 라인 단위로 데이터 가져올 리스트 변수
baud = 115200  # 시리얼 보드레이트(통신속도)
InvThread = False  # 인벤토리 쓰레드 변수
param_RFIDPWR = 1200
param_RFID_Rate = 10  # 스캔 주기도 조절할 수 있어야 함. (1초에 3번)

nodeName = f"blb_carrier_rfid"
# publish_topic_name = 'GOAL' #테스트용 변수
rospy.init_node(nodeName, anonymous=False, log_level=rospy.INFO)
param_InvRFID = rospy.get_param("~param_InvRFID", default=False)
rfidReaderID = rospy.get_param("~readerID", default="L")
# publish_topic_name = f'{NEST_TOPIC.RFID.value}_{rfidFront}'
port = f"/dev/ttCRFID_{rfidReaderID}"  # 시리얼 포트
serRFID = serial.Serial(port, baud, timeout=None)
# print(param_InvRFID)
InvThread = param_InvRFID
btarrayInitRegion = bytearray([0xBB, 0x00, 0x07, 0x00, 0x01, 0x06, 0x0E, 0x7E])
btarrayInitFHSS = bytearray([0xBB, 0x00, 0xAD, 0x00, 0x01, 0xFF, 0x7E, 0x7E])
# serRFID.write(btarrayInitRegion) #RFID REGION 한국으로 초기화.
serRFID.write(
    btarrayInitFHSS
)  # RFID FHSs 초기화. 직전 초기화 명령어에 텀을 두고 보낸다. (그래서 이제서야 튀어나오는거야)
strparam_InvRFID = nameof(param_InvRFID)
strNotag = f"EPC:NOTAG,DEVID:{rfidReaderID}"
runFromLaunch = rospy.get_param(f"~{ROS_PARAMS.startReal.name}", default=False)
lastUpdateTimeStamp = getDateTime()

# ROSLAUNCH 에서 2개의 rfid 노드를 동시에 실행한 경우에는 10초 늦춰서 실행한다.
if runFromLaunch and rfidReaderID == "R":
    rospy.loginfo(f"RFID {rfidReaderID} Waiting 10s...")
    time.sleep(10)


# 데이터 처리할 함수
def parsing_data(data):
    try:
        global lastUpdate
        lastUpdate = lastUpdate + 1
        tmp = "".join(data)
        sEPC = ""
        sTID = "NONE"
        sEPC_TID = tmp[12:]
        sPC = sEPC_TID[0:2]
        if sEPC_TID == "":
            return strNotag
        sPCRaw = int("0x" + sPC, 16)
        sPCBin = bin(sPCRaw)[1:5]
        sEPC_Len = int("0" + sPCBin, 2) * 4
        sEPC = sEPC_TID[4 : sEPC_Len + 4]
        tmp = "".join(data)
        # if tmp.startswith("BB0222001"):
        #     return f"{RFID_RESULT.EPC.name}:{sEPC},{RFID_RESULT.TID.name}:{sTID},{RFID_RESULT.SEQ.name}:{lastUpdate},{RFID_RESULT.ID.name}:{rfidFront}"
        if tmp.startswith("BB0139001") or tmp.startswith("BB0222001"):
            if tmp.startswith("BB0139001"):
                print(tmp)
                sEPC_TID = tmp[12:]
                sPC = sEPC_TID[0:2]
                sPCRaw = int("0x" + sPC, 16)
                sPCBin = bin(sPCRaw)[1:5]
                sEPC_Len = int("0" + sPCBin, 2) * 4
                sEPC = sEPC_TID[4 : sEPC_Len + 4]
                sTID = sEPC_TID[4 + sEPC_Len : -4]
                print(sEPC, sTID)
            # return f"EPC:{sEPC},TID:{sTID},SEQ:{lastUpdate},ID:{rfidFront}"
            #return f"{RFID_RESULT.EPC.name}:{sEPC},{RFID_RESULT.TID.name}:{sTID},{RFID_RESULT.SEQ.name}:{lastUpdate},{RFID_RESULT.DEVID.name}:{rfidReaderID}"
            return f"{RFID_RESULT.EPC.name}:{sEPC},{RFID_RESULT.SEQ.name}:{lastUpdate},{RFID_RESULT.DEVID.name}:{rfidReaderID}"
        elif tmp.startswith("BB01B70002"):  # RF파워 조회
            extracted_part = tmp[10:14]
            decimal_value = int(extracted_part, 16)
            return f"{RFID_RESULT.PWR.name}:{decimal_value},{RFID_RESULT.SEQ.name}:{lastUpdate},{RFID_RESULT.DEVID.name}:{rfidReaderID}"
        return strNotag
    except Exception as e:
        print(traceback.format_exc())
        # tmp = ''.join(data)
        # sEPC = ''
        # sTID = 'NONE'
        # sEPC_TID = tmp[12:]
        # sPC = sEPC_TID[0:2]

        # print(sPC)
        return strNotag
        # print (e)


# RFID명령어 체크썸 계산 함수
def getCRC_RFID(data):
    crcTmp = 0
    for d in data:
        # c = str(d)
        # crcTmp += int(c,16)
        crcTmp += d
    result = crcTmp % 256
    # print(result)
    return result


def getRFCmd(rfpwr):
    # btarrayInitPWR = bytearray([0xBB,0x00,0xB6,0x00,0x02,0x07,0xD0,0x8F,0x7E])
    btarrayPWRHead = bytearray([0x00, 0xB6, 0x00, 0x02])
    btarrayPWRHead.extend(rfpwr.to_bytes(2, byteorder="big", signed=True))
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
    rospy.loginfo(f"RFID PWR:{start}")
    serRFID.write(btarray)


def RFIDControl(start):
    global serRFID
    global InvThread
    global param_InvRFID
    if start:
        serRFID.write(bytearray([0xBB, 0x00, 0x22, 0x00, 0x00, 0x22, 0x7E]))
    InvThread = param_InvRFID = start
    rospy.set_param(strparam_InvRFID, InvThread)
    rospy.loginfo(f"RFID:{start},DEV:{rfidReaderID}")


class RFID_KLM900:
    def __init__(self):
        self.INV_service = rospy.Service(
            f"{ServiceBLB.RFID_INV.value}_{rfidReaderID}",
            SetBool,
            self.InvRFID_SET,
        )
        rospy.loginfo(f"{port} RFID Started")

    def InvRFID_SET(self, req):
        reqbool = req.data
        RFIDControl(reqbool)
        return SetBoolResponse(True, f"RFID_{rfidReaderID}:{reqbool}")


if __name__ == "__main__":
    pub = rospy.Publisher(TopicName.RFID.name, String, queue_size=1)
    rate = rospy.Rate(param_RFID_Rate)
    RFIDPwr(param_RFIDPWR)
    RFID_KLM900()

    while not rospy.is_shutdown():
        dataRead = 0
        td = getDateTime() - lastUpdateTimeStamp
        # if param_InvRFID:
        if InvThread:
            serRFID.write(bytearray([0xBB, 0x00, 0x22, 0x00, 0x00, 0x22, 0x7E]))
        elif td.total_seconds() >= 10:
            serRFID.write(btarray)
            lastUpdateTimeStamp = getDateTime()

        bufCnt = serRFID.inWaiting()
        for c in serRFID.read(size=bufCnt):
            line.append(format(c, "x").rjust(2, "0").upper())
            dataRead = dataRead + 1
            if c == 126:  # 라인의 끝을 만나면..
                hello_str = parsing_data(line)
                if len(hello_str) > 10:
                    pub.publish(hello_str)
                    rospy.loginfo(hello_str)
                # else:
                #     rospy.logdebug(hello_str)

                # line 변수 초기화
                del line[:]
        # else:
        #     del line[:]

        rate.sleep()
