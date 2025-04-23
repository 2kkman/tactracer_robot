#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import requests
import time
from UtilBLB import *
dic_485ex = {}
def callbackModbus(data):
    global dic_485ex
    try:
        recvData = data.data
        recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)

        # 수신 메세지에는 무조건 MBID 필드가 있으며 없으면 무시된다.
        mbid = recvDataMap.get(MotorWMOVEParams.MBID.name, None)
        if mbid is None:
          return      
        dic_485ex.update(recvDataMap)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(f'ERR callbackModbus:{message}')


class SSEClient:
    def __init__(self):
        self.seq = 0
        self.max_retries = 3  # 최대 재시도 횟수
        self.retry_delay = 10  # 기본 재시도 대기 시간 (초)
        self.max_retry_delay = 10  # 최대 재시도 대기 시간 (초)
        self.connected = False
        self.last_message_time = time.time()
        self.heartbeat_timeout = 30  # 하트비트 타임아웃 시간 (초)
        rospy.Subscriber("MB_15", String, callbackModbus)

    def check_connection_status(self):
        """서버 연결 상태를 확인하고 필요시 재연결을 시도"""
        if self.connected and time.time() - self.last_message_time > self.heartbeat_timeout:
            rospy.logwarn("No messages received for too long. Connection might be dead.")
            self.connected = False
            return False
        return self.connected

    def sse_client(self, url, publisher):
        """
        Connect to an SSE server and publish received messages to a ROS topic.
        :param url: The URL of the SSE server.
        :param publisher: The ROS publisher for the /RFID topic.
        """
        retry_count = 0
        current_delay = self.retry_delay

        while not rospy.is_shutdown():
            try:
                if not self.connected:
                    rospy.loginfo(f"Attempting to connect to SSE server (attempt {retry_count + 1})")
                
                with requests.get(url, stream=True, timeout=10) as response:
                    if response.status_code != 200:
                        raise requests.exceptions.RequestException(
                            f"Failed to connect: {response.status_code}")
                    
                    self.connected = True
                    retry_count = 0
                    current_delay = self.retry_delay
                    
                    rospy.loginfo("Connected to SSE server. Receiving RFID data...")
                    self.last_message_time = time.time()
                    self.seq = 0
                    buffer = ""
                    for chunk in response.iter_content(chunk_size=1, decode_unicode=True):
                        if rospy.is_shutdown():
                            break
                        
                        if chunk:
                            buffer += chunk
                            if chunk == sDivEmart:
                                message = buffer.strip()
                                #rospy.loginfo(f"Recv Data: {message}")
                                self.last_message_time = time.time()
                                self.seq += 1
                                if len(message) > 10:
                                    message = message.replace('data:', "")
                                    message = message.replace(sDivEmart, "")
                                    recvDataMap = getDic_strArr(message, '=', sDivItemComma)
                                    if message.find('ALIVE') > 0:
                                        self.seq -= 1
                                        message = json.dumps(recvDataMap)
                                        print(message)
                                        publisher.publish(message)
                                    else:
                                        keys2 = [key.strip() for key in message.split(sDivItemComma) 
                                                if len(key.strip()) > 1]
                                        keys = list(dict.fromkeys(keys2))
                                        cur_pos = dic_485ex.get(MonitoringField.CUR_POS.name,MIN_INT)
                                        cur_spd = dic_485ex.get(MonitoringField.CUR_SPD.name,MIN_INT)
                                        #epc_rssi = key.split(sep=sDivSemiCol)
                                        
                                        #ts = getCurrentTime(spliter="", includeDate=True)
                                        ts= getDateTime().timestamp()
                                        result = [{
                                            RFID_RESULT.EPC.name: key.split(sDivSemiCol)[0],  # EPC 값
                                            RFID_RESULT.RSSI.name: key.split(sDivSemiCol)[1],  # RSSI 값
                                            RFID_RESULT.DEVID.name: BLB_RFID_IP,
                                            RFID_RESULT.TIMESTAMP.name: ts,
                                            MonitoringField.CUR_POS.name:cur_pos,
                                            MonitoringField.CUR_SPD.name:cur_spd,
                                            #RFID_RESULT.inventoryMode.name: "on",
                                            RFID_RESULT.SEQ.name: self.seq
                                        } for key in keys]
                                        epcRSSI = try_parse_int(result[0].get(RFID_RESULT.RSSI.name))
                                        if abs(epcRSSI) < 50:
                                            message = json.dumps(result[0])
                                            print(message)
                                            publisher.publish(message)
                                            #rospy.loginfo(f"Published Data: {message},{dic_485ex}")
                                            #rospy.loginfo(f"Published Data: {message}")
                                buffer = ""
                        
                        # 주기적으로 연결 상태 체크
                        if not self.check_connection_status():
                            raise requests.exceptions.ConnectionError("Connection lost")

            except (requests.exceptions.RequestException, 
                    requests.exceptions.ConnectionError) as e:
                self.connected = False
                retry_count += 1
                
                if retry_count > self.max_retries:
                    rospy.logfatal(f"Failed to connect after {self.max_retries} attempts. Exiting...")
                    return
                
                # 지수 백오프로 재시도 간격 증가
                current_delay = min(current_delay * 2, self.max_retry_delay)
                rospy.logwarn(f"Connection error: {e}")
                rospy.loginfo(f"Reconnecting in {current_delay} seconds... "
                            f"(Attempt {retry_count} of {self.max_retries})")
                time.sleep(current_delay)
                
            except Exception as e:
                message = traceback.format_exc()
                rospy.logerr(f"Unexpected error: {e},{message}")
                self.connected = False
                time.sleep(self.retry_delay)
sse_port = 6000
if __name__ == "__main__":
    rospy.init_node("rfid_sse_client", anonymous=False)
    rfid_publisher = rospy.Publisher(TopicName.RFID.name, String, queue_size=1)
    sse_url = rospy.get_param("~sse_url", f"http://{BLB_RFID_IP}:{sse_port}/EPC")
    #sse_url = rospy.get_param("~sse_url", f"http://172.30.1.36:9001/EPC")
    
    sse_client = SSEClient()
    rospy.loginfo("Starting RFID SSE client...")
    
    try:
        sse_client.sse_client(sse_url, rfid_publisher)
    except rospy.ROSInterruptException:
        rospy.loginfo("RFID SSE client stopped.")