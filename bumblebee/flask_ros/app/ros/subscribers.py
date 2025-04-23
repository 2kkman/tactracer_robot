import rospy
from std_msgs.msg import String  # 예시, 실제 메시지 타입에 따라 변경
from ..utils import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

camImgTopic = f"{ServiceBLB.IMG_PUBLISH.value}0"
lsSubTopicList = [TopicName.BLB_STATUS_MONITOR.name, 'MB_15',TopicName.BMS.name,TopicName.ARD_CARRIER.name,TopicName.BLB_STATUS.name]
class RosSubscriber:
    def callback(self, data,topic_name):
        recvDataMap = {}
        recvData = (str)(data.data)

        if is_json(recvData):
            recvDataMap = json.loads(recvData)
        else:
            recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        #rospy.loginfo(recvDataMap)
        lock.acquire()
        # if topic_name == TopicName.BMS.name:
        #   print(recvDataMap)
        try:
            dictGlobal.update(recvDataMap)
        finally:
            # Lock을 해제해서 다른 Thread도 사용할 수 있도록 만든다.
            lock.release()
        
    def __init__(self):
        self.latest_message = None
        for topic in lsSubTopicList:
            rospy.Subscriber(topic, String, self.callback, queue_size=ROS_TOPIC_QUEUE_SIZE,callback_args=topic)
        #rospy.Subscriber(camImgTopic, String, self.callbackImgRecv)
        #print(data.data)
    
    # def callbackImgRecv(self, data):
    #     try:
    #         # ROS 이미지 메시지를 OpenCV 이미지로 변환
    #         cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #     except CvBridgeError as e:
    #         rospy.logerr("CvBridge Error: {0}".format(e))

    def get_latest_message(self):
        return self.latest_message