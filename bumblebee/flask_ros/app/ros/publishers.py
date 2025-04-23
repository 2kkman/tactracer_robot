import rospy
from std_msgs.msg import String  # 예시, 실제 메시지 타입에 따라 변경
from ..utils import *

class RosPublisher:
    def __init__(self):
        self.publisher = rospy.Publisher(TopicName.BLB_CMD.name, String, queue_size=10)
        self.publisher_cmd_device = rospy.Publisher(TopicName.CMD_DEVICE.name, String, queue_size=10)
        self.publisher_blbstatus_https = rospy.Publisher(TopicName.BLB_STATUS_HTTPS.name, String, queue_size=10)

    def publish_message(self, message):
        self.publisher.publish(String(message))

    def publish_message_cmddevice(self, message):
        self.publisher_cmd_device.publish(String(message))        
    
    def publish_message_status(self, message):
        self.publisher_blbstatus_https.publish(String(message))                