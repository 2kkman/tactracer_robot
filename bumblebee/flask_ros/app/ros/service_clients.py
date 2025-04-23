import rospy
from std_srvs.srv import SetBool  # 예시, 실제 서비스 타입에 따라 변경
from ..utils import *

class RosServiceClient:
    def __init__(self):
        self.service_proxy = rospy.ServiceProxy(ServiceBLB.CTL_BLB.name, utilboxData)
        self.service_CMDARD_QBI = rospy.ServiceProxy(ServiceBLB.CMDARD_QBI.value, Kill)
        self.service_CMDARD_ITX = rospy.ServiceProxy(ServiceBLB.CMDARD_ITX.value, Kill)

    def call_service(self, data):
        try:
            response = self.service_proxy(data)
            #print(response)
            return response
        except rospy.ServiceException as e:
            return str(e)
    
    def call_service_CMDARD_QBI(self, data):
        try:
            response = self.service_CMDARD_QBI(data)
            print(response)
            return response
        except rospy.ServiceException as e:
            return str(e)        

    def call_service_CMDARD_ITX(self, data):
        try:
            response = self.service_CMDARD_ITX(replace_string(data))
            print(response)
            return response
        except rospy.ServiceException as e:
            return str(e)        