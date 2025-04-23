#!/usr/bin/env python3
import os
import sys
import rospy
import rosnode
import json
from std_srvs.srv import *
from rospy_tutorials.srv import *
from turtlesim.srv import *
from tta_blb.srv import *
from UtilBLB import *
from Util import *
import serial
import subprocess
from varname import *
import termios, sys
import time
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import Imu, Temperature, RelativeHumidity
import tf

class DataRecorder:
    def __init__(self):
        rospy.init_node("ard_carrier_node")
        self.pub = rospy.Publisher("/ARD_CARRIER", String, queue_size=10)
        self.data_dict = {
            "I_DOOR_1_BOTTOM": "0", "I_DOOR_1_HOME": "0", "I_DOOR_1_TOP": "1",
            "I_DOOR_2_BOTTOM": "0", "I_DOOR_2_HOME": "0", "I_DOOR_2_TOP": "1",
            "O_V12_NC": "0", "L1": "3,1000", "L0": "3,1000", "TILT_ANGLE": "145",
            "LOAD1": "1", "LOAD2": "2", "GOR_SVN": "-0.34,-2.41,218.06",
            "GAV_SVN": "-0.00,0.00,0.00", "GLA_SVN": "0.42,-0.06,9.96", "GAV_TUN": "0.00",
            "TEMPN": "26.03", "TIMESTAMP": rospy.get_time()
        }
        
        self.CMD_service = rospy.Service(ServiceBLB.CMDARD_ITX.value, Kill, self.update_data)
        #self.service = rospy.Service("/CMDARD_QBI", Trigger, self.update_data)
        rospy.Timer(rospy.Duration(1), self.publish_data)
        rospy.loginfo("ARD_CARRIER Node Started")

    def publish_data(self, event):
        self.data_dict["TIMESTAMP"] = rospy.get_time()
        msg = json.dumps(self.data_dict)
        self.pub.publish(msg)

    def update_data(self, req):
        #input_str = req.message  # Trigger 서비스 요청에서 입력된 문자열
        input_str = req.name
        self.SendArd(input_str)
        rospy.loginfo(input_str)
        #return TriggerResponse(success=True, message="Data updated")
        resp = KillResponse()
        return resp        
      
    def SendArd(self, msg : str):
        try:
            if msg.count(sDivFieldColon) == 1:
              key, value = msg.split(sDivFieldColon)
              #key, value = key.strip(), value.strip()
              if key in self.data_dict:
                  self.data_dict[key] = value
              else:
                  rospy.logwarn(f"Unknown key: {key}")
              return
            
            items = msg.split(",") if "," in msg else [msg]
            for item in items:
                if ":" in item:
                    key, value = item.split(":")
                    #key, value = key.strip(), value.strip()
                    if key in self.data_dict:
                        self.data_dict[key] = value
                    else:
                        rospy.logwarn(f"Unknown key: {key}")
                else:
                    rospy.logwarn(f"Invalid format: {item}")
        except Exception as e:
            rospy.logerr(f"Error parsing input: {e}")
            
if __name__ == "__main__":
    try:
        recorder = DataRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
