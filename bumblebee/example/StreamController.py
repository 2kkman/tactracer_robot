#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from dynamic_reconfigure.client import Client
import traceback

class StreamController:

    def __init__(self):
        self.client_name = 'itops_f07/camera'
        self.param_stream = 'start_stream'        


    def config_callback(self, config):
        """_summary_

        Args:
            config (_type_): _description_
        """
        rospy.loginfo(f"Config set to {self.param_stream}".format(**config))


    def handle_stream(self, req):
        """_summary_

        Args:
            req (bool): enable/disable stream

        Returns:
            SetBoolResponse: stream status
        """
        client = Client(self.client_name)

        try :
            client.update_configuration({self.param_stream: req.data})
            if req.data :
                rospy.loginfo("Stream Start")
                return SetBoolResponse(success=True, message=f"Status:{req.data}")
            else :   
                rospy.loginfo("Stream Stop")
                return SetBoolResponse(success=True, message=f"Status:{req.data}")
        except Exception as e:
            message = traceback.format_exc()
            rospy.loginfo(f"{message}")
            return SetBoolResponse(success=False, message=f"{message}")
            
            
    def stream_service(self):
        rospy.init_node('stream_controller')
        rospy.Service('stream_controller', SetBool, self.handle_stream)
        rospy.loginfo("Ready to Stream Control.")
        rospy.spin()

if __name__ == "__main__":
    stream_control = StreamController()
    stream_control.stream_service()
