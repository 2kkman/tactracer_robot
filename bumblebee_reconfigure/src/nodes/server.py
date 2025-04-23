#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from bumblebee_reconfigure.cfg import TutorialsConfig

def callback(config, level):
    # 매개변수 변경 시 호출되는 콜백 함수
    rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\
    {str_param}, {bool_param}, {size}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("bumblebee_reconfigure", anonymous = False)

    srv = Server(TutorialsConfig, callback)
    rospy.spin()
