#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("Config set to {int_param}, {double_param}, {str_param}, {bool_param}, {size}".format(**config))

if __name__ == "__main__":
    rospy.init_node("bumblebee_reconfigure_client")
    
    # dynamic_tutorials 노드의 동적 매개변수를 변경하기 위한 클라이언트 객체 생성
    client = dynamic_reconfigure.client.Client("bumblebee_reconfigure", timeout=30, config_callback=callback)

    r = rospy.Rate(0.1)
    x = 0
    b = False
    while not rospy.is_shutdown():
        x = x+1
        if x>10:
            x=0
        b = not b
        client.update_configuration({"int_param":x, "double_param":(1/(x+1)), "str_param":str(rospy.get_rostime()), "bool_param":b, "size":1})
        r.sleep()