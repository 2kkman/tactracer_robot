#!/usr/bin/env python

import rospy
from dynamic_reconfigure.client import Client

def set_parameters():

    rospy.init_node('dynamic_reconfigure_client')

    client = Client('itops_f07/camera', config_callback=config_callback)

    client.update_configuration({"start_stream": True})
    curr_config = get_parameters()
    rospy.loginfo("Updated parameters: {start_stream}".format(**curr_config))

def get_parameters():

    rospy.init_node('dynamic_reconfigure_client')

    client = Client('itops_f07/camera', config_callback=config_callback)

    current_config = client.get_configuration()

    rospy.loginfo("Current parameters: {start_stream}".format(**current_config))
    return current_config

def config_callback(config):
    rospy.loginfo("Config set to {start_stream}".format(**config))

if __name__ == '__main__':
    try:
        set_parameters()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

