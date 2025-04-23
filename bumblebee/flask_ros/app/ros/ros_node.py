import rospy
from ..utils import *

def init_ros_node():
    rospy.init_node('flask_ros_bridge', anonymous=True)

def shutdown_ros_node():
    rospy.signal_shutdown('Flask application shutting down')