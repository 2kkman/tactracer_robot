import os
import rospy
import time
from UtilBLB import *

def get_ros_info():
    try:
        # ROS Master URI
        master_uri = os.environ.get('ROS_MASTER_URI', 'Unknown')

        # ROS Slave URI (This is typically the URI of the machine running the node)
        slave_uri = rospy.get_node_uri()

        # Master and Slave hostnames
        master_hostname = get_hostname(master_uri.split('//')[1].split(':')[0] if master_uri != 'Unknown' else 'Unknown')
        slave_hostname = get_hostname(slave_uri.split('//')[1].split(':')[0] if slave_uri else 'Unknown')

        # ROS start time (when this function is called after node initialization)
        ros_start_time = time.ctime(rospy.get_rostime().to_time())

        return {
            'master_uri': master_uri,
            'slave_uri': slave_uri,
            'master_hostname': master_hostname,
            'slave_hostname': slave_hostname,
            'ros_start_time': ros_start_time
        }

    except Exception as e:
        rospy.logerr(f"Failed to get ROS info: {e}")
        return {
            'master_uri': 'Error',
            'slave_uri': 'Error',
            'master_hostname': 'Error',
            'slave_hostname': 'Error',
            'ros_start_time': 'Error'
        }

# Example usage
if __name__ == '__main__':
    rospy.init_node('ros_info_node', anonymous=True)
    ros_info = get_ros_info()
    for key, value in ros_info.items():
        print(f"{key}: {value}")
