import os
import rospy
import socket
import time
from xmlrpc.client import ServerProxy

def get_local_ip():
    """
    Get the local IP address of the machine.
    This method opens a socket to a public DNS server (Google's) and gets the IP address of the local interface used to reach it.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
        return local_ip
    except Exception as e:
        rospy.logerr(f"Failed to get local IP address: {e}")
        return 'Unknown'

def get_connected_slave_ips():
    """
    Get the IP addresses of slaves connected to the ROS master.
    """
    slave_ips = []
    try:
        master_uri = rospy.get_master().getUri()
        master_host, master_port = master_uri.split('//')[1].split(':')
        master = ServerProxy(master_uri)
        state = master.getSystemState(rospy.get_name())[2]

        for sub_state in state:  # subscribers, publishers, services
            for uri in sub_state[1]:
                try:
                    slave_ip = uri.split('//')[1].split(':')[0]
                    if slave_ip != master_host and slave_ip not in slave_ips:
                        slave_ips.append(slave_ip)
                except IndexError:
                    continue
    except Exception as e:
        rospy.logerr(f"Failed to get connected slave IP addresses: {e}")
    return slave_ips

def get_ros_info():
    try:
        # ROS Master URI
        master_uri = os.environ.get('ROS_MASTER_URI', 'Unknown')

        # Resolve master hostname
        if master_uri != 'Unknown':
            master_ip = master_uri.split('//')[1].split(':')[0]
            master_hostname = socket.gethostbyaddr(master_ip)[0]
        else:
            master_hostname = 'Unknown'

        # Get local IP
        local_ip = get_local_ip()

        # Get connected slave IPs
        slave_ips = get_connected_slave_ips()

        # ROS start time (when this function is called after node initialization)
        ros_start_time = time.ctime(rospy.get_rostime().to_time())

        return {
            'master_uri': master_uri,
            'local_ip': local_ip,
            'master_hostname': master_hostname,
            'slave_ips': slave_ips,
            'ros_start_time': ros_start_time
        }

    except Exception as e:
        rospy.logerr(f"Failed to get ROS info: {e}")
        return {
            'master_uri': 'Error',
            'local_ip': 'Error',
            'master_hostname': 'Error',
            'slave_ips': 'Error',
            'ros_start_time': 'Error'
        }

# Example usage
if __name__ == '__main__':
    rospy.init_node('ros_info_node', anonymous=True)
    ros_info = get_ros_info()
    for key, value in ros_info.items():
        print(f"{key}: {value}")
