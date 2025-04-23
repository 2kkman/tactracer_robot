import rospy
import os
import time
import subprocess
import sys
from UtilBLB import *
from Util import *
def ping_master(master_uri):
    response = subprocess.call(['ping', '-c', '1', master_uri])
    # ping 호출이 성공하면 0을 반환
    return response == 0

def monitor_connection(master_uri, interval=10):
    while not rospy.is_shutdown():
        #connected = ping_master(master_uri)
        connected = check_ros_master()
        if connected:
            rospy.loginfo(f"Connected to ROS Master - {master_uri}")
        else:
            rospy.loginfo(f"Lost connection to ROS Master. Attempting to reconnect...- {master_uri}")
            # ROS 노드 재시작 로직 (여기서는 rospy를 재시작하는 예시)
            include_path = GetUbutuParam(UbuntuEnv.SCR_DIR.name)
            reboot_script_path = f'{include_path}/.reBoot'
            os.system(reboot_script_path)
            #rospy.signal_shutdown(f"Lost connection to Master- {master_uri}")
            time.sleep(2)  # 재시작 전에 잠시 대기
            #os.execv(__file__, sys.argv)  # 현재 스크립트를 재실행
        time.sleep(interval)

if __name__ == '__main__':
    rospy.init_node('connection_monitor')
    master_uri = GetUbutuParam(UbuntuEnv.ROS_MASTER_URI.name)
    master_ip = extract_hostname_from_uri(master_uri)
    #= rospy.get_param('~master_uri', '192.168.0.1')  # ROS 마스터 URI 설정
    monitor_connection(master_ip)
