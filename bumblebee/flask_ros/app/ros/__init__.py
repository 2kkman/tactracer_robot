import rospy
import threading
from ..utils import *
import requests
from requests.exceptions import RequestException

def send_agv_pause_resume(ipaddress: str, pause: int, resume: int, suspended: int):
    api_url_address = f"https://{ipaddress}:9000/nav_state"
    
    # URL 파라미터로 데이터를 전송
    params = {
        "pause": pause,
        "resume": resume,
        "suspended": suspended
    }
    
    try:
        response = requests.get(api_url_address, params=params, verify=False)
        response.raise_for_status()  # Raises a HTTPError if the status is 4xx, 5xx
        return response.json()
    except RequestException as e:
        print(f"An error occurred: {e}")
        return None


def init_ros():
    if rospy.get_node_uri() == None:
        rospy.init_node('flask_ros_node', anonymous=True)
    
    # ROS 스핀을 별도의 스레드에서 실행
    def spin_thread():
        rospy.spin()
    
    threading.Thread(target=spin_thread, daemon=True).start()

# 다른 ROS 관련 모듈들을 여기서 import
from .publishers import RosPublisher
from .subscribers import RosSubscriber
from .service_clients import RosServiceClient

# ROS 초기화 함수를 외부에서 호출할 수 있도록 export
__all__ = ['init_ros', 'RosPublisher', 'RosSubscriber', 'RosServiceClient']