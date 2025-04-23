#!/usr/bin/env python

import rospy
from dynamic_reconfigure.client import Client

def callback(config):
    print("Reconfigure Request: {0}".format(config))

if __name__ == "__main__":
    # 노드 초기화
    rospy.init_node("dynamic_reconfigure_client")

    # dynamic_reconfigure 클라이언트 생성
    client = Client("/bumblebee_reconfigure_3D", timeout=5)

    # 파라미터 값을 가져와서 수정
    params = client.get_configuration()
    print("Current Parameters: {0}".format(params))

    # 원하는 변수의 값을 수정
    new_params = {'range_min_z': 0.9}  # 예시로 'your_variable_name' 값을 10으로 설정
    client.update_configuration(new_params)
    params = client.get_configuration()
    print("Current Parameters: {0}".format(params))
    

    rospy.spin()
