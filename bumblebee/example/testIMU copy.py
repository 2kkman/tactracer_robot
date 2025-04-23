#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
import json
import UtilBLB

def getROS_Header(frame_id_str):
  msgTmp = Header()
  msgTmp.frame_id = frame_id_str
  return msgTmp

# 예시 IMU 데이터 (이 부분을 실제 데이터를 받아오는 코드로 변경 가능)
imu_data_json = '{"accel":{"x":9.809988975524902,"y":-0.0,"z":0.0},"gyro":{"x":0.0,"y":0.0,"z":0.0},"magnet":{"x":5.875,"y":-0.0,"z":-48.375}}'
seq = 1
def imu_publisher():
    global seq
    # ROS 노드 초기화
    rospy.init_node('imu_publisher', anonymous=True)
    
    # IMU와 Magnetometer 토픽 발행 설정
    imu_pub = rospy.Publisher('/imu_data', Imu, queue_size=10)
    magnet_pub = rospy.Publisher('/magnet_data', Vector3, queue_size=10)

    # 주기적으로 데이터를 발행할 주기 설정 (10Hz)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # IMU 메시지 작성
        imu_msg = Imu()
        # IMU 데이터 파싱
        imu_data = json.loads(imu_data_json)
        # imu_msg.header = Header()
        # imu_msg.header.stamp = rospy.Time.now()
        # imu_msg.header.frame_id = "imu_link"
        seq+=1
        frame_id_Range = 'imu'
        imu_msg = Imu()
        imu_msg.header = getROS_Header(frame_id_Range)
        imu_msg.header.stamp = rospy.get_rostime()
        imu_msg.header.seq = seq
        # 가속도 데이터 설정
        imu_msg.linear_acceleration.x = imu_data['gyro']['x']
        imu_msg.linear_acceleration.y = imu_data['gyro']['y']
        imu_msg.linear_acceleration.z = imu_data['gyro']['z']

        # 자이로 데이터 설정
        imu_msg.angular_velocity.x = imu_data['gyro']['x']
        imu_msg.angular_velocity.y = imu_data['gyro']['y']
        imu_msg.angular_velocity.z = imu_data['gyro']['z']

        # 임시로 orientation을 0으로 설정
        imu_msg.orientation.x = imu_data['accel']['x']
        imu_msg.orientation.y = imu_data['accel']['y']
        imu_msg.orientation.z = imu_data['accel']['z']
        imu_msg.orientation.w = 0

        # IMU 메시지 발행
        imu_pub.publish(imu_msg)

        # Magnetometer 데이터 발행
        magnet_msg = Vector3()
        magnet_msg.x = imu_data['magnet']['x']
        magnet_msg.y = imu_data['magnet']['y']
        magnet_msg.z = imu_data['magnet']['z']

        magnet_pub.publish(magnet_msg)

        # 주기에 맞게 대기
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass
