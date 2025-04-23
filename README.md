# Bumblebee 
범블비는 식당에서 서빙을 하는 로봇입니다.

# bumblebee_gui
범블비 사용에 필요한 GUI를 제공합니다.
GUI를 통해 범블비의 이송정보를 입력하고, 범블비의 이송을 시작할 수 있습니다.
도착시 메시지를 받으면 범블비의 이송을 종료합니다.
제어센터를 통해 메시지를 전달 받을수 있고 TTS기능을 통해 음성으로 안내가 가능합니다.

# bumblebee_reconfigure
로봇에 사용되는 LiDAR의 정보를 받아와서, ROS의 토픽으로 게시하는 노드
LiDAR의 ROI를 설정하여 특정 범위 내의 물체를 감지할 수 있습니다.
다이나믹셀을 통해 ROI를 손쉽게 설정할 수 있습니다.
Node : server_publisher_lidar.py
Node : server_publisher_laser.py

## Environment
 - OS : Ubuntu 20.04
 - PLATFORM : ROS Noetic
 - LANGUAGE : Python 3.8.10
 - SENSOR : Itops 3D LiDAR F07, Ldlidar Ld06
 - LIBRARY : pyserial, pyyaml, numpy, rospy, rospkg, sensor_msgs, std_msgs, time, math, os, sys, threading, time, yaml 
 - TEST

