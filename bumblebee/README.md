# tta_blb
  Tactracer BumbleBee Ros Package
- RFID 태그를 통해 위치를 파악하고 목표 지점으로 이동하는 것을 목표
- Point Cloud를 통해 물체 감지 및 위치 파악을 목표
- Lidar로 수집되는 포인트 데이터에서 범위를 지정하고 범위내에 물체 감지 구현을 목표
- 감지된 물체와의 로봇과의 거리를 계산하고 목표지점까지 안전하게 이동하는 것을 목표
- 카메라를 통해 마커을 인식하고 목표 테이블을 찾아 이동하는 것을 목표
- 음성 메시지를 TTS로 변환하여 스피커로 출력을 목표
- 터치스크린을 통해 사용자의 편의성을 높이는 것을 목표
- 아두이노를 통해 모터 제어 및 센서 제어를 구현을 목표
- ROS를 통해 각 기능들을 통합을 목표
  

## Environment
- Ubuntu 20.04
- ROS Noetic
- Python 3.8.10
- Sensor : Itops 3D LiDAR F07, Ldlidar Ld06, Speaker, Camera, Arduino, 
          Touch Sensor, RFID Reader
- library : pyserial, pyyaml, numpy, rospy, rospkg, sensor_msgs, std_msgs,
          time, math, os, sys, threading, time, yaml, gtts, playsound

## Node
- node_ArdBLB.py : 아두이노를 통해 모터 제어 및 센서 제어를 구현
- node_CamArucoMarker.py : 카메라를 통해 마커을 인식하고 목표 테이블을 찾아 이동
- node_CtlCenter_callback.py : 컨트롤 센터에서 명령을 받아 각 노드로 명령을 전달
- node_CtlCenter_func.py : 컨트롤 센터에서 사용하는 메서드 모음
- node_CtlCenter_globals.py : 컨트롤 센터에서 사용하는 전역 변수 모음
- node_CtlCenter_import.py : 컨트롤 센터에서 사용하는 import 모음
- node_CtlCenter_main.py : 컨트롤 센터 메인 노드
- node_KLM900_EX.py : RFID 리더기를 구동하여 토픽을 발행하는 노드
- node_MasterCheck.py : 마스터 노드가 살아있는지 확인하는 노드
- node_ModbusIF.py : Modbus I/F를 통해 모터를 제어하는 노드
- node_MQTT.py : MQTT를 통해 메시지를 발행하는 노드
- node_Pickle.py : Pickle을 통해 설정 데이터를 저장하고 불러오는 노드
- node_RFID_Target.py : RFID 태그를 통해 위치를 파악하고 목표 지점으로 이동
- node_SafetyLidar.py : Lidar로 수집되는 포인트 데이터에서 범위를 지정하고 범위내에 물체 감지하는 노드
- node_schedule.py : 스케줄을 통해 노드를 실행하는 노드
- node_TTS.py : 메시지를 TTS로 음성 변환하여 스피커로 출력하는 노드