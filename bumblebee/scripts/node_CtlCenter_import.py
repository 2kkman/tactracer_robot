#!/usr/bin/env python3
import rospy
from dynamic_reconfigure.client import Client
import json
import copy
import serial
import os
import subprocess
import termios, sys
import time
import threading
import tf
import numpy as np
import pcl
import datetime
import open3d as o3d
import ros_numpy
from typing import List, Sequence
from dataclasses import dataclass, field
from sensor_msgs.msg import Imu, Temperature, RelativeHumidity, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs

from UtilBLB import *
from varname import *


mqttTopicSetNode = MQTT_TOPIC_VALUE.MSG_CROSS_REQUEST.value
frame_id_Range = "map"
BLB_ANDROID_PORT = HTTP_COMMON_PORT
    
#node_CHARGING_STATION = NODE_SPECIAL_VALUE.CHARGING_STATION.value
node_CHARGING_STATION = NODE_SPECIAL_VALUE.CHARGING_STATION.value
node_KITCHEN_STATION = NODE_SPECIAL_VALUE.KITCHEN_STATION.value
node_NOT_TABLE = NODE_SPECIAL_VALUE.NOT_TABLE.value
node_name = "node_CtlCenter"
dicLidarCropProfile = {LidarCropProfile.MOTOR_H: {
        LIDAR_CROP_PARAMS.range_min_z.name : 0.05,
        LIDAR_CROP_PARAMS.range_max_z.name : 1,
        LIDAR_CROP_PARAMS.range_max_x.name : 2,
        LIDAR_CROP_PARAMS.range_min_y.name : LIDAR_MIN_Y,
        LIDAR_CROP_PARAMS.range_max_y.name : LIDAR_MAX_Y
    },
        LidarCropProfile.MOTOR_V :    {
        LIDAR_CROP_PARAMS.range_max_x.name : 0.52,
        LIDAR_CROP_PARAMS.range_min_y.name : LIDAR_MIN_Y,
        LIDAR_CROP_PARAMS.range_max_y.name : LIDAR_MAX_Y,
        LIDAR_CROP_PARAMS.range_min_z.name : 0.0,
        LIDAR_CROP_PARAMS.range_max_z.name : 0.05 #-0.16 에서 수정해보자
    },
          LidarCropProfile.SERVING_ARM:      {
        LIDAR_CROP_PARAMS.range_min_z.name : -0.05,
        LIDAR_CROP_PARAMS.range_max_z.name : 0,
        LIDAR_CROP_PARAMS.range_max_x.name : 3,
        LIDAR_CROP_PARAMS.range_min_y.name : -0.05,
        LIDAR_CROP_PARAMS.range_max_y.name : 0.05
    },    LidarCropProfile.CHECK_GROUND:      {
        LIDAR_CROP_PARAMS.range_min_z.name : -0.1,
        LIDAR_CROP_PARAMS.range_max_z.name : 0.1,
        LIDAR_CROP_PARAMS.range_max_x.name : 4,
        LIDAR_CROP_PARAMS.range_min_y.name : -0.1,
        LIDAR_CROP_PARAMS.range_max_y.name : 0.1
    },
}

pub_BLB_STATUS = rospy.Publisher(TopicName.BLB_STATUS.name, String, queue_size=1)  # UI로 명령어 처리결과 및 상태값 송신
pub_topic2mqtt = rospy.Publisher(TopicName.SEND_MQTT.name, String, queue_size=1)
pub_TOPIC_LIST = rospy.Publisher(TopicName.TOPIC_LIST.name, String, queue_size=1)
pub_DF = rospy.Publisher(TopicName.JOB_DF.name, String, queue_size=1)
pub_JOBLIST = rospy.Publisher(TopicName.JOB_LIST.name, String, queue_size=1)
pub_JOBPath = rospy.Publisher(TopicName.JOB_PATH.name, String, queue_size=1)
# runFromLaunch = rospy.get_param(f"~{ROS_PARAMS.startReal.name}", default=False)
runFromLaunch = False
pub_ka = rospy.Publisher(TopicName.KEEPALIVE.name, String, queue_size=1)
#pub_motorPos = rospy.Publisher(TopicName.MOTOR_POS.name, String, queue_size=1)
pub_ros_config = rospy.Publisher(TopicName.ROS_CONFIG.name, String, queue_size=1)

#pub_cmdDevice = rospy.Publisher(TopicName.CMD_DEVICE.name, String, queue_size=1)  #
tf_buffer = None
tf_listener = None
transformed_cloud_pub = rospy.Publisher('/transformed_scan', PointCloud2, queue_size=10)
obstacles_cloud_pub = rospy.Publisher(TopicName.OBS.name, PointCloud2, queue_size=10)
lastTimeStamp = getDateTime()

#암 속도 조절.
adjustrate = SPEED_RATE_ARM

#6번 리프트 모터
ACC_LIFT_UP = getSpeedTableInfo(ModbusID.MOTOR_V.value,SPEEDTABLE_FIELDS.ACC_CCW.name)
ACC_LIFT_DOWN = getSpeedTableInfo(ModbusID.MOTOR_V.value,SPEEDTABLE_FIELDS.ACC_CW.name)
DECC_LIFT_UP = getSpeedTableInfo(ModbusID.MOTOR_V.value,SPEEDTABLE_FIELDS.DECC_CCW.name)
DECC_LIFT_DOWN = getSpeedTableInfo(ModbusID.MOTOR_V.value,SPEEDTABLE_FIELDS.DECC_CW.name)
SPD_LIFT = getSpeedTableInfo(ModbusID.MOTOR_V.value,SPEEDTABLE_FIELDS.SPD.name)

#10번 2관절 모터
SPD_EXTEND_ARM2 = getSpeedTableInfo(ModbusID.BAL_ARM2.value,SPEEDTABLE_FIELDS.SPD.name,adjustrate)
ACC_ARM2_EXTEND = getSpeedTableInfo(ModbusID.BAL_ARM2.value,SPEEDTABLE_FIELDS.ACC_CW.name,adjustrate)
ACC_ARM2_FOLD = getSpeedTableInfo(ModbusID.BAL_ARM2.value,SPEEDTABLE_FIELDS.ACC_CCW.name,adjustrate)
DECC_ARM2 = getSpeedTableInfo(ModbusID.BAL_ARM2.value,SPEEDTABLE_FIELDS.DECC_CW.name,adjustrate)

#11번 서빙 텔레스코픽 모터
ACC_ST = getSpeedTableInfo(ModbusID.TELE_SERV_MAIN.value,SPEEDTABLE_FIELDS.ACC_CW.name,adjustrate)
DECC_ST = getSpeedTableInfo(ModbusID.TELE_SERV_MAIN.value,SPEEDTABLE_FIELDS.DECC_CW.name,adjustrate)

#13번 1관절 모터
SPD_ARM1 = getSpeedTableInfo(ModbusID.BAL_ARM1.value,SPEEDTABLE_FIELDS.SPD.name,adjustrate)
ACC_ARM1 = getSpeedTableInfo(ModbusID.BAL_ARM1.value,SPEEDTABLE_FIELDS.ACC_CW.name,adjustrate)
DECC_ARM1 = getSpeedTableInfo(ModbusID.BAL_ARM1.value,SPEEDTABLE_FIELDS.DECC_CW.name,adjustrate)

#15번 주행 모터
ACC_MOVE_H = getSpeedTableInfo(ModbusID.MOTOR_H.value,SPEEDTABLE_FIELDS.ACC_CCW.name,SPEED_RATE_H)
DECC_MOVE_H = getSpeedTableInfo(ModbusID.MOTOR_H.value,SPEEDTABLE_FIELDS.ACC_CW.name,SPEED_RATE_H)
SPD_MOVE_H = getSpeedTableInfo(ModbusID.MOTOR_H.value,SPEEDTABLE_FIELDS.SPD.name,SPEED_RATE_H)

#27번 메인회전 모터
SPD_540 =  getSpeedTableInfo(ModbusID.ROTATE_MAIN_540.value,SPEEDTABLE_FIELDS.SPD.name)
ACC_540 = getSpeedTableInfo(ModbusID.ROTATE_MAIN_540.value,SPEEDTABLE_FIELDS.ACC_CW.name)
DECC_540 = getSpeedTableInfo(ModbusID.ROTATE_MAIN_540.value,SPEEDTABLE_FIELDS.DECC_CW.name)

#9번 밸런싱 텔레스코픽 모터
SPD_BALTELE = getSpeedTableInfo(ModbusID.TELE_BALANCE.value,SPEEDTABLE_FIELDS.SPD.name,adjustrate)
ACC_BT = getSpeedTableInfo(ModbusID.TELE_BALANCE.value,SPEEDTABLE_FIELDS.ACC_CW.name,adjustrate)
DECC_BT = getSpeedTableInfo(ModbusID.TELE_BALANCE.value,SPEEDTABLE_FIELDS.DECC_CW.name,adjustrate)

#31번 트레이 모터
SPD_360 =  getSpeedTableInfo(ModbusID.ROTATE_SERVE_360.value,SPEEDTABLE_FIELDS.SPD.name)
ACC_360_DOWN = getSpeedTableInfo(ModbusID.ROTATE_SERVE_360.value,SPEEDTABLE_FIELDS.ACC_CW.name)
DECC_360_DOWN = getSpeedTableInfo(ModbusID.ROTATE_SERVE_360.value,SPEEDTABLE_FIELDS.DECC_CW.name)
ACC_360_UP = getSpeedTableInfo(ModbusID.ROTATE_SERVE_360.value,SPEEDTABLE_FIELDS.ACC_CCW.name)
DECC_360_UP = getSpeedTableInfo(ModbusID.ROTATE_SERVE_360.value,SPEEDTABLE_FIELDS.DECC_CCW.name)

dicServExpand = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value, True, 345000, DEFAULT_RPM_SLOW, ACC_ST,DECC_ST)
dicServFold = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value, True, -200000, DEFAULT_RPM_SLOW,ACC_ST,DECC_ST)    
dicBackHome = getMotorMoveDic(ModbusID.MOTOR_H.value, True, 0, 2000,3000,3000)    
dicCaliHome = getMotorMoveDic(ModbusID.MOTOR_H.value, False, -200001, 200,3000,3000)    
# NODE_KITCHEN_EPC = "E2009A3030033AF000000821"
# EPC_NODE7 = "E2009A3030033AF000000136"
# EPC_NODE4 = "E2009A3030033AF000000923"
# targetEPC = {NODE_KITCHEN_EPC : 1, EPC_NODE7:7,EPC_NODE4:4}