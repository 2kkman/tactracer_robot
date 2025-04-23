#!/usr/bin/env python3
import os
import smbus
import time, sys
import rospy
import roslib
import datetime
#from UTIL import *

from imusensor.MPU9250 import MPU9250
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

global seq

def getROS_Header(frame_id_str):
  msgTmp = Header()
  msgTmp.frame_id = frame_id_str
  return msgTmp

def getROS_Param(param_name):
  if rospy.has_param(param_name):
    return rospy.get_param(param_name)
  return None

def talker():
      global seq
      global imu
      frame_id_Range = 'imu'
      Imu_msg = Imu()
      Imu_msg.header = getROS_Header(frame_id_Range)
      Imu_msg.header.stamp = rospy.get_rostime()
      Imu_msg.header.seq = seq      
      pub2 = rospy.Publisher(frame_id_Range,Imu,queue_size=10)

      rate = rospy.Rate(3) #send 3 time per second

      while not rospy.is_shutdown():
        imu.readSensor()
        imu.computeOrientation()

        #print ("roll: {0} ; pitch : {1} ; yaw : {2}".format(imu.roll, imu.pitch, imu.yaw))
        #print ("Accel x: {0} ; Accel y : {1} ; Accel z : {2}".format(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2]))
        #print ("Gyro x: {0} ; Gyro y : {1} ; Gyro z : {2}".format(imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2]))
        #print ("Mag x: {0} ; Mag y : {1} ; Mag z : {2}".format(imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]))

        Imu_msg.angular_velocity.x = imu.GyroVals[0]
        Imu_msg.angular_velocity.y = imu.GyroVals[1]
        Imu_msg.angular_velocity.z = imu.GyroVals[2]

        Imu_msg.linear_acceleration.x = imu.AccelVals[0]
        Imu_msg.linear_acceleration.y = imu.AccelVals[1]
        Imu_msg.linear_acceleration.z = imu.AccelVals[2]

        Imu_msg.orientation.z = imu.yaw
        Imu_msg.orientation.w = imu.Temp
        Imu_msg.orientation.x = imu.roll
        Imu_msg.orientation.y = imu.pitch


        seq+=1
        # Imu_msg.header.stamp = rospy.Time.now()
        Imu_msg.header.stamp = rospy.get_rostime()
        Imu_msg.header.seq = seq

        rospy.loginfo(Imu_msg)
        pub2.publish(Imu_msg)
        rate.sleep()

if __name__=='__main__':
    try:
        seq = 0

        # Start ranging
        address = 0x68
        bus = smbus.SMBus(1)
        imu = MPU9250.MPU9250(bus, address)
        imu.begin()

        rospy.init_node('node_Imu', anonymous = True)
        talker()
    except rospy.ROSInterruptException:
        pass

