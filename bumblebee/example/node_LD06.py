#! /usr/bin/env python3
import datetime
import os
import rosnode
import rospy
from varname import *
import time
from sensor_msgs.msg import LaserScan # LaserScan 메시지 사용준비
from std_msgs.msg import String
import numpy
import math
sDivField = ":"
sDivItem = ","

param_Lidar_Angle = 30
param_Lidar_Alarm = 0.05
param_Lidar_hide = False
param_Lidar_Rate = 1


lidar_points = None
publish_topic_name = 'scan_alarm_util'
#publish_topic_name = 'GOAL' #테스트용 변수
dic_lidar = {}
dic_lidar_converted = {}
lastUpdateAlarm = datetime.datetime.now()
alarm_interval = 5000
lastActionTime= datetime.datetime.now()
que = []
def getStr_fromDicPrivate(dicTmp, spliterItemValue, spliterLine):
  strReturn = ''
  for sKey in dicTmp.keys():
    sValue = dicTmp[sKey]
    strReturn += f'{sKey}{spliterItemValue}{sValue}{spliterLine}'

  return strReturn[0:-1]

def getStr_fromDic(dicTmp, spliterItemValue, spliterLine, head_or_tail = None, padString= None):
  strReturn = ''
  if head_or_tail is None:
      return getStr_fromDicPrivate(dicTmp, spliterItemValue, spliterLine)

  for sKey in dicTmp.keys():
    sValue = dicTmp[sKey]
    sKeyCurrent =''
    if head_or_tail is True:
      sKeyCurrent = sKey + padString
    else:
      sKeyCurrent = padString+sKey
    strReturn += f'{sKeyCurrent}{spliterItemValue}{sValue}{spliterLine}'

  return strReturn[0:-1]

def lidar_callback(data):
    global lidar_points # 라이다 토픽이 들어오면 실행되는 콜백 함수 정의
    lidar_points = data.ranges

rospy.init_node('Lidar_Alarm',anonymous=False) # Lidar 이름의 노드 생성
rate = rospy.Rate(param_Lidar_Rate)
rospy.Subscriber("/scanutil", LaserScan, lidar_callback, queue_size=1) # LaserScan 토픽이 오면 콜백 함수가 호출되도록 세팅
pub = rospy.Publisher(publish_topic_name, String, queue_size=10)
pubCMD = rospy.Publisher('/CMD', String, queue_size=10)
pubKeepAlive = rospy.Publisher('/GOAL', String, queue_size=10)

# def getStr_fromDicPrivate(dicTmp, spliterItemValue, spliterLine):
#   strReturn = ''
#   for sKey in dicTmp.keys():
#     sValue = dicTmp[sKey]
#     strReturn += f'{sKey}{spliterItemValue}{sValue}{spliterLine}'

#   return strReturn[0:-1]

# def getStr_fromDic(dicTmp, spliterItemValue, spliterLine, head_or_tail = None, padString= None):
#   strReturn = ''
#   if head_or_tail is None:
#       return getStr_fromDicPrivate(dicTmp, spliterItemValue, spliterLine)

#   for sKey in dicTmp.keys():
#     sValue = dicTmp[sKey]
#     sKeyCurrent =''
#     if head_or_tail is True:
#       sKeyCurrent = sKey + padString
#     else:
#       sKeyCurrent = padString+sKey
#     strReturn += f'{sKeyCurrent}{spliterItemValue}{sValue}{spliterLine}'

#   return strReturn[0:-1]


while not rospy.is_shutdown():
    if(lidar_points == None):
        #continue
        #CES 이후에 launch 연동으로 변경
        # nodecheck = rosnode.get_node_names()
        # if '/LD06' not in nodecheck:
        #     td = datetime.datetime.now() - lastActionTime
        #     if td.total_seconds() > 10:
        #         os.system('~/.ydlstart')
        #         lastActionTime= datetime.datetime.now()
        #         rospy.loginfo(f'{rospy.get_name()} : Lidar Start at {lastActionTime}')
        continue
    if(len(lidar_points) == 0):
        continue
      
    fullRange = len(lidar_points)
    rtn = ""
    dic_lidar.clear()
    dic_lidar_converted.clear()
    angleIncrement = 360 / fullRange

    alarm_list = []
    try:
        for i in range(fullRange): # 30도씩 건너뛰면서 12개 거리값만 출력
            iAngle = (format(i * angleIncrement,'.0f'))

            if numpy.isnan(lidar_points[i]) == False:
                dic_lidar_converted[str(iAngle).zfill(3)] = format(lidar_points[i],'.2f')
                dic_lidar[i] = format(lidar_points[i],'.2f')
    except:
        pass
            # else:
            #     rospy.loginfo(lidar_points[i])
            # if i <= param_Lidar_Angle or i >= (fullRange - param_Lidar_Angle):
            #     if lidar_points[i] > 0:
            #         rtFloat = format(lidar_points[i],'.2f')
            #         rtCurrent = str(rtFloat)
            #         rtn += f'{iAngle}:{rtCurrent}' + "`"
            #         if lidar_points[i] < param_Lidar_Alarm:
            #             dic_lidar[iAngle] = rtFloat
            #             alarm_list.append(lidar_points[i])

            #j = ((int)(i*(720 / sp))) +1
            #rtn += str(format(lidar_points[j],'.2f')) + ", "
        # alarm_graph =  numpy.mean(alarm_list)
        # que.insert(0,alarm_graph)
        # if len(que) >= 5:
        #     startVal = que[0]
        #     endVal = que[-1]
        #     print(f'{startVal}:{endVal},{startVal - endVal}')
        #     if startVal > endVal + 0.2: #점점 거리가 짧아지면(20CM 이상) 알람
        #         pubKeepAlive.publish('ID:620211129200857301,DIR:F,SPD:45,RANGE:-1')
        #         del que[:]
        #     else:
        #         que.pop()

    #print(f'{len(lidar_points)}/{rtn[:-2]}' )
    # if param_Lidar_hide is False:
    #     rospy.loginfo(f'{fullRange}:{rtn}')
    if len(dic_lidar) > 0:
        lidar_Result = getStr_fromDic(dic_lidar_converted,sDivField,sDivItem)
        #rospy.loginfo(lidar_Result)
        pub.publish(lidar_Result)
    #print(f'{len(lidar_points)}' )
    rate.sleep()
    #time.sleep(1)
