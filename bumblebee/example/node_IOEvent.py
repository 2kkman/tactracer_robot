#!/usr/bin/env python3
import rospy
import rosnode
#/opt/ros/noetic/share/std_srvs/srv/Empty.srv , SetBool.srv, Trigger.srv
from std_srvs.srv import *
#/opt/ros/noetic/share/rospy_tutorials/srv/AddTwoInts.srv
# input : a,b (정수) , return sum (정수)
from rospy_tutorials.srv import *
#/opt/ros/noetic/share/turtlesim/srv/Kill.srv
# input : string, return : 없음
#from turtlesim.srv import *
#from tta_blb.srv import *
#import srvSingleParam, srvSingleParamResponse

import serial
import os
import subprocess
from varname import *
import termios, sys
import time
from Util import *
from UtilGPIO import *
from SPG_Keys import *
#from ServoNano import *
from varname import *
from std_msgs.msg import Header
from std_msgs.msg import String
sDivFieldColon = StrParser.sDivColon2.value
sDivItemComma = StrParser.sDivComma3.value
sDivEmart =StrParser.sDivEmart1.value
sDivSlash =StrParser.sDivSlash.value
sDivSemiCol = StrParser.sDivSemiColon.value

dicStatus = {}
publish_topic_name = 'IOEvent'
MBID = 2
source_GPIO_topic_name = f'MB_{MBID}'
pub_ka = rospy.Publisher(publish_topic_name, String, queue_size=1)
pub_cmd = rospy.Publisher(NEST_TOPIC.CMD.value, String, queue_size=10)

def prtMsg(sendbuf):
    print(sendbuf)
    rospy.loginfo(sendbuf)

def SendKeepAlive(sendbuf):
    global pub_ka
    if pub_ka is not None:
        pub_ka.publish(sendbuf)
        prtMsg(sendbuf)

class GPOService():
    def __init__(self):
        self.GPIO_service = rospy.Service(ServiceList.NEST_GPO_SET.value, AddTwoInts, self.GPO_SET)
        rospy.loginfo('GPO Service is Started')

    def GPO_SET(self, req):
        global pub_cmd
        sendStr = ''
        reqNumber = req.a
        reqStatus = 256 if isTrue(req.b) else 512
        if reqNumber == 0:
            if isTrue(req.b): #전부 ON
                sendStr = f'MBID{sDivFieldColon}{MBID}{sDivItemComma}{NEST_TOPIC.CMD.value}{sDivFieldColon}WOUTON'
            else: #전부 OFF
                sendStr = f'MBID{sDivFieldColon}{MBID}{sDivItemComma}{NEST_TOPIC.CMD.value}{sDivFieldColon}WOUTOFF'
        else:
            sendStr = f'MBID{sDivFieldColon}{MBID}{sDivItemComma}{NEST_TOPIC.CMD.value}{sDivFieldColon}WOUT{reqNumber}{sDivItemComma}GPOVALUE{sDivFieldColon}{reqStatus}'
        pub_cmd.publish(sendStr)
        resp = AddTwoIntsResponse()
        resp.sum = 1
        return resp

def callbackGPIOTopic(data):
    global dicStatus 
    bReplace = False
    dicCurr = getDic_strArr(data.data,sDivFieldColon,sDivItemComma)
    for k,v in dicCurr.items():
        ValueOld = dicStatus.get(k, '')
        if v != ValueOld:
            SendKeepAlive(f'{k}{sDivFieldColon}{v}')
            bReplace = True
    if bReplace:
        dicStatus = dicCurr

if __name__ == "__main__":
    rospy.init_node(f'node_{publish_topic_name}',anonymous=False) #노드 생성
    rospy.Subscriber(source_GPIO_topic_name, String, callbackGPIOTopic, queue_size=1) # LaserScan 토픽이 오면 콜백 함수가 호출되도록 세팅
    GPOService()
    rospy.spin()
