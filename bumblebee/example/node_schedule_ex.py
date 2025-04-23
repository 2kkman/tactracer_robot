#!/usr/bin/env python3
import schedule
import time
import datetime
import sys
import traceback
import os
#from UtilGPIO import NEST_TOPIC, StrParser
import rospy
#from Util import *
from std_msgs.msg import Header
from std_msgs.msg import String
#from SPG_Keys import *
#from UtilGPIO import *
from std_srvs.srv import *
#import rosclean
publish_topic_name='GOAL'
node_name = 'util_schedule'
# start_timeParamName = f"/{node_name}/start_time"
# debugmode_ParamName = f'/{node_name}/debugmode'
baseScriptDir = '/root/catkin_ws/src/tta_blb/scripts/'
# start_time = rospy.get_param(f'{start_timeParamName}
rospy.init_node(node_name, anonymous = True)

# debugmode =  rospy.get_param(f'{debugmode_ParamName}', default=True)
# pub = None
# if not debugmode:
#     rospy.init_node(node_name, anonymous = True)
#     pub = rospy.Publisher(publish_topic_name, String, queue_size=1)

#유틸박스에서는 아래는 사용하지 않는다.
# def event_function():
#     global pub
#     now = getDateTime()
#     eventmsg = f'MAP:{now}'
#     rospy.loginfo(f'Event fired and published to topic {publish_topic_name} : {eventmsg}')
#     if not debugmode:
#         pub.publish(eventmsg)    

#스케쥴 모듈이 동작시킬 코드 : 현재 시간 출력
def cmd_function(scriptpath):
    rospy.loginfo(f'Run {scriptpath}')
    os.system(scriptpath)

#프로그램을 종료시키기 위한 함수
def exit():
    rospy.loginfo("function exit")
    sys.exit()
    
#1초마다 test_fuction을 동작시키다가 "22:21"이 되면 프로그램 종료
#schedule.every(1).seconds.do(test_function)
#월요일 아침에 ros로그를 삭제한다. 일단 보류. rosclean
schedule.every().monday.at("06:00").do(cmd_function, f'{baseScriptDir}.rosclean')
#월요일 아침에 재부팅 한다
schedule.every().day.at('07:00').do(cmd_function, f'{baseScriptDir}.reBoot')
#event_function()
# schedule.every().day.at(start_time).do(event_function)

#무한 루프를 돌면서 스케쥴을 유지한다.
while True:
    schedule.run_pending()
    rospy.loginfo(schedule.get_jobs())
    # if rospy.has_param(start_timeParamName):
    #     tmp = rospy.get_param(start_timeParamName)
    #     print(schedule.get_jobs())
    #     if tmp != start_time:
    #         print(f'{time.ctime()} Change event schedule from {start_time} to {tmp}')
    #         schedule.clear()
    #         start_time = tmp
    #         schedule.every().day.at(start_time).do(event_function)
    time.sleep(1)