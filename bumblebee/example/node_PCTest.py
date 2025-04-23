#! /usr/bin/env python3
import threading
from attr import fields
from Util import *
import os
import rosnode
import rospy
from varname import *
import time
from sensor_msgs.msg import LaserScan # LaserScan 메시지 사용준비
from sensor_msgs.msg import PointCloud2 as pc2
from std_msgs.msg import String
from sensor_msgs.msg import PointField
from laser_geometry import LaserProjection
import numpy

sDivField = ":"
sDivItem = ","

param_Lidar_Angle = 30
param_Lidar_Alarm = 3.8
param_Lidar_hide = True
param_Lidar_Rate = 0.5
outCloud :pc2 = pc2()


lidar_points = None
publish_topic_name = 'laserPC'
#publish_topic_name = 'GOAL' #테스트용 변수
dic_lidar = {}
lastUpdateAlarm = getDateTime()
alarm_interval = 5000
lastActionTime= getDateTime()
que = []
laserProj = LaserProjection()
rospy.init_node('laserPC',anonymous=False) # Lidar 이름의 노드 생성
rate :float = rospy.Rate(param_Lidar_Rate)
pub = rospy.Publisher(publish_topic_name, pc2, queue_size=1)
data_finish = False
seq_int = 0
max_row_step = 0
CloudResult:pc2 = pc2()
pub_mode = False
lock = threading.Lock()
z_range = 100

def lidar_callback(data : LaserScan):
    global lock
    if pub_mode:
        return
    global outCloud,seq_int,max_row_step,data_finish,CloudResult,lastUpdateAlarm
    
    #data_id : str = str(data.header.stamp.secs)+str(data.header.stamp.nsecs)
    data_id = data.header.seq
    print(data_id)
    if data_id in dic_lidar:
        
        dataCnt = len(dic_lidar)
        print(dataCnt)
        #CloudResult :pc2 = pc2()
        for sID in sorted(dic_lidar):
            seq_int+=1
            if seq_int % 10000 == 0:
                print(sID, seq_int)
            
            CloudTmp :pc2 = dic_lidar.get(sID, None)
            if CloudTmp == None:
                continue
            
            point_step = CloudTmp.point_step
            cur_idx = 8
            by =  bytearray(CloudTmp.data)
            
            while cur_idx+point_step < CloudTmp.row_step:
                #z_scale = int(seq_int  * (z_range /dataCnt))
                z_scale = seq_int
                z_scale_by =  z_scale.to_bytes(4,byteorder='little', signed = 'True')
                number = 0
                for byTmp in z_scale_by:
                    by[cur_idx+number] = byTmp 
                    number = number +1
                #by[cur_idx] = seq_int
                cur_idx += point_step
            by = by.rjust(max_row_step,b'\0')
            CloudTmp.data = bytes(by)
            total_row_step = CloudTmp.row_step
            if seq_int == 1: #CloudResult 가 초기값이라면 (나중에 수정)
                CloudResult = CloudTmp
            else:
                CloudResult.data=CloudResult.data + CloudTmp.data
                #print(f'TimeStamp = {sID},Len = {len(CloudResult.data)},row_step = {len(CloudTmp.data)},point_step = {point_step}')
                # for fieldTmp in CloudResult.fields:
                #     fieldTmp2 : PointField = fieldTmp
                #     if fieldTmp2.name == 'z':
                #         print(fieldTmp)
                #         print(type(fieldTmp))
        data_finish = True    
           
    else:
        lock.acquire()
        #data_finish = False
        CloudTmp :pc2 = laserProj.projectLaser(data)
        if CloudTmp.row_step > max_row_step:
            max_row_step = CloudTmp.row_step
        dic_lidar[data_id] = CloudTmp
        lock.release()
             
    
    #cloud_out = laserProj.projectLaser(data)
    # CloudTmp = laserProj.projectLaser(data)
    # pub.publish(CloudTmp)
    # rospy.loginfo(CloudTmp)
sub = rospy.Subscriber("/scan", LaserScan, lidar_callback) # LaserScan 토픽이 오면 콜백 함수가 호출되도록 세팅


while not rospy.is_shutdown():
    if data_finish:
        #lock.acquire()
        pub_mode = True
        dic_lidar.clear()
        #rospy.loginfo(CloudResult)
        rospy.loginfo(seq_int)
        #seq_int = 0
        CloudResult.row_step = len(CloudResult.data)
        CloudResult.width =  (int)(CloudResult.row_step / CloudResult.point_step)
        pub.publish(CloudResult)
        #lock.release()  
        

    # if(lidar_points == None):
    #     nodecheck = rosnode.get_node_names()
    #     if '/ydlidar_node' not in nodecheck:
    #         td = getDateTime() - lastActionTime
    #         if td.total_seconds() > 10:
    #             os.system('~/.ydlstart')
    #             lastActionTime= getDateTime()
    #             rospy.loginfo(f'{rospy.get_name()} : Lidar Start at {lastActionTime}')
    #     continue

    # if rospy.has_param(nameof(param_Lidar_hide)):
    #     param_Lidar_hide = rospy.get_param(nameof(param_Lidar_hide))
    # if rospy.has_param(nameof(param_Lidar_Angle)):
    #     param_Lidar_Angle = rospy.get_param(nameof(param_Lidar_Angle))
    # if rospy.has_param(nameof(param_Lidar_Alarm)):
    #     param_Lidar_Alarm = rospy.get_param(nameof(param_Lidar_Alarm))
    # if rospy.has_param(nameof(param_Lidar_Rate)): #스캔 주기도 조절할 수 있어야 함.
    #     Tmp = rospy.get_param(nameof(param_Lidar_Rate))
    #     if Tmp != param_Lidar_Rate:
    #         param_Lidar_Rate = Tmp
    #         rate = rospy.Rate(param_Lidar_Rate)


    # fullRange = len(lidar_points)
    # rtn = ""
    # dic_lidar.clear()

    # alarm_list = []
    # for i in range(fullRange): # 30도씩 건너뛰면서 12개 거리값만 출력
    #     if i <= param_Lidar_Angle or i >= (fullRange - param_Lidar_Angle):
    #         if lidar_points[i] > 0:
    #             rtFloat = format(lidar_points[i],'.2f')
    #             rtCurrent = str(rtFloat)
    #             rtn += f'{i}:{rtCurrent}' + "`"
    #             if lidar_points[i] < param_Lidar_Alarm:
    #                 dic_lidar[i] = rtFloat
    #                 alarm_list.append(lidar_points[i])


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
    # if len(dic_lidar) > 0:
    #     lidar_Result = getStr_fromDic(dic_lidar,sDivField,sDivItem)
    #     pub.publish(lidar_Result)
    #print(f'{len(lidar_points)}' )
    rate.sleep()
    #time.sleep(1)
