#!/usr/bin/env python3
import math
import rospy
import roslib
from varname import *
from std_msgs.msg import Header
from std_msgs.msg import String
import argparse, socket, time, json, datetime, platform, psutil, requests, pprint, uuid, sys
from Util import *
from UtilBLB import *

runFromLaunch = False
param_DUST_show = True
byteToGiga = 1e+9
# def getROS_Header(frame_id_str):
#   msgTmp = Header()
#   msgTmp.frame_id = frame_id_str
#   return msgTmp
#hostname = socket.gethostname()
hostname = GetMachineStr()

def prtMsg(sendbuf):
    if runFromLaunch:
      rospy.loginfo(sendbuf)
    else:
      print(sendbuf)

def get_ros_info():
    dictRosInfo = {
            MonitorROS.master_uri.name:STATUS_MOTOR.UNKNOWN.name,
            MonitorROS.slave_uri.name: STATUS_MOTOR.UNKNOWN.name,
            MonitorROS.master_hostname.name: STATUS_MOTOR.UNKNOWN.name,
            MonitorROS.slave_hostname.name: STATUS_MOTOR.UNKNOWN.name,
            MonitorROS.ros_start_time.name: STATUS_MOTOR.UNKNOWN.name
        }
    try:
        # ROS Master URI
        master_uri = GetUbutuParam(UbuntuEnv.ROS_MASTER_URI.name)

        # ROS Slave URI (This is typically the URI of the machine running the node)
        slave_uri = rospy.get_node_uri()

        # Master and Slave hostnames
        master_hostname = get_hostname(master_uri.split('//')[1].split(':')[0] if master_uri != 'Unknown' else 'Unknown')
        slave_hostname = get_hostname(slave_uri.split('//')[1].split(':')[0] if slave_uri else 'Unknown')

        # ROS start time (when this function is called after node initialization)
        #ros_start_time = time.ctime(rospy.get_rostime().to_time())
        ros_start_time = getCurrentTime()
        dictRosInfo[MonitorROS.master_uri.name] = master_uri
        dictRosInfo[MonitorROS.slave_uri.name] = slave_uri
        dictRosInfo[MonitorROS.master_hostname.name] = master_hostname
        dictRosInfo[MonitorROS.slave_hostname.name] = slave_hostname
        dictRosInfo[MonitorROS.ros_start_time.name] = ros_start_time
    except Exception as e:
        rospy.logerr(f"Failed to get ROS info: {e}")
    return dictRosInfo

def get_bandwidth():
    # Get net in/out
    net1_out = psutil.net_io_counters().bytes_sent
    net1_in = psutil.net_io_counters().bytes_recv

    time.sleep(1)

    # Get new net in/out
    net2_out = psutil.net_io_counters().bytes_sent
    net2_in = psutil.net_io_counters().bytes_recv

    # Compare and get current speed
    if net1_in > net2_in:
        current_in = 0
    else:
        current_in = net2_in - net1_in

    if net1_out > net2_out:
        current_out = 0
    else:
        current_out = net2_out - net1_out

    network = {MonitorPC.RX.name : current_in, MonitorPC.TX.name : current_out}
    return network

def talker():      
      pubTopic = TopicName.BLB_STATUS_MONITOR.name
      pub = rospy.Publisher(pubTopic, String, queue_size=10)
      rate = rospy.Rate(1) #send 1 time per second
      returnDic = {}
      cpu_count = psutil.cpu_count()

      while not rospy.is_shutdown():
        cpu_usage = psutil.cpu_percent(interval=1)
        memory_stats = psutil.virtual_memory()
        memory_total = round(memory_stats.total / byteToGiga,2)
        memory_used = round(memory_stats.used /byteToGiga, 2)
        memory_used_percent = round(memory_stats.percent,2)
        returnDic[MonitorPC.CPU_TEMP.name] = get_cpu_temperature_ubuntu()
        returnDic[MonitorPC.CPU_USAGE.name] = cpu_usage
        returnDic[MonitorPC.MEMORY_TOTAL_SIZE.name] = memory_total
        returnDic[MonitorPC.MEMORY_USED_SIZE.name] = memory_used
        returnDic[MonitorPC.MEMORY_USED_PERCENT.name] = memory_used_percent
        returnDic[MonitorPC.NUM_OF_DISPLAYS.name] = len(get_connected_monitors())
        disk_info = psutil.disk_partitions()
        x=disk_info[0]
        
          # Try fixes issues with connected 'disk' such as CD-ROMS, Phones, etc.
        try:
            devname = GetLastString(x.device, sDivSlash)
            disk = {
                MonitorPC.DISK_ID.name : devname,
                MonitorPC.PARTITION_TYPE.name : x.fstype,
                MonitorPC.DISK_TOTAL_SIZE.name : round( psutil.disk_usage(x.mountpoint).total / byteToGiga,2) ,
                MonitorPC.DISK_USED_SIZE.name : round(psutil.disk_usage(x.mountpoint).used/ byteToGiga,2),
                MonitorPC.DISK_USAGE_RATE.name : psutil.disk_usage(x.mountpoint).percent
            }
            returnDic.update(disk)

            #print("\tDisk name",disk["name"], "\tMount Point:", disk["mount_point"], "\tType",disk["type"], "\tSize:", disk["total_size"] / 1e+9,"\tUsage:", disk["used_size"] / 1e+9, "\tPercent Used:", disk["percent_used"])
        except Exception as e:
            prtMsg(traceback.format_exc())

        # Bandwidth Info
        #network_stats = get_bandwidth()
        #ros_status = get_ros_info()
        #returnDic.update(network_stats)
        #returnDic.update(ros_status)
        # # Network Info
        # nics = []
        # print("NICs:")
        # for name, snic_array in psutil.net_if_addrs().items():
        #     # Create NIC object
        #     nic = {
        #         "name": name,
        #         "mac": "",
        #         "address": "",
        #         "address6": "",
        #         "netmask": ""
        #     }
        #     # Get NiC values
        #     for snic in snic_array:
        #         if snic.family == -1:
        #             nic["mac"] = snic.address
        #         elif snic.family == 2:
        #             nic["address"] = snic.address
        #             nic["netmask"] = snic.netmask
        #         elif snic.family == 23:
        #             nic["address6"] = snic.address
        #     nics.append(nic)
        #     print("\tNIC:",nic["name"], "\tMAC:", nic["mac"], "\tIPv4 Address:",nic["address"], "\tIPv4 Subnet:", nic["netmask"], "\tIPv6 Address:", nic["address6"])

        # # Platform Info
        # system = {
        #     "name" : platform.machine(),
        #     "version" : platform.release()
        # }
        # returnDic.update(system)
        dicSend = {}
        for k,v in returnDic.items():
            dicSend[f'{k}_{hostname}'] = v
        
        sendbuf=json.dumps(dicSend)
        if pub is not None:
          pub.publish(sendbuf)
          prtMsg(sendbuf) 

        rate.sleep()
if __name__=='__main__':
    try:
        rospy.init_node(f'node_Monitor_{hostname}', anonymous = True)
        runFromLaunch = rospy.get_param("~startReal", default=False)
        print(f'runFromLaunch : {runFromLaunch}')
        
        talker()
    except rospy.ROSInterruptException:
        pass
