#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointCloud
from sensor_msgs import point_cloud2
#from sensor_msgs import ChannelFloat32 
from geometry_msgs.msg import Point32
import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg

def callback_point_cloud2(msg):
    # Convert PointCloud2 to PointCloud
    #cloud_gen = point_cloud2.read_points(msg, field_names=("x", "y", "z","rgba"), skip_nans=True)
    cloud_gen = point_cloud2.read_points(msg,skip_nans=True)
    cloud = []
    for p in cloud_gen:
        cloud.append([p[0], p[1], p[2]])
    
    # Populate the PointCloud message
    output = PointCloud()
    msgPT32 = Point32()
    msgChannel = sensor_msgs.msg.ChannelFloat32()
    msgChannel.
    output.channels =msgChannel
    output.header = msg.header
    for pt in cloud:
        if pt[0] != 0:
            msgPT32.x = pt[0] 
            msgPT32.y = pt[1] 
            msgPT32.z = pt[2] * 1
            #point = output.points
            # point.x = pt[0]
            # point.y = pt[1]
            # point.z = pt[2]
            output.points.append(msgPT32)
    
    # Publish the PointCloud message
    pub.publish(output)

if __name__ == '__main__':
    rospy.init_node('pointcloud2_to_pointcloud', anonymous=True)
    
    # Create a subscriber and publisher
    sub = rospy.Subscriber("/scan_3D", PointCloud2, callback_point_cloud2)
    pub = rospy.Publisher("/output_pointcloud", PointCloud, queue_size=10)
    
    rospy.spin()
