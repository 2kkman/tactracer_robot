#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField, PointCloud
from std_msgs.msg import Header
import open3d as o3d
import numpy as np
import ros_numpy
import sensor_msgs
vis = o3d.visualization.VisualizerWithKeyCallback()
pcd = o3d.geometry.PointCloud()
publish_topic_name = 'TestPC_PC2'
import Util
import UtilGPIO
import pcl
from sensor_msgs import point_cloud2
rospy.init_node('pointcloud_listener', anonymous=True)

pub_ka = rospy.Publisher(publish_topic_name, PointCloud2, queue_size=1)

def callback(input_ros_msg):
    global pcd
    global vis
    global pub_ka
    pcd.points = o3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(input_ros_msg))
    # T= [[0.992133 , 0 , -0.121753 , 491.436], 
    # [0 , -0.999096 , -0.0056368, 22.5665], 
    # [-0.121886 , 0.00063143, -0.993027 , 2205.69], 
    # [0 , 0 , 0 , 1]]
    T= [[1 , 1 , 1 , 491.436], 
    [1 , 1 , 0, 22.5665], 
    [-0.1 , 0, -1 , 2205.69], 
    [0 , 0 , 0 , 1]]
    pcd=pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    pc = np.asarray(pcd.points)
    pc_array = np.zeros(len(pc), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
    ])
    pc_array['x'] = pc[:, 0]
    pc_array['y'] = pc[:, 1]
    pc_array['z'] = pc[:, 2]
    pc_array['intensity'] = 0
    # fields = [PointField('x', 0, PointField.FLOAT32, 1),
    #       PointField('y', 4, PointField.FLOAT32, 1),
    #       PointField('z', 8, PointField.FLOAT32, 1),
    #       PointField('rgb', 16, PointField.UINT32, 1),
    #       ]
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'laser_link_L'
    #p = pcl.PointCloud(np.array(pc_array, dtype=np.float32))
    #pc_msg = ros_numpy.msgify(PointCloud2, pc_array, stamp='str1', frame_id='str2')
    pc2 = ros_numpy.msgify(PointCloud2, pc_array, stamp=header.stamp, frame_id=header.frame_id)
    #print(type(pc_msg))
    #pc2 = point_cloud2.create_cloud(header, fields, pc_array)
    #print(p)
    pub_ka.publish(pc2)
    
    #print(pcd.points)
    # Initialise the visualiser
    vis.clear_geometries() 
    vis.add_geometry(pcd)
    #vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    #vis.run()
    #open3d.visualization.draw_geometries([pcd])
    # 실행 코드 부분 
    #print(type(pcd.points))
    # open3d.visualization.draw_geometries([pcd],
    #                               zoom=0.3412,
    #                               front=[0.4257, -0.2125, -0.8795],
    #                               lookat=[2.6172, 2.0475, 1.532],
    #                               up=[-0.0694, -0.9768, 0.2024])
    #pub.publish(cloud_new)
nodeFrom = '/cyglidar_h2/camera_L/points'
nodeTo = publish_topic_name
if __name__ == "__main__":
    rospy.Subscriber(nodeFrom, PointCloud2, callback)
    #pub = rospy.Publisher("/velodyne_points_new", PointCloud, queue_size=1)
    vis.create_window(
        window_name='Segmented Scene',
        width=960,
        height=540,
        left=480,
        top=270)
    #vis.create_window()
    #vis.get_render_option().background_color = [0.0, 0.0, 0.0]
    #vis.get_render_option().point_size = 3
    vis.add_geometry(pcd)
    #vis.run()

    rospy.spin()