#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from bumblebee_reconfigure.cfg import Ld06Config
import numpy as np
import open3d as o3d
import ros_numpy
import copy
from std_msgs.msg import Header, String
from sensor_msgs.msg import PointCloud2, LaserScan
from laser_geometry import LaserProjection
from visualization_msgs.msg import Marker, MarkerArray
import rviz_visualizer as visual

vis = o3d.visualization.VisualizerWithKeyCallback()
pcd = o3d.geometry.PointCloud()
v3d = o3d.utility.Vector3dVector

def laserscan_callback(input_ros_msg):
    global pcd
    global vis
    global pub_ka
    global v3d
    
    # print(type(input_ros_msg))
        
    if isinstance(input_ros_msg, LaserScan):
        # rospy.loginfo("type: %s", type(input_ros_msg))
        pc2_msg = LaserProjection().projectLaser(input_ros_msg)
        
        pcd.points = v3d(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg))
                
    else:
        rospy.loginfo("Unknown message type: %s", type(input_ros_msg))

    
    voxel_size = 0.02
    
    if voxel_size > 0:    
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        
    rangeMax =  np.array([rangeMaxX, rangeMaxY, rangeMaxZ])
    rangeMin =  np.array([rangeMinX, rangeMinY, rangeMinZ])
    
    # 거리 필터 적용
    crop_data = crop_to_pcd(pcd, minbound=rangeMin, maxbound=rangeMax)
    
    vis.clear_geometries() 
    
    if crop_data.is_empty():
        pass
    else:
        # with o3d.utility.VerbosityContextManager(
        # o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            crop_data.cluster_dbscan(eps=0.1, min_points=5, print_progress=False))
        T = np.eye(4)
        T[:3, :3] = crop_data.get_rotation_matrix_from_xyz((0, np.pi/2 * 4, np.pi/2 * 3 ))
        T[0, 3] = 1
        T[1, 3] = 1.3
        
        R_filtered_pcd = copy.deepcopy(crop_data).transform(T)
        
        # R_filtered_pcd=R_filtered_pcd.transform([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 3]])

        # vis.add_geometry(R_filtered_pcd)
        
        if viewO3D:
            vis.add_geometry(R_filtered_pcd)
            bounding_boxes(R_filtered_pcd, labels)
        
        if viewPublishPointcloud:
            publish_pointcloud(crop_data, labels)
            line_marker()

# 이전 매개변수 값들을 저장하는 전역 딕셔너리
previous_config = {}

def config_callback(config, level):
    
    global previous_config
    # 매개변수 변경 시 호출되는 콜백 함수
    for key in config.keys():
        if key not in previous_config or config[key] != previous_config[key]:
            rospy.loginfo("Parameter [{}] {}".format(key, config[key]))
            
    # 현재 매개변수 값을 이전 매개변수 값으로 저장
    previous_config = config.copy()
    
    # 매개변수 변경 시 호출되는 콜백 함수
    global viewO3D, viewPublishPointcloud, subscribeTopicName, publishTopicName, distanceTopicName, publishFrameId, rangeMaxX, rangeMaxY, rangeMaxZ, rangeMinX, rangeMinY, rangeMinZ
    
    viewO3D = config.view_o3d
    viewPublishPointcloud = config.view_publish_pointcloud
    subscribeTopicName = config.subscribe_topic_name
    publishTopicName = config.publish_topic_name
    distanceTopicName = config.distance_topic_name
    publishFrameId = config.publish_frame_id

    rangeMaxX = config.range_max_x
    rangeMaxY = config.range_max_y
    rangeMaxZ = config.range_max_z
    rangeMinX = config.range_min_x 
    rangeMinY = config.range_min_y
    rangeMinZ = config.range_min_z
            
    return config

def initialise():
    rospy.loginfo("range_max_x %s", rospy.get_param("~range_max_x", default=5))
    rospy.loginfo("range_max_y %s", rospy.get_param("~range_max_y", default=5))
    rospy.loginfo("range_max_z %s", rospy.get_param("~range_max_z", default=5))
    rospy.loginfo("range_min_x %s", rospy.get_param("~range_min_x", default=-5))
    rospy.loginfo("range_min_y %s", rospy.get_param("~range_min_y", default=-5))
    rospy.loginfo("range_min_z %s", rospy.get_param("~range_min_z", default=-5))
    

def crop_to_pcd(pcd, minbound, maxbound):
    crop_pcd = pcd.crop(
        o3d.geometry.AxisAlignedBoundingBox(min_bound=minbound, max_bound=maxbound))
    return crop_pcd


def distance_between_points(point1, point2):
    return np.linalg.norm(point2 - point1)


def point_to_str(point):
    pointToStr = point_to_list(point)
    return ",".join(pointToStr)


def point_to_list(point):
    return list(map(lambda x: "{:.3f}".format(x), point))
   
    
def bounding_boxes(pcd, labels):
    bounding_boxes = []
    volumes = []
    
    # max_label = labels.max()
    # colors = np.random.rand(max_label + 1, 3)
    # pcd.colors = o3d.utility.Vector3dVector(colors[labels])
    
    for label in np.unique(labels):
        cluster_points = pcd.select_by_index(np.where(labels == label)[0])
        bounding_box = cluster_points.get_axis_aligned_bounding_box()
        center_box = cluster_points.get_center()
        # 바운딩 박스의 가로, 세로, 높이 값 구하기
        width = bounding_box.max_bound[0] - bounding_box.min_bound[0]
        height = bounding_box.max_bound[1] - bounding_box.min_bound[1]
        depth = bounding_box.max_bound[2] - bounding_box.min_bound[2]
        distance = distance_between_points(np.zeros(3),center_box)
        volume = width * height * depth
        # if width > 0.1:
        # print(f"박스[{label+1}]의 부피: {volume:.3f} 입방 단위, 깊이:{depth:.2f}m, 중심점:{distance:.2f}m")
        volumes.append(volume)
        bounding_box.color = (1, 0, 0)
        bounding_boxes.append(bounding_box)

        center_point = point_to_str(center_box)
        # print(f"box[{label+1}] Center: {center_point} width: {width:.2f} height: {height:.2f} depth: {depth:.2f} distance: {distance:.2f}")
        distance_center_point = f"boxCenter: {center_point} width: {width:.2f} height: {height:.2f} depth: {depth:.2f}"
        pub_distance.publish(distance_center_point)
            
    for bounding_box in bounding_boxes:
        vis.add_geometry(bounding_box)
        vis.update_geometry(bounding_box)
        
        # vis.add_3d_label(volumes)
    # vis.add_geometry(filtered_pcd)
    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    
    return bounding_boxes


def publish_pointcloud(input_ros_msg, labels):
    global pub_ka
    
    pc = np.asarray(input_ros_msg.points)
    pc_array = np.zeros(len(pc), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
    ])
    pc_array['x'] = pc[:, 0]
    pc_array['y'] = pc[:, 1]
    pc_array['z'] = pc[:, 2]
    pc_array['intensity'] = labels

    pc2 = get_msgify(pc_array)
    pub_ka.publish(pc2)
  
    
def get_msgify(pc_array):
    
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = publishFrameId
    return ros_numpy.msgify(PointCloud2, pc_array, stamp=header.stamp, frame_id=header.frame_id)   

def line_marker():
    global rviz_pub
    ids = list(range(0, 100))

    # linelist (라인리스트)
    line_points = [[rangeMinX, rangeMinY, 0], [rangeMaxX, rangeMinY, 0], [rangeMinX, rangeMaxY, 0], [rangeMaxX, rangeMaxY, 0]]
    linelist = visual.linelist_rviz(
        name="linelist",
        id=ids.pop(),
        lines=line_points,
        color_r=255,
        color_g=0,
        color_b=0,
        scale=0.01,
    )
    all_markers = visual.marker_array_rviz([linelist])

    rviz_pub.publish(all_markers)
    
def main():
    rospy.init_node("bumblebee_Itops_reconfigure", anonymous = False)
    # initialise()
    Server(Ld06Config, config_callback)
    global pub_ka, pub_distance, rviz_pub
    
    pub_ka = rospy.Publisher(publishTopicName, PointCloud2, queue_size=1)
    pub_distance = rospy.Publisher(distanceTopicName, String, queue_size=1)
    rviz_pub = rospy.Publisher("/visual_rviz", MarkerArray, queue_size=0)
    
    if viewO3D:
        vis.create_window(
            window_name='LaserScan Scene',
            width=960,
            height=540,
            left=480,
            top=500)
        opt = vis.get_render_option()
        opt.point_size = 2
    rospy.Subscriber(subscribeTopicName, LaserScan, laserscan_callback)
    rospy.spin()

if __name__ == "__main__":
    main()