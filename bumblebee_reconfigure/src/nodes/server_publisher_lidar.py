#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from bumblebee_reconfigure.cfg import ItopsConfig   
import numpy as np
import open3d as o3d
import ros_numpy
import copy
from std_msgs.msg import Header, String
from laser_geometry import LaserProjection
from sensor_msgs.msg import PointCloud2, PointField, PointCloud, LaserScan


def callback(input_ros_msg):
    """_summary_

    Args:
        input_ros_msg (_type_): _description_
    """
    global pcd
    global vis
    global pub_ka
    
    v3d = o3d.utility.Vector3dVector
    
    # print(type(input_ros_msg))
    
    if isinstance(input_ros_msg, PointCloud):
        # rospy.loginfo("type: %s", type(input_ros_msg))
        # pcd.points = v3d(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg))
        pass
    elif isinstance(input_ros_msg, LaserScan):
        # rospy.loginfo("type: %s", type(input_ros_msg))
        # pc2_msg = LaserProjection().projectLaser(input_ros_msg)
        # pcd.points = v3d(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg))
        pass
    elif isinstance(input_ros_msg, PointCloud2):
        # rospy.loginfo("type: %s", type(input_ros_msg))        
        pcd.points = v3d(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(input_ros_msg))
    else:
        rospy.loginfo("Unknown message type: %s", type(input_ros_msg))
    # pcd.compute_vertex_normals()
    # print(f"포인트 클라우드의 크기: {len(pcd.points)}")
    # 거리 필터 적용
    voxel_size = 0.02
    if voxel_size > 0:    
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        
    # rangeMax =  np.array([rangeMaxY, rangeMaxZ, rangeMaxX])
    # rangeMin =  np.array([rangeMinY, rangeMinZ, rangeMinX])
    rangeMax =  np.array([rangeMaxX, rangeMaxY, rangeMaxZ])
    rangeMin =  np.array([rangeMinX, rangeMinY, rangeMinZ])
    
    #print(rangeMax, rangeMin)
    
    #F07 라이다의 PC2 위상을 로봇 3D 컨벤션 표준으로 변경한다.
    #Z -> 표준X , X -> 표준y * -1 , Y -> 표준Z * -1
    pcd_tf=pcd.transform([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
    
    #정의해둔 rangeXYZ 에 맞게 PC2 를 잘라낸다.
    crop_data = None
    try:
        crop_data = pcd.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=rangeMin, max_bound=rangeMax))
    except Exception as e:
        print(rangeMin,rangeMax,e)
    
    if crop_data is None:
        return
    
    vis.clear_geometries() 
    R_filtered_pcd = copy.deepcopy(crop_data)
    #F07 라이다의 Z,Y축이 거울처럼 반전되어있기 때문에 반전시킨다
    #R_filtered_pcd=R_filtered_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    
    
    # alpha = 0.03
    # R_filtered_pcd= o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)

    if R_filtered_pcd.is_empty():
        pass
    else:
        labels = np.array(
            R_filtered_pcd.cluster_dbscan(eps=0.1, min_points=11, print_progress=False))
    
        if viewO3D:
            vis.add_geometry(R_filtered_pcd)

        if viewPublishPointcloud:
            publish_pointcloud(crop_data, labels)
            bounding_boxes(R_filtered_pcd, labels)


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
    # rospy.loginfo("view_o3d, {view_o3d}".format(**config))
    # rospy.loginfo("view_publish_pointcloud, {view_publish_pointcloud}".format(**config))
    # rospy.loginfo("subscribe_topic_name, {subscribe_topic_name}".format(**config))
    # rospy.loginfo("publish_topic_name, {publish_topic_name}".format(**config))
    # rospy.loginfo("publish_frame_id, {publish_frame_id}".format(**config))
    
    viewO3D = False#config.view_o3d
    viewPublishPointcloud = config.view_publish_pointcloud
    subscribeTopicName = config.subscribe_topic_name
    publishTopicName = config.publish_topic_name
    distanceTopicName = config.distance_topic_name
    publishFrameId = config.publish_frame_id

    # rospy.loginfo("range_max_xyz, [{range_max_x}, {range_max_y}, {range_max_z}]".format(**config))
    # rospy.loginfo("range_min_xyz, [{range_min_x}, {range_min_y}, {range_min_z}]".format(**config))
    rangeMaxX = config.range_max_x
    rangeMaxY = config.range_max_y
    rangeMaxZ = config.range_max_z
    rangeMinX = config.range_min_x 
    rangeMinY = config.range_min_y
    rangeMinZ = config.range_min_z
    # rangeMaxZ = config.range_max_x
    # rangeMaxX = config.range_max_y
    # rangeMaxY = config.range_max_z
    # rangeMinZ = config.range_min_x 
    # rangeMinX = config.range_min_y
    # rangeMinY = config.range_min_z
    return config

def initialise():

    rospy.loginfo("range_max_x %s", rospy.get_param("~range_max_x", default=5))
    rospy.loginfo("range_max_y %s", rospy.get_param("~range_max_y", default=5))
    rospy.loginfo("range_max_z %s", rospy.get_param("~range_max_z", default=5))
    rospy.loginfo("range_min_x %s", rospy.get_param("~range_min_x", default=-5))
    rospy.loginfo("range_min_y %s", rospy.get_param("~range_min_y", default=-5))
    rospy.loginfo("range_min_z %s", rospy.get_param("~range_min_z", default=-5))
    
vis = o3d.visualization.VisualizerWithKeyCallback()
pcd = o3d.geometry.PointCloud()


def crop_to_pcd(pcd, minbound, maxbound):
    """_summary_

    Args:
        pcd (points): pointcloud data
        minbound (LIST): min bound
        maxbound (LIST): max bound

    Returns:
        _type_: _description_
    """
    crop_pcd = pcd.crop(
        o3d.geometry.AxisAlignedBoundingBox(min_bound=minbound, max_bound=maxbound))
    return crop_pcd


def between_points(point1, point2):
    return np.linalg.norm(point2 - point1)


def point_to_str(point):
    """_summary_

    Args:
        point (points): _description_

    Returns:
        _type_: _description_
    """
    pointToStr = point_to_list(point)
    return ",".join(pointToStr)


def point_to_list(point):
    return list(map(lambda x: "{:.3f}".format(x), point))
   
    
def bounding_boxes(pcd, labels):
    """_summary_

    Args:
        pcd (points): pointcloud data
        labels (channels): pointcloud cluster labels

    Returns:
        _type_: bounding boxes
    """
    
    bounding_boxes = []
    volumes = []
    for label in np.unique(labels):
        cluster_points = pcd.select_by_index(np.where(labels == label)[0])
        bounding_box = cluster_points.get_axis_aligned_bounding_box()
        center_box = cluster_points.get_center()
        # 바운딩 박스의 가로, 세로, 높이 값 구하기
        width = between_points(bounding_box.min_bound[0], bounding_box.max_bound[0])
        height = between_points(bounding_box.min_bound[1], bounding_box.max_bound[1])
        depth = between_points(bounding_box.min_bound[2], bounding_box.max_bound[2])
        distance = between_points(np.zeros(3),center_box)
        volume = width * height * depth
        if width > 0.1:
            #print(f"박스[{label+1}]의 부피: {volume:.3f} 입방 단위, 깊이:{depth:.2f}m, 중심점:{distance:.2f}m")
            volumes.append(volume)
            bounding_box.color = (1, 0, 0)
            bounding_boxes.append(bounding_box)

            center_point = point_to_str(center_box)
            distance_center_point = f"boxCenter:{center_point}`width:{height:.2f}`height: {width:.2f}`depth: {depth:.2f}`distance:{distance:.2f}"
            pub_distance.publish(distance_center_point)
            
    for bounding_box in bounding_boxes:
        vis.add_geometry(bounding_box)
        # vis.add_3d_label(volumes)
    # vis.add_geometry(filtered_pcd)
    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    
    return bounding_boxes


def publish_pointcloud(input_ros_msg, labels):
    """_summary_

    Args:
        input_ros_msg (sensor_msgs): LaserScan data, PointCloud2 data, PointCloud data
        labels (np.float32): intensity, distance, rgb
    """
    global pub_ka
    global pub_mean
    
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
    # numpy를 사용해 각각의 평균값 계산
    x_mean = np.mean(pc_array['x'])
    x_std = np.std(pc_array['x'])
    x_mean = round(float(x_mean),4)
    x_std = round(float(x_std),4)
    #y_mean = np.mean(pc_array['y'])
    #z_mean = np.mean(pc_array['z'])
    #print(x_mean,y_mean,z_mean)
    fMsg = f'{x_mean},{x_std},{len(input_ros_msg.points)}'
    pub_mean.publish(fMsg)
    rospy.loginfo(fMsg)
    
  
    
def get_msgify(pc_array):
    """_summary_

    Args:
        pc_array (pointcloud): pointcloud array data

    Returns:
        _type_: pointcloud2 data
    """
    
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = publishFrameId
    return ros_numpy.msgify(PointCloud2, pc_array, stamp=header.stamp, frame_id=header.frame_id)   


def main():
    rospy.init_node("bumblebee_Itops_reconfigure", anonymous = False)
    # initialise()
    Server(ItopsConfig, config_callback)
    global pub_ka, pub_distance,pub_mean
    
    pub_ka = rospy.Publisher(publishTopicName, PointCloud2, queue_size=1)
    pub_distance = rospy.Publisher(distanceTopicName, String, queue_size=1)
    pub_mean = rospy.Publisher('detect_cropped_distance', String, queue_size=1)
    
    if viewO3D:
        vis.create_window(
            window_name='Segmented Scene',
            width=960,
            height=540,
            left=480,
            top=270)
        opt = vis.get_render_option()
        opt.point_size = 2
    rospy.Subscriber(subscribeTopicName, PointCloud2, callback)
    rospy.spin()

    pass


if __name__ == "__main__":
    main()