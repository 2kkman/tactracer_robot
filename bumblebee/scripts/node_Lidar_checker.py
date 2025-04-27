#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from UtilBLB import *


# ▼ 테스트 파라미터 ▼
BASKET_WIDTH_CM = 60
BASKET_LENGTH_CM = 55
LIDAR_TO_BASKET_BOTTOM_CM = 10
Z_IGNORE_THRESHOLD_MIN = 0.0        # 아래로 향하므로 Z는 점점 커짐
Z_IGNORE_THRESHOLD_MAX = 3.0        # 3m까지만 탐지
MIN_VALID_POINTS = 5
mm_float_precion = 3
dicAndroid = {}
# 글로벌
cropped_pub = None
#marker_pub = None
last_log_time = 0
fieldvalue = f'topicname={TopicName.ANDROID.name}'

log_period = float(rospy.get_param(f"~{ROS_PARAMS.lidar_interval.name}", default=0.2))
ground_threshold = float(rospy.get_param(f"~{ROS_PARAMS.lidar_gnd_margin.name}", default=0.04))
ground_distance_limit = float(rospy.get_param(f"~{ROS_PARAMS.lidar_gnd_limit.name}", default=0.560))
point_threshold = int(rospy.get_param(f"~{ROS_PARAMS.lidar_obstacle_points.name}", default=3))

ipAddr = GetMasterIP()
# Ground보다 위쪽(bin_z > ground_z) 중 points threshold 넘는 것만 배열로 리턴하는 버전
def find_obstacle_candidates(points, tilt_deg=31, left_x=-0.33, right_x=0.28, crop_y=0.55, max_descend_distance=2.0,
                              bin_size=0.005, point_threshold=3, ground_threshold =0.04 ):
    """
    Ground 계산 후, Ground보다 라이다쪽에 있는 장애물 후보 리스트를 리턴하는 버전
    points: np.array(N, 3)
    tilt_deg: 라이다 틸트 각도
    crop_x, crop_y: crop 범위 (m)
    max_descend_distance: 최대 하강 거리 (m)
    bin_size: z binning 간격 (m)
    point_threshold: 장애물 후보로 인정할 최소 포인트 수
    """
    # import numpy as np

    tilt_rad = np.deg2rad(tilt_deg)
    cos_t = np.cos(tilt_rad)
    sin_t = np.sin(tilt_rad)

    valid_points = points[~np.isnan(points[:, 2])]
    if valid_points.shape[0] == 0:
        return max_descend_distance, None, []

    # (1) 틸트 회전 적용
    rotated_points = np.zeros_like(valid_points)
    rotated_points[:, 0] = valid_points[:, 0]
    rotated_points[:, 1] = valid_points[:, 1] * cos_t + valid_points[:, 2] * sin_t
    rotated_points[:, 2] = -valid_points[:, 1] * sin_t + valid_points[:, 2] * cos_t

    # (2) crop
    cropped_points = []
    for x, y, z in rotated_points:
        if (left_x <= x <= right_x) and (0 <= y <= crop_y):
            cropped_points.append((x, y, z))

    if not cropped_points:
        return max_descend_distance, None, []

    cropped_points = np.array(cropped_points)

    # (3) Z 오름차순 정렬
    z_values = np.sort(cropped_points[:, 2])

    # (4) Z binning
    z_min, z_max = np.min(z_values), np.max(z_values)
    bins = np.arange(z_min, z_max + bin_size, bin_size)
    hist, bin_edges = np.histogram(z_values, bins=bins)

    # (5) Ground bin 탐색 (가장 포인트 많은 bin)
    if hist.size == 0:
        return max_descend_distance, None, []        
    max_bin_idx = np.argmax(hist)
    ground_z_start = min(round(bin_edges[max_bin_idx],mm_float_precion),ground_distance_limit)
    ground_z_end = bin_edges[max_bin_idx + 1]
    #ground_z_center = (ground_z_start + ground_z_end) / 2.0

    # (6) 장애물 후보 찾기
    obstacle_candidates = []
    for idx in range(len(hist)):
        if hist[idx] == 0:
            continue

        bin_start = bin_edges[idx]
        bin_end = bin_edges[idx + 1]
        bin_center = round((bin_start + bin_end) / 2.0, mm_float_precion)
        diff_bin_gnd = ground_z_start - bin_center 
        if diff_bin_gnd > ground_threshold and hist[idx] > point_threshold:
            obstacle_candidates.append({bin_center:int(hist[idx])})
            # obstacle_candidates.append({
            #     "center_z": bin_center,
            #     "point_count": hist[idx]
            # })
    
    # (7) 최종 descendable_distance는 Ground 기준
    #descendable_distance = np.clip(ground_z_center, 0, max_descend_distance)
    return ground_z_start, cropped_points, obstacle_candidates

def get_filtered_points(pointcloud):
    half_x = BASKET_WIDTH_CM / 200.0
    half_y = BASKET_LENGTH_CM / 200.0
    valid_points = []
    for point in pc2.read_points(pointcloud, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = point
        x = -x
        y = -y
        if -half_x <= x <= half_x and -half_y <= y <= half_y:
            if Z_IGNORE_THRESHOLD_MIN <= z <= Z_IGNORE_THRESHOLD_MAX:
                valid_points.append((x, y, z))
    return valid_points

def create_pc2(points, frame_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]
    return pc2.create_cloud(header, fields, points)

# def create_marker_list(points, frame_id):
#     markers = []
#     for i, (x, y, z) in enumerate(points):
#         m = Marker()
#         m.header.frame_id = frame_id
#         m.header.stamp = rospy.Time.now()
#         m.ns = "descend"
#         m.id = i
#         m.type = Marker.SPHERE
#         m.action = Marker.ADD
#         m.pose.position.x = x
#         m.pose.position.y = y
#         m.pose.position.z = z
#         m.pose.orientation.w = 1.0
#         m.scale.x = 0.03
#         m.scale.y = 0.03
#         m.scale.z = 0.03
#         m.color.r = 1.0
#         m.color.g = 0.0
#         m.color.b = 0.0
#         m.color.a = 0.8
#         markers.append(m)
#     return markers

# def create_marker(z, frame_id):
#     marker = Marker()
#     marker.header.frame_id = frame_id
#     marker.header.stamp = rospy.Time.now()
#     marker.ns = "descend"
#     marker.id = 0
#     marker.type = Marker.CUBE
#     marker.action = Marker.ADD
#     marker.pose.position.x = 0.0
#     marker.pose.position.y = 0.0
#     marker.pose.position.z = z / 2.0
#     marker.pose.orientation.w = 1.0
#     marker.scale.x = BASKET_WIDTH_CM / 100.0
#     marker.scale.y = BASKET_LENGTH_CM / 100.0
#     marker.scale.z = abs(z)
#     marker.color.r = 1.0
#     marker.color.g = 0.5
#     marker.color.b = 0.0
#     marker.color.a = 0.5
#     return marker


def pointcloud_callback(msg):
    #global cropped_pub, marker_pub, last_log_time
    global cropped_pub,last_log_time,pub_obstacle
    now = rospy.get_time()
    points = get_filtered_points(msg)

    if len(points) < MIN_VALID_POINTS:
        if now - last_log_time > log_period:
            rospy.loginfo("✅ 장애물 없음 (유효 포인트 부족)")
            last_log_time = now
        return

    z_vals = [p[2] for p in points]
    closest_obstacle_z = min(z_vals)  # 아래로 향하므로 가장 작은 Z가 가장 가까운 장애물

    lidar_to_basket_bottom = LIDAR_TO_BASKET_BOTTOM_CM / 100.0
    descendable = closest_obstacle_z - lidar_to_basket_bottom
    descendable = max(0.0, descendable)

    if now - last_log_time > log_period:
        # bReturn,strResult=API_call_http(ipAddr, HTTP_COMMON_PORT, 'DATA1', fieldvalue)
        # recvDataMap = json.loads(strResult)
        
        angle_y = try_parse_int((dicAndroid.get(DataKey.Angle_Y.name)),MIN_INT)
        if angle_y == MIN_INT:
            return
        #recvDataMap = ast.literal_eval(strResult)
        #print(recvDataMap)

        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        
        #distanceMax, cropped_points = calculate_descendable_distance_with_rotation(points,angle_y)
        #distanceMax, cropped_points = calculate_descendable_distance_with_rotation(points,angle_y)
        #distanceMax, cropped_points= calculate_descendable_distance(points,angle_y,-0.3,0.25)
        ground_z_center, cropped_points, lsDicObstacle= find_obstacle_candidates(points,angle_y,point_threshold=point_threshold,ground_threshold=ground_threshold)
        returnObstanceData = {}
        if lsDicObstacle:
            first_item = next(iter(lsDicObstacle[0].items()))
            obstacle_distance, bin_points = first_item
            returnObstanceData[OBSTACLE_INFO.LASTSEEN.name] = getDateTime().timestamp()
            returnObstanceData[OBSTACLE_INFO.GND_DISTANCE.name] = ground_z_center
            returnObstanceData[OBSTACLE_INFO.OBSTACLE_DISTANCE.name] =obstacle_distance
            returnObstanceData[OBSTACLE_INFO.OBSTACLE_POINTS.name] = bin_points
            data_out = json.dumps(returnObstanceData)
            pub_obstacle.publish(data_out)
            print(returnObstanceData)
        #bResult,obstacleBox = detect_obstacle_with_ground_estimation(cropped_points,300,0.02,0.03,0.01)
        #rospy.loginfo(f"현재각도:{angle_y},거리:{distanceMax*100:.1f}")      
        #rospy.loginfo(f"판정결과:{bResult},장애물정보:{obstacleBox}")      
        if cropped_points is not None:
            cropped_pub.publish(create_pc2(cropped_points,msg.header.frame_id))
        last_log_time = now
    # for m in create_marker_list(points, msg.header.frame_id):
    #     marker_pub.publish(m)

    # marker = create_marker(closest_obstacle_z, msg.header.frame_id)
    # marker_pub.publish(marker)

def callbackAndroid(data,topic_name=''):
    global dicAndroid
    try:
        recvData = data.data
        if is_valid_python_dict_string(recvData):
            recvDataMap = ast.literal_eval(recvData)
        elif is_json(recvData):
            recvDataMap = json.loads(recvData)
        else:
            recvDataMap = getDic_strArr(recvData, sDivFieldColon, sDivItemComma)
        dicAndroid.update(recvDataMap)
    except Exception as e:
        message = traceback.format_exc()
        rospy.loginfo(message)

def main():
    #capture_frame_from_mjpeg()
    #global cropped_pub, marker_pub
    global cropped_pub,pub_obstacle
    rospy.init_node("lidar_descend_checker", anonymous=True)
    
    rospy.Subscriber("/itops_f07/camera/points", PointCloud2, pointcloud_callback)
    rospy.Subscriber(TopicName.ANDROID.name, String, callbackAndroid)
    cropped_pub = rospy.Publisher(TopicName.LIDAR_CROPPED.name, PointCloud2, queue_size=1)
    #marker_pub = rospy.Publisher("/descend/obstacle_marker", Marker, queue_size=10)
    pub_obstacle = rospy.Publisher(f"{TopicName.LIDAR_OBSTACLE.name}", String, queue_size=ROS_TOPIC_QUEUE_SIZE)

    rospy.loginfo("✅ 하강 거리 측정 + RViz 시각화 실행 중 *)")
    rospy.spin()

if __name__ == "__main__":
    main()

