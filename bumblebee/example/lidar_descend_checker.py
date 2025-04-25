#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

# ▼ 테스트 파라미터 ▼
BASKET_WIDTH_CM = 50
BASKET_LENGTH_CM = 55
LIDAR_TO_BASKET_BOTTOM_CM = 10
Z_IGNORE_THRESHOLD_MIN = 0.0        # 아래로 향하므로 Z는 점점 커짐
Z_IGNORE_THRESHOLD_MAX = 3.0        # 3m까지만 탐지
MIN_VALID_POINTS = 5
LOG_PERIOD = 1.0                    # 몇 초마다 출력할지 (초)

# 글로벌
cropped_pub = None
marker_pub = None
last_log_time = 0


def get_filtered_points(pointcloud):
    half_x = BASKET_WIDTH_CM / 200.0
    half_y = BASKET_LENGTH_CM / 200.0
    valid_points = []

    for point in pc2.read_points(pointcloud, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = point
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


def create_marker(z, frame_id):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "descend"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = z / 2.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = BASKET_WIDTH_CM / 100.0
    marker.scale.y = BASKET_LENGTH_CM / 100.0
    marker.scale.z = abs(z)
    marker.color.r = 1.0
    marker.color.g = 0.5
    marker.color.b = 0.0
    marker.color.a = 0.5
    return marker


def pointcloud_callback(msg):
    global cropped_pub, marker_pub, last_log_time
    now = rospy.get_time()
    points = get_filtered_points(msg)

    if len(points) < MIN_VALID_POINTS:
        if now - last_log_time > LOG_PERIOD:
            rospy.loginfo("✅ 장애물 없음 (유효 포인트 부족)")
            last_log_time = now
        return

    z_vals = [p[2] for p in points]
    closest_obstacle_z = min(z_vals)  # 아래로 향하므로 가장 작은 Z가 가장 가까운 장애물

    lidar_to_basket_bottom = LIDAR_TO_BASKET_BOTTOM_CM / 100.0
    descendable = closest_obstacle_z - lidar_to_basket_bottom
    descendable = max(0.0, descendable)

    if now - last_log_time > LOG_PERIOD:
        rospy.loginfo(f"⚠️ 하강 가능 거리: {descendable*100:.1f} cm (Z={closest_obstacle_z:.3f})")
        last_log_time = now

    cropped_pc2 = create_pc2(points, msg.header.frame_id)
    cropped_pub.publish(cropped_pc2)

    marker = create_marker(closest_obstacle_z, msg.header.frame_id)
    marker_pub.publish(marker)


def main():
    global cropped_pub, marker_pub
    rospy.init_node("lidar_descend_checker", anonymous=True)

    rospy.Subscriber("/itops_f07/camera/points", PointCloud2, pointcloud_callback)
    cropped_pub = rospy.Publisher("/descend/cropped_points", PointCloud2, queue_size=1)
    marker_pub = rospy.Publisher("/descend/obstacle_marker", Marker, queue_size=1)

    rospy.loginfo("✅ 하강 거리 측정 + RViz 시각화 실행 중 (/descend/*)")
    rospy.spin()


if __name__ == "__main__":
    main()
