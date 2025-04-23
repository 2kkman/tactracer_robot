import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pcl
import numpy as np
from UtilBLB import *
def is_robot_arm_clear(start_point, end_point, width, thickness, height, cloud):
    """
    Check if the area for the robot arm is clear.
    
    :param start_point: (x, y, z) tuple for the start point of the robot arm
    :param end_point: (x, y, z) tuple for the end point of the robot arm
    :param width: Width of the robot arm
    :param thickness: Thickness of the robot arm
    :param height: Height of the robot arm extension
    :param cloud: PointCloud2 message
    :return: Tuple (is_clear, PointCloud2 of obstructing points)
    """
    # Create a PCL PointCloud from the PointCloud2 message
    points_list = list(pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
    pcl_data = pcl.PointCloud()
    pcl_data.from_list(points_list)

    # Define the target area (bounding box)
    x_min = min(start_point[0], end_point[0]) - width / 2
    x_max = max(start_point[0], end_point[0]) + width / 2
    y_min = min(start_point[1], end_point[1]) - thickness / 2
    y_max = max(start_point[1], end_point[1]) + thickness / 2
    z_min = min(start_point[2], end_point[2]) - height / 2
    z_max = max(start_point[2], end_point[2]) + height / 2

    # Apply PassThrough filters to the PCL data
    passthrough = pcl_data.make_passthrough_filter()

    # Filter by x axis
    passthrough.set_filter_field_name("x")
    passthrough.set_filter_limits(x_min, x_max)
    cloud_filtered = passthrough.filter()

    # Filter by y axis
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name("y")
    passthrough.set_filter_limits(y_min, y_max)
    cloud_filtered = passthrough.filter()

    # Filter by z axis
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name("z")
    passthrough.set_filter_limits(z_min, z_max)
    cloud_filtered = passthrough.filter()

    # If the filtered cloud is not empty, there is an obstruction
    is_clear = cloud_filtered.size == 0

    if is_clear:
        return True, None
    else:
        # Convert the filtered cloud to PointCloud2 message
        header = cloud.header
        obstructing_points = pc2.create_cloud_xyz32(header, cloud_filtered.to_list())
        return False, obstructing_points

def point_cloud_callback(msg):
    msg2 = remove_points_near_origin(msg, 0.2)
    
    # Example start and end points, dimensions
    start_point = (0.0, 0.0, 0.0)  # Example start point
    end_point = (0.5, 0.0, 0.0)  # Example end point (50 cm extension)
    width = 0.1  # 10 cm width
    thickness = 0.1  # 10 cm thickness
    height = 1.0  # 1 m height

    is_clear, obstructing_points = is_robot_arm_clear(start_point, end_point, width, thickness, height, msg2)
    if is_clear:
        rospy.loginfo("Area is clear for the robot arm to extend.")
    else:
        rospy.logwarn("Area is not clear. Obstruction detected.")
        obstacles_cloud_pub.publish(obstructing_points)

if __name__ == '__main__':
    rospy.init_node('robot_arm_clear_check')
    obstacles_cloud_pub = rospy.Publisher('/OBS', PointCloud2, queue_size=10)
    rospy.Subscriber('/scan_3D', PointCloud2, point_cloud_callback)
    rospy.spin()
