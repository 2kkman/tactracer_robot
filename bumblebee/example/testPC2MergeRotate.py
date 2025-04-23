import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import math

def rotate_point(point, angle):
    """
    Rotate a point around the z-axis by the given angle.
    
    :param point: The point to rotate (x, y, z).
    :param angle: The angle in radians.
    :return: Rotated point (x, y, z).
    """
    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)
    x, y, z = point
    x_new = x * cos_theta - y * sin_theta
    y_new = x * sin_theta + y * cos_theta
    return x_new, y_new, z

def merge_point_clouds(angle_pc2_dict):
    """
    Merge multiple PointCloud2 messages into a single PointCloud2 message.
    
    :param angle_pc2_dict: A dictionary with angles as keys and PointCloud2 messages as values.
    :return: A single merged PointCloud2 message.
    """
    all_points = []
    header = None

    for angle, pc2_msg in angle_pc2_dict.items():
        # Convert angle from degrees to radians
        theta = np.radians(angle)
        
        # Extract points from the PointCloud2 message
        points = pc2.read_points(pc2_msg, field_names=("x", "y", "z"), skip_nans=True)
        rotated_points = [rotate_point(point, theta) for point in points]
        
        all_points.extend(rotated_points)
        
        # Save the header from one of the messages (assuming all headers are similar)
        if header is None:
            header = pc2_msg.header

    # Create a new PointCloud2 message from the combined points
    merged_pc2 = pc2.create_cloud_xyz32(header, all_points)
    return merged_pc2

if __name__ == '__main__':
    rospy.init_node('merge_point_clouds_node')

    # Example usage
    # Replace this with your actual angle to PointCloud2 dictionary
    angle_pc2_dict = {
        0: PointCloud2(),   # Example PointCloud2 messages
        90: PointCloud2(),
        180: PointCloud2(),
        270: PointCloud2()
    }

    merged_pc2 = merge_point_clouds(angle_pc2_dict)

    # Example publisher to publish the merged PointCloud2
    merged_cloud_pub = rospy.Publisher('/merged_pointcloud', PointCloud2, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        merged_cloud_pub.publish(merged_pc2)
        rate.sleep()
