import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs
import numpy as np
from UtilBLB import *
import datetime

def point_cloud_callback(msg):
    global lastTimeStamp 
    if not isTimeExceeded(lastTimeStamp, 1000):
        return
    lastTimeStamp = getDateTime()
    try:
        # Transform listener
        transform = tf_buffer.lookup_transform("laser_frame", msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

        # Rotate the pointcloud
        msg2 = tf2_sensor_msgs.do_transform_cloud(msg, transform)
        cloud = pc2.read_points(msg2, field_names=("x", "y", "z"), skip_nans=True)
        
        # Convert to list of points
        points = list(cloud)
        rotated_points = rotate_point_cloud(points, 'x', 0)

        # Publish the transformed PointCloud2 message
        transformed_pc2 = pc2.create_cloud_xyz32(msg2.header, rotated_points)
        transformed_cloud_pub.publish(transformed_pc2)
        
        # Convert to Numpy array
        np_points = np.array(rotated_points, dtype=np.float32)

        # Convert to PCL PointCloud
        pcl_data = pcl.PointCloud()
        pcl_data.from_array(np_points)

        # Define the target area
        target_position = (x, y, z)
        target_dimensions = (0.1, 0.05, 0.05)

        if is_area_clear(pcl_data, target_position, target_dimensions):
            rospy.loginfo("Area is clear for the robot arm to extend.")
        else:
            rospy.loginfo("Area is not clear. Obstruction detected.")
            # Publish the obstacles point cloud
            obstacles_pc2 = pc2.create_cloud_xyz32(msg2.header, np_points)
            obstacles_cloud_pub.publish(obstacles_pc2)

    except tf2_ros.LookupException as e:
        rospy.logerr(f"Transform lookup error: {e}")
    except tf2_ros.ExtrapolationException as e:
        rospy.logerr(f"Transform extrapolation error: {e}")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")

if __name__ == '__main__':
    rospy.init_node('robot_arm_space_check')
    x, y, z = 0.5, 0.0, 0.0  # Example target position, modify as needed
    dim_x,dim_y,dim_z = 1, 0.1, 0.1
    # Create a tf2 buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transformed_cloud_pub = rospy.Publisher('/transformed_scan', PointCloud2, queue_size=10)
    obstacles_cloud_pub = rospy.Publisher('/obs', PointCloud2, queue_size=10)
    lastTimeStamp = getDateTime()
    
    rospy.Subscriber('/scan_3D', PointCloud2, point_cloud_callback)
    rospy.spin()
