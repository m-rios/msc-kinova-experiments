import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
import numpy as np

if __name__ == '__main__':
    rospy.init_node('save_cloud_to_file')
    cloud_data = rospy.wait_for_message('/camera/depth/points', PointCloud2)
    xyzs = ros_numpy.point_cloud2.get_xyz_points(ros_numpy.numpify(cloud_data))
    np.save('points.npy', xyzs)
