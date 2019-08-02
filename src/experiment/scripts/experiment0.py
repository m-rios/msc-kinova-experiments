import rospy
import ros_numpy
import numpy as np
import tf

from arm_controller import ArmController
from orthographic import OrthoNet
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_from_matrix, euler_matrix
from scipy.spatial.transform import Rotation as R


def rotm2quat(m):
    tr = np.trace(m)
    if tr > 0:
        s = np.sqrt(tr + 1.) * 2;
        qw = s/4.
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1,1] - m[2,2]) * 2
        qw = (m[2,1] - m[1,2]) / s
        qx = 0.25 * s
        qy = (m[0,1] + m[1,0]) / s
        qz = (m[0,2] + m[2,0]) / s
    elif (m[1,1] > m[2,2]):
        s = np.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2
        qw = (m[0,2] - m[2,0]) / s
        qx = (m[0,1] + m[1,0]) / s
        qy = 0.25 * s
        qz = (m[1,2] + m[2,1]) / s
    else:
        s = np.sqrt(1.0 + m[2,2]- m[0,0] - m[1,1]) * 2
        qw = (m[1,0] - m[0,1]) / s
        qx = (m[0,2] + m[2,0]) / s
        qy = (m[1,2] + m[2,1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


if __name__ == '__main__':
    rospy.init_node('experiment_0')
    listener = tf.TransformListener()

    pub = rospy.Publisher('grasp_pose', PoseStamped, latch=True, queue_size=1)
    cloud_data = rospy.wait_for_message('/camera/depth/points', PointCloud2)
    xyzs = ros_numpy.point_cloud2.get_xyz_points(ros_numpy.numpify(cloud_data))
    rospy.sleep(1)

    onet = OrthoNet()
    while not rospy.is_shutdown():
        position, z, y, width = onet.predict(xyzs, roi=[-2, 1, -.15, .25, 0, 0.2], predictor=OrthoNet.manual_predictor)

        position = position.squeeze()
        z = z.squeeze()
        y = y.squeeze()
        x = np.cross(y, z)

        r = np.vstack((x, y, z)).T

        rpy = euler_from_matrix(r)
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])

        pose = PoseStamped()
        # pose.header.frame_id = 'm1n6s200_link_base'
        pose.header.frame_id = 'camera_depth_optical_frame'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        print pose
        pose = listener.transformPose('m1n6s200_link_base', pose)
        print pose

        pub.publish(pose)
        rospy.sleep(1)

        controller = ArmController()
        p = pose.pose.position
        x, y, z = (p.x, p.y, p.z)
        p = pose.pose.orientation
        quat = [p.x, p.y, p.z, p.w]
        r, p, yw = euler_from_quaternion(quat)
        controller.grasp(x, y, z, r, p, yw)
        controller.open_fingers()
        controller.move_to(0, -0.2, .3, -np.pi, 0, 0)
        # raw_input('Press ENTER to continue')
        break
