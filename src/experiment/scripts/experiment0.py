import rospy
import ros_numpy
import numpy as np
import tf

from arm_controller import ArmController
from orthographic import OrthoNet
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_from_matrix, euler_matrix
from scipy.spatial.transform import Rotation as R


def fix_vector(v, point)
    """
    Make sure the approach path lies between object and base arm
    :param v: approach vector w.r.t camera
    :param point: grasp point w.r.t camera
    :return: fixed approach vector
    """
    v =


if __name__ == '__main__':
    rospy.init_node('experiment_0')
    listener = tf.TransformListener()

    pub = rospy.Publisher('grasp_pose', PoseStamped, latch=True, queue_size=1)
    cloud_data = rospy.wait_for_message('/camera/depth/points', PointCloud2)
    xyzs = ros_numpy.point_cloud2.get_xyz_points(ros_numpy.numpify(cloud_data))
    rospy.sleep(1)

    base_platform = PointStamped(Header(frame_id='m1n6s200_link_base'), Point())
    base_platform = listener.transformPoint('camera_depth_optical_frame', base_platform)
    base_platform = np.array([base_platform.point.x, base_platform.point.y, base_platform.point.z])


    onet = OrthoNet()
    controller = ArmController()
    while not rospy.is_shutdown():
        positions, zs, ys, widths = onet.predict(xyzs, roi=[-2, 1, -.15, .25, 0, 0.2], predictor=OrthoNet.manual_predictor)

        for idx in range(len(positions)):
            position = positions[idx]
            z = zs[idx]
            y = ys[idx]
            width = widths[idx]

            position = position.squeeze()
            z = z.squeeze()
            # Force z to be the closest direction to base platform
            if z[2]
            if np.abs(np.linalg.norm(position - base_platform)) < np.abs(np.linalg.norm(position + z - base_platform)):
                print 'z should be inverted'
            y = y.squeeze()
            x = np.cross(y, z)

            r = np.vstack((x, y, z)).T

            rpy = euler_from_matrix(r)
            q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])

            pose = PoseStamped()
            pose.header.frame_id = 'camera_depth_optical_frame'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = position[0]
            pose.pose.position.y = position[1]
            pose.pose.position.z = position[2]
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            pose = listener.transformPose('m1n6s200_link_base', pose)

            pub.publish(pose)
            rospy.sleep(1)


            p = pose.pose.position
            x, y, z = (p.x, p.y, p.z)
            p = pose.pose.orientation
            quat = [p.x, p.y, p.z, p.w]
            r, p, yw = euler_from_quaternion(quat)
            if controller.plan_grasp(x, y, z, r, p, yw, width):
                print 'Found successful grasp'
                controller.grasp(x, y, z, r, p, yw, width)
                controller.open_fingers()
                controller.move_to(0, -0.2, .3, -np.pi, 0, 0)
                break
            print 'Grasp unsuccessful, mirroring approach'



            print 'Mirrored approach unsuccessful, skipping'
        break
