import rospy
import ros_numpy
import numpy as np
import tf

from arm_controller import ArmController
from orthographic import OrthoNet
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from scipy.spatial.transform import Rotation as R

if __name__ == '__main__':
   rospy.init_node('experiment_0')
   listener = tf.TransformListener()


   # pub = rospy.Publisher('marker', Marker, latch=True, queue_size=1)
   pub = rospy.Publisher('grasp_pose', PoseStamped, latch=True, queue_size=1)
   cloud_data = rospy.wait_for_message('/camera/depth/points', PointCloud2)
   xyzs = ros_numpy.point_cloud2.get_xyz_points(ros_numpy.numpify(cloud_data))
   rospy.sleep(1)

   onet = OrthoNet()
   # position, orientation, y, width = onet.predict(xyzs, roi=[-2, 1, -.15, .25, 0, 0.2], predictor=OrthoNet.manual_predictor)
   position, orientation, angle, width = onet.predict(xyzs, roi=[-2, 1, -.15, .25, 0, 0.2], predictor=OrthoNet.manual_predictor)
   print('Orientation {}'.format(orientation))
   print('Angle: {}'.format(angle))
   angle = np.arccos(np.dot(orientation.squeeze(), angle.squeeze()))
   print('Angle: {}'.format(np.degrees(angle)))

   tr, qt = listener.lookupTransform('m1n6s200_link_base', 'camera_depth_optical_frame', rospy.Time(0))

   r = R.from_quat(np.array(qt))
   position = r.apply(position)
   position = np.add(position, tr)

   orientation = r.apply(orientation)

   # quat = R.from_rotvec(orientation).as_quat()
   # rpy = R.from_rotvec(orientation*angle).as_euler('zyx')
   # quat = quaternion_from_euler(rpy[0, 0], rpy[0, 1], rpy[0, 2])

   pose = PoseStamped()
   pose.header.frame_id = 'm1n6s200_link_base'
   # pose.header.frame_id = 'camera_depth_optical_frame'
   pose.header.stamp = rospy.Time.now()
   pose.pose.position.x = position[0, 0]
   pose.pose.position.y = position[0, 1]
   pose.pose.position.z = position[0, 2]
   s = np.sin(angle/2.)
   pose.pose.orientation.x = orientation[0, 0] * s
   pose.pose.orientation.y = orientation[0, 1] * s
   pose.pose.orientation.z = orientation[0, 2] * s
   pose.pose.orientation.w = np.cos(angle/2.)

   # pose.pose.orientation.x = quat[0, 3]
   # pose.pose.orientation.y = quat[0, 0]
   # pose.pose.orientation.z = quat[0, 1]
   # pose.pose.orientation.w = quat[0, 2]

   # pose.pose.orientation.x = quat[3]
   # pose.pose.orientation.y = quat[0]
   # pose.pose.orientation.z = quat[1]
   # pose.pose.orientation.w = quat[2]
   # print 'sending'
   pub.publish(pose)

   # controller = ArmController()
   # x, y, z = position[0]
   # p = pose.pose.orientation
   # quat = [p.w, p.x, p.y, p.z]
   # r, p, yw = euler_from_quaternion(quat)
   # controller.grasp(x, y, z, r, p, yw)
   # controller.open_fingers()
   # controller.move_to(0, -0.2, .3, -np.pi, 0, 0)



