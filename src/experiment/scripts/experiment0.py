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
   position, orientation, angle, width = onet.predict(xyzs, roi=[-2, 1, -.15, .25, 0, 0.2], predictor=OrthoNet.manual_predictor)
   # position = np.array([[-0.03678347, -0.14620348,  0.67425354]])
   # orientation = np.array([[-0.02538166, -0.73814097,  0.84091622]])

   tr, qt = listener.lookupTransform('m1n6s200_link_base', 'camera_depth_optical_frame', rospy.Time(0))

   r = R.from_quat(np.array(qt))
   position = r.apply(position)
   position = np.add(position, tr)

   orientation = r.apply(orientation)
   orientation = np.add(orientation, tr)

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


   # print 'sending'
   pub.publish(pose)

   # controller = ArmController()
   # x, y, z = position[0]
   # p = pose.pose.orientation
   # quat = [p.w, p.x, p.y, p.z]
   # r, p, yw = euler_from_quaternion(quat)
   # controller.grasp(x, y, z, r, p, yw)
   # controller.open_fingers()



