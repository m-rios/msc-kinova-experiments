import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor, Grasp, GripperTranslation, MoveItErrorCodes
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from std_srvs.srv import Empty
import tf
import copy

import sys
import math
import numpy as np

class MoveIt(object):

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = PlanningSceneInterface()
        self.add_table()
        # self.clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)

        self.arm = MoveGroupCommander("arm")
        # self.arm.set_goal_joint_tolerance(0.1)
        self.gripper = MoveGroupCommander("gripper")

        # already default
        self.arm.set_planner_id("RRTConnectkConfigDefault")

        self.end_effector_link = self.arm.get_end_effector_link()

        self.arm.allow_replanning(True)
        self.arm.set_planning_time(5)

        self.transformer = tf.TransformListener()

        rospy.sleep(2) # allow some time for initialization of moveit

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def _open_gripper(self):
        joint_trajectory = JointTrajectory()
        joint_trajectory.header.stamp = rospy.get_rostime()
        joint_trajectory.joint_names = ["m1n6s200_joint_finger_1", "m1n6s200_joint_finger_2"]

        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = [0, 0]
        joint_trajectory_point.time_from_start = rospy.Duration(5.0)

        joint_trajectory.points.append(joint_trajectory_point)
        return joint_trajectory

    def _close_gripper(self):
        joint_trajectory = JointTrajectory()
        joint_trajectory.header.stamp = rospy.get_rostime()
        joint_trajectory.joint_names = ["m1n6s200_joint_finger_1", "m1n6s200_joint_finger_2"]

        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = [1.2, 1.2]
        joint_trajectory_point.time_from_start = rospy.Duration(5.0)

        joint_trajectory.points.append(joint_trajectory_point)
        return joint_trajectory


    # Template function for creating the Grasps
    def _create_grasp(self, x, y, z, roll, pitch, yaw):
        grasp = Grasp()

        # pre_grasp
        grasp.pre_grasp_posture = self._open_gripper()
        grasp.pre_grasp_approach.direction.header.frame_id = self.end_effector_link
        grasp.pre_grasp_approach.direction.vector.z = 1.0
        grasp.pre_grasp_approach.direction.vector.y = 0.0
        grasp.pre_grasp_approach.direction.vector.x = 0.0
        grasp.pre_grasp_approach.min_distance = 0.05
        grasp.pre_grasp_approach.desired_distance = 0.1

        # grasp
        grasp.grasp_posture = self._close_gripper()
        grasp.grasp_pose.pose.position.x = x
        grasp.grasp_pose.pose.position.y = y
        grasp.grasp_pose.pose.position.z = z
        q = quaternion_from_euler(roll, pitch, yaw)
        grasp.grasp_pose.pose.orientation.x = q[0]
        grasp.grasp_pose.pose.orientation.y = q[1]
        grasp.grasp_pose.pose.orientation.z = q[2]
        grasp.grasp_pose.pose.orientation.w = q[3]
        grasp.grasp_pose.header.frame_id = "m1n6s200_link_base"

        # post_grasp
        grasp.post_grasp_retreat.direction.header.frame_id = self.end_effector_link
        grasp.post_grasp_retreat.direction.vector.z = -1.0
        grasp.post_grasp_retreat.direction.vector.x = 0.0
        grasp.post_grasp_retreat.direction.vector.y = 0.0
        grasp.post_grasp_retreat.min_distance = 0.05
        grasp.post_grasp_retreat.desired_distance = 0.25

        return [grasp]

    # Template function, you can add parameters if needed!
    def grasp(self, x, y, z, roll, pitch, yaw):

        self.add_object('object', [0.37, -0.24, 0.1, math.pi , 0., 0.], [0.1, 0.1, 0.1])

        grasps = self._create_grasp(x, y, z, roll, pitch, yaw)
        result = self.arm.pick('object', grasps)
        self.remove_object()

        if result == MoveItErrorCodes.SUCCESS:
            print 'Success grasp'
            return True
        else:
            print 'Failed grasp'
            return False

    def open_fingers(self):
        self.gripper.set_joint_value_target([0.0, 0.0])
        self.gripper.go(wait=True)
        rospy.sleep(2.0)

    def close_fingers(self):
        self.gripper.set_joint_value_target([1.3, 1.3])
        self.gripper.go(wait=True)
        rospy.sleep(2.0)

    def move_to(self, x, y, z, roll, pitch, yaw, frame_id ="m1n6s200_link_base"):
        q = quaternion_from_euler(roll, pitch, yaw)
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.arm.set_pose_target(pose, self.end_effector_link)
        plan = self.arm.plan()
        success = True
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

    def remove_object(self, object='object'):
        self.scene.remove_attached_object(self.end_effector_link, object)
        self.scene.remove_world_object(object)
        rospy.loginfo("Object removed")

    def add_object(self, name, pose, size):
        object_pose = PoseStamped()
        object_pose.header.frame_id = "m1n6s200_link_base"
        object_pose.pose.position.x = pose[0]
        object_pose.pose.position.y = pose[1] 
        object_pose.pose.position.z = pose[2]
        q = quaternion_from_euler(*pose[3:])
        object_pose.pose.orientation.x = q[0]
        object_pose.pose.orientation.y = q[1]
        object_pose.pose.orientation.z = q[2]
        object_pose.pose.orientation.w = q[3]
        self.scene.add_box(name, object_pose, size)
        
    def add_table(self):
        self.add_object('table', [0, 0, -0.005, 0, 0, 0], (2, 2, 0.01))



