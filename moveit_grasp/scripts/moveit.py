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
        self.clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)
        
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
    def _create_grasps(self, x, y, z, z_max, rotation):
        grasps = []
        
        # You can create multiple grasps and add them to the grasps list
        grasp = Grasp()  # create a new grasp
        
        # Set the pre grasp posture (the fingers)
        grasp.pre_grasp_posture = self._open_gripper()
        # Set the grasp posture (the fingers)
        grasp.grasp_posture = self._close_gripper()
        # Set the position of where to grasp
        grasp.grasp_pose.pose.position.x = x
        grasp.grasp_pose.pose.position.y = y
        grasp.grasp_pose.pose.position.z = z
        # Set the orientation of the end effector
        q = quaternion_from_euler(math.pi, 0.0, rotation)
        grasp.grasp_pose.pose.orientation.x = q[0]
        grasp.grasp_pose.pose.orientation.y = q[1]
        grasp.grasp_pose.pose.orientation.z = q[2]
        grasp.grasp_pose.pose.orientation.w = q[3]
        grasp.grasp_pose.header.frame_id = "m1n6s200_link_base"
        # Set the pre_grasp_approach
        grasp.pre_grasp_approach.direction.header.frame_id = self.end_effector_link
        grasp.pre_grasp_approach.direction.vector.z = 1.0
        grasp.pre_grasp_approach.direction.vector.y = 0.0
        grasp.pre_grasp_approach.direction.vector.x = 0.0
        grasp.pre_grasp_approach.min_distance = 0.05
        grasp.pre_grasp_approach.desired_distance = 0.1
        # # Set the post_grasp_approach
        grasp.post_grasp_retreat.direction.header.frame_id = self.end_effector_link
        grasp.post_grasp_retreat.direction.vector.z = -1.0
        grasp.post_grasp_retreat.direction.vector.x = 0.0
        grasp.post_grasp_retreat.direction.vector.y = 0.0
        grasp.post_grasp_retreat.min_distance = 0.05
        grasp.post_grasp_retreat.desired_distance = 0.25
        
        grasp.grasp_pose.header.frame_id = "m1n6s200_link_base" # setting the planning frame (Positive x is to the left, negative Y is to the front of the arm)
        
        grasps.append(grasp) # add all your grasps in the grasps list, MoveIT will pick the best one

        for z_offset in np.arange(z+0.02, z_max, 0.01):
            new_grasp = copy.deepcopy(grasp)
            new_grasp.grasp_pose.pose.position.z = z_offset
            grasps.append(new_grasp)
        return grasps
    
    # Template function, you can add parameters if needed!
    def grasp(self, x, y, z, z_max, rotation, size):
        print '******************* grasp'
        # Object distance:
        obj_dist = np.linalg.norm(np.asarray((x, y, z)))
        if obj_dist > 0.5:
            rospy.loginfo("Object too far appart ({} m), skipping pick".format(obj_dist))
            return False

        # Add collision object, easiest to name the object, "object"
        object_pose = PoseStamped()
        object_pose.header.frame_id = "m1n6s200_link_base"
        object_pose.pose.position.x = x
        object_pose.pose.position.y = y
        object_pose.pose.position.z = z
        q = quaternion_from_euler(math.pi, 0.0, rotation)
        object_pose.pose.orientation.x = q[0]
        object_pose.pose.orientation.y = q[1]
        object_pose.pose.orientation.z = q[2]
        object_pose.pose.orientation.w = q[3]

        self.scene.add_box("object", object_pose, size)

        rospy.sleep(0.5)
        self.clear_octomap()
        rospy.sleep(1.0)

        # Create and return grasps
        # z += size[2]/2  # Focus on the top of the object only
        # z += size[2]/2 + 0.02  # Focus on the top of the object only

        grasps = self._create_grasps(x, y, z, z_max, rotation)
        print '******************************************************************************'
        result = self.arm.pick('object', grasps)  # Perform pick on "object", returns result
        print '******************************************************************************'
        # self.move_to(x, y, z + 0.15, rotation)

        if result == MoveItErrorCodes.SUCCESS:
            print 'Success grasp'
            return True
        else:
            print 'Failed grasp'
            return False

    def clear_object(self, x, y, z, z_max, rotation, size):
        print '******************* clear_object'

        self.move_to_waypoint()
        success = self.grasp(x, y, z, z_max, rotation, size)

        if success:
            self.move_to_waypoint()
            success = self.move_to_drop_zone()
            if success:
                print 'success move to drop zone'
            else:
                print 'failed move to drop zone'


        self.open_fingers()
        self.remove_object()
        rospy.sleep(1.0)
        self.close_fingers()

        return success

    def open_fingers(self):
        print '******************* open_fingers'
        self.gripper.set_joint_value_target([0.0, 0.0])
        self.gripper.go(wait=True)
        rospy.sleep(2.0)
        
    def close_fingers(self):
        print '******************* close_fingers'
        self.gripper.set_joint_value_target([1.3, 1.3])
        self.gripper.go(wait=True)
        rospy.sleep(2.0)
    
    def move_to(self, x, y, z, rotation, frame_id ="m1n6s200_link_base"):
        print '******************* move_to'
        q = quaternion_from_euler(math.pi, 0.0, rotation)
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
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success


    def move_to_waypoint(self):
        print '******************* move_to_waypoint'
        return self.move_to(0.35, 0, 0.25, 1.57)

    def rtb(self):
        print '******************* rtb'
        self.move_to_drop_zone()
        # pose = PoseStamped()
        # pose.header.frame_id = 'base_footprint'
        # pose.pose.position.x = -0.191258927921
        # pose.pose.position.y = 0.1849306168113
        # pose.pose.position.z = 0.813729734732
        # pose.pose.orientation.x = -0.934842026356
        # pose.pose.orientation.y = 0.350652799078
        # pose.pose.orientation.z = -0.00168532388516
        # pose.pose.orientation.w = 0.0557688079539
        #
        # self.arm.set_pose_target(pose, self.end_effector_link)
        # plan = self.arm.plan()
        # self.arm.go(wait=True)
        # self.arm.stop()
        # self.arm.clear_pose_targets()

    def move_to_drop_zone(self):
        print '******************* move_to_drop_zone'
        pose = PoseStamped()
        pose.header.frame_id = "m1n6s200_link_base"
        pose.pose.position.x = 0.2175546259709541
        pose.pose.position.y = 0.18347985269448372
        pose.pose.position.z = 0.16757751444136426

        pose.pose.orientation.x = 0.6934210704552356
        pose.pose.orientation.y =  0.6589390059796749
        pose.pose.orientation.z = -0.23223137602833943
        pose.pose.orientation.w = -0.17616808290725341
        
        self.arm.set_pose_target(pose, self.end_effector_link)
        plan = self.arm.plan()
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

    def print_position(self):
        pose =  self.arm.get_current_pose()
        self.transformer.waitForTransform("m1n6s200_link_base", "base_footprint", rospy.Time.now(), rospy.Duration(10))
        eef_pose = self.transformer.transformPose("m1n6s200_link_base", pose)
        
        orientation = eef_pose.pose.orientation
        orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = euler_from_quaternion(orientation)

        print "z:", eef_pose.pose.position.x
        print "y:", eef_pose.pose.position.y
        print "z:", eef_pose.pose.position.z
        print "yaw (degrees):", math.degrees(euler[2])
    
    def remove_object(self):
        self.scene.remove_attached_object(self.end_effector_link, "object")
        self.scene.remove_world_object("object")
        rospy.loginfo("Object removed")

         
        
