#!/usr/bin/python

from moveit import MoveIt
import rospy
from moveit_grasp.msg import pickAction, pickResult
import actionlib

rospy.init_node('test')
result = pickResult()

moveit = MoveIt()
moveit.open_fingers()
moveit.rtb()

# def execute(self, goal):
#     print('+++++++++++++++++++++++++++++++++++++++++++++++++')
#     rospy.loginfo('Picking request received')
#     self.moveit.close_fingers()
#     x, y, z = goal.request.position
#     z_max = goal.request.z_max
#     rospy.loginfo("Received z_max from behav: {}".format(z_max))
#     rospy.loginfo("Received z from behav: {}".format(z))
#     success = self.moveit.clear_object(x, y, z, z_max, goal.request.yaw, self.object_size[goal.request.object_label])
#     self.moveit.rtb()
#     self.result = pickResult()  # Reset object array
#     if success:
#         self.server.set_succeeded(self.result)
#     else:
#         self.server.set_aborted(self.result)





