#!/usr/bin/python

import rospy
import actionlib
from grasp_server.msg import pickAction, pickResult
from moveit import MoveIt


class GraspActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('object_pick', pickAction, self.execute, False)
        self.result = pickResult()
        self.moveit = MoveIt()
        self.object_size = {
            "evergreen": [0.215, 0.05, 0.095],
            "tomatosoup": [0.1, 0.03, 0.13],
            "basetech":  [0.16, 0.045, 0.17],
            "eraserbox": [0.155, 0.05, 0.055],
            "usbhub": [0.13, 0.07, 0.075]
        }
        self.server.start()

    def execute(self, goal):
        print('+++++++++++++++++++++++++++++++++++++++++++++++++')
        rospy.loginfo('Picking request received')
        self.moveit.close_fingers()
        x, y, z = goal.request.position
        z_max = goal.request.z_max
        rospy.loginfo("Received z_max from behav: {}".format(z_max))
        rospy.loginfo("Received z from behav: {}".format(z))
        success = self.moveit.clear_object(x, y, z, z_max, goal.request.yaw, self.object_size[goal.request.object_label])
        self.result = pickResult()  # Reset object array
        if success:
            self.server.set_succeeded(self.result)
        else:
            self.moveit.rtb()
            self.server.set_aborted(self.result)


if __name__ == '__main__':
  rospy.init_node('grasp_action_server')
  server = GraspActionServer()
  rospy.spin()
