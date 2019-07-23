import rospy
from moveit import MoveIt
from position_action_client import move_to_position
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math
import argparse
import numpy as np

if __name__ == '__main__':
    rospy.init_node('kinova_tests')
    moveit = MoveIt()
    if moveit.grasp(0.37, -0.24, 0.1, math.pi , 0., 0.):
        print 'Success!'
    else:
        print 'Failure :('
    moveit.open_fingers()
