import rospy
from arm_controller import ArmController
import numpy as np

if __name__ == '__main__':
    rospy.init_node('width test')
    controller = ArmController()
    controller.set_fingers_width(0.11)
    # controller.close_fingers()