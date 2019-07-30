import rospy
from arm_controller import ArmController
import argparse
import numpy as np

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('pose', type=float, nargs=6)
    args = parser.parse_args()
    pose = args.pose
    pose[3:] = np.radians(pose[3:])

    rospy.init_node('move_kinova_to')
    controller = ArmController()
    # moveit.move_to(0.37, -0.24, 0.1, math.pi , 0., 0.)
    controller.move_to(*pose)
