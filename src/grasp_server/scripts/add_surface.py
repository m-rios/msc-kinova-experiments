import sys
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node('add_surface')
    moveit_commander.roscpp_initialize(sys.argv)
    scene = PlanningSceneInterface()
    object_pose = PoseStamped()
    object_pose.header.frame_id = "m1n6s200_link_base"
    object_pose.pose.position.x = 0.
    object_pose.pose.position.y = 0.
    object_pose.pose.position.z = -0.005
    q = quaternion_from_euler(0., 0.0, 0.)
    object_pose.pose.orientation.x = q[0]
    object_pose.pose.orientation.y = q[1]
    object_pose.pose.orientation.z = q[2]
    object_pose.pose.orientation.w = q[3]
    scene.add_box("table", object_pose, (2, 2, 0.01))
    rospy.sleep(1)
    print 'Done'
