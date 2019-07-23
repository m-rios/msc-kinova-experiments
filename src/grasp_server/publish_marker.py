#! /usr/bin/python
import rospy
from visualization_msgs.msg import Marker

if __name__ == '__main__':
    rospy.init_node('publish_marker')
    pub = rospy.Publisher('marker', Marker, latch=True, queue_size=1)
    rospy.sleep(1)

    marker = Marker()

    marker.header.frame_id = "m1n6s200_link_base"
    marker.header.stamp = rospy.Time.now()
    marker.ns = 'debug'
    marker.id = 1
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.pose.orientation.w = 1
    marker.pose.position.x = 0.37
    marker.pose.position.y = -0.23
    marker.pose.position.z = 0.0475
    marker.scale.x = 0.06
    marker.scale.y = 0.048
    marker.scale.z = 0.155
    marker.color.a = 1.0
    marker.color.b = 1.0

    while not rospy.is_shutdown():
        pub.publish(marker)
        rospy.sleep(1)

