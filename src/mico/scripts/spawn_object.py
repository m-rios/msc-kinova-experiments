#!/usr/bin/env python
import rospy 
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
import tf
import rospkg
from math import pi
from random import shuffle, uniform


def SpawnObject(object_name, x, y, z, rotation):
    path = rospack.get_path("mico")
    with open(path +  "/sdf/" + str(object_name) + ".sdf") as f:
        box_xml = f.read()
        
    orientation = Quaternion()
    quaternion = tf.transformations.quaternion_from_euler(0, 0, rotation)
    orientation.x = quaternion[0]
    orientation.y = quaternion[1]
    orientation.z = quaternion[2]
    orientation.w = quaternion[3]
    
    pose = Pose(Point(x, y, z), orientation)
    
    spawn_model(object_name, box_xml, "", pose, "world")            
        
if __name__ == "__main__":
    rospy.init_node("set_objects")
    rospack = rospkg.RosPack()
    print 'Waiting for service...'
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    print 'Connected to service'
    
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    
    
    pos = [0.3, 0, 0.1]
    objects = ["box0", "box1", "box2", "box3", "box4"]
    
    for object in objects:
        delete_model(object)
    
    shuffle(objects)
    
    rotation = 0; # radians
    SpawnObject(objects[0], pos[0], pos[1], pos[2], rotation)
    
    print 'Done'

# randomly set all objects on the table..
# there are 5 objects, 2 on table 1, 2 on 1 side, 1 on the other side

# need locations of table...

# table1 on middle_pos 
#table2 on middle_pos (0.48, 3.85)

