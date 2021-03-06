#!/usr/bin/env python

# Import modules
import numpy as np
from visualization_msgs.msg import Marker
import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from franka_description.srv import *

# function to load parameters and request PickPlace service
def franka_mover():

    # TODO: Get parameter "/grasp_list" from ros param server and save it to variable grasp_list, grasp_list is a python dictionary data type
    grasp_list = rospy.get_param('/grasp_list')[0]

    # TODO: Loop through all the picking items, the name and pose of the picking items is stored in ros parameter "/grasp_list"
    for obj in grasp_list:
        object_name = String()
        object_name.data = obj

        pick_pose = Pose()
        pick_pose.position.x = grasp_list[obj]['position']['x']
        pick_pose.position.y = grasp_list[obj]['position']['y']
        pick_pose.position.z = grasp_list[obj]['position']['z'] + 0.13  #+ offset for gripper from contact point
        pick_pose.orientation.x = grasp_list[obj]['orientation']['x']
        pick_pose.orientation.y = grasp_list[obj]['orientation']['y']
        pick_pose.orientation.z = grasp_list[obj]['orientation']['z']
        pick_pose.orientation.w = grasp_list[obj]['orientation']['w']
        print ("Pick pose: ",pick_pose)
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
            resp = pick_place_routine(object_name, pick_pose)
            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('franka_picking', anonymous=True)
    franka_mover()

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
