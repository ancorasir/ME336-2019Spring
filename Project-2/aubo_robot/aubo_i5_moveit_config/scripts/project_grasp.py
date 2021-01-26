#!/usr/bin/env python

# Import modules
import numpy as np
import rospy, tf, sys, time
from geometry_msgs.msg import Pose, PoseStamped
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import tf2_ros
import tf2_geometry_msgs
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
from io_interface import *
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as IM
import cv2

# camera matrix of realsense
camera_info = CameraInfo()
camera_info.K = [616.3787841796875, 0.0, 434.0303955078125, 0.0, 616.4257202148438, 234.33065795898438, 0.0, 0.0, 1.0]
camera_info.header.frame_id = 'camera_color_optical_frame'
camera_info.height = 480
camera_info.width = 848

def image_callback(color, depth):
    # TODO: transform sensor_msgs/Image to numpy array and save it jpg files
    try:
        color_image = cv_bridge.imgmsg_to_cv2(color, "rgb8")
        depth_image = cv_bridge.imgmsg_to_cv2(depth, desired_encoding = "32FC1")
    except CvBridgeError as cv_bridge_exception:
        rospy.logerr(cv_bridge_exception)
    im = IM.fromarray(color_image)
    im.save("/home/bionicdl/catkin_ws/color_image.jpg")

    # TODO: detect circles and other features in the color image
    color_image = cv2.medianBlur(color_image,5)
    gray = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1.2,20,
                                param1=50,param2=30,minRadius=5,maxRadius=15)
    circles = np.uint16(np.around(circles)) #shape (1, n, 3)
    u,v,r = circles[0,1]
    # draw the outer circle
    cv2.circle(color_image,(u,v),r,(0,255,0),2)
    # draw the center of the circle
    cv2.circle(color_image,(u,v),2,(0,0,255),3)
    cv2.imshow('detected circles',color_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # TODO: find the x, y, z of the pick point in the camera coordinate
    z = depth_image[v,u]/1000
    if z<70:
        pass
    x = (u-434)/616.379*z
    y = (v-234.3)/616.426*z
    grasp = PoseStamped()
    grasp.pose.position.x = x
    grasp.pose.position.y = y
    grasp.pose.position.z = z
    print ("pick pose: ", grasp)

    # TODO: transform the picking pose in the robot base coordinate
    tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    listener = tf2_ros.TransformListener(tfBuffer)
    try:
        trans = tfBuffer.lookup_transform('world', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print "Find transform failed"
    pose_transformed = tf2_geometry_msgs.do_transform_pose(grasp, trans)
    pose_transformed.pose.position.z += 0.13 # offset for tool0 to the suction cup
    pose_transformed.pose.orientation.x = 1
    pose_transformed.pose.orientation.y = 0.0
    pose_transformed.pose.orientation.z = 0.0
    pose_transformed.pose.orientation.w = 0.0

    print "*************************************************"
    print ("pick pose transformed: ", pose_transformed)

    # move the robot to the detected point
    try:
        robot_mover(pose_transformed)
    except rospy.ROSInterruptException:
        pass

def robot_mover(pose_transformed):
    global group
    pose_transformed.pose.position.z = pose_transformed.pose.position.z + 0.05
    group.set_pose_target(pose_transformed, end_effector_link='wrist3_Link') #wrist3_Link
    plan = group.plan()
    raw_input('Press enter to continue: ')
    group.execute(plan)

    pose_transformed.pose.position.z = pose_transformed.pose.position.z - 0.060
    group.set_pose_target(pose_transformed, end_effector_link='wrist3_Link')
    plan = group.plan()
    group.execute(plan)
    time.sleep(1)

    pose_transformed.pose.position.z = pose_transformed.pose.position.z + 0.1
    group.set_pose_target(pose_transformed, end_effector_link='ee_link')
    plan = group.plan()
    group.execute(plan)
    time.sleep(1)

    group.set_joint_value_target(home_joint_positions)
    plan = group.plan()
    group.execute(plan)
    time.sleep(10)

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('perception', anonymous=True)

    # initialize cv_bridge
    cv_bridge = CvBridge()

    # Create move group of MoveIt for motion planning
    moveit_commander.roscpp_initialize(sys.argv)
    global group, robot, scene
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")
    group.set_planner_id('RRTConnectkConfigDefault')
    group.set_num_planning_attempts(5)
    group.set_planning_time(5)
    group.set_max_velocity_scaling_factor(0.5)

    # Go to the home pose waiting for picking instruction
    group.set_joint_value_target(home_joint_positions)
    plan = group.plan()
    raw_input('Press enter to continue: ')
    group.execute(plan)
    time.sleep(5)

    # TODO: Create Subscribers to rectified color image and depth image
    color_sub = message_filters.Subscriber("/camera/color/image_rect_color", Image)
    depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)

    ts = message_filters.TimeSynchronizer([color_sub, depth_sub], 1)
    ts.registerCallback(image_callback)

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
