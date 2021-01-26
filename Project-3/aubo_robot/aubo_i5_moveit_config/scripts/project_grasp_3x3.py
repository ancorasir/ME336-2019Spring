#!/usr/bin/env python

# Import modules
import numpy as np
import rospy, tf, sys, time
from geometry_msgs.msg import Pose, PoseStamped
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
import tf2_ros
import tf2_geometry_msgs
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
from io_interface import *
from PIL import Image as IM
import cv2
import pyrealsense2 as rs
from game import *


# camera matrix of realsense
camera_info = CameraInfo()
# camera_info.K = [616.3787841796875, 0.0, 434.0303955078125, 0.0, 616.4257202148438, 234.33065795898438, 0.0, 0.0, 1.0]
camera_info.K = [931.6937866210938, 0.0, 624.7894897460938, 0.0, 931.462890625, 360.5186767578125, 0.0, 0.0, 1.0]
camera_info.header.frame_id = 'camera_color_optical_frame'
camera_info.height = 720
camera_info.width = 1280

def robot_player(color_image, depth_image, ai_move):
    # TODO: transform sensor_msgs/Image to numpy array and save it jpg files
    # try:
    #     color_image = cv_bridge.imgmsg_to_cv2(color, "rgb8")
    #     depth_image = cv_bridge.imgmsg_to_cv2(depth, desired_encoding = "32FC1")
    # except CvBridgeError as cv_bridge_exception:
    #     rospy.logerr(cv_bridge_exception)
    # im = IM.fromarray(color_image)
    # im.save("/home/bionicdl/catkin_ws/color_image.jpg")
    # TODO: detect circles and other features in the color image
    # color_image = cv2.medianBlur(color_image,5)
    gray = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1.2,20,param1=50,param2=30,minRadius=5,maxRadius=15)
    circles = np.uint16(np.around(circles)) #shape (1, n, 3)
    found = False
    for i in range(circles.shape[1]):
        u,v,r = circles[0,i]
        if v>410:
            found = True
            break
    if not found:
        print("Fail to found any piece to pick! Please ensure there is piece for the robot!")
        return False
    # draw the outer circle
    cv2.circle(color_image,(u,v),r,(0,255,0),2)
    # draw the center of the circle
    # cv2.circle(color_image,(u,v),2,(0,0,255),3)
    # cv2.imshow('detected circles',color_image)
    # cv2.waitKey(2000)
    # cv2.destroyAllWindows()
    # find the x, y, z of the pick point in the camera coordinate
    z = 0.9927
    x = (u-624.79)/931.694*z
    y = (v-360.52)/931.463*z
    grasp = PoseStamped()
    grasp.pose.position.x = x
    grasp.pose.position.y = y
    grasp.pose.position.z = z
    pick = tf2_geometry_msgs.do_transform_pose(grasp, trans)
    pick.pose.position.z += 0.13 # offset for tool0 to the suction cup 0.13
    pick.pose.orientation.x = 1
    pick.pose.orientation.y = 0.0
    pick.pose.orientation.z = 0.0
    pick.pose.orientation.w = 0.0
    set_states()
    pick.pose.position.z = pick.pose.position.z + 0.05
    group.set_pose_target(pick, end_effector_link='wrist3_Link') #wrist3_Link
    plan = group.plan()
    # raw_input('Press enter to go pick position: ')
    group.execute(plan)
    pick.pose.position.z = pick.pose.position.z - 0.060
    group.set_pose_target(pick, end_effector_link='wrist3_Link')
    plan = group.plan()
    group.execute(plan)
    time.sleep(1)
    set_digital_out(0, False)
    time.sleep(1)
    pick.pose.position.z = pick.pose.position.z + 0.1
    group.set_pose_target(pick, end_effector_link='wrist3_Link')
    plan = group.plan()
    group.execute(plan)
    time.sleep(1)
    place = PoseStamped()
    place.pose.position.x = checkerboard_x[ai_move]
    place.pose.position.y = checkerboard_y[ai_move]
    place.pose.position.z = checkerboard_z
    place_robot = tf2_geometry_msgs.do_transform_pose(place, trans)
    place_robot.pose.position.z += 0.05 + 0.13
    place_robot.pose.orientation.x = 1
    place_robot.pose.orientation.y = 0.0
    place_robot.pose.orientation.z = 0.0
    place_robot.pose.orientation.w = 0.0
    group.set_pose_target(place_robot)
    plan = group.plan()
    # raw_input('Press enter to go place position: ')
    group.execute(plan)
    time.sleep(1)
    place_robot.pose.position.z = place_robot.pose.position.z - 0.04
    group.set_pose_target(place_robot, end_effector_link='wrist3_Link')
    plan = group.plan()
    group.execute(plan)
    time.sleep(1)
    set_digital_out(0, True)
    time.sleep(1)
    place_robot.pose.position.z = place_robot.pose.position.z + 0.04
    group.set_pose_target(place_robot, end_effector_link='wrist3_Link')
    plan = group.plan()
    group.execute(plan)
    time.sleep(1)
    group.set_joint_value_target(home_joint_position)
    plan = group.plan()
    # raw_input('Press enter to go home position: ')
    group.execute(plan)


# TODO: ROS node initialization
rospy.init_node('perception', anonymous=True)

# TODO: Create move group of MoveIt for motion planning
moveit_commander.roscpp_initialize(sys.argv)
global group, robot, scene
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")
group.set_planner_id('RRTConnectkConfigDefault')
group.set_num_planning_attempts(5)
group.set_planning_time(5)
group.set_max_velocity_scaling_factor(0.5)

# TODO: transform the picking pose in the robot base coordinate
tfBuffer = tf2_ros.Buffer(rospy.Duration(1200.0))
listener = tf2_ros.TransformListener(tfBuffer)
try:
    trans = tfBuffer.lookup_transform('world', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(1.0))
except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print "Find transform failed"

# TODO: Go to the home pose waiting for picking instruction
home_joint_position = [-80.25/180*3.14, 2.0/180*3.14, 125.8/180*3.14, 34.1/180*3.14, 96.1/180*3.14, 12.7/180*3.14]
group.set_joint_value_target(home_joint_position)
plan = group.plan()
# raw_input('Press enter to go the home position: ')
group.execute(plan)
time.sleep(1)

# initiate realsense
points = rs.points()
pipeline= rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
align_to = rs.stream.color
align = rs.align(align_to)

time.sleep(3)
frames = pipeline.wait_for_frames()
aligned_frames = align.process(frames)
aligned_depth_frame = aligned_frames.get_depth_frame()
color_frame = aligned_frames.get_color_frame()
depth_image_background = np.asanyarray(aligned_depth_frame.get_data())
color_image_background = np.asanyarray(color_frame.get_data())

gray = cv2.cvtColor(color_image_background, cv2.COLOR_BGR2GRAY)
template = cv2.imread('chessboard.png',0)
w, h = template.shape[::-1]
res = cv2.matchTemplate(gray,template,4)
min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
top_left = max_loc
bottom_right = (top_left[0] + w, top_left[1] + h)
cv2.rectangle(color_image_background,top_left, bottom_right, [0,0,255], 1)
cv2.imshow("Detected chess board!", color_image_background)
cv2.waitKey(2000)
cv2.destroyAllWindows()

cu,cv=top_left
checkerboard_z = 0.9907
checkerboard_pix = np.array([[cu+16,cv+16],[cu+16+31,cv+16],[cu+16+62,cv+16],
                             [cu+16,cv+16+31],[cu+16+31,cv+16+31],[cu+16+62,cv+16+31],
                             [cu+16,cv+16+62],[cu+16+31,cv+16+62],[cu+16+62,cv+16+62]])
checkerboard_x = np.multiply(checkerboard_pix[:,0]-624.79,checkerboard_z/931.69)
checkerboard_y = np.multiply(checkerboard_pix[:,1]-360.52,checkerboard_z/931.46)

# Start game
game = TicTacToe()

def detect_person_move(color_image, game):
    for i in range(9):
        u,v = checkerboard_pix[i]
        if sum(np.abs(np.mean(color_image_background[v-10:v+10,u-10:u+10].astype(float).reshape(400,3),axis=0) - np.mean(color_image[v-10:v+10,u-10:u+10].astype(float).reshape(20*20,3),axis=0))>60)>0:
            if game.board[i]==" ":
                person_move = i
                print("Detected person_move is at : %s"%person_move)
                return True, person_move
    print("Fail to detect the person move!")
    return False, False

while game.gameOver() == False:
    # raw_input('Press enter to when you are ready to place next one: ')
    time.sleep(5)
    detected = False
    while not detected:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        detected, person_move = detect_person_move(color_image, game)
        if not detected:
            print("Please place you piece X again!")
            time.sleep(6)

    game.makeMove(person_move, "X")
    game.show()

    if game.gameOver() == True:
        break

    print("Computer choosing move...")
    ai_move = make_best_move(game, -1, "O")
    game.makeMove(ai_move, "O")
    game.show()
    result = robot_player(color_image, depth_image, ai_move)

print("Game Over. " + game.whoWon() + " Wins")
