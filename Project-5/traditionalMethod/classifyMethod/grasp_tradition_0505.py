#!/usr/bin/env python
#creat by lxb on 2019.05.05
import cv2
import numpy as np
import sys
import time
import copy

sys.path.append('../toolkit')
sys.path.append('../toolkit/handE_controller')
from realsense_real_controller import getImageFromRealsense
from ur_realtime_controller import *
from siftMatch import *
from DetectForeground import *
from rpy2rotationVector import *
from gripper_controller import *


switch_showSegment = True
switch_showMatch = False
switch_showPose = False

robot_ip = "192.168.1.10"
port = 30003

def robotPlay(colorImage,depthImage):
    groundImg = np.load('groundImg2.npy')
    rect = []
    rect = DetectForeground(groundImg,colorImage)

    if(switch_showSegment == True):
        drawing = colorImage.copy()
        for i in range(len(rect)):
            x = rect[i][0]
            y = rect[i][1]
            w = rect[i][2]
            h = rect[i][3]
            cv2.rectangle(drawing,(x,y),(x+w,y+h),(255,255,255),2)
        cv2.imshow('Segment',drawing)
        cv2.waitKey()
        cv2.destroyAllWindows()
    #sift match for categories
    #templates path
    dog_kat = "../template/kat.png"
    dog_at = "../template/at.png"
    dog_aa = "../template/aa.png"
    dog_tt = "../template/tt.png"
    imgKAT = cv2.imread(dog_kat,1)
    imgAT = cv2.imread(dog_at,1)
    imgAA = cv2.imread(dog_aa,1)
    imgTT = cv2.imread(dog_tt,1)
    count = 0
    while count < len(rect):
        ymin = rect[count][1]
        xmin = rect[count][0]
        ymax = ymin + rect[count][3]
        xmax = xmin + rect[count][2]
        if (xmax < 450 or xmin > 1000):
            count = count + 1
            continue
        crop_img = colorImage[ymin-10:ymax+10,xmin-10:xmax+10,:]
        # cv2.imshow('crop_img',crop_img)
        # cv2.waitKey()
        # cv2.destroyAllWindows()
        #match from templates
        matchNum = np.zeros(4)
        angle = np.zeros(4)
        matchNum[0],angle[0] = siftMatch(imgKAT,crop_img,switch_showMatch)
        matchNum[1],angle[1] = siftMatch(imgAT,crop_img,switch_showMatch)
        matchNum[2],angle[2] = siftMatch(imgAA,crop_img,switch_showMatch)
        matchNum[3],angle[3] = siftMatch(imgTT,crop_img,switch_showMatch)

        matchIndex = np.where(matchNum == max(matchNum))
        if(len(matchIndex[0]) != 1 or angle[matchIndex[0][0]] != angle[matchIndex[0][0]]):
            count = count + 1
            continue
        print("class: ",matchIndex[0][0])
        print("angle: ",angle[matchIndex[0][0]])
        count = count + 1

        #center of an object
        pick_point = [(xmin+xmax)/2,(ymin+ymax)/2]
        grasp_point = np.array([[pick_point]], dtype=np.float32)
        gp_base = cv2.perspectiveTransform(grasp_point, image2baseMatrix)
        #pick = [x,y,z,r,p,yaw]
        pick = np.zeros(6)
        pick[0] = gp_base[0][0][0]
        pick[1] = gp_base[0][0][1]
        pick[2] = 160 + 0
        pick[3] = 3.14
        pick[4] = -0.04
        pick[5] = 0


        init_rpy = [3.14,0,-0.026]
        pick_rpy = init_rpy
        pick_rpy[2] = init_rpy[2]-angle[matchIndex[0][0]]

        rotationVector = rpy2rotation(pick_rpy[0],pick_rpy[1],pick_rpy[2])
        robot_controller.movej(pick[0], pick[1], pick[2]+40, rotationVector[0], rotationVector[1], rotationVector[2], 0.5, 0.6)
        targtePosition = np.zeros(6)
        targtePosition[0] = pick[0]
        targtePosition[1] = pick[1]
        targtePosition[2] = pick[2]+40
        targtePosition[3] = rotationVector[0]
        targtePosition[4] = rotationVector[1]
        targtePosition[5] = rotationVector[2]
        print("targetPos: ", targtePosition)
        robot_controller.verifyPostion(targtePosition)

        robot_controller.movej(pick[0], pick[1], pick[2], rotationVector[0], rotationVector[1], rotationVector[2], 0.3, 0.4)
        targtePosition = np.zeros(6)
        targtePosition[0] = pick[0]
        targtePosition[1] = pick[1]
        targtePosition[2] = pick[2]
        targtePosition[3] = rotationVector[0]
        targtePosition[4] = rotationVector[1]
        targtePosition[5] = rotationVector[2]
        robot_controller.verifyPostion(targtePosition)
        gripper_controller.closeGripper()
        time.sleep(1.5)
        robot_controller.movej(pick[0], pick[1], pick[2]+40, rotationVector[0], rotationVector[1], rotationVector[2], 0.5, 0.6)
        targtePosition = np.zeros(6)
        targtePosition[0] = pick[0]
        targtePosition[1] = pick[1]
        targtePosition[2] = pick[2]+40
        targtePosition[3] = rotationVector[0]
        targtePosition[4] = rotationVector[1]
        targtePosition[5] = rotationVector[2]
        robot_controller.verifyPostion(targtePosition)
        # time.sleep(1.5)
        # raw_input('Press enter to continue')

        #place position
        place_position_0 = [568.40, 3.51, 165.08, 3.14, -0.04, -0.0]
        place_position_1 = [568.40, 3.51+55, 165.08, 3.14, -0.04, -0.0]
        place_position_2 = [568.40+55, 3.51, 165.08, 3.14, -0.04, -0.0]
        place_position_3 = [568.40+55, 3.51+55, 165.08, 3.14, -0.04, -0.0]

        place_position_up = [592.27, 35.06, 241.18, 3.14, -0.04, -0.0]
        place_position_mid = [426.94, 329.75, 313.88, 3.14, -0.04, -0.0]
        waypoint_2 = [449.43, 329.2, 264.48, 3.14, -0.241, 0.01]


        jigsaw_type = matchIndex[0][0]
        if jigsaw_type == 1:
            robot_controller.movej(place_position_0[0], place_position_0[1], place_position_0[2]+40, place_position_0[3], place_position_0[4], place_position_0[5], 0.3, 0.45)
            targtePosition = copy.deepcopy(place_position_0)
            targtePosition[2] = place_position_0[2]+40
            robot_controller.verifyPostion(targtePosition)
            robot_controller.movej(place_position_0[0], place_position_0[1], place_position_0[2], place_position_0[3], place_position_0[4], place_position_0[5], 0.3, 0.3)
            robot_controller.verifyPostion(place_position_0)
        elif jigsaw_type == 3:
            robot_controller.movej(place_position_1[0], place_position_1[1], place_position_1[2]+40, place_position_1[3], place_position_1[4], place_position_1[5], 0.3, 0.45)
            targtePosition = copy.deepcopy(place_position_1)
            targtePosition[2] = place_position_1[2]+40
            robot_controller.verifyPostion(targtePosition)
            robot_controller.movej(place_position_1[0], place_position_1[1], place_position_1[2], place_position_1[3], place_position_1[4], place_position_1[5], 0.3, 0.3)
            robot_controller.verifyPostion(place_position_1)
        elif jigsaw_type == 4:
            robot_controller.movej(place_position_2[0], place_position_2[1], place_position_2[2]+40, place_position_2[3], place_position_2[4], place_position_2[5], 0.3, 0.45)
            targtePosition = copy.deepcopy(place_position_2)
            targtePosition[2] = place_position_2[2]+40
            robot_controller.verifyPostion(targtePosition)
            robot_controller.movej(place_position_2[0], place_position_2[1], place_position_2[2], place_position_2[3], place_position_2[4], place_position_2[5], 0.3, 0.3)
            robot_controller.verifyPostion(place_position_2)
        elif jigsaw_type == 2:
            robot_controller.movej(place_position_3[0], place_position_3[1], place_position_3[2]+40, place_position_3[3], place_position_3[4], place_position_3[5], 0.3, 0.45)
            targtePosition = copy.deepcopy(place_position_3)
            targtePosition[2] = place_position_3[2]+40
            robot_controller.verifyPostion(targtePosition)
            robot_controller.movej(place_position_3[0], place_position_3[1], place_position_3[2], place_position_3[3], place_position_3[4], place_position_3[5], 0.3, 0.3)
            robot_controller.verifyPostion(place_position_3)

        # time.sleep(1.5)
        gripper_controller.openGripper()
        time.sleep(1.3)
        robot_controller.movej(place_position_up[0], place_position_up[1], place_position_up[2], place_position_up[3], place_position_up[4], place_position_up[5], 0.5, 0.5)
        robot_controller.verifyPostion(place_position_up)
        # time.sleep(2)
        count = count + 1
    robot_controller.movej(home_tcp_position[0], home_tcp_position[1], home_tcp_position[2], home_tcp_position[3], home_tcp_position[4], home_tcp_position[5], 0.4, 0.6)
    robot_controller.verifyPostion(home_tcp_position)


if __name__ == '__main__':
    #calibration
    xy1 = [439.76,729.71] #from tp
    xy2 = [458.04,497.03]
    xy3 = [236.57, 743.48]
    xy4 = [260.350,500.420]

    uv1 = [896,434] #from image process
    uv2 = [678,456]
    uv3 = [904,244]
    uv4 = [676,274]
    #perspective transformation
    pts1 = np.float32([uv1,uv2,uv3,uv4])
    pts2 = np.float32([xy1,xy2,xy3,xy4])
    image2baseMatrix = cv2.getPerspectiveTransform(pts1,pts2)
    # controller initiation
    robot_controller = URController(robot_ip,port)
    gripper_controller = HandEController(robot_ip,port)
    #go to home_tcp_position
    home_tcp_position = [-52.09, 497.91, 272.88, 2.7, -1.621, -0.035]
    robot_controller.movej(home_tcp_position[0], home_tcp_position[1], home_tcp_position[2], home_tcp_position[3], home_tcp_position[4], home_tcp_position[5], 0.4, 0.6)
    time.sleep(0.5)
    robot_controller.verifyPostion(home_tcp_position)
    gripper_controller.activeGripper()
    while(True):
        colorImage,depthImage,_,_ = getImageFromRealsense()
        robotPlay(colorImage,depthImage)
