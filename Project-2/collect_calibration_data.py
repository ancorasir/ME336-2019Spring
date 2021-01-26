#!/usr/bin/env python
import numpy as np
import rospy, tf, sys, time
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
import cv2
import pyrealsense2 as rs

# camera matrix of realsense415 with half resolution
camera_info = CameraInfo()
camera_info.K = [931.6937866210938, 0.0, 624.7894897460938, 0.0, 931.462890625, 360.5186767578125, 0.0, 0.0, 1.0]
camera_info.header.frame_id = 'camera_color_optical_frame'
camera_info.height = 720
camera_info.width = 1280

# TODO: Construct 3D calibration grid across workspace
# User options (change me)
# --------------- Setup options ---------------
workspace_limits = np.asarray([[0.3-0.08, 0.3+0.08], [-0.524-0.1,-0.524+0.1], [0.5-0.02, 0.5+0.02]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
calib_grid_step = 0.04
checkerboard_size = (3,3)
refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# ---------------------------------------------
gridspace_x = np.linspace(workspace_limits[0][0], workspace_limits[0][1], 1 + (workspace_limits[0][1] - workspace_limits[0][0])/calib_grid_step)
gridspace_y = np.linspace(workspace_limits[1][0], workspace_limits[1][1], 1 + (workspace_limits[1][1] - workspace_limits[1][0])/calib_grid_step)
gridspace_z = np.linspace(workspace_limits[2][0], workspace_limits[2][1], 1 + (workspace_limits[2][1] - workspace_limits[2][0])/calib_grid_step)
calib_grid_x, calib_grid_y, calib_grid_z = np.meshgrid(gridspace_x, gridspace_y, gridspace_z)
num_calib_grid_pts = calib_grid_x.shape[0]*calib_grid_x.shape[1]*calib_grid_x.shape[2]
calib_grid_x.shape = (num_calib_grid_pts,1)
calib_grid_y.shape = (num_calib_grid_pts,1)
calib_grid_z.shape = (num_calib_grid_pts,1)
calib_grid_pts = np.concatenate((calib_grid_x, calib_grid_y, calib_grid_z), axis=1)

# realsense
points = rs.points()
pipeline= rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)
align_to = rs.stream.color
align = rs.align(align_to)

# TODO: calculate the position (x, y, z) of the checkerboard center
def image_callback(color_image, depth_image, iter):
    gray = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
    checkerboard_found, corners = cv2.findChessboardCorners(gray, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
    if checkerboard_found:
        corners_refined = cv2.cornerSubPix(gray, corners, (3,3), (-1,-1), refine_criteria)

        # Get observed checkerboard center 3D point in camera space
        checkerboard_pix = np.round(corners_refined[4,0,:]).astype(int)
        checkerboard_z = np.mean(np.mean(depth_image[checkerboard_pix[1]-20:checkerboard_pix[1]+20,checkerboard_pix[0]-20:checkerboard_pix[0]+20])) * depth_scale
        print("Found checkerboard, Z = ", checkerboard_z)
        checkerboard_x = np.multiply(checkerboard_pix[0]-624.79,checkerboard_z/931.69)
        checkerboard_y = np.multiply(checkerboard_pix[1]-360.52,checkerboard_z/931.46)
        if checkerboard_z > 0:
            # Save calibration point and observed checkerboard center
            observed_pt = np.array([checkerboard_x,checkerboard_y,checkerboard_z])

        # Draw and display the corners
        vis = cv2.drawChessboardCorners(color_image, (1,1), corners_refined[4,:,:], checkerboard_found)
        cv2.imwrite('images/color_image_checkerboard_%02d.png' % iter, vis)
        return observed_pt

# ROS node initialization
rospy.init_node('perception', anonymous=True)

# Create move group of MoveIt for motion planning
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")
group.set_planner_id('RRTConnectkConfigDefault')
group.set_num_planning_attempts(10)
group.set_planning_time(5)
group.set_max_velocity_scaling_factor(0.5)

# Go through all the grid points and calculate the position of the checkerboard
tool = PoseStamped()
tool.header.frame_id = "world"
tool.pose.orientation.x = 0
tool.pose.orientation.y = 0
tool.pose.orientation.z = 0
tool.pose.orientation.w = 1

measured_pts = []
observed_pts = []
for iter in range(num_calib_grid_pts):
    tool.pose.position.x = calib_grid_pts[iter,0]
    tool.pose.position.y = calib_grid_pts[iter,1]
    tool.pose.position.z = calib_grid_pts[iter,2]
    (plan, fracktion) = group.compute_cartesian_path([group.get_current_pose().pose, tool.pose], eef_step=0.01, jump_threshold=0.0, avoid_collisions=True)
    print("******************************************************")
    print("tool pose: ",tool)
    raw_input("Please enter:")
    group.execute(plan)
    time.sleep(1)

    # take color and depth images
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    cv2.imwrite('images/color_image_%02d.png' % iter, color_image)
    observed_pt = image_callback(color_image, depth_image, iter)
    observed_pts.append(observed_pt)
    measured_pts.append(calib_grid_pts[iter,:])

np.savez("calibration_data.npz",observed_pts,measured_pts)
pipeline.stop()
print("Collecting data Finished and data saved!")
