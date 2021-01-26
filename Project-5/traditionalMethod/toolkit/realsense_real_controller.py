#!/usr/bin/env python
import numpy as np
import pyrealsense2 as rs
import time

# realsense
points = rs.points()
pipeline= rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 30)
config.enable_stream(rs.stream.infrared, 2, 1280, 720, rs.format.y8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

profile = pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)


def getImageFromRealsense():
    frames = pipeline.wait_for_frames()
    irL_frame = frames.get_infrared_frame(1)
    irR_frame = frames.get_infrared_frame(2)
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    if(not aligned_depth_frame or not color_frame):
        return
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    image_L = np.asanyarray(irL_frame.get_data())
    image_R = np.asanyarray(irR_frame.get_data())
    return color_image,depth_image,image_L,image_R
