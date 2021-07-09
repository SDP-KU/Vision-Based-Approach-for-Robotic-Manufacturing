from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox
import numpy as np
import sys, time, math, cv2, keyboard
import matplotlib.pyplot as plt #library for plotting and visualization
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
import cv2.aruco as aruco
from Detect_Holes_Function import HoleDetec

def Move(robot, position, char, ENDEFFECTOR_TO_CAM):
    # path = os.path.dirname(os.path.abspath(__file__)) # get path to save the pictures

    # MoveL
    FROM_HOLE_10CM = transl(0,0,-80)

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))
    #Intrensic Parameters
    color_profile = rs.video_stream_profile(pipeline_profile.get_stream(rs.stream.color))
    color_intrinsics = color_profile.get_intrinsics()
    w, h = color_intrinsics.width, color_intrinsics.height
    #Find RGB 
    found_rgb = False   #Intial Value
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)
    # Start streaming
    pipeline.start(config)

    robot.MoveL(position * FROM_HOLE_10CM)
    # name = "start " + char
    # frames = pipeline.wait_for_frames()
    # color_frame = frames.get_color_frame()
    # frame  = np.asanyarray(color_frame.get_data())
    # cv2.imwrite(str(path+"/"+name+".jpg"), cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            
    # Call Hole Detection Code
    while True and char != "aruco":
        px = 320
        py = 246
        x1,y1 = HoleDetec()
        delta_x = (x1-px)*0.26*0.5
        delta_y = (y1-py)*0.26*0.5
        # input('press enter to correct position')
        x = transl(delta_x, delta_y, 0)
        current_position = robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_CAM
        robot.MoveL(current_position*x)
        x2,y2 = HoleDetec()
        if abs(x2-px) <= 5 and abs(y2-py) <= 5:
            break

    # name = "finish " + char
    # frames = pipeline.wait_for_frames()
    # color_frame = frames.get_color_frame()
    # frame  = np.asanyarray(color_frame.get_data())
    # cv2.imwrite(str(path+"/"+name+".jpg"), cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    # Stop streaming
    pipeline.stop()
