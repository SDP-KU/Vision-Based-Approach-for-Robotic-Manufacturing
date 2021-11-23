import math3d as m3d
from IPython import embed
import pygame, sys, time, urx, cv2, keyboard, os, math
import numpy as np
import pyrealsense2 as rs

pipeline, config = rs.pipeline(), rs.config()
pipeline_profile = config.resolve(rs.pipeline_wrapper(pipeline))
device = pipeline_profile.get_device()
color_profile = rs.video_stream_profile(pipeline_profile.get_stream(rs.stream.color))
color_intrinsics = color_profile.get_intrinsics()
px, py = color_intrinsics.ppx, color_intrinsics.ppy ## use these values instead of px & py
found_rgb = False   #Intial Value
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

lower_range, upper_range = np.array([0, 200, 0]), np.array([179, 255, 255]) # HSV Mask Parameters

# Hole Detection Parameters:
detector, params = cv2.SimpleBlobDetector_create(), cv2.SimpleBlobDetector_Params()
params.filterByArea, params.filterByCircularity, params.filterByInertia = True, True, True
params.minArea, params.maxArea = 150, 6000
params.minCircularity, params.minInertiaRatio = 0.1, 0.1
params.minDistBetweenBlobs, opacity = 100, 0.1
detector = cv2.SimpleBlobDetector_create(params)

center_x, center_y, b =  0, 0, 100

def operation_detect():
    pipeline.start(config)
    x_v, y_v, count = [], [], 0
    global b , center_x, center_y
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if  not color_frame:
            continue
        frame  = np.asanyarray(color_frame.get_data())
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_range, upper_range)
        mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        im = np.asanyarray(mask_3)
        time.sleep(1)
        keypoints = detector.detect(im)
        if keypoints is not None:
            for k in keypoints:
                cv2.circle(frame, (int(k.pt[0]), int(k.pt[1])), int(k.size/3), (0, 0, 255), -1)
                cv2.line(frame, (int(k.pt[0])-20, int(k.pt[1])), (int(k.pt[0])+20, int(k.pt[1])), (0,0,0), 2)
                cv2.line(frame, (int(k.pt[0]), int(k.pt[1])-20), (int(k.pt[0]), int(k.pt[1])+20), (0,0,0), 2)
                x_v.append(int(k.pt[0]))
                y_v.append(int(k.pt[1]))
            cv2.addWeighted(frame, opacity, im, 1 - opacity, 0, im)
            cv2.imshow("Output", cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)) 
            cv2.waitKey(1)
        b_v, xy, all_b_list, out = [], [], [], 1
        p = math.sqrt((px*px) + (py*py))
        count = count + 1
        for x,y in zip(x_v,y_v):
            xy_i = math.sqrt((x*x) + (y*y))
            xy.append(xy_i)
            all_b_list.append(abs(xy_i - p))
            if abs(xy_i - p) <= b:
                center_x, center_y, b, out = x, y, abs(xy_i - p), 0
                b_v.append(b)

        if count == 3:
            print (" (ㆆ_ㆆ) ")
            min_value_index = all_b_list.index(min(all_b_list))
            center_x, center_y = x_v[min_value_index], y_v[min_value_index]
            print("Closest Circle is:", center_x,",", center_y)
            print ("Differenace with the optical center: ", (px-center_x),",",(py-center_y))
            break
        elif out == 0:
            print("Closest Circle is:", center_x,",", center_y)
            print ("Differenace with the optical center: ", (px-center_x),",",(py-center_y))
            break
        else:
            continue
    pipeline.stop()
    return (center_x,center_y)

def operation_correct():
    rob = urx.Robot("192.168.50.110", use_rt=True) # connect to UR10
    while True:
        x,y = operation_detect()
        if abs(x-px) <= 1 and abs(y-py) <= 1: break
        else:
            if abs(x-px) >= 10 and abs(y-py) >= 10: delta_x, delta_y, a, t = (x-px) * 5, (y-py) * 5, 0.2, 0.2
            else: delta_x, delta_y, a, t = (x-px) * 0.08, (y-py) * 0.08, 0.2, 0.05
            move = [-delta_x, 0, -delta_y ,0,0,0]
            rob.speedl(move, a, t)  
            continue
    rob.close()     # Close robot
    # get time taken to run the for loop code   
    print ("(ง︡'-'︠)ง")


def operation_force(): ## NEED TO FIX THE DIRECTION OF THIS ONE
    rob = urx.Robot("192.168.50.110", use_rt=True) # connect to UR10   
    r = [0,-0.003,0,0,0,0]
    while True:
        rob.speedl(r, 0.2, 0.1)    # Start Moving
        f = rob.get_force(wait=True)    # Check Force While Moving
        print(f)
        if f < 53 or f == None: continue # Stop When Limit Force is Detected (The part need to be reged in order for this to work)
        else: break 
    rob.close()     # Close robot
    
# operation_force()
