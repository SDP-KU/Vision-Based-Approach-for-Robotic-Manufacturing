import math3d as m3d
from IPython import embed
import pygame, sys, time, urx, cv2, keyboard, os, math
import numpy as np
import pyrealsense2 as rs

# Initiate Camera
pipeline = rs.pipeline()
config = rs.config()
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))
color_profile = rs.video_stream_profile(pipeline_profile.get_stream(rs.stream.color))
color_intrinsics = color_profile.get_intrinsics()
w, h = color_intrinsics.width, color_intrinsics.height
found_rgb = False   #Intial Value
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

# HSV Mask Parameters:
lower_range, upper_range = np.array([0, 123, 0]), np.array([179, 255, 255])

# Hole Detection Parameters:
detector, params = cv2.SimpleBlobDetector_create(), cv2.SimpleBlobDetector_Params()
params.filterByArea, params.filterByCircularity, params.filterByInertia = True, True, True
params.minArea, params.maxArea = 150, 6000
params.minCircularity, params.minInertiaRatio = 0.1, 0.1
params.minDistBetweenBlobs, opacity = 100, 0.1
detector = cv2.SimpleBlobDetector_create(params)

b=100

def operation_detect():
    pipeline.start(config)
    x_v, y_v = [], []
    center_x, center_y = 0, 0
    global b
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
        px, py = 320, 246
        count, b_v = 0, []
        # px, py, b = 572, 283, 100
        p = math.sqrt((px*px) + (py*py))
        for x,y in zip(x_v,y_v):
            xy = math.sqrt((x*x) + (y*y))
            count = count + 1
            if abs(xy - p) <= b:
                center_x, center_y = x, y
                b = abs(xy - p)
                b_v.append(b)
                print (b_v)

                ## CHECK the condition of b and count
            else:
                continue
        if count == 3:
            if b_v[0] == b_v[1] == b_v[2]:
                print("Closest Circle is:", center_x,",", center_y)
                print ("Differenace with the optical center: ", (px-center_x),",",(py-center_y))
            break
        elif center_x == 0 and center_y == 0:
            continue
        else:
            print("Closest Circle is:", center_x,",", center_y)
            print ("Differenace with the optical center: ", (px-center_x),",",(py-center_y))
            break



        ## if b did not change in 3 iterations break Flase, 

    # cv2.destroyAllWindows()
    pipeline.stop()
    print (b)
    return (center_x,center_y)

# operation_detect()

def operation_correct():
    # get starting time
    start = time.time()
    rob = urx.Robot("192.168.50.110", use_rt=True) # connect to UR10
    # C2E_x, C2E_y, C2E_z = -0.0689, -0.1042, 0.21400 # AINT GONNA USE THIS
    # C2E_rx, C2E_ry, C2E_rz = -1.6229758, -0.0024435, 2.38062460 # AINT GONNA USE THIS
    # cam = [C2E_x,C2E_y,C2E_z,C2E_rz,C2E_ry,C2E_rx] # AINT GONNA USE THIS
    # cam = m3d.Transform(cam) # position matrix 3x3 represent both orientation and translation
    px, py = 320 , 246
    while True:
        x,y = operation_detect()
        if abs(x-px) <= 1 and abs(y-py) <= 1:
            break
        else:
            if abs(x-px) >= 10 and abs(y-py) >= 10:
                delta_x = (x-px) * 5
                delta_y = (y-py) * 5
                a, t = 0.2, 0.2
            else:
                delta_x = (x-px) * 0.1
                delta_y = (y-py) * 0.1
                a, t = 0.4, 0.1
            # new_pos = m3d.Transform([delta_x, delta_y, 0, 0, 0, 0]) # AINT GONNA USE THIS
            # move_to = m3d.Transform(rob.get_pose() * cam * new_pos).get_pose_vector() # AINT GONNA USE THIS
            move = [-delta_x, 0, -delta_y ,0,0,0]
            rob.speedl(move, a, t)  
            continue
    rob.close()     # Close robot

    # get time taken to run the for loop code   
    elapsed_time_fl = (time.time() - start)
    print ("Hole Detection & Correction Total Time: ",elapsed_time_fl)

def operation_force():
    # get starting time
    start = time.time()

    rob = urx.Robot("192.168.50.110", use_rt=True) # connect to UR10   
    r = [0,-0.007, -0.003 ,0,0,0] # Speed Vector
    r = [0,0,0.003,0,0,0]
    while True:
        rob.speedl(r, 0.15, 0.1)    # Start Moving
        f = rob.get_force(wait=True)    # Check Force While Moving
        print(f)
        if f < 65 or f == None: # Stop When Limit Force is Detected
            continue
        else:
            break
    rob.close()     # Close robot

    # get time taken to run the for loop code 
    elapsed_time_fl = (time.time() - start)
    print (elapsed_time_fl)

def operation_drill():
    # get starting time
    start = time.time()

    rob = urx.Robot("192.168.50.110", use_rt=True) # connect to UR10
    # Set Digital Outputs to Turn the Drill On
    rob.set_digital_out(1, False)
    rob.set_digital_out(2, True)
    time.sleep(2)     # Keep it working 
    rob.set_digital_out(1, True)     # Set Digital Outputs to Turn the Drill Off
    rob.close()     # Close robot

    # get time taken to run the for loop code 
    elapsed_time_fl = (time.time() - start)
    print ("Drill On/Off Time: ",elapsed_time_fl)

operation_correct()
