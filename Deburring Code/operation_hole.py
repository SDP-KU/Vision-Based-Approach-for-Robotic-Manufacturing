import math3d as m3d
from IPython import embed
import pygame, sys, time, urx
import numpy as np
import math
import cv2, keyboard, os
import pyrealsense2 as rs
from hole_detect import operation_detect

def operation():
    rob = urx.Robot("192.168.50.110", use_rt=True) # connect to UR10
    C2E_x, C2E_y, C2E_z = -0.0689, -0.1042, 0.21400
    C2E_rx, C2E_ry, C2E_rz = -1.6229758, -0.0024435, 2.38062460
    cam = [C2E_x,C2E_y,C2E_z,C2E_rz,C2E_ry,C2E_rx]
    cam = m3d.Transform(cam) # position matrix 3x3 represent both orientation and translation
    px, py = 320 , 246
    while True:
        x,y = operation_detect()
        # input("enter")
        if abs(x-px) <= 1 and abs(y-py) <= 1:
            break
        else:
            # Hole detection & move to hole center
            delta_x = (x-px) * 0.15
            delta_y = (y-py) * 0.15
            new_pos = m3d.Transform([delta_x, delta_y, 0, 0, 0, 0])
            # input("enter")
            move_to = m3d.Transform(rob.get_pose() * cam * new_pos).get_pose_vector() 
            move = [-delta_x,0,-delta_y  ,0,0,0]
            rob.speedl(move, 0.15, 0.15)  
            continue
    rob.close()     # Close robot

def force_sense():
    rob = urx.Robot("192.168.50.110", use_rt=True) # connect to UR10   
    r = [0,-0.008, -0.002 ,0,0,0] 
    while True:
        rob.speedl(r, 0.15, 0.1)    
        f = rob.get_force(wait=True)
        print(f)
        if f < 65 or f == None:
            continue
        else:
            break
    rob.close()     # Close robot


# move out a bit (rear deburring)

def Drill_on():
    rob = urx.Robot("192.168.50.110", use_rt=True) # connect to UR10
    # Set Digital Outputs to Turn the Drill On
    rob.set_digital_out(1, False)
    rob.set_digital_out(2, True)
    time.sleep(2)     # Keep it working 
    rob.set_digital_out(1, True)     # Set Digital Outputs to Turn the Drill Off
    rob.close()     # Close robot

# move out a bit (front deburring)

# move out completly

# replace drill pose with camera
