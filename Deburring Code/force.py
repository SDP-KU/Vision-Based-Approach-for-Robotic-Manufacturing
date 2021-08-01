import urx
import math3d as m3d
from IPython import embed
import time
import pygame
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R


def force():   
    rob = urx.Robot("192.168.50.110", use_rt=True)
    D2E_rad = [-121.1162/1000, -135.2056/1000, 144.74650/1000, 1.55860250, -0.00987130, -0.67725880]    
    R = np.matrix([ [ 0.7792554,  0.6266286, -0.0098711],[ -0.0153331,  0.0033170, -0.9998769],[ -0.6265188,  0.7793109,  0.0121929 ]]) # Rotation Matrix
    # vec = [1.5551518,0.6770787,-0.0196741] # euler zyx
    # vec1 = [1.4967209, 0.5187476, -0.5400428] # Axis with angle magnitude (radians) [x, y, z]
    R1 = np.transpose(R)
    x = [0,0,0.08]
    vec = np.multiply( R1 , x)
    # print(vec)
    r = [0,-0.008, -0.002 ,0,0,0] 
    r_new = [-0.00078969,-0.07999015, -0.097543 ,0,0,0]
    r1 = [-0.0501215, 0.06234487, 0.00097543 ,0,0,0] # transpose
    input()
    while True:
        rob.speedl(r, 0.15, 0.1)    
        f = rob.get_force(wait=True)
        print(f)
        if f < 55 or f == None:
            # input()
            continue
        else:
            break
        False

    rob.close()

# def force():
#     rob = urx.Robot("192.168.50.110", use_rt=True) # Get Robot
#     force = rob.get_force(wait=True) # Get Force
#     print (force) # Print Force
#     rob.close() # Close Robot
#     return force
# def a():
#     rob = urx.Robot("192.168.50.110", use_rt=True)
#     D2E_rad = [-121.1162/1000, -135.2056/1000, 144.74650/1000, 1.55860250, -0.00987130, -0.67725880]
#     tool = [0,0,0.08]
#     R = np.matrix([ [ 0.7792554, -0.0000508, -0.6267064],[ -0.6266286,  0.0156877, -0.7791601],[ 0.0098711,  0.9998769,  0.0121929 ]])
#     R1 = np.transpose(R)
#     # print(R)
#     flange = np.multiply(R1 , tool)
#     # print(flange)
#     # new = m3d.Transform(D2E_rad) * rob.get_pose()
#     new = R * rob.get_pose()
#     rob.set_pose(new)
#     tool1 = [0,0,0.08]
#     x = [0.00078969, -0.07999015, 0.00097543, 0,0,0]
#     input()
#     while True:
#         rob.speedl([0,0,0.08,0,0,0], 0.15, 0.3)    
#         f = rob.get_force(wait=True)
#         print(f)
#         if f < 40 or f == None:
#             input()
#             continue
#         else:
#             break
#         False
#     rob.close()
