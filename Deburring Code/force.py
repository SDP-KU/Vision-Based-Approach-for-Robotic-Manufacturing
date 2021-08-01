import urx
import math3d as m3d
from IPython import embed
import time
import pygame
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R


def force_sense():   
    rob = urx.Robot("192.168.50.110", use_rt=True)
    r = [0,-0.008, -0.002 ,0,0,0] 
    while True:
        rob.speedl(r, 0.15, 0.1)    
        f = rob.get_force(wait=True)
        print(f)
        if f < 65 or f == None:
            continue
        else:
            break
        False

    rob.close()
    time.sleep(2)
