import urx
from IPython import embed
# import logging
import time
import pygame
import math3d as m3d
from math import pi
import sys

def Drill_on():
    # Get robot
    robot = urx.Robot("192.168.50.110")
    # Set Digital Outputs to Turn the Drill On
    robot.set_digital_out(1, False)
    robot.set_digital_out(2, True)
    # Keep it working 
    time.sleep(2)
    # Set Digital Outputs to Turn the Drill Off
    robot.set_digital_out(1, True)
    # Close robot
    robot.close()
