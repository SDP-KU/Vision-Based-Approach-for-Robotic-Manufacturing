import urx
from IPython import embed
import time
import pygame
import sys

def force():
    rob = urx.Robot("192.168.50.110", use_rt=True) # Get Robot
    force = rob.get_force() # Get Force
    print (force) # Print Force
    rob.close() # Close Robot
    return force
