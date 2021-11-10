import math3d as m3d
from IPython import embed
import pygame, sys, time, urx, cv2, keyboard, os, math
import numpy as np
import pyrealsense2 as rs
import keyboard



def operation_on_off():  # this code works
    rob = urx.Robot("192.168.50.110", use_rt=True) # connect to UR10
    volt = rob.get_analog_in(0, wait=False)
    while True:
        key = input ("hit something\n")
        # open
        if key == ('f'): # forward
            rob.set_digital_out(1, False)
            rob.set_digital_out(2, True) 
            print(rob.get_analog_in(0, wait=False))
        # close
        if key == ('b'): # backward
            rob.set_digital_out(1, True)
            rob.set_digital_out(2, False)
            print(rob.get_analog_in(0, wait=False))
        # break
        if key == ('s'): # stop
            rob.set_digital_out(1, False)
            rob.set_digital_out(2, False)
            print(rob.get_analog_in(0, wait=False))

# operation_on_off()


def testing(char):
    rob = urx.Robot("192.168.50.110", use_rt=True) # connect to UR10
    volt = rob.get_analog_in(0, wait=False)
    print(volt)
    while char == "start": # we need volt to be 4.8 <= volt <= 4.9
        if volt <= 3.3: # forward
            input('enter forward')
            rob.set_digital_out(1, False)
            rob.set_digital_out(2, True)
            time.sleep(0.5) # wait for it to move then stop and check
            rob.set_digital_out(1, False)
            rob.set_digital_out(2, False)
            volt = rob.get_analog_in(0, wait=False)
            print (volt)
            continue
        elif volt >=3.6: # backward
            input('enter backward')
            rob.set_digital_out(1, True)
            rob.set_digital_out(2, False)
            time.sleep(0.5) # wait for it to move then stop and check
            rob.set_digital_out(1, False)
            rob.set_digital_out(2, False)
            volt = rob.get_analog_in(0, wait=False)
            print (volt)
            continue
        break

    while char == "forward": # we need volt to be 6.8 <= volt <= 6.9
        if volt <= 6.6: # forward
            input('enter forward')
            rob.set_digital_out(1, False)
            rob.set_digital_out(2, True)
            time.sleep(0.5) # wait for it to move then stop and check
            rob.set_digital_out(1, False)
            rob.set_digital_out(2, False)
            volt = rob.get_analog_in(0, wait=False)
            print (volt)
            continue
        elif volt >= 6.9: # backward
            input('enter backward')
            rob.set_digital_out(1, True)
            rob.set_digital_out(2, False) 
            time.sleep(0.5) # wait for it to move then stop and check
            rob.set_digital_out(1, False)
            rob.set_digital_out(2, False)
            volt = rob.get_analog_in(0, wait=False)
            print (volt)
            continue
        break
    rob.close() 

    
# testing("start")
# testing("forward")
