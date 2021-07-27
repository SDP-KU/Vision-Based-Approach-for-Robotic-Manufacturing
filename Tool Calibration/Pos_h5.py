# from ToolCalibration import Positions
from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox
import numpy as np
import sys, time, math, cv2, keyboard
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
import h5py
import copy

# establish a link with the simulator
RDK = Robolink()
# gather robot, tool and reference frames from the station
robot = RDK.Item('UR10 A', ITEM_TYPE_ROBOT)

# Connect To Robot
RUN_ON_ROBOT = True
if RDK.RunMode() != RUNMODE_SIMULATE:
    RUN_ON_ROBOT = False
if RUN_ON_ROBOT:
    # Connect to the robot using default IP
    success = robot.Connect() # Try to connect once
    status, status_msg = robot.ConnectedState()
    if status != ROBOTCOM_READY:
        # Stop if the connection did not succeed
        print(status_msg)
        raise Exception("Failed to connect: " + status_msg)
    
    # This will set to run the API programs on the robot and the simulator (online programming)
    RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Note: This is set automatically when we Connect() to the robot through the API

# Create file to save vectors in
  #-- Creating h5 file
data = h5py.File("Final_Calb.hdf5", "w")
MatA=[]
MatD=[]

def Take_Pose():
    while True:
        ask = input("proceed (y/n)")
        if ask == 'y':
            n=0
            # get the current robot joints
            robot_joints = robot.Joints()

            # get the robot position from the joints
            robot_position = robot.SolveFK(robot_joints)

            #-- Get Base to Flange matrix
            robot_pose_matrix = copy.deepcopy(np.array(robot_position)) #Matrix A
            print(robot_pose_matrix)

            #--Get distance values 
            x = input("x: ")
            y = input("y: ")
           

            #Convert to cm
            x = int(x)*50
            y = int(y)*40

            #-- Get Workpiece to hole matrix
            Coord_matrix=np.array([[1,0,0,x],[0,1,0,y],[0,0,1,0],[0,0,0,1]]) #Matrix D
            print(Coord_matrix)
        
            #-- Store data
            MatA.append(robot_pose_matrix)
            MatD.append(Coord_matrix)
        else:
            break
Take_Pose()


#-- Writing data in h5 file
data.create_dataset('B2F', data=MatA)
data.create_dataset('W2H', data=MatD)

data.close()

#print (MatA)
#print (MatD)

# Read H5 file
f = h5py.File("Final_Calb.hdf5", "r")
# Get and print list of datasets within the H5 file
datasetNames = [n for n in f.keys()]
for n in datasetNames:
    print(n)

f.close()


