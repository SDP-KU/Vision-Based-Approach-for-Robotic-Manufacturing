# from ToolCalibration import Positions
from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox
import numpy as np
import sys, time, math, cv2, keyboard, h5py, copy
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
from aruco import ArUco

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
data = h5py.File("cam3_STRATA.h5", "w")
MatA=[]
MatD=[]

def Take_Pose():
    i = 1
    while True:
        ask = input("proceed (y/n)")
        if ask == 'y':
            # get the robot position from the joints
            robot_position = robot.SolveFK(robot.Joints())

            #-- Get Base to Flange matrix
            robot_pose_matrix = copy.deepcopy(np.array(robot_position)) # Matrix A
            print(robot_pose_matrix)

            #-- Get ArUco Pos Matrix
            tvec, rot_mat = ArUco(i)
            i += 1
            Aruco_matrix = np.array([[rot_mat[0][0], rot_mat[0][1], rot_mat[0][2], tvec[0]],[rot_mat[1][0], 
                rot_mat[1][1], rot_mat[1][2], tvec[1]],[rot_mat[2][0], rot_mat[2][1], rot_mat[2][2], tvec[2]],[0,0,0,1]]) # Matrix D
            print(Aruco_matrix)

            #-- Store data
            MatA.append(robot_pose_matrix)
            MatD.append(Aruco_matrix)
        else:
            break

Take_Pose()

#-- Writing data in h5 file
data.create_dataset('C_AR', data=MatD)
data.create_dataset('B_T0', data=MatA)

#-- Close File
data.close()
