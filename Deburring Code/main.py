from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox
import numpy as np
import sys, time, math, cv2, keyboard
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
import cv2.aruco as aruco
from Detect_Holes_Function import HoleDetec
from ArUco_Detect import ArUco
from Move_to_Hole import Move
from Drill import Drill_on

## USER DEFINED
# camera position with respect to end-effector 
C2E_x = -0.0689*1000
C2E_y = -0.1042*1000
C2E_z = 0.21400*1000
C2E_rx = np.multiply(-1.6229758, 180/math.pi) 
C2E_ry = np.multiply(-0.0024435, 180/math.pi)
C2E_rz = np.multiply(2.38062460, 180/math.pi)

# Drill position with respect to end-effector
D2E_x = -121.1162
D2E_y = -135.2056
D2E_z = 144.74650
D2E_rx = np.multiply(1.55860250, 180/math.pi) 
D2E_ry = np.multiply(-0.0098713, 180/math.pi)
D2E_rz = np.multiply(-0.6772588, 180/math.pi)

# establish a link with the simulator
RDK = Robolink()
# gather robot, tool and reference frames from the station
robot = RDK.Item('UR10 A', ITEM_TYPE_ROBOT)

# # Connect To Robot
# RUN_ON_ROBOT = True
# if RDK.RunMode() != RUNMODE_SIMULATE:
#     RUN_ON_ROBOT = False
# if RUN_ON_ROBOT:
#     # Connect to the robot using default IP
#     success = robot.Connect() # Try to connect once
#     status, status_msg = robot.ConnectedState()
#     if status != ROBOTCOM_READY:
#         # Stop if the connection did not succeed
#         print(status_msg)
#         raise Exception("Failed to connect: " + status_msg)
    
#     # This will set to run the API programs on the robot and the simulator (online programming)
#     RDK.setRunMode(RUNMODE_RUN_ROBOT)
#     # Note: This is set automatically when we Connect() to the robot through the API

# get the current robot joints
robot_joints = robot.Joints()
# get the robot position from the joints
robot_position = robot.SolveFK(robot_joints)
# get the robot configuration (robot joint state)
robot_config = robot.JointsConfig(robot_joints)

# ArUco Detection
aruco_pos = ArUco()

# Get the position matrix of the marker
ENDEFFECTOR_TO_CAM = robodk.KUKA_2_Pose([C2E_x,C2E_y,C2E_z,C2E_rz,C2E_ry,C2E_rx])
ENDEFFECTOR_TO_DRILL= robodk.KUKA_2_Pose([D2E_x, D2E_y, D2E_z, D2E_rz,D2E_ry,D2E_rx])

# add tool (camera)
Drill = robot.AddTool(ENDEFFECTOR_TO_DRILL, tool_name='Rivet') 
Camera = robot.AddTool(ENDEFFECTOR_TO_CAM, tool_name='Camera')
robot.setTool(Camera) # Set Active Tool

table = RDK.Item('Table') # Add Table Item
ur_base = RDK.Item('UR10 Base') # Add UR10 Base Item

# Add ArUco Target and Initialze it Position
aruco_marker = RDK.Item('ARUCO', ITEM_TYPE_TARGET)
aruco_target = RDK.AddTarget('aruco', itemparent=aruco_marker)
CAM_TO_ARUCO = robodk.KUKA_2_Pose(aruco_pos)
aruco_pose = robot_position * ENDEFFECTOR_TO_CAM * CAM_TO_ARUCO
# pose1 = aruco_pose * transl(-70, 35, 0)
aruco_target.setPose(aruco_pose)
new_robot_joints = robot.SolveIK(aruco_pose)
new_robot_config = robot.JointsConfig(new_robot_joints)

# set speed
speed_linear = 20
robot.setSpeed(speed_linear, speed_joints=3, accel_linear=-1, accel_joints=-1)

# collision 
check_collision = COLLISION_ON

def CreateHole( name, pos, x,y,z ):
    Hole_Item = RDK.Item(name, ITEM_TYPE_TARGET)        # Add Item
    Hole_Target = RDK.AddTarget(name, itemparent=aruco_marker)      # Create Target
    Hole_Pose = pos * transl(x,y,z)     # Get Position
    Hole_Set_Pose = Hole_Target.setPose(Hole_Pose)      # Set Position
    Robot_Joints = robot.SolveIK(Hole_Pose, joints_approx=None, tool=ENDEFFECTOR_TO_DRILL, reference=None)      # Calculate New Robot Joints
    Robot_Config = robot.JointsConfig(Robot_Joints)     # Calculate Robot Configuration For New Joints
    return (Hole_Pose)

# Create and initialize Holes, return Hole pose
pose1 = CreateHole("Hole 1", aruco_pose , -70, 35, 0)
# pose2 = CreateHole("Hole 2", pose1, -50, 0, 0)
# pose3 = CreateHole("Hole 3", pose2, -50, 0, 0)
# pose4 = CreateHole("Hole 4", pose3, -50, 0, 0)
# pose5 = CreateHole("Hole 5", pose4, 0, 45, 0)
# pose6 = CreateHole("Hole 6", pose5, 50, 0, 0)
# pose7 = CreateHole("Hole 7", pose6, 50, 0, 0)
# pose8 = CreateHole("Hole 8", pose7, 50, 0, 0)
# pose9 = CreateHole("Hole 9", pose8, 50, 0, 0)
# pose10 = CreateHole("Hole 10", pose9, 50, 0, 0)
# pose11 = CreateHole("Hole 11", pose10, 50, 0, 0)
# pose12 = CreateHole("Hole 12", pose11, 50, 0, 0)
# pose13 = CreateHole("Hole 13", pose12, 50, 0, 0)

# Move to Target
Move(robot, aruco_pose, "aruco", ENDEFFECTOR_TO_CAM) # Move to ArUco
Move(robot, pose1, "hole 1", ENDEFFECTOR_TO_CAM) # Move to Hole 1

input("press enter to proceed")
robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_CAM*ENDEFFECTOR_TO_DRILL) # approximate value transl(-40,-70,20)
# Drill_on()

# Hole Position must change again !!!

# Move(robot, pose2, "hole 2", ENDEFFECTOR_TO_CAM) # Move to Hole 2
# Move(robot, pose3, "hole 3", ENDEFFECTOR_TO_CAM) # Move to Hole 3
# Move(robot, pose4, "hole 4", ENDEFFECTOR_TO_CAM) # Move to Hole 4
# Move(robot, pose5, "hole 5", ENDEFFECTOR_TO_CAM) # Move to Hole 5
# Move(robot, pose6, "hole 6", ENDEFFECTOR_TO_CAM) # Move to Hole 6
# Move(robot, pose7, "hole 7", ENDEFFECTOR_TO_CAM) # Move to Hole 7
# Move(robot, pose8, "hole 8", ENDEFFECTOR_TO_CAM) # Move to Hole 8
# Move(robot, pose9, "hole 9", ENDEFFECTOR_TO_CAM) # Move to Hole 9
# Move(robot, pose10, "hole 10", ENDEFFECTOR_TO_CAM) # Move to Hole 10
# Move(robot, pose11, "hole 11", ENDEFFECTOR_TO_CAM) # Move to Hole 11
# Move(robot, pose12, "hole 12", ENDEFFECTOR_TO_CAM) # Move to Hole 12
# Move(robot, pose13, "hole 13", ENDEFFECTOR_TO_CAM) # Move to Hole 13
