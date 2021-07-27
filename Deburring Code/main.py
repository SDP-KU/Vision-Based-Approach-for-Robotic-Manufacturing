from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox
import numpy as np
import sys, time, math, cv2, keyboard
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
import cv2.aruco as aruco
from hole_detect import HoleDetec
from ArUco_detect import ArUco
from move_to_hole import Move
from drill import Drill_on
import rtde_control

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
D2E_ry = np.multiply(-0.00987130, 180/math.pi)
D2E_rz = np.multiply(-0.67725880, 180/math.pi)

# establish a link with the simulator
RDK = Robolink()
# gather robot, tool and reference frames from the station
robot = RDK.Item('UR10 A', ITEM_TYPE_ROBOT)

def connect():
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

connect() # Connect with the Robot

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
aruco_target.setPose(aruco_pose)
new_robot_joints = robot.SolveIK(aruco_pose)
new_robot_config = robot.JointsConfig(new_robot_joints)

robot.setSpeed(speed_linear =20, speed_joints=3, accel_linear=-1, accel_joints=-1) # set speed

check_collision = COLLISION_ON # collision 

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
pose2 = CreateHole("Hole 2", pose1, -50, 0, 0)
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

# Move to ArUco
Move(robot, aruco_pose, "aruco", ENDEFFECTOR_TO_CAM) # Move to ArUco

def Holes(pose, char):
    Move(robot, pose, char, ENDEFFECTOR_TO_CAM) # Move to Hole
    robot.setTool(Drill) # Set active tool to Drill
    target = transl(33,77,30) # Move to x,y position of the Camera, move forward in the z
    input("press enter to proceed")
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*target) # Move Drill to infront of the Hole
    input("press enter to proceed")
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(0,0,53)) # Move Drill inside the Hole
    input("press enter to proceed")
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(0,0,-35)) # Move the Drill out until the Debarring tip is around the walls of the Hole
    input("press enter to proceed")
    Drill_on() # Strat Debarring
    connect() # Re-Connect with the Robot
    robot.setSpeed(speed_linear =20, speed_joints=3, accel_linear=-1, accel_joints=-1) # Re-Set Speed
    input("press enter to proceed")
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(0,0,-18)) # Take Drill out of the Hole
    input("press enter to proceed")
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(-33,-77,-30)) # Put Camera infront of the Hole again
    robot.setTool(Camera) # Re-set acvtive tool to Camera


Holes(pose1, "hole 1")
# Holes(pose2, "hole 2")

# def force():
#     #---Parameters
#     task_frame = robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL                                #Tool frame
#     selection_vector = [0, 0, 1, 0, 0, 0]                                                          #Component with force constrain 
#     wrench_down = [0, 0, 2, 0, 0, 0]                                                             #Force constrain
#     # wrench_up = [0, 0, 10, 0, 0, 0]                                                              #Force constrain
#     force_type = 2                                                                                 #1=point force, 2=no transformtion, 3=motion force
#     # limits = [2, 2, 1.5, 1, 1, 1]                                   #Speed limit?
#     # dt = 1.0/500  # 2ms
#     # joint_q = [-1.54, -1.83, -2.28, -0.59, 1.60, 0.023]             #RoboDK
#     # #---Move to initial joint position with a regular moveJ
#     # rtde_c.moveJ(joint_q)                                           #RoboDK

#     #---Execute 500Hz control loop for 4 seconds, each cycle is 2ms
#     #for i in range(2000):
#         ##start = time.time()
#         # First move the robot down for 2 seconds, then up for 2 seconds
#         #if i > 1000:
#            # rtde_c.forceMode(task_frame, selection_vector, wrench_up, force_type, limits)
#         #else:
#     rtde_c.forceMode(task_frame, selection_vector, wrench_down, force_type, limits)
#         #end = time.time()
#         #duration = end - start
#         #if duration < dt:
#             #time.sleep(dt - duration)

#     rtde_c.forceModeStop()
#     rtde_c.stopScript()
