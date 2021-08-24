from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox
import numpy as np
import sys, time, math, cv2, keyboard
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
import cv2.aruco as aruco
from ArUco_detect import ArUco

## USER DEFINED
# camera position with respect to end-effector   __NEW__
C2E_x = -0.0750*1000
C2E_y = -0.1168*1000
C2E_z = 0.2231*1000
C2E_rx = np.multiply(-1.5721943, 180/math.pi) 
C2E_ry = np.multiply(0.0050954, 180/math.pi)
C2E_rz = np.multiply(2.3598663, 180/math.pi)

# Brush position with respect to end-effector
B2E_x = -138.2393
B2E_y = -124.7527
B2E_z = 141.2898
B2E_rx = np.multiply(1.5750029, 180/math.pi) 
B2E_ry = np.multiply(0.0040261, 180/math.pi)
B2E_rz = np.multiply(-0.7659436, 180/math.pi)

# establish a link with the simulator
RDK = Robolink()
# gather robot, tool and reference frames from the station
robot = RDK.Item('UR10 A', ITEM_TYPE_ROBOT)

def connect_DK():
    # Connect To Robot
    RUN_ON_ROBOT = True
    if RUN_ON_ROBOT:
        # Connect to the robot using default IP
        success = robot.ConnectSafe() # Try to connect multiple times
        status, status_msg = robot.ConnectedState()
        if status != ROBOTCOM_READY:
            # Stop if the connection did not succeed
            print(status_msg)
            raise Exception("Failed to connect: " + status_msg)
        # This will set to run the API programs on the robot and the simulator (online programming)
        RDK.setRunMode(RUNMODE_RUN_ROBOT)
        # Note: This is set automatically when we Connect() to the robot through the API

connect_DK() # Connect with the Robot

# get the current robot joints
robot_joints = robot.Joints()
# get the robot position from the joints
robot_position = robot.SolveFK(robot_joints)
# get the robot configuration (robot joint state)
robot_config = robot.JointsConfig(robot_joints)

aruco_pos = ArUco() # ArUco Detection

# Get the position matrix of the marker 4x4
ENDEFFECTOR_TO_CAM = robodk.KUKA_2_Pose([C2E_x,C2E_y,C2E_z,C2E_rz,C2E_ry,C2E_rx])
ENDEFFECTOR_TO_BRUSH= robodk.KUKA_2_Pose([B2E_x, B2E_y, B2E_z, B2E_rz, B2E_ry, B2E_rx])

Brush = robot.AddTool(ENDEFFECTOR_TO_BRUSH, tool_name='Brush') # Add Brush
Camera = robot.AddTool(ENDEFFECTOR_TO_CAM, tool_name='Camera') # Add Camera
robot.setTool(Camera) # Set Active Tool

table = RDK.Item('Table') # Add Table Item
ur_base = RDK.Item('UR10 Base') # Add UR10 Base Item

# Add ArUco Target and Initialze it Position
def aruco_pose():
    aruco_marker = RDK.Item('ARUCO', ITEM_TYPE_TARGET)
    aruco_target = RDK.AddTarget('aruco', itemparent=aruco_marker)
    CAM_TO_ARUCO = robodk.KUKA_2_Pose(aruco_pos)
    aruco_pose_ = robot_position * ENDEFFECTOR_TO_CAM * CAM_TO_ARUCO
    aruco_target.setPose(aruco_pose_)
    new_robot_joints = robot.SolveIK(aruco_pose_)
    new_robot_config = robot.JointsConfig(new_robot_joints)
    return aruco_pose_

def speed():
    robot.setSpeed(speed_linear =20, speed_joints=3, accel_linear=-1, accel_joints=-1)

speed() # Set Speed

def CreateHole( name, pos, x,y,z ):
    Hole_Item = RDK.Item(name, ITEM_TYPE_TARGET)        # Add Item
    Hole_Target = RDK.AddTarget(name, itemparent=aruco_marker)      # Create Target
    Hole_Pose = pos * transl(x,y,z)     # Get Position
    Hole_Set_Pose = Hole_Target.setPose(Hole_Pose)      # Set Position
    Robot_Joints = robot.SolveIK(Hole_Pose, joints_approx=None, tool=ENDEFFECTOR_TO_BRUSH, reference=None)      # Calculate New Robot Joints
    Robot_Config = robot.JointsConfig(Robot_Joints)     # Calculate Robot Configuration For New Joints
    return (Hole_Pose)

# Create and initialize Holes, return Hole pose
# pose1 = CreateHole("Hole 1", aruco_pose() , 80, 30, 0)
# pose2 = CreateHole("Hole 2", pose1, 50, 0, 0)
# pose3 = CreateHole("Hole 3", pose2, 50, 0, 0)
# pose4 = CreateHole("Hole 4", pose3, 50, 0, 0)
# pose5 = CreateHole("Hole 5", pose4, 50, 0, 0)
# pose6 = CreateHole("Hole 6", pose5, 0, 40, 0)
# pose7 = CreateHole("Hole 7", pose6, -50, 0, 0)
# pose8 = CreateHole("Hole 8", pose7, -50, 0, 0)
# pose9 = CreateHole("Hole 9", pose8, -50, 0, 0)
# pose10 = CreateHole("Hole 10", pose9, -50, 0, 0)


########### CHANGE FROM HERE ###########

# Add Paint Target and Initialze it Position
def paint_pose():
    paint = RDK.Item('PAINT', ITEM_TYPE_TARGET)
    paint_target = RDK.AddTarget('paint', itemparent=paint)
    paint_pose_ = robot_position * ENDEFFECTOR_TO_BRUSH * transl(0,-350,50) * rotx(-23.6)  # xyz depends on the position of the paint holder on the table
    paint_target.setPose(paint_pose_)
    new_robot_joints_paint = robot.SolveIK(paint_pose_)
    new_robot_config_paint = robot.JointsConfig(new_robot_joints_paint)
    return paint_pose_


robot.MoveL(aruco_pose() * transl(0,0,-250))
input("enter")
robot.setTool(Brush)
robot.MoveL(paint_pose() * transl(0,0,-15))
input("enter")
robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_BRUSH * transl(0,0,55))
input("enter")
robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_BRUSH * transl(0,0,-60))

def operation(pose): ##__BIGGEST CHANGE NEEDED HERE__##
    robot.MoveL(pose * transl(0,0,-135))
    # Move(robot, pose, "aruco", ENDEFFECTOR_TO_CAM) # Move to Hole
    operation_correct() # hole detection
    connect_DK() # Re-Connect with the Robot
    speed() # Re-set Speed
    robot.setTool(Drill) # Set active tool to Drill
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(33,77,15)) # Move Drill to infront of the Hole, Move to x,y position of the Camera, move forward in the z
    input("enter")
    operation_force() # insert and force sensing
    connect_DK() # Re-Connect with the Robot
    speed() # Re-set Speed
    input("enter")
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(0,0,-33)) # Move the Drill out until the Debarring tip is around the walls of the Hole (REAR)
    input("enter")
    operation_drill() # Strat Debarring
    connect_DK() # Re-Connect with the Robot
    speed() # Re-set Speed
    # input("enter")
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(0,0,-6)) # Move the Drill out until the Debarring tip is around the walls of the Hole (FRONT)
    # input("enter")
    operation_drill() # Strat Debarring
    connect_DK() # Re-Connect with the Robot
    speed() # Re-set Speed
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(0,0,-27)) # Take Drill out of the Hole
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(-33,-77,-25)) # Put Camera infront of the Hole again
    robot.setTool(Camera) # Re-set acvtive tool to Camera

# operation(pose1)
# operation(pose2)
# operation(pose3)
# operation(pose4)
# operation(pose5)
# operation(pose6)
# operation(pose7)
# operation(pose8)
# operation(pose9)
# operation(pose10)
# operation(pose11)
# operation(pose12)
# operation(pose13)
