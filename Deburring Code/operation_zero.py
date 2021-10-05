from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox
import numpy as np
import sys, time, math, cv2, keyboard
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
from ArUco_detect import ArUco
from operation_hole import operation_correct, operation_force, operation_drill

# get starting time
start = time.time()

## USER DEFINED
# camera position with respect to end-effector
C2E_x = -6.80029175e-02*1000
C2E_y = 1.12672779e-01*1000
C2E_z =  -1.78175184e-02*1000
C2E_rx = np.multiply(1.5550587, 180/math.pi) 
C2E_ry = np.multiply(0.0001366, 180/math.pi)
C2E_rz = np.multiply(-2.3656934, 180/math.pi)

# Drill position with respect to end-effector  ##__ NEED RE-DOING __##
D2E_x = 7.8143
D2E_y = -180.7194
D2E_z = 126.1667
D2E_rx = np.multiply(1.5450804, 180/math.pi) 
D2E_ry = np.multiply(0.000781, 180/math.pi)
D2E_rz = np.multiply(0.0300131, 180/math.pi)

# establish a link with the simulator
RDK = Robolink()
# gather robot, tool and reference frames from the station
robot = RDK.Item('UR10 A', ITEM_TYPE_ROBOT)

def connect_DK():     # Connect To Robot
    RUN_ON_ROBOT = True
    if RUN_ON_ROBOT:    # Connect to the robot using default IP
        success = robot.ConnectSafe()   # Try to connect multiple times
        status, status_msg = robot.ConnectedState()
        if status != ROBOTCOM_READY:    # Stop if the connection did not succeed
            print(status_msg)
            raise Exception("Failed to connect: " + status_msg)
        RDK.setRunMode(RUNMODE_RUN_ROBOT)   # This will set to run the API programs on the robot and the simulator (online programming)

connect_DK() # Connect with the Robot
robot_joints = robot.Joints()   # get the current robot joints
robot_position = robot.SolveFK(robot_joints)   # get the current robot position from the joints
aruco_pos = ArUco() # ArUco Detection

# Get the position matrix of the tools 4x4
ENDEFFECTOR_TO_CAM = robodk.KUKA_2_Pose([C2E_x,C2E_y,C2E_z,C2E_rz,C2E_ry,C2E_rx])
ENDEFFECTOR_TO_DRILL= robodk.KUKA_2_Pose([D2E_x, D2E_y, D2E_z, D2E_rz, D2E_ry, D2E_rx])

Drill = robot.AddTool(ENDEFFECTOR_TO_DRILL, tool_name='Drill') # Add Drill
Camera = robot.AddTool(ENDEFFECTOR_TO_CAM, tool_name='Camera') # Add Camera
robot.setTool(Camera) # Set Active Tool

table, ur_base = RDK.Item('Table'), RDK.Item('UR10 Base') # Add Table Item & UR10 Base Item

# Add ArUco Target and Initialze it Position
aruco_marker = RDK.Item('ARUCO', ITEM_TYPE_TARGET)
aruco_target = RDK.AddTarget('aruco', itemparent=aruco_marker)
CAM_TO_ARUCO = robodk.KUKA_2_Pose(aruco_pos)
aruco_pose = robot_position * ENDEFFECTOR_TO_CAM * CAM_TO_ARUCO
aruco_target.setPose(aruco_pose)

def speed(): robot.setSpeed(speed_linear =20, speed_joints=3, accel_linear=-1, accel_joints=-1)
speed() # Set Speed

def CreateHole( name, pos, x,y,z ):
    Hole_Item = RDK.Item(name, ITEM_TYPE_TARGET)        # Add Item
    Hole_Target = RDK.AddTarget(name, itemparent=aruco_marker)      # Create Target
    Hole_Pose = pos * transl(x,y,z)     # Get Position
    Hole_Set_Pose = Hole_Target.setPose(Hole_Pose)      # Set Position
    Robot_Joints = robot.SolveIK(Hole_Pose, joints_approx=None, tool=ENDEFFECTOR_TO_DRILL, reference=None)      # Calculate New Robot Joints
    return (Hole_Pose)

# Create and initialize Holes, return Hole pose
pose1 = CreateHole("Hole 1", aruco_pose , 80, 30, 0)
# pose2 = CreateHole("Hole 2", pose1, 50, 0, 0)
# pose3 = CreateHole("Hole 3", pose2, 50, 0, 0)
# pose4 = CreateHole("Hole 4", pose3, 50, 0, 0)
# pose5 = CreateHole("Hole 5", pose4, 50, 0, 0)
# pose6 = CreateHole("Hole 6", pose5, 0, 40, 0)
# pose7 = CreateHole("Hole 7", pose6, -50, 0, 0)
# pose8 = CreateHole("Hole 8", pose7, -50, 0, 0)
# pose9 = CreateHole("Hole 9", pose8, -50, 0, 0)
# pose10 = CreateHole("Hole 10", pose9, -50, 0, 0)

input('enter')

robot.MoveL(aruco_pose * transl(0,0,-125)) # Move to ArUco 

input('enter')

def operation(pose):
    robot.MoveL(pose * transl(0,0,-60))    
    input('enter')
    operation_correct() # hole detection
    connect_DK() # Re-Connect with the Robot
    speed() # Re-set Speed
    robot.setTool(Drill) # Set active tool to Drill
    input('enter STOP THE PROGRAM')
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(33,81,5)) # Move Drill to infront of the Hole, Move to x,y position of the Camera, move forward in the z
    operation_force() # insert and force sensing
    connect_DK() # Re-Connect with the Robot
    speed() # Re-set Speed
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(0,0,-35)) # Move the Drill out until the Debarring tip is around the walls of the Hole (REAR)
    operation_drill() # Strat Debarring
    connect_DK() # Re-Connect with the Robot
    speed() # Re-set Speed
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(0,0,-5)) # Move the Drill out until the Debarring tip is around the walls of the Hole (FRONT)
    operation_drill() # Strat Debarring
    connect_DK() # Re-Connect with the Robot
    speed() # Re-set Speed
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(0,0,-29)) # Take Drill out of the Hole
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(-33,-81,-5)) # Put Camera infront of the Hole again
    robot.setTool(Camera) # Re-set acvtive tool to Camera

operation(pose1)
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

# get time taken to run the for loop code 
elapsed_time_fl = (time.time() - start)
print ("Total Computation Time: ",elapsed_time_fl, "Seconds")
