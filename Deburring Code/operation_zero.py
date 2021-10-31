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
C2E_x = -0.0334*1000
C2E_y = 0.1329*1000
C2E_z =  0.2080*1000
C2E_rx = np.multiply(-1.5946364, 180/math.pi) 
C2E_ry = np.multiply(0.0401067, 180/math.pi)
C2E_rz = np.multiply(0.013144, 180/math.pi)

# Drill position with respect to end-effector
D2E_x = 0.0565
D2E_y = 192.5735
D2E_z = 117.3004
D2E_rx = np.multiply(-1.5750547, 180/math.pi) 
D2E_ry = np.multiply(-0.0001207, 180/math.pi)
D2E_rz = np.multiply(-0.0331271, 180/math.pi)

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

def speed(): robot.setSpeed(speed_linear =25, speed_joints=4, accel_linear=-1, accel_joints=-1)
speed() # Set Speed

def CreateHole( name, pos, x,y,z ):
    Hole_Item = RDK.Item(name, ITEM_TYPE_TARGET)        # Add Item
    Hole_Target = RDK.AddTarget(name, itemparent=aruco_marker)      # Create Target
    Hole_Pose = pos * transl(x,y,z)     # Get Position
    Hole_Set_Pose = Hole_Target.setPose(Hole_Pose)      # Set Position
    Robot_Joints = robot.SolveIK(Hole_Pose, joints_approx=None, tool=ENDEFFECTOR_TO_DRILL, reference=None)      # Calculate New Robot Joints
    return (Hole_Pose)

# Create and initialize Holes, return Hole pose
pose1 = CreateHole("Hole 1", aruco_pose , -105, -10, 0)
pose2 = CreateHole("Hole 2", pose1, -50, 0, 0)
pose3 = CreateHole("Hole 3", pose2, -50, 0, 0)
pose4 = CreateHole("Hole 4", pose3, -50, 0, 0)

print("start")

robot.MoveL(aruco_pose * transl(0,0,-180)) # Move to ArUco 

def operation(pose):
    robot.MoveL(pose * transl(0,0,-140))    
    operation_correct() 
    connect_DK() 
    speed() 
    robot.setTool(Drill) 
    # input('enter')
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(-32, -82.5, 0)) 
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(0, 0, 12))
    # input('enter')
    operation_force() 
    connect_DK() 
    speed() 
    input('enter')
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(0,0,-35)) 
    # input('enter')
    # operation_drill() 
    connect_DK() 
    speed() 
    # input('enter')
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(0,0,-8))
    # input('enter')
    # operation_drill() 
    connect_DK() 
    speed() 
    # input('enter')
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(0,0,-25)) # Take Drill out of the Hole
    robot.MoveL(robot.SolveFK(robot.Joints())*ENDEFFECTOR_TO_DRILL*transl(32, 82.5 ,12)) # Put Camera infront of the Hole again
    robot.setTool(Camera) # Re-set acvtive tool to Camera

operation(pose1)
operation(pose2)
operation(pose3)
operation(pose4)

print("done (TvT)")

# get time taken to run the for loop code 
elapsed_time_fl = (time.time() - start)
print ("Total Computation Time: ",elapsed_time_fl, "Seconds")
