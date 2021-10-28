from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox
import numpy as np
import sys, time, math, cv2, keyboard
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
from ArUco_detect import ArUco
from operation_hole import operation_correct, operation_force, operation_on_off

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

# actuator position with respect to end-effector 
A2E_x = 7.8143
A2E_y = -180.7194
A2E_z = 126.1667
A2E_rx = np.multiply(1.5450804, 180/math.pi) 
A2E_ry = np.multiply(0.000781, 180/math.pi)
A2E_rz = np.multiply(0.0300131, 180/math.pi)

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
ENDEFFECTOR_TO_ACT= robodk.KUKA_2_Pose([A2E_x, A2E_y, A2E_z, A2E_rz, A2E_ry, A2E_rx])

Actuator = robot.AddTool(ENDEFFECTOR_TO_ACT, tool_name='Actuator') # Add Actuator
Camera = robot.AddTool(ENDEFFECTOR_TO_CAM, tool_name='Camera') # Add Camera
robot.setTool(Actuator) # Set Active Tool

table, ur_base = RDK.Item('Table'), RDK.Item('UR10 Base') # Add Table Item & UR10 Base Item

def create_target(name, x, y, z, rx, ry, rz): # mm & deg (Euler's zyx)
    item = RDK.Item(name)
    target = RDK.AddTarget(name, itemparent=item)
    ind_pose = robodk.KUKA_2_Pose([x, y, z, rz, ry, rx])
    target.setPose(ind_pose)
    return ind_pose

## 3 FIXED POINTS: Q-TIPS, PAINT, TRASH-BIN (x = 270 is similar to the ACT frame for the downward motion)
# the up-down distance is the y axes
q_tip = create_target('Q Tip', 100, -600, 50, 270, 0, 0)
trash = create_target('Bin', 200, -600, 50, 270, 0, 0)
paint = create_target('Paint', 300, -400, 50, 270, 0, 0)

############## TEST ONLY #################
# robot.MoveL(q_tip * transl(0,-50,0))
# robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,20,0))
# robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,-30,0))
# robot.MoveL(paint * transl(0,-50,0))
# robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,20,0))
# robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,-30,0))
# robot.MoveL(robot_joints)
# robot.MoveL(trash * transl(0,-50,0))
# robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,20,0))
# robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,-30,0))
# robot.MoveL(robot_joints)

# input('enter')

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
pose2 = CreateHole("Hole 2", pose1, 50, 0, 0)
pose3 = CreateHole("Hole 3", pose2, 50, 0, 0)
pose4 = CreateHole("Hole 4", pose3, 50, 0, 0)
# pose5 = CreateHole("Hole 5", pose4, 50, 0, 0)
# pose6 = CreateHole("Hole 6", pose5, 0, 40, 0)
# pose7 = CreateHole("Hole 7", pose6, -50, 0, 0)
# pose8 = CreateHole("Hole 8", pose7, -50, 0, 0)
# pose9 = CreateHole("Hole 9", pose8, -50, 0, 0)
# pose10 = CreateHole("Hole 10", pose9, -50, 0, 0)

def paint(pose_1, pose_2):
    # move to pick the q-tips then dip in the paint
    robot.MoveL(q_tip * transl(0,-50,0))
    operation_on_off(True)
    connect_DK() # Re-Connect with the Robot
    speed() # Re-set Speed
    robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,20,0))
    operation_on_off(False)
    connect_DK() # Re-Connect with the Robot
    speed() # Re-set Speed
    robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,-30,0))
    robot.MoveL(paint * transl(0,-50,0))
    robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,20,0))
    robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,-30,0))
    robot.MoveL(robot_joints)

    # Aruco, Hole Detection & Applying Paint
    setTool(Camera) # change to Camera to detect
    robot.MoveL(aruco_pose * transl(0,0,-125)) # Move to ArUco 
    robot.MoveL(pose_1 * transl(0,0,-60)) # Move to hole
    operation_correct() # hole detection & correction
    connect_DK() # Re-Connect with the Robot
    speed() # Re-set Speed 
    ## Save current join setup to use with actuator
    hole_pose_1 = robot.SolveFK(robot.Joints())
    setTool(Actuator)
    robot.MoveL(hole_pose_1) # Move ACT to hole
    operation_force() # Apply paint
    connect_DK() # Re-Connect with the Robot
    speed() # Re-set Speed
    robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,-30,0)) ## check which axis to move back
    setTool(Camera)

    # Move back, move to the second hole and then trash
    robot.MoveL(pose_2 * transl(0,0,-60)) # Move to hole
    operation_correct() # hole detection & correction
    connect_DK() # Re-Connect with the Robot
    speed() # Re-set Speed 
    ## Save current join setup to use with actuator
    hole_pose_2 = robot.SolveFK(robot.Joints())
    setTool(Actuator)
    robot.MoveL(hole_pose_2) # Move ACT to hole
    operation_force() # Apply paint
    connect_DK() # Re-Connect with the Robot
    speed() # Re-set Speed
    robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,-30,0)) ## check which axis to move back

    # Trash the q-tip
    robot.MoveL(trash * transl(0,-50,0))
    robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,20,0))
    operation_on_off(False)
    operation_on_off(True)
    robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,-30,0))
    robot.MoveL(robot_joints)


paint(pose1, pose2)
paint(pose3, pose4)

# get time taken to run the for loop code 
elapsed_time_fl = (time.time() - start)
print ("Total Computation Time: ",elapsed_time_fl, "Seconds")
