from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox
import numpy as np
import sys, time, math, cv2, keyboard
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
from ArUco_detect import ArUco
from operation_hole import operation_correct, operation_force
from test import testing

# get starting time
start = time.time()

## USER DEFINED
# camera position with respect to end-effector
C2E_x = -0.0272*1000
C2E_y = 0.0652*1000
C2E_z = 0.0300*1000
C2E_rx = np.multiply(-1.5813607, 180/math.pi) 
C2E_ry = np.multiply(-0.0057037, 180/math.pi)
C2E_rz = np.multiply(0.0533863, 180/math.pi)

# actuator position with respect to end-effector 
A2E_x = 7.8143
A2E_y = -8.7194
A2E_z = 20.1667
A2E_rx = np.multiply(-1.5813607, 180/math.pi)
A2E_ry = np.multiply(-0.0057037, 180/math.pi)
A2E_rz = np.multiply(0.0533863, 180/math.pi)

# establish a link with the simulator
RDK = Robolink()
# gather robot, tool and reference frames from the station
robot = RDK.Item('UR10 A', ITEM_TYPE_ROBOT)

def connect_DK():     # Connect To Robot
    RUN_ON_ROBOT = True
    if RUN_ON_ROBOT:    
        success = robot.ConnectSafe()   
        status, status_msg = robot.ConnectedState()
        if status != ROBOTCOM_READY: 
            print(status_msg)
            raise Exception("Failed to connect: " + status_msg)
        RDK.setRunMode(RUNMODE_RUN_ROBOT)   

connect_DK() # Connect to the Robot
robot_joints = robot.Joints()   # get the current robot joints
robot_position = robot.SolveFK(robot_joints)   # get the current robot position from the joints
aruco_pos = ArUco() # ArUco Detection

# Get the position matrix of the tools 4x4
ENDEFFECTOR_TO_CAM = robodk.KUKA_2_Pose([C2E_x,C2E_y,C2E_z,C2E_rz,C2E_ry,C2E_rx])
ENDEFFECTOR_TO_ACT= robodk.KUKA_2_Pose([A2E_x, A2E_y, A2E_z, A2E_rz, A2E_ry, A2E_rx])

Actuator = robot.AddTool(ENDEFFECTOR_TO_ACT, tool_name='Actuator') # Add Actuator
Camera = robot.AddTool(ENDEFFECTOR_TO_CAM, tool_name='Camera') # Add Camera
robot.setTool(Actuator) # Set Active Tool

table, ur_base = RDK.Item('Table'), RDK.Item('UR10 Base') 

def create_target(name, x, y, z, rx, ry, rz): # mm & deg (Euler's zyx)
    item = RDK.Item(name)
    target = RDK.AddTarget(name, itemparent=item)
    ind_pose = robodk.KUKA_2_Pose([x, y, z, rz, ry, rx])
    target.setPose(ind_pose)
    return ind_pose

## 3 FIXED POINTS: Q-TIPS, PAINT, TRASH-BIN
# the up-down distance is the y axes
q_tip = create_target('Q Tip', 100, -500, 80, 180, 0, 180)
paint = create_target('Paint', 100, -400, 80, 180, 0, 180)
trash = create_target('Bin', 200, -500, 80, 180, 0, 180)

# Add ArUco Target and Initialze it Position
aruco_marker = RDK.Item('ARUCO', ITEM_TYPE_TARGET)
aruco_target = RDK.AddTarget('aruco', itemparent=aruco_marker)
CAM_TO_ARUCO = robodk.KUKA_2_Pose(aruco_pos)
aruco_pose = robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_CAM * CAM_TO_ARUCO
aruco_target.setPose(aruco_pose)

def speed(): robot.setSpeed(speed_linear =20, speed_joints=5, accel_linear=-1, accel_joints=-1)
speed() # Set Speed

def CreateHole( name, pos, x,y,z ):
    Hole_Item = RDK.Item(name, ITEM_TYPE_TARGET)      
    Hole_Target = RDK.AddTarget(name, itemparent=aruco_marker)   
    Hole_Pose = pos * transl(x,y,z)
    Hole_Set_Pose = Hole_Target.setPose(Hole_Pose)  
    Robot_Joints = robot.SolveIK(Hole_Pose, joints_approx=None, tool=ENDEFFECTOR_TO_ACT, reference=None) 
    return (Hole_Pose)

# Create and initialize Holes, return Hole pose
pose1 = CreateHole("Hole 1", aruco_pose , 110, 30, 0)
pose2 = CreateHole("Hole 2", pose1, 50, 0, 0)
pose3 = CreateHole("Hole 3", pose2, 50, 0, 0)
pose4 = CreateHole("Hole 4", pose3, 50, 0, 0)
# pose5 = CreateHole("Hole 5", pose4, 50, 0, 0)
# pose6 = CreateHole("Hole 6", pose5, 0, 40, 0)
# pose7 = CreateHole("Hole 7", pose6, -50, 0, 0)
# pose8 = CreateHole("Hole 8", pose7, -50, 0, 0)
# pose9 = CreateHole("Hole 9", pose8, -50, 0, 0)
# pose10 = CreateHole("Hole 10", pose9, -50, 0, 0)

def get_paint():
    # move to pick the q-tips then dip in the paint
    robot.MoveL(q_tip * transl(0,0,-120))
    # testing("forward")
    # connect_DK()
    # speed()
    robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,0,20))
    # testing("start")
    # connect_DK()
    # speed()
    robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,0,-20))
    robot.MoveL(paint * transl(0,0,-120))
    robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,0,20))
    robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,0,-30))
    robot.MoveL(robot_joints)

def painting(pose_1, pose_2):
    # Aruco, Hole Detection & Applying Paint
    robot.setTool(Camera)
    robot.MoveL(aruco_pose * transl(0,0,-200))
    aruco_pose_2 = robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_CAM * robodk.KUKA_2_Pose(ArUco()) 
    aruco_target.setPose(aruco_pose_2)
    robot.MoveL(aruco_pose_2 * transl(0,0,-200)) 
    aruco_pose_3 = robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_CAM * robodk.KUKA_2_Pose(ArUco()) 
    aruco_target.setPose(aruco_pose_3)
    robot.MoveL(aruco_pose_3 * transl(0,0,-200)) 

    robot.MoveL(pose_1 * transl(0,0,-120)) 
    operation_correct() 
    operation_correct()
    connect_DK() 
    speed() 

    ### FROM HERE DOES NOT WORK PROPERLY !!!
    ## Save current join pose to use with actuator (NOT SURE OF THIS ONE !! TEST)
    hole_pose_1 = robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_CAM * transl(0,0,-120)
    robot.setTool(Actuator)
    robot.MoveL(hole_pose_1)

    operation_force() 
    connect_DK()
    speed()
    robot.MoveL(hole_pose_1) ## check which axis to move back
    robot.setTool(Camera)

    # 2nd hole
    robot.MoveL(pose_2 * transl(0,0,-120))
    operation_correct()
    connect_DK() 
    speed() 

    ## Save current join pose to use with actuator
    hole_pose_2 = robot.SolveFK(robot.Joints())  * ENDEFFECTOR_TO_CAM * transl(0,0,-120)
    setTool(Actuator)
    robot.MoveL(hole_pose_2) 
    operation_force() 
    connect_DK() 
    speed() 
    robot.MoveL(hole_pose_2)

    # Trash the q-tip
    robot.MoveL(trash * transl(0,0,-120))
    robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,0,20))
    # testing("forward")
    # testing("start")
    robot.MoveL(robot.SolveFK(robot.Joints()) * ENDEFFECTOR_TO_ACT * transl(0,0,-30))
    robot.MoveL(robot_joints)


# get_paint()

painting(pose1, pose2)
# painting(pose3, pose4)

# get time taken to run the for loop code 
elapsed_time_fl = (time.time() - start)
print ("Total Computation Time: ",elapsed_time_fl, "Seconds")
