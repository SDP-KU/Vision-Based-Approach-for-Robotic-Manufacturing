#!/usr/bin/env python  
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
import roslib
import tf
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import scipy.spatial.transform 
from geometry_msgs.msg import PointStamped


## First initialize `moveit_commander`_ and a `rospy`_ node:
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

# reference frame for this robot:
planning_frame = 'ur10_base_link' 
print(planning_frame)
# end-effector link for this group:
move_group.set_end_effector_link('tool0')
eef_link = move_group.get_end_effector_link()
print(eef_link) 

# list of all the groups in the robot:
group_names = robot.get_group_names()

# state of the robot:
robot.get_current_state()
# Misc variables
robot = robot
scene = scene
move_group = move_group
eef_link = eef_link
group_names = group_names


# close all function
def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

# robot transformation matrix (base to end-effector) 
def tf_M():
  rospy.sleep(1)
  listener = tf.TransformListener()
  # get the tran vec and rot quat from tf listener  
  while not rospy.is_shutdown():
    try:
      (tvec,rvec) = listener.lookupTransform('/ur10_base_link', '/tool0', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue

    # Convert it back to vector
    rotation = R.from_quat(rvec)
    # rotation matrix from base to end effctor
    M = rotation.as_dcm()
    
    return(M,tvec)

# camera -> end-effector matrix
def cam_M():
  Trans=[[0.9998, 0.0199, -0.0057,  -0.0420],
 [0.0061 , -0.0155  ,  0.9999 , 0.1391],
 [0.0198 , -0.9997 ,-0.0156 ,0.2078],
 [ 0.      ,    0.      ,    0.     ,     1.   ,    ]]
 #  Trans=[[-0.92544921, -0.00239401, -0.37886412,  0.00173008],
 #  [-0.37886939 , 0.002364  ,  0.92544714 , 0.08117633],
 #  [-0.00131989 , 0.99999434 ,-0.00309478 ,-0.01618667],
 #  [ 0.      ,    0.      ,    0.     ,     1.       ]]

  M_cam,tvec_c=TransBack(Trans)
  return(M_cam,tvec_c)

# tool -> cam matrix
def tool_M():
  tool = np.array([[0.9995,0,0.0331,0.0000565],
                    [-0.0331,-0.0043,0.9994,0.1925735],
                    [0.0001 ,-1.,-0.0043 , 0.1173004],
                    [0.  ,0.  ,0.  ,1.  ]])
  cam=[[0.9998, 0.0199, -0.0057,  -0.0420],
  [0.0061 , -0.0155  ,  0.9999 , 0.1391],
  [0.0198 , -0.9997 ,-0.0156 ,0.2078],
  [ 0.      ,    0.      ,    0.     ,     1.   ,    ]]
  t=np.matmul(np.linalg.inv(cam),tool)
  return (t)
  

# Transformation Matrix generator
def trans(M,T):
  trans=np.array([[M[0][0],M[0][1],M[0][2],T[0]],[M[1][0],M[1][1],M[1][2],T[1]],[M[2][0],M[2][1],M[2][2],T[2]],[0,0,0,1]])
  return(trans)

# transformation Back generator 
def TransBack(M):
  tvec=[M[0][3],M[1][3],M[2][3]]
  matrix=[[M[0][0],M[0][1],M[0][2]],
          [M[1][0],M[1][1],M[1][2]],
          [M[2][0],M[2][1],M[2][2]]]
  return(matrix,tvec)

# aruco pose matrix 
def Pose_M(pose):
  tvec=[pose.position.x,pose.position.y,pose.position.z]
  quat =[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
  rotation = R.from_quat(quat)
  Rp = rotation.as_dcm()
 
  return(Rp,tvec)

# aruco -> base matrix
def aruco_base(p):
  R1,T1= tf_M ()
  R2,T2= cam_M()
  R3,T3=Pose_M(p)
  trans1=trans(R1,T1)
  trans2=trans(R2,T2)
  T3=[T3[0]/1000,T3[1]/1000,T3[2]/1000]
  trans3=trans(R3,T3)
  # print "end effector-base \n"
  # print(trans1)
  # print "\ncamera- end effector \n"
  # print (trans2)
  # print "(\naruco -camera \n"
  # print(trans3)
  # print "\ncamera - base\n"
  # mlt=np.matmul(trans1,trans2)
  # print(mlt)
  aruco_base=np.matmul(np.matmul(trans1,trans2),trans3)
  print "\naruco -base \n"
  print(aruco_base)
  return(aruco_base)

# tool -> base matrix
def tool_base():
  R1,T1= tf_M ()
  R2,T2= cam_M()
  trans3=tool_M()
  trans1=trans(R1,T1)
  trans2=trans(R2,T2)
  tool_base=np.matmul(np.matmul(trans1,trans2),trans3)
  return(tool_base)

# camera -> base matrix
def cam_base(p):
  # R1 & T1 are base to endeffctor (from Tf_M)
  R1,T1= tf_M ()
  R2,T2= cam_M()
  I=np.identity(3)
  trans1=trans(R1,T1)
  trans2=trans(R2,T2)
  trans3=trans(I,p)
  cam_base=np.matmul(np.matmul(trans1,trans2),trans3)
  return(cam_base)

# get the hole transformation matrix 
hole_trans=np.array([])
def cam_hole (data):
  x=data.point.x
  y=data.point.y
  z=data.point.z
  Vec=np.array([x,y,z])
  hole_pose=cam_base(Vec)
  global hole_trans
  hole_trans=hole_pose


def go_to_pose_goal(Pose,x,y,z):
  R_ba,T_ba=TransBack(Pose)
  R_ec,T_ec=cam_M()
  trans_ec=trans(R_ec,T_ec)

  vec=[float(x)/1000,float(y)/1000,float(z)/1000]
  P_bt=np.matmul(vec,R_ba)+T_ba
  trans_bt=trans(R_ba,P_bt)
  transf=np.matmul(trans_bt,np.linalg.inv(trans_ec))
  print(transf)
  Rf,Tf=TransBack(transf)
  r=R.from_dcm(Rf)
  quat=r.as_quat()

  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.orientation.x = quat[0]
  pose_goal.orientation.y = quat[1]
  pose_goal.orientation.z = quat[2]
  pose_goal.orientation.w = quat[3]
  pose_goal.position.x = Tf[0]
  pose_goal.position.y = Tf[1]
  pose_goal.position.z = Tf[2]
      
  move_group.set_pose_target(pose_goal)

  plan = move_group.go(wait=True)
  move_group.stop()
  move_group.clear_pose_targets()

  current_pose = move_group.get_current_pose().pose
  return all_close(pose_goal, current_pose, 0.01)


def main():
  rospy.init_node('move_group', anonymous=True) 
  print " "
  print "---------------------------------------------------------- \n"
  print "Welcome to the robot movement using Python by Bushra \n"
  print "---------------------------------------------------------- \n"
  print "============Press `Enter` to begin the motion ..."
  raw_input()
  print "============ Press `Enter` to execute a movement using a pose goal ..."
  raw_input()
  #get the aruco pose message
  pose=rospy.wait_for_message("pose",Pose)
  v=[pose.position.x,pose.position.y,pose.position.z]
  # keep going to center until the error is less than 0.5 mm
  while abs(v[0])>0.5 or abs(v[1])>0.5 :
    p=aruco_base(pose)
    go_to_pose_goal(p,0, 0, -110)
    pose=rospy.wait_for_message("pose",Pose)
    v=[pose.position.x,pose.position.y,pose.position.z]
  print "next pose"
  raw_input()
  i=0
  a=0
  # go to the 4 holes in the lift of the aruco
  while i<4:
    go_to_pose_goal(p,-72+a,34.0,-110)
    hole_pos = rospy.wait_for_message("hole_center", PointStamped)
    x=hole_pos.point.x
    y=hole_pos.point.y
    z=hole_pos.point.z
    Vec=np.array([x,y,z])
    hole_pose=cam_base(Vec)
    go_to_pose_goal(hole_pose,0, 0, 0)
    a=a-50
    print "a:"
    print(a)
    i=i+1
    print "next pose"
    raw_input()

  print "============ demo complete!"
 

def pose():
  rospy.init_node('PicToPose', anonymous=True)
  #rospy.Subscriber("hole_center",PointStamped, cam_hole)
 # rospy.Subscriber("pose",Pose, main)
  rospy.spin()
        
        
if __name__ == '__main__':
    try:
        # main()
        tool_M()
    except rospy.ROSInterruptException:
        pass
