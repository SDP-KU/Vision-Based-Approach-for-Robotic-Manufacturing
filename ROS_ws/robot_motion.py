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

## First initialize `moveit_commander`_ and a `rospy`_ node:
moveit_commander.roscpp_initialize(sys.argv)
#rospy.init_node('move_group_python_interface', anonymous=True)

    
robot = moveit_commander.RobotCommander()


scene = moveit_commander.PlanningSceneInterface()

group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

# reference frame for this robot:
#move_group.set_planning_frame('ur10_base_link')
planning_frame = 'ur10_base_link' #move_group.get_planning_frame()
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
    #get postion of end effctor from base
    p = geometry_msgs.msg.Pose()
    p.position.x = tvec[0]
    p.position.y = tvec[1]
    p.position.z = tvec[2]
    # Make sure the quaternion is valid and normalized
    p.orientation.x = rvec[0]
    p.orientation.y = rvec[1]
    p.orientation.z = rvec[2]
    p.orientation.w = rvec[3]
   
    return(M,tvec)
# camera -> end-effector matrix
def cam_M():
  # Trans=[[-0.99977665, 0.01048481,0.01834987, 0.03690504],
  # [-0.01830651, 0.00421686, -0.99982353, -0.13537845],
  # [-0.01056033, -0.99993614,-0.00402398,0.20485623], [0,0,0 ,1]]

  Trans=[[-0.7071, 0, -0.7071, -0.07],
  [-0.7071, 0, 0.7071, 0.07],
  [0., 1., 0., 0.], [0,0,0 ,1]]
  M_cam,tvec_c=TransBack(Trans)
  return(M_cam,tvec_c)

# Transformation Matrix generator
def trans(M,T):
  trans=[[M[0][0],M[0][1],M[0][2],T[0]],[M[1][0],M[1][1],M[1][2],T[1]],[M[2][0],M[2][1],M[2][2],T[2]],[0,0,0,1]]
  return(trans)

# pose matrix 
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
  # trans2=[[-0.99977665, 0.01048481,0.01834987, 0.03690504],
  # [-0.01830651, 0.00421686, -0.99982353, -0.13537845],
  # [-0.01056033, -0.99993614,-0.00402398,0.20485623], [0,0,0 ,1]]
  trans2=[[-0.7071, 0, -0.7071, -0.07],
  [-0.7071, 0, 0.7071, 0.07],
  [0., 1., 0., 0.], [0,0,0 ,1]]
  T3=[T3[0]/1000,T3[1]/1000,T3[2]/1000]
  trans3=trans(R3,T3)
  print "end effector-base \n"
  print(trans1)
  print "\ncamera- end effector \n"
  print(trans2)
  print "\naruco -camera \n"
  print(trans3)
  print "\ncamera - base\n"
  mlt=np.matmul(trans1,trans2)
  print(mlt)
  aruco_base=np.matmul(np.matmul(trans1,trans2),trans3)
  print "\naruco -base \n"
  print(aruco_base)
  return(aruco_base)

# camera -> base matrix
def cam_base(p):
  # R1 & T1 are base to endeffctor (from Tf_M)
  R1,T1= tf_M ()
  R2,T2= cam_M()
 
  trans1=trans(R1,T1)
  trans2=trans(R2,T2)

  cam_base=np.matmul(trans1,trans2)
  return(cam_base)

# transformation Back generator 
def TransBack(M):
  tvec=[M[0][3],M[1][3],M[2][3]]
  matrix=[[M[0][0],M[0][1],M[0][2]],
          [M[1][0],M[1][1],M[1][2]],
          [M[2][0],M[2][1],M[2][2]]]
  return(matrix,tvec)

def start_pose():
  # R1 & T1 are base to endeffctor
  R1,T1= tf_M ()
  trans1=trans(R1,T1)
  r=R.from_dcm(R1) 
  quat=r.as_quat()

  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.orientation.x = quat[0]
  pose_goal.orientation.y = quat[1]
  pose_goal.orientation.z = quat[2] 
  pose_goal.orientation.w = quat[3]
  pose_goal.position.x = T1[0]
  pose_goal.position.y = T1[1]
  pose_goal.position.z = T1[2]
  move_group.set_pose_target(pose_goal)
  plan = move_group.go(wait=True)
  move_group.stop()
  move_group.clear_pose_targets()

  current_pose = move_group.get_current_pose().pose
  return all_close(pose_goal, current_pose, 0.01)

def go_to_pose_goal(Pose,x,y,z):
  R_ba,T_ba=TransBack(Pose)
  R_ec,T_ec=cam_M()
  trans_ec=trans(R_ec,T_ec)
  print(trans_ec)
  print(R_ec)


  vec=[float(x)/1000,float(y)/1000,float(z)/1000]
  P_bt=np.matmul(vec,R_ba)+T_ba
  trans_bt=trans(R_ba,P_bt)
  print(trans_bt)
  inv=np.linalg.inv(trans_ec)
  print(inv)
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


def main(pose1):
  print " "
  print "---------------------------------------------------------- \n"
  print "Welcome to the robot movement using Python by Bushra \n"
  print "---------------------------------------------------------- \n"
  print "============Press `Enter` to begin the motion ..."
  raw_input()
  start_pose()
  p=aruco_base(pose1)
  print("base_aruco")
  print(p)
  print "============ Press `Enter` to execute a movement using a pose goal ..."
  raw_input()
  go_to_pose_goal(p,0, 0, -100)
  print "next pose"
  raw_input()
  go_to_pose_goal(p, 70.0, 30.0, -100)
  print "next pose"
  raw_input()
  go_to_pose_goal(p,115.0, 30.0, -100)
  print "next pose"
  raw_input()
  go_to_pose_goal(p, 160.0, 30.0, -100)
  print "next pose"
  raw_input()
  go_to_pose_goal(p,205.0, 30.0, -100)
  print "next pose"
  raw_input()
  go_to_pose_goal(p,205.0, 65, -100)

  print "============ demo complete!"
 

def pose():
  rospy.init_node('PicToPose', anonymous=True)
  rospy.Subscriber("pose",Pose, main)
  rospy.spin()
        
        


if __name__ == '__main__':
    try:
        pose()
    except rospy.ROSInterruptException:
        pass
