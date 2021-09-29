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



def tf_M():
  #rospy.init_node('listener')
    rospy.sleep(1)
    listener = tf.TransformListener()
  # get the tran vec and rot quat from tf listener 
  
    (tvec,rvec) = listener.lookupTransform('/ur10_base_link', '/tool0', rospy.Time(0))
  
  


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

def cam_M():
  tvec_c=np.array([-0.075,-0.1168,0.2231])
  rvec_c=np.array([-1.5721943,-0.0050954,2.38062460])
  rotation = R.from_rotvec(rvec_c)
  M_cam = rotation.as_dcm()
  return(M_cam,tvec_c)

def trans(M,T):
  trans=[[M[0][0],M[0][1],M[0][2],T[0]],[M[1][0],M[1][1],M[1][2],T[1]],[M[2][0],M[2][1],M[2][2],T[2]],[0,0,0,1]]
  return(trans)

def Pose_M(pose):
  tvec=[pose.position.y,pose.position.z,pose.position.x]
  quat =[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
  rotation = R.from_quat(quat)
  Rp = rotation.as_dcm()
  M=[[Rp[0][0],Rp [0][1],Rp[0][2]],
  [Rp[2][0],Rp [2][1],Rp[2][2]],
  [Rp[1][0],Rp [1][1],Rp[1][2]]]
  return(M,tvec)

def cam_base(p):
  # R1 & T1 are base to endeffctor
  R1=[[ 0.99990384 , 0.01049366 ,-0.00906592],
 [-0.0117748 ,  0.98779455, -0.15531674],
 [ 0.00732542 , 0.15540856 , 0.98782312]]
  T1= [0.021463172010453002, -0.5217860586364362, 0.4222618857956175]
  R2,T2= cam_M()
  pR,pT= pose_Form(p)
  trans1=trans(R1,T1)
  trans2=trans(R2,T2)
  trans3=trans(pR,pT)
  cam_base=np.matmul(np.matmul(trans1,trans2),trans3)

  return(cam_base)

def aruco_base(p):
  # R1 & T1 are base to endeffctor
  R1=[[ 0.99990384 , 0.01049366 ,-0.00906592],
 [-0.0117748 ,  0.98779455, -0.15531674],
 [ 0.00732542 , 0.15540856 , 0.98782312]]
  T1= [0.021463172010453002, -0.5217860586364362, 0.4222618857956175]
  R2,T2= cam_M()
  R3,T3=Pose_M(p)
  trans1=trans(R1,T1)
  trans2=trans(R2,T2)
  print(T3)
  T3=[-T3[0]/1000,T3[1]/1000,T3[2]/1000]
  trans3=trans(R3,T3)
  print(trans3)
  aruco_base=np.matmul(np.matmul(trans1,trans2),trans3)

  return(aruco_base)

def TransBack(M):
  tvec=[M[0][3],M[1][3],M[2][3]]
  matrix=[[M[0][0],M[0][1],M[0][2]],[M[1][0],M[1][1],M[1][2]],[M[2][0],M[2][1],M[2][2]]]

  return(matrix,tvec)



def start_pose(p):
  # R1 & T1 are base to endeffctor
  R1= [[ 0.99990384 , 0.01049366 ,-0.00906592],
 [-0.0117748 ,  0.98779455, -0.15531674],
 [ 0.00732542 , 0.15540856 , 0.98782312]]
  T1= [0.021463172010453002, -0.5217860586364362, 0.4222618857956175]
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
  

  # joint_goal =[-1.1561644713031214, -1.1738622824298304, -2.414623800908224,
  #  -1.3685420195208948, 4.795976638793945, 0.35716137290000916]

  # move_group.go(joint_goal, wait=True)
  # move_group.stop()


  # current_joints = move_group.get_current_joint_values()
  # return all_close(joint_goal, current_joints, 0.01)



def go_to_pose_goal(Pose,x,y,z):
  R_ba,T_ba =TransBack(Pose)
  R_ec,T_ec=cam_M()
  trans_ec=trans(R_ec,T_ec)

  vec=[float(y)/1000,float(x)/1000,float(z)/1000]
  P_bt=np.matmul(R_ba,vec)+T_ba
  trans_bt=trans(R_ba,P_bt)
  transf=np.matmul(trans_bt,np.linalg.inv(trans_ec))
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



def main(pose1):
  print " "
  print "---------------------------------------------------------- \n"
  print "Welcome to the robot movement using Python by Bushra \n"
  print "---------------------------------------------------------- \n"
  print "============ Press `Enter` to begin the motion"
  raw_input()
  start_pose(pose1)
  print (pose1)
  p=aruco_base(pose1)
  print "============ Press `Enter` to execute a movement using a pose goal ..."
  raw_input()
  go_to_pose_goal(p,0, 0, 300)
  print "next pose"
  raw_input()
  go_to_pose_goal(p, 70.0, 30.0,300)
  print "next pose"
  raw_input()
  go_to_pose_goal(p,115.0, 30.0, 300)
  print "next pose"
  raw_input()
  go_to_pose_goal(p, 160.0, 30.0, 300)
  print "next pose"
  raw_input()
  go_to_pose_goal(p,205.0, 30.0, 300)
  print "next pose"
  raw_input()
  go_to_pose_goal(p,205.0, 65, 300)

  print "============  demo complete!"
 

def pose():
  rospy.init_node('PicToPose', anonymous=True)
  rospy.Subscriber("pose",Pose, main)
  rospy.spin()
        
        


if __name__ == '__main__':
    try:
      pose()
    except rospy.ROSInterruptException:
      pass
