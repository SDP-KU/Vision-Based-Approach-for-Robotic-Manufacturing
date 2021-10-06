#!/usr/bin/env python

 # Ros libraries
from numpy.lib.function_base import vectorize
import roslib
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
# numpy and scipy
import numpy as np
from scipy.ndimage import filters
# OpenCV
import cv2 
from cv2 import aruco
from cv_bridge import CvBridge
# Python libs
import sys, time, math
import math
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge

# Ros Messages
from sensor_msgs.msg import CompressedImage

bridge = CvBridge()

#Intrensic Parameters
Cameramatrix=np.array([[611.301, 0, 320.367],[0, 611.506, 246.129],[0,0,1]])
#Find RGB 

# ArUco Detection Code
# Start streaming

#---Parametrs 
markersize= 9 #[cm]
#---camera 
cameradist=np.array([0.,    0.,   0.,   0.,    0.])

#---Aruco Dictionary
arucodic=aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
parameters=aruco.DetectorParameters_create()
#  parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
parameters.cornerRefinementWinSize = 5
parameters.cornerRefinementMinAccuracy = 0.001
parameters.cornerRefinementMaxIterations = 5

# publishing the pose
pub = rospy.Publisher('pose', Pose, queue_size=1)

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    color_image= bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    #--Convert to gray scale
    gray=cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY) # put the saved pic
    #---Find aruco markers
    corners, ids, rejected=  aruco.detectMarkers(image=gray, dictionary=arucodic, parameters=parameters)
    if ids != None :
        # the following line is not right 
        ret= aruco.estimatePoseSingleMarkers(corners,markersize, Cameramatrix, cameradist)
        #---unpack output
        rvec = ret[0][0][0]
        tvec = ret[1][0][0]
    
        # Convert it back to vector
        tvec = np.multiply(tvec, 10) # cm 2 mm
        rotation = R.from_rotvec(rvec)
        print(rvec)
        rotation_M=rotation.as_dcm()
        R0=np.array([[1.,0.,0.],[0.,-1.,0.],[0.,0.,-1.]])
        R1_M=np.matmul(R0, rotation_M)
        print("A")
        print(rotation_M)
        print(R0)
        print(R1_M)
        R1=R.from_dcm(R1_M)
        quat = R1.as_quat()
        print(quat)

        p = Pose()
        p.position.x = tvec[0]
        p.position.y = tvec[1]
        p.position.z = tvec[2]
        # Make sure the quaternion is valid and normalized
        p.orientation.x = quat[0]
        p.orientation.y = quat[1]
        p.orientation.z = quat[2]
        p.orientation.w = quat[3]
        pub.publish(p)

def PicToPose():
        rospy.init_node('PicToPose', anonymous=True)
        rospy.Subscriber("/camera/color/image_raw",Image, callback)
        
        rospy.spin()


if __name__ == '__main__':
    try:
        PicToPose()
    except rospy.ROSInterruptException:
        pass                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
