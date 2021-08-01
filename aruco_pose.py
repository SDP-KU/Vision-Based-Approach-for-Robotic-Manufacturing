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
import aruco as aruco
from cv_bridge import CvBridge
# Python libs
import sys, time, math
import math
from scipy.spatial.transform import Rotation as R

# Ros Messages
from sensor_msgs.msg import CompressedImage

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def PicToPose():
        rospy.Subscriber("/camera/color/image_raw",Image)
        #Intrensic Parameters
        Cameramatrix=np.array([[611.301, 0, 320.367],[0, 611.506, 246.129],[0,0,1]])
        #Find RGB 
    
        # ArUco Detection Code
        # Start streaming
       
        #---Parametrs 
        markersize= 9 #[cm]
        #---camera 
        cameradist=np.array([0.,    0.,   0.,   0.,    0.])
        while True:
            #---Aruco Dictionary
            arucodic=aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
            parameters=aruco.DetectorParameters_create()
          #  parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            parameters.cornerRefinementWinSize = 5
            parameters.cornerRefinementMinAccuracy = 0.001
            parameters.cornerRefinementMaxIterations = 5
            color_image= cv2.imread('/camera/color/image_raw')
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
                # Convert Vector r and (180,0,0) to Rotation Matrix
                rvec_mat = R.from_rotvec(rvec)
                rot_mat = R.from_rotvec([3.14,0,0])
                rvec_mat = rvec_mat.as_matrix()
                rot_mat = rot_mat.as_matrix()
                # Add the Rotation Matrices
                rotation_matrix_final = np.multiply(rvec_mat, rot_mat)
                # Convert it back to vector
                r = R.from_matrix(rotation_matrix_final)
                rvec = r.as_euler('zyx', degrees=True)
                tvec = np.multiply(tvec, 10) # cm 2 mm
                aruco_pos = np.concatenate((tvec,rvec), axis=0)



                # print (rvec)
                # print (tvec)
                cv2.imshow("test", cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
                # imwrite
                cv2.waitKey(1)
                break
            else:
                cv2.imshow("test", cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
                cv2.waitKey(1)

                # publishing the pose
                pub = rospy.Publisher('pose', Pose, queue_size=1)
                pub.publish(aruco_pos)
                rospy.init_node('PicToPose', anonymous=True)
        return (aruco_pos)



if __name__ == '__main__':
    try:
        PicToPose()
    except rospy.ROSInterruptException:
        pass                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
