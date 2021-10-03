import numpy as np
import sys, time, math, keyboard, math, cv2
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
import cv2.aruco as aruco

def ArUco():
    start = time.time() # get starting time

    pipeline, config = rs.pipeline(), rs.config()
    pipeline_profile = config.resolve(rs.pipeline_wrapper(pipeline))
    device = pipeline_profile.get_device()
    color_profile = rs.video_stream_profile(pipeline_profile.get_stream(rs.stream.color))
    color_intrinsics = color_profile.get_intrinsics()
    ## USE the values from the camera to construct the camera matrix and to set the resolution of the frame
    w, h = color_intrinsics.width, color_intrinsics.height 
    fx, fy = color_intrinsics.fx, color_intrinsics.fy
    ppx, ppy = color_intrinsics.ppx, color_intrinsics.ppy

    found_rgb = False   #Intial Value
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    Cameramatrix=np.array([[fx, 0, ppx],[0, fy, ppy],[0,0,1]])
    print (Cameramatrix)
    input('enter')
    
    cameradist=np.array([0.,    0.,   0.,   0.,    0.])
    arucodic = aruco.Dictionary_get(aruco.DICT_6X6_1000)
    parameters = aruco.DetectorParameters_create()
    markersize = 9 # [cm]
    pipeline.start(config)
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame, color_frame = frames.get_depth_frame(), frames.get_color_frame()
        if not depth_frame or not color_frame: continue
        color_image = np.asanyarray(color_frame.get_data())
        h1, w1, _ = np.shape(color_image)
        if w1 != w & h1 != h: ## NOT SURE IF THIS WORKS !!!
            config.enable_stream(rs.stream.color, w1, h1, rs.format.rgb8, 30)

        gray=cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        corners, ids, __ = aruco.detectMarkers(image=gray, dictionary=arucodic, parameters=parameters)
        if np.all(ids != None) :
            ret = aruco.estimatePoseSingleMarkers(corners,markersize, Cameramatrix, cameradist)
            rvec, tvec = ret[0][0][0], np.multiply(ret[1][0][0], 10)
            aruco_pos = np.concatenate((tvec,rvec), axis=0)
            
            print (rvec)
            print (tvec)

            cv2.imshow("test", cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
            cv2.waitKey(1)
            cv2.destroyAllWindows()
            break
        else:
            cv2.imshow("test", cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
            cv2.waitKey(1)
            cv2.destroyAllWindows()
    pipeline.stop()

    # get time taken to run the for loop code 
    elapsed_time_fl = (time.time() - start)
    print (elapsed_time_fl)

    return (aruco_pos)

if __name__ == "__main__":
    ArUco()
