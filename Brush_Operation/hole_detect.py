import numpy as np
import time, sys, math, cv2, keyboard, os
import pyrealsense2 as rs

def HoleDetec_Manual():
    # A required callback method that goes into the trackbar function.
    def nothing(x):
        pass

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    #Intrensic Parameters
    color_profile = rs.video_stream_profile(pipeline_profile.get_stream(rs.stream.color))
    color_intrinsics = color_profile.get_intrinsics()
    w, h = color_intrinsics.width, color_intrinsics.height

    #Find RGB 
    found_rgb = False   #Intial Value

    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    # Start streaming
    pipeline.start(config)
    x_v = []
    y_v = []

    # Create a window named trackbars.
    cv2.namedWindow("Trackbars")

    # Now create 6 trackbars that will control the lower and upper range of 
    # H,S and V channels. The Arguments are like this: Name of trackbar, 
    # window name, range,callback function. For Hue the range is 0-179 and
    # for S,V its 0-255.
    cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
    cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
    cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

    while True:
        
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if  not color_frame:
            continue
        
        # Convert the BGR image to HSV image.
        frame  = np.asanyarray(color_frame.get_data())
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Get the new values of the trackbar in real time as the user changes 
        # them
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")
    
        # Set the lower and upper HSV range according to the value selected
        # by the trackbar
        lower_range = np.array([l_h, l_s, l_v])
        upper_range = np.array([u_h, u_s, u_v])
        
        # Filter the image and get the binary mask, where white represents 
        # your target color
        mask = cv2.inRange(hsv, lower_range, upper_range)
    
        # You can also visualize the real part of the target color (Optional)
        res = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Converting the binary mask to 3 channel image, this is just so 
        # we can stack it with the others
        mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # stack the mask, orginal frame and the filtered result
        stacked = np.hstack((mask_3,frame,res))
        # Show this stacked frame at 50% of the size.
        cv2.imshow('Trackbars',cv2.resize(cv2.cvtColor(stacked, cv2.COLOR_BGR2RGB),None,fx=0.3,fy=0.3))
        key = cv2.waitKey(1)
        if key == ord('h'):
            # Setup BlobDetector
            detector, params = cv2.SimpleBlobDetector_create(), cv2.SimpleBlobDetector_Params()
            params.filterByArea, params.filterByCircularity, params.filterByInertia = True, True, True
            params.minArea, params.maxArea = 150, 6000
            params.minCircularity, params.minInertiaRatio = 0.1, 0.1
            params.minDistBetweenBlobs, opacity = 100, 0.1
            detector = cv2.SimpleBlobDetector_create(params)
            x_v = []
            y_v = []
            # Create a detector with the parameters
            detector = cv2.SimpleBlobDetector_create(params)
            # Convert images to numpy arrays
            im = np.asanyarray(mask_3)
            overlay = im.copy()
            keypoints = detector.detect(im)
            if keypoints is not None:
                for k in keypoints:
                    cv2.circle(frame, (int(k.pt[0]), int(k.pt[1])), int(k.size/3), (0, 0, 255), -1)
                    cv2.line(frame, (int(k.pt[0])-20, int(k.pt[1])), (int(k.pt[0])+20, int(k.pt[1])), (0,0,0), 2)
                    cv2.line(frame, (int(k.pt[0]), int(k.pt[1])-20), (int(k.pt[0]), int(k.pt[1])+20), (0,0,0), 2)
                    x_v.append(int(k.pt[0]))
                    y_v.append(int(k.pt[1]))
                cv2.addWeighted(frame, opacity, im, 1 - opacity, 0, im)
                cv2.imshow("Output", cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                cv2.waitKey(1) & 0xff
            else:
                print(":'(")

        # If the user presses ESC then exit the program
        if key == 27:
            break
        
        # If the user presses `s` then print this array.
        if key == ord('s'):
            thearray = [[l_h,l_s,l_v],[u_h, u_s, u_v]]
            print(thearray)

    # Release the camera & destroy the windows.    
    cv2.waitKey(1) & 0xff


if __name__ == "__main__":
    HoleDetec_Manual()
