import sys
import cv2 as cv
import numpy as np
def main(data):
    
    gray = cv.cvtColor(data, cv.COLOR_BGR2GRAY)
    
    
    gray = cv.medianBlur(gray, 5)
    
    
    rows = gray.shape[0]
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 8,
                               param1=100, param2=30,
                               minRadius=1, maxRadius=30)
    
    px_list=[]
    py_list=[]
    center_x=0
    center_y=0
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv.circle(data, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv.circle(data, center, radius, (255, 0, 255), 3)
            px_list.append(i[0])
            py_list.append(i[1])
        
        cv.imshow("detected circles", data)
        cv.waitKey(0)
        px=320.36712646484375
        py=246.1293182373047
        p = math.sqrt((px*px)+(py*py))
        b=300

        for x,y in zip(px_list,py_list):
            xy=math.sqrt((x*x)+(y*y))
            if abs(xy-p)<=b:
                center_x=x
                center_y=y
                b=abs(xy-p)
    print("Closest Circle is:", center_x, center_y)

    delta_x=abs(px-center_x)*0.2/611.3007202148438
    delta_y=abs(py-center_y)*0.2/611.50634765625

    print("Next pose is :", delta_x, delta_y)

    
    
    return (delta_x,delta_y)
if __name__ == "__main__":
    main(sys.argv[1:])