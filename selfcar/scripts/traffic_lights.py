#! /usr/bin/env python3

import cv2  #import OpenCV
import numpy as np  #import numpy for arrays
import rospy	#ros
from sensor_msgs.msg import Image	#import ROS image msg type for camera input
from std_msgs.msg import Int8
from cv_bridge import CvBridge , CvBridgeError	#import CV_Bridge to transform image from ros to Opencv

#https://stackoverflow.com/questions/66589945/opencv-traffic-light-detection
#https://github.com/HevLfreis/TrafficLight-Detector/blob/master/src/main.py

bridge = CvBridge()
pub = rospy.Publisher("/traffic_lights" , Int8, queue_size = 5)  #publish lights topic

def image_callback(ros_image):  #calback function for received images
    global bridge 
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")  #convert Image from ROS to OpenCV as RGB
    except CvBridgeError as e:
        print(e)
    
    light_msg = Int8()
    
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0,100,100])
    upper_red1 = np.array([10,255,255])
    lower_red2 = np.array([160,100,100])
    upper_red2 = np.array([180,255,255])
    lower_green = np.array([40,50,50])
    upper_green = np.array([90,255,255])
    lower_yellow = np.array([15,150,150])
    upper_yellow = np.array([35,255,255])
    
    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    maskg = cv2.inRange(hsv_image, lower_green, upper_green)
    masky = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    maskr = cv2.add(mask1, mask2)
    
    size = cv_image.shape
    r_circles = cv2.HoughCircles(maskr, cv2.HOUGH_GRADIENT, 1, 80, param1=50, param2=10, minRadius=20, maxRadius=30)
    g_circles = cv2.HoughCircles(maskg, cv2.HOUGH_GRADIENT, 1, 60, param1=50, param2=10, minRadius=20, maxRadius=30)
    y_circles = cv2.HoughCircles(masky, cv2.HOUGH_GRADIENT, 1, 30, param1=50, param2=5, minRadius=20, maxRadius=30)
    
    res_red = cv2.bitwise_and(cv_image, cv_image, mask=maskr)
    blur_red_img = cv2.medianBlur(res_red, 5)
    ccimg_red = cv2.cvtColor(blur_red_img, cv2.COLOR_HSV2BGR)
    cimg_red = cv2.cvtColor(ccimg_red, cv2.COLOR_BGR2GRAY)
    
    res_yell = cv2.bitwise_and(cv_image, cv_image, mask=masky)
    blur_yell_img = cv2.medianBlur(res_yell, 5)
    ccimg_yell = cv2.cvtColor(blur_yell_img, cv2.COLOR_HSV2BGR)
    cimg_yell = cv2.cvtColor(ccimg_yell, cv2.COLOR_BGR2GRAY)
    
    res_green = cv2.bitwise_and(cv_image, cv_image, mask=maskg)
    blur_green_img = cv2.medianBlur(res_green, 5)
    ccimg_green = cv2.cvtColor(blur_green_img, cv2.COLOR_HSV2BGR)
    cimg_green = cv2.cvtColor(ccimg_green, cv2.COLOR_BGR2GRAY)
    
    if r_circles is not None:
            print("Red Ligth Found")
            r_circles = np.uint16(np.around(r_circles))
            for i in r_circles[0, :]:
                cv2.circle(cv_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(cv_image, (i[0], i[1]), 2, (0, 0, 255), 3)
            #cv2.imshow('Detected Red Circle', cimg_red)
            light_msg = 1
            cv2.imshow('Original Image', cv_image)
            cv2.waitKey(3)
            
    elif y_circles is not None:
						print("Yellow Ligth Found")
						y_circles = np.uint16(np.around(y_circles))
						for i in y_circles[0, :]:
								cv2.circle(cv_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
								cv2.circle(cv_image, (i[0], i[1]), 2, (0, 0, 255), 3)
						#cv2.imshow('Detected Yellow Circle', cimg_yell)
						light_msg = 2
						cv2.imshow('Original Image', cv_image)
						cv2.waitKey(3)
						
    elif g_circles is not None:
						print("Green Ligth Found")
						g_circles = np.uint16(np.around(g_circles))
						for i in g_circles[0, :]:
								cv2.circle(cv_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
								cv2.circle(cv_image, (i[0], i[1]), 2, (0, 0, 255), 3)
						#cv2.imshow('Detected Green Circle', cimg_green)
						light_msg = 3
						cv2.imshow('Original Image', cv_image)
						cv2.waitKey(3)
						
    else:
						print("No Ligths Found")	
						light_msg = 0
		
    pub.publish(light_msg)
    print(light_msg)



def main():
    rospy.init_node("traffic_lights_detect")		#initialize node motor control
    img_sub = rospy.Subscriber("/webcam" , Image , image_callback)	#subscribe RaspiCam Images topic

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('ShutDown!...')

    cv2.destroyAllWindows()

if __name__=='__main__':
    main()
