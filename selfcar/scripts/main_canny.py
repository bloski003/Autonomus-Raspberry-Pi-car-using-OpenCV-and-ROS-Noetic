#! /usr/bin/env python3

import cv2  #import OpenCV
import numpy as np  #import numpy for arrays
import rospy		#ros
from sensor_msgs.msg import Image		#import ROS image msg type for camera input
from sensor_msgs.msg import Range		#import ROS Range msg type for sonar
from geometry_msgs.msg import Twist		#import ROS Twist msg type cmd_vel message
from std_msgs.msg import Int8  #for traffic lights
from cv_bridge import CvBridge , CvBridgeError		#import CV_Bridge to transform image from ros to Opencv 
import sys
from utils import *  #Import my utlis with the used functions

# https://github.com/SravanChittupalli/Lane-following-bot-in-Gazebo

bridge = CvBridge()
prop = 0
diff = 0
integral = 0
kP = 0.0009		#PID controller for steering
kD = 0.018
kI = 0.0000009
integralActivatingZone = 10
last_error = 0
pub = rospy.Publisher("/cmd_vel" , Twist, queue_size = 5)  #publish cmd_vel topic
tot_error = 0
sonar_flag = 0   #flag if object detected 10 cm from robot
lights_flag = 0

def give_steering_angle(error):
				global kP , kD , kI , last_error , integralActivatingZone , prop , diff , integral , pub , tot_error

				motor_msg = Twist()  #create Twist msg
    
				if (sonar_flag == 1 or lights_flag == 1):  #object 10cm or red/yellow light stop robot
						motor_msg.linear.x = 0
						motor_msg.linear.y = 0
						motor_msg.linear.z = 0

						motor_msg.angular.x = 0
						motor_msg.angular.y = 0
						motor_msg.angular.z = 0
				
				else:
						if( error < integralActivatingZone and error != 0):
								tot_error += error
						else:
								tot_error = 0
						
						if error == 0:
								diff = 0
								prop = 0
						

						prop += error                 *kP
						diff += (error - last_error)  *kD
						integral += tot_error         *kI

						last_error = error

						steering_val_to_motors = prop+diff+integral
						#print(steering_val_to_motors)
						if (steering_val_to_motors) > 3:
							steering_val_to_motors = 3
						elif (steering_val_to_motors) < -3:
							steering_val_to_motors  -3
						else:
							steering_val_to_motors = steering_val_to_motors
							
						print(steering_val_to_motors)
						motor_msg.linear.x = 1
						motor_msg.linear.y = 0
						motor_msg.linear.z = 0

						motor_msg.angular.x = 0
						motor_msg.angular.y = 0
						motor_msg.angular.z = -steering_val_to_motors		#copy steering value to cmd_vel msg

				pub.publish(motor_msg)  #publish msg
				rospy.sleep(0.005)

def image_callback(ros_image):  #calback function for received images
    global bridge 
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")  #convert Image from ROS to OpenCV as RGB
    except CvBridgeError as e:
        print(e)
    
    gray = cv2.cvtColor(cv_image , cv2.COLOR_BGR2GRAY)		#convert image to gray scale
    canny = apply_canny(gray)		#apply canny edge detector

    white_edge = detect_white(cv_image)		#find white edges
    fin_image = cv2.bitwise_or(white_edge , canny)

    ROI = get_reqd_region(fin_image)  #get Region of Interest, the image portion next to robot
    
    lines = cv2.HoughLinesP(ROI , 1, np.pi/180 , 50 , np.array([]) , minLineLength=10 , maxLineGap=70)  #Hough Transform
    #print(lines)
    averaged_lines = average_slope_intercept(cv_image , lines)		#find the average of slope to keep desired lines
    line_image = display_lines(cv_image , averaged_lines)		#display found lines
    combo_image = cv2.addWeighted(cv_image , 0.8 , line_image , 1 , 1)		#combine original image with the lines detected
    
    if len(averaged_lines) > 0:
        steering_error = compute_steering_angles(cv_image , averaged_lines)		#find steering angle from detected lines
        correction_image = show_steering_corection(combo_image , steering_error)
        #Draw the line which shows how much the bot is deviated from current heading 
        combo_image = cv2.addWeighted(combo_image , 0.9 , correction_image , 1 , 1)
        give_steering_angle(steering_error)
    
    #My heading
    cv2.line(combo_image , (600 , cv_image.shape[0]) , (600 , cv_image.shape[0]-300) , (0 , 255 , 0) , 5 ) #draw lines in OG img

    Stacked_imgs = get_imgs_stacked(0.7, ([cv_image, combo_image, ROI],[canny, white_edge, line_image])) #show omg in one window
    cv2.imshow("Output Images" , Stacked_imgs)  #display images window
    cv2.waitKey(3)
    
def range_callback(msg):  #calback function for ultrasonic sensor
		range1 = msg.range		#get the range from ROS Range msg type
		#print(range1)
		global sonar_flag
		if (range1 <= 10.0):		#if range smaller than 10cm set flag
				sonar_flag = 1
		else:
				sonar_flag = 0			#no object nearby
				
def lights_callback(msg):  #calback function for ultrasonic sensor
		lights = msg.data		#get the range from ROS Range msg type
		#print(range1)
		global lights_flag
		if (lights == 0):		#if red or yellow set flag
				lights_flag = 0	#no light
		elif (lights == 1): #red light
				lights_flag = 1
		elif (lights == 2): #yellow light
				lights_flag = 1		
		elif (lights == 3): #green light
				lights_flag = 0
		else:
				lights_flag = 0

def main():
    rospy.init_node("motor_control")		#initialize node motor control
    img_sub = rospy.Subscriber("/webcam" , Image , image_callback)	#subscribe RaspiCam Images topic
    sonar_sub = rospy.Subscriber("/ultrasound" , Range , range_callback)		#subscribe ultrasonic topic
    lights_sub = rospy.Subscriber("/traffic_lights" , Int8 , lights_callback)		#subscribe traffic_lights topic

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('ShutDown!...')

    cv2.destroyAllWindows()

if __name__=='__main__':
    main()
