#!/usr/bin/python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Img_sub:
	def __init__(self):
		image_topic = "/webcam"
		rospy.Subscriber(image_topic, Image, self.image_callback)
		self.bridge = CvBridge()
	
	def image_callback(self, msg):
		try:
			cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
			gray_img = self.get_gray_img(cv2_img)
			blurred_img = self.get_blur_img(gray_img)
			edges = self.get_Canny_img(blurred_img)
			cv2.imshow("RGB Image", cv2_img)
			cv2.waitKey(3)
			#cv2.imshow("Canny Edgess Image", edges)
		except CvBridgeError as e:
			print(e)
			return
		#gray_img = self.get_gray_img(cv2_img)
		#blurred_img = self.get_blur_img(gray_img)
		#edges = self.get_Canny_img(blurred_img)
		#cv2.imshow("Canny Edges Image", edges)
      		
	def get_gray_img(self, rgb_img):
		gray_scale_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2GRAY)
		return gray_scale_img
    
	def get_blur_img(self, gray_img):
		blurred_img = cv2.GaussianBlur(gray_img, (5,5), 0)
		return blurred_img
    	
	def get_Canny_img(self, blurred_img):
		low_threshold = 50
		high_threshold = 150
		edges = cv2.Canny(blurred_img, low_threshold, high_threshold)
		cv2.imshow("Canny Edges Image", edges)
		cv2.waitKey(3)
		return edges


if __name__ == '__main__':
	try:
		rospy.init_node('image_listener')
		sub_obj = Img_sub()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Node terminated")



