#!/usr/bin/env python3

#run oN pi
#ALSO SUBSCRIBE HERE TO ENCODER VALUES TO ADJUST???

import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist #floats 64


class CMD_VEL:
	def __init__(self):
		cmd_vel = "/cmd_vel"
		rospy.Subscriber(cmd_vel, Twist, self.cmd_vel_callback)
	
	def cmd_vel_callback(self, msg):
		LENGTH = 0.135 #m
		RADIO= 0.0334 #m
		#implement the equations for wL and wR
		wR_f = (msg.linear.x + msg.angular.z * (LENGTH/2)) / RADIO
		wL_f = (msg.linear.x - msg.angular.z * (LENGTH/2)) / RADIO
		#convert float wR and wL into int8 0-255 PWM
		wR = self.get_translate_w(wR_f, 0, 32, 0, 255)
		wL = self.get_translate_w(wL_f, 0, 32, 0, 255)
		#NOW publish wR and wL to ARDUINO and I have wheel speeds
		pubwR = rospy.Publisher('/wR_pub', Int16, queue_size = 5)
		rate = rospy.Rate(10)
		pubwR.publish(wR)
		pubwL = rospy.Publisher('/wL_pub', Int16, queue_size = 5)
		rate = rospy.Rate(10)
		pubwL.publish(wL)
		#also publish /odom (what to pub in odom just cmd_vel??) think so
		
		return
		
	def get_translate_w(self, val, old_min, old_max, new_min, new_max):
		old_Span = old_max - old_min
		new_Span = new_max - new_min
		scaled = float(val - old_min) / float(old_Span)
		PWM_val = new_min + (scaled * new_Span)
		return int(PWM_val)
	

if __name__ == '__main__':
	try:
		rospy.init_node('diff_driv_ctrl')
		sub_obj = CMD_VEL()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Node terminated")
