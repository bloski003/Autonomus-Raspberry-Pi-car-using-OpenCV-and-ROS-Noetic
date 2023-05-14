#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist #floats 64
from sensor_msgs.msg import Range


class CMD_VEL:
	def __init__(self):
		curve_topic = "/curve_value"
		rospy.Subscriber(curve_topic, Int8, self.curve_callback)
		rospy.Subscriber("/ultrasound", Int8, self.ultrasound_callback)
	
	global range_flag;
		
	def curve_callback(self, msg):
		#Here get curve_value int8 and transform to Twist 
		curve_val = msg.data #copy msg content into curve_val and
		if(curve_val > 60):
			curve_val == 60
		elif(curve_val < -60):
			curve_val == -60
		else:
			curve_val == curve_val
		curve_val_t = self.get_translate_curve(curve_val, -60, 60, -1, 1) #ANGULAR range -1 to 1 same as Linear
		print(curve_val)
		print(curve_val_t)
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 5) #also define here the publisher of cmd_vel
		rate = rospy.Rate(10)
		twist_msg = Twist() #create twist msg type
		
		if(range_flag == 1):
			twist_msg.linear.x = 0.0
			twist_msg.angular.z = 0.0
			pub.publish(twist_msg)
		else:
			pass
			
		if curve_val_t <= 0.03 and curve_val_t >= -0.03:
			twist_msg.linear.x = 0.3 
			twist_msg.angular.z = 0.0
			pub.publish(twist_msg)
		elif curve_val_t < -0.03:
			twist_msg.linear.x = 0.3
			twist_msg.angular.z = curve_val_t
			pub.publish(twist_msg)
		elif curve_val_t > 0.03:
			twist_msg.linear.x = 0.3
			twist_msg.angular.z = curve_val_t
			pub.publish(twist_msg)
		return
		
	
	def ultrasound_callback(self, msg):
		#callback for ultrasonic sensor
		range = msg.range #copy msg range content into range
		if(range < 0.3):
			range_flag == 1
		else:
			range_flag == 0
		print(range)
		return
      		
	def get_translate_curve(self, val, old_min, old_max, new_min, new_max):
		old_Span = old_max - old_min
		new_Span = new_max - new_min
		scaled = float(val - old_min) / float(old_Span)
		curve_val_t = new_min + (scaled * new_Span)
		return curve_val_t
    

if __name__ == '__main__':
	try:
		rospy.init_node('cmd_vel')
		sub_obj = CMD_VEL()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Node terminated")
