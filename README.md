# Autonomus-Raspberry-Pi-car-using-OpenCV-and-ROS-Noetic

Project for Autonomous Robot to detect and follow a road using OpenCV with Canny Edge detector and Hough Transform. Aditionally, implemnts ultrasonic sensor reading to avoid frontal collisions
and circle hough transform to detect a traffic light and stop if the light is red or yellow and continue if its green. The developed system is based on a 4GB Raspberry Pi 4 with a Raspberry pi
Camera version 1.3 to gather the images. Then are pusblished to a ROS network subscribed by a computer to perform the image detection. ROS Noetic is used and the following link contains the Raspberry Pi 
Image with OpenCV, Rosserial and ROS Noetic inatalled.
Pi Image Link: https://caledonianac-my.sharepoint.com/:f:/g/personal/pdemel200_caledonian_ac_uk/EvEb9cotwGpLpWY3g6C9TpABpliTvR43O0RFBC3G09CeKw?e=cIMWIE
To control the robot an arduino uno is used with an L298N motor driver power froma  battery bank. The Arduino Uno has the rosserial library installed to connect with the Pi.

Video Demo: https://caledonianac-my.sharepoint.com/:f:/g/personal/pdemel200_caledonian_ac_uk/EpGEJ9IFNfhOnvms_ifHKBgB0JfzM_hAtQbMBE_UnVpO0g?e=eubKbv

