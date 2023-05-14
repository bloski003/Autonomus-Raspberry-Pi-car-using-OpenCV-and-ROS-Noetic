#!/usr/bin/python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class Hist_sub:
	def __init__(self):
		image_topic = "/webcam"
		rospy.Subscriber(image_topic, Image, self.image_callback)
		self.bridge = CvBridge()
	
	def image_callback(self, msg):
		try:
			cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
			self.initTrackBarVals = [102, 80, 20, 214]
			resize_img = cv2.resize(cv2_img,(480,240)) # RESIZE
			self.init_track_bars(self.initTrackBarVals)
			curve = self.get_line_curve(resize_img)
			print(curve)
			#cv2.imshow("RGB Image", cv2_img)
			#cv2.waitKey(3)
		except CvBridgeError as e:
			print(e)
			return
		
	
	def get_line_curve(self, img, display=2): #0 no display, 1 display result, 2 display everything
		curveList = []
		avgVal = 10
		imgCopy = img.copy()
		imgResult = img.copy()
		imgThres = self.get_threshold_img(img)
		
		hT, wT, cT = img.shape
		points = self.val_track_bars()
		imgWrap = self.get_wrap_img(imgThres, points, wT, hT)
		imgWarpPoints = self.draw_points(imgCopy, points)
		
		middlePoint, imgHist = self.get_histogram(imgWrap, minPer=0.5,display=True, region=4)
		curveAveragePoint, imgHist = self.get_histogram(imgWrap, minPer=0.9, display=True)
		curveRow = curveAveragePoint - middlePoint
		
		curveList.append(curveRow)
		if len(curveList)>avgVal:
			curveList.pop(0)
		curve = int(sum(curveList)/len(curveList))
		
		if display != 0:
			imgInvWarp = self.get_wrap_img(imgWrap, points, wT, hT, inv=True)
			imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
			imgInvWarp[0:hT // 3, 0:wT] = 0, 0, 0
			imgLaneColor = np.zeros_like(img)
			imgLaneColor[:] = 0, 255, 0
			imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
			imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)
			midY = 450
			cv2.putText(imgResult, str(curve), (wT // 2 - 80, 85), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 255), 3)
			cv2.line(imgResult, (wT // 2, midY), (wT // 2 + (curve * 3), midY), (255, 0, 255), 5)
			cv2.line(imgResult, ((wT // 2 + (curve * 3)), midY - 25), (wT // 2 + (curve * 3), midY + 25), (0, 255, 0), 5)
			for x in range(-30,30):
				w = wT // 20
				cv2.line(imgResult, (w * x + int(curve // 50), midY - 10), (w * x + int(curve // 50), midY + 10), (0, 0, 255), 2)
				
		if display == 2:
			imgStacked = self.get_imgs_stacked(0.7, ([img, imgWarpPoints, imgWrap],[imgHist, imgLaneColor, imgResult]))
			cv2.imshow('ImageStack', imgStacked)
			cv2.waitKey(30)
		
		elif display == 1:
			cv2.imshow('Resutlt', imgResult)
			cv2.waitKey(30)
		return curve
	
	def nothing(self, a):
		pass
	
	
	def get_threshold_img(self, img):
		hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		lower = np.array([85, 0, 0])
		upper = np.array([179, 160, 255])
		thr_img = cv2.inRange(hsv_img, lower, upper)
		return thr_img
	
	def val_track_bars(self, wT=480, hT=240):
		widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
		heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
		widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
		heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
		points = np.float32([(widthTop, heightTop), (wT-widthTop, heightTop), (widthBottom , heightBottom), (wT-widthBottom, heightBottom)])
		return points
		
	def init_track_bars(self, intialTracbarVals, wT=480, hT=240):
		cv2.namedWindow("Trackbars")
		cv2.resizeWindow("Trackbars", 360, 240)
		cv2.createTrackbar("Width Top", "Trackbars", intialTracbarVals[0],wT//2, self.nothing)
		cv2.createTrackbar("Height Top", "Trackbars", intialTracbarVals[1], hT, self.nothing)
		cv2.createTrackbar("Width Bottom", "Trackbars", intialTracbarVals[2],wT//2, self.nothing)
		cv2.createTrackbar("Height Bottom", "Trackbars", intialTracbarVals[3], hT, self.nothing)
	
		
	def draw_points(self, img, points):
		for x in range(0,4):
			cv2.circle(img,(int(points[x][0]),int(points[x][1])),15,(0,0,255),cv2.FILLED)
		return img
		
	def get_histogram(self, img, minPer=0.1, display=False, region=1):
		if region == 1:
			histValues = np.sum(img, axis=0)
		else:
			histValues = np.sum(img[img.shape[0]//region:,:], axis =0)
		
		maxValue = np.max(histValues)
		minValue = minPer * maxValue
		indexArray = np.where(histValues >= minValue) # ALL INDICES WITH MIN VALUE OR ABOVE
		basePoint = int(np.average(indexArray)) # AVERAGE ALL MAX INDICES VALUES
		imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
		for x, intensity in enumerate(histValues):
			cv2.line(imgHist, (x, img.shape[0]), (x, img.shape[0]- round(intensity) // 255//region), (255, 0, 255),1)
			cv2.circle(imgHist, (basePoint, img.shape[0]), 20, (0, 255, 255), cv2.FILLED)
		return basePoint, imgHist
		
	def get_wrap_img(self, img, points, w, h, inv=False):
		pts1 = np.float32(points)
		pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
		if inv:
			matrix = cv2.getPerspectiveTransform(pts2,pts1)
		else:
			matrix = cv2.getPerspectiveTransform(pts1, pts2)
		imgWrap = cv2.warpPerspective(img,matrix,(w,h))
		return imgWrap
		
	def get_imgs_stacked(self,scale, imgArray):
		rows = len(imgArray)
		cols = len(imgArray[0])
		rowsAvailable = isinstance(imgArray[0], list)
		width = imgArray[0][0].shape[1]
		height = imgArray[0][0].shape[0]
		if rowsAvailable:
			for x in range ( 0, rows):
				for y in range(0, cols):
					if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:imgArray[x][y]= cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
					else: imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
					if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
			imageBlank = np.zeros((height, width, 3), np.uint8)
			hor = [imageBlank]*rows
			hor_con = [imageBlank]*rows
			for x in range(0, rows):
				hor[x] = np.hstack(imgArray[x])
			ver = np.vstack(hor)
		else:
			for x in range(0, rows):
				if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
					imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
				else:
					imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
				if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
			hor= np.hstack(imgArray)
			ver = hor
		return ver
    


if __name__ == '__main__':
	try:
		rospy.init_node('hist_sub')
		sub_obj = Hist_sub()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Node terminated")
