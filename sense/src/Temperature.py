#!/usr/bin/env python
# -*- coding: utf-8 -*-


import cv2
import numpy as np


BlowerBound = np.array([0, 80, 139])
BupperBound = np.array([85, 255, 255])

x2 = 0
y2 = 0
area = 0
def temperature(hsv, sensor_position, x1, y1):

	global x2
	global y2
	global area

	Bmask = cv2.inRange(hsv, BlowerBound, BupperBound)
	Gcontours, _ = cv2.findContours(Bmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	if len(Gcontours) > 0:
	        for i in range(len(Gcontours)):
	            # Get area value
	            area_1 = cv2.contourArea(Gcontours[i])
	            if area_1 > 100:  # minimum yellow area
			area = area_1
	            	rect1 = cv2.minAreaRect(Gcontours[i])
	                (x2, y2), (w2, h2), angle2 = cv2.minAreaRect(Gcontours[i])
	cv2.imshow("asasdfasdfasdfasdfadfdf", hsv)
	if x2 is not 0 and y2 is not 0:
		print(x2,y2)
		if sensor_position is 2:
			if y2 < y1+5 and y2 > y1 -5:
				if abs(x2-x1) < 30 and x2 - x1 < 0:
					return 1, area * 0.2
				else:
					return 0,0
				
			else:
				return 0, 0

		elif sensor_position is 3:
			if y2 < y1+5 and y2 > y1 -5:
				if abs(x2-x1) < 30 and x2 - x1 > 0:
					return 1, area * 0.2
				else:
					return 0,0
			else:
				return 0, 0

		elif sensor_position is 0: 
			if x2 < x1+5 and x2 > x1 -5:
				if abs(y1 - y2) < 60 and y2 - y1 < 0:
					return 1, area * 0.2
				else:
					return 0,0
			else:
				return 0, 0

		else:

			if x2 < x1+5 and x2 > x1 -5:
				if abs(y1 - y2) < 60 and y2 - y1 > 0:
					return 1, area * 0.2
			else:
				return 0, 0
			
	else:
		return 0, 0

