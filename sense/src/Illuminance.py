#!/usr/bin/env python
# -*- coding: utf-8 -*-


import cv2
import numpy as np


BlowerBound = np.array([0, 52, 148])
BupperBound = np.array([88, 255, 255])

x2 = 0
y2 = 0
area_1 = 0
def illuminance(hsv, sensor_position, x1, y1):

	global x2
	global y2
	global area_1

	Bmask = cv2.inRange(hsv, BlowerBound, BupperBound)
	Gcontours, _ = cv2.findContours(Bmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	if len(Gcontours) > 0:
	        for i in range(len(Gcontours)):
	            # Get area value
	            area = cv2.contourArea(Gcontours[i])
	            if area > 300:  # minimum yellow area
			area_1 = area
	            	rect1 = cv2.minAreaRect(Gcontours[i])
	                (x2, y2), (w2, h2), angle2 = cv2.minAreaRect(Gcontours[i])
	
	if x2 is not 0 and y2 is not 0:

		if sensor_position is 2:
			if y2 < y1+5 and y2 > y1 -5:
				if x2 - x1 < 0:
					return 1, area_1 * 0.1 / abs(x2-x1) * 600
				else:
					return 0, 0
				
			else:
				return 0, 0

		elif sensor_position is 3:
			if y2 < y1+5 and y2 > y1 -5:
				if x2 - x1 > 0:
					return 1, area_1 * 0.1 / abs(x2-x1) * 600
				else:
					return 0, 0

			else:
				return 0, 0

		elif sensor_position is 0: 
			if x2 < x1+5 and x2 > x1 -5:
				if y2 - y1 < 0:
					return 1, area_1 * 0.1 / abs(y2-y1) * 600
				else:
					return 0,0
			else:
				return 0, 0

		else:

			if x2 < x1+5 and x2 > x1 -5:
				if y2 - y1 > 0:
					return 1, area_1 * 0.1 / abs(y2-y1) * 600
				else:
					return 0, 0
			else:
				return 0, 0
			
	else:
		return 0, 0

