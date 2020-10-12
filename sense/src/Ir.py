#!/usr/bin/env python
# -*- coding: utf-8 -*-


import cv2
import numpy as np


def ir(hsv, sensor_position, x1, y1):
	
	img_f = "/home/robot/catkin_ws/src/test/src/ir.png"
	img = cv2.imread(img_f, cv2.IMREAD_COLOR)
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	gau = cv2.GaussianBlur(gray, (15,15), 0)
	thr = cv2.adaptiveThreshold(gau, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 4)
	medi = cv2.medianBlur(thr, 3)
	cv2.imshow("asdasdfsdfas", medi)

	
	print(medi[297,297])
	if int(medi[int(x1), int(y1)]) is 255:
		return 1, 1
	else:
		return 0, 0
	
