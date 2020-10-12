#!/usr/bin/env python

import cv2
import numpy as np
import rospy 
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage

rospy.init_node("test", anonymous=True)

def test(data):
	np_arr = np.fromstring(data.data, np.uint8)
	cam = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	gray = cv2.cvtColor(cam, cv2.COLOR_BGR2GRAY)
	template = cv2.imread("/home/robot/Desktop/icon/fire.jpg", cv2.IMREAD_GRAYSCALE)
	w ,h = template.shape[::-1]

	res = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)
	loc = np.where(res >= 0.8)
	for pt in zip(*loc[::-1]):
		print(pt)
		cv2.rectangle(cam, pt, (pt[0] + w, pt[1]+h), (0,0,255),2)
	cv2.imshow("asdf", template)
	cv2.imshow("asdkflj", gray)
	cv2.waitKey(1)
	


rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, test)

rospy.spin()
