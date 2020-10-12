#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
import copy

import serial

from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray

from sensor_msgs.msg import CompressedImage

mode = 0

ser = serial.Serial('/dev/ttyACM1', 9600)


rospy.init_node("cv2")

#pub_speed = rospy.Publisher("/speed", Int32, queue_size = 1)
#pub_direction = rospy.Publisher("/direction", Int32, queue_size = 1)

BlowerBound = np.array([73, 70, 33])
BupperBound = np.array([104, 200, 112])

direction = 0
velocity = 0

sensor_position = list()
sensor_distance = list()
sensor_if = list()
sensor_direction = list()
sensor_speed = list()

msg = "00"

k = 0
t = 0
x2 = 0
y2 = 0

def test(data):
	global k
	global dis
	global direction
	global velocity
	global x2
	global y2
	global t
	global msg
	global k
	global sensor_position
	global sensor_distance
	global sensor_if
	global sensor_direction
	global sensor_speed
	global msg

	if mode is 1:	
		np_arr = np.fromstring(data.data, np.uint8)
		cam = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		gray = cv2.cvtColor(cam, cv2.COLOR_BGR2GRAY)
		gau = cv2.GaussianBlur(gray, (15,15), 0)
		thr = cv2.adaptiveThreshold(gau, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 4)
		medi = cv2.medianBlur(thr, 3)
		edge = cv2.Canny(gray, 200, 250)
	
		hsv = cv2.cvtColor(cam, cv2.COLOR_BGR2HSV)
		Bmask = cv2.inRange(hsv, BlowerBound, BupperBound)
		Gcontours, _ = cv2.findContours(Bmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		if len(Gcontours) > 0:
		        for i in range(len(Gcontours)):
		            # Get area value
		            area = cv2.contourArea(Gcontours[i])
		            if area > 50:  # minimum yellow area
		            	rect1 = cv2.minAreaRect(Gcontours[i])
		                (x2, y2), (w2, h2), angle2 = cv2.minAreaRect(Gcontours[i])
				box1 = cv2.boxPoints(rect1)
				box1 = np.int0(box1)
				T1 = (int(x2), int(y2))
				cv2.putText(cam, str(T1), (int(x2), int(y2)), cv2.FONT_HERSHEY_PLAIN, 2, (0,255,255), 2)
				cam = cv2.drawContours(cam, [box1], -1, (0,255,0), 3)
	


		t = 0
		for i in range(k):
			if t is 1:
				print("gpg")
				break
			else:
				if sensor_position[i] is 0:
					if sensor_if[i] is 0:
						for j in range(sensor_distance[i]):
							if int(medi[int(x2)][int(y2 + j)]) is 0:
								direction = sensor_direction[i]
								velocity = sensor_speed[i]
								msg = str(direction) + str(velocity)
								print("front")
								t = 1
								break
					
					else:
						for j in range(sensor_distance[i]):
							if medi[x2, y2 + j] is 0:
								break
							else:
								if j is sensor_distance[i] - 1:
									direction = sensor_direction[i]
									velocity = sensor_speed[i]
						
				elif sensor_position[i] is 1:
					if sensor_if[i] is 0:
						for j in range(sensor_distance[i]):
							if int(medi[int(x2), int(y2 - j)]) is 0:
								direction = sensor_direction[i]
								velocity = sensor_speed[i]
								msg = str(direction) + str(velocity)
								print("back")
								t = 1
								break
					
					else:
						for j in range(sensor_distance[i]):
							if int(medi[int(x2), int(y2 - j)]) is 0:
								break
							else:
								if j is sensor_distance[i] - 1:
									direction = sensor_direction[i]
									velocity = sensor_speed[i]
				
				elif sensor_position[i] is 2:
					if sensor_if[i] is 0:
						for j in range(sensor_distance[i]):
							if medi[x2 + j, y2] is 255:
								direction = sensor_direction[k]
								velocity = sensor_speed[k]
								break
					
					else:
						for j in range(sensor_distance[i]):
							if medi[x2 + j, y2] is 0:
								break
							else:
								if j is sensor_distance[i] - 1:
									direction = sensor_direction[k]
									velocity = sensor_speed[k]

				elif sensor_position[i] is 3:
					if sensor_if[i] is 0:
						for j in range(sensor_distance[i]):
							if medi[x2 - j, y2] is 255:
								direction = sensor_direction[k]
								velocity = sensor_speed[k]
								break
					
					else:
						for j in range(sensor_distance[i]):
							if medi[x2 - j, y2] is 0:
								break
							else:
								if j is sensor_distance[i] - 1:
									direction = sensor_direction[k]
									velocity = sensor_speed[k]

			
		#print(edge[255][135])
		SendData(msg)		
		cv2.imshow("adfasdf", cam)
		cv2.imshow("adf", medi)
		cv2.waitKey(1)
		#pub_speed.publish(velocity)
		#pub_direction.publish(direction)



def mode(data):
	global mode
	mode = data.data

def sensor(data):
	global k
	global sensor_position
	global sensor_distance
	global sensor_if
	global sensor_direction
	global sensor_speed
	if k is not 3:
		print(data.data[5])
		sensor_position.append(data.data[1])
		sensor_distance.append(data.data[2])
		sensor_if.append(data.data[3])
		sensor_direction.append(data.data[4])
		sensor_speed.append(data.data[5])
		k = k + 1
		
def SendData(msg):
	ser.write(msg.encode())



rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, test)
rospy.Subscriber("/sensor", Int32MultiArray, sensor)
rospy.Subscriber("/mode", Int32, mode)

rospy.spin()
