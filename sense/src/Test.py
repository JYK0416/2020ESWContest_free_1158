#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
import copy
import Sonar 
import Temperature
import Sound
import Persondetection
import Illuminance
import Ir

import serial

from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy
from sensor_msgs.msg import CompressedImage

mode = 0

ser = serial.Serial('/dev/ttyACM1', 9600)


rospy.init_node("go")

pub = rospy.Publisher("/test_value", Int32, queue_size = 1)


BlowerBound = np.array([73, 70, 33])
BupperBound = np.array([104, 200, 112])

RlowerBound = np.array([158, 66, 85])
RupperBound = np.array([255, 255, 255])

direction = 4
velocity = 9



msg = "00"

k = 0
t = 0
x1 = 0
y1 = 0

x2 = 0
y2 = 0

on_ = 0

sensor_what = 0
sensor_position = 0
area = 0
def test(data):
	global direction
	global velocity
	global msg
	global sensor_what
	global sensor_position
	global x2
	global y2
	global area
	global on_

	if int(on_) is 1:
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
			    area_ = cv2.contourArea(Gcontours[i])
			    if area_ > 300:  # minimum yellow area
				area = area_
			    	rect1 = cv2.minAreaRect(Gcontours[i])
			        (x2, y2), (w2, h2), angle2 = cv2.minAreaRect(Gcontours[i])
				box1 = cv2.boxPoints(rect1)
				box1 = np.int0(box1)
				T1 = (int(x2), int(y2))
				cv2.putText(cam, str(T1), (int(x2), int(y2)), cv2.FONT_HERSHEY_PLAIN, 2, (0,255,255), 2)
				cam = cv2.drawContours(cam, [box1], -1, (0,255,0), 3)
		if x2 > 0 and y2 > 0:
			if int(sensor_what) is 1:
				#print("hh")
				able, value = Sonar.sonar(hsv, sensor_position, x2, y2)
				if int(able) is 1:
					print(value)
					pub.publish(value)
				else:
					pub.publish(0)
			elif int(sensor_what) is 3:
				able, value = Ir.ir(hsv, sensor_position, x2, y2)
				if int(able) is 1:
					print("ir", value)
					pub.publish(value)
				else:
					pub.publish(0)
			elif int(sensor_what) is 4:
				able, value = Temperature.temperature(hsv, sensor_position, x2, y2)
				if int(able) is 1:
					print("temp", value)
					pub.publish(value)
				else:
					pub.publish(0)
			elif int(sensor_what) is 6:
				able, value = Persondetection.persondetection(hsv, sensor_position, x2, y2)
				if int(able) is 1:
					print("person", value)
					pub.publish(value)
				else:
					pub.publish(0)
			elif int(sensor_what) is 7:
				able, value = Sound.sound(hsv, sensor_position, x2, y2)
				if int(able) is 1:
					print("sound", value)
					pub.publish(value)
				else:
					pub.publish(0)
			elif int(sensor_what) is 8:
				able, value = Illuminance.illuminance(hsv, sensor_position, x2, y2)
				if int(able) is 1:
					print("ill", value)
					pub.publish(value)
				else:
					pub.publish(0)
			else:
				pub.publish(0)
		
		msg = str(direction) + str(velocity)
		
		#print(edge[255][135])
		SendData(msg)
		#print("msg",msg)
		#print("chai", (x2 - x1))	
		cv2.imshow("adfasdf", cam)
		#cv2.imshow("adf", medi)
		cv2.waitKey(1)
		#pub_speed.publish(velocity)
		#pub_direction.publish(direction)
	else:
		pass



def mode(data):
	global mode
	mode = data.data
def sensorwhat(data):
	global sensor_what
	sensor_what = data.data

def sensordirection(data):
	global sensor_position
	sensor_position = data.data

def SendData(msg):
	ser.write(msg.encode())

def joy(data):

	global direction
	if int(data.data) is 0:
		direction = 0
	elif int(data.data) is 1:
		direction = 1
	elif int(data.data) is 2:
		direction = 2
	elif int(data.data) is 3:
		direction = 3
	elif int(data.data) is 4:
		direction = 4
def on(data):
	global on_
	on_ = data.data

rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, test)
rospy.Subscriber("/mode", Int32, mode)
rospy.Subscriber("test_sensor", Int32, sensorwhat)
rospy.Subscriber("/test_position", Int32, sensordirection)
rospy.Subscriber("/test_motor", Int32, joy)
rospy.Subscriber("/test_onoff", Int32, on)
rospy.spin()
