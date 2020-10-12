#!/usr/bin/env python
# -*- coding: utf-8 -*-


import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
import rospy

rospy.init_node("asdfasdfasdfasdfdfas")
def onChange(x):
    pass

def setting_bar():
    cv2.namedWindow('HSV_settings')

    cv2.createTrackbar('H_MAX', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('H_MAX', 'HSV_settings', 255)
    cv2.createTrackbar('H_MIN', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('H_MIN', 'HSV_settings', 0)
    cv2.createTrackbar('S_MAX', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('S_MAX', 'HSV_settings', 255)
    cv2.createTrackbar('S_MIN', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('S_MIN', 'HSV_settings', 0)
    cv2.createTrackbar('V_MAX', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('V_MAX', 'HSV_settings', 255)
    cv2.createTrackbar('V_MIN', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('V_MIN', 'HSV_settings', 0)


def test(data):
	np_arr = np.fromstring(data.data, np.uint8)
	cam = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        H_MAX = cv2.getTrackbarPos('H_MAX', 'HSV_settings')
        H_MIN = cv2.getTrackbarPos('H_MIN', 'HSV_settings')
        S_MAX = cv2.getTrackbarPos('S_MAX', 'HSV_settings')
        S_MIN = cv2.getTrackbarPos('S_MIN', 'HSV_settings')
        V_MAX = cv2.getTrackbarPos('V_MAX', 'HSV_settings')
        V_MIN = cv2.getTrackbarPos('V_MIN', 'HSV_settings')
        lower = np.array([H_MIN, S_MIN, V_MIN])
        higher = np.array([H_MAX, S_MAX, V_MAX])
        hsv = cv2.cvtColor(cam, cv2.COLOR_BGR2HSV)
        Gmask = cv2.inRange(hsv, lower, higher)
        G = cv2.bitwise_and(cam, cam, mask = Gmask)

        #cv2.imshow('cam_load',cam)
        cv2.imshow('G',G)
        cv2.waitKey(1)

setting_bar()

rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, test)

rospy.spin()
