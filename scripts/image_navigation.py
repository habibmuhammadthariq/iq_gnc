#! /usr/bin/env python
#ros library
#import rospy
#import the API
#from iq_gnc.py_gnc_functions import *
#print the colours
#from iq_gnc.PrintColours import *
# Importing Point message from package geometry_msgs.
#from geometry_msgs.msg import Point
#import opencv library 
import cv2

cap = cv2.VideoCapture(0)

while True:
    _, img = cap.read()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #red color
    low_red = np.array([161, 155, 84])
    high_red = np_array([179, 255, 255])
    #blue color
    #low_blue = np.array([94, 80, 2])
    #high_blue = np.array([126, 255, 255])
    #green color
    #low_green = np.array([25, 52, 72])
    #high_green = np.array([102, 255, 255])
    #every color except white
    #low = np.array([0, 42, 0])
    #high = np.array([179, 255, 255])
    red_mask = cv2.inRange(hsv, low_red, high_red)
    red = cv2.bitwise_and(image, image, mask=red_mask)

    cv2.imshow("Original Image", image)
    cv2.imshow("Red Filter", red)

    key = cv2.waitKey(1)
    if key == 27:
        break
