#! /usr/bin/env python
# computer vision
import cv2
import numpy as np
import modul
import time
# ros quadcopter
# ros library
import rospy
# import the API
from iq_gnc.py_gnc_functions import *
# print the colours
from iq_gnc.PrintColours import *
# Importing Point message from package geometry_msgs.
from geometry_msgs.msg import Point
#
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class detect_an_object(object):
    def __init__(self, params):
        self.cx_obj = 0
        self.cy_obj = 0
        self.cx_frm = 0
        self.cy_frm = 0
        self.distance = 0
        self.altitude = params.get('altitude')
        self.startFly = params.get('start')
        self.contours = []  # this is list
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber(
            "/webcam", Image, self.img_cb)  # 10 -> queue_size
        self.drone = params.get('droneApi')

        #temp
        self.x = 0
        self.y = 0

    def img_cb(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #detect object
        self.contours = modul.finding_object(cv_img)
        self.img = cv_img
        if len(self.contours) > 0:
            self.centroid()
            self.distance = self.object_distance() #---

        #get current position of the drone
        current_pos = Point()
        current_pos = self.drone.get_current_location()
        print ("Lokasi saat ini : {},{}".format(current_pos.x, current_pos.y))


        # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
        rate = rospy.Rate(0.5)

        #ask drone to fly into desired waypoint
        if len(self.contours) > 0:
            #get next destination
            (x_direction, y_direction, next) = self.next_destination()
            # print ("Perintah berikutnya : {},{}".format(self.x, self.y))
            
            if next: 
                next_pos = Point()
                next_pos.x = x_direction + current_pos.x
                next_pos.y = y_direction + current_pos.y
                print ("Next destination is {},{} - altitude : {}".format(next_pos.x,next_pos.y, self.altitude))

                self.drone.set_destination(next_pos.x, next_pos.y, self.altitude, 0)
                # rate.sleep()
            else:
                """
                kalo ada perintah khusus ketika objek telah berada tepat di tengah quadcopter
                silahkan letakkan disini
                """

                # calculate time in air in minute
                now = time.time()
                tia = (now -self.startFly)/60
                print ("Time in air : {}".format(tia))
                if (tia) > 0.5:
                    # ask the drone back to home then land
                    # if the drone has flown for more than 30's
                    self.drone.set_mode("RTL")
                    rate.sleep()
                    self.drone.land()
                    rospy.loginfo(CGREEN2 + "All waypoints already reached. Then land" + CEND)
                    
                    #stop subscribing this topic
                    self.img_sub.unregister()

                    """
                    1. stop spinning the ros node.
                    2. as i know, the rospy.spin() method will always run until 
                       we give the true value into rospy.is_shutdown 
                    """
                    rospy.signal_shutdown("")
        else:
            pass


        # show up the detected object in the frame
        cv2.imshow("Object Following", self.img)
        cv2.waitKey(1)

    def get_ojbect(self):
        return self.contours

    def next_destination(self):
        next = False
        x = 0.0
        y = 0.0

        xfrm_xobj = self.cx_obj-self.cx_frm
        yfrm_yobj = self.cy_obj-self.cy_frm

        if len(self.contours) > 0:
            next = True
            
            if (xfrm_xobj < -20) and (yfrm_yobj < -20):
                x = -0.2
            elif (xfrm_xobj < -20) and (yfrm_yobj > 20):
                x = -0.2
            elif (xfrm_xobj > 20) and (yfrm_yobj < -20):
                x = 0.2
            elif (xfrm_xobj > 20) and (yfrm_yobj > 20):
                x = 0.2
            else:
                if (self.distance > 30):
                    y = 0.2
                else:
                    next = False

        print("Apakah ada perintah lanjut : {}".format(next))

        return [x, y, next]
            # return next

    def centroid(self):
        self.cx_obj, self.cy_obj = modul.centroid_image_v2(self.contours)
        self.cx_frm, self.cy_frm = modul.centroid_frame()
        # print("cX and cY object : {},{} - cX and cY frame : {},{}".format(self.cx_obj,
        #   self.cy_obj, self.cx_frm, self.cy_frm))

        # draw line from centroid image into centroid object
        cv2.line(self.img, (self.cx_obj, self.cy_obj),
                 (self.cx_frm, self.cy_frm), (255, 255, 255), 4)

        cv2.putText(self.img, "Distance in X direction : {} and Y direction : {}".format(
            self.cx_frm-self.cx_obj, self.cy_frm-self.cy_obj), (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    def object_distance(self):
        if len(self.contours) > 0:
            # finding distance from camera into image
            marker = modul.detail_object(self.contours, "biggest_contour")

            # we can get focal_length using find_distance_from_camera_to_image.py file and of course we need to change any variable there
            distance = modul.distance_to_camera(
                19.5, 784.615384615, marker[1][0])

            # print out the result at the image
            cv2.putText(self.img, "Distance camera to object : {}".format(
                distance), (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

            return distance

def main():
    # initialize ros node
    rospy.init_node("ObjectFollowing", anonymous="True")

    drone = gnc_api()

    #  wait for FCU connection
    drone.wait4connect()
    #  wait for the mode to be SWITCH
    drone.wait4start()
    #  create local reference frame
    drone.initialize_local_frame()
    #  request altitude for takeoff
    rospy.loginfo(CYELLOW2 + CBLINK +
       "Waiting for user to fill the request Altitude" + CEND)
    altitude = input("Request Altitude : ")
    #  take off
    drone.takeoff(altitude)

    # get current time then print to calculate flight time
    start = time.time()
    print("Current time : {}".format(time.ctime(start)))

    # initialize detect_an_object class as an object
    params = {
        "altitude": altitude,
        "start": start,
        "droneApi": drone
    }
    objek = detect_an_object(params)


if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except KeyboardInterrupt:
        exit()
