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


class detect_an_object:
    def __init__(self):
        self.cx_obj = 0
        self.cy_obj = 0
        self.cx_frm = 0
        self.cy_frm = 0
        self.contours = []  # this is list
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber(
            "/webcam", Image, self.img_cb)  # 10 -> queue_size

    def img_cb(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except rospy.ROSInterruptException as e:
            print(e)

        self.contours = modul.finding_object(cv_img)
        self.img = cv_img
        if len(self.contours) > 0:
            self.centroid()
            self.object_distance()

        # show up the detected object in the frame
        cv2.imshow("Object Following", self.img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows

    def get_ojbect(self):
        return self.contours

    def next_destination(self):
        next = True
        x = 0
        y = 0

        xfrm_xobj = self.cx_frm-self.cx_obj
        yfrm_yobj = self.cy_frm-self.cy_obj

        if len(self.contours) > 0:
            if (xfrm_xobj < -30) and (yfrm_yobj < -30):
                x = -1
                y = -1
            elif (xfrm_xobj < -30) and (yfrm_yobj > 30):
                x = -1
                y = 1
            elif (xfrm_xobj > 30) and (yfrm_yobj < -30):
                x = 1
                y = -1
            elif (xfrm_xobj > 30) and (yfrm_yobj > 30):
                x = 1
                y = 1
            else:
                next = False

            return x, y, next

    def centroid(self):
        self.cx_obj, self.cy_obj = modul.centroid_image_v2(self.contours)
        self.cx_frm, self.cy_frm = modul.centroid_frame()
        # print("cX and cY object : {},{} - cX and cY frame : {},{}".format(self.cx_obj,
        #   self.cy_obj, self.cx_frm, self.cy_frm))
        # draw line from centroid image into centroid object
        cv2.line(self.img, (self.cx_obj, self.cy_obj),
                 (self.cx_frm, self.cy_frm), (255, 255, 255), 4)

    def object_distance(self):
        if len(self.contours) > 0:
            # finding distance from camera into image
            marker = modul.detail_object(self.contours, "biggest_contour")
            # we can get focal_length using find_distance_from_camera_to_image.py file and of course we need to change any variable there
            distance = modul.distance_to_camera(
                19.5, 784.615384615, marker[1][0])

            # print out the result
            cv2.putText(self.img, "Distance camera to object : {}".format(
                distance), (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

            return distance


# Create an object for the API and make it as global variable
drone = gnc_api()


def main():
    # initialize ros node
    rospy.init_node("ObjectFollowing", anonymous="True")
    # get current time then print
    start = time.time()
    print("Current time : {}".format(time.ctime(start)))

    # # wait for FCU connection
    drone.wait4connect()
    # # wait for the mode to be SWITCH
    drone.wait4start()
    # # create local reference frame
    drone.initialize_local_frame()
    # # request altitude for takeoff
    rospy.loginfo(CYELLOW2 + CBLINK +
                  "Waiting for user to fill the request Altitude" + CEND)
    altitude = input("Request Altitude : ")
    # # take off
    drone.takeoff(altitude)

    # initialize detect_an_object class as an object
    objek = detect_an_object()

    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(1)

    while True:

        if len(objek.contours) > 0:
            # get centroid object
            next = objek.next_destination()

            # Ask drone to fly into the desire position
            cur_pos = drone.get_current_location()
            x = next[0] + cur_pos.x
            y = next[1] + cur_pos.y

            drone.set_destination(x, y, altitude, 0)
            rate.sleep()

            if not next[2]:
                drone.set_mode("RTL")
                rate.sleep()
                break
        else:
            # calculate time in air in minute
            # if the drone has flown for 30 seconds
            now = time.time()
            tia = (start - now)/60
            if (tia) < 0.5:
                continue

    # make the drone land
    drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints already reached. Then land" + CEND)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
