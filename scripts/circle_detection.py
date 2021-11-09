#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import modul

#note
#knwon_width=19.5
#focal_length=784.615384615

def callback(data):
    #bridge = CvBridge()
    try:
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print (e)

    #start and find the object
    marker = modul.finding_object(cv_img)
    if len(marker) > 0:
        #get centroid of the object and frame
        cx_img, cy_img = modul.centroid_image_v2(marker)
        cx_frm, cy_frm = modul.centroid_frame()
        print "cX and cY image : {},{} - cX and cY frame : {},{}".format(cx_img,cy_img,cx_frm,cy_frm)
    
        #draw line from centroid image into centroid frame
        cv2.line(modul.img, (cx_img,cy_img), (cx_frm,cy_frm), (255,255,255), 4)

        #finding distance from camera to image
        detected_marker = modul.detail_object(marker,"biggest_contour")
        #we can get focal_length using find_distance_from_camera_to_image.py file and of course we need to change any variable there
        distance = modul.distance_to_camera(19.5, 784.615384615, detected_marker[1][0])

        #print out the result
        cv2.putText(modul.img, "distanece   : {} cm".format(distance), (30,25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)
        cv2.putText(modul.img, "x direction : {} px".format(cx_frm-cx_img), (30,45), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)
        cv2.putText(modul.img, "y direction : {} px".format(cy_frm-cy_img), (30,65), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 2)
    
        #displaying the result in original image
        cv2.imshow("Result", modul.img)
        cv2.waitKey(1)

        try:
            img_pub.publish(bridge.cv2_to_imgmsg(modul.img, "bgr8"))
        except CvBridgeError as e:
            print (e)

def sub_pub():
    global img_pub
    global bridge
    rospy.init_node("CircleDetection", anonymous=True)
    rospy.Subscriber("/webcam", Image, callback)
    img_pub = rospy.Publisher("/detected_object", Image, queue_size=1)
    bridge = CvBridge()	
    rospy.spin()

if __name__ == '__main__':
    try:
        sub_pub()
    except cv2.ROSInterruptException:
        pass
    #modul.destroy()
