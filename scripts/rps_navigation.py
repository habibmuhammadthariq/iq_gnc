#! /usr/bin/env python
#ros library
import rospy
#import the API
from iq_gnc.py_gnc_functions import *
#print the colours
from iq_gnc.PrintColours import *
#tensorflow library
import numpy as np		#backup
import tensorflow as tf
#keras to load model
from tensorflow import keras
#computer vision library
import cv2


#variable declaration to get image
cap = cv2.VideoCapture(0)
#set image size
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 150)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 150)
cap.set(cv2.CAP_PROP_FPS, 15)

def rps_detection(img):
    #load model
    model = keras.models.load_model('rock_paper_scissors_model')
	
    x = image.img_to_array(img)
    x = np.expand_dims(x, axis=0)

    images = np.vstack([x])
    classes = model.predict(images, batch_size=10)
	
    if classes[0,0]!=0:
        print('paper')
        result = 1
    elif classes[0,1]!=0:
        print('rock')
        result = 2
    else:
        print('scissors')
        result = 3
	
    return (result)

def rock():
    #Specify some waypoints to
    goals = [[0, 0, 3, 0], [5, 0, 3, -90], [5, 5, 3, 0],
             [0, 5, 3, 90], [0, 0, 3, 180], [0, 0, 3, 0]]
    i = 0

    while i < len(goals):
        drone.set_destination(
            x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3])
        self.rate.sleep()
        if drone.check_waypoint_reached():
            i += 1
def paper():
    #Specify some waypoints
    goals = [[0, 0, 3, 0], [5, 0, 3, -40]]
    i = 0

    while i < len(goals):
        drone.set_destination(
            x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3])
        self.rate.sleep()
        if drone.check_waypoint_reached():
            i += 1
	
    # Land after all waypoints is reached.
    drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)
    
def scissors():
    drone.land()
    rospy.loginfo(CGREEN2 + "Landing now." + CEND)
     
def main(self):
    # Initializing the ROS node.
    rospy.init_node("rps_navigation", anonymous=True)

    # Create an object for the API.
    drone = gnc_api()	
    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    drone.wait4start()
    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 2m.
    drone.takeoff(3)

    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    self.rate = rospy.Rate(3)
	
	#image detectioin looping
    while True :
        ret, img = capt.read()
		
	#hand gesture detection
        hand_gesture = rps_detection(img)
        if(hand_gesture==1):
            paper()
        elif(hand_gesture==2):
            rock()
        else:
            scissors()
		
    # Used to keep the node running.
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
	


