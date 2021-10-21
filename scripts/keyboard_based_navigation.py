#! /usr/bin/env python
#ros library
import rospy
#import the API
from iq_gnc.py_gnc_functions import *
#print the colours
from iq_gnc.PrintColours import *
# Importing Point message from package geometry_msgs.
from geometry_msgs.msg import Point

# Create an object for the API and make it as global variable
drone = gnc_api()	

def get_next_destination(altitude):
    print ("\tW")
    print ("A\t\tD")
    print ("\tS")
    print ("Note : ")
    print ("W : move forward")
    print ("A : move left")
    print ("D : move right")
    print ("S : move backward")

    command = input("input command : ")
  
    #get current posisiton
    current_pos = Point()
    current_pos = drone.get_current_location()

    #default value declaration
    x = 0
    y = 0
    z = altitude
    next = True

    #set next destination
    if (command == 'W'):
        y = 2
    elif (command == 'S'):
        y = -2
    elif (command == 'A'):
        x = -2
    elif (command == 'D'):
        x = 2
    else:
        next = False

    #final data destination
    x = x + current_pos.x
    y = y + current_pos.y

    #data feedback
    #x is move left or right command
    #y is forward or backward command
    #z is altitude. absolut value
    #next is continue or stop command
    return [x,y,altitude,next]
     
def main():
    # Initializing the ROS node.
    rospy.init_node("rps_navigation", anonymous=True)

    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    drone.wait4start()
    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 2m.
    altitude = input("Request altitude : ")
    drone.takeoff(altitude)

    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)
    
    while True:
        next_destination = get_next_destination(altitude) 

        drone.set_destination(next_destination[0],next_destination[1],next_destination[2],0)
        rate.sleep()
    
        if not next_destination[3]:
            break  
 
    # Land after all waypoints is reached.
    drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)
 	
    # Used to keep the node running.
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
