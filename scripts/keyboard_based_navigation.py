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

    command = raw_input("input command : ")
  
    #get current posisiton
    current_pos = Point()
    current_pos = drone.get_current_location()

    #default value declaration
    x = 0
    y = 0
    z = altitude
    next = True

    #set next destination value
    if (command == 'w'):
        y = 2
    elif (command == 's'):
        y = -2
    elif (command == 'a'):
        x = -2
    elif (command == 'd'):
        x = 2
    else:
        next = False

    #adding current position with the desired position of x and y orientation 
    x = x + current_pos.x
    y = y + current_pos.y

    #data feedback
    #x is move left or right command
    #y is forward or backward command
    #z is altitude. this will set as absolut value as we get from parameter
    #next is continue or stop command. bool data type
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
    # Request altitude for takeoff.
    rospy.loginfo(CYELLOW2 + CBLINK +
                      "Waiting for user to fill request altitude" + CEND)
    altitude = input("Request altitude : ")
    #take off into desired altitude
    drone.takeoff(altitude)

    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)
    
    while True:
	#getting next destination of the drone from keyboard input
        next_destination = get_next_destination(altitude) 
	#make drone fly into desired position
        drone.set_destination(next_destination[0],next_destination[1],next_destination[2],0)
        rate.sleep()
    
        if not next_destination[3]:
            break  
 
    # Land after all waypoints is reached.
    drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
