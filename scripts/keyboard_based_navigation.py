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
    print ("\tW\t\t\t\tI")
    print ("A\t\tD\t\tJ\t\tL")
    print ("\tS\t\t\t\tK")
    #print ("  Q\tE")
    print ("Note : ")
    print ("W : move forward")
    print ("A : move left")
    print ("D : move right")
    print ("S : move backward")
    print ("I : up")
    print ("J : turn left")
    print ("L : turn right")
    print ("K : down")
    print ("space : RTL")
    print ("... : land and disarm motor ")

    command = raw_input("input command : ")
  
    #get current posisiton
    current_pos = Point()
    current_pos = drone.get_current_location()
    #get current heading
    current_heading = drone.get_current_heading()

    #default value declaration on position and heading variable
    x = 0
    y = 0
    z = 0
    psi = 0
    next = True

    #give new value to variable as user input
    if (command == 'w'):
        y = 2
    elif (command == 's'):
        y = -2
    elif (command == 'a'):
        x = -2
    elif (command == 'd'):
        x = 2
    elif (command == 'i'):
        z = 1
    elif (command == 'k'):
        z = -1
    elif (command == 'j'):
        psi = 90
    elif (command == 'l'):
        psi = -90
    elif (command == chr(32)): #32 is ascii code of space button
        drone.set_mode("RTL")
        next = False
    else:
        next = False

    #adjusting variable value based on quadcopter heading value
    #if ((current_heading < 5 and current_heading > -5) or (current_heading < -175 and current_heading > -185)):
    if ((current_heading < -175 and current_heading > -185) or (current_heading < 185 and current_heading > 175)):
        x = x*-1
        y = y*-1
    elif (current_heading < 95 and current_heading > 85):
        temp = x
        x = y*-1
        y = temp
    elif (current_heading < -85 and current_heading > -95):
        temp = x
        x = y
        y = temp*-1

    #finishing value of next destination
    x = x + current_pos.x
    y = y + current_pos.y
    z = z + altitude
    psi = psi + current_heading
    #set heading into 90 or -90 if current heading value is 270 or -270
    if (psi < 265 and psi > 275): 
        psi = -90
    elif (psi < -265 and psi > -275):
        psi = 90
    print ("x direction : {}, y direction : {}, altitude : {}, heading : {}".format(x,y,z,psi))

    #data feedback
    #x is move left or right command
    #y is forward or backward command
    #z is altitude. absolut value
    #next is continue or stop command
    return [x,y,z,psi,next]
     
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
        #get next destination value
        next_destination = get_next_destination(altitude) 
        #display all of next destination value
        #print ("x direction : {}, y direction : {}, altitude : {}, angle : {}".format(next_destination[0],next_destination[1],next_destination[2],next_destination[3]))

        drone.set_destination(next_destination[0],next_destination[1],next_destination[2],next_destination[3])
        rate.sleep()
        #update altitude value with the new one
        altitude = next_destination[2]

        if not next_destination[4]:
            break  
 
    # Land after all waypoints is reached.
    drone.land()
    rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)
    #shutdown node
    rospy.signal_shutdown("KeyboardInterupt") 

if __name__ == '__main__':
    try:
        main()
        #used to keep the node running (looping)
        rospy.spin()
    except KeyboardInterrupt:
        #rospy.signal_shutdown("KeyboardInterupt")
        exit()
