#!/usr/bin/env python
import rospy
import numpy as np
from openloop_control.msg import path

# Variables
msg = path()

# Stop Condition
def stop():
    # Stop message
    print("Stopping")

if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("Path_Generator")
    rospy.on_shutdown(stop)

    hz = 100 # Frequency (Hz)
    rate = rospy.Rate(hz)

    # Publishers and subscribers
    path_pub = rospy.Publisher("/pose", path, queue_size=1)

    print("The Path Generator is Running")
    # Run the node
    while not rospy.is_shutdown():
        # Prepare variables for publish the custom message
        msg.distances = []
        msg.angles = []
        msg.linear = rospy.get_param("/linear")
        msg.angular = rospy.get_param("/angular")
	
	    # Get parameters from the parameter file
        isTime = rospy.get_param("/isTime")
        time = rospy.get_param("/time")
        points = rospy.get_param("/points")

	    # Initialize variables for the distance and angle calculations
        position = [0, 0]
        point = 0
        while point < 4: # Calculate for the 4 points
            xd, yd = points[point][0], points[point][1] # Coordinates desired
            x, y = position[0], position[1] # Current coordinates

            dist = np.sqrt((x-xd)**2 + (y-yd)**2) # Distance desired
            angle = np.arctan2((yd-y), (xd-x)) # Angle desired
            if (angle > np.pi): # Get the smaller angle required
                angle -= -2*np.pi
            elif (angle < -np.pi):
                angle += 2*np.pi
	    
            msg.distances.append(dist)
            msg.angles.append(angle)

            if isTime: # If the user introduce time instead of velocities
                msg.linear[point] = dist / time[point]
                msg.angular[point] = 0.5
            
	    # If the velocity is not viable for the vehicle
            if (msg.linear[point] > 15.0 or msg.linear[point] < -15.0):
                rospy.loginfo("Linear velocity is not viable for the puzzlebot")
                msg.linear[point] = 0.0
            if (msg.angular[point] > 3.2 or msg.angular[point] < -3.2):
                rospy.loginfo("Angular velocity is not viable for the puzzlebot")
                msg.angular[point] = 0.0

            position = points[point]
            point += 1
	
	    # Publish the custom message
        path_pub.publish(msg)
        rate.sleep()