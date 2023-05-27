#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Bool
from closedloop_control.msg import path

# Create a variable for the custom message
goal = path()

# Initialize the goal values
goal.currGoal.x = 0.0
goal.currGoal.y = 0.0

# Initialize the velocities
goal.linear = 0.0
goal.angular = 0.0

position = [0, 0] # Current position
point = 0 # For iterate along the points received

# Stop Condition
def stop():
    # Stop message
    print("Stopping")

# For getting the flag to change the current goal
def callback(msg):
    global next
    next = msg.data

if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("Path_Generator")
    rospy.on_shutdown(stop)

    hz = 91 # Frequency (Hz)
    rate = rospy.Rate(hz)

    # Publishers and subscribers
    path_pub = rospy.Publisher("/goal", path, queue_size = 91) # Publish the current goal and velocities to the controller
    rospy.Subscriber("/nextGoal", Bool, callback) # Get the flag from the controller

    print("The Path Generator is Running")
    # Run the node
    while not rospy.is_shutdown():
	# Get parameters from the parameter file
        points = rospy.get_param("/points")
        isTime = rospy.get_param("/isTime")
        time = rospy.get_param("/time")
        Vlinear = rospy.get_param("/linear")
        Vangular = rospy.get_param("/angular")
        
        # For moving to the next goal
        if next == True:
            if point < len(points):
                position = points[point]
            else:
                position = [0.0, 0.0]
            point += 1
            next = False # Change flag
        
        if (point < len(points)+1):
            # Get the current goal for the custom message
            if (point < len(points)):
                goal.currGoal.x = points[point][0]
                goal.currGoal.y = points[point][1]
            else:
                goal.currGoal.x = 0.0
                goal.currGoal.y = 0.0

            # Get the velocities for the custom message
            goal.linear = Vlinear[point]
            goal.angular = Vangular[point]

            # If the user introduce time instead of velocities
            if (isTime):
                xd, yd = goal.currGoal.x, goal.currGoal.y # Coordinates desired
                x, y = position[0], position[1] # Current coordinates
                dist = np.sqrt((x-xd)**2 + (y-yd)**2) # Distance

                # Calculate the velocities according to the time
                goal.linear = dist / time[point]
                goal.angular = Vangular[point]
            
            # If the velocity is not viable for the vehicle
            if (goal.linear > 15.0 or goal.linear < -15.0):
                rospy.loginfo("Linear velocity is not viable for the puzzlebot")
                goal.linear = 0.0
            if (goal.angular > 3.2 or goal.angular < -3.2):
                rospy.loginfo("Angular velocity is not viable for the puzzlebot")
                goal.angular = 0.0
        else:
            goal.currGoal.x = 0.0
            goal.currGoal.y = 0.0
            goal.linear = 0.0
            goal.angular = 0.0
	
	# Publish the custom message
        path_pub.publish(goal)
        rate.sleep()
