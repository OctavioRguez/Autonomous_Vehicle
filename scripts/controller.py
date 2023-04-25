#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from openloop_control.msg import path

class Controller:
    #Variables
    def __init__(self):
        self.velocity = Twist() #Save the velocities in x, y, z
        self.velocity.linear.x = 0.0 #Linear velocity in x (m/s)
        self.velocity.angular.z = 0.0 #Angular velocity in z (rad/s)

        self.v_max = 0.0 #Linear velocity for the vehicle (m/s)
        self.w_max = 0.0 #Angular velocity for the vehicle (rad/s)
        self.init_time = 0.0 #Initial time (s)
        self.r = 0.05 #Radius of the wheels (m)
        self.l = 0.18 #Distance between wheels (m)

        self.dist = 0.0 #Distance (m)
        self.angle = np.deg2rad(90) #Angle (rad)
        self.dist_desired = 0.0 #For the distance between the current and desired point (m)
        self.angle_desired = 0.0 #For the angle between the current and desired point (rad)
        self.dist_error = 0.0 #Error between dist_desired and dist
        self.angle_error = 0.0 #Error between angle_desired and angle

        self.point_number = 0 #For iterate along the points list
        self.flag = None #Flag to reset init_time
        self.linear = False #Flag to move in a linear direction
        self.next = True #Flag to move to the next point
        self.init = False #Flag to start controlling when getting the values from path generator

        #For getting the values from the path generator node
        self.angles = []
        self.distances = []
        self.linearV = []
        self.angularV = []
    
    #Function for callback
    def callback(self, msg):
        self.angles = msg.angles
        self.distances = msg.distances
        self.linearV = msg.linear
        self.angularV = msg.angular
        self.init = True #Flag to start the controller

    #To initialize time
    def initialize_time(self):
        if (self.flag):
            self.init_time = rospy.get_time() #Restart the variable
            self.flag = False #Change flag

    #To precalculate variables
    def precalculations(self):
        if (self.next and self.init):
            #Create variables for simplifying the calculations
            self.dist_desired = self.distances[self.point_number]
            self.angle_desired = self.angles[self.point_number]
            self.v_max = self.linearV[self.point_number]
            self.w_max = self.angularV[self.point_number]
            self.next = False #Change flag

    #Controlling the angular velocity
    def control_angular(self, dt):
        theta = (self.velocity.angular.z * dt) / 2.0 #Angle during the time
        self.angle_error = self.angle_desired - (self.angle + theta)

        if (self.angle_error > 0.04): #If the angle is smaller than the angle desired
            self.velocity.angular.z = self.w_max
        elif (self.angle_error < -0.04): #If the angle is bigger than the angle desired
            self.velocity.angular.z = -self.w_max
        else:
            #Reset variables
            self.angle = self.angle_desired
            self.velocity.angular.z = 0.0

            #Change flags
            self.flag = True
            self.linear = True

    def control_linear(self, dt): #Controlling the linear velocity
        self.dist_error = self.dist_desired - self.dist
        if (self.dist_error > 0.1): #If the error is greater than 2 cm
            self.velocity.linear.x = self.v_max
        else:
            #Reset variables
            self.velocity.linear.x = 0.0

            #Change flags
            self.flag = True
            self.linear = False
            self.next = True

            #Change to next point/coordinate
            self.point_number += 1
        
        self.dist = self.velocity.linear.x * dt #Distance during the time

#Stop Condition
def stop():
    #Stop message
    print("Stopping")

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Controller")
    rospy.on_shutdown(stop)

    hz = 100 #Frequency (Hz)
    rate = rospy.Rate(hz)
    controller = Controller() #Controller class object

    #Publishers and subscribers
    in_velocity = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("/pose", path, controller.callback)

    print("The Control is Running")

    #Run the node
    while not rospy.is_shutdown():
        #Travel to the 4 points
        if (controller.point_number < 4 and controller.init):
            if (controller.flag == None): #First iteration
                controller.flag = True
            else:
                controller.initialize_time()

            if (not controller.flag): #If the flag is false
                dt = rospy.get_time() - controller.init_time
                controller.precalculations()

                #For controlling only one velocity per execution
                if (controller.linear):
                    controller.control_linear(dt)
                else:
                    controller.control_angular(dt)
        
        in_velocity.publish(controller.velocity) #Publish the velocities
        rospy.loginfo("Linear velocity: %f m/s", controller.velocity.linear.x)
        rospy.loginfo("Angular velocity: %f m/s", controller.velocity.angular.z)

        rate.sleep()
