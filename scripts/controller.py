#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from closedloop_control.msg import path

class Controller:
    #Variables
    def __init__(self):
        self.velocity = Twist() #Save the velocities in x, y, z
        self.velocity.linear.x = 0.0 #Linear velocity in x (m/s)
        self.velocity.angular.z = 0.0 #Angular velocity in z (rad/s)

        self.kpt = 0.3 #Translacional proporcional constant
        self.kpr = 1.0 #Rotational proporcional constant
        self.error = 0.05 #Admitible error for the angle and distance

        self.wr = 0.0 #Angular velocity from right wheel (rad/s)
        self.wl = 0.0 #Angular velocity from left wheel (rad/s)
        self.v_max = 0.0 #Linear velocity for the vehicle (m/s)
        self.w_max = 0.0 #Angular velocity for the vehicle (rad/s)
        self.init_time = 0.0 #Initial time (s)
        self.r = 0.05 #Radius of the wheels (m)
        self.l = 0.19 #Distance between wheels (m)

        self.dist = 0.0 #Distance (m)
        self.angle = np.deg2rad(0) #Angle (rad)
        self.angle_desired = 0.0 #For the angle between the current and desired point (rad)
        self.angle_error = 0.0 #Error between angle_desired and angle

        self.x, self.y = 0.0, 0.0 #Current coordinates
        self.xd, self.yd = 0.0, 0.0 #Coordinates desired

        self.reset = None #Flag to get dt
        self.init = False #Flag to start controlling when getting the values from path generator
        self.linear = False #Flag to start moving in a linear trayectory
        self.next = False #Flag to move to the next goal
    
    #Function for getting the goal and velocities from path generator
    def path(self, msg):
        self.xd, self.yd = msg.currGoal.x, msg.currGoal.y
        self.v_max = msg.linear
        self.w_max = msg.angular
        self.init = True #Flag to start the controller
    
    #Functions for getting the angular velocities from the wheels
    def right(self, msg):
        self.wr = msg.data
    def left(self, msg):
        self.wl = msg.data

    #To initialize time
    def initialize_time(self):
        self.init_time = rospy.get_time() #Start variable
        self.reset = True #Change flag

    #To precalculate variables
    def calculations(self, dt):
        #Position during time
        self.x += (self.r * ((self.wr + self.wl)/2) * dt * np.cos(self.angle))
        self.y += (self.r * ((self.wr + self.wl)/2) * dt * np.sin(self.angle))
        self.dist = np.sqrt((self.x - self.xd)**2 + (self.y - self.yd)**2) #Distance desired

        self.angle += (self.r * (self.wr - self.wl) / self.l) * dt #Angle during the time
        self.angle_desired = np.arctan2((self.yd - self.y), (self.xd - self.x)) #Angle desired

        self.angle_error = self.angle - self.angle_desired
        #Keep the angle between pi and -pi
        if (self.angle_error > np.pi):
            self.angle_error -= 2*np.pi
        elif (self.angle_error < -np.pi):
            self.angle_error += 2*np.pi

        self.next = False #Change flag for keeping the current goal

    #Controlling the angular velocity
    def control_angular(self):
        #If the angle error is not within the range (-0.05, 0.05) units
        if ((self.angle_error > self.error or self.angle_error < -self.error)):
            self.velocity.angular.z = -self.kpr * self.angle_error

            #Keep the angular velocity in boundaries according to w_max (-w_max, w_max)
            if (self.velocity.angular.z > 0.0 and self.velocity.angular.z > self.w_max):
                self.velocity.angular.z = self.w_max
            elif (self.velocity.angular.z < 0.0 and self.velocity.angular.z < -self.w_max):
                self.velocity.angular.z = -self.w_max
        else:
            #Reset variables
            self.velocity.angular.z = 0.0

            #Change flag
            if (self.dist > self.error):
                self.linear = True

    def control_linear(self): #Controlling the linear velocity
        if (self.dist > self.error): #If the error in the distance is greater than 5 cm
            self.velocity.linear.x = self.v_max * np.tanh(self.dist * self.kpt / self.v_max) #Sigmoid with v_max as a limit
        else:
            #Reset variables
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0

            #Change flag
            self.linear = False

            #Change to next point/coordinate
            self.next = True

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
    in_velocity = rospy.Publisher("/cmd_vel", Twist, queue_size = 1) #Publish the velocities for the vehicle
    in_next = rospy.Publisher("/nextGoal", Bool, queue_size = 1) #Publish the flag for the path_generator
    rospy.Subscriber("/goal", path, controller.path) #Get the current goal and velocities from the path generator
    rospy.Subscriber("/wr", Float32, controller.right) #Get the angular velocity from the right wheel
    rospy.Subscriber("/wl", Float32, controller.left) #Get the angular velocity from the left wheel

    print("The Control is Running")

    #Run the node
    while not rospy.is_shutdown():
        #Travel to the 4 points
        if (controller.init):
            if (controller.reset == None): #First iteration
                controller.initialize_time()
            elif (controller.reset):
                dt = rospy.get_time() - controller.init_time #Get dt
                controller.reset = False

            if (not controller.reset): #If the flag is false
                controller.calculations(dt)

                #Controlling the vehicle
                if (controller.linear):
                    controller.control_linear() #Control the distance traveled by the vehicle

                    #Adjust the angle during the travel
                    if (controller.dist > 0.2):
                        controller.control_angular()
                    else:
                        controller.velocity.angular.z = 0.0
                else:
                    controller.control_angular() #Control the angle of the vehicle
        
        in_velocity.publish(controller.velocity) #Publish the velocities
        in_next.publish(controller.next) #Publish the current status for the goal

        rospy.loginfo("Linear velocity: %f m/s", controller.velocity.linear.x)
        rospy.loginfo("Angular velocity: %f m/s", controller.velocity.angular.z)
        rate.sleep()
