#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from closedloop_control.msg import path

class Controller:
    # Variables
    def __init__(self):
        self.__velocity = Twist() # Save the velocities in x, y, z
        self.__velocity.linear.x = 0.0 # Linear velocity in x (m/s)
        self.__velocity.angular.z = 0.0 # Angular velocity in z (rad/s)

        self.__kpt = 1.0 # Translacional proporcional constant
        self.__kpr = 1.0 # Rotational proporcional constant
        self.__error = 0.05 # Admitible error for the angle and distance

        self.__wr = 0.0 # Angular velocity from right wheel (rad/s)
        self.__wl = 0.0 # Angular velocity from left wheel (rad/s)
        self.__vMax = 0.0 # Linear velocity for the vehicle (m/s)
        self.__wMax = 0.0 # Angular velocity for the vehicle (rad/s)
        self.__init_time = 0.0 # Initial time (s)
        self.__r = 0.05 # Radius of the wheels (m)
        self.__l = 0.19 # Distance between wheels (m)

        self.__dist = 0.0 # Distance (m)
        self.__angle = np.deg2rad(0) # Angle (rad)
        self.__angle_desired = 0.0  # For the angle between the current and desired point (rad)
        self.__angle_error = 0.0 # Error between angle_desired and angle

        self.__x, self.__y = 0.0, 0.0 # Current coordinates
        self.__xd, self.__yd = 0.0, 0.0 # Coordinates desired

        self._reset = None # Flag to get dt
        self._init = False # Flag to start controlling when getting the values from path generator
        self._linear = False # Flag to start moving in a linear trayectory
        self._next = False # Flag to move to the next goal

	self.__alpha = 0.1
    
    # Function for getting the goal and velocities from path generator
    def _pathCallback(self, msg):
        self.__xd, self.__yd = msg.currGoal.x, msg.currGoal.y
        self.__vMax = msg.linear
        self.__wMax = msg.angular
        self._init = True # Flag to start the controller
    
    # Functions for getting the angular velocities from the wheels
    def _rightCallback(self, msg):
        self.__wr = (1 - self.__alpha) * self.__wr + self.__alpha * msg.data

    def _leftCallback(self, msg):
        self.__wl = (1 - self.__alpha) * self.__wl + self.__alpha * msg.data

    # To initialize time
    def _initialize_time(self):
        self.__init_time = rospy.get_time() # Start variable
        self._reset = True # Change flag

    # To precalculate variables
    def _calculations(self, dt):
        # Position during time
        self.__x += (self.__r * ((self.__wr + self.__wl)/2) * dt * np.cos(self.__angle))
        self.__y += (self.__r * ((self.__wr + self.__wl)/2) * dt * np.sin(self.__angle))
        self.__dist = np.sqrt((self.__x - self.__xd)**2 + (self.__y - self.__yd)**2) # Distance desired

        self.__angle += (self.__r * (self.__wr - self.__wl) / self.__l) * dt # Angle during the time
        self.__angle_desired = np.arctan2((self.__yd - self.__y), (self.__xd - self.__x)) # Angle desired

	rospy.loginfo("Angle: %f rad", self.__angle)

        self.__angle_error = self.__angle - self.__angle_desired
        # Keep the angle between pi and -pi
        if (self.__angle_error > np.pi):
            self.__angle_error -= 2*np.pi
        elif (self.__angle_error < -np.pi):
            self.__angle_error += 2*np.pi

        self._next = False # Change flag for keeping the current goal

    # Controlling the angular velocity
    def control_angular(self):
        # If the angle error is not within the range (-0.05, 0.05) units
        if ((self.__angle_error > self.__error or self.__angle_error < -self.__error)):
	    # Sigmoid with w_max as a limit
            self.__velocity.angular.z = self.__wMax * np.tanh(-self.__angle_error * self.__kpr / self.__wMax) 
        else:
            # Reset variables
            self.__velocity.angular.z = 0.0

            # Change flag
            if (self.__dist > self.__error):
                self._linear = True

    def control_linear(self): # Controlling the linear velocity
        if (self.__dist > self.__error): # If the error in the distance is greater than 5 cm
	    # Sigmoid with v_max as a limit
            self.__velocity.linear.x = self.__vMax * np.tanh(self.__dist * self.__kpt / self.__vMax)
        else:
            # Reset variables
            self.__velocity.linear.x = 0.0
            self.__velocity.angular.z = 0.0

            # Change flag
            self._linear = False

            # Change to next point/coordinate
            self._next = True

    # Access needed for class attributes
    def getInitTime(self):
	return self.__init_time

    def getDistance(self):
	return self.__dist

    def getVelocity(self):
	return self.__velocity

# Stop Condition
def stop():
    # Stop message
    print("Stopping")

if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("Controller")
    rospy.on_shutdown(stop)

    hz = 91 # Frequency (Hz)
    rate = rospy.Rate(hz)

    controller = Controller() # Controller class object

    # Publishers and subscribers
    in_velocity = rospy.Publisher("/cmd_vel", Twist, queue_size = 91) # Publish the velocities for the vehicle
    in_next = rospy.Publisher("/nextGoal", Bool, queue_size = 91) # Publish the flag for the path_generator

    rospy.Subscriber("/goal", path, controller._pathCallback) # Get the current goal and velocities from the path generator
    rospy.Subscriber("/wr", Float32, controller._rightCallback) # Get the angular velocity from the right wheel
    rospy.Subscriber("/wl", Float32, controller._leftCallback) # Get the angular velocity from the left wheel

    print("The Controller is Running")

    # Run the node
    while not rospy.is_shutdown():
        # Travel to the 4 points
        if (controller._init):
            if (controller._reset == None): # First iteration
                controller._initialize_time()
            elif (controller._reset):
                dt = rospy.get_time() - controller.getInitTime() # Get dt
                controller._reset = False

            if (not controller._reset): # If reset is false
                controller._calculations(dt)

                # Controlling the vehicle
                if (controller._linear):
                    controller.control_linear() # Control the distance traveled by the vehicle

                    # Adjust the angle during the travel
		    """
                    if (controller.getDistance() > 0.2):
                        controller.control_angular()
                    else:
                        controller.getVelocity().angular.z = 0.0
		    """
                else:
                    controller.control_angular() # Control the angle of the vehicle
        
        in_velocity.publish(controller.getVelocity()) # Publish the velocities
        in_next.publish(controller._next) # Publish the current status for the goal

        #rospy.loginfo("Linear velocity: %f m/s", controller.getVelocity().linear.x)
        #rospy.loginfo("Angular velocity: %f rad/s", controller.getVelocity().angular.z)
        rate.sleep()
