#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist

class Controller:
    # Variables
    def __init__(self):
        self.__velocity = Twist() # Save the velocities in x, y, z
        self.__velocity.linear.x = 0.0 # Linear velocity in x (m/s)
        self.__velocity.angular.z = 0.0 # Angular velocity in z (rad/s)

        self.__kpt = 0.05 # Translacional proporcional constant
        self.__kpr = 2.0 # Rotational proporcional constant
        self.__error = 0.0 # Error in the line following
        self.__errorTolerance = 4.0 # Admitible error for the angle and distance
        self.__obj = "None" # Object detected
        
        self.__vMax = 0.15 # Linear velocity for the vehicle (m/s)
        self.__wMax = 0.5 # Angular velocity for the vehicle (rad/s)

        self._curve = False # Flag when the vehicle detects a curve
        self._init = False # Flag to start controlling when getting the values from path generator

    def _objectCallback(self, msg):
        self.__obj = msg.data

    def _errorCallback(self, msg):
        self.__error = msg.data
        self._init = True

    def __changeCurve(self, event):
        self._curve = False

    def detectCurve(self):
        if (not self._curve):
            self.__kpt = 0.05

        if (self.__error > 40.0 or self.__error < -40.0):
            self._curve = True
            self.__kpt = 0.025
            rospy.Timer(rospy.Duration(10), self.__changeCurve)
        
    # Controlling the angular velocity
    def control_angular(self):
        # If the error from the line is not within the range (-0.05, 0.05) units
        if (self.__error > self.__errorTolerance or self.__error < -self.__errorTolerance):
            self.__velocity.angular.z = self.__wMax * np.tanh(self.__error * self.__kpr / self.__wMax)
            """
            if (self.__color == "Green"):
                # Sigmoid with w_max as a limit
                self.__vMax = 0.1
                self.__velocity.angular.z = self.__wMax * np.tanh(self.__error * self.__kpr / self.__wMax)
            elif (self.__color == "Yellow"):
                self.__velocity.angular.z = self.__wMax/2
            else:
                self.__velocity.angular.z = 0.0
            """
        else:
            # Reset variables
            self.__velocity.angular.z = 0.0

    # Controlling the linear velocity
    def control_linear(self):
        self.__velocity.linear.x = self.__vMax * np.tanh(self.__kpt / self.__vMax)
        """
        if (self.__color == "Green"):
            # Sigmoid with v_max as a limit
            self.__velocity.linear.x = self.__vMax * np.tanh(self.__vMax * self.__kpt / self.__vMax)
        elif (self.__color == "Yellow"):
            self.__velocity.linear.x = self.__vMax/2
        else:
            self.__velocity.linear.x = 0.0
        """

    # Access needed for class attributes
    def getVelocity(self):
        return self.__velocity

    def resetVelocities(self):
        self.__velocity.linear.x = 0.0
        self.__velocity.angular.z = 0.0

# Stop Condition
def stop():
    # Stop message
    print("Stopping")

if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("Controller")
    rospy.on_shutdown(stop)

    hz = 10 # Frequency (Hz)
    rate = rospy.Rate(hz)

    controller = Controller() # Controller class object

    # Publishers and subscribers
    in_velocity = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)# Publish the velocities for the vehicle

    rospy.Subscriber("/object", String, controller._objectCallback)
    rospy.Subscriber("/error", Float32, controller._errorCallback)

    print("The Controller is Running")

    # Run the node
    while not rospy.is_shutdown():
        # Start following the line
        if (controller._init):
            controller.detectCurve()
            controller.control_linear()
            controller.control_angular()
        else:
            controller.resetVelocities()
		
        in_velocity.publish(controller.getVelocity()) # Publish the velocities
        rate.sleep()
