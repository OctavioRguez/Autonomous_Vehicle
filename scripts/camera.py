#!/usr/bin/env python
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image

class Camera:
    # Variables
    def __init__(self):
        self.__cam_port =  'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=10/1 ! nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true'
        self.__cam = cv2.VideoCapture(self.__cam_port)
        self.__bridge = cv_bridge.CvBridge() # Bridge to conversion between imgmsg and cv2

    # Function for starting the camera
    def startCamera(self):
        self.__frame = self.__cam.read()[1] # Get actual frame
        self.__frame = cv2.cvtColor(self.__frame, cv2.COLOR_BGR2RGB)
        self.__frame = cv2.GaussianBlur(self.__frame, (5,5), 0) # Apply gaussian blur

    # Transform to image from cv2 to Image msg for publishing
    def getImage(self):
        return self.__bridge.cv2_to_imgmsg(self.__frame, encoding = 'rgb8')

    # Function for releasing the camera
    def releaseCamera(self):
        self.__cam.release()
        print("Stopping") # Stop message

if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("Camera")

    hz = 10 # Frequency (Hz)
    rate = rospy.Rate(hz)

    camera = Camera() # Camera class object

    # Publishers and subscribers
    camera_pub = rospy.Publisher("/video_source/raw", Image, queue_size = 1) # Publish the camera image

    print("The Camera is Running")
    rospy.on_shutdown(camera.releaseCamera)

    # Run the node
    while not rospy.is_shutdown():
        camera.startCamera() # Get the camera image

        camera_pub.publish(camera.getImage()) # Publish the current frame
        rate.sleep()
