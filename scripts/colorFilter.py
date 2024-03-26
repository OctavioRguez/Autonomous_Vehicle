#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import cv_bridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

class ImageColorFilter:
    # Variables
	def __init__(self):
		self._flag = False # Flag to start once the first frame is received
		self.__bridge = cv_bridge.CvBridge() # Bridge to conversion between imgmsg and cv2

		self.__color = "None" # Save the current color detected
		self.__msgRedFrame = Image() # Frame for red light detection
		self.__msgGreenFrame = Image() # Frame for green light detection
		self.__msgYellowFrame = Image() # Frame for green light detection

		# HSV values for red
		self.__redLowRange = np.array([0, 100, 20])
		self.__redHighRange = np.array([20, 255, 255])

		# HSV values for green
		self.__greenLowRange = np.array([36, 100, 100])
		self.__greenHighRange = np.array([75, 255, 255])

		# HSV values for yellow
		self.__yellowLowRange = np.array([20, 100, 100])
		self.__yellowHighRange = np.array([30, 255, 255])

    # Callback to receive the frames from camera
	def _retrieveImage(self, msg):
		try:
			# Conversion from imgmsg to cv2
			self.__img = self.__bridge.imgmsg_to_cv2(msg, desired_encoding = 'passthrough')
			# Change flag to start the Image processing
			self._flag = True
		except cv_bridge.CvBridgeError as e:
			rospy.loginfo(e) # Catch an error

    # Apply color filtering (red, yellow or green)
	def __colorFilter(self, lowRange, highRange):
		hsv = cv2.cvtColor(self.__img, cv2.COLOR_BGR2HSV)

		# HSV values for the desired color
		mask = cv2.inRange(hsv, lowRange, highRange)

		# Apply mask to the current frame
		img_Filtered = cv2.bitwise_and(self.__img, self.__img, mask = mask)

		# Transform to grayscale
		img_Binary = cv2.cvtColor(img_Filtered, cv2.COLOR_BGR2GRAY)

		# Create kernel for processing
		kernel = np.ones((3, 3), np.uint8)

		# Apply morphological operations
		img_eroded = cv2.erode(img_Binary, kernel, iterations = 1) #Erode
		img_Processed = cv2.morphologyEx(img_eroded, cv2.MORPH_CLOSE, kernel, iterations = 8) #Close

		# Final threshold
		threshB = cv2.threshold(img_Processed, 1, 255, cv2.THRESH_BINARY_INV)[1]
		return threshB

	def _blobDetection(self):
		# Define parameters for the blob detection
		params = cv2.SimpleBlobDetector_Params()
		params.filterByArea = True
		params.minArea = 20000
		params.maxArea = 60000
		params.filterByCircularity = True
		params.minCircularity = 0.7
		params.filterByConvexity = False
		params.filterByInertia = False

		# Create the object for the blob detection with the parameters defined before
		detector = cv2.SimpleBlobDetector_create(params)

		# Apply color filtering
		frameRed = self.__colorFilter(self.__redLowRange, self.__redHighRange)
		frameGreen = self.__colorFilter(self.__greenLowRange, self.__greenHighRange)
		frameYellow = self.__colorFilter(self.__yellowLowRange, self.__yellowHighRange)

		# Detect blobs in filtered image
		keypointsRed = detector.detect(frameRed)
		keypointsGreen = detector.detect(frameGreen)
		keypointsYellow = detector.detect(frameYellow)
		
		# Check if there are blobs detected
		if (keypointsRed):
			self.__color = "Red"
		elif (keypointsGreen):
			self.__color = "Green"
		elif (keypointsYellow):
			self.__color = "Yellow"
		else:
			self.__color = "None"
		
		rospy.loginfo(self.__color)

		# Draw blobs in the frames
		redBlobs = cv2.drawKeypoints(frameRed, keypointsRed, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		greenBlobs = cv2.drawKeypoints(frameGreen, keypointsGreen, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		yellowBlobs = cv2.drawKeypoints(frameYellow, keypointsYellow, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

		#Convert the images from cv2 to imgmsg
		self.__msgRedFrame = self.__bridge.cv2_to_imgmsg(redBlobs, encoding = 'rgb8')
		self.__msgGreenFrame = self.__bridge.cv2_to_imgmsg(greenBlobs, encoding = 'rgb8')
		self.__msgYellowFrame = self.__bridge.cv2_to_imgmsg(yellowBlobs, encoding = 'rgb8')

    #For accesing class attributes
	def getColor(self):
		return self.__color

	def getRedFrame(self):
		return self.__msgRedFrame

	def getGreenFrame(self):
		return self.__msgGreenFrame

	def getYellowFrame(self):
		return self.__msgYellowFrame

# Stop Condition
def stop():
    # Stop message
    print("Stopping")

if __name__=='__main__':
	# Initialise and Setup node
	rospy.init_node("Color_Filtering")
	rospy.on_shutdown(stop)

	hz = 2 # Frequency (Hz)
	rate = rospy.Rate(hz)

	image = ImageColorFilter() # Image class object

	# Publishers and subscribers
	rospy.Subscriber("/video_source/raw", Image, image._retrieveImage) # Get the image from the camera

	red_pub = rospy.Publisher("/redFilter/raw", Image, queue_size = 2) # Publish the red filter image
	green_pub = rospy.Publisher("/greenFilter/raw", Image, queue_size = 2) # Publish the green filter image
	yellow_pub = rospy.Publisher("/yellowFilter/raw", Image, queue_size = 2) # Publish the green filter image
	color_pub = rospy.Publisher("/color", String, queue_size = 2) # Publish the color detected with the filters

	print("The Color Filtering is Running")

	# Run the node
	while not rospy.is_shutdown():
		# Start blob detection
		if (image._flag):
			image._blobDetection()
        
		red_pub.publish(image.getRedFrame()) # Publish the red filter
		green_pub.publish(image.getGreenFrame()) # Publish the green filter
		yellow_pub.publish(image.getYellowFrame()) # Publish the green filter
		color_pub.publish(image.getColor()) # Publish the color detected

		rate.sleep()