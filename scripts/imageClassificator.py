#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import cv_bridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

class Classificator:
    # Variables
    def __init__(self):
        self.__INPUT_WIDTH = 640
        self.__INPUT_HEIGHT = 640

        self.prueba = Image()
        self.__obj = "None"
        self.__bridge = cv_bridge.CvBridge() # Bridge to conversion between imgmsg and cv2

        self.__class_list = ["Construction", "Giveway", "Green", "Left", "Red", "Right", "Stop", "Straight", "Yellow"]
        #self.__class_list = ["forward", "giveway", "greenlight", "left", "redlight", "right", "round", "stop", "working", "yellowlight"]
        self.__colors = [(255, 0, 0), (0, 0, 125), (0, 255, 0), (255, 125, 125),(0, 0, 255), (255, 200, 200), (238, 174, 245), (0, 0, 200),(0, 0, 125), (245, 227, 66)]

        self._flag = False

    # Callback to receive the frames from camera
    def _retrieveImage(self, msg):
        try:
            # Conversion from imgmsg to cv2
            self.__img = self.__bridge.imgmsg_to_cv2(msg, desired_encoding = 'passthrough')
            self._flag = True
        except cv_bridge.CvBridgeError as e:
            rospy.loginfo(e) # Catch an error

    def __build_model(self, is_cuda):
        net = cv2.dnn.readNet("/home/puzzlebot/catkin_ws/src/final_puzzlebot/model/best.onnx")
        if is_cuda:
            print("Attempty to use CUDA")
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        else:
            print("Running on CPU")
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        return net
    
    def __detect(self, image, net):
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (self.__INPUT_WIDTH, self.__INPUT_HEIGHT), swapRB=True, crop=False)
        net.setInput(blob)
        preds = net.forward()
        return preds

    def __wrap_detection(self, input_image, output_data):
        class_ids = []
        confidences = []
        boxes = []

        rows = output_data.shape[0]

        image_width, image_height, _ = input_image.shape

        x_factor = image_width / self.__INPUT_WIDTH
        y_factor =  image_height / self.__INPUT_HEIGHT

        for r in range(rows):
            row = output_data[r]
            confidence = row[4]
            if confidence >= 0.4:

                classes_scores = row[5:]
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
                class_id = max_indx[1]
                if (classes_scores[class_id] > .25):

                    confidences.append(confidence)

                    class_ids.append(class_id)

                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                    left = int((x - 0.5 * w) * x_factor)
                    top = int((y - 0.5 * h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45) 

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes

    def __format_yolov5(self, img):
        result = cv2.resize(img, [640, 640], interpolation = cv2.INTER_AREA)
        return result
    
    def startDetection(self):
        self.__net = self.__build_model(1) #Define si opencv se corre con CUDA o con CPU 0 = CPU, 1 = CUDA 
        inputImage = self.__format_yolov5(self.__img)
        outs = self.__detect(inputImage, self.__net)
        class_ids, confidences, boxes = self.__wrap_detection(inputImage, outs[0])
        for (classid, confidence, box) in zip(class_ids, confidences, boxes):
            color = self.__colors[int(classid) % len(self.__colors)]
            cv2.rectangle(inputImage, box, color, 2)
            cv2.rectangle(inputImage, (box[0], box[1] - 20), (box[0] + box[2], box[1]), color, -1)
            cv2.putText(inputImage, self.__class_list[classid], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))
            self.__obj = self.__class_list[classid]
        
        self.prueba = self.__bridge.cv2_to_imgmsg(inputImage, encoding = 'rgb8')
    
    def getObject(self):
        return self.__obj

# Stop Condition
def stop():
    print("Stopping") # Stop message

if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("Classificator")
    rospy.on_shutdown(stop)

    hz = 10 # Frequency (Hz)
    rate = rospy.Rate(hz)

    classificator = Classificator() # Classificator class object

    # Publishers and subscribers
    rospy.Subscriber("/video_source/raw", Image, classificator._retrieveImage) # Get the image from the camera

    object_pub = rospy.Publisher("/object", String, queue_size = 1)
    prueba_pub = rospy.Publisher("/detect/raw", Image, queue_size = 1)

    print("The Classificator is Running")

    # Run the node
    while not rospy.is_shutdown():
        if (classificator._flag):
            classificator.startDetection()
            object_pub.publish(classificator.getObject())
	        prueba_pub.publish(classificator.prueba)

        rate.sleep()
