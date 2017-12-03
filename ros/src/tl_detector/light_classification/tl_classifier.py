from styx_msgs.msg import TrafficLight
import cv2
import numpy as np


class TLClassifier(object):
    def __init__(self):
        # TODO load classifier
        self.RED_MIN = np.array([0, 180, 180], np.uint8)
        self.RED_MAX = np.array([10, 255, 255], np.uint8)
        self.THRESHOLD = 60

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction
        # convert image to HSV
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        frame_threshed = cv2.inRange(img_hsv, self.RED_MIN, self.RED_MAX)
        count = cv2.countNonZero(frame_threshed)
        if (count > self.THRESHOLD):
            return TrafficLight.RED
        else:
            return TrafficLight.UNKNOWN
