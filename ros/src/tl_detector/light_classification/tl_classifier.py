from styx_msgs.msg import TrafficLight
import cv2 # TODO: needed?
import numpy as np
import rospy

# tensorflow imports
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile

from collections import defaultdict
from io import StringIO
from os.path import dirname, abspath 
from matplotlib import pyplot as plt
from PIL import Image
# TODO: provide utils module!!! pow pow pow
from utils import label_map_util
from utils import visualization_utils as vis_util

PATH = os.path.dirname(os.path.realpath(__file__))
PATH_TO_CKPT = os.path.join(PATH,"red_inference_graph/frozen_inference_graph.pb")
PATH_TO_LABELS = os.path.join(PATH, "training/object-detection.pbtxt")  
NUM_CLASSES = 1 # y u no two classes but one? Christian: If class "RED" is detectet, stop the car at the tl?

#Simulator ToDo: Get Images from Simulator
#PATH_TO_TEST_IMAGES_DIR = 'test_images/red' #
#TEST_IMAGE_PATHS = [os.path.join(PATH_TO_TEST_IMAGES_DIR, 'out000{}.png'.format(2*i)) for i in range(24, 26)]
#image = Image.open(image_path)

# Size, in inches, of the output images.
IMAGE_SIZE = (12, 8) #Not necessary?

class TLClassifier(object):
    def __init__(self):
        # load dem graph
        # TODO: is there something we should save to self?

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        category_index = label_map_util.create_category_index(categories)
	#self.dbg_idx = 0

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        state = TrafficLight.UNKNOWN #default to unknown 
        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                # Definite input and output Tensors for self.detection_graph
                image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                # Each box represents a part of the image where a particular object was detected.
                detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                # Each score represent the level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

                # the array based representation of the image will be used later in order to prepare the
                # result image with boxes and labels on it.
		# image fromcv2 format already in numpy ndarray
                image_np = image
                # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                image_np_expanded = np.expand_dims(image_np, axis=0)
                # Actual detection.
                (boxes, scores, classes, num) = sess.run(
                     [detection_boxes, detection_scores, detection_classes, num_detections],
                     feed_dict={image_tensor: image_np_expanded})
                if max(scores[0]) > 0.5:
                      state = TrafficLight.RED #return state?
                #     # Visualization of the results of a detection.
                #     vis_util.visualize_boxes_and_labels_on_image_array(
                #         image_np,
                #         np.squeeze(boxes),
                #         np.squeeze(classes).astype(np.int32),
                #         np.squeeze(scores),
                #         category_index,
                #         use_normalized_coordinates=True,
                #         line_thickness=8)
                #     plt.figure(figsize=IMAGE_SIZE)
                #     plt.imshow(image_np)

        # TODO: return dem labels
        #rospy.logerr("BAAAACOOOOOONNNNN!!!!!")
        return state

