#!/usr/bin/env python
import rospy

from styx_msgs.msg import TrafficLight
import tensorflow as tf
import os
from os import path
#from PIL import Image
import numpy as np
import cv2
import sys

class TLClassifier(object):
    def __init__(self):

        rospy.loginfo( " TLClassifier Start")

        PATH_TO_CKPT = rospy.get_param('~model_path')

        # List of the strings that is used to add correct label for each box.
        #PATH_TO_LABELS = 'mscoco_label_map.pbtxt'

        # number of classes for COCO dataset
        NUM_CLASSES = 90
        self.detection_graph = tf.Graph()

        with self.detection_graph.as_default():
          od_graph_def = tf.GraphDef()

          if os.path.isfile(PATH_TO_CKPT):
              rospy.loginfo( " Path:%s", PATH_TO_CKPT)
              with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                rospy.loginfo( " Model read complete")

                od_graph_def.ParseFromString(serialized_graph)
                return_el = tf.import_graph_def(od_graph_def, name='')

        self.sess = tf.Session(graph=self.detection_graph)
        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        self.current_light = TrafficLight.UNKNOWN
        self.init_flag = True
        rospy.loginfo( " TLClassifier END")



    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        traffic_light = TrafficLight.UNKNOWN
        max_boxes_to_draw=20
        min_score_thresh=0.5
        traffic_light_label=10

        (im_height, im_width, im_channel) = image.shape

        image_arr =  np.array(image[...,::-1]).reshape((im_height, im_width, 3)).astype(np.uint8) #BGR to RGB conversion

        #np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)
        # model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_arr, axis=0)
        (boxes, scores, classes, num) = self.sess.run(
              [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
              feed_dict={self.image_tensor: image_np_expanded})

        if 0:
            rospy.loginfo(num)
            rospy.loginfo(boxes.shape)
            rospy.loginfo(scores.shape)
            rospy.loginfo(classes.shape)

        min_score_thresh = 0.5
        Threshold = 0.01

        for i in range(0, min(max_boxes_to_draw, boxes.shape[1])):

            if scores[0][i] > min_score_thresh and classes[0][i] == traffic_light_label:
                #rospy.loginfo(" box[%d]---", i)
                ymin, xmin, ymax, xmax = (tuple(boxes[0][i].tolist()))
                (left, right, top, bottom) = ((xmin * im_width, xmax * im_width,
                                              ymin * im_height, ymax * im_height))

                #rospy.loginfo(" left %f %f %f %f", left, right, top, bottom)
                crop_img = image_arr[int(top):int(bottom), int(left):int(right), :]#image.crop((left, top, right, bottom))

                desired_dim = (30, 90) # width, height

                img = cv2.resize(np.array(crop_img), desired_dim, interpolation=cv2.INTER_LINEAR)

                if 0: # to view what is detected  in the box
                    name = "image"
                    cv2.imshow('name', img)
                    cv2.waitKey(20)

                img_hsv=cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

                # lower mask (0-10)
                lower_red = np.array([0,70,50])
                upper_red = np.array([10,255,255])
                mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

                # upper mask (170-180)
                lower_red = np.array([170,70,50])
                upper_red = np.array([180,255,255])
                mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

                # red pixels' mask
                mask = mask0+mask1

                #rospy.loginfo(mask0)
                # Compare the percentage of red values
                rate = np.count_nonzero(mask) / float(desired_dim[0] * desired_dim[1])
                #rospy.loginfo("red pixel count %f", rate)


                if rate > Threshold:
                    traffic_light = TrafficLight.RED
                    break
                else:
                    traffic_light = TrafficLight.GREEN
        return traffic_light

if __name__ == '__main__':
    try:
        classifer = TLClassifier()
        file_name = sys.argv[1]
        test_img = cv2.imread(file_name)
        color = classifier.get_classification(test_img)
        print(color)
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
