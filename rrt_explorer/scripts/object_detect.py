#! /usr/bin/python

import rospy
import std_msgs.msg
import cv2
import pickle
import numpy as np
from random import shuffle
from scipy.misc import imread, imresize
from timeit import default_timer as timer

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sys
import rospkg
rospack  = rospkg.RosPack()
pkg_path = rospack.get_path('rrt_explorer')

import time

from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes
from os.path import expanduser

class DarkNetDetectionOverlay():
    def __init__(self):
        self.node_name = "dark_net_object_detection"
        rospy.init_node(self.node_name)

        self.conf_thresh = 0.25
        self.boxes_list  = BoundingBoxes()

        # Create unique and somewhat visually distinguishable bright
        # colors for the different classes.
        # Depends on the detector
        self.num_classes = 20
        self.class_colors = []
        for i in range(0, self.num_classes):
            # This can probably be written in a more elegant manner
            hue = 255*i/self.num_classes
            col = np.zeros((1,1,3)).astype("uint8")
            col[0][0][0] = hue
            col[0][0][1] = 128 # Saturation
            col[0][0][2] = 255 # Value
            cvcol = cv2.cvtColor(col, cv2.COLOR_HSV2BGR)
            col = (int(cvcol[0][0][0]), int(cvcol[0][0][1]), int(cvcol[0][0][2]))
            self.class_colors.append(col)

        self.bridge             = CvBridge() # Create the cv_bridge object
        self.to_draw            = cv2.imread(pkg_path+ '/resource/start.jpg')
        self.image_sub          = rospy.Subscriber("/usb_cam/image_raw", Image, self.detect_image,queue_size=1)
        self.bounding_boxes_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.overlay_boxes,queue_size=1)

    def detect_image(self, ros_image):
        try:
            self.image_orig = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)

    def overlay_boxes(self, boxes):
        self.boxes_list = boxes

        self.to_draw = self.image_orig #cv2.cvtColor(self.image_orig, cv2.COLOR_BGR2RGB)
        colorIndex = 0

        for box in self.boxes_list.bounding_boxes:
            print box

            xmin = box.xmin
            ymin = box.ymin
            xmax = box.xmax
            ymax = box.ymax

            print xmin, ymin, xmax, ymax

            # Draw the box on top of the to_draw image
            cv2.rectangle(self.to_draw, (xmin, ymin), (xmax, ymax), self.class_colors[colorIndex], 2)

            text     = box.Class + " " + ('%.2f' % box.probability)
            text_top = (xmin, ymin-10)
            text_bot = (xmin + 80, ymin + 5)
            text_pos = (xmin + 5, ymin)

            cv2.rectangle(self.to_draw, text_top, text_bot, self.class_colors[colorIndex], -1)
            cv2.putText(self.to_draw, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,0,0), 1)
            colorIndex = colorIndex + 1

        cv2.imshow("SSD result", self.to_draw)
        cv2.waitKey(1)
        return

def main(args):
    try:
        DarkNetDetectionOverlay()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down object detection overlay"
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

