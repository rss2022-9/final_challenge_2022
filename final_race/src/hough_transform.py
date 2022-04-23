"""
@file hough_lines.py (taken from https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html)
@brief This program demonstrates line finding with the Hough transform
"""
import sys
import math
from turtle import right

from matplotlib.lines import Line2D
import cv2 as cv
import numpy as np

import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from visual_servoing.msg import ConeLocation, ConeLocationPixel

import find_lines


class HoughTransform():
    """
     A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
      
        # Subscribe to ZED camera RGB frames
        self.goal_point_pub= rospy.Publisher("/relative_cone_px", ConeLocationPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/track_line_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        # publishing pixels in the (u,v) frame (not converted to car frame


        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        #[(start_point,end_point)]
        lines = find_lines(image)


        goal_point = ConeLocationPixel()
        goal_point.u, goal_point.v = self.goal_point_of_lane(3,lines)
        self.goal_point_pub.publish(goal_point)

        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)

    def goal_point_of_lane(lane_number, lines):
        left_bound = lines[lane_number]
        right_bound = lines[lane_number+1]

        #left_bound[1] is end point [0] -> x value
        x_goal = (left_bound[1][0] + right_bound[1][0])//2
        y_goal = (left_bound[1][1] + right_bound[1][1])//2
        return x_goal,y_goal


if __name__ == '__main__':
    try:
        rospy.init_node('HoughTransform', anonymous=True)
        HoughTransform()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass