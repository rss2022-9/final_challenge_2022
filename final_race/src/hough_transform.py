#!/usr/bin/env python
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
        # image.shape = 376*672

        #[(start_point,end_point)]
        lines = find_lines.find_lines(image, probabalisticHough = False, allLines=False)
        goal_point = ConeLocationPixel()
        goal_point.u, goal_point.v = find_lines.find_intercept_point(lines[0],lines[1])
        goal_point.v = goal_point.v # TODO: hacky trick is +50 when using intercept, make sure the intercept on the plane
        self.goal_point_pub.publish(goal_point)
        # print(goal_point)
        self.pub_debug_img(image, lines, goal_point)


    def pub_debug_img(self, image, lines, goal_point):
        # plot the detected lines and goal poinsts in the images
        blue = (255,0,0)
        green= (0,255,0)
        red  = (0,0,255)
        radius = 20
        thickness = 2

        debug_img = cv2.circle(image, (goal_point.u, goal_point.v), radius, blue, thickness)
        # debug_img = cv2.circle(debug_img, ((x1+x2)/2, (y1+y2)/2), radius, blue, thickness)
        for line in lines:
            debug_img = cv2.line(debug_img, line[0], line[1], green, thickness)
        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
        self.debug_pub.publish(debug_msg)

    def goal_point_of_lane(self, lines):
        #takes two lines 
        left_bound = lines[0]
        right_bound = lines[1]
        # rospy.loginfo("goal_point func")

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