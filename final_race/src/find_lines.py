#!/usr/bin/env python
"""
@file hough_lines.py (taken from https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html)
@brief This program demonstrates line finding with the Hough transform
"""

import sys
import math
from tkinter import Y
from turtle import left
import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
# from torch import true_divide


def find_lines(img, probabalisticHough = True, allLines=False):
    
    # crop the upper 1/3 of the image to black
    shape = img.shape
    for i in range(0,int(shape[0])/2):
      img[i,:] = 0

    # ##filtering for white
    # hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # #define range of white
    # sensitivity = 15
    # lower_white = np.array([0,0,255-sensitivity])
    # upper_white = np.array([255,sensitivity,255])

    # #threshold the hsv image to get only white
    # mask = cv.inRange(hsv_img, lower_white, upper_white)

    # #bitwase-AND mask and original img
    # res  = cv.bitwise_and(img, img, mask = mask)

    # ##end white filtering
    
    gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    bridge = CvBridge()
    bw_image = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 100])  # 22-35 90-100 80-100
    upper_white = np.array([180, 255, 225])
    path_map = cv.inRange(bw_image,lower_white, upper_white)
    image_pub = rospy.Publisher("path_seen", Image, queue_size=10)
    imageout = bridge.cv2_to_imgmsg(path_map)
    
    

    #img, threshold 1, threshold 2, apertureSize (for the Sobel operator)
    #increase min threshold to filter out gray lines
    edges = cv.Canny(gray_img, 80, 255, apertureSize = 3)
    imageout = bridge.cv2_to_imgmsg(edges)
    image_pub.publish(imageout)
    
    # Copy edges to the images that will display the results in BGR
    cedges = cv.cvtColor(edges, cv.COLOR_GRAY2BGR)
    cedgesP = cv.cvtColor(edges, cv.COLOR_GRAY2BGR)
    
    lines_returned = []


    #probabilstic Hough
    if (probabalisticHough):
      #probabilitistic transform
      linesP = cv.HoughLinesP(edges, 1, np.pi / 180, 100, None, 50, 10)
      
      if linesP is not None:
        max_left_line_slope = 0 # has positive slope
        min_right_line_slope = 0 # has negative slope
        # left_line = ((0,0),(0,0)) #(x,y)
        # right_line = ((0,0),(0,0))
        left_line = ((0,370), (670,0))
        right_line = ((0,0), (670,370))

        for i in range(0, len(linesP)):
            l = linesP[i][0]
            
            #returning all lines
            if (allLines):
              lines_returned.append(((l[0], l[1]), (l[2], l[3])))
              #cv.line(cedgesP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)
            #returning two lines
            else:
              slope = (l[3] - l[1])/(l[2]-l[0])
              if (slope > 0 and slope > max_left_line_slope):
                max_left_line_slope = slope
                left_line = ((l[0], l[1]), (l[2], l[3]))
              elif (slope < 0 and slope < min_right_line_slope):
                min_right_line_slope = slope
                right_line = ((l[0], l[1]), (l[2], l[3]))
        if (allLines==False):
          lines_returned.append(left_line)
          lines_returned.append(right_line)

    #standard hough
    else:
      lines = cv.HoughLines(edges, 1, np.pi / 180, 150)
      #print(len(lines))
      if lines is not None:
        min_left_line_theta = np.pi/2 #
        min_right_line_theta = 0 # (rho negative is right line)
        # left_line = ((0,0),(0,0)) 
        # right_line = ((0,0),(0,0))
        left_line = ((0,370), (670,0))
        right_line = ((0,0), (670,370))

        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            # print(rho,theta)
            a = math.cos(theta)
            b = math.sin(theta) 
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))

            #all lines
            if (allLines):
              lines_returned.append((pt1,pt2))
              #cv.line(cedges, pt1, pt2, (0,0,255), 3, cv.LINE_AA)
            #two lines
            else:
              if (rho > 0 and theta < min_left_line_theta):
                min_left_line_theta = theta
                left_line = (pt1,pt2)
              elif (rho < 100 and theta > min_right_line_theta):
                min_right_line_theta = theta
                right_line = (pt1,pt2)
        if (allLines == False):
          lines_returned.append(left_line)
          lines_returned.append(right_line)   


    
    # image_print([img, cedges])
    # cv.imshow("Source", img)
    # cv.imshow("Detected Lines (in red) - Standard Hough Line Transform", cedges)
    # cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cedgesP)
    
    # cv.waitKey()
    if (allLines):
      lines_returned = sorted(lines_returned,key = lambda x: x[0][0])

    return lines_returned

def find_intercept_point(line1, line2):
  """
  line: (startpoint, endpoint)
  return: intercept point (x,y)
  """

  startpoint1 = line1[0]
  endpoint1   = line1[1]
  startpoint2 = line2[0]
  endpoint2   = line2[1]

  # # transform x = v, y = -u
  # x1 = startpoint1[1]
  # y1 = -startpoint1[0]
  # x2 = startpoint2[1]
  # y2 = -startpoint2[0]

  # xend1 = endpoint1[1]
  # yend1 = -endpoint1[0]
  # xend2 = endpoint2[1]
  # yend2 = -endpoint2[0]
    # transform x = u, y = -v
  x1 = startpoint1[0]
  y1 = -startpoint1[1]
  x2 = startpoint2[0]
  y2 = -startpoint2[1]

  xend1 = endpoint1[0]
  yend1 = -endpoint1[1]
  xend2 = endpoint2[0]
  yend2 = -endpoint2[1]

  # x1 = startpoint1[0]
  # y1 = startpoint1[1]
  # x2 = startpoint2[0]
  # y2 = startpoint2[1]

  # xend1 = endpoint1[0]
  # yend1 = endpoint1[1]
  # xend2 = endpoint2[0]
  # yend2 = endpoint2[1]

  slope1 = True
  slope2 = True
  # if startpoint1[0] - endpoint1[0] is 0:
  #   slope1 = False
  # if startpoint2[0] - endpoint2[0] is 0:
  #   slope2 = False
  if x1-xend1 is 0:
    slope1 = False
  if x2-xend2 is 0:
    slope2 = False
  
  if slope1 is True and slope2 is True:
    k1 = float(yend1-y1)/float(xend1-x1)
    k2 = float(yend2-y2)/float(xend2-x2)
    if k1 == k2:
      print("parallel lines")
    x_intercept = float(k1*x1-k2*x2+y2-y1)/(k1-k2)
    y_intercept = k1*x_intercept-k1*x1+y1

    y_hack = -230
    x1_hack = (k1*x1-y1+y_hack)/k1
    x2_hack = (k2*x2-y2+y_hack)/k2 # assume k!=0
    x_hack   = (x1_hack+x2_hack)/2

    return (int(x_hack), -int(y_hack))

  elif slope1 is True and slope2 is False:
    k1 = (yend1-y1)/(xend1-x1)
    x_intercept = x2
    y_intercept = k1*x_intercept + y1-k1*x1
  elif slope1 is False and slope2 is True:
    k2 = (yend2-y2)/(xend2-x2)
    x_intercept = x1
    y_intercept = k2*x_intercept + y2-k2*x2
  else:
    print("Parallel lines detected")
    return -1
  
  # transform back to u, v frame
  # return (-y_intercept, x_intercept)
  return (int(x_intercept), -int(y_intercept))

  # if slope1 is True and slope2 is True:
  #   k1 = (startpoint1[1]-endpoint1[1])/(startpoint1[0]-endpoint1[0])
  #   k2 = (startpoint2[1]-endpoint2[1])/(startpoint2[0]-endpoint2[0])
  #   if k1 == k2:
  #     # print("Parallel lines detected")
  #     return -1
  #   x_intercept = (k1*x1-k2*x2+y2-y1)/(k1-k2)
  #   y_intercept = k1*x_intercept-k1*x1+y1
  # elif slope1 is True and slope2 is False:
  #   k1 = (startpoint1[1]-endpoint1[1])/(startpoint1[0]-endpoint1[0])
  #   x_intercept = x2
  #   y_intercept = k1*x_intercept + y1-k1*x1
  # elif slope1 is False and slope2 is True:
  #   k2 = (startpoint2[1]-endpoint2[1])/(startpoint2[0]-endpoint2[0])
  #   x_intercept = x1
  #   y_intercept = k2*x_intercept + y2-k2*x2
  # else:
  #   # print("Parallel lines detected")
  #   return -1

  # return (x_intercept, -y_intercept)

  def uv_to_rel_x(self, U,V):
        PTS_IMAGE_PLANE = [[343.0, 363.0],
                   [139.0, 276.0],
                   [347.0, 281.0],
                   [562.0, 291.0],
                   [229.0, 238.0],
                   [349.0, 243.0],
                   [472.0, 246.0],
                   [263.0, 223.0],
                   [347.0, 224.0],
                   [432.0, 227.0]]

        PTS_GROUND_PLANE = [[8.4, 0.0],
                        [21.25, 17.25],
                        [21.25, 0.0],
                        [21.25, -17.25],
                        [42.5, 17.25],
                        [42.5, 0.0],
                        [42.5, -17.25],
                        [63.75, 17.25],
                        [63.75, 0.0],
                        [63.75, -17.25]]

        METERS_PER_INCH = 0.0254

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        h, err = cv.findHomography(np_pts_image, np_pts_ground)

        ONES = np.ones(U.shape, dtype=np.float32)
        UV = np.array([U,V,ONES], dtype=np.float32)
        XY = np.einsum('ij,jk->ik', h, UV)
        scaling_factor = 1.0/XY[2,:]
        orange_locations = XY * scaling_factor
        orange_locations = orange_locations[0:2,:]
        distance = np.linalg.norm(orange_locations, axis=0)
        x = orange_locations[1,:] + 0.06
        y = orange_locations[0,:]
        return (x,y)
