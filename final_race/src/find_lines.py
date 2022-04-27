#!/usr/bin/env python
"""
@file hough_lines.py (taken from https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html)
@brief This program demonstrates line finding with the Hough transform
"""

import sys
import math
from turtle import left
import cv2 as cv
import numpy as np
from torch import true_divide

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

    

def find_lines(img, probabalisticHough = True, allLines=False):
    """
    Input:
		img: np.3darray; the input image with a cone to be detected. BGR.

    Returns:
        lines_returned: an array of tuples (pt1,pt2) reprsenting the start and end points
                        all lines detected in the image

    """
    
    # crop the upper 1/3 of the image to black
    shape = img.shape
    print(shape)
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

    #img, threshold 1, threshold 2, apertureSize (for the Sobel operator)
    #increase min threshold to filter out gray lines
    edges = cv.Canny(gray_img, 100, 200, apertureSize = 3)
    
    # Copy edges to the images that will display the results in BGR
    cedges = cv.cvtColor(edges, cv.COLOR_GRAY2BGR)
    cedgesP = cv.cvtColor(edges, cv.COLOR_GRAY2BGR)
    
    lines_returned = []


    #probabilstic Hough
    if (probabalisticHough):
      #probabilitistic transform
      linesP = cv.HoughLinesP(edges, 1, np.pi / 180, 200, None, 50, 10)
      
      if linesP is not None:
        max_left_line_slope = 0 # has positive slope
        min_right_line_slope = 0 # has negative slope
        left_line = ((0,0),(0,0)) #(x,y)
        right_line = ((0,0),(0,0))

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
              lines_returned.append(left_line)
              lines_returned.append(right_line)

    #standard hough
    else:
      lines = cv.HoughLines(edges, 1, np.pi / 180, 150, None, 0, 0)
      if lines is not None:
        min_left_line_theta = 90 #
        min_right_line_theta = 90 # (rho negative is right line)
        left_line = ((0,0),(0,0)) 
        right_line = ((0,0),(0,0))

        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
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
              elif (rho < 0 and theta < min_right_line_theta):
                min_right_line_theta = theta
                right_line = (pt1,pt2)
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

  x1 = startpoint1[0]
  y1 = startpoint1[1]
  x2 = startpoint2[0]
  y2 = startpoint2[1]

  slope1 = True
  slope2 = True
  if startpoint1[0] - endpoint1[0] is 0:
    slope1 = False
  if startpoint2[0] - endpoint2[0] is 0:
    slope2 = False
  
  if slope1 is True and slope2 is True:
    k1 = (startpoint1[1]-endpoint1[1])/(startpoint1[0]-endpoint1[0])
    k2 = (startpoint2[1]-endpoint2[1])/(startpoint2[0]-endpoint2[0])
    if k1 == k2:
      print("Parallel lines detected")
      return -1
    x_intercept = (k1*x1-k2*x2+y2-y1)/(k1-k2)
    y_intercept = k1*x_intercept-k1*x1+y1
  elif slope1 is True and slope2 is False:
    k1 = (startpoint1[1]-endpoint1[1])/(startpoint1[0]-endpoint1[0])
    x_intercept = x2
    y_intercept = k1*x_intercept + y1-k1*x1
  elif slope1 is False and slope2 is True:
    k2 = (startpoint2[1]-endpoint2[1])/(startpoint2[0]-endpoint2[0])
    x_intercept = x1
    y_intercept = k2*x_intercept + y2-k2*x2
  else:
    print("Parallel lines detected")
    return -1

  return (x_intercept, y_intercept)
