
"""
@file hough_lines.py (taken from https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html)
@brief This program demonstrates line finding with the Hough transform
"""

import sys
import math
import cv2 as cv
import numpy as np

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

    

def find_lines(img):
    """
    Input:
		img: np.3darray; the input image with a cone to be detected. BGR.

    Returns:
        lines_returned: an array of tuples (pt1,pt2) reprsenting the start and end points
                        all lines detected in the image

    """
    # Edge detection
    
    #img, threshold 1, threshold 2, apertureSize (for the Sobel operator)
    edges = cv.Canny(img, 50, 200, apertureSize = 3)
    
    # Copy edges to the images that will display the results in BGR
    cedges = cv.cvtColor(edges, cv.COLOR_GRAY2BGR)
    cedgesP = np.copy(cedges)

    #edges -> output of edge detector
    rho = 1 #resolution param in pixels
    theta = np.pi/180 #resolution param (1 degree )
    min_intersections = 150 #number of intersections to detect a line
    lines = cv.HoughLines(edges, 1, np.pi / 180, 150, None, 0, 0)
    
    #drawing the lines

    lines_returned = []
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            lines_returned.append((pt1,pt2)) #start point, end point
            cv.line(cedges, pt1, pt2, (0,0,255), 3, cv.LINE_AA)
    
    
    #probabilitistic transform
    linesP = cv.HoughLinesP(edges, 1, np.pi / 180, 50, None, 50, 10)
    
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv.line(cedgesP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)
    
    cv.imshow("Source", img)
    cv.imshow("Detected Lines (in red) - Standard Hough Line Transform", cedges)
    cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cedgesP)
    
    cv.waitKey()

    return sorted(lines_returned,key = lambda x: x[0][0])