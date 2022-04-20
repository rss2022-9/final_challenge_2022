#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry

class FinalRace(object):
    """
    Completes a lap around a track.
    """
    def __init__(self):
        self.sub_topic = rospy.get_param("~sub_topic")
        self.pub_topic = rospy.get_param("~pub_topic")

        self.sub = rospy.Subscriber(self.sub_topic, Odometry, self.callback)
        self.pub = rospy.Publisher(self.pub_topic, Odometry, queue_size=10)

        # Things to do:
        # 1) Subscribe to the zed camera image
        # 2) Process the image to find the lane
        # 3) Convert the lane into a single target pixel/path of pixels
        # 4) Transform the goal from (u, v) to (x, y) in car frame (using homography transform?)
        # 5) Use pure pursuit to drive towards (x, y) coords

    def callback(self):
        pass

if __name__=="__main__":
    rospy.init_node("final_race")
    fr = FinalRace()
    rospy.spin()
