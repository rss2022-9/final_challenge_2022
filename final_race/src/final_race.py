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

    def callback(self):
        pass

if __name__=="__main__":
    rospy.init_node("final_race")
    fr = FinalRace()
    rospy.spin()
