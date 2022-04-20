#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry

class CityDriving(object):
    """
    Drives around a simulated city environment
    """
    def __init__(self):
        self.sub_topic = rospy.get_param("~sub_topic")
        self.pub_topic = rospy.get_param("~pub_topic")

        self.sub = rospy.Subscriber(self.sub_topic, Odometry, self.callback)
        self.pub = rospy.Publisher(self.pub_topic, Odometry, queue_size=10)

        # Things to do:
        # 1) Subscribe to the zed camera image and lidar
        # 2) Identify orange line in image
        # 3) Use line following to follow the line
        # 4) While following, if a stop sign is found, stop
        # 5) If stopped at a sign, once a few secs have passed continue line following
        # 6) Car wash if time permits

    def callback(self):
        pass

if __name__=="__main__":
    rospy.init_node("city_driving")
    cd = CityDriving()
    rospy.spin()