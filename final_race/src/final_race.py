#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visual_servoing.msg import ConeLocation, ConeLocationPixel

class FinalRace(object):
    """
    Completes a lap around a track.
    """
    def __init__(self):
        self.target_topic = rospy.get_param("~target_topic")
        self.drive_topic = rospy.get_param("~drive_topic")

        self.target_sub = rospy.Subscriber(self.target_topic, ConeLocation, self.PPController)
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=1)
        
        self.lookahead        = 1.0
        self.speed            = 1.0
        self.wheelbase_length = 0.325

        # Things to do:
        # 1) Subscribe to the zed camera image
        # 2) Process the image to find the lane
        # 3) Convert the lane into a single target pixel/path of pixels
        # 4) Transform the goal from (u, v) to (x, y) in car frame (using homography transform?)
        # 5) Use pure pursuit to drive towards (x, y) coords

    def PPController(self, target):
        """
        Target is a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.
        Units are in meters.
        """
        x = target.x_pos
        y = target.y_pos
        x = -y # from camera frame to PP frame
        tr = (self.lookahead**2)/(2*x) if x != 0 else 0 # Turning radius from relative x
        ang = -np.arctan(self.wheelbase_length/tr) # Angle from turning radius
        output = max(min(ang,0.34),-0.34)
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = rospy.Time.now()
        drive_cmd.header.frame_id = 'base_link'
        drive_cmd.drive.speed = self.speed
        drive_cmd.drive.steering_angle = output
        self.drive_pub.publish(drive_cmd)

if __name__=="__main__":
    rospy.init_node("final_race")
    fr = FinalRace()
    rospy.spin()
