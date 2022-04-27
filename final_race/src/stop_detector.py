#!/usr/bin/env python

import numpy as np
import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray

class StopDetector:
    def __init__(self):
        detector_topic = rospy.get_param("~detector_topic", "/stop_sign_bbox")
        drive_topic = rospy.get_param("~drive_topic", "/vesc/high_level/ackermann_cmd_mux/input/nav_0")

        #3 States: Searching, Stopping, Starting
        #Searching: The car hasn't seen a stop sign within range recently and is looking for one. Switches to waiting.
        #Waiting: The car has seen a stop sign within range and is currently stopped in front of it. Switches to starting.
        #Starting: The car was very recently stopped and is just getting moving again. Switches to searching.
        self.state = "Searching"

        #Expected size (in pixels) of the bounding box at the correct stopping distance
        self.stop_height = 100
        self.stop_width = 100

        #Time stamp for calculating durations
        self.prev_time = rospy.get_time()
        self.wait_time = 2 #Wait at each sign for wait_time seconds
        self.reset_time = 2 #Don't stop again for at least reset_time seconds
        
        self.detector_sub = rospy.Subscriber(detector_topic, Float32MultiArray, self.callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        
    def callback(self, bbox):
        rospy.loginfo(bbox)
        if self.state == "Searching":
            self.search(bbox)
        elif self.state == "Waiting":
            self.wait()
        elif self.state == "Starting":
            self.start()

    def search(self, bbox):
        height = bbox[3] - bbox[1]
        width = bbox[2] - bbox[0]

        #If the stop sign is roughly the right distance away, stop the car
        if abs(height - self.stop_height) < 20 and abs(width - self.stop_width) < 20:
            self.drive_pub.publish(self.make_drive_cmd(0, 0))
            self.prev_time = rospy.get_time()
            self.state = "Waiting"

    def wait(self):
        #Keep the car stopped until wait_time has passed
        if rospy.get_time() - self.prev_time < self.wait_time:
            self.prev_time = rospy.get_time()
            self.state = "Starting"
        else:
            self.drive_pub.publish(self.make_drive_cmd(0, 0))

    def start(self):
        #Wait until reset_time has passed before searching for a stop sign again
        if rospy.get_time() - self.prev_time > self.reset_time:
            self.state = "Searching"

    def make_drive_cmd(self, speed, angle):
        drive_cmd = AckermannDriveStamped()
        drive_cmd.header.stamp = rospy.Time.now()
        drive_cmd.drive.speed = speed
        drive_cmd.drive.steering_angle = angle
        return drive_cmd

if __name__=="__main__":
    try:
        rospy.init_node("StopDetector", anonymous=True)
        StopDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass