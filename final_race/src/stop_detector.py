#!/usr/bin/env python

import numpy as np
import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray

class StopDetector:
    def __init__(self):
        detector_topic = rospy.get_param("~detector_topic", "/stop_sign_bbox")
        drive_sub_topic = rospy.get_param("~drive_sub_topic", "/vesc/low_level/ackermann_cmd_mux/input/navigation")
        drive_topic = rospy.get_param("~drive_topic", "/vesc/low_level/ackermann_cmd_mux/input/safety")

        #3 States: Searching, Stopping, Starting
        #Searching: The car hasn't seen a stop sign within range recently and is looking for one. Switches to waiting.
        #Waiting: The car has seen a stop sign within range and is currently stopped in front of it. Switches to starting.
        #Starting: The car was very recently stopped and is just getting moving again. Switches to searching.
        self.state = "Searching"

        #Expected size (in pixels) of the bounding box at the correct stopping distance
        self.stop_height = 50
        self.stop_width = 50

        #Time stamp for calculating durations
        self.prev_time = rospy.get_time()
        self.wait_time = 1 #Wait at each sign for wait_time seconds
        self.reset_time = 3 #Don't stop again for at least reset_time seconds
        
        self.detector_sub = rospy.Subscriber(detector_topic, Float32MultiArray, self.check_state)
        self.drive_sub = rospy.Subscriber(drive_sub_topic, AckermannDriveStamped, self.callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        
    def callback(self, drive_cmd):
        #If the car should be waiting, override the navigation to set the speed to 0
        #Else, do nothing and let the car keep driving
        if self.state == "Waiting":
            new_drive_cmd = AckermannDriveStamped()
            new_drive_cmd.header.stamp = rospy.Time.now()
            new_drive_cmd.drive.speed = 0
            new_drive_cmd.drive.steering_angle = drive_cmd.drive.steering_angle
            self.drive_pub.publish(new_drive_cmd)

    def check_state(self, bbox):
        if self.state == "Searching" and len(bbox.data) == 4:
            self.search(bbox.data)
        elif self.state == "Waiting":
            self.wait()
        elif self.state == "Starting":
            self.start()

    def search(self, bbox):
        height = bbox[3] - bbox[1]
        width = bbox[2] - bbox[0]

        #If the stop sign is roughly the right distance away, stop the car
        if abs(height - self.stop_height) < 100 and abs(width - self.stop_width) < 100:
            self.prev_time = rospy.get_time()
            self.state = "Waiting"
            rospy.loginfo("Swtiching to waiting")
        else:
            self.state = "Searching"

    def wait(self):
        #Keep the car stopped until wait_time has passed
        if rospy.get_time() - self.prev_time > self.wait_time:
            self.prev_time = rospy.get_time()
            self.state = "Starting"
            rospy.loginfo("Swtiching to starting")
        else:
            self.state = "Waiting"

    def start(self):
        #Wait until reset_time has passed before searching for a stop sign again
        if rospy.get_time() - self.prev_time > self.reset_time:
            self.state = "Searching"
            rospy.loginfo("Swtiching to searching")
        else:
            self.state = "Starting"

if __name__=="__main__":
    try:
        rospy.init_node("StopDetector", anonymous=True)
        StopDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass