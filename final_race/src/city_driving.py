#!/usr/bin/env python

import numpy as np
import rospy
import cv2

from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image


PTS_IMAGE_PLANE = [[526.0, 286.0],
                   [240.0, 287.0],
                   [292.0, 243.0],
                   [442.0, 243.0]]

PTS_GROUND_PLANE = [[23.62, -9.75],
                    [23.62, 9.75],
                    [43.12, 9.75],
                    [43.12, -9.75]]

METERS_PER_INCH = 0.0254

class CityDriving:
    def __init__(self):
        if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
            rospy.logerr("ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length")

        #Initialize data into a homography matrix
        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        drive_topic = rospy.get_param("~drive_topic", "/vesc/low_level/ackermann_cmd_mux/input/navigation")
        self.wheelbase_length = 0.325
        self.lookahead = 0.6
        self.thresh = 0.01
        self.speed = 0.5
        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)
        self.bridge = CvBridge()
        
        # Publishers
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        self.debug_pub = rospy.Publisher("/debugging", String, queue_size=10)
        self.image_pub = rospy.Publisher("i_see", Image, queue_size=10)
        self.ang_pub = rospy.Publisher("ang_here", Float32, queue_size=10)

        # Subscribers
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.follow_line)
        
    def follow_line(self,image_msg):
        img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        bw_img = self.cd_color_segmentation(img)
        orange_locations = self.convert_to_real(bw_img)
        rel_x = self.find_rel_x(orange_locations)
        ang = self.PPController(rel_x)
        self.ang_pub.publish(ang)
        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.speed = self.speed
        drive_cmd.drive.steering_angle = ang
        self.drive_pub.publish(self.drive_cmd)

    def cd_color_segmentation(self, img, template="optional"):
        hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        lower_orange = np.array([0, 60, 70])  # 22-35 90-100 80-100
        upper_orange = np.array([100, 255, 165])
        path_map = cv2.inRange(hsv_img,lower_orange, upper_orange)
        imageout = self.bridge.cv2_to_imgmsg(path_map)
        self.image_pub.publish(imageout)
        return path_map

    def convert_to_real(self,img):
        white_pix = np.argwhere(img == 255)
        V, U = white_pix[:,0], white_pix[:,1]
        ONES = np.ones(U.shape, dtype=np.float32)
        UV = np.array([U,V,ONES], dtype=np.float32)
        XY = np.einsum('ij,jk->ik', self.h, UV)
        scaling_factor = 1.0/XY[2,:]
        orange_locations = XY * scaling_factor
        return orange_locations[0:2,:]

    def find_rel_x(self,orange_locations):
        low = self.lookahead - self.thresh
        high = self.lookahead + self.thresh
        distances = np.linalg.norm(orange_locations, axis=0)
        indists = (distances<high) & (distances>low)
        rel_xs = orange_locations[1,indists]
        rel_x = -np.average(rel_xs)
        #print(min(distances),max(distances))
        #print(rel_x)
        return rel_x

    def PPController(self, rel_x):
        x = rel_x
        tr = (self.lookahead**2)/(2*x) if x != 0 else 0 # Turning radius from relative x
        ang = -np.arctan(self.wheelbase_length/tr) # Angle from turning radius
        return ang

if __name__=="__main__":
    try:
        rospy.init_node("CityDriving", anonymous=True)
        CityDriving()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass