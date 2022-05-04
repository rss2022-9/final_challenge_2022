#!/usr/bin/env python

import numpy as np
import rospy
import cv2

from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image


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
        self.thresh = 0.05
        #self.speed = rospy.get_param("~city_speed", 0.5) #Moved the speed setting down to the line where it's used so I could use rosparam
        self.rel_x = 0.0
        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)
        self.bridge = CvBridge()

        # Publishers
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        self.image_pub = rospy.Publisher("path_seen", Image, queue_size=10)
        self.error_pub = rospy.Publisher("city_path_error", Float32, queue_size=10)

        # Subscribers
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.follow_line)
        
    def follow_line(self,image_msg):
        img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8") # Convert image to cv2
        bw_img = self.cd_color_segmentation(img) # return b&w image with white being detected orange
        orange_locations = self.convert_to_real(bw_img) # return real world location of orange points
        rel_x = self.find_rel_x(orange_locations) # get relative x using locations and lookahead circle
        ang = self.PPController(rel_x) # drive angle given relative x of target point
        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.speed = rospy.get_param("~city_speed", 0.5)
        drive_cmd.drive.steering_angle = ang
        #print(ang)
        self.drive_pub.publish(drive_cmd)
        self.error_pub.publish(self.rel_x)

    def cd_color_segmentation(self, img, template="optional"):
        """
        Filter out the orange parts of the image and return
        the mask as a black and white image where white is
        the orange portions of the image, which in this case gives us
        the line to follow.
        """
        hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        # lower_orange = np.array([0, 60, 70])  # 22-35 90-100 80-100
        # upper_orange = np.array([100, 255, 165])
        # rospy.loginfo(hsv_img)
        lower_orange = np.array([10, 140, 160])  # 26-32, 98-99, 100
        upper_orange = np.array([22, 255, 255])
        path_map = cv2.inRange(hsv_img,lower_orange, upper_orange)
        imageout = self.bridge.cv2_to_imgmsg(path_map)
        self.image_pub.publish(imageout)
        return path_map

    def convert_to_real(self,img):
        """
         Return the real world locaation of the white points in an image
         in this caase giving us the real world coordinates of the points
         on the orange line.
        """
        white_pix = np.argwhere(img == 255)
        V, U = white_pix[:,0], white_pix[:,1]
        ONES = np.ones(U.shape, dtype=np.float32)
        UV = np.array([U,V,ONES], dtype=np.float32)
        XY = np.einsum('ij,jk->ik', self.h, UV)
        scaling_factor = 1.0/XY[2,:]
        orange_locations = XY * scaling_factor
        return orange_locations[0:2,:]

    def find_rel_x(self,orange_locations):
        """
        Using the real world location of the orange points filter
        out those in the look ahead circle and average them to get the
        relative x position of the target point on the orange path
        """
        self.lookahead = rospy.get_param("~lookahead", 0.5)
        low = self.lookahead - self.thresh
        high = self.lookahead + self.thresh
        distances = np.linalg.norm(orange_locations, axis=0)
        indists = abs(distances-self.lookahead) <= self.thresh
        rel_xs = orange_locations[1,indists] + 0.05
        if rel_xs.size != 0:
            rel_x = np.average(rel_xs)
            self.rel_x = rel_x
        return self.rel_x

    def PPController(self, rel_x):
        """
        Pure pursuit controller which takes in the relative x of a 
        target point and returns the appropriate drive angle current
        convention is x positive left to right
        """
        x = rel_x
        tr = (self.lookahead**2)/(2*x) if x != 0.0 else 0.0001 # Turning radius from relative x
        ang = np.arctan(self.wheelbase_length/tr) # Angle from turning radius
        return ang

if __name__=="__main__":
    try:
        rospy.init_node("CityDriving", anonymous=True)
        CityDriving()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass