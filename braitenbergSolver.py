"""
-----BraitenburgSolver-----
A ROS based maze solver for use with the Turtlebot, using almost solely Braitenberg vehicle behaviours.

Made by Wilfred Michael Boyce
"""

import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from kobuki_msgs.msg import BumperEvent


class braitenburgSolver:  # The ROS interaction class

    def __init__(self):
        rospy.init_node('braitenburgSolver')

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.laser_scan = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.bumper_callback)

        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        self.cv_image = None
        self.laser_data = None
        self.bumper_data = None

    def image_callback(self, image_data):  # Take the image frame and assign it to a class variable
        cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        self.cv_image = cv_image

    def laser_callback(self, laser_data):  # Take the laser data and assign it to a class variable
        self.laser_data = laser_data.ranges

    def bumper_callback(self, bumper_data):  # Take the bumper state and assign it to a class variable
        self.bumper_data = bumper_data.state


if __name__ == "__main__":
    solver = braitenburgSolver()  # Initialise the main ROS interface

    while not rospy.is_shutdown():

        ranges = solver.laser_data
        t = Twist()

        if ranges:  # Check that laser_data has recieved data yet
            right = ranges[0]
            left = ranges[639]
            forward = np.nanmean(ranges[160:480])
        else:  # If not, assign 0 to prevent missing data issues
            right = 0
            left = 0
            forward = 0

        if not np.isnan(left) and not np.isnan(right) and not np.isnan(forward):  # Check if any values are too close to register
            t = Twist()

