import cv2 as cv
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from kobuki_msgs.msg import BumperEvent
from cv_bridge import CvBridge, CvBridgeError


def run():
    rospy.spin()


class solver:

    def __init__(self):
        rospy.init_node('solver')
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.laser_scan = rospy.Subscriber('/scan', LaserScan, self.path)
        #self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom)
        self.bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.bumper)
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.wall = False
        self.bumper_data = 0

    def odom(self, odom_msg):
        self.orientation = odom_msg.pose.pose.orientation.z

    def bumper(self, bumper_data):
        self.bumper_data = bumper_data.state

    def path(self, laser_msg):
        ranges = laser_msg.ranges

        right = ranges[0]
        left = ranges[639]
        forward = np.nanmean(ranges[160:480])

        if forward > 0.5 and not np.isnan(forward) and not np.isnan(left) and not np.isnan(right) and self.bumper_data == 0:
            t = Twist()
            t.linear.x = 0.1
            err = left-right
            t.angular.z = err/5
            self.publisher.publish(t)
        else:
            t = Twist()
            min_dist = min(ranges)
            min_index = ranges.index(min_dist)
            max_dist = max(ranges)
            max_index = ranges.index(max_dist)

            err = max_index - min_index
            t.angular.z = err/50
            t.linear.x = -0.3
            self.publisher.publish(t)

    def image_callback(self, image_data):
        cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        hsv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        h, w, d = hsv_image.shape
        mask = cv.inRange(hsv_image, np.array([0, 180, 50]), np.array([255, 255, 255]))
        res = cv.bitwise_and(cv_image, cv_image, mask=mask)

        cv.imshow("Camera", cv_image)
        cv.imshow("Result", res)
        cv.waitKey(1)


if __name__ == "__main__":
    s = solver()
    run()