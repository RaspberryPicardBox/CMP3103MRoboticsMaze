import cv2 as cv
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError


def run():
    rospy.spin()


class solver:

    def __init__(self):
        rospy.init_node('solver')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.laser_scan = rospy.Subscriber('/scan', LaserScan, self.path)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom)
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.wall = False

    def odom(self, odom_msg):
        self.orientation = odom_msg.pose.pose.orientation.z

    def path(self, laser_msg):
        ranges = laser_msg.ranges
        if not self.wall:
            self.find_wall(ranges)
        else:
            self.follow_wall(ranges)

    def image_callback(self, image_data):
        cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        hsv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        h, w, d = hsv_image.shape
        mask = cv.inRange(hsv_image, np.array([0, 180, 50]), np.array([255, 255, 255]))
        res = cv.bitwise_and(cv_image, cv_image, mask=mask)

        cv.imshow("Camera", cv_image)
        #cv.imshow("Result", res)
        cv.waitKey(1)

    def find_wall(self, ranges):
        self.wall = False
        forward = np.nanmean(ranges[320])
        t = Twist()

        min_dist = min(ranges)
        min_index = ranges.index(min_dist)

        err = min_index - 320

        if abs(err) > 0:
            t.angular.z = err / 100
            self.publisher.publish(t)

        if forward > 0.5:
            t.linear.x = 0.1
            self.publisher.publish(t)
        else:
            self.wall = True
        return

    def follow_wall(self, ranges):
        t = Twist()
        right = np.nanmean(ranges[0])
        left = np.nanmean(ranges[639])
        forward = np.nanmean(ranges[320])

        if left < 0.3:
            t.angular.z = 0.2
            self.publisher.publish(t)
            print("left is too close")

        if left > 0.7:
            t.angular.z = -0.2
            self.publisher.publish(t)
            print("left is too far")

        if forward > 0.5:
            t.linear.x = 0.1
            self.publisher.publish(t)
            print("forward")


if __name__ == "__main__":
    s = solver()
    run()