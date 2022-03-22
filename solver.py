import cv2 as cv
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from kobuki_msgs.msg import BumperEvent
from cv_bridge import CvBridge, CvBridgeError


def run():
    rospy.spin()


class solver:

    def __init__(self):
        rospy.init_node('solver')

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.laser_scan = rospy.Subscriber('/scan', LaserScan, self.path)
        self.bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.bumper)
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        self.bumper_data = 0
        self.previous_error = [0]

    def bumper(self, bumper_data):
        self.bumper_data = bumper_data.state

    def path(self, laser_msg):
        ranges = laser_msg.ranges
        self.corridor_follower(ranges)

    def image_callback(self, image_data):
        t = Twist()

        cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        hsv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        h, w, d = hsv_image.shape
        mask = cv.inRange(hsv_image, np.array([0, 180, 50]), np.array([255, 255, 255]))
        res = cv.bitwise_and(cv_image, cv_image, mask=mask)

        h, w, d = res.shape

        moments = cv.moments(mask)

        if moments['m00'] > 0:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            cv.circle(res, (cx, cy), 20, (0, 0, 255), -1)

        cv.imshow("Camera", cv_image)
        cv.imshow("Result", res)
        cv.waitKey(1)

    def corridor_follower(self, ranges):
        right = ranges[0]
        left = ranges[639]
        forward = np.nanmean(ranges[160:480])

        if forward > 0.5 and not np.isnan(forward) and not np.isnan(left) and not np.isnan(
                right) and self.bumper_data == 0:
            t = Twist()
            t.linear.x = 0.1

            # PID controller thanks to: https://projects.raspberrypi.org/en/projects/robotPID/
            if len(self.previous_error) > 10:
                previous_error_value = self.previous_error.pop()
            else:
                previous_error_value = self.previous_error[len(self.previous_error) - 1]

            Kp = 0.15
            Kd = 0.05
            Ki = 0.001

            err = ((left - right) * Kp) + (previous_error_value * Kd) + (sum(self.previous_error) * Ki)
            self.previous_error.append(err)

            t.angular.z = err
            self.publisher.publish(t)
        else:
            t = Twist()

            min_dist = min(ranges)
            min_index = ranges.index(min_dist)
            max_dist = max(ranges)
            max_index = ranges.index(max_dist)

            err = max_index - min_index

            t.angular.z = err / 2
            t.linear.x = -0.3
            self.publisher.publish(t)


if __name__ == "__main__":
    s = solver()
    run()