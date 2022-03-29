"""
-----BraitenburgSolver-----
A ROS based maze solver for use with the Turtlebot, using almost solely Braitenberg vehicle behaviours.

Made by Wilfred Michael Boyce
"""
from wx.lib.pubsub.py2and3 import values

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
        self.velocity_array = []
        self.angular_array = []

    def image_callback(self, image_data):  # Take the image frame and assign it to a class variable
        cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        self.cv_image = cv_image

    def laser_callback(self, laser_data):  # Take the laser data and assign it to a class variable
        self.laser_data = laser_data.ranges

    def bumper_callback(self, bumper_data):  # Take the bumper state and assign it to a class variable
        self.bumper_data = bumper_data.state


# Forward kinematics for the wheels on the turtlebot from Twist. Code from Marc Hanheide -
# https://github.com/LCAS/teaching/blob/a6bef8f36f6eec70ebd99701671736b4bf5a542b/cmp3103m-code-fragments
# /kinematics_diffdrive.py#L16-L21
def forward_kinematics(w_l, w_r):
    c_l = wheel_radius * w_l
    c_r = wheel_radius * w_r
    v = (c_l + c_r) / 2
    a = (c_r - c_l) / (2 * robot_radius)
    return (v, a)


if __name__ == "__main__":

    wheel_radius = .06
    robot_radius = .2

    solver = braitenburgSolver()  # Initialise the main ROS interface

    while not rospy.is_shutdown():

        ranges = solver.laser_data
        t = Twist()

        if ranges:  # Check that laser_data has recieved data yet
            right = np.nanmean(ranges[0:120])
            left = np.nanmean(ranges[520:639])
            forward = np.nanmean(ranges[160:480])
        else:  # If not, assign 0 to prevent missing data issues
            right = 0
            left = 0
            forward = 0

        if not np.isnan(left) and not np.isnan(right) and not np.isnan(forward):
            # Check if any values are too close to register
            t = Twist()

            # Calculation weights
            dst_chk = 0.5
            hate_weight = 0.8
            fear_weight = 1
            control_weight = 1

            if forward > dst_chk and left > dst_chk and right > dst_chk:

                solver.velocity_array = []
                solver.angular_array = []

                # -----Braitenburg 'hate' behaviour for corridor following and object avoidance-----
                (v, a) = forward_kinematics(right, left)
                solver.velocity_array.append(v * hate_weight)  # Add the hate behaviour to the movements
                solver.angular_array.append(a * hate_weight)

                # -----Braitenburg 'fear' behaviour for avoiding red-----
                hsv_img = cv.cvtColor(solver.cv_image, cv.COLOR_BGR2HSV)
                h, w, d = hsv_img.shape

                red_img = cv.inRange(hsv_img, np.array((0, 100, 50)), np.array((10, 255, 255)))

                left_img = red_img[:, :w/2]  # Cut the image horizontally to get sides of vision
                right_img = red_img[:, w/2:]

                left_input = np.mean(left_img)/2
                right_input = np.mean(right_img)/2

                cv.imshow("Red", red_img)
                cv.waitKey(1)

                (v, a) = forward_kinematics(left_input, right_input)
                solver.velocity_array.append(-(v * fear_weight))  # Add the fear behaviour to the movements
                solver.angular_array.append(a * fear_weight * 5)

                # -----Extract the kinematic results and publish-----
                t.linear.x = sum(solver.velocity_array)*control_weight
                t.angular.z = sum(solver.angular_array)*control_weight
                solver.publisher.publish(t)

            else:
                # Rotate to the side with greatest distance if stuck
                if left > right:
                    t.angular.z = 0.2
                else:
                    t.angular.z = -0.2
                solver.publisher.publish(t)
