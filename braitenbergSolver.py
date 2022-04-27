"""
-----Braitenberg Solver-----
A ROS based maze solver for use with the Turtlebot, using combinations of Braitenberg vehicle behaviours and
OpenCV based computer vision.
This Turtlebot controller will navigate a maze, sticking to the left wall, avoiding red obstacles and racing towards
blue and green goals.
Made by Wilfred Michael Boyce
"""
from numpy import nanmean

import rospy
import cv2 as cv
import numpy as np
import time
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry


class braitenbergSolver:  # The ROS interaction class

    def __init__(self):
        rospy.init_node('braitenbergSolver')

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.laser_scan = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.bumper_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        self.cv_image = None
        self.hsv_img = None
        self.laser_data = None
        self.bumper_data = 0
        self.angular_pos = 0
        self.velocity_array = []
        self.angular_array = []

    def image_callback(self, image_data):  # Take the image frame and assign it to a class variable
        cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        hsv_img = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
        self.cv_image = cv_image
        self.hsv_img = hsv_img

    def laser_callback(self, laser_data):  # Take the laser data and assign it to a class variable
        self.laser_data = laser_data.ranges

    def bumper_callback(self, bumper_data):  # Take the bumper state and assign it to a class variable
        self.bumper_data = bumper_data.state

    def odom_callback(self, odom_data):
        self.angular_pos = odom_data.pose.pose.orientation.z


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

    solver = braitenbergSolver()  # Initialise the main ROS interface

    leftMem = []

    while not rospy.is_shutdown():

        ranges = solver.laser_data
        t = Twist()

        if ranges:  # Check that laser_data has recieved data yet
            right = np.nanmean(ranges[0:120])
            left = np.nanmean(ranges[520:639])
            forward = np.nanmean(ranges[160:480])
        else:  # If not, assign 0 to prevent missing data issues
            right = 0
            left = 1
            forward = 0
            ranges = [0]
            leftMem.append(left)

        if np.isnan(left) or np.isnan(right) or np.isnan(forward) or left == 0 or right == 0 or forward == 0 or solver.bumper_data == 1:
            t.linear.x = -0.2
            t.angular.z = -0.2
            solver.publisher.publish(t)

        if not np.isnan(left) and not np.isnan(right) and not np.isnan(forward):
            # Check if any values are too close to register
            t = Twist()

            # Calculation weights
            dst_chk = 0.5
            hate_weight = 0.8
            red_thresh = 5
            love_weight_blue = 0.65
            blue_thresh = 40
            love_weight_green = 1
            control_weight = 1
            left_bias = 0.4

            if len(leftMem) < 100:
                leftMem.append(left)
            else:
                leftMem.pop(0)
                leftMem.append(left)

            print(np.nanmean(leftMem))
            print("Actual " + str(left))

            if min(ranges) > dst_chk and solver.bumper_data == 0:

                # Braitenberg behaviours
                solver.velocity_array = []
                solver.angular_array = []

                # -----Braitenberg 'hate' behaviour for corridor following and object avoidance-----
                (v, a) = forward_kinematics(right * left_bias, np.nanmean(leftMem))
                solver.velocity_array.append(v * hate_weight)  # Add the hate behaviour to the movements
                solver.angular_array.append(a * hate_weight)

                # -----Braitenberg 'fear' behaviour for avoiding red----
                red_img = cv.inRange(solver.hsv_img, np.array((0, 100, 50)), np.array((10, 255, 255)))
                h, w, d = solver.hsv_img.shape

                if np.mean(red_img) > red_thresh:
                    left_img = red_img[:, :w / 2]  # Cut the image horizontally to get sides of vision
                    left_input = np.mean(left_img) / 5
                    while left_input > 0:  # This behaviour is different to emulate a "wall" on the red
                        red_img = cv.inRange(solver.hsv_img, np.array((0, 100, 50)), np.array((10, 255, 255)))
                        left_img = red_img[:, :w / 2]
                        left_input = np.mean(left_img) / 5

                        t.angular.z = -0.2  # Rotating until no longer seeing avoids stuck conditions
                        solver.publisher.publish(t)

                # -----Braitenberg 'love' behaviour for going to blue-----
                blue_img = cv.inRange(solver.hsv_img, np.array((100, 100, 50)), np.array((150, 255, 255)))

                if np.mean(blue_img) < blue_thresh:
                    left_img = blue_img[:, :w / 2]  # Cut the image horizontally to get sides of vision
                    right_img = blue_img[:, w / 2:]

                    left_input = -np.mean(left_img) / 10
                    right_input = -np.mean(right_img) / 10

                    (v, a) = forward_kinematics(left_input, right_input)
                    solver.angular_array.append(a * love_weight_blue)  # Add the love behaviour to the movements

                # -----Extract the kinematic results and publish-----
                t.linear.x = max(-0.1, min(sum(solver.velocity_array), 0.1))*control_weight
                t.angular.z = max(-0.1, min(sum(solver.angular_array), 0.1))*control_weight
                solver.publisher.publish(t)

                # -----Braitenberg 'love' behaviour for going to green-----
                green_img = cv.inRange(solver.hsv_img, np.array((60, 100, 50)), np.array((80, 255, 255)))

                left_img = green_img[:, :w / 2]  # Cut the image horizontally to get sides of vision
                right_img = green_img[:, w / 2:]

                left_input = -np.mean(left_img) / 10
                right_input = -np.mean(right_img) / 10

                if np.mean(green_img) > 30:
                    quit()

                (v, a) = forward_kinematics(left_input, right_input)
                solver.angular_array.append(a * love_weight_green)  # Add the love behaviour to the movements

                # -----Extract the kinematic results and publish-----
                t.linear.x = max(-0.1, min(sum(solver.velocity_array), 0.1)) * control_weight  # Clamp the max and min
                t.angular.z = max(-0.1, min(sum(solver.angular_array), 0.1)) * control_weight
                solver.publisher.publish(t)

            else:
                min_dist = min(ranges)
                min_index = ranges.index(min_dist)
                max_dist = max(ranges)
                max_index = ranges.index(max_dist)

                #err = max(-0.2, min((max_index - min_index), 0.2))

                err = -0.2

                t.angular.z = err
                solver.publisher.publish(t)
