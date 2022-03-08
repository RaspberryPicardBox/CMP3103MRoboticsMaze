import numpy
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def run():
    rospy.spin()


class objectAvoidance:

    def __init__(self):
        rospy.init_node('objectAvoidance')
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.laser_scan = rospy.Subscriber('/scan', LaserScan, self.avoid)

    def avoid(self, laser_msg):
        t = Twist()
        if laser_msg.ranges[320] < 1.0:
            t.angular.z = 0.5
            self.publisher.publish(t)
        else:
            t.linear.x = 0.5
            self.publisher.publish(t)


o = objectAvoidance()
run()
