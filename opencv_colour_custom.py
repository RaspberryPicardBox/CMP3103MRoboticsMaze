import rospy
from std_msgs.msg import String
import cv2
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class image_converter:

    def __init__(self):

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw",
            Image, self.image_callback)

        self.image_pub = rospy.Publisher("/image_result", String, queue_size=1)

    def image_callback(self, data):
        cv2.namedWindow("Image window")
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((0, 150, 50)),
                                 numpy.array((255, 255, 255)))

        _, hsv_contours, hierarchy = cv2.findContours(
            hsv_thresh.copy(),
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)

        for c in hsv_contours:
            a = cv2.contourArea(c)
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0), 3)

        self.image_pub.publish(str(numpy.mean(hsv_img)))

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node('image_converter')
    ic = image_converter()
    rospy.spin()