import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class imageRecognition:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)

    def callback(self, data):
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
                cv2.drawContours(hsv_img, c, -1, (255, 0, 0), 3)

        cv2.imshow("Image window", hsv_img)
        cv2.waitKey(1)


if __name__ == "__main__":
    imageRecognition()
    rospy.init_node('image_recognition', anonymous=True)
    rospy.spin()
    cv2.destroyAllWindows()
