#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math
import sys
import tf
import numpy as np
from sensor_msgs.msg import Image
import baxter_interface
from geometry_msgs.msg import Pose2D
 
R_sub = [0,0]
 
def backprojection(data):
    global R_sub
    R_sub = np.array([[data.x],[data.y]])
 
class tracking(object):
    def __init__(self):
        self.bridge = CvBridge()
        # bridge to use opencv to process images
        self.image_sub = rospy.Subscriber(
        "/cameras/right_hand_camera/image", Image, self.callback)
        self.image_pub = rospy.Publisher(
            '/image_bridge', Image, latch=True, queue_size=10)
        self.PR_coo = rospy.Publisher(
            'Pcoordinate_right', Pose2D, latch=True, queue_size=10)
 
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # convert images from ROS msg format to opencv
        greenLower = (29, 65, 47)
        greenUpper = (84, 175, 255)
        # set color hsv threshold
        cv_image = cv2.medianBlur(cv_image, 5)
        cv_image = cv2.bilateralFilter(cv_image, 5, 100, 100)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            cv2.imshow("Image windowR1",  mask)
            cv2.circle(cv_image, (int(x), int(y)),
                       int(radius), (0, 255, 255), 2)
            cv2.circle(cv_image, (int(x), int(y)), 5, (0, 0, 255), -1)
            cv2.circle(cv_image, (R_sub[0], R_sub[1]), 5, (255, 0, 0), -1)
            cv2.imshow("Image windowR2",  cv_image)
            cv2.waitKey(3)
 
            self.PR_coo.publish(x,y,0)
 
def main(args):
    rospy.init_node('RightLimb', anonymous=True)
    rospy.Subscriber('/backprojectr', Pose2D, backprojection)
 
    tracking()
 
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
 
cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)
