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



L_sub = [0,0]

def backprojection(data):
    global L_sub
    L_sub = np.array([[data.x],[data.y]])


class tracking(object):
    def __init__(self):
        self.bridge = CvBridge()
        # subscribe image message from webcam
        self.image_sub = rospy.Subscriber(
            "/cameras/left_hand_camera/image", Image, self.callback)
        # setup topic for images with detection results
        self.image_pub = rospy.Publisher(
            '/image_bridge', Image, latch=True, queue_size=10)
        self.PL_coo = rospy.Publisher(
            'Pcoordinate_left', Pose2D, latch=True, queue_size=10)


    def callback(self, data):
        # convert images from ROS msg format to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # construct the argument parse and parse the arguments
        greenLower = (29, 65, 47)
        greenUpper = (84, 175, 255)
        #pts = deque(maxlen=args["buffer"])
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
            cv2.imshow("Image windowL1",  mask)
            cv2.circle(cv_image, (int(x), int(y)),
                       int(radius), (0, 255, 255), 2)
            cv2.circle(cv_image, (int(x), int(y)), 5, (0, 0, 255), -1)
	    print('L_sub', L_sub)
            cv2.circle(cv_image, (L_sub[0], L_sub[1]), 5, (255, 0, 0), -1)

            cv2.imshow("Image windowL2",  cv_image)
            cv2.waitKey(3)

            self.PL_coo.publish(x,y,0)

def main(args):
    rospy.init_node('LeftLimb', anonymous=True)
    rospy.Subscriber('/backprojectl', Pose2D, backprojection)
    leftjointangles = baxter_interface.Limb('left').joint_angles()
    leftjointangles['left_s0'] = -1*leftjointangles['left_s0']
    leftjointangles['left_e0'] = -1*leftjointangles['left_e0']
    leftjointangles['left_w0'] = -1*leftjointangles['left_w0']
    leftjointangles['left_w2'] = -1*leftjointangles['left_w2']
    baxter_interface.Limb('right').set_joint_positions(leftjointangles, raw=False)

    tracking()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
