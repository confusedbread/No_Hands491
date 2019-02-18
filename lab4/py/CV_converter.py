#!/usr/bin/env python

import rospy
import cv2
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class cheese_burger():
    def __init__(self):
        #self.cb_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)
        self.raw_image_sub = rospy.Subscriber("/camera/image",Image, self.pattie)

    def pattie(self,msg):

        cv_bridge = CvBridge()
        try:
            self.cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("HB_Camera", self.cv_image)
        cv2.waitKey(3)


def main ():
    ic = cheese_burger()
    rospy.init_node("Image_Converter")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()