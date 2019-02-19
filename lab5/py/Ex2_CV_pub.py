#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


class unagi():

    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.image_sub = rospy.Subscriber("/camera/image",Image,self.rice)

    def rice(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,'bgr8'))
        except CvBridgeError as e:
            print(e)

def main():
    bowl = unagi()
    rospy.init_node("Image_Pub")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
