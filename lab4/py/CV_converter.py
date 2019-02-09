#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class cheese_burger():
    def __init__(self):
        self.cb_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)
        self.raw_image_sub = rospy.Subscriber("/camera/image",Image, self.pattie)

    def pattie(self,msg):

        cv_bridge = CvBridge()
        try:
            self.cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        self.cb_pub.publish(self.cv_image)

def main ():
    rospy.init_node("Image_Converter")
    try:
        cheese_burger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()