#!/usr/bin/env python
import sys 
import rospy
import cv2
import numpy as np 
import pytesseract 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class cheese_pizza:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image",Image,self.cheese)
        self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
        self.image_text = ""
        self.traj = Twist()
        self.speed = 0.2

    def cheese(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_image.shape
        
        height = height - (height % 32)
        width = width - (width % 32)

        #cv2.resize(cv_image,)
        res = cv2.resize(cv_image,(width, height), interpolation = cv2.INTER_CUBIC)

        cropped = res[40:75,135:185]

        cv2.imshow("HB_Camera",cropped)
        cv2.imshow("HB_Trans",res)

        self.thor(cropped)
        self.spin_to_win()

        self.cmd_pub.publish(self.traj)

        cv2.waitKey(3)
        

    def thor(self, img):
        config = ('-l eng --oem 1 --psm 3')
        self.image_text = pytesseract.image_to_string(img,config=config)
        #rospy.loginfo("{}".format(self.image_text))

    def spin_to_win(self):
        
        if self.image_text.isdigit():
            self.speed = float(self.image_text)/100.0

        if self.image_text.lower().find("stop") > 0:
            self.traj.linear.x = 0.0
            self.traj.angular.z = 0.0
        else:
            if self.image_text.lower() == "right":
                self.traj.linear.x = 0.0
                self.traj.angular.z = 0.0
            elif self.image_text.lower() == "left":
                self.traj.linear.x = 0.0
                self.traj.angular.z = -0.0
            else:
                self.traj.linear.x = 0.0
                self.traj.angular.z = 0.0
        rospy.loginfo("Text:'{}' X:{} Z:{}".format(self.image_text,self.traj.linear.x,self.traj.angular.z))
    
def main():
    ic = cheese_pizza()
    rospy.init_node("Image_Converter")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()