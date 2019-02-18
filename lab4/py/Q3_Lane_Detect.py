#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image",Image,self.callback)

  def pizza_slice(self,img,vertices):
    mask = np.zeros_like(img)
    channel_count = 1
    match_mask_color = (255,) * channel_count
    cv2.fillPoly(mask, vertices, match_mask_color)
    
    # Returning the image only where mask pixels match
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)



    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) # Convert to grayscale
    cv_image = cv2.blur(cv_image,(5,5))	 # Blur image
    cv_image = cv2.Canny(cv_image,50,150)

    
    height, width = cv_image.shape

    rospy.loginfo("{} {}".format(height,width))

    region_of_interest_vertices = [
        (-75, height),
        (width / 2, (height / 2)-30),
        (width+ 75, height),
    ]

    cropped_image = self.pizza_slice(cv_image,np.array([region_of_interest_vertices], np.int32))

    cv2.imshow("Image window", cropped_image)
    cv2.waitKey(3)



def main(args):
  ic = image_converter()
  rospy.init_node('test_cv', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
