#!/usr/bin/env python

# Author: Felix Chiang
# Date: 1/30/2019
# Description: Robot will travel in a circle for 4 seconds completing a full 
#              rotation. After the circle in complete it will then pause for
#              3 seconds and repeat this protocol.
#

import rospy
from math import pi
from geometry_msgs.msg import Twist

class circle():
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.go_circle()
    
    def go_circle(self):
        go = True
        traj = Twist()

        while not rospy.is_shutdown():
            
            if go:
                traj.linear.x = 0.5
                traj.angular.z = 90*pi/180
            else:
                traj.linear.x = 0
                traj.angular.z = 0
            
            self.pub.publish(traj)

            if go:
                go = False
                rospy.sleep(4.)
            else:
                go = True
                rospy.sleep(3.)

            

if __name__ == '__main__':
    rospy.init_node('circle_node')
    try:
        circle()
    except rospy.ROSInterruptException:
        pass
