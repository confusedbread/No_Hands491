#!/usr/bin/env python

import rospy
from math import pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class BlueBerry_Pie():
    def __init__(self):
        self.orgin_roll = None
        self.orgin_pitch = None
        self.orgin_yaw = None
        self.ODOM_ERR = 0.03

        self.set_euler()
        
        self.traj = Twist()
        self.rate = rospy.Rate(100)
        self.cur_msg = []
        self.cmd_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        rospy.loginfo("BlueBerry Pie is Initiated")
        self.bake_the_cake()

    def set_euler(self):
        msg = rospy.wait_for_message("odom",Odometry)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.orgin_roll, self.orgin_pitch, self.orgin_yaw) = euler_from_quaternion (orientation_list)

    def test_orientation(self):
        msg = rospy.wait_for_message("odom",Odometry)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        
        rospy.loginfo("Original Yaw : {} :: Cur_Yaw {} :: abs {}".format(self.orgin_yaw, yaw,abs(self.orgin_yaw - yaw)))

        if -0.58 < yaw - self.orgin_yaw and yaw - self.orgin_yaw < 0.02:
            return True
        else:
            return False

    def start_circle(self):
        rospy.loginfo("Start Circle")
        self.traj.linear.x = 0.5
        self.traj.angular.z = 90*pi/180
        self.cmd_pub.publish(self.traj)
        rospy.loginfo("Twist Sent: x = {} : z = {}".format(self.traj.linear.x,self.traj.angular.z))
        rospy.sleep(1.)

    def stop(self):
        self.traj.linear.x = 0
        self.traj.angular.z = 0
        self.cmd_pub.publish(self.traj)
        rospy.loginfo("Stopping : Wait for 3 Seconds")
        rospy.sleep(3.)

    def bake_the_cake(self):

        while not rospy.is_shutdown():
            if self.test_orientation():
                self.stop()
                self.start_circle()
            
            self.rate.sleep()
            
def main():
    rospy.init_node("better_circle_node")
    try:
        BlueBerry_Pie()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()