#!/usr/bin/env python

import rospy
from math import pi, atan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Apple_Pie():

    def __init__(self,x = 2, y =2):
        self.position = None
        self.cmd_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.traj = Twist()
        self.rate = rospy.Rate(60)

        self.goalx = x
        self.goaly = y
        #self.goalz = 0

        self.odom_sub = rospy.Subscriber ('/odom', Odometry, self.read_odm)
        self.baking_cake()

    def read_odm(self,msg):
        self.position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

    def correct_orientation(self):
        
        x_len = self.goalx - self.position.x
        y_len = self.goaly - self.position.y

        oren_to_goal = atan(y_len/x_len)

        #rospy.loginfo("radian : {}".format(oren_to_goal))

        if  abs(self.yaw - oren_to_goal) < 0.05 or \
            (abs(self.position.x - self.goalx) < 0.05 and \
            abs(self.position.y - self.goaly) < 0.05):
            self.traj.angular.z = 0.0
        elif self.yaw < oren_to_goal:
            #self.traj.linear.x = 0.0
            self.traj.angular.z = 0.3
        elif self.yaw > oren_to_goal:
            #self.traj.linear.x = 0.0
            self.traj.angular.z = -0.3
        else:
            rospy.loginfo("Wut?")

        self.cmd_pub.publish(self.traj)

        return self.traj.angular.z == 0.0

    def travel_forward(self):
        
        if abs(self.position.x - self.goalx) < 0.05 and abs(self.position.y - self.goaly) < 0.05:
            self.traj.linear.x = 0.0
        elif self.goalx > self.position.x:
            self.traj.linear.x = 0.3
        else:
            self.traj.linear.x = -0.3
        
        self.cmd_pub.publish(self.traj)

    def baking_cake(self):
        msg = rospy.wait_for_message("odom",Odometry)
        rospy.loginfo("Baking Start")
        while not self.correct_orientation():
            pass
        rospy.loginfo("Proceed towards goal")
        while not rospy.is_shutdown():
            if self.correct_orientation():
                self.travel_forward() 

            self.rate.sleep()

def main():
    rospy.init_node("Traverse_Node")
    try:
        print("Input X Cordinate: ")
        goalx = input()
        print("Input Y Cordinate: ")
        goaly = input()
        Apple_Pie(goalx, goaly)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()