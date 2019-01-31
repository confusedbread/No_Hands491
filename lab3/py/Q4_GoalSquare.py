#!/usr/bin/env python

import rospy
from math import pi, atan, sin, cos, sqrt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Lemon_Drop_Pie():

    def __init__(self,x = 2, y =2, r=1):
        self.position = None
        self.obj_radius = r
        self.cmd_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.traj = Twist()
        self.rate = rospy.Rate(60)

        self.goalx = x
        self.goaly = y
        #self.goalz = 0

        self.waypointx = None
        self.waypointy = None

        self.odom_sub = rospy.Subscriber ('/odom', Odometry, self.read_odm)
        self.baking_cake()

    def read_odm(self,msg):
        self.position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

    def correct_orientation(self):
        
        x_len = self.waypointx - self.position.x
        y_len = self.waypointy - self.position.y

        oren_to_goal = atan(y_len/x_len)

        #rospy.loginfo("radian : {}".format(oren_to_goal))

        if  abs(self.yaw - oren_to_goal) < 0.05 or \
            (abs(self.position.x - self.waypointx) < 0.05 and \
            abs(self.position.y - self.waypointy) < 0.05):
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

    def calculate_arcpoint(self):
        midx = (self.goalx + self.position.x)/2
        midy = (self.goaly + self.position.y)/2

        hyp_c_a = sqrt(midx**2 + midy**2)
        hyp_c_b = sqrt(hyp_c_a**2 + self.obj_radius**2)

        theta_a = atan(midy/midx)
        theta_b = atan(self.obj_radius/hyp_c_b)

        theta_r = theta_a + theta_b        

        self.obj_y = hyp_c_b * sin(theta_r)
        self.obj_x = hyp_c_b * cos(theta_r)

    def travel_forward(self):
        
        arrive = False

        if abs(self.position.x - self.waypointx) < 0.05 and abs(self.position.y - self.waypointy) < 0.05:
            self.traj.linear.x = 0.0
            arrive = True
        elif self.waypointx > self.position.x:
            self.traj.linear.x = 0.3
        else:
            self.traj.linear.x = -0.3
        
        self.cmd_pub.publish(self.traj)

        return arrive

    def baking_cake(self):
        #Insure first message comes in to populate Position
        msg = rospy.wait_for_message("odom",Odometry)

        #Calculate Arcpoint from midpoint
        self.calculate_arcpoint()
        rospy.loginfo("ArcPoint Calculated: {},{}".format(self.obj_x,self.obj_y))
        
        #Configurate Waypoint for arcpoint
        self.waypointx = self.obj_x
        self.waypointy = self.obj_y
        rospy.loginfo("WayPoint Set for: {},{}".format(self.waypointx,self.waypointy))
        rospy.loginfo("Baking Start")

        while not self.correct_orientation():
            pass
        rospy.loginfo("Proceed to Object WayPoint")
        while not rospy.is_shutdown():
            if self.correct_orientation():
                if self.travel_forward():
                    rospy.loginfo("Arrived at WayPoint")
                    self.waypointx = self.goalx
                    self.waypointy = self.goaly
 
            self.rate.sleep()

def main():
    rospy.init_node("Traverse_Node")
    try:
        print("Input X Cordinate: ")
        goalx = input()
        print("Input Y Cordinate: ")
        goaly = input()
        print("Input Radisu")
        r = input()
        Lemon_Drop_Pie(goalx, goaly, r)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()