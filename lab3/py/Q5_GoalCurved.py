#!/usr/bin/env python

import rospy
from math import pi, atan, sin, cos, sqrt,asin
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#https://stackoverflow.com/questions/28910718/give-3-points-and-a-plot-circle
def define_circle(p1, p2, p3):
    """
    Returns the center and radius of the circle passing the given 3 points.
    In case the 3 points form a line, returns (None, infinity).
    """
    temp = p2[0] * p2[0] + p2[1] * p2[1]
    bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
    cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
    det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])

    if abs(det) < 1.0e-6:
        return (None, float('Inf'))

    # Center of circle
    cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
    cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det

    radius = sqrt((cx - p1[0])**2 + (cy - p1[1])**2)
    return circle(cx,cy,radius)

class circle():
    def __init__(self, x, y, rad):
        self.x = x
        self.y = y
        self.r = rad

class CC_Pie():

    def __init__(self, x=2, y=2, r=1):
        self.position = None
        self.obj_radius = r
        self.waypoints = []
        self.goalx = x
        self.goaly = y
        self.num_steps = 20


        self.cmd_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.traj = Twist()
        self.odom_sub = rospy.Subscriber ('/odom', Odometry, self.read_odm)
        self.rate = rospy.Rate(60)

        self.baking_cake()

    def read_odm(self, msg):
        self.position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

    def pre_calculate(self):
        self.calculate_arcpoint()
        self.calculate_circle()
        self.step_sampling()

    def calculate_arcpoint(self):
        midx = (self.goalx + self.position.x)/2
        midy = (self.goaly + self.position.y)/2

        hyp_c_a = sqrt(midx**2 + midy**2)
        hyp_c_b = sqrt(hyp_c_a**2 + self.obj_radius**2)

        theta_a = atan(midy/midx)
        if midx < 0 :
            theta_a = pi-theta_a

        theta_b = atan(self.obj_radius/hyp_c_a)
 
        theta_r = theta_a + theta_b        

        self.obj_y = hyp_c_b * sin(theta_r)
        self.obj_x = hyp_c_b * cos(theta_r)

    def calculate_circle(self):
        ph = (self.position.x, self.position.y)
        pm = (self.obj_x, self.obj_y)
        rospy.loginfo("Arc Midpoint - ({},{})".format(self.obj_x,self.obj_y))
        pg = (self.goalx, self.goaly)
        self.circle_ = define_circle (ph, pm, pg)
        rospy.loginfo("Circle - x:{}, y:{}, r:{}".format(self.circle_.x,self.circle_.y,self.circle_.r))

    def step_sampling(self):
        midx = (self.goalx + self.position.x)/2
        midy = (self.goaly + self.position.y)/2
        hyp_c_a = sqrt((midx-self.position.x)**2 + (midy-self.position.y)**2)

        theta_a = asin(hyp_c_a/self.circle_.r) * 2

        theta_step = theta_a/self.num_steps

        theta_n = asin(abs(self.position.y - self.circle_.y)/self.circle_.r)
        
        for i in range(1,self.num_steps):
            if self.circle_.x < self.position.x:
                rospy.loginfo("Hello?")
                x = self.circle_.r * cos((theta_n+theta_step*i))
                
                y = self.circle_.r * sin((theta_n+theta_step*i))
                rospy.loginfo("{} ({},{})".format(i,x,y))
            else:
                x = self.circle_.r * cos(pi-(theta_n+theta_step*i))
                y = self.circle_.r * sin(pi-(theta_n+theta_step*i))
            self.waypoints.append((x+self.circle_.x ,y+self.circle_.y))

    def set_orientation(self,waypoint):
        #rospy.loginfo("Correcting to ({},{})".format(waypoint[0],waypoint[1]))

        x_len = waypoint[0] - self.position.x
        y_len = waypoint[1] - self.position.y

        oren_to_way = (atan(y_len/x_len))
        if x_len < 0 :
            oren_to_way = oren_to_way + pi

        #rospy.loginfo("Current Yaw {} to Orientated Yaw {}".format(self.yaw,oren_to_way))

        if  abs(self.yaw - oren_to_way) < 0.05 :
            self.traj.angular.z = 0.0
        elif self.yaw < oren_to_way:
            #self.traj.linear.x = 0.0
            self.traj.angular.z = 0.3
        elif self.yaw > oren_to_way:
            self.traj.angular.z = -0.3
            #self.traj.linear.x = 0.0
            #self.traj.angular.z = -0.3
        else:
            rospy.loginfo("Wut?")

        self.cmd_pub.publish(self.traj)

        return self.traj.angular.z == 0.0

    def correct_orientation(self,waypoint):
        #rospy.loginfo("Correcting to ({},{})".format(waypoint[0],waypoint[1]))

        x_len = waypoint[0] - self.position.x
        y_len = waypoint[1] - self.position.y

        oren_to_way = (atan(y_len/x_len))

        if x_len < 0 :
            oren_to_way = oren_to_way + pi

        wumbo = self.traj.linear.x / self.circle_.r

        #rospy.loginfo("Current Yaw {} to Orientated Yaw {}".format(self.yaw,oren_to_way))

        if  abs(self.yaw - oren_to_way) < 0.05 :
            self.traj.angular.z = 0.0
        elif self.yaw < oren_to_way:
            #self.traj.linear.x = 0.0
            self.traj.angular.z = 0.3 
        elif self.yaw > oren_to_way:
            #self.traj.linear.x = 0.0
            self.traj.angular.z = -0.3
        else:
            rospy.loginfo("Wut?")

        self.cmd_pub.publish(self.traj)

        return self.traj.angular.z == 0.0

    def travel_forward(self,waypoint):
        
        waypointx = waypoint[0]
        waypointy = waypoint[1]

        if abs(self.position.x - self.goalx) < 0.15  and abs(self.position.y - self.goaly) < 0.15 :
            self.traj.linear.x = 0.0
        elif waypointx > self.position.x : 
            self.traj.linear.x = 0.2
        else:
            self.traj.linear.x = 0.2
        self.cmd_pub.publish(self.traj)

    def baking_cake(self):
        msg = rospy.wait_for_message("odom", Odometry)

        self.pre_calculate()
        cur_waypoint = self.waypoints[0]
        #rospy.loginfo("First WayPoint Set {},{}".format(cur_waypoint[0],cur_waypoint[1]))
        while not self.set_orientation(cur_waypoint):
            pass
        rospy.loginfo("Orientated - Begin Arc Traversal")

        for i in range(0,len(self.waypoints)):
            rospy.loginfo("[{}] Waypoint - ({},{})".format(i,self.waypoints[i][0],self.waypoints[i][1]))
        
        while not rospy.is_shutdown():
            #rospy.loginfo("Cur_Waypoint = ({},{})".format(cur_waypoint[0],cur_waypoint[1]))
            #rospy.loginfo("Cur_Position = ({},{})".format(self.position.x, self.position.y))
            
            if(self.correct_orientation(cur_waypoint)):
                self.travel_forward(cur_waypoint)

            if abs(self.position.x - cur_waypoint[0]) < 0.2 and abs(self.position.y - cur_waypoint[1]) < 0.2:
                if len(self.waypoints) == 0:
                    cur_waypoint = (self.goalx,self.goaly)
                else:
                    cur_waypoint = self.waypoints.pop(0)
                    rospy.loginfo("New waypoint - ({},{})".format(cur_waypoint[0],cur_waypoint[1]))
            
            self.rate.sleep()

    
def main():
    rospy.init_node("Curve_Traverse_Node")
    try:
        print("Input X Cordinate: ")
        goalx = input()
        print("Input Y Cordinate: ")
        goaly = input()
        print("Input Radisu")
        r = input()
        CC_Pie(goalx, goaly, r)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()