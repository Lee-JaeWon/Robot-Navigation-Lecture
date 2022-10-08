#!/usr/bin/env python
import rospy
import tf   
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg
from math import sqrt, cos, sin, pi, atan2
import numpy
import sys

class PID:
    def __init__(self, Kp, Td, Ti, dt):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.dt = dt
        
    def update_control(self, current_error, reset_prev=False):
        # todo: implement this

        self.curr_error_deriv = (self.curr_error - self.prev_error)/self.dt
        # print(self.curr_error)
        P_eq = self.Kp * self.curr_error
        self.sum_error += self.curr_error
        I_eq = self.Ti * self.sum_error
        D_eq = self.Td * self.curr_error_deriv

        PID_eq = P_eq + I_eq + D_eq

        self.control = PID_eq 

        self.prev_error = self.curr_error
        self.curr_error = current_error
        self.prev_error_deriv = self.curr_error_deriv


        return self.control
        
    def get_control(self):
        return self.control
        
class WallFollowerHusky:
    def __init__(self):
        rospy.init_node('wall_follower_husky', anonymous=True)

        self.forward_speed = rospy.get_param("~forward_speed")
        self.desired_distance_from_wall = rospy.get_param("~desired_distance_from_wall")
        self.hz = 50
        self.count = 0

        # todo: set up the command publisher to publish at topic '/husky_1/cmd_vel'
        # using geometry_msgs.Twist messages
        self.cmd_pub = rospy.Publisher("/husky_1/cmd_vel", Twist, queue_size=10)

        # todo: set up the laser scan subscriber
        # this will set up a callback function that gets executed
        # upon each spinOnce() call, as long as a laser scan
        # message has been published in the meantime by another node
        self.laser_sub = rospy.Subscriber("/husky_1/scan", LaserScan, self.laser_scan_callback)
        # self.pid = PID(2.8,3.0, 0.0, 0.02) # PDI
        self.pid = PID(2.3, 2.0, 0.01, 0.02) # PDI        
        
    def laser_scan_callback(self, msg):
        # todo: implement this
        # Populate this command based on the distance to the closest
        # object in laser scan. I.e. compute the cross-track error
        # as mentioned in the PID slides.

        # You can populate the command based on either of the following two methods:
        # (1) using only the distance to the closest wall
        # (2) using the distance to the closest wall and the orientation of the wall
        #
        # If you select option 2, you might want to use cascading PID control. 
  
        # cmd.angular.z = 5

        wall_dist = min(msg.ranges) - self.desired_distance_from_wall
        
        angle = self.pid.update_control(wall_dist)

        linear = Vector3(self.forward_speed, 0, 0) # self.forward_speed

        angular = Vector3(0, 0, angle)
        twist = Twist(linear, angular)

        self.cmd_pub.publish(twist)
            
    def run(self):
        rate = rospy.Rate(self.hz)
        rospy.spin()
        while not rospy.is_shutdown():
            rate.sleep()

    
if __name__ == '__main__':
    wfh = WallFollowerHusky()
    wfh.run()


