#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from pkg.msg import Observation
from tf.transformations import euler_from_quaternion

import numpy as np
import os
import time
import rosgraph
import socket

from pkg.drivers import DisparityExtender as Driver

"""
NOTE: Following code enables F1Tenth - Docker - ROS integration.  
Please don't change it, unless you know what you're doing
"""


def _unpack_odom(pose: PoseWithCovariance, twist: TwistWithCovariance):
    pose_x = pose.pose.position.x
    pose_y = pose.pose.position.y

    euler = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
                                   pose.pose.orientation.w])
    pose_theta = euler[2]

    linear_vel_x = twist.twist.linear.x
    linear_vel_y = twist.twist.linear.y
    angular_vel_z = twist.twist.angular.z
    return {
        'pose_x': pose_x,
        'pose_y': pose_y,
        'pose_theta': pose_theta,
        'linear_vel_x': linear_vel_x,
        'linear_vel_y': linear_vel_y,
        'angular_vel_z': angular_vel_z,
    }


class ROSRunner:
    def __init__(self, driver, agent_name):
        self.driver = driver
        self.agent_name = agent_name
        self.pub_drive = None
        self.roscore_started = False
        self.last_healthy_time = None

    def lidar_callback(self, data):
        ranges = np.asarray(data.ranges)
        self.last_healthy_time = time.time()
        return self.driver.process_lidar(ranges)

    def observation_callback(self, data):
        self.last_healthy_time = time.time()

        if hasattr(self.driver, 'process_observation'):
            # FIXME: HANDLE Reversing odom to params & additional parameter to agents
            ego_odom = _unpack_odom(data.ego_pose, data.ego_twist)
            speed, angle = self.driver.process_observation(
                ranges=data.ranges,
                ego_odom=ego_odom,
            )
        elif hasattr(self.driver, 'process_lidar'):
            # For users whom developed their lidar function
            speed, angle = self.lidar_callback(data)

        # create message & publish
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.pub_drive.publish(msg)

    def run(self):
        rospy.init_node('gym_agent_%s' % self.agent_name, anonymous=True)
        self.pub_drive = rospy.Publisher('/%s/drive' % self.agent_name, AckermannDriveStamped, queue_size=5)

        # start listening
        rospy.Subscriber('/%s/observations' % self.agent_name, Observation, self.observation_callback)
        rospy.sleep(3)

        while not rospy.core.is_shutdown():
            current_time = time.time()

            # if it passed more than 5 seconds since last sensor
            # try to connect to ROS, if you cannot
            if self.last_healthy_time and current_time > self.last_healthy_time + 5:
                try:
                    rosgraph.Master('/rostopic').getPid()
                except socket.error:
                    print ("ROS is shutdown, shutting down agent node")
                    return
                else:
                    print ("ROS is running, but agent didn't receive observation for a long time. ")
                    print ("You can shutdown your agent docker container if you want to.")
                    self.last_healthy_time = time.time()

            rospy.rostime.wallsleep(0.04)


if __name__ == "__main__":
    agent_name = os.environ.get("F1TENTH_AGENT_NAME")
    runner = ROSRunner(Driver(), agent_name)

    print("Starting Agent: %s" % agent_name)

    # launch
    runner.run()

    print("Agent run finished")
