#!/usr/bin/env python3

import roslib
import rospy
import math
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Twist, PoseStamped
from time import time


class MockRover:
    def __init__(self):
        rospy.Subscriber("/cmd_vel", Twist, self.update_velocity)
        self.position_pub = rospy.Publisher("/rover/pose", PoseStamped, queue_size=10)
        goal = PoseStamped()
        self.velocity = Twist()
        self.position = goal
        self.last_received_time = time()
        self.broadcaster = tf.TransformBroadcaster()

    def update_velocity(self, msg: Twist):
        self.velocity = msg

    def update_position(self):
        dt = time() - self.last_received_time

        self_euler = euler_from_quaternion(
            [
                self.position.pose.orientation.x,
                self.position.pose.orientation.y,
                self.position.pose.orientation.z,
                self.position.pose.orientation.w,
            ]
        )

        self.position.pose.position.x += (
            self.velocity.linear.x * dt * math.cos(self_euler[2])
        )
        # rospy.loginfo(f"lin.y: {self.velocity.linear.y}, dt: {dt}, math.sin(self_euler[2])={math.sin(self_euler[2])}")
        self.position.pose.position.y += (
            self.velocity.linear.x * dt * math.sin(self_euler[2])
        )
        self.position.pose.position.z += 0

        # Convert self.position.pose.orietination -> rx, ry, rz
        # new_rx = rx + velocity.angular.x * dt
        # ...
        # quat = quaternion_from_euler(new_rx, ...)

        quaternion = quaternion_from_euler(
            self_euler[0] + self.velocity.angular.x * dt,
            self_euler[1] + self.velocity.angular.y * dt,
            self_euler[2] + self.velocity.angular.z * dt,
        )

        self.position.pose.orientation.x = quaternion[0]
        self.position.pose.orientation.y = quaternion[1]
        self.position.pose.orientation.z = quaternion[2]
        self.position.pose.orientation.w = quaternion[3]

        self.last_received_time = time()

    def run(self):
        rate = rospy.Rate(30)
        self.last_received_time = time()
        while not rospy.is_shutdown():

            self.update_position()
            self.position_pub.publish(self.position)
            self.broadcaster.sendTransform(
                (
                    self.position.pose.position.x,
                    self.position.pose.position.y,
                    self.position.pose.position.z,
                ),
                (
                    self.position.pose.orientation.x,
                    self.position.pose.orientation.y,
                    self.position.pose.orientation.z,
                    self.position.pose.orientation.w,
                ),
                rospy.Time.now(),
                "base_link",
                "odom",
            )

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("mock_rover")
    MockRover().run()
