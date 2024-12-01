#!/usr/bin/env python3

import rospy
import math
import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry

class MockJackal:
    def __init__(self):
        rospy.init_node("mock_jackal_simulation")

        # Publisher for robot's pose
        self.pose_pub = rospy.Publisher("/rover/pose", Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscriber for cmd_vel
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)

        # Parameters for simulation
        self.rate = rospy.Rate(30)
        self.pose = [0.0, 0.0, 0.0]  # x, y, theta
        self.velocity = [0.0, 0.0]  # linear, angular
        self.last_time = rospy.Time.now()

    def cmd_vel_cb(self, msg: Twist):
        """Callback to receive velocity commands."""
        self.velocity[0] = msg.linear.x
        self.velocity[1] = msg.angular.z

    def update_pose(self):
        """Update the robot's pose based on received velocities."""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Update position
        self.pose[2] += self.velocity[1] * dt  # Update theta (angular position)
        self.pose[2] = math.atan2(math.sin(self.pose[2]), math.cos(self.pose[2]))  # Normalize theta
        self.pose[0] += self.velocity[0] * math.cos(self.pose[2]) * dt  # Update x
        self.pose[1] += self.velocity[0] * math.sin(self.pose[2]) * dt  # Update y

        # Publish transform
        self.publish_transform()

    def publish_transform(self):
        """Publish the robot's current pose as a transform."""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.pose[0]
        t.transform.translation.y = self.pose[1]
        t.transform.translation.z = 0.0

        # Convert theta to quaternion
        q = self.yaw_to_quaternion(self.pose[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def publish_pose(self):
        """Publish the robot's pose as an Odometry message."""
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.pose[0]
        odom.pose.pose.position.y = self.pose[1]
        odom.pose.pose.position.z = 0.0

        q = self.yaw_to_quaternion(self.pose[2])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.angular.z = self.velocity[1]

        self.pose_pub.publish(odom)

    def yaw_to_quaternion(self, yaw):
        """Convert a yaw angle to a quaternion."""
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

    def run(self):
        """Run the simulation loop."""
        while not rospy.is_shutdown():
            self.update_pose()
            self.publish_pose()
            self.rate.sleep()


if __name__ == "__main__":
    rover = MockJackal()
    rover.run()