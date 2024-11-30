#!/usr/bin/env python3

import rospy
import math
import tf2_ros
from geometry_msgs.msg import Twist
from tf2_geometry_msgs import TransformStamped


class JackalNavigation:
    def __init__(self):
        rospy.init_node("jackal_navigation")

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Velocity Publisher
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Parameters
        self.goal_frame = "goal"
        self.robot_frame = "base_link"
        self.rate = rospy.Rate(30)

    def get_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.robot_frame, self.goal_frame, rospy.Time(0), rospy.Duration(1.0)
            )
        except tf2_ros.LookupException:
            rospy.logwarn("Goal transform not found")
            return None

    def compute_cmd_vel(self, transform):
        dx = transform.transform.translation.x
        dy = transform.transform.translation.y
        distance = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx)

        cmd_vel = Twist()
        cmd_vel.linear.x = min(0.5, 0.5 * distance)
        cmd_vel.angular.z = 2.0 * angle
        return cmd_vel, distance

    def run(self):
        while not rospy.is_shutdown():
            transform = self.get_transform()
            if transform:
                cmd_vel, distance = self.compute_cmd_vel(transform)

                if distance < 0.5:  # Stop if close to the goal
                    cmd_vel = Twist()  # Stop the robot
                    rospy.loginfo("Goal reached")

                self.cmd_vel_pub.publish(cmd_vel)

            self.rate.sleep()


if __name__ == "__main__":
    nav = JackalNavigation()
    nav.run()