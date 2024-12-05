#!/usr/bin/env python3

import rospy
import math
import tf2_ros
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32


class JackalNavigation:
    def __init__(self):
        rospy.init_node("mock_movebase_combined", anonymous=True)

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Velocity Publisher
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Parameters
        self.goal_frame = "goal"
        self.robot_frame = "base_link"
        self.rate = rospy.Rate(30)

        # Subscriber to green bounding box center
        self.green_bounding_box_sub = rospy.Subscriber("/green_bounding_box_center", Point, self.green_bounding_box_callback)

        self.green_box_center = None

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
        cmd_vel.linear.x = min(1, 1 * distance)
        cmd_vel.angular.z = 1.5 * angle
        return cmd_vel, distance

    def green_bounding_box_callback(self, msg):
        # Update the green box center (use only x and y)
        self.green_box_center = msg

    def run(self):
        while not rospy.is_shutdown():
            transform = self.get_transform()
            cmd_vel = Twist()

            if transform:
                cmd_vel, distance = self.compute_cmd_vel(transform)

                if distance < 0.5:  # Stop if close to the goal
                    cmd_vel = Twist()  # Stop the robot
                    rospy.loginfo("Goal reached")

            if self.green_box_center:
                # Adjust robot movement based on the green bounding box center
                center_x = self.green_box_center.x
                center_y = self.green_box_center.y
                rospy.loginfo(f"Center of green bounding box: ({center_x}, {center_y})")

                # For simplicity, use the x coordinate of the bounding box to adjust robot movement
                # Move towards the center of the green box (for front of the robot)
                cmd_vel.linear.x = max(0.1, min(1.0, (center_x - 320) / 320))  # Assume image width is 640px

            self.cmd_vel_pub.publish(cmd_vel)
            rospy.loginfo(f"Distance to goal: {distance:.2f}")

            self.rate.sleep()


if __name__ == "__main__":
    nav = JackalNavigation()
    nav.run()
