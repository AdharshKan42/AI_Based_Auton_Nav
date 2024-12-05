#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import sys
import select
import time

def create_goal_transform():
    # Initialize TF2 buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    try:
        rospy.loginfo(f"before lookup: {time.time()}")
        # Look up the transform from odom to base_link
        transform = tf_buffer.lookup_transform(
            "odom", "base_link", rospy.Time(0), rospy.Duration(5.0)
        )

        rospy.loginfo(f"after lookup: {time.time()}")


        # Create a new TransformStamped message for the goal
        goal_transform = TransformStamped()
        goal_transform.header.stamp = rospy.Time.now() 
        goal_transform.header.frame_id = "odom"
        goal_transform.child_frame_id = "goal"

        # Copy the translation and rotation from the transform
        goal_transform.transform.translation = transform.transform.translation
        goal_transform.transform.rotation = transform.transform.rotation

        # Publish the goal transform
        tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        tf_broadcaster.sendTransform(goal_transform)

        rospy.loginfo("Goal transform created:")
        rospy.loginfo(goal_transform)
    except tf2_ros.LookupException as e:
        rospy.logwarn("Failed to create goal transform: %s", e)

if __name__ == "__main__":
    rospy.init_node("goal_transform_creator")

    rospy.loginfo("Press 'w' to create a waypoint, 'q' to quit.")

    while not rospy.is_shutdown():
        try:
            key = input("Press w or 'q': ").strip()  # Blocking input
            if key == "w":
                rospy.loginfo("creating transform.")
                create_goal_transform() 
            elif key == "q":
                rospy.loginfo("Quitting.")
                break
        except Exception as e:
            rospy.logwarn(f"Keyboard listener error: {e}")