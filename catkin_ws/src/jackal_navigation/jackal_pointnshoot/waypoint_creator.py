#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

waypoints = []


def save_waypoint():
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    try:
        transform = tf_buffer.lookup_transform(
            "map", "base_link", rospy.Time.now(), rospy.Duration(5.0)
        )
        waypoints.append(transform)
        rospy.loginfo("Waypoint saved: %s", transform.transform)
    except tf2_ros.LookupException:
        rospy.logwarn("Failed to save waypoint")


if __name__ == "__main__":
    rospy.init_node("waypoint_creator")

    rospy.loginfo("Press Enter to save a waypoint, 'q' to quit")

    while input() != "q":
        save_waypoint()
