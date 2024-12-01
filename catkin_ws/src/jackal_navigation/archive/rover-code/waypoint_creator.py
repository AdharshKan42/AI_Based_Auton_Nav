#!/usr/bin/env python3

import roslib
import rospy
import math
import geometry_msgs.msg
from time import time
from geometry_msgs.msg import TransformStamped
from waypoint_server.msg import Waypoint, WaypointArray
from functools import reduce
import os
from pathlib import Path
import tf2_ros
from typing import List


waypoints: List[TransformStamped] = []
save_dir = Path(__file__).parent.parent / "waypoints"


def export():
    global waypoints, save_dir
    if len(waypoints) == 0:
        return
    with open(save_dir / f"{time()}.launch", "w") as file:
        for index, waypoint in enumerate(waypoints):
            x = waypoint.transform.translation.x
            y = waypoint.transform.translation.y
            z = waypoint.transform.translation.z
            rx = waypoint.transform.rotation.x
            ry = waypoint.transform.rotation.y
            rz = waypoint.transform.rotation.z
            rw = waypoint.transform.rotation.w
            file.write(
                f'<node pkg="tf2_ros" type="static_transform_publisher" name="waypoint_broadcaster_{index}" args="{x} {y} {z} {rx} {ry} {rz} {rw} map waypoint_{index}" />\n'
            )


if __name__ == "__main__":
    rospy.init_node("waypoint_creator")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.on_shutdown(export)

    while not rospy.is_shutdown():

        if input("Press enter to save current location! 'q' to quit").lower() == "q":
            break

        try:
            goal_transform = tf_buffer.lookup_transform(
                "map", "base_link", rospy.Time.now(), rospy.Duration(5)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logerr("Failed to save waypoint")
            continue

        waypoints.append(goal_transform)

        rospy.loginfo(
            f"Waypoint saved! {goal_transform.transform.translation.x, goal_transform.transform.translation.y, goal_transform.transform.translation.z}"
        )
