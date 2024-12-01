#!/usr/bin/env python3

import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from time import time
import geometry_msgs.msg
from waypoint_server.msg import Waypoint, WaypointArray
from functools import reduce

waypoints = []


def all_waypoints_cb(msg):
    global waypoints
    waypoints = list(
        map(
            lambda waypoint: waypoint.markers,
            filter(lambda waypoint: len(waypoint.markers) == 2, msg.waypoints),
        )
    )


if __name__ == "__main__":
    rospy.init_node("gate_broadcaster")

    tfBroadcaster = tf.TransformBroadcaster()
    tfListener = tf.TransformListener()

    subscriber = rospy.Subscriber("/waypoints/all", WaypointArray, all_waypoints_cb)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        for left, right in waypoints:
            try:
                (trans, rot) = tfListener.lookupTransform(
                    f"fiducial_{left}", f"fiducial_{right}", rospy.Time(0)
                )
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                continue

            trans = (trans[0] * 0.5, trans[1] * 0.5, trans[2] * 0.5)

            tfBroadcaster.sendTransform(
                trans,
                rot,
                rospy.Time.now(),
                f"fiducial_gate_{left}_{right}",
                f"fiducial_{left}",
            )

        rate.sleep()
