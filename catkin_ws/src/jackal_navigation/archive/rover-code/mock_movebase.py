#!/usr/bin/env python3

import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from time import time

# from std_srvs import Trigger

name = ""
markers = []
marker_index = 0

# def waypoint_cb(msg: Waypoint):
#     global name
#     global markers
#     name = msg.name
#     markers = msg.markers

if __name__ == "__main__":
    rospy.init_node("mock_movebase")

    listener = tf.TransformListener()

    # rospy.wait_for_message("/tf")

    publisher = rospy.Publisher("/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10)
    test_transforms = [
        "/fiducial_1",
        "/fiducial_2",
        "/fiducial_3",
        "/fiducial_4",
        "/goal",
    ]
    found_time = None

    while not rospy.is_shutdown():

        if found_time is not None:
            if time() - found_time < 5:
                rospy.loginfo_throttle(1, "Waiting for 5 seconds")
                cmd_vel = geometry_msgs.msg.Twist()
                publisher.publish(cmd_vel)
                # rate.sleep()
                continue
            else:
                found_time = None

        index = 0
        while not listener.canTransform(
            "/base_link", test_transforms[index], rospy.Time(0)
        ):
            # only increment if test_transforms can not
            if index < len(test_transforms) - 1:
                index += 1

        (trans, rot) = [], []
        try:
            (trans, rot) = listener.lookupTransform(
                "/base_link", test_transforms[index], rospy.Time(0)
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            # just ignore
            continue

        # subscriber = rospy.Subscriber('/waypoints/current', Waypoint, waypoint_cb)
        # try:
        #     # (trans, rot) = listener.lookupTransform('/base_link', "/waypoints/current", rospy.Time(0))
        #     if len(markers) == 0:
        #         (trans, rot) = listener.lookupTransform('/base_link', name, rospy.Time(0))
        #     else:
        #         (trans, rot) = listener.lookupTransform('/base_link', markers[marker_index], rospy.Time(0))

        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     # just ignore
        #     continue

        cmd_vel = geometry_msgs.msg.Twist()

        angular = 2 * math.atan2(trans[1], trans[0])
        distance = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        linear = min(0.5, 0.5 * distance)

        cmd_vel.linear.x = linear
        cmd_vel.angular.z = angular

        publisher.publish(cmd_vel)

        if distance <= 1:
            if test_transforms[index] != "/goal":
                test_transforms.remove(test_transforms[index])
                found_time = time()

        rate.sleep()
