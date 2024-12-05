#!/usr/bin/env python3

import roslib
import rospy
import math
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from time import time

currentGoal = PoseStamped()
currentGoal.header.frame_id = "odom"


def clicked_point_cb(msg: PoseStamped):
    currentGoal.pose = msg.pose


if __name__ == "__main__":
    rospy.init_node("mock_rover")
    tfbroadcaster = tf.TransformBroadcaster()
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, clicked_point_cb)
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        tfbroadcaster.sendTransform(
            (
                currentGoal.pose.position.x,
                currentGoal.pose.position.y,
                currentGoal.pose.position.z,
            ),
            (0, 0, 0, 1),
            rospy.Time.now(),
            "goal",
            "map",
        )
        rate.sleep()
