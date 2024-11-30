#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros

current_goal = PoseStamped()
current_goal.header.frame_id = "map"


def clicked_point_cb(msg: PoseStamped):
    global current_goal
    current_goal = msg


if __name__ == "__main__":
    rospy.init_node("goal_broadcaster")

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, clicked_point_cb)

    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if current_goal:
            tf_broadcaster.sendTransform(
                (current_goal.pose.position.x, current_goal.pose.position.y, 0),
                (0, 0, 0, 1),
                rospy.Time.now(),
                "goal",
                "map",
            )
        rate.sleep()
