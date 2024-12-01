#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros
from geometry_msgs.msg import TransformStamped


current_goal = PoseStamped()
current_goal.header.frame_id = "map"


def clicked_point_cb(msg: PoseStamped):
    global current_goal
    current_goal = msg


# if __name__ == "__main__":
#     rospy.init_node("goal_broadcaster")

#     rospy.Subscriber("/move_base_simple/goal", PoseStamped, clicked_point_cb)

#     transform = TransformStamped()
#     transform.header.stamp = rospy.Time.now()
#     transform.header.frame_id = "map"
#     transform.child_frame_id = "goal"
#     transform.transform.translation.x = current_goal.pose.position.x
#     transform.transform.translation.y = current_goal.pose.position.y
#     transform.transform.translation.z = 0
#     transform.transform.rotation.x = 0
#     transform.transform.rotation.y = 0
#     transform.transform.rotation.z = 0
#     transform.transform.rotation.w = 1

#     tf_broadcaster.sendTransform(transform)

if __name__ == "__main__":
    rospy.init_node("goal_broadcaster")

    # Initialize the TransformBroadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Subscribe to the RViz 2D Goal topic
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, clicked_point_cb)

    # Main loop
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if current_goal:  # If there is a valid goal
            # Create the TransformStamped message
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "map"
            transform.child_frame_id = "goal"
            transform.transform.translation.x = current_goal.pose.position.x
            transform.transform.translation.y = current_goal.pose.position.y
            transform.transform.translation.z = 0
            transform.transform.rotation.x = 0
            transform.transform.rotation.y = 0
            transform.transform.rotation.z = 0
            transform.transform.rotation.w = 1

            # Use tf_broadcaster to send the transform
            tf_broadcaster.sendTransform(transform)

        # Sleep to maintain the rate
        rate.sleep()
