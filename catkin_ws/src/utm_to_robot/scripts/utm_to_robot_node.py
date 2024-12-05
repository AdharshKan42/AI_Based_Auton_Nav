#!/usr/bin/env python

import rospy
import math
import utm
import numpy as np
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TransformStamped
import tf
import tf2_ros


class UTMTOROVER:
    def __init__(self):
        rospy.init_node("utm_to_robot", anonymous=True)

        # Subscribers
        self.gps_fix_sub = rospy.Subscriber(
            "/navsat/fix", NavSatFix, self.gps_fix_sub_cb
        )
        self.navheading_sub = rospy.Subscriber(
            "/imu/data", Imu, self.navheading_sub_cb
        )

        # TF Broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.rate = rospy.Rate(50)  # 50 Hz

        self.navheading = None
        self.utm_x = None
        self.utm_y = None

        # Transform from UTM to Map
        self.T_UM = None

        # Main loop
        self.run()

    def gps_fix_sub_cb(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude

        easting, northing, zone_number, zone_letter = utm.from_latlon(
            self.lat, self.lon
        )
        self.utm_x = easting
        self.utm_y = northing

    def navheading_sub_cb(self, msg):
        # The /navheading topic gives the absolute orientation CCW positive,
        # starting South = 0 degrees.
        # 1. Convert quaternion to yaw.
        yaw_from_north = 2 * math.atan2(msg.orientation.z, msg.orientation.w)
        # 2. Express this heading in ENU frame (East = 0)
        self.navheading = (yaw_from_north - math.pi / 2) % (2 * math.pi)

    def broadcast_transform(self):
        # Initialize map frame with the initial pose of robot in utm frame.
        # This means our map frame coincides with base_link frame at the start.
        if self.T_UM is None:
            if (
                self.navheading is not None
                and self.utm_x is not None
                and self.utm_y is not None
            ):
                # Print current status
                rospy.loginfo("UTM -> MAP: ")
                rospy.loginfo(f"GPS COORDINATES: {self.lat=}, {self.lon=}")
                rospy.loginfo(
                    f"Navheading(East=0, degrees): {math.degrees(self.navheading)},"
                    f"UTM X coord: {self.utm_x}, UTM Y coord: {self.utm_y}"
                )

                # Set the translation and rotation
                translation = [self.utm_x, self.utm_y, 0.0]
                rotation = tf.transformations.quaternion_from_euler(
                    0, 0, self.navheading
                )

                # Store this UTM to Map frame
                t_UM = np.array(translation).reshape(3, 1)
                quaternion = np.array(
                    [0.0, 0.0, math.sin(self.navheading / 2), math.cos(self.navheading / 2)]
                )
                R_UM = Rotation.from_quat(quaternion).as_matrix()
                T_UM = np.hstack((R_UM, t_UM))
                T_UM = np.vstack((T_UM, np.array([0, 0, 0, 1])))
                self.T_UM = T_UM

                # Send the static transform
                static_transform = TransformStamped()
                static_transform.header.stamp = rospy.Time.now()
                static_transform.header.frame_id = "utm"
                static_transform.child_frame_id = "map"
                static_transform.transform.translation.x = translation[0]
                static_transform.transform.translation.y = translation[1]
                static_transform.transform.translation.z = translation[2]
                static_transform.transform.rotation.x = rotation[0]
                static_transform.transform.rotation.y = rotation[1]
                static_transform.transform.rotation.z = rotation[2]
                static_transform.transform.rotation.w = rotation[3]

                self.static_tf_broadcaster.sendTransform(static_transform)
        else:
            # Compute UTM->Baselink from sensor,
            # then publish transform MAP->BASELINK based on UTM->MAP and this.
            # Notation: U = utm frame, B = base link frame, M = map frame.
            # Given T_UB, I need T_MB

            # Compute T_UB from gps data
            t_UB = np.array([self.utm_x, self.utm_y, 0.0]).reshape(3, 1)
            cos_heading = np.cos(self.navheading)
            sin_heading = np.sin(self.navheading)
            R_UB = np.array(
                [
                    [cos_heading, -sin_heading, 0],
                    [sin_heading, cos_heading, 0],
                    [0, 0, 1],
                ]
            )
            T_UB = np.hstack((R_UB, t_UB))
            T_UB = np.vstack((T_UB, np.array([0, 0, 0, 1])))

            # Compute T_MB from T_UB = T_UM @ T_MB
            T_MB = np.linalg.inv(self.T_UM) @ T_UB

            # Publish T_MB
            translation = T_MB[:3, 3]
            rotation = Rotation.from_matrix(T_MB[:3, :3]).as_quat()
            self.tf_broadcaster.sendTransform(
                translation,
                rotation,
                rospy.Time.now(),
                "base_link",
                "map",
            )

    def run(self):
        while not rospy.is_shutdown():
            self.broadcast_transform()
            self.rate.sleep()


if __name__ == "__main__":
    try:
        UTMTOROVER()
    except rospy.ROSInterruptException as e:
        rospy.logerr("Service did not process request: " + str(e))
        pass
