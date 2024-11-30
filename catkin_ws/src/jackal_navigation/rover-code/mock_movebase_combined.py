#!/usr/bin/env python3

import roslib
import rospy
import math

# import tf
import geometry_msgs.msg
from time import time
from geometry_msgs.msg import TransformStamped, Twist
from std_srvs.srv import Trigger, TriggerResponse
from waypoint_server.msg import Waypoint, WaypointArray
import tf2_ros
import pyzed.sl as sl


class RoverNavigationController:
    def __init__(self, rate: float):
        """Initializes RoverNavigationController.

        Args:
            rate: Rate (in hz) to operate the navigation loop

        """
        self.rate = rate

        # Current Goal
        self.have_reached_goal = False
        self.found_time = None
        self.markers = []
        self.goal_frame = "goal"
        self.waypoint_index = -1
        self.state = "STARTUP"

        # Services
        # Integrates with the waypoint server's waypoint/done service. This should be called upon reaching the final
        # goal of a waypoint (either the goal itself or the markers)
        rospy.wait_for_service("/waypoints/done")
        self._service_waypoint_done = rospy.ServiceProxy("/waypoints/done", Trigger)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_wait_time = rospy.Duration(
            0.3
        )  # How long to wait when looking up transforms before giving up

        # Subscribers
        rospy.Subscriber("/waypoints/current", Waypoint, self.current_waypoint_callback)

        # Publishers
        self.drive_publisher = rospy.Publisher(
            "/drive/cmd_vel/auto", geometry_msgs.msg.Twist, queue_size=1
        )
        self.previous_cmd_vel = Twist()
        self.beta = 0.2

    def waypoint_done(self):
        """Method to call when waypoint has been completed."""

        try:
            response: TriggerResponse = self._service_waypoint_done()
            if not response.success:
                rospy.logwarn(
                    f"/waypoints/done indciated no more waypoints: {response.message}"
                )
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            return False

        rospy.loginfo("Reached goal successfully!")
        self.have_reached_goal = True
        self.found_time = time()
        return True

    def current_waypoint_callback(self, msg: Waypoint):
        """Update the current waypoint based on data from the waypoint server.

        If the callback sends the same goal, ignore it.

        Args:
            msg (Waypoint): Contains the name of the goal frame and the relevant markers to navigate to
        """
        # TODO: Check for name of waypoint equality
        if msg.markers != self.markers:
            self.goal_frame = "wp_" + msg.name
            rospy.logerr(f"New goal! {self.markers} -> {msg.markers}")
            self.markers = msg.markers
            self.have_reached_goal = False
            self.state = "STARTUP"

    # TODO: Implement method
    def obstacle_detected(self) -> bool:
        return False

    def pause(self, start, elapsed: float):
        while time() - start <= elapsed:
            cmd_vel = Twist()
            self.drive_publisher.publish(cmd_vel)

    def run(self):
        # initialize parameters for state machine
        rate = rospy.Rate(self.rate)
        goal_transform = None  # the goal to go to

        aruco_move = False  # whether we are moving towards an aruco marker or not
        arucos_found = 0  # how many arucos we have found for the goal
        one_gate_post = False  # boolean value describing if only one of the gate posts has been found

        spinning = False  # whether we are spinning or not

        spiraling = False  # are we spiraling?
        spin_time = 15  # the number of seconds we want the circle spinning to take
        rotate_count = 0  # number of times we have started rotating

        max_speed = 1  # max speed that we clamp to, changes dymanically

        waiting = False  # whether we are waiting for next waypoint or not

        prev_state = self.state

        while not rospy.is_shutdown():
            # Looksup Transforms
            now = rospy.Time(0)

            # View state changes
            if prev_state != self.state:
                print(f"State changed from {prev_state} -> {self.state}")
                prev_state = self.state

            # describes the state where we startup the navigation process
            if self.state == "STARTUP":
                waiting = False
                if self.found_time is not None:
                    if time() - self.found_time < 5:
                        rospy.logerr_throttle(1, "Waiting for 5 seconds...")
                        self.drive_publisher.publish(Twist())
                        self.state = "STARTUP"
                        rate.sleep()
                        continue
                    else:
                        rospy.logerr("Continuing navigation!")
                        self.found_time = None

                        # In startup we reset all the flags so that after we find an aruco and wait 5 seconds we are fresh and ready for the next one ECKSDEE XD
                        goal_transform = None  # the goal to go to
                        aruco_move = False  # whether we are moving towards an aruco marker or not
                        arucos_found = 0  # how many arucos we have found for the goal
                        one_gate_post = False  # boolean value describing if only one of the gate posts has been found
                        spinning = False  # whether we are spinning or not
                        spiraling = False  # are we spiraling?
                        rotate_count = 0  # number of times we have started rotating
                        max_speed = 1  # max speed that we clamp to, changes dymanically
                        waiting = (
                            False  # whether we are waiting for next waypoint or not
                        )

                        max_speed = 1
                        self.state = "NAV_TO_GOAL"
                        rate.sleep()
                        continue
                else:
                    self.state = "NAV_TO_GOAL"

            # describes the state where we find how to move to goal
            elif self.state == "NAV_TO_GOAL":
                # Describes whether or not the goal we are headed towards is our "final" destination
                # - No Markers -> name is final location, so always true
                # - 1 Marker   -> name is NOT final location, so only true if we see the marker
                # - 2 Markers  -> name is NOT final location, nor are individual markers, so only true if we see the gate

                aruco_move = False
                one_gate_post = False
                rospy.loginfo(len(self.markers))

                if len(self.markers) == 1:  # If we are going to a post
                    try:
                        goal_transform = self.tf_buffer.lookup_transform(
                            "base_link",
                            f"fiducial_{self.markers[0]}",
                            now,
                            self.tf_wait_time,
                        )
                        self.state = "NAV_TO_ARUCO"
                        continue
                    except (
                        tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException,
                    ):
                        rospy.logwarn_throttle(
                            1,
                            f"(Post) Failed to lookup transform from 'base_link' -> 'fiducial_{self.markers[0]}'...",
                        )
                elif len(self.markers) == 2:  # If we are going to a gate

                    # case 1: we see both fiducials
                    try:
                        goal_transform = self.tf_buffer.lookup_transform(
                            "base_link",
                            f"fiducial_gate_{self.markers[0]}_{self.markers[1]}",
                            now,
                            self.tf_wait_time,
                        )
                        one_gate_post = False
                        self.state = "NAV_TO_ARUCO"
                        continue
                    except (
                        tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException,
                    ):
                        rospy.logwarn_throttle(
                            1,
                            f"(Gate) Failed to lookup transform from 'base_link' -> 'fiducial_gate_{self.markers[0]}_{self.markers[1]}'...",
                        )

                    # case 2: we see left fiducial
                    try:
                        # TODO: Add case where only one post of a gate is found and rover tries to find gate tranforms (aka looping around post?)
                        goal_transform = self.tf_buffer.lookup_transform(
                            "base_link",
                            f"fiducial_{self.markers[0]}",
                            now,
                            self.tf_wait_time,
                        )
                        one_gate_post = True
                        self.state = "NAV_TO_ARUCO"
                        continue
                    except (
                        tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException,
                    ):
                        rospy.logwarn_throttle(
                            1,
                            f"(Gate) Failed to lookup transform from 'base_link' -> 'fiducial_{self.markers[0]}'...",
                        )

                    # case 3: we try right fiducial
                    try:
                        # TODO: Add case where only one post of a gate is found and rover tries to find gate tranforms (aka looping around post?)
                        goal_transform = self.tf_buffer.lookup_transform(
                            "base_link",
                            f"fiducial_{self.markers[1]}",
                            now,
                            self.tf_wait_time,
                        )
                        one_gate_post = True
                        self.state = "NAV_TO_ARUCO"
                        continue
                    except (
                        tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException,
                    ):
                        rospy.logwarn_throttle(
                            1,
                            f"(Gate) Failed to lookup transform from 'base_link' -> 'fiducial_{self.markers[1]}'...",
                        )

                # else we are going to goal
                if goal_transform is None:
                    try:
                        goal_transform = self.tf_buffer.lookup_transform(
                            "base_link", self.goal_frame, now, rospy.Duration(5)
                        )
                    except (
                        tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException,
                    ):
                        rospy.logwarn_throttle(
                            1,
                            f"(Goal) Failed to lookup transform from 'base_link' -> '{self.goal_frame}'...",
                        )
                        rate.sleep()

                        self.state = "STARTUP"
                        continue

                if spinning:
                    self.state = "SPIN_ROVER"
                elif spiraling:
                    self.state = "SPIRAL_ROVER"
                elif waiting:
                    self.state = "WAITING_FOR_GOAL"
                else:
                    self.state = "MOVE_ROVER"

            # describes the state where we find how to move to aruco
            elif self.state == "NAV_TO_ARUCO":
                max_speed = 0.5
                aruco_move = True
                spinning = True if one_gate_post else False
                if spinning:
                    self.state = "SPIN_ROVER"
                else:
                    self.state = "MOVE_ROVER"

            # moving the rover to goal
            elif self.state == "MOVE_ROVER":
                goal_transform: TransformStamped = goal_transform

                cmd_vel = Twist()
                distance = math.sqrt(
                    goal_transform.transform.translation.x**2
                    + goal_transform.transform.translation.y**2
                )
                if distance > 2.5:
                    angular = 4 * math.atan2(
                        goal_transform.transform.translation.y,
                        goal_transform.transform.translation.x,
                    )
                    linear = min(1.0, 0.5 * distance)
                    cmd_vel.linear.x = linear
                    cmd_vel.angular.z = angular
                else:  # Close enough to goal
                    if not aruco_move:
                        self.state = "FOUND_GOAL"
                    else:
                        self.state = "FOUND_ARUCO"
                    continue

                rospy.loginfo(f"Distance: {distance}")

                # Avoid Obstacles
                if self.obstacle_detected():
                    state = "AVOID_OBSTACLE"
                    continue
                rospy.loginfo(self.goal_frame)

                # If we didnt find goal this run then we move towards goal!
                # Smooth Velocity
                cmd_vel.linear.x = (
                    cmd_vel.linear.x * self.beta
                    + self.previous_cmd_vel.linear.x * (1 - self.beta)
                )
                cmd_vel.linear.x = (
                    0 if abs(cmd_vel.linear.x) < 0.01 else cmd_vel.linear.x
                )
                cmd_vel.angular.z = (
                    cmd_vel.angular.z * self.beta
                    + self.previous_cmd_vel.angular.z * (1 - self.beta)
                )
                cmd_vel.angular.z = (
                    0 if abs(cmd_vel.angular.z) < 0.01 else cmd_vel.angular.z
                )

                # Send Velocity Command
                cmd_vel.linear.x = min(max_speed, max(-max_speed, cmd_vel.linear.x))
                cmd_vel.angular.z = -min(1.0, max(-1.0, cmd_vel.angular.z))

                self.drive_publisher.publish(cmd_vel)
                self.previous_cmd_vel = cmd_vel

                rate.sleep()
                rospy.loginfo(f"Distance: {distance}")
                goal_transform = None
                self.state = "NAV_TO_GOAL"

            # spinning the rover to find more arucos
            elif self.state == "SPIN_ROVER":
                # we perform the command to spin
                # then test to see if any new waypoints are found
                cmd_vel = Twist()

                # arbitrary values to spin the rover in a circle
                cmd_vel.linear.x = 0.2
                # we want to complete a circle in 15 seconds (must travel 24 degrees per rate (10) cycles ( 1 second ))
                cmd_vel.angular.z = (math.pi * (360 / spin_time)) / (180)

                # Smooth Velocity
                # cmd_vel.linear.x = cmd_vel.linear.x * self.beta + self.previous_cmd_vel.linear.x * (1 - self.beta)
                # cmd_vel.linear.x = 0 if abs(cmd_vel.linear.x) < 0.01 else cmd_vel.linear.x
                # cmd_vel.angular.z = cmd_vel.angular.z * self.beta + self.previous_cmd_vel.angular.z * (1 - self.beta)
                # cmd_vel.angular.z = 0 if abs(cmd_vel.angular.z) < 0.01 else cmd_vel.angular.z

                # Send Velocity Command
                cmd_vel.linear.x = min(1.0, max(-1.0, cmd_vel.linear.x))
                cmd_vel.angular.z = -min(1.0, max(-1.0, cmd_vel.angular.z))

                self.drive_publisher.publish(cmd_vel)
                self.previous_cmd_vel = cmd_vel
                rate.sleep()

                # keep track of how long we have been spinning
                rotate_count += 1
                print(f"rotate count: {rotate_count}")

                # Moves in a circle for 15 seconds (spin_time * self.rate)
                if rotate_count >= spin_time:
                    # when we are done with spinning in circle we try spiraling
                    rotate_count = 0
                    spinning = False
                    goal_transform = None
                    self.state = "SPIRAL_ROVER"
                else:
                    # continue spinning
                    spinning = True
                    goal_transform = None
                    self.state = "NAV_TO_GOAL"

            # spiral the rover looking for arucos
            # if not found then we enter waiting for goal until the next waypoint is incremented manually
            elif self.state == "SPIRAL_ROVER":
                # we perform the command to spiral
                # then test to see if any new waypoints are found
                cmd_vel = Twist()

                # change the linear speed as we move along makes it smooth
                # not needed to be changing we can immediately set to 1, 1.5 as long as its enough for the angle
                cmd_vel.linear.x = min(1.5, 0.5 + (rotate_count / 15))

                one_degree_per_second = math.pi / (180)
                needed_deg_for_circle = 360 / spin_time
                deg_in_rotation = one_degree_per_second * needed_deg_for_circle

                # if this code does not work remove the times 10 at the end (150 sec spiral)
                angular_vel_spin_spiral = max(
                    deg_in_rotation
                    - ((rotate_count * deg_in_rotation) / (spin_time * self.rate * 4)),
                    0,
                )
                cmd_vel.angular.z = angular_vel_spin_spiral

                # Smooth Velocity
                # cmd_vel.linear.x = cmd_vel.linear.x * self.beta + self.previous_cmd_vel.linear.x * (1 - self.beta)
                # cmd_vel.linear.x = 0 if abs(cmd_vel.linear.x) < 0.01 else cmd_vel.linear.x
                # cmd_vel.angular.z = cmd_vel.angular.z * self.beta + self.previous_cmd_vel.angular.z * (1 - self.beta)
                # cmd_vel.angular.z = 0 if abs(cmd_vel.angular.z) < 0.01 else cmd_vel.angular.z

                # Send Velocity Command
                # Note: max speed change to 1
                cmd_vel.linear.x = min(1, max(-1, cmd_vel.linear.x))
                cmd_vel.angular.z = -min(1, max(-1, cmd_vel.angular.z))
                self.drive_publisher.publish(cmd_vel)
                self.previous_cmd_vel = cmd_vel
                rotate_count += 1
                rate.sleep()

                if angular_vel_spin_spiral == 0:
                    # we are done with the spiral so we just wait until next waypoint sent
                    rotate_count = 0
                    spiraling = False
                    goal_transform = None
                    self.state = "WAITING_FOR_GOAL"
                else:
                    spiraling = True
                    goal_transform = None
                    self.state = "NAV_TO_GOAL"

            # TODO: Make waiting for goal option (publish empty cmd_vels until next goal is moved to manually)
            # making the waypoint server skip going to this waypoint! (talk)
            # will be buggy
            elif self.state == "WAITING_FOR_GOAL":
                rospy.loginfo("Unable to reach current waypoint, aborting!")
                self.waypoint_done()
                arucos_found = 0

                # # Send an zero'd cmd_vel to signify abort sequence?
                # cmd_vel = Twist()

                # cmd_vel.linear.x = 0
                # cmd_vel.angular.z = 0

                # # Send Velocity Command
                # self.drive_publisher.publish(cmd_vel)
                # self.previous_cmd_vel = cmd_vel
                # rate.sleep()
                # waiting = True

                # self.state is not here because we will get startup when callback is called

            # TODO: obstacle avoidance
            elif self.state == "AVOID_OBSTACLE":
                pass

            # goal post found
            elif self.state == "FOUND_GOAL":
                if arucos_found == len(self.markers):
                    rospy.loginfo("Sending waypoint done service call!")
                    self.waypoint_done()
                    arucos_found = 0
                    self.state = "STARTUP"
                else:
                    self.state = "SPIN_ROVER"

            # aruco marker found
            elif self.state == "FOUND_ARUCO":
                arucos_found += 1
                if arucos_found == len(self.markers):
                    self.waypoint_done()
                    arucos_found = 0
                    self.state = "STARTUP"
                else:
                    self.state = "NAV_TO_GOAL"

            else:
                raise Exception("State not supported")


if __name__ == "__main__":
    rospy.init_node("rover_navigation")
    rospy.loginfo("Starting navigation")

    controller = RoverNavigationController(rate=10)
    controller.run()
