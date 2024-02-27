#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt


class NavTest():
    def __init__(self):
        rospy.init_node('exploring_slam', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # Time to pause at each target position (unit: s)
        self.rest_time = rospy.get_param("~rest_time", 2)

        # Is it simulated?
        self.fake_test = rospy.get_param("~fake_test", True)

        # reach the state of the target
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']

        # Set the position of the target point
        # Click the 2D Nav Goal button in rviz, then click a point on the map
        # You will see the coordinate information of the point in the terminal
        locations = dict()

        locations['1'] = Pose(Point(4.589, -0.376, 0.000),
                              Quaternion(0.000, 0.000, -0.447, 0.894))
        locations['2'] = Pose(Point(4.231, -6.050, 0.000),
                              Quaternion(0.000, 0.000, -0.847, 0.532))
        locations['3'] = Pose(Point(-0.674, -5.244, 0.000),
                              Quaternion(0.000, 0.000, 0.000, 1.000))
        locations['4'] = Pose(Point(-5.543, -4.779, 0.000),
                              Quaternion(0.000, 0.000, 0.645, 0.764))
        locations['5'] = Pose(Point(-4.701, -0.590, 0.000),
                              Quaternion(0.000, 0.000, 0.340, 0.940))
        locations['6'] = Pose(Point(2.924, 0.018, 0.000),
                              Quaternion(0.000, 0.000, 0.000, 1.000))

        # Post a message to control the robot
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the message of the move_base server
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # 60s waiting time limit
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")

        # Save the robot's initial position in rviz
        initial_pose = PoseWithCovarianceStamped()

        # Save variables for success rate, running time, and distance
        n_locations = len(locations)
        n_goals = 0
        n_successes = 0
        i = n_locations
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        location = ""
        last_location = ""

        # Make sure there is an initial position
        while initial_pose.header.stamp == "":
            rospy.sleep(1)

        rospy.loginfo("Starting navigation test")

        # Start main loop, random navigation
        while not rospy.is_shutdown():
            # If all points have been gone, start sorting again
            if i == n_locations:
                i = 0
                sequence = sample(list(locations), n_locations)

                # If the last point is the same as the first point, skip
                if sequence[0] == last_location:
                    i = 1

            # Get the next target point in the current sort
            location = sequence[i]

            # Track driving distance
            # use the updated initial position
            if initial_pose.header.stamp == "":
                distance = sqrt(pow(locations[location].position.x -
                                    locations[last_location].position.x, 2) +
                                pow(locations[location].position.y -
                                    locations[last_location].position.y, 2))
            else:
                rospy.loginfo("Updating current pose.")
                distance = sqrt(pow(locations[location].position.x -
                                    initial_pose.pose.pose.position.x, 2) +
                                pow(locations[location].position.y -
                                    initial_pose.pose.pose.position.y, 2))
                initial_pose.header.stamp = ""

            # Store the last position and calculate the distance
            last_location = location

            # add 1 to the counter
            i += 1
            n_goals += 1

            # Set the next target point
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = locations[location]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            # Let the user know the next position
            rospy.loginfo("Going to: " + str(location))

            # Go to the next position
            self.move_base.send_goal(self.goal)

            # Five minute time limit
            finished_within_time = self.move_base.wait_for_result(
                rospy.Duration(300))

            # Check to see if it arrived successfully
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    n_successes += 1
                    distance_traveled += distance
                    rospy.loginfo("State:" + str(state))
                else:
                    rospy.loginfo(
                        "Goal failed with error code: " + str(goal_states[state]))

            # run time
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0

            # Output all the information of this navigation
            rospy.loginfo("Success so far: " + str(n_successes) + "/" +
                          str(n_goals) + " = " +
                          str(100 * n_successes/n_goals) + "%")

            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")

            rospy.sleep(self.rest_time)

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


def trunc(f: float, n: int) -> float:
    formatted_num = format(f, f".{n}f")
    return float(formatted_num)


if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exploring SLAM finished.")
