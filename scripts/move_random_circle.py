#!/usr/bin/env python

""" move_random.py

    Move to random goals using move_base     
"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import radians, pi, sqrt, sin, cos, atan2
import random


class MoveRandom():
    def __init__(self):
        rospy.init_node('move_random', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        # Size of the area in which to move?
        self.x_min = rospy.get_param("~x_min", -9.0)  # meters
        self.x_max = rospy.get_param("~x_max", 9.0)  # meters
        self.y_min = rospy.get_param("~y_min", -6.0)  # meters
        self.y_max = rospy.get_param("~y_max", 6.0)  # meters
        self.radius = rospy.get_param("~radius", 1.0)  # meters

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")

        rospy.loginfo("Start pursuing random goals")
        while not rospy.is_shutdown():
            # generate random goal
            goal = self.generate_goal()

            # and move to it
            self.move_to(goal)

            # then move on circular path around goal
            x = goal.target_pose.pose.position.x
            y = goal.target_pose.pose.position.y
            for step in range(36):
                angle = radians(step * 10)
                dx, dy = pol2cart(self.radius, angle)
                goal.target_pose.pose.position.x = x + dx
                goal.target_pose.pose.position.y = y + dy
                self.move_to(goal)

            # wait some time before pursuing next goal
            #rospy.sleep(random.uniform(0, 2))

    def generate_goal(self):
        ''' Generate random goal for move_base
        '''

        # Generate random pose components
        x = random.uniform(self.x_min, self.x_max)
        y = random.uniform(self.y_min, self.y_max)
        yaw = random.uniform(-pi, pi)
        q = quaternion_from_euler(0.0, 0.0, yaw)

        # Create goal from components
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = '/map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(x, y, 0.0)
        goal.target_pose.pose.orientation = Quaternion(*q)

        return goal

    def move_to(self, goal):
        ''' Move to goal using move_base
        '''
        self.move_base.send_goal(goal)
        # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(
            rospy.Duration(10))

        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Succeeded")
            else:
                rospy.loginfo("Failed")

    def shutdown(self):
        rospy.loginfo("Shutdown...")
        self.move_base.cancel_goal()
        rospy.sleep(1)


def cart2pol(x, y):
    """ Carthesian to Polar coordinates
    """
    rho = sqrt(x**2 + y**2)
    phi = atan2(y, x)
    return (rho, phi)


def pol2cart(rho, phi):
    """ Polar to Carthesian coordinates
    """
    x = rho * cos(phi)
    y = rho * sin(phi)
    return (x, y)


if __name__ == '__main__':
    try:
        MoveRandom()
    except rospy.ROSInterruptException:
        rospy.loginfo("Stopped")
