#!/usr/bin/env python3
import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from tf2_geometry_msgs import PointStamped
from visualization_msgs.msg import Marker
from husky_highlevel_controller.msg import DriveAction, DriveActionGoal, DriveGoal
import actionlib

import numpy as np
import math

class Drive():
    def __init__(self):
        # Createn Action Client
        self.client = actionlib.SimpleActionClient('drive_to_target', DriveAction)
        rospy.loginfo('Waiting for action server...')
        self.client.wait_for_server()

        rospy.loginfo('Action server started.')

        # Fill in the goal here
        goal = DriveGoal()
        goal.start = True

        self.client.send_goal(goal, done_cb = self.done_cb, active_cb = self.active_cb, feedback_cb = self.feedback_cb)

        finished_before_timeout = self.client.wait_for_result(timeout=rospy.Duration.from_sec(10.0))

        if (finished_before_timeout):
            state = self.client.get_state()
            rospy.loginfo("Action finished: %s", str(state.to_string()))
        else:
            rospy.loginfo('Action timed out.')
            self.client.cancel_goal()

    def done_cb(self, state, result):
        rospy.loginfo("Action Server succeeded with terminal state %d", state)

    def active_cb(self):
        rospy.loginfo("Action Server is active")

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback recieved: Remaining distance %f/%f", feedback.remaining_distance, feedback.initial_distance)


if __name__ == '__main__':
    rospy.init_node('drive_action_client')
    controller = Drive()

    # Eport Env Variable so LaserScan is available
    # export HUSKY_LMS1XX_ENABLED=1
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
