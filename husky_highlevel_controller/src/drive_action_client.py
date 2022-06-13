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
        self.client.wait_for_server()

        # Fill in the goal here
        goal = DriveGoal()
        goal.start = True

        self.client.send_goal(goal)
        rospy.sleep(5)
        self.client.cancel_goal()

    def fb_callback(self):
        print("Feedback recieved")

if __name__ == '__main__':
    rospy.init_node('drive_action_client')
    controller = Drive()

    # Eport Env Variable so LaserScan is available
    # export HUSKY_LMS1XX_ENABLED=1
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
