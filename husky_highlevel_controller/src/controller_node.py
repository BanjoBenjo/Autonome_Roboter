#!/usr/bin/env python3
import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from husky_highlevel_controller import util
from husky_highlevel_controller.msg import Target

import math

class Controller():
    def __init__(self):
        # load parameter
        laser_queue_size = rospy.get_param("/queue_size")
        laser_topic_name = rospy.get_param("/topic_name")
        target_queue_size = rospy.get_param("/target_queue_size")
        target_topic_name = rospy.get_param("/target_topic_name")

        # Create Subscriber
        rospy.Subscriber(laser_topic_name, LaserScan, callback=self.laser_callback, queue_size=laser_queue_size)
      
        # Create publisher
        self.scan_publisher = rospy.Publisher("scan_filtered", LaserScan, queue_size=1)
        self.target_publisher = rospy.Publisher(target_topic_name, Target, queue_size=target_queue_size)

        # tf2 buffer and listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfBuffer)


    def laser_callback(self, data):
        filtered_msg = util.filter_scan(data)
        self.scan_publisher.publish(filtered_msg)

        # return when empty scan 
        if not len(filtered_msg.ranges):
            return

        angle_min = filtered_msg.angle_min
        angle_increment = filtered_msg.angle_increment

        mid_index = math.floor(len(filtered_msg.ranges)/2)
        range = filtered_msg.ranges[mid_index]
        angle_rad = angle_min + angle_increment * mid_index

        self.publish_target(range, angle_rad)


    def publish_target(self, range, angle):
        target_msg = Target()
        target_msg.header.frame_id = "base_laser"
        target_msg.range = range
        target_msg.angle = angle
        self.target_publisher.publish(target_msg)


if __name__ == '__main__':
    rospy.init_node('husky_highlevel_controller')
    controller = Controller()

    # Eport Env Variable so LaserScan is available
    # export HUSKY_LMS1XX_ENABLED=1
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
