#!/usr/bin/env python3
import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from tf2_geometry_msgs import PointStamped
from visualization_msgs.msg import Marker
from husky_highlevel_controller.msg import Target

import numpy as np
import math

class Drive():
    def __init__(self):
        # load parameter
        target_queue_size = rospy.get_param("/target_queue_size")
        target_topic_name = rospy.get_param("/target_topic_name")

        # Create Subscriber
        rospy.Subscriber(target_topic_name, Target, callback=self.target_callback, queue_size=target_queue_size)
      
        # Create publisher
        self.cmd_vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.marker_publisher = rospy.Publisher("visualization_marker", Marker, queue_size=1)

        self.current_angle = 0
        self.current_position = None

        # tf2 buffer and listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfBuffer)


    def target_callback(self, target_msg):

        range = target_msg.range
        angle = target_msg.angle

        # convert radian angle to degree
        angle_deg = self.rad_to_deg(angle)

        # calculate x,y coordinates of nearest points (laser_frame)
        laser_x, laser_y = self.polar_to_karth(range, angle)
        # transform coordinates from laser frame to odom
        odom_x, odom_y = self.tf_laser_to_odom(laser_x, laser_y)

        if odom_x and odom_y :
            self.publish_marker(odom_x, odom_y, "odom")

        # orient and drive Robo to nearest point
        if abs(angle_deg) > 5:
            self.send_drive_cmd(angle= -np.sign(angle_deg) * 0.2)
        elif range > 0.5:
            self.send_drive_cmd(speed=0.5)
        else:
            self.send_drive_cmd(speed=0)

    def rad_to_deg(self, rad):
        return rad * 180 / np.pi

    def polar_to_karth(self, r, phi):
        x = r * np.cos(phi)
        y = r * np.sin(phi)
        return x, y

    def publish_marker(self, x, y, frame):
        marker = Marker()
        marker.header.frame_id = frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0
        marker.color.b = 0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        self.marker_publisher.publish(marker)

    def tf_laser_to_odom(self, laser_x, laser_y):
        laser_point = PointStamped()
        laser_point.header.frame_id = "base_laser"
        laser_point.header.stamp = rospy.Time(0)
        laser_point.point.x= laser_x
        laser_point.point.y= laser_y
        laser_point.point.z= 0.0

        try:
            p = self.tfBuffer.transform(laser_point, "odom", timeout=rospy.Duration(10.0))
            return p.point.x, p.point.y

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None, None

    def get_vector3(self, x=0, y=0, z=0):
        vector = Vector3()
        vector.x = float(x)
        vector.y = float(y)
        vector.z = float(z)

        return vector

    def send_drive_cmd(self, speed=0, angle=0):
        twist_msg = Twist()
        twist_msg.linear = self.get_vector3(x=speed)
        twist_msg.angular = self.get_vector3(z=angle)

        self.cmd_vel_publisher.publish(twist_msg)


if __name__ == '__main__':
    rospy.init_node('husky_highlevel_controller')
    controller = Drive()

    # Eport Env Variable so LaserScan is available
    # export HUSKY_LMS1XX_ENABLED=1
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
