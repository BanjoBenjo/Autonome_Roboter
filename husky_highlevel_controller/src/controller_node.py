#!/usr/bin/env python3
import rospy
from husky_highlevel_controller import util
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from tf2_geometry_msgs import PointStamped
from visualization_msgs.msg import Marker
import tf2_ros

import numpy as np
import math

class Controller():
    def __init__(self):
        # load parameter
        laser_queue_size = rospy.get_param("/queue_size")
        laser_topic_name = rospy.get_param("/topic_name")

        # Create Subscirber
        rospy.Subscriber(laser_topic_name, LaserScan, callback=self.laser_callback, queue_size=laser_queue_size)
      
        # Create publisher
        self.scan_publisher = rospy.Publisher("scan_filtered", LaserScan, queue_size=1)
        self.cmd_vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.marker_publisher = rospy.Publisher("visualization_marker", Marker, queue_size=1)

        self.current_angle = 0
        self.current_position = None

        # tf2 buffer, listener and broadcaster
        self.tfBuffer = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfBuffer)


    def laser_callback(self, data):
        filtered_msg = util.filter_scan(data)
        self.scan_publisher.publish(filtered_msg)

        angle_min = filtered_msg.angle_min
        angle_increment = filtered_msg.angle_increment

        if not len(filtered_msg.ranges):
            return

        mid_index = math.floor(len(filtered_msg.ranges)/2)
        range = filtered_msg.ranges[mid_index]
        angle_rad = angle_min + angle_increment * mid_index
        angle_deg = self.rad_to_deg(angle_rad)

        x, y = self.polar_to_karth(range, angle_rad)

        self.tf_laser_to_odom(x,y)

        #self.publish_marker(x,y, "base_laser")

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
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0
        marker.color.b = 0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        self.marker_publisher.publish(marker)

    def get_vector3(self, x=0, y=0, z=0):
        vector = Vector3()
        vector.x = float(x)
        vector.y = float(y)
        vector.z = float(z)

        return vector

    def tf_laser_to_odom(self, laser_x, laser_y):
        try:
            #trans = self.tfBuffer.lookup_transform('base_laser', 'odom', rospy.Time())

            laser_point = PointStamped()
            laser_point.header.frame_id = "base_laser"
            laser_point.header.stamp = rospy.Time()
            laser_point.point.x= laser_x
            laser_point.point.y= laser_y
            laser_point.point.z= 0.0
            laser_point.header.stamp = rospy.Time(0)

            #p = self.tfBuffer.transform(laser_point, "odom")
            p = self.tfBuffer.transform(laser_point, "odom", timeout=rospy.Duration(10.0))

            self.publish_marker(p.point.x, p.point.y, "odom")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

   

    def send_drive_cmd(self, speed=0, angle=0):
        twist_msg = Twist()
        twist_msg.linear = self.get_vector3(x=speed)
        twist_msg.angular = self.get_vector3(z=angle)

        self.cmd_vel_publisher.publish(twist_msg)


if __name__ == '__main__':
    rospy.init_node('husky_highlevel_controller')
    controller = Controller()

    # Eport Env Variable so LaserScan is available
    # export HUSKY_LMS1XX_ENABLED=1
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()