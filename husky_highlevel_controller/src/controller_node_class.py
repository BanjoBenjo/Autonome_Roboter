#!/usr/bin/env python3
import rospy
from husky_highlevel_controller import util
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import numpy as np


class Controller():
    def __init__(self):
        # load parameter
        laser_queue_size = rospy.get_param("/queue_size")
        laser_topic_name = rospy.get_param("/topic_name")

        # Create Subscirber
        rospy.Subscriber(laser_topic_name, LaserScan, callback=self.laser_callback, queue_size=laser_queue_size)

        # Create publisher
        self.scan_publisher = rospy.Publisher("scan_filtered", LaserScan,queue_size=1)
        self.cmd_vel_publisher = rospy.Publisher("cmd_vel", LaserScan,queue_size=1)
        
    def laser_callback(self, data):
        filtered_msg = util.filter_scan(data)
        self.scan_publisher.publish(filtered_msg)

        angle_min = filtered_msg.angle_min
        angle_increment = filtered_msg.angle_increment
        range = filtered_msg.ranges[2]
        angle = angle_min + 2 * angle_increment

        

        if np.abs(angle) > 1:
            self.send_drive_cmd(angle=np.sign(angle) * 0.3)
        elif range > 0.05:
            self.send_drive_cmd(speed=0.2)
        else:
            self.send_drive_cmd(speed=0)

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
    controller = Controller()

    # Eport Env Variable so LaserScan is available
    # export HUSKY_LMS1XX_ENABLED=1
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()