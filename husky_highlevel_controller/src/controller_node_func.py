#!/usr/bin/env python3
import rospy
from husky_highlevel_controller import util
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import numpy as np


def get_vector3(x=0, y=0, z=0):
    vector = Vector3()
    vector.x = float(x)
    vector.y = float(y)
    vector.z = float(z)

    return vector


def send_drive_cmd(speed=0, angle=0):
    print("In send Drive")
    twist_msg = Twist()
    twist_msg.linear = get_vector3(x=speed)
    twist_msg.angular = get_vector3(z=angle)

    cmd_vel_publisher.publish(twist_msg)


def laser_callback(data):
    filtered_msg = util.filter_scan(data)
    scan_publisher.publish(filtered_msg)

    # print("In Laser Callback")

    # angle_min = filtered_msg.angle_min
    # angle_increment = filtered_msg.angle_increment
    # range = filtered_msg.ranges[2]
    # angle = angle_min + 2 * angle_increment

    # if np.abs(angle) > 1:
    #     send_drive_cmd(angle=np.sign(angle) * 0.3)
    # elif range > 0.05:
    #     send_drive_cmd(speed=0.2)
    # else:
    #     send_drive_cmd(speed=0)


if __name__ == '__main__':
    rospy.init_node('husky_highlevel_controller')
    laser_queue_size = rospy.get_param("/queue_size")
    laser_topic_name = rospy.get_param("/topic_name")

    # Create Subscirber
    rospy.Subscriber(laser_topic_name, LaserScan, callback=laser_callback, queue_size=laser_queue_size)

    # Create publisher
    scan_publisher = rospy.Publisher("scan_filtered", LaserScan,queue_size=1)
    cmd_vel_publisher = rospy.Publisher("cmd_vel", LaserScan,queue_size=1)

    # Eport Env Variable so LaserScan is available
    # export HUSKY_LMS1XX_ENABLED=1
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()