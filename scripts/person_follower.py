#!/usr/bin/env python3

# TOPICS:
#   cmd_vel: publish to, used for setting robot velocity
#   scan   : subscribing, where the wall is

import rospy
import math

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# How close we will get to person and max distance to person.
distance = 0.4
max_dist = 2

class FollowPerson(object):
    """ This node publishes Twist messages containing linear and angular velocities"""

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('follow_person')

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)


    def process_scan(self, data):
        # Determine closeness to and position of person by looking at scan data, 
        #   set angular velocity based on that information, and
        #   publish to cmd_vel.

        # The ranges field is a list of 360 number where each number
        #   corresponds to the distance to the closest obstacle from the
        #   LiDAR at various angles. Each measurement is 1 degree apart.

        # The first entry in the ranges list corresponds with what's directly
        #   in front of the robot.
        angle_deg = 0
        for angle, dist in enumerate(data.ranges): #traverse through the scan results; 
            if max_dist > dist >= distance:
                angle_deg = angle #if person within acceptable range, record the angle
        if angle_deg < 180:
            self.twist.angular.z = 0.08 * angle_deg #if angle<180 turn left
        else:
            self.twist.angular.z = 0.08 * (angle_deg - 360) #else turn right
        if data.ranges[0] >= distance:  
            self.twist.linear.x = 0.2 #if nothing directly in front, move forward
        else:
            self.twist.linear.x = 0 #else stop
        self.robot_movement_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = FollowPerson()
    node.run()