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

# How close we will get to wall and error range to operate within
distance = 0.4
error = 0.1

class FollowWall(object):
    """ This node publishes Twist messages containing linear and angular velocities"""

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('follow_wall')

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
        # Determine closeness to and position of wall by looking at scan data, 
        #   set angular velocity based on that information, and
        #   publish to cmd_vel.

        non_zero_list = []
        non_zero_angle = []
        for angle, dist in enumerate(data.ranges): #find non-zero distances in ranges
            if dist != 0:
                non_zero_list.append(dist)
                non_zero_angle.append(angle)
        min_distance = min(non_zero_list)
        angle = non_zero_angle[non_zero_list.index(min(non_zero_list))] #find the min distance to wall and direction
        print("minimum distance is", min_distance)
        if min_distance > (distance + error): #need to find a wall
            print("Looking for wall")
            if angle < 180:
                self.twist.angular.z = 0.08 * angle
            else:
                self.twist.angular.z = 0.08 * (angle - 360) #use similar logic as in person_follower
        else: #got close enough to the wall to start following
            print("Near the wall")
            # when next to the wall, if turtlebot is tilted towards the wall, make a right turn, and vice versa
            if angle < 90: 
                self.twist.angular.z = 0.008 * (angle - 90) 
            elif 180 > angle > 90:
                self.twist.angular.z = 0.008 * angle
        self.twist.linear.x = 0.05 
        self.robot_movement_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = FollowWall()
    node.run()