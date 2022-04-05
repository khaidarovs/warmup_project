#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3

distance = 4
turns = 4

class DriveSquare(object):

    def __init__(self):
        rospy.init_node('drive_square')
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):
        current_turn = 0
        # my_linear = Vector3(0.5, 0, 0)
        # my_angular = Vector3(0, 0, 0)

        while current_turn <= turns:
            current_distance = 0
            while current_distance <= distance:
                my_twist = Twist(linear=Vector3(0.2, 0, 0), angular=Vector3())
                rospy.sleep(1)
                self.robot_movement_pub.publish(my_twist)
                current_distance += 1

            my_twist = Twist(linear=Vector3(), angular=Vector3(0, 0, 0.7854)) #we need angular velocity * time to be equal to 1.5708, which is the number of radians in a 90 degree turn
            self.robot_movement_pub.publish(my_twist)
            rospy.sleep(2.2) #made it 2.2 because 2 wasn't enough for a roughly 90 degree turn
            my_twist = Twist(linear=Vector3(), angular=Vector3(0, 0, 0))
            self.robot_movement_pub.publish(my_twist)
            # rospy.sleep(1)
            current_turn += 1
        
        my_twist = Twist(linear=Vector3(), angular=Vector3())
        self.robot_movement_pub.publish(my_twist)

if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()