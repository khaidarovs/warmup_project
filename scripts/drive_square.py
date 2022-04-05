#!/usr/bin/env python3
import rospy
# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

# x-axis distance and number of turns the turtlebot should 
# travel and make respectively
distance = 4
turns = 4

class DriveSquare(object):
    """ This node publishes Twist messages containing linear and angular velocities"""

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square')
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):
        current_turn = 0
        while current_turn <= turns:
            current_distance = 0
            while current_distance <= distance:
                # setup the Twist message we want to send for turtlebot to move forward
                my_twist = Twist(linear=Vector3(0.2, 0, 0), angular=Vector3())
                rospy.sleep(1) #sleep 1 second before sending the message again
                self.robot_movement_pub.publish(my_twist)
                current_distance += 1

            # Once the turtlebot moved forward, stop and make a left turn. Rationale for using 0.7854
            # as angular z-velocity is explained in README file
            my_twist = Twist(linear=Vector3(), angular=Vector3(0, 0, 0.7854)) 
            self.robot_movement_pub.publish(my_twist)
            # Turning time. Determined that 2.2 was better than 2 through trial and error
            rospy.sleep(2.2) 
            my_twist = Twist(linear=Vector3(), angular=Vector3())
            self.robot_movement_pub.publish(my_twist)
            current_turn += 1
        
        # When turtlebot drove 1 square, stop the turtlebot
        my_twist = Twist(linear=Vector3(), angular=Vector3())
        self.robot_movement_pub.publish(my_twist)

if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()