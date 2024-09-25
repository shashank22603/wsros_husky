#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

def move_turtle(side_length, husky_ns):
    # Initialize the node
    rospy.init_node(f'turtle_mover_{husky_ns}', anonymous=True)
    
    # Publisher to control the Husky's velocity
    vel_pub = rospy.Publisher(f'/{husky_ns}/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    
    # Rate of execution
    rate = rospy.Rate(10)  # 10 Hz

    # Calculate angle for a regular pentagon (108 degrees in radians)
    angle = math.radians(72)

    # Define movement duration based on side length
    move_time = side_length / 2.0  # Arbitrary factor for speed adjustment

    # Move the Husky in a polygon shape
    for i in range(5):  # Change this if you want more or less sides
        # Move forward
        vel_msg = Twist()
        vel_msg.linear.x = 2.0  # Speed of Husky
        vel_msg.angular.z = 0.0  # No rotation

        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < move_time:
            vel_pub.publish(vel_msg)
            rate.sleep()

        # Stop before rotating
        vel_msg.linear.x = 0.0
        vel_pub.publish(vel_msg)

        # Rotate
        vel_msg.angular.z = 1.5  # Speed of rotation
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < (angle / 1.5):
            vel_pub.publish(vel_msg)
            rate.sleep()

        # Stop the rotation
        vel_msg.angular.z = 0.0
        vel_pub.publish(vel_msg)

    # Stop the Husky completely after the movement
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    vel_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        # Example side length; you can adjust this
        side_length = 50
        
        # Move each Husky
        move_turtle(side_length, 'husky_1')
        move_turtle(side_length, 'husky_2')
        move_turtle(side_length, 'husky_3')
        
    except rospy.ROSInterruptException:
        pass
