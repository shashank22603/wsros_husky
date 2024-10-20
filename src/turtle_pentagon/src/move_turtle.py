#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_turtle():
    rospy.init_node('move_turtle')

    # Get the namespace from the parameter server
    husky_ns = rospy.get_param('~husky_ns', 'husky_1')  # Default to 'husky_1' if not set

    # Create a publisher that publishes to the velocity controller of the specified Husky robot
    pub = rospy.Publisher(f'/{husky_ns}/husky_velocity_controller/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    vel_msg = Twist()

    # Set desired motion parameters (example motion)
    vel_msg.linear.x = 0.5
    vel_msg.angular.z = 0.5

    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_turtle()
    except rospy.ROSInterruptException:
        pass

