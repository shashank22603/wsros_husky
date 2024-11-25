#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

class HuskyMover:
    def __init__(self, husky_ns):
        self.husky_ns = husky_ns
        self.pub = rospy.Publisher(f'/{self.husky_ns}/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.pattern_duration = 10  # Duration for each pattern in seconds
        self.current_pattern = 0

    def move_husky(self):
        start_time = time.time()
        
        while not rospy.is_shutdown():
            elapsed_time = time.time() - start_time

            # Change patterns every pattern_duration seconds
            if elapsed_time > self.pattern_duration:
                self.current_pattern = (self.current_pattern + 1) % 2
                start_time = time.time()  # Reset timer
            
            move_cmd = Twist()

            # Define patterns
            if self.current_pattern == 0:
                # Pattern 1: Square formation
                if self.husky_ns == "husky_1":
                    move_cmd.linear.x = 0.1  # Move forward
                elif self.husky_ns == "husky_2":
                    move_cmd.linear.x = 0.1
                elif self.husky_ns == "husky_3":
                    move_cmd.linear.x = 0.1
                elif self.husky_ns == "husky_4":
                    move_cmd.linear.x = 0.1
            else:
                # Pattern 2: Rhombus formation
                if self.husky_ns == "husky_1":
                    move_cmd.linear.x = 0.5  # Move forward
                    move_cmd.angular.z = 0.3  # Turn slightly
                elif self.husky_ns == "husky_2":
                    move_cmd.linear.x = 0.5
                    move_cmd.angular.z = -0.3  # Turn slightly
                elif self.husky_ns == "husky_3":
                    move_cmd.linear.x = 0.5
                    move_cmd.angular.z = 0.3
                elif self.husky_ns == "husky_4":
                    move_cmd.linear.x = 0.5
                    move_cmd.angular.z = -0.3

            self.pub.publish(move_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('husky_mover', anonymous=True)
        husky_ns = rospy.get_param("~husky_ns")  # Get the namespace parameter
        mover = HuskyMover(husky_ns)
        mover.move_husky()
    except rospy.ROSInterruptException:
        pass
