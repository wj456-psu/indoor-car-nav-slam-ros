#!/usr/bin/env python
import math
import rospy
from math import sqrt
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class DistanceCalculator:
    def __init__(self):
        self.last_pos = None
        self.total_distance = 0.0
        self.x = 0.0  # Initialize x and y here
        self.y = 0.0
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def calculate_distance(self):
        if self.last_pos is not None:
            distance = sqrt((self.x - self.last_pos[0])**2 + (self.y - self.last_pos[1])**2)
            self.total_distance += distance
            rospy.loginfo(f"Traveled distance in this segment: {distance} meters")
        self.last_pos = (self.x, self.y)
        rospy.loginfo(f"Total distance traveled: {self.total_distance} meters")
        
class VelocityCalculator:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.samples = 0
        self.total_linear_velocity = 0.0
        self.avg_linear_velocity = 0.0
        self.max_linear_velocity = 0.0
        self.avg_velocity_pub = rospy.Publisher('/avg_velocity', Float32, queue_size=10)
        self.max_velocity_pub = rospy.Publisher('/max_velocity', Float32, queue_size=10)

    def odom_callback(self, odom_data):
        linear_velocity_x = abs(odom_data.twist.twist.linear.x)
        self.total_linear_velocity += linear_velocity_x
        self.samples += 1
        self.max_linear_velocity = max(self.max_linear_velocity, linear_velocity_x)

    def calculate_avg_velocity(self):
        if self.samples > 0 and self.total_linear_velocity != 0:
            self.avg_linear_velocity = self.total_linear_velocity / self.samples
            rospy.loginfo(f'Average Linear Velocity: {self.avg_linear_velocity} m/s')
            avg_velocity_msg = Float32()
            avg_velocity_msg.data = self.avg_linear_velocity
            self.avg_velocity_pub.publish(avg_velocity_msg)

    def find_max_velocity(self):
        if self.total_linear_velocity != 0:
            rospy.loginfo(f'Max Linear Velocity: {self.max_linear_velocity} m/s')
            max_velocity_msg = Float32()
            max_velocity_msg.data = self.max_linear_velocity
            self.max_velocity_pub.publish(max_velocity_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('distance_and_velocity_calculator')
        distance_calculator = DistanceCalculator()
        velocity_calculator = VelocityCalculator()
        rate = rospy.Rate(1)  # Set the rate at which both distance and velocity are calculated

        while not rospy.is_shutdown():
            rospy.loginfo(f"==================================================")
            # Calculate distance
            distance_calculator.calculate_distance()
            # Calculate velocity
            velocity_calculator.calculate_avg_velocity()
            velocity_calculator.find_max_velocity()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
