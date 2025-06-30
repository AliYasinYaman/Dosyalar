#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
import math

class OdometryNode:
    def __init__(self):
        # Parameters
        self.wheel_radius = 0.135  # Wheel radius in meters
        self.wheel_base = 0.5  # Distance between wheels in meters (example value, adjust if needed)

        # Robot State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Robot's orientation in radians

        # Motor RPM Defaults
        self.left_rpm = 0.0
        self.right_rpm = 0.0

        # Initialize Subscribers
        rospy.Subscriber('/drive_system_left_motors_feedbacks', Float64MultiArray, self.left_motor_callback)
        rospy.Subscriber('/drive_system_right_motors_feedbacks', Float64MultiArray, self.right_motor_callback)
        rospy.Subscriber('/imu1/data', Imu, self.imu_callback)

        # Initialize Publisher
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        # Debug logs
        rospy.loginfo("Odometry node initialized")

    def left_motor_callback(self, msg):
        self.left_rpm = sum(msg.data) / len(msg.data)  # Average RPM from front and rear motors
        rospy.loginfo(f"Left Motor RPM: {self.left_rpm}")

    def right_motor_callback(self, msg):
        self.right_rpm = sum(msg.data) / len(msg.data)  # Average RPM from front and rear motors
        rospy.loginfo(f"Right Motor RPM: {self.right_rpm}")

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        _, _, self.theta = tf.transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        rospy.loginfo(f"IMU Orientation (Theta): {self.theta}")

    def compute_odometry(self):
        # Convert RPM to linear velocity
        left_v = (self.left_rpm * 2 * math.pi / 60) * self.wheel_radius
        right_v = (self.right_rpm * 2 * math.pi / 60) * self.wheel_radius

        # Compute linear and angular velocities
        v = (left_v + right_v) / 2  # Linear velocity
        omega = (right_v - left_v) / self.wheel_base  # Angular velocity

        # Time step
        dt = 0.1  # Assuming 10 Hz loop rate

        # Update robot's position and orientation
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        # Publish Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(*quaternion)

        self.odom_pub.publish(odom_msg)
        rospy.loginfo(f"Published Odometry: x={self.x}, y={self.y}, theta={self.theta}")

    def spin(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.compute_odometry()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('odometry_assignment', anonymous=True)
    node = OdometryNode()
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass
