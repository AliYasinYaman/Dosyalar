#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class PathPublisher:
    def __init__(self):
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.path = Path()
        self.path.header.frame_id = "odom"
        self.max_path_length = 1000  # Max poses to keep

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.header.stamp = rospy.Time.now()
        self.path.poses.append(pose)

        # Limit path length to avoid memory overflow
        if len(self.path.poses) > self.max_path_length:
            rospy.loginfo("Path length exceeded max, clearing old poses")
            self.path.poses.pop(0)

        self.path_pub.publish(self.path)
        rospy.loginfo(f"Published path with {len(self.path.poses)} poses")

if __name__ == "__main__":
    rospy.init_node('path_publisher', anonymous=True)
    pp = PathPublisher()
    rospy.spin()
