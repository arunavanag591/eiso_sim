#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class odometry_to_posestamped():
    def __init__(self):

        self.pose_pub = rospy.Publisher("/pose_stamped", PoseStamped, queue_size=50)
        rospy.Subscriber("/multirotor/truth/NED", Odometry, self.Odometry_callback)


    def Odometry_callback(self, odom_msg):
        pose_msg = PoseStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose.pose

        self.pose_pub.publish(pose_msg)
