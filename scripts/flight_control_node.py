#!/usr/bin/env python
import rospy
from flight_control import camera_state

rospy.init_node("flight_control_node", anonymous=False)
cam_publisher = camera_state()

loop_rate = rospy.Rate(1)
while not rospy.is_shutdown():
    cam_publisher.spin()
    loop_rate.sleep()
