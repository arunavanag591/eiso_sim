#!/usr/bin/env python


import rospy
import time
import os
import yaml
from Odometry_to_PoseStamped import odometry_to_posestamped

from rosflight_msgs.msg import *
from rosflight_msgs.srv import *
from std_srvs.srv import *


def main():


    file_path = os.path.dirname(__file__)
    if file_path != "":
        os.chdir(file_path)

    stream = open("../params/script_params.yaml", 'r')
    dictionary = yaml.safe_load(stream)
    # for key, value in dictionary.items():
    #     print (key + " : " + str(value))


    rospy.init_node('RC', anonymous=False)

    # Setup conversion node #

    otp = odometry_to_posestamped()

    r = rospy.Rate(50)

    # Setup publisher #

    pub = rospy.Publisher('/multirotor/RC', RCRaw, queue_size=20)

    # Setup parameters #

    if dictionary["use_imu"]:

        rospy.wait_for_service('calibrate_imu')
        imu = rospy.ServiceProxy('calibrate_imu', Trigger)
        imu()

    if dictionary["use_mixer"]:

        rospy.wait_for_service('param_set')
        mixer = rospy.ServiceProxy('param_set', ParamSet)
        mixer("MIXER", dictionary["mixer"])

    channel = rospy.ServiceProxy('param_set', ParamSet)

    if dictionary["use_arm_channel"]:

        channel('ARM_CHANNEL', dictionary["channel"])

    if dictionary["use_min_throttle"]:

        minimum = rospy.ServiceProxy('param_set', ParamSet)
        minimum("MIN_THROTTLE", dictionary["min_throttle"])

    rospy.sleep(5.0)

    i = 0

    while not rospy.is_shutdown():

        cmd = RCRaw()
        cmd.values = [1500, 1500, 1000, 1500, 1000, 2000, 2000, 1000]

        if i == 300:
            channel('ARM_CHANNEL', 6)
            i += 1

        elif 49 < i < 300:
            i += 1

        elif i < 50:
            cmd.values[4] = 2000
            i += 1

        pub.publish(cmd)

        r.sleep()

    rospy.loginfo('RC node has shutdown')
    rospy.signal_shutdown()


if __name__ == '__main__':
    main()
