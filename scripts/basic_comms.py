#!/usr/bin/env python

import rospy
import rospkg
import roslaunch
import RC_code


def main():

	# Setting up launch files and nodes #
	rospack = rospkg.RosPack()
	path = rospack.get_path('eiso_sim')

	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
	roslaunch.configure_logging(uuid)
	rotor_launch = roslaunch.parent.ROSLaunchParent(uuid, [path+"/launch/sim.launch"])
	rotor_launch.start()
	rospy.loginfo("started")

	rospy.sleep(5.0)

	node = roslaunch.core.Node(package='rosflight', 
			    node_type='rosflight_io', name='rosflight_io', namespace='/', 
				args='_udp:=true', output='screen')

	flight_launch = roslaunch.scriptapi.ROSLaunch()
	flight_launch.start()

	flight_launch.launch(node)

	rospy.sleep(3.0)

	RC_code.main()


if __name__ == '__main__':
	main()