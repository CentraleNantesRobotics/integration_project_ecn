#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

rospy.init_node('state')
rospy.init_node('trajectory')

