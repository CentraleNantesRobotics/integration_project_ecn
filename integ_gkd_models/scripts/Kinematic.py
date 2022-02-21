#!/usr/bin/env python
from __future__ import print_function

import rospy
import yaml
import os

from math import cos, sin
from sensor_msgs.msg import JointState
from gkd_models.srv import Kinematic,KinematicResponse


# load robot parameters from yaml file
path=os.path.dirname(__file__)

with open(os.path.join(path,'RobotParam.yml')) as f :
	yaml_dict = yaml.safe_load(f)
	print(yaml_dict)
	l1 = yaml_dict["l1"]
	l2 = yaml_dict["l2"]
	


def handle_Kinematic(req):
	
	# recover joint velocities from request
	theta=req.input.position
	theta_dot=req.input.velocity
	
	# compute end effector velocity
	velocity=[0,0]
	velocity[0]  = -l1*theta_dot[0]*sin(theta[0])-l2*(theta_dot[0]+theta_dot[1])*sin(theta[0]+theta[1])
	velocity[1]  = l1*theta_dot[0]*cos(theta[0])+l2*(theta_dot[0]+theta_dot[1])*cos(theta[0]+theta[1])
	return KinematicResponse(velocity)

def Kinematic_server():
    rospy.init_node('Kinematic_server')
    s = rospy.Service('Kinematic', Kinematic, handle_Kinematic)
    print("Kinematic Direct Model")
    rospy.spin()

if __name__ == "__main__":
    Kinematic_server()
