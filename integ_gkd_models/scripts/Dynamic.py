#!/usr/bin/env python
from __future__ import print_function

import rospy
import yaml
import numpy as np #np.dot
import os.path


from math import cos, sin
from sensor_msgs.msg import JointState
from gkd_models.msg import dyn_mats
from gkd_models.srv import Dynamic ,DynamicResponse

# load robot parameters :
	# mi = mass 
	# li = arm lenght 
	# Izi = z axis inertia
	# ci = arm joint to center of mass distance
	
path=os.path.dirname(__file__)

with open(os.path.join(path,'RobotParam.yml')) as f :
	yaml_dict = yaml.safe_load(f)
	l1 = yaml_dict.get("l1")
	l2 = yaml_dict.get("l2")
	m1 = yaml_dict.get("m1")
	m2 = yaml_dict.get("m2")
	Iz1 = yaml_dict.get("Iz1")
	Iz2 = yaml_dict.get("Iz2")
	g = yaml_dict.get("g")
	c1 = yaml_dict.get("c1")
	c2 = yaml_dict.get("c2")


def handle_Dynamic(req):
	
	# recover joint positions, velocities and acceleration from request
	theta = req.input.position
	theta_d = req.input.velocity
	theta_d_d = req.input.effort

	Z1 = m1*c1**2 + m2*(l1**2+c2**2+2*l1*c2*cos(theta[1]) + Iz1 + Iz2)
	Z2 = m2*(c2**2+l1*c2*cos(theta[1])) + Iz2
	Z3 = m2*c2**2 + Iz2
	Z4 = m2*c2*g*cos(theta[0]+theta[1])+(m1*c1+m2*l1)*g*cos(theta[0])
	Z5 = m2*c2*g*cos(theta[0]+theta[1])
	h = -m2*l1*c2*sin(theta[1])
	
	# compute final matrixes
	# T = D.qdotdot + C.qdot + G
	D=[[ Z1 , Z2 ],[ Z2 , Z3 ]]
	
	C=[[h * theta_d[1], h * (theta_d[0]+theta_d[1]) ],[ -h * theta_d[0], 0]]
	
	G=[[ Z4] , [Z5 ]]
	
	output=dyn_mats()
	E = np.dot(C,theta_d)+G
		
	M = []
	N = []
	for m in D:
		M += m
	for n in E:
		N += n
	output.m_coeffs = M
	output.n_coeffs = N	
	return DynamicResponse(output)

def Dynamic_server():
    rospy.init_node('Dynamic_server')
    s = rospy.Service('Dynamic', Dynamic, handle_Dynamic)
    #print("Dynamic Model Direct")
    rospy.spin()

if __name__ == "__main__":
    Dynamic_server()
