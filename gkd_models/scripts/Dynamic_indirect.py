#!/usr/bin/env python
from __future__ import print_function

import rospy
import yaml
import numpy as np #np.dot
import os.path

from math import cos, sin
from sensor_msgs.msg import JointState
from gkd_models.srv import Dynamic_inverse,Dynamic_inverseResponse

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

def handle_Dynamic_inverse(req):
	theta = req.input.position
	theta_d = req.input.velocity
	efforts = req.input.effort
	
	Z1 = m1*c1**2 + m2*(l1**2+c2**2+l1*l2*cos(theta[1]) + Iz1 + Iz2
	Z2 = m2*(c2**2+l1*c2*cos(theta[1])) + Iz2
	Z3 = m2*c2**2 + Iz2
	Z4 = m2*c2*g*cos(theta[0]+theta[1])
	Z5 = m2*c2*g*cos(theta[0]+theta[1])+(m1*c2+m2*l1)*g*cos(theta[0])
	h = -0.5*m2*l1*l2*sin(theta[1])
	
	D=[[ Z1 , Z2 ],[ Z2 , Z3 ]]
	
	C=[[h * theta_d[1], h * (theta_d[0]+theta_d[1]) ],[ -h * theta_d[0], 0]]
	
	G=[ Z4 , Z5 ]
	
	output=JointState()
	Gamma = np.linalg.inv(D)*(efforts - np.dot(C,theta_d) - G)
	output.effort=Gamma
	return Dynamic_inverseResponse(output) #????


def Dynamic_inverse_server():
    rospy.init_node('Dynamic_inverse_server')
    s = rospy.Service('Dynamic', Dynamic, handle_Dynamic_inverse)
    print("Dynamic Model Indirect")
    rospy.spin()
    
    
if __name__ == "__main__":
	Dynamic_inverse_server()
