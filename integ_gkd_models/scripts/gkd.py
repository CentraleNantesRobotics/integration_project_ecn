#!/usr/bin/env python
from __future__ import print_function

import rospy
import yaml
import numpy as np #np.dot
import os.path


from math import cos, sin,atan2, sqrt
from sensor_msgs.msg import JointState
from gkd_models.srv import Dynamic,DynamicResponse,Kinematic,KinematicResponse,MGD,MGDResponse,MGI,MGIResponse

	
path=os.path.dirname(__file__)

with open(os.path.join(path,'RobotParam.yml')) as f :
	yaml_dict = yaml.safe_load(f)
	l1 = yaml_dict.get("l1")
	l2 = yaml_dict.get("l2")
	m0 = yaml_dict.get("m0")
	m1 = yaml_dict.get("m1")
	m2 = yaml_dict.get("m2")
	Iz1 = yaml_dict.get("Iz1")
	Iz2 = yaml_dict.get("Iz2")
	c1 = yaml_dict.get("c1")
	c2 = yaml_dict.get("c2")
	g = yaml_dict.get("g")


def handle_Dynamic(req):
	
	# recover joint positions, velocities and acceleration from request
	theta = req.input.position
	theta_d = req.input.velocity
	theta_d_d = req.input.effort
	
	# compute intermediate variables
	Z1=Iz2+m2*c2**2
	Z2=m2*l2*c2
	Z3=m2*l1*c2
	Z4=Iz1+m1*c1**2
	Z5=m2*l1**2
	Z6=m2*l1*(2*c2-l2)
	Z7=m2*l1*l2
	Z8=m2*l1*(l2-c2)
	Z9=(m2*c2+m0*l2)*g
	Z10=(m0*l1+m1*c1+m2*l1)*g
	
	# compute final matrixes
	# T = D.qdotdot + C.qdot + G
	D=[[Z1-Z2+Z3*cos(theta[1]) , Z1],[Z1+Z4-Z2+Z5+Z6*cos(theta[1]) , Z1+Z3*cos(theta[1])]]
	
	C=[[-Z3*theta_d[0]*sin(theta[1]) , 0],[(-Z7*theta_d[0]-Z8*theta_d[1])*sin(theta[1]) , (Z3*theta_d[1]-Z8*theta_d[0])*sin(theta[1])]]
	
	G=[-Z9*cos(theta[0]+theta[1]),-Z9*cos(theta[0]+theta[1])+Z10*cos(theta[0])]
	
	output=JointState()
	output.effort=(np.dot(D,req.input.effort)+np.dot(C,theta_d)+G).tolist()
	
	return DynamicResponse(output)
	
def handle_Kinematic(req):
	
	# recover joint velocities from request
	theta=req.input.position
	theta_dot=req.input.velocity
	
	# compute end effector velocity
	velocity=[0,0]
	velocity[0]  = -l1*theta_dot[0]*sin(theta[0])-l2*(theta_dot[0]+theta_dot[1])*sin(theta[0]+theta[1])
	velocity[1]  = l1*theta_dot[0]*cos(theta[0])+l2*(theta_dot[0]+theta_dot[1])*cos(theta[0]+theta[1])
	return KinematicResponse(velocity)	
	
def handle_MGD(req):
	theta = req.input.position
	pos=[0,0]
	pos[0] = l1*cos(theta[0])+l2*cos(theta[0]+theta[1])	
	pos[1] = l1*sin(theta[0])+l2*sin(theta[0]+theta[1])
	return MGDResponse(pos)	

def handle_MGI(req):
	
	x=req.pos[0]
	y=req.pos[1]
	
	c2=(x**2+y**2-l1**2-l2**2)/(2*l1*l2)
	if req.high_elbow :
		s2=-sqrt(1-c2**2)
	else :
		s2=sqrt(1-c2**2)
		
	s1=((l1+l2*c2)*y-l2*s2*x)/(x**2+y**2)
	c1=((l1+l2*c2)*x+l2*s2*y)/(x**2+y**2)
	
	output=JointState()
	output.position.append(atan2(s1,c1))
	output.position.append(atan2(s2,c2))
	
	return MGIResponse(output)

def gkd_server():
    rospy.init_node('gkd_server')
    s1 = rospy.Service('Dynamic', Dynamic, handle_Dynamic)
    s2 = rospy.Service('Kinematic', Kinematic, handle_Kinematic)
    s3 = rospy.Service('MGD', MGD, handle_MGD)
    s4 = rospy.Service('MGI', MGI, handle_MGI)
    print("Services are operational")
    rospy.spin()

if __name__ == "__main__":
    gkd_server()
