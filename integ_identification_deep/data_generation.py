import numpy as np
from random import random
import signal
from subprocess import Popen
from subprocess import check_output

# OPTION ROBOTIQUE 2021-2022    PROJET INTEG     -IDENTIFICATION-                N.FRAPPEREAU & J.DELACOUX

# In this file you'll find the script we used to generate data for our training algorithm
# We need to collect data from GAZEBO simulations run with different sets of parameters (masses, inertias and centers of mass)

# This script first creates a 6 dimentional sampled array of our parameters to test for every combination of parameters within preset limits
# The limits we set on the parameters are quite random for now, we might have to revise this later on if we face issues.





def gather_data(mass_min,mass_max,inertia_min,inertia_max,r1_min,r1_max,r2_min,r2_max):
		# ~ 
		# ~ Generates a given number of pseudo-random datapoints, spanning the entire coordinate space for the training algorithm. 
	# ~ We hope it is a statistically complete representation of the space.                                                   .
	# ~ 
	# ~ 
	data_points = 50
	for i in range(data_points):
		mass_1 = round(random()*(mass_max-mass_min) + mass_min, 3)
		mass_2 = round(random()*(mass_max-mass_min) + mass_min, 3)
		inertia_1 = round(random()*(inertia_max-inertia_min) + inertia_min, 3)
		inertia_2 = round(random()*(inertia_max-inertia_min) + inertia_min, 3)
		com_1 = round(random()*(r1_max-r1_min) + r1_min, 3)
		com_2 = round(random()*(r2_max-r2_min) + r2_min, 3)
		simulation = Popen(f'roslaunch integ_gazebo arm.launch mass_1:={mass_1} mass_2:={mass_2} ixx_1:={inertia_1} ixx_2:={inertia_2} com_1:={com_1} com_2:={com_2} gui:=false --no-summary'.split())
		record_data = Popen(f'rosbag record -q -O /user/eleves/nfrapperea2019/ros/src/integration_project_ecn/integ_identification_deep/dataset/{inertia_1}-{inertia_2}-{mass_1}-{mass_2}-{com_1}-{com_2} /joint_states '.split())
		playback_input = Popen('rosbag play -q input_data.bag'.split())
		#attendre 4 secondes de temps de simulation, ptet que ça peut se faire en 0,5s de temps réel ??
		time.sleep(6)   #dans le doute
		#end it all
		simulation.send_signal(signal.SIGINT)
		record_data.send_signal(signal.SIGINT)
		playback_input.send_signal(signal.SIGINT)
		while True:
			time.sleep(1)
			out = check_output(['ps','-A']).decode().splitlines()
			out = [line.split()[-1] for line in out]
			if any(proc in out for proc in ('rosmaster', 'gzclient', 'gzserver')):
				continue
			break
	







if __name__ == '__main__':


	# Range between which we want to sim masses
	mass_min = 0.5
	mass_max = 2

	# Range between which we want to sim inertias
	inertia_min = 0.1
	inertia_max = 1

	# in Gazebo: l1 = 0.8 and l2 = 0.6
	r1_min = 0.1    # minimal distance between the center of mass of link2 and the first axis of rotation (joint1)
	r1_max = 0.8
	r2_min = 0.1    # minimal distance between the center of mass of link2 and the second axis of rotation (joint2)
	r2_max = 0.6
	
	gather_data(mass_min,mass_max,inertia_min,inertia_max,r1_min,r1_max,r2_min,r2_max)



