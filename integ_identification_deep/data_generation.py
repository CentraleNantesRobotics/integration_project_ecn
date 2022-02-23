import numpy as np
import signal
from subprocess import Popen
from subprocess import check_output
import time

# OPTION ROBOTIQUE 2021-2022    PROJET INTEG     -IDENTIFICATION-                N.FRAPPEREAU & J.DELACOUX

# In this file you'll find the script we used to generate data for our training algorithm
# We need to collect data from GAZEBO simulations run with different sets of parameters (masses, inertias and centers of mass)

# This script first creates a 6 dimentional sampled array of our parameters to test for every combination of parameters within preset limits
# The limits we set on the parameters are quite random for now, we might have to revise this later on if we face issues.





def gather_data(mass_min,mass_max,inertia_min,inertia_max,r1_min,r1_max,r2_min,r2_max):
    ### create an array containing all possible combinations of the different parameters within their range
    ### loop until you reach the last combination possible (which is when all the parameters are at their maximum value)
    ### during the loop, simulate the robot in a horizontal position in gazebo using the parameters for a duration of 3 to 4 seconds
    ### play previously bagged input data, bag the output position data alongside the input efforts
    ### close the simulation, save the bag data file wi
    ###                                                         .
    ###                                                         .
    ###
    
	Masses_1 = 1
	Masses_2 = 1
    Inertias_1 = 12
	Inertias_2 = 1 
	CoM_1 = 0.4                #These values are hardcoded in for now because we can't extract the length of each link yet
	CoM_2 = 0.3                                                         .

    Masses_1 = np.linspace(mass_min, mass_max, 5)
    Masses_2 = np.linspace(mass_min, mass_max, 5)
    Inertias_1 = np.linspace(inertia_min, inertia_max, 5)
    Inertias_2 = np.linspace(inertia_min, inertia_max, 5)
    CoM_1 = np.linspace(r1_min, r1_max, 5)
    CoM_2 = np.linspace(r2_min, r2_max, 5)


    List_Parameters = [mass_min, mass_min, inertia_min, inertia_min, r1_min, r2_min]
    simulation = Popen(f'roslaunch integ_gazebo arm.launch Izz_1:={Inertias_1} Izz_2:={Inertias_2} gui:=false'.split())
	record_data = Popen(f'rosbag record -O {Inertias_1}-{Inertias_2}-{Masses_1}-{Masses_2}-{CoM_1}-{CoM_2} "liste des topis à enregistrer"'.split())
	playback_input = Popen('rosbag play input_data.bag'.split())
	#attendre 4 secondes de temps de simulation, ptet que ça peut se faire en 0,5s de temps réel ??
	time.sleep(4) 	#dans le doute

	#end it all
	simulation.send_signal(signal.SIGINT)  

	while True:
		time.sleep(1)
		out = check_output(['ps','-A']).decode().splitlines()
		out = [line.split()[-1] for line in out]
		if any(proc in out for proc in ('rosmaster', 'gzclient', 'gzserver')):
			continue
		break








if __name__ == '__main__':


    # Range between which we want to sim masses
    mass_min = 0.1
    mass_max = 2

    # Range between which we want to sim inertias
    inertia_min = 0.1
    inertia_max = 2

    # in Gazebo: l1 = 0.8 and l2 = 0.6
    r1_min = 0.1    # minimal distance between the center of mass of link2 and the first axis of rotation (joint1)
    r1_max = 0.8
    r2_min = 0.1    # minimal distance between the center of mass of link2 and the second axis of rotation (joint2)
    r2_max = 0.6

    gather_data(mass_min,mass_max,inertia_min,inertia_max,r1_min,r1_max,r2_min,r2_max)



