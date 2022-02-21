import numpy as np
import signal
from subprocess import Popen
from subprocess import check_output
import time

# OPTION ROBOTIQUE 2021-2022    PROJET INTEG     -IDENTIFICATION-                FRAPPEREAU DELACOUX

# In this file you'll find the script we used to generate data for our training algorithm
# We need to collect data from GAZEBO simulations all run with different sets of parameters (masses, inertias and centers of mass)

# This script first creates a 6 dimentional sampled array to test for every combination of parameters within a given space
# The limits we set on the parameters are quite random, we might have to revise this.



Inertias_1 = 12
Inertias_2 = 1 
Masses_1 = 1
Masses_2 = 1
CoM_1 = 0.4                #These values are hardcoded in for now because we can't extract the length of each link yet
CoM_2 = 0.3


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

