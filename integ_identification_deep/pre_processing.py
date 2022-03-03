from os import listdir
from os.path import isfile, join
import rosbag
import numpy as np
from scipy.signal import savgol_filter

# OPTION ROBOTIQUE 2021-2022    PROJET INTEG     -IDENTIFICATION-                N.FRAPPEREAU & J.DELACOUX

# In this file you'll find the script we used to pre-process data for our training algorithm
# The data has already been "collected" and is inside .bag files in the /dataset folder.
# We need to extract that data and put it in a format usable by our training algorithm - we chose np.arrays 

# This function lists the files in the /dataset folder, reads them, extracts the data from them and put it in a np.array
# When this is done for a .bag file, we close it and go to the next one in the /dataset folder, until there aren't any .bag file left



def pre_process():
	# ~ Number of columns of the final data file: - 8 columns for the input of our neural network : (theta1, omega1, effort1, acceleration1, theta2, omega2, effort2, acceleration2)   
	# ~											  - 6 columns for the desired output of our neural network : (I1, I2, m1, m2, com1, com2)
	dim = 14
	
    # ~ List all files in the dataset folder
    bag_list = [f for f in listdir('dataset/') if isfile(join('dataset/', f))]

    # ~ Definition of the overall data array which is the concatenation of the arrays of data extracted from each .bag file
    data = np.zeros((0,dim))

    for bag_file in bag_list:
        # ~ Read a bag file from the list of files
        bag = rosbag.Bag('dataset/'+bag_file)

        # ~ Extracts the parameter values from the bag name and saves them in a np.array
        parameters = bag_file.split('-')
        parameters[-1] = parameters[-1][:-4]
        parameters = [float(parameter) for parameter in parameters]
        parameters = np.array(parameters)
                
        # ~ Declares the data array from this SPECIFIC bag file
        input_data = np.zeros((0,dim))

        # ~ Extracting data from bag file, appending the parameter values at the end of each line
        for topic, msg, t in bag.read_messages(topics=['/joint_states']):
            new_line = np.array([msg.position[0],msg.velocity[0],msg.effort[0],0,msg.position[1],msg.velocity[1],msg.effort[1],0])
            new_line = np.append(new_line, parameters, 0)
            input_data = np.vstack((input_data, new_line.reshape(1,dim)))

        # ~ Calculating accelerations of both links using a Savitzky-Golay filter of the 3rd order with a window size of 5 applied to the speed data
        input_data[:,3] = savgol_filter(input_data[:,1], 5, 3, deriv=1, delta=0.1)
        input_data[:,7] = savgol_filter(input_data[:,5], 5, 3, deriv=1, delta=0.1)

        # ~ Removing values around mechanical stops
        index = 0
        while index < len(input_data):
            if abs(input_data[index][0])>3.0 or abs(input_data[index][4])>3.0:
                input_data = np.delete(input_data, index, 0)
            else :
                index += 1


        # ~ Remove first and last 3 lines because the Savitzky-Golay derivative of the speed is not well defined there
        input_data = input_data[3:-3]

        # ~ Add the current bag's data to the global data array
        data = np.vstack((data, input_data))
        
        # ~ Close .bag file
        bag.close()
    
    return data

