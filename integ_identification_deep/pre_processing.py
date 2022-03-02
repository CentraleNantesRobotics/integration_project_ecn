from os import listdir
from os.path import isfile, join
import rosbag
import numpy as np
import random
from scipy.signal import savgol_filter

def pre_process():
    # ~ List all files in the dataset folder
    bag_list = [f for f in listdir('dataset/') if isfile(join('dataset/', f))]

    # ~ Definition of the training data array
    data = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0])

    for bag_file in bag_list:
        # ~ Read a bag file from the list of files
        bag = rosbag.Bag('dataset/'+bag_file)

        # ~ Extracts the parameter values from the bag name
        parameters = bag_file.split('-')
        parameters[-1] = parameters[-1][:-4]
        parameters = [float(parameter) for parameter in parameters]
        parameters = np.array(parameters)
        
        # ~ Data array from this SPECIFIC bag file
        input_data = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0])

        # ~ Extracting data from bag file, appending the parameter values at the end of each line
        for topic, msg, t in bag.read_messages(topics=['/joint_states']):
            new_line = np.array([msg.position[0],msg.velocity[0],msg.effort[0],0,msg.position[1],msg.velocity[1],msg.effort[1],0])
            new_line = np.append(new_line, parameters, 0)
            input_data = np.vstack((input_data, new_line))

        # ~ Calculating accelerations of both links		/!\ PAS SUR DU TOUT DE LA QUALITE DE LA DERIVEE SECONDE OBTENUE PAR CETTE METHODE /!\
        input_data[:,3] = savgol_filter(input_data[:,1], 5, 3, deriv=1, delta=0.1)
        input_data[:,7] = savgol_filter(input_data[:,5], 5, 3, deriv=1, delta=0.1)

        # ~ for i in range(2,len(input_data)-1):
            # ~ accel1 = (input_data[i+1][1]-input_data[i-1][1])/0.2
            # ~ accel2 = (input_data[i+1][5]-input_data[i-1][5])/0.2

            # ~ print((input_data[i,3]-accel1)/accel1)
            # ~ print((input_data[i,7]-accel2)/accel2)

        # ~ Removing values around mechanical stops
        index = 0
        while index < len(input_data):
            if abs(input_data[index][0])>3.0 or abs(input_data[index][4])>3.0:
                input_data = np.delete(input_data, index, 0)
            else :
                index += 1

        # ~ input_data = np.append(input_data, parameter_array, 1)

        # ~ Remove first line (only zeros) and last line (no accelation value)
        input_data = input_data[2:-1]

        # ~ Add the current bag's data to the global data array
        data = np.vstack((data, input_data))[1:]
        
        # ~ Close .bag file
        bag.close()
    
    
    
    # ~ Shuffle the data 
    np.take(data,np.random.rand(data.shape[0]).argsort(),axis=0,out=data)
    return data



if __name__ == '__main__':
        pre_process()
