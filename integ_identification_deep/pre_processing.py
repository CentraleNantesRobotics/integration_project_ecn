from os import listdir
from os.path import isfile, join
import rosbag
import numpy as np

# ~ List all files in the dataset folder
bag_list = [f for f in listdir('dataset/') if isfile(join('dataset/', f))]

# ~ Read a bag file from the list of files
bag = rosbag.Bag('dataset/'+bag_list[0])

# ~ Extracts the parameter values from the bag name
parameters = bag_list[0].split('-')
parameters[-1] = parameters[-1][:-4]

input_data = np.array([])

for topic, msg, t in bag.read_messages(topics=['/joint_states']):
    input_data.append([msg.position[0],msg.velocity[0],msg.effort[0],msg.position[1],msg.velocity[1],msg.effort[1]])

print(input_data)

bag.close()
