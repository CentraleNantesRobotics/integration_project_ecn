import importlib
try:
	importlib.reload(pre_processing)
except:
	pass
from pre_processing import pre_process
import numpy as np
import matplotlib.pyplot as plt
from sklearn.neural_network import MLPRegressor
from sklearn.model_selection import train_test_split
import time

# OPTION ROBOTIQUE 2021-2022    PROJET INTEG     -IDENTIFICATION-                N.FRAPPEREAU & J.DELACOUX

# In this file you'll find our training algorithm
# The data has already been "collected" and extracted
# All that's left to do is to choose a model architecture and fit it to 
# our datasets.

# This function lists the files in the /dataset folder, reads them, 
# extracts the data from them and put it in a np.array
# When this is done for a .bag file, we close it and go to the next one 
# in the /dataset folder, until there aren't any .bag file left


start_time = time.time()

# ~ Preprocess the existing data
# ~ It could be interesting to save this pre-processed data somewhere 
# ~ and load it when this script is executed
data = pre_process()

print(f'Size of the dataset : {data.shape}')

# ~ Defines the input of our system. As explained in the 
# ~ pre_processing.py script, the first 8 columns of the data file 
# ~ represent the (theta1, omega1, effort1, acceleration1, theta2, 
# ~ omega2, effort2, acceleration2) which are what we feed our network
X = data[:,:8]

# ~ Defines the desired output of our system. We try to predict the 
# ~ parameter values independently in our first approach. It is quite 
# ~ frankly bound to fail as these parameters aren't observable on their
# ~ own, and only linear combinations of them are.
# ~ Nevertheless, in this first approach, this is what we do.
Y_I1 = data[:,8]
Y_I2 = data[:,9]
Y_m1= data[:,10]
Y_m2 = data[:,11]
Y_com1 = data[:,12]
Y_com2 = data[:,13]


# ~ Split the input data into train and test data then fit our model to 
# ~ best evaluate the parameters. 
x_train, x_test, y_train_I1, y_test_I1 = train_test_split(X, Y_I1, random_state=16)

# ~ We use the MLP regressor with its default parameters. It could be 
# ~ interesting to change the model and or it's parameters 
# ~ in order to get better results
inertia1_reg = MLPRegressor()

# ~ Fit our model to best evaluate the parameters. 
inertia1_reg.fit(x_train, y_train_I1)

# ~ The .score() function returns the r² of the regression : 
# ~ closer to 1.0 ==> better
print(f'R² of the Inertia1 estimator : {inertia1_reg.score(x_test, y_test_I1)}')


# ~ We do that for every parameter we want to estimate
x_train, x_test, y_train_I2, y_test_I2 = train_test_split(X, Y_I2, random_state=16)
inertia2_reg = MLPRegressor()
inertia2_reg.fit(x_train, y_train_I2)
print(f'R² of the Inertia2 estimator : {inertia2_reg.score(x_test, y_test_I2)}')

x_train, x_test, y_train_m1, y_test_m1 = train_test_split(X, Y_m1, random_state=16)
mass1_reg = MLPRegressor()
mass1_reg.fit(x_train, y_train_m1)
print(f'R² of the mass1 estimator : {mass1_reg.score(x_test, y_test_m1)}')

x_train, x_test, y_train_m2, y_test_m2 = train_test_split(X, Y_m2, random_state=16)
mass2_reg = MLPRegressor()
mass2_reg.fit(x_train, y_train_m2)
print(f'R² of the mass2 estimator : {mass2_reg.score(x_test, y_test_m2)}')

x_train, x_test, y_train_com1, y_test_com1 = train_test_split(X, Y_com1, random_state=16)
com1_reg = MLPRegressor()
com1_reg.fit(x_train, y_train_com1)
print(f'R² of the center_of_mass1 estimator : {com1_reg.score(x_test, y_test_com1)}')

x_train, x_test, y_train_com2, y_test_com2 = train_test_split(X, Y_com2, random_state=16)
com2_reg = MLPRegressor()
com2_reg.fit(x_train, y_train_com2)
print(f'R² of the center_of_mass1 estimator : {com2_reg.score(x_test, y_test_com2)}')



print("--- Global training and evaluation time :  %s seconds ---" % (time.time() - start_time))
