import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import filedialog
from scipy.io import loadmat
# from scipy.io import loadmat
# from scipy.signal import filtfilt
# from scipy.signal import butter
import scipy.io as sio
import scipy.signal as sig



# Dynamic identification of the 2R robot in horizontal plane.
#
# Input data: a mat file, with the following information (all vectors
# must have the same length).
# t: time in s, with an assumed constant time interval between samples.
# q1, q2: encoder positions in radians. Position is that of the arm, so
#         if the encoder is on the motor output shaft, the gear ratio 
#         must have been taken into account already.
# Gam1, Gam2: motor torques at rotor, assumed to have been 
#             corrected for gear efficiency in case of real data
#             Must be multiplied by gear ratio.
# 
# Notations:
#   ns         : number of time samples in data file
#   q1, q2     : arm positions in radians.
#   q1f, q2f   : low pass filtered arm positions in radians
#   q1d, q2d   : 1st order derivatives of q1, q2, calculated from q1f,q2f.
#   q1dd, q2dd : 2nd order derivatives of q1, q2, calculated from q1f,q2f.
#   Gam1, Gam2 : torques applied to links, in Nm.


# Set g to zero when testing horizontal motion.
gravity = True

if gravity:
    g = 9.81
else:
    g = 0.0

# Inertial parameters (nominal values from SolidWorks).
# Friction parameters have no nominal value, but they need to be
# initialized. Init value is zero.

m1 = 7.1 
m2 = 3.18  
I1 = 0.0237  
I2 = 0.0359  
c1 = 0.066  
c2 = 0.0825  
n1 = 15  
n2 = 15  
Ia1 = 38e-6  
Ia2 = 38e-6  
l1 = 0.28  

Fs1 = 0.0 
off1 = 0.0 
Fv1 = 0.0 

Fs2 = 0.0 
off2 = 0.0
Fv2 = 0.0 

zz1 = I1 + m1*c2**2 
zz2 = I2 + m2*c2**2 
mx1 = m1*c1  
mx2 = m2*c2  

# Identification tunable parameters

removeTime = 0   # Parts of signal affected by filter transient

# Load data from file
# Initialisation de l'interface graphique pour la sélection de fichier
root = tk.Tk()
root.withdraw()

# Sélection d'un fichier .mat
file_path = filedialog.askopenfilename(title="Select file", filetypes=[("MAT files", "*.mat")])

if file_path:
    # Chargement du fichier .mat
    data = sio.loadmat(file_path)
        
        
    q1 = data['q1'].squeeze()  
    q2 = data['q2'].squeeze()    
    Gam1 = data['Gam1'].squeeze()    
    Gam2 = data['Gam2'].squeeze()    
        
    # Ajustement des paramètres -------------> Utile???
    Gam1 = n1 * Gam1
    Gam2 = n2 * Gam2

    # Calcul de la période d'échantillonnage et de la fréquence d'échantillonnage
    t = data['t'].squeeze()  # Assurez-vous que 't' est bien le vecteur temps dans vos données
    ns = len(t)
    Ts = (t[-1] - t[0]) / (ns - 1)
    Fs = 1 / Ts
    
    # Affichage des informations
    print('----------------------------------------------------------------')
    print(f'File: {file_path}')
    print(f'Sampling frequency: {Fs}')
    print('----------------------------------------------------------------')
else:
    print("Aucun fichier sélectionné.")

    
# Fréquence de coupure
CuttingFreq = 20  # Hz

# Fréquence normalisée
Wn = CuttingFreq / (Fs / 2)

# Calcul des coefficients du filtre Butterworth
B, A = sig.butter(4, Wn)

# Application du filtre passe-bas
q1f = sig.filtfilt(B, A, q1)
q2f = sig.filtfilt(B, A, q2)

# Calcul des dérivées par différence centrée
q1d = np.zeros_like(q1f)
q2d = np.zeros_like(q2f)
q1dd = np.zeros_like(q1f)
q2dd = np.zeros_like(q2f)

q1d[1:-1] = (q1f[2:] - q1f[:-2]) / (2 * Ts)
q2d[1:-1] = (q2f[2:] - q2f[:-2]) / (2 * Ts)
q1dd[1:-1] = (q1d[2:] - q1d[:-2]) / (2 * Ts)
q2dd[1:-1] = (q2d[2:] - q2d[:-2]) / (2 * Ts)

# Calcul des éléments de W13 et W23
W13 = l1 * (2 * np.cos(q2f) * q1dd + np.cos(q2f) * q2dd - np.sin(q2f) * q2d**2 - 2 * np.sin(q2f) * q1d * q2d) + g * np.cos(q1f + q2f)

W23 = l1 * (np.cos(q2f) * q1dd + np.sin(q2f) * q1d**2) + g * np.cos(q1f + q2f)

# Construction des matrices régresseurs W1 et W2
W1 = np.column_stack([q1dd, q1dd + q2dd, W13, np.zeros(ns), np.sign(q1d), np.ones(ns), q1d, np.zeros(ns), np.zeros(ns), np.zeros(ns)])
W2 = np.column_stack([np.zeros(ns), q1dd + q2dd, W23, (n2**2) * q2dd, np.zeros(ns), np.zeros(ns), np.zeros(ns), np.sign(q2d), np.ones(ns), q2d])

#plot q1, dq1, ddq1
# plt.figure()
# plt.subplot(3, 1, 1)
# plt.plot(t, q1, label='q1')
# plt.plot(t, q1f, label='q1f')
# plt.legend()
# plt.grid()
# plt.subplot(3, 1, 2)
# plt.plot(t, q1d, label='dq1')
# plt.legend()
# plt.grid()
# plt.subplot(3, 1, 3)
# plt.plot(t, q1dd, label='ddq1')
# plt.legend()
# plt.grid()
# plt.show()


# We now apply a decimate filer on the columns of W to keep less lines
# and apply a low pass filter to the columns. The subsamplig parameter
# R in decimate(X,R) is set to a the cutting frequency used above.
# See help of the decimate function for the formula below. 
# R = round(0.8*Fs/2/CuttingFreq) ; Here the value is 50.
# For values above 13, it is recommended to decimate in two or more steps.
# I have yet to determine the two decimation factors automatically, so
# the code below is OK for an overall R = 50, by decimating by 10 and 5.

# Decimate W and Gamma. Set Chebyshev cut-off frequency to the cut-off 
# frequency used to filter position.
# Note: that's when the torque gets low-pass filtered.

# Decimate data corresponding to joint 1
R = 50
W1dec = []
W1dec2 = []

for j in range(W1.shape[1]):
    W1dec.append(sig.decimate(sig.decimate(W1[:, j], 10), 5))
    W1dec2.append(sig.decimate(W1[:, j], 50))

# Convertir les listes en tableaux NumPy
W1dec = np.array(W1dec).transpose()
W1dec2 = np.array(W1dec2).transpose()

# Assuming Gam1 is a 1D numpy array or a column in a 2D array
Gam1dec = sig.decimate(sig.decimate(Gam1, 10), 5)

# Decimate data corresponding to joint 2
W2dec = []
# Assuming W2 is a 2D numpy array
for j in range(W2.shape[1]):
    W2dec.append(np.array(sig.decimate(sig.decimate(W2[:, j], 10), 5)))  # Fix: Append the decimated array as a numpy array
W2dec = np.array(W2dec).transpose()
# Assuming Gam2 is a 1D numpy array or a column in a 2D array
Gam2dec = sig.decimate(sig.decimate(Gam2, 10), 5)

# Remove that part of the data which is affected by the transient 
# response of the Butteworth filter and Chebyshev filter used by decimate.

# Number of samples to remove, at original sampling rate.
# removeTime has been determined experimentally, but in theory, it is
# possible to determine it by analyzing the poles of the Butterworth and
# Chebyshev filters.

# Calculate the number of samples to remove at the original sampling rate
remSamples = round(removeTime / Ts)
# Adjust for the decimated sampling rate
remSamples = np.ceil(remSamples / R).astype(int)

# Define the start and end indices for trimming
istart = 1 + remSamples
iend = len(Gam1dec) - remSamples  # Assuming Gam1dec is a 1D numpy array

# Trim the signals
Gam1dec = Gam1dec[istart:iend]
Gam2dec = Gam2dec[istart:iend]
W1dec = W1dec[istart:iend,:]
W2dec = W2dec[istart:iend,:]

# Regroup the trimmed signals into single systems
Wdec = np.vstack((W1dec, W2dec))
Gamdec = np.concatenate((Gam1dec, Gam2dec))

# Now remove equations corresponding to a speed below a certain 
# threshold. Indeed, the Coulomb friction model is known not to be 
# accurate at low speed. Also, at low speed, sign(qd) may spark...
# (and it does).

# Speed threshold
speed_thresh = 0.1  # rd/s

# Filtred Matrix/vector
# Decimate the speed signals
q1d_dec = sig.decimate(q1d, R)
q2d_dec = sig.decimate(q2d, R)

# Initialize filtered matrix/vector
Wf = np.array([]).reshape(0, Wdec.shape[1])
Gamf = np.array([])

# Filter data points based on speed threshold
for i in range(len(q1d_dec)):
    if abs(q1d_dec[i]) > speed_thresh and abs(q2d_dec[i]) > speed_thresh:
        Wf = np.vstack([Wf, Wdec[i, :]])
        Gamf = np.append(Gamf, Gamdec[i])

# Parameter calculation
Ja = zz1 + n1**2 * Ia1 + l1**2 * m2

# Nominal parameters array
Knominal = np.array([Ja, zz2, mx2, Ia2, Fs1, off1, Fv1, Fs2, off2, Fv2])
parNames = np.array(["Ja", "zz2", "mx2", "Ia2", "Fs1", "of1", "Fv1", "Fs2", "of2", "Fv2"])

# Here we do not analyze possible dependency relations between parameters.
# We are expected to be working with minimal (independent) parameters.
# It is the case here.

# Solve overdetermined system:
Kident = np.linalg.pinv(Wdec) @ Gamdec


# We calculate here the estimation error of the parameters.
# Typically, those identified with a high percentage of error are 
# parameters that contribute little to the torque and are later discarded.
sigma_rho = np.linalg.norm(Gamdec - Wdec @ Kident) / np.sqrt(Wdec.shape[0] - Wdec.shape[1])
cov_mat = sigma_rho**2 * np.linalg.inv(Wdec.T @ Wdec)

print('\nIdentification with all independent parameters.')
print('Beware: for parameters close to 0, rel. sigma may not make much sense...')
print('Nominal values for friction parameters are unknown.\n')
print('Param        Nominal       Identified         sigma    rel. sigma (%)')

# Display results for each parameter
for i, name in enumerate(parNames):
    sigma = np.sqrt(cov_mat[i, i])
    rel_sigma = sigma / abs(Kident[i]) * 100 if Kident[i] != 0 else np.inf  # Avoid division by zero
    print(f' {name:<12}{Knominal[i]:12.6g}{Kident[i]:12.6g}{sigma:12.6g}{rel_sigma:12.1f}')

print('\nCondition number of Wdec:', np.linalg.cond(Wdec), '\n')

# Now based on above results, you have to decide which parameters are
# worth keeping in the model. To eliminate the others, you need to 
# eliminate the corresponding columns of W and the corresponding number
# of terms in K.
W = np.vstack([W1, W2])

Ja_id = 0.384973
zz2_id = 0.0772272
mx2_id = 0.0394623
Ia2_id = -0.000184724
Fs1_id = -0.0920058
of1_id = -0.09789
Fv1_id = 0.421948
Fs2_id = 0.268327
of2_id = 0.0596759
Fv2_id = 0.238069

# Identification des paramètres à élimer
del_col = [] # Choose the columns to eliminate in K_ident
W = np.delete(W, del_col, axis=1)
K_ident_opt = np.delete(np.array([Ja_id, zz2_id, mx2_id, Ia2_id, Fs1_id, of1_id, Fv1_id, Fs2_id, of2_id, Fv2_id]), del_col)
parNames = np.delete(parNames, del_col)


# Then use the identified parameters to create the (simplified) model of
# the robot. Use it to calculate the torque, and compare to the measured
# t
# Calculate the torque using the simplified model
Gam_id = np.dot(W, K_ident_opt)
Gam_id_1 = Gam_id[:16563]
Gam_id_2 = Gam_id[16563:]

Gam = np.concatenate((Gam1, Gam2))

# Plot the torques to compare them 

# Set up a figure and two subplots next to each other
plt.figure(figsize=(20, 4))  # Adjust the figure size to accommodate both plots side by side

# Plot Torque 1 and Identified Torque 1 in the first subplot
plt.subplot(1, 2, 1)  # 1 row, 2 columns, 1st subplot
plt.plot(t, Gam1, label='Torque 1')  # Plot the first torque
plt.plot(t, Gam_id_1, label='Identified Torque 1')  # Plot the identified torque 1
plt.xlabel('Time [s]')
plt.ylabel('Torque [Nm]')
plt.title('Torque 1 and Identified Torque 1 Comparison')
plt.legend()

# Plot Torque 2 and Identified Torque 2 in the second subplot
plt.subplot(1, 2, 2)  # 1 row, 2 columns, 2nd subplot
plt.plot(t, Gam2, label='Torque 2')  # Plot the second torque
plt.plot(t, Gam_id_2, label='Identified Torque 2')  # Plot the identified torque 2
plt.xlabel('Time [s]')
plt.ylabel('Torque [Nm]')
plt.title('Torque 2 and Identified Torque 2 Comparison')
plt.legend()

plt.show()  # Display the plots

# Then we recalculate the estimation error of the parameters.
# with the simplified model.
sigma_rho = np.linalg.norm(Gam - W @ K_ident_opt) / np.sqrt(W.shape[0] - W.shape[1])
cov_mat = sigma_rho**2 * np.linalg.inv(W.T @ W)

print('\nIdentification with the new independent parameters.')
print('Param        Nominal       Identified         sigma    rel. sigma (%)')

# Display results for each parameter
for i, name in enumerate(parNames):
    sigma = np.sqrt(cov_mat[i, i])
    rel_sigma = sigma / abs(K_ident_opt[i]) * 100 if K_ident_opt[i] != 0 else np.inf  # Avoid division by zero
    print(f' {name:<12}{Knominal[i]:12.6g}{K_ident_opt[i]:12.6g}{sigma:12.6g}{rel_sigma:12.1f}')

print('\nCondition number of W:', np.linalg.cond(W), '\n')