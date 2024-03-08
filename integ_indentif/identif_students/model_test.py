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

removeTime = 0.20       # Parts of signal affected by filter transient

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

parNames = np.array(["Ja", "zz2", "mx2", "Ia2", "Fs1", "of1", "Fv1", "Fs2", "of2", "Fv2"])
K_ident = np.array([Ja_id, zz2_id, mx2_id, Ia2_id, Fs1_id, of1_id, Fv1_id, Fs2_id, of2_id, Fv2_id])

# Identification des paramètres à élimer
del_col = [] # Choose the columns to eliminate in K_ident
K_ident = np.delete(K_ident, del_col)
W = np.delete(W, del_col, axis=1)

Gam_id = np.dot(W, K_ident)
Gam_id_1 = Gam_id[:16563]
Gam_id_2 = Gam_id[16563:]

# Plot the torques to compare them 
# Plot Torque 1 and Identified Torque 1
plt.figure(figsize=(10, 4))  # Create a new figure for Torque 1 comparisons
plt.plot(t, Gam1, label='Torque 1')  # Plot the first torque
plt.plot(t, Gam_id_1, label='Identified Torque 1')  # Plot the identified torque 1
plt.xlabel('Time [s]')
plt.ylabel('Torque [Nm]')
plt.title('Torque 1 and Identified Torque 1 Comparison')
plt.legend()
plt.show()

# Plot Torque 2 and Identified Torque 2
plt.figure(figsize=(10, 4))  # Create a new figure for Torque 2 comparisons
plt.plot(t, Gam2, label='Torque 2')  # Plot the second torque
plt.plot(t, Gam_id_2, label='Identified Torque 2')  # Plot the identified torque 2
plt.xlabel('Time [s]')
plt.ylabel('Torque [Nm]')
plt.title('Torque 2 and Identified Torque 2 Comparison')
plt.legend()
plt.show()
