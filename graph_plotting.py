import numpy as np
import matplotlib.pyplot as plt

nlo_vtol = np.load('nlo_vtol.npy').squeeze()
xkf_vtol = np.load('xkf_vtol.npy').squeeze()
true = np.load('true_vtol.npy').squeeze()
refs = np.load('refs.npy')
noisy_vtol = np.load('noisy_vtol.npy').squeeze()

nlo_vtol_error = nlo_vtol[:,:2] - refs 
xkf_vtol_error = xkf_vtol[:,:2] - refs 
true_error = true[:,:2] - refs 

# Plot the state iterations in z
plt.figure()
plt.plot(range(nlo_vtol.shape[0]),nlo_vtol[:,0],label='NLO VTOL States',color='blue')
plt.plot(range(nlo_vtol.shape[0]),xkf_vtol[:,0],label='XKF VTOL States',color='red')
plt.plot(range(nlo_vtol.shape[0]),refs[:,0],label='Refs',color='black')
plt.plot(range(nlo_vtol.shape[0]),true[:,0],label='True VTOL',color='green')
plt.plot(range(nlo_vtol.shape[0]),noisy_vtol[:,0],label='Noisy VTOL States',color='purple')
plt.legend()
plt.xlabel('Timestep')
plt.ylabel('Pos(m)')
plt.title('State plots z value. NLO-only Observer')

# Plot the state iterations in h
plt.figure()
plt.plot(range(nlo_vtol.shape[0]),nlo_vtol[:,1],label='NLO VTOL States',color='blue')
plt.plot(range(nlo_vtol.shape[0]),xkf_vtol[:,1],label='XKF VTOL States',color='red')
plt.plot(range(nlo_vtol.shape[0]),refs[:,1],label='Refs',color='black')
plt.plot(range(nlo_vtol.shape[0]),true[:,1],label='True VTOL',color='green')
plt.plot(range(nlo_vtol.shape[0]),noisy_vtol[:,1],label='Noisy VTOL States',color='purple')
plt.legend()
plt.xlabel('Timestep')
plt.ylabel('Pos(m)')
plt.title('State plots h value. NLO-only Observer')

# Plot the error in z
plt.figure()
plt.plot(range(nlo_vtol.shape[0]),nlo_vtol_error[:,0],label='NLO VTOL States',color='blue')
plt.plot(range(nlo_vtol.shape[0]),xkf_vtol_error[:,0],label='XKF VTOL States',color='red')
plt.plot(range(nlo_vtol.shape[0]),true_error[:,0],label='Refs',color='black')
plt.legend()
plt.xlabel('Timestep')
plt.ylabel('Error')
plt.title('Error in the z value')

# Plot the error in h
plt.figure()
plt.plot(range(nlo_vtol.shape[0]),nlo_vtol_error[:,1],label='NLO VTOL States',color='blue')
plt.plot(range(nlo_vtol.shape[0]),xkf_vtol_error[:,1],label='XKF VTOL States',color='red')
plt.plot(range(nlo_vtol.shape[0]),true_error[:,1],label='Refs',color='black')
plt.legend()
plt.xlabel('Timestep')
plt.ylabel('Error')
plt.title('Error in the h value')
plt.show()
