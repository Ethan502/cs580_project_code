import matplotlib.pyplot as plt
import VTOLParam as P
import numpy as np
from signalGenerator import signalGenerator
from VTOLAnimation import VTOLAnimation
from dataPlotter import dataPlotter
from VTOLDynamics import VTOLDynamics
from ctrlPID import ctrlPID
from observer import Observer 


vtol = VTOLDynamics(alpha=0.2)
z_reference = signalGenerator(amplitude=2.5, frequency=0.08, y_offset=3)
h_reference = signalGenerator(amplitude=3, frequency=0.03,y_offset=5)
dataPlot = dataPlotter()
animation = VTOLAnimation()
controller = ctrlPID()
observer = Observer()

t = P.t_start
u_prev = np.zeros((2,1))
dt = P.Ts

# A few things to help save data
nlo_estimates = []
xkf_estimates = []
refs = []
xkf_vtol = []
nlo_vtol = []

while t < P.t_end:
    t_next = t + P.t_plot
    while t < t_next:
        zref = z_reference.square(t)
        href = h_reference.square(t)
        n = np.array([[0.0],[0.0],[0.0]])
        d = np.array([[0.0],[0.0]])
        z = vtol.state[0][0]
        h = vtol.state[1][0]
        theta = vtol.state[2][0]
        # refs.append([zref,href])

        # Generate noisy measurements
        noisy_z = z + np.random.normal(0,0.5)
        noisy_h = h + np.random.normal(0,0.2)
        noisy_theta = theta + np.random.normal(0,0.017)
        
        # Run the measurements through the observer
        observer_state,nlo_state = observer.update(vtol.state,np.array([noisy_z,noisy_h,noisy_theta]),u_prev,dt)
        # nlo_estimates.append(nlo_state.copy())
        # xkf_estimates.append(observer_state.copy())
        obs_z,obs_h,obs_theta = observer_state[0][0],observer_state[1][0],observer_state[2][0]
        # obs_z,obs_h,obs_theta = nlo_state[0][0],nlo_state[1][0],nlo_state[2][0]

        # Feed measurements into the controller and update the VTOL state
        # u = controller.update(href,zref,z,h,theta)
        # u = controller.update(href,zref,noisy_z,noisy_h,noisy_theta)
        u = controller.update(href,zref,obs_z,obs_h,obs_theta)
        y = vtol.update(u + d)
        nlo_vtol.append(vtol.state.copy())

        u_prev = u
        t = t + P.Ts
    
    animation.update(vtol.state)
    dataPlot.update(t,vtol.state,zref,href,u[0][0],u[1][0])
    plt.pause(0.01)


refs = np.array(refs)
nlo_estimates = np.array(nlo_estimates)
xkf_estimates = np.array(xkf_estimates)
xkf_vtol = np.array(xkf_vtol)
nlo_vtol = np.array(nlo_vtol)

# np.save('xkf_output.npy',xkf_estimates)
# np.save('nlo_output.npy',nlo_estimates)
# np.save('xkf_vtol.npy',xkf_vtol)
# np.save('nlo_vtol.npy',nlo_vtol)
# np.save('refs.npy',refs)
# np.save('true_vtol.npy',nlo_vtol)


print('Press key to close')
plt.waitforbuttonpress()
plt.close()
