import numpy as np
import VTOLParam as P
alpha = 0.2


mr = P.mr * (1.+alpha*(2.*np.random.rand()-1.))
# mass of center
mc = P.mc * (1.+alpha*(2.*np.random.rand()-1.))
# distance from center to engine
d = P.d * (1.+alpha*(2.*np.random.rand()-1.))
# mew drag
mu = P.mu * (1.+alpha*(2.*np.random.rand()-1.))  

M = mc + 2*mr

A = np.array([
    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, -mu/M, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
])
C = np.array([
    [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
])
H = np.array([
    [1.0, 0.0, 0.0],   # z correction
    [0.0, 1.0, 0.0],   # h correction
    [0.0, 0.0, 1.0],   # theta correction
    [1.5, 0.0, 0.0],   # zdot correction
    [0.0, 1.5, 0.0],   # hdot correction
    [0.0, 0.0, 1.5],   # thetadot correction
])

print(np.linalg.eigvals(A-H@C))
