import numpy as np
import VTOLParam as P
class Observer():
    def __init__(self):
        self.state = np.array([
            [P.z0],  # initial base angle
            [P.h0],  # initial panel angle
            [P.theta0],  # initial angular velocity of base
            [P.zdot0],
            [P.hdot0],
            [P.thetadot0]  # initial angular velocity of panel
        ])
        self.R = np.diag([0.5**2,0.2**2,0.017**2])
        self.Q = np.diag([0.03**2,0.05**2])
        self.Q_process = np.diag([1e-8, 1e-8, 1e-9, 1e-7, 1e-7, 1e-8])
        self.C = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        ])
        self.nlo = NLO(self)

        alpha = 0.2
        self.mr = P.mr * (1.+alpha*(2.*np.random.rand()-1.))
        # mass of center
        self.mc = P.mc * (1.+alpha*(2.*np.random.rand()-1.))
        # distance from center to engine
        self.d = P.d * (1.+alpha*(2.*np.random.rand()-1.))
        # mew drag
        self.mu = P.mu * (1.+alpha*(2.*np.random.rand()-1.))  
        # gravity
        self.g = P.g
        # intertia of the center
        self.Jc = P.Jc * (1.+alpha*(2.*np.random.rand()-1.))

        self.mew = self.state
        # self.sigma = np.eye(self.mew.shape[0])
        self.sigma = np.diag([4.0, 4.0, 2.0, 4.0, 4.0, 2.0])

    def f(self, state, u, d = 0.0):
        # Return xdot = f(x,u)
        z = state[0][0]
        h = state[1][0]
        theta = state[2][0]
        zdot = state[3][0]
        hdot = state[4][0]
        thetadot = state[5][0]

        fr = u[0][0]
        fl = u[1][0]
        # The equations of motion.
        M = np.array([[self.mc + 2*self.mr,0,0],
                    [0,self.mc + 2*self.mr,0],
                    [0,0,self.Jc + 2*self.mr*self.d**2]])
        C = np.array([[-1*(fr+fl)*np.sin(theta) - self.mu * zdot + d],
                    [-1*(self.mc + 2*self.mr)*self.g + (fr+fl)*np.cos(theta)],
                    [self.d*(fr - fl)]])

        tmp = np.linalg.inv(M) @ C
        zddot = tmp[0][0]
        hddot = tmp[1][0]
        thetaddot = tmp[2][0]

        # build xdot and return
        xdot = np.array([[zdot], [hdot], [thetadot], [zddot], [hddot], [thetaddot]])
        return xdot

    def linearization(self,u,state,dt):
        z     = state[0][0]
        h     = state[1][0]
        theta = state[2][0]
        zdot  = state[3][0]
        hdot  = state[4][0]
        thetadot = state[5][0]

        fr = u[0,0]
        fl = u[1,0]

        M    = self.mc + 2.0*self.mr
        Jbar = self.Jc + 2.0*self.mr*self.d**2

        s = np.sin(theta)
        c = np.cos(theta)

        # Jacobian of the state with respect to the state
        F = np.array([
            [0.0, 0.0, 0.0, 1.0,      0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0,      1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0,      0.0, 1.0],
            [0.0, 0.0, -(fr+fl)*c/M, -self.mu/M, 0.0, 0.0],
            [0.0, 0.0, -(fr+fl)*s/M,  0.0,      0.0, 0.0],
            [0.0, 0.0, 0.0,           0.0,      0.0, 0.0],
        ])

        # Jacobian of the state with respect to the motion
        G = np.array([
            [0.0,         0.0        ],
            [0.0,         0.0        ],
            [0.0,         0.0        ],
            [-s/M,        -s/M       ],
            [ c/M,         c/M       ],
            [ self.d/Jbar, -self.d/Jbar],
        ])

        I = np.eye(F.shape[0]) 

        return I+dt*F,G*dt

    def h(self,state):
        # Measurement is linear so h(x) is just Cx
        z_pred = self.C @ state
        H = self.C
        return z_pred,H
    
    def minimizedAngle(self,theta):
        return (theta + 3*np.pi)%(2*np.pi)-np.pi
        
    def update(self,prev_x,measurement,u_prev,dt):
        # First we need to update the NLO to get the stable state measurement
        nlo_updated_state = self.nlo.update(u_prev,measurement,dt)
        # Now linearize about the NLO's state. Return the Jacobian linearization for the state with respect to the motion (G), and one with respect to the previous state (F)
        F,G = self.linearization(u_prev,nlo_updated_state,dt)
        # Now calculate mu_bar and sigma_bar
        mu_bar = nlo_updated_state
        # sigma_bar = F @ self.sigma @ F.T + (G @ self.Q @ G.T)
        sigma_bar = F @ self.sigma @ F.T #+self.Q_process
        # Now lets do the innovation step
        z_pred,H = self.h(nlo_updated_state)
        y = measurement - z_pred
        S = H @ sigma_bar @ H.T + self.R
        # Calculate the Kalman gain
        K = sigma_bar @ H.T @ np.linalg.inv(S)
        # Now the correction step
        self.mew = mu_bar + K @ y
        self.mew[2] = self.minimizedAngle(self.mew[2])
        I = np.eye(len(self.sigma))
        self.sigma = (I - (K @ H)) @ sigma_bar


        
        return self.mew,nlo_updated_state
        
        

class NLO:
    def __init__(self,parent):
        # self.state = np.array([0,0,0]).reshape(3,1)
        self.parent = parent
        self.state = parent.state
        self.H = np.array([
            [1.0, 0.0, 0.0],   # z correction
            [0.0, 1.0, 0.0],   # h correction
            [0.0, 0.0, 1.0],   # theta correction
            [1.5, 0.0, 0.0],   # zdot correction
            [0.0, 1.5, 0.0],   # hdot correction
            [0.0, 0.0, 1.5],   # thetadot correction
        ])

    def update(self,u,measurement,dt):
        # Reshape the measurement to a column vector
        measurement = measurement.reshape(3,1)
        # Innovvation of the NLO        
        innovation = measurement - self.parent.C @ self.state.reshape(6,1)
        # Update the NLO and reutrn
        self.state = self.state + dt * (self.parent.f(self.state,u) + self.H @ innovation)
        self.state[2] = self.parent.minimizedAngle(self.state[2])
        return self.state

