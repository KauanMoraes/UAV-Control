import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from Trajectory import circular_trajectory, line_trajectory
from config import m, g, Jja,Jjb
from dynamics import drone_dynamics

class Controller:
    def __init__(self,trajectory = line_trajectory):
        self.load_parameters(trajectory)

    def load_parameters(self,trajectory):
        self.traj_fn = trajectory
        self.m = m
        self.g = g
        self.Jja = Jja
        self.Jjb = Jjb

        # Gains altitude
        self.KP_Z = 8.5
        self.KD_Z = 5

        # Gains attitude inner loop
        self.KP_PSI = 4.0
        self.KD_PSI = 1.5

        self.KP_PHI = 10
        self.KD_PHI = 5.0

        self.KP_THETA = 10
        self.KD_THETA = 5.0

        self.KP_X = 1.4  # 2.0
        self.KD_X = 2.1

        self.KP_Y = 1.4  # 2.0
        self.KD_Y = 2.1

        # Hypothesis of small angles XY
        self.MAX_ANGLE = np.deg2rad(15)

        # Disturbance — wind step force in inertial frame [N]
        self.DIST_FORCE = np.array([3.0, 0.0, 0.0])  # 3N in X
        self.DIST_START = 10.0                          # onset time [s]
        self.DIST_END   = 30.0                         # end time [s]

        # References
        self.x_d = 0.0
        self.y_d = 0.0
        self.z_d = 0.0
    
    
    def control_z(self,state,z_d,vz_d):
        z = state[2]
        z_dot = state[5]
        phi = state[6]
        theta = state[7]
        self.z_d = self.traj_fn

        e_z = z_d - z
        e_dot_z = vz_d - z_dot
        kh = np.cos(phi) * np.cos(theta)

        u_z = self.KP_Z*e_z + self.KD_Z*e_dot_z #Control law for altitude

        f = self.m * (self.g + u_z) / kh

        return f



    def xy_controller(self,state, x_d, y_d, vx_d, vy_d, ax_d=0.0, ay_d=0.0):
        R_psi = np.array([[np.cos(state[8]),-np.sin(state[8])],
                        [np.sin(state[8]),np.cos(state[8])]])
        R_psi_inv = np.linalg.inv(R_psi)
        x = state[0]
        y = state[1]

        vx = state[3]
        vy = state[4]

        ex = x_d - x
        ey = y_d - y

        evx = vx_d - vx
        evy = vy_d - vy

        U_x = self.KP_X * ex + self.KD_X * evx + ax_d
        U_y = self.KP_Y * ey + self.KD_Y * evy + ay_d
        Uxy = np.array([U_x,U_y])

        arr_thphi = R_psi_inv@ Uxy *self.m/self.f   # array [sin(theta_d)cos(phi_d), -sin(phi_d)]
        if np.abs(arr_thphi[1])>1:
            print(f'-sen(phi_d):{arr_thphi[1]}')
        phi_d = -np.arcsin(np.clip(arr_thphi[1],-1,1))
        
        if np.abs(arr_thphi[0]/np.cos(phi_d))>1:
            print(f'sin(theta_d)cos(phi_d):{arr_thphi[0]/np.cos(phi_d)}')
        theta_d = np.arcsin(np.clip(arr_thphi[0]/np.cos(phi_d),-1,1))

        theta_d = np.clip(theta_d, -self.MAX_ANGLE, self.MAX_ANGLE)
        phi_d = np.clip(phi_d, -self.MAX_ANGLE, self.MAX_ANGLE)

        return phi_d, theta_d

    def attitude_controller(self, state, phi_d, theta_d, psi_d=0.0, vpsi_d=0.0):
        phi = state[6]
        theta = state[7]
        psi = state[8]
        # Ômega: [p,q,r] (aprox [phi_dot, theta_dot, psi_dot] in low theta,phi)
        p = state[9]
        q = state[10]
        r = state[11]
        # Law for attitude control, with a PD controller
        U_phi = self.KP_PHI * (phi_d - phi) - self.KD_PHI * p 
        U_theta = self.KP_THETA * (theta_d - theta) - self.KD_THETA * q
        U_psi = self.KP_PSI * (psi_d - psi) - self.KD_PSI * (r-vpsi_d)

        tau_phi = q*r*(Jja[2,2]-Jja[1,1]) + U_phi*Jjb[0,0]
        tau_theta = p*r*(Jja[0,0]-Jja[2,2]) + U_theta*Jjb[1,1]
        tau_psi = p*q*(Jja[1,1]-Jja[0,0]) + U_psi*Jjb[2,2]

        return tau_phi, tau_theta, tau_psi

    def outer_controller(self,state,x_d,y_d,z_d):
        KPO = 1 # outer controller constant
        vx_d = KPO * (x_d-state[0])
        vy_d = KPO * (y_d-state[1])
        vz_d = KPO * (z_d-state[2])
        return vx_d,vy_d,vz_d


    def closed_loop_dynamics(self, t, state):
        self.x_d_t, self.y_d_t, self.z_d_t, self.vx_d_t, self.vy_d_t, self.vz_d_t = self.traj_fn(t)
        # self.vx_d_t, self.vy_d_t, self.vz_d_t = self.outer_controller(state,self.x_d_t,
        #                                                               self.y_d_t,self.z_d_t)
        self.f = self.control_z(state,self.z_d_t,self.vz_d_t)
        phi_d, theta_d = self.xy_controller(
            state,
            self.x_d_t,
            self.y_d_t,
            self.vx_d_t,
            self.vy_d_t
        )
        tau_phi, tau_theta, tau_psi = self.attitude_controller(
            state,
            phi_d,
            theta_d,
            psi_d=0.0,
            vpsi_d=0.0
        )

        control = np.array([self.f, tau_phi, tau_theta, tau_psi])
        state_dot = drone_dynamics(t, state, control)

        # Apply wind disturbance as external force on velocity states
        if self.DIST_START <= t <= self.DIST_END:
            state_dot[3:6] += self.DIST_FORCE / self.m

        return state_dot

