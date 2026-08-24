import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from Trajectory import circular_trajectory, line_trajectory
from config import m, g, Jja,Jjb
from dynamics import drone_dynamics

class Controller:
    def __init__(self, delta_t, trajectory = line_trajectory):
        self.delta_t = delta_t
        self.load_parameters(trajectory)

    def load_parameters(self,trajectory):
        self.traj_fn = trajectory
        self.m = m
        self.g = g
        self.Jja = Jja
        self.Jjb = Jjb

        # Gains altitude
        self.KP_Z = 4.0
        self.KD_Z = 3.0

        # Gains attitude inner loop
        self.KP_PSI = 40.0
        self.KD_PSI = 12.0

        self.KP_PHI = 100.0
        self.KD_PHI = 20.0

        self.KP_THETA = 100.0
        self.KD_THETA = 20.0

        self.KP_X = 8.0
        self.KD_X = 5.0

        self.KP_Y = 8.0
        self.KD_Y = 5.0

        # Hypothesis of small angles XY
        self.MAX_ANGLE = np.deg2rad(15)

        # Disturbance — wind step force in inertial frame [N]
        self.DIST_FORCE = np.array([2.5, 0.0, 0.0])  # 3N in X  o controle theta satura
        self.DIST_START = 10.0                          # onset time [s]
        self.DIST_END   = 20.0                         # end time [s]

        # References
        self.x_d = 0.0
        self.y_d = 0.0
        self.z_d = 0.0

        self.intgr_vx = 0.0
        self.intgr_vy = 0.0
        self.intgr_vz = 0.0

        self.ax_d = 0
        self.ay_d = 0
        self.az_d = 0
        self.dot_x_d = 0
        self.dot_y_d = 0
        self.dot_z_d = 0

    def control_z(self,state,vz_d, az_d = 0.0):
        z = state[2]
        z_dot = state[5]
        phi = state[6]
        theta = state[7]

        #e_z = z - z_d
        e_dot_z =  z_dot - vz_d
        kh = np.cos(phi) * np.cos(theta)

        intgr_vz = state[14] if len(state) >= 15 else self.intgr_vz
        u_z = self.KP_Z*intgr_vz - self.KD_Z*e_dot_z + az_d #Control law for altitude

        f = self.m * (self.g + u_z) / kh

        return f



    def xy_controller(self,state, f, vx_d, vy_d, ax_d=0.0, ay_d=0.0):
        R_psi = np.array([[np.cos(state[8]),-np.sin(state[8])],
                        [np.sin(state[8]),np.cos(state[8])]])
        R_psi_inv = np.linalg.inv(R_psi)
        # x = state[0]
        # y = state[1]

        vx = state[3]
        vy = state[4]

        # ex = x - x_d
        # ey = y - y_d

        evx = vx - vx_d
        evy = vy - vy_d

        intgr_vx = state[12] if len(state) >= 15 else self.intgr_vx
        intgr_vy = state[13] if len(state) >= 15 else self.intgr_vy

        U_x = self.KP_X * intgr_vx - self.KD_X * evx + ax_d
        U_y = self.KP_Y * intgr_vy - self.KD_Y * evy + ay_d
        Uxy = np.array([U_x,U_y])
        safe_f = np.maximum(f, 0.5 * self.m * self.g)
        arr_thphi = R_psi_inv@ Uxy *self.m/safe_f   # array [sin(theta_d)cos(phi_d), -sin(phi_d)]
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
        self.KPO = 1.5 # outer controller constant
        KPO  = self.KPO
        vx_d = KPO * (x_d-state[0])+self.dot_x_d
        vy_d = KPO * (y_d-state[1])+self.dot_y_d
        vz_d = KPO * (z_d-state[2])+self.dot_z_d
        return vx_d,vy_d,vz_d


    def closed_loop_dynamics(self, t, state):
        self.x_d_t, self.y_d_t, self.z_d_t = self.traj_fn(t)
        self.vx_d_t, self.vy_d_t, self.vz_d_t = self.outer_controller(state,
                                                self.x_d_t, self.y_d_t, self.z_d_t)
        # Como o solver Runge-Kutta do solve_ivp avalia vários sub-passos no tempo,
        # salvar um estado anterior (old_vx_d_t) diretamente aqui dentro não funciona 
        # A derivada exata de vx_d = KPO * (x_d - x) é ax_d = KPO * (dot_x_d - v_x).
        dt_sim = self.delta_t/3
        x_d_old, y_d_old, z_d_old = self.traj_fn(t - dt_sim) if t>0 else self.traj_fn(t)
        self.dot_x_d = (self.x_d_t - x_d_old) / dt_sim
        self.dot_y_d = (self.y_d_t - y_d_old) / dt_sim
        self.dot_z_d = (self.z_d_t - z_d_old) / dt_sim
        KPO  = self.KPO
        self.ax_d = KPO * (self.dot_x_d - state[3])
        self.ay_d = KPO * (self.dot_y_d - state[4])
        self.az_d = KPO * (self.dot_z_d - state[5])
        
        if len(state) >= 15:
            self.intgr_vx = state[12]
            self.intgr_vy = state[13]
            self.intgr_vz = state[14]

        self.f = self.control_z(state,self.vz_d_t,self.az_d)

        # o qnt o termo da integral deve mudar
        d_intgr_vx = self.vx_d_t - state[3]
        d_intgr_vy = self.vy_d_t - state[4]
        d_intgr_vz = self.vz_d_t - state[5]

        # ANTI-WINDUP: Prevent integral terms from building up when saturated
        MAX_INT_XY = 4.0 / self.KP_X
        if abs(self.intgr_vx) > MAX_INT_XY and np.sign(d_intgr_vx) == np.sign(self.intgr_vx):
            d_intgr_vx = 0
        if abs(self.intgr_vy) > MAX_INT_XY and np.sign(d_intgr_vy) == np.sign(self.intgr_vy):
            d_intgr_vy = 0
            
        MAX_INT_Z = 15.0 / self.KP_Z
        if abs(self.intgr_vz) > MAX_INT_Z and np.sign(d_intgr_vz) == np.sign(self.intgr_vz):
            d_intgr_vz = 0

        phi_d, theta_d = self.xy_controller(
            state,
            self.f,
            self.vx_d_t,
            self.vy_d_t,
            self.ax_d,
            self.ay_d
        )
        tau_phi, tau_theta, tau_psi = self.attitude_controller(
            state,
            phi_d,
            theta_d,
            psi_d=0.0,
            vpsi_d=0.0
        )

        control = np.array([self.f, tau_phi, tau_theta, tau_psi])
        state_dot = drone_dynamics(t, state[:12], control)

        # Apply wind disturbance as external force on velocity states
        if self.DIST_START <= t <= self.DIST_END:
            state_dot[3:6] += self.DIST_FORCE / self.m

        if len(state) >= 15:
            return np.concatenate((state_dot, [d_intgr_vx, d_intgr_vy, d_intgr_vz]))
        else:
            self.intgr_vx += self.delta_t * d_intgr_vx
            self.intgr_vy += self.delta_t * d_intgr_vy
            self.intgr_vz += self.delta_t * d_intgr_vz
            return state_dot

