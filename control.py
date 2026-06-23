import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from Trajectory import circular_trajectory, line_trajectory
from config import m, g
from dynamics import drone_dynamics


# Order
x_d = 2.0
y_d = 1.0
z_d = 1.0

# Gains altitude
KP_Z = 8.5
KD_Z = 5

# Gains attitude inner loop
KP_PHI = 10
KD_PHI = 2.0

KP_THETA = 10
KD_THETA = 2.0

KP_PSI = 4.0
KD_PSI = 1.5

KP_X = 3.0
KD_X = 3.1

KP_Y = 3.0
KD_Y = 3.1

# Hypothesis of small angles XY
MAX_ANGLE = np.deg2rad(15)

# References
x_d = 2.0
y_d = 1.0
z_d = 1.0
    
    
def  control_z(state):
    z = state[2]
    z_dot = state[5]
    phi = state[6]
    theta = state[7]

    e_z = z_d - z
    e_dot_z = - z_dot
    kh = np.cos(phi) * np.cos(theta)

    u_z = KP_Z*e_z + KD_Z*e_dot_z #Control law for altitude

    f = m * (g + u_z) / kh

    return f



def xy_controller(state, x_d, y_d, vx_d=0.0, vy_d=0.0):
    x = state[0]
    y = state[1]

    vx = state[3]
    vy = state[4]

    ex = x_d - x
    ey = y_d - y

    evx = vx_d - vx
    evy = vy_d - vy

    ax_des = KP_X * ex + KD_X * evx
    ay_des = KP_Y * ey + KD_Y * evy

    theta_d = ax_des / g
    phi_d = -ay_des / g

    theta_d = np.clip(theta_d, -MAX_ANGLE, MAX_ANGLE)
    phi_d = np.clip(phi_d, -MAX_ANGLE, MAX_ANGLE)

    return phi_d, theta_d

def attitude_controller(state, phi_d, theta_d, psi_d=0.0):
    phi = state[6]
    theta = state[7]
    psi = state[8]

    p = state[9]
    q = state[10]
    r = state[11]
    # Law for attitude control, with a PD controller
    tau_phi = KP_PHI * (phi_d - phi) - KD_PHI * p 
    tau_theta = KP_THETA * (theta_d - theta) - KD_THETA * q
    tau_psi = KP_PSI * (psi_d - psi) - KD_PSI * r

    return tau_phi, tau_theta, tau_psi


def closed_loop_dynamics(t, state, traj_fn=circular_trajectory):
    x_d_t, y_d_t, vx_d_t, vy_d_t = traj_fn(t)
    phi_d, theta_d = xy_controller(
        state,
        x_d_t,
        y_d_t,
        vx_d_t,
        vy_d_t
    )
    f = control_z(state)
    tau_phi, tau_theta, tau_psi = attitude_controller(
        state,
        phi_d,
        theta_d,
        psi_d=0.0
    )

    control = np.array([f, tau_phi, tau_theta, tau_psi])
    return drone_dynamics(t, state, control)

