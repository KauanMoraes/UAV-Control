import numpy as np


def circular_trajectory(t, R=2.0, omega=0.3):
    x_d = R * np.cos(omega * t)
    y_d = R * np.sin(omega * t)
    z_d = 1.0

    vx_d = -R * omega * np.sin(omega * t)
    vy_d = R * omega * np.cos(omega * t)
    vz_d = 0.0
    # ax_d = - omega**2 * x_d
    # ay_d = - omega**2 * y_d
    # az_d = 0.0
    return x_d, y_d, z_d, vx_d, vy_d, vz_d#, ax_d, ay_d, az_d


def line_trajectory(t):
    x_d = 0.2 * t
    y_d = 1.0
    z_d = 1.0

    vx_d = 0.2
    vy_d = 0.0
    vz_d = 0.0
    ax_d = 0.0
    ay_d = 0.0
    az_d = 0.0
    return x_d, y_d, z_d, vx_d, vy_d, vz_d, ax_d, ay_d, az_d

def z_rampa(t):
    x_d = 1.0
    y_d = 1.0
    vx_d = 0.0
    vy_d = 0.0

    z_d = 1
    vz_d = 0.0
    # ax_d = 0.0
    # ay_d = 0.0
    # az_d = 0.0
    return x_d, y_d, z_d, vx_d, vy_d, vz_d#, ax_d, ay_d, az_d