import numpy as np


def circular_trajectory(t, R=2.0, omega=0.3):
    x_d = R * np.cos(omega * t)
    y_d = R * np.sin(omega * t)
    z_d = 1.0

    return x_d, y_d, z_d

def line_trajectory(t):
    x_d = 0.2 * t
    y_d = 1.0
    z_d = 1.0

    return x_d, y_d, z_d

def z_rampa(t):
    x_d = 0.0
    y_d = 0.1*t
    z_d = 1.0*t
    return x_d, y_d, z_d
