import numpy as np


def circular_trajectory(t, R=2.0, omega=0.3):
    x_d = R * np.cos(omega * t)
    y_d = R * np.sin(omega * t)

    vx_d = -R * omega * np.sin(omega * t)
    vy_d = R * omega * np.cos(omega * t)

    return x_d, y_d, vx_d, vy_d


def line_trajectory(t):
    x_d = 0.2 * t
    y_d = 1.0

    vx_d = 0.2
    vy_d = 0.0

    return x_d, y_d, vx_d, vy_d