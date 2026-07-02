import numpy as np

def rotation_matrix(phi, theta, psi):
    """
    Rotation body Matrix -> inertial
    Euler Angles ZYX : yaw psi, pitch theta, roll phi -> Global: North, East, Down
    """

    cphi, sphi = np.cos(phi), np.sin(phi)
    ctheta, stheta = np.cos(theta), np.sin(theta)
    cpsi, spsi = np.cos(psi), np.sin(psi)

    R = np.array([
        [cpsi*ctheta, cpsi*stheta*sphi - spsi*cphi, cpsi*stheta*cphi + spsi*sphi],
        [spsi*ctheta, spsi*stheta*sphi + cpsi*cphi, spsi*stheta*cphi - cpsi*sphi],
        [-stheta,      ctheta*sphi,                         ctheta*cphi]
    ])

    return R