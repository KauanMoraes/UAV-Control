import numpy as np # type: ignore
from config import m, g, J, J_inv
from rotation import rotation_matrix

def euler_rates(phi, theta, omega):
    """
    Converts the body angular velocities omega = [p, q, r] 
    to Euler angle derivatives [phi_dot, theta_dot, psi_dot].
    """

    p, q, r = omega

    cphi, sphi = np.cos(phi), np.sin(phi)
    ctheta, ttheta = np.cos(theta), np.tan(theta)

    E = np.array([
        [1, sphi*ttheta, cphi*ttheta],
        [0, cphi,       -sphi],
        [0, sphi/ctheta, cphi/ctheta]
    ])

    return E @ omega


def drone_dynamics(t, state, control):
    """
    state =
    [x, y, z,
     vx, vy, vz,
     phi, theta, psi,
     p, q, r]

    control =
    [f, tau_phi, tau_theta, tau_psi]
    """

    x, y, z = state[0:3]
    v = state[3:6]

    phi, theta, psi = state[6:9]
    omega = state[9:12]

    f = control[0]
    tau = np.array(control[1:4])

    R = rotation_matrix(phi, theta, psi)

    gravity = np.array([0, 0, -g])
    thrust_body = np.array([0, 0, f])
    thrust_inertial = R @ thrust_body

    # Translation
    v_dot = gravity + thrust_inertial / m

    # Rotation Newton-Euler
    omega_dot = J_inv @ (tau - np.cross(omega, J @ omega))

    # Cinematics
    p_dot = v
    angles_dot = euler_rates(phi, theta, omega)

    state_dot = np.zeros(12)
    state_dot[0:3] = p_dot
    state_dot[3:6] = v_dot
    state_dot[6:9] = angles_dot
    state_dot[9:12] = omega_dot

    return state_dot