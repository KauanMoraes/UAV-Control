import numpy as np

from src.drone.parameters import DroneParameters
from src.drone.state import DroneState

from src.math.rotation import rotation_matrix


class RigidBodyDynamics:
    """
    6DOF rigid body dynamics based on
    Newton-Euler equations.
    """

    def __init__(
        self,
        parameters: DroneParameters
    ):
        self.parameters = parameters

    def euler_rates(
        self,
        phi,
        theta,
        omega
    ):

        p, q, r = omega

        cphi = np.cos(phi)
        sphi = np.sin(phi)

        ctheta = np.cos(theta)
        ttheta = np.tan(theta)

        E = np.array([
            [1, sphi * ttheta, cphi * ttheta],
            [0, cphi, -sphi],
            [0, sphi / ctheta, cphi / ctheta]
        ])

        return E @ omega

    def derivatives(
        self,
        t,
        state_vector,
        control
    ):

        state = DroneState.from_vector(
            state_vector
        )

        phi, theta, psi = state.attitude

        omega = state.angular_velocity

        thrust = control[0]

        tau = np.array(
            control[1:4]
        )

        R = rotation_matrix(
            phi,
            theta,
            psi
        )

        gravity = np.array([
            0,
            0,
            -self.parameters.gravity
        ])

        thrust_body = np.array([
            0,
            0,
            thrust
        ])

        thrust_inertial = (
            R @ thrust_body
        )

        velocity_dot = (
            gravity
            + thrust_inertial
            / self.parameters.mass
        )

        omega_dot = (
            self.parameters.inertia_inv
            @ (
                tau
                - np.cross(
                    omega,
                    self.parameters.inertia
                    @ omega
                )
            )
        )

        position_dot = (
            state.velocity
        )

        attitude_dot = (
            self.euler_rates(
                phi,
                theta,
                omega
            )
        )

        derivative = DroneState(
            position=position_dot,
            velocity=velocity_dot,
            attitude=attitude_dot,
            angular_velocity=omega_dot
        )

        return derivative.to_vector()