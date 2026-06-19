# simulation/simulation.py

import numpy as np

from scipy.integrate import solve_ivp

from simulation.result import SimulationResult

from drone.state import DroneState


class Simulation:

    def __init__(
        self,
        drone,
        trajectory,
        position_controller,
        attitude_controller,
        altitude_controller,
        t_final: float = 25.0,
        num_points: int = 5000
    ):

        self.drone = drone

        self.trajectory = trajectory

        self.position_controller = position_controller

        self.attitude_controller = attitude_controller

        self.altitude_controller = altitude_controller

        self.t_final = t_final

        self.num_points = num_points

        self.control_history = []

    def closed_loop_dynamics(
        self,
        t: float,
        state_vector: np.ndarray
    ) -> np.ndarray:

        state = DroneState.from_vector(
            state_vector
        )

        reference = self.trajectory.reference(
            t
        )

        phi_ref, theta_ref = (
            self.position_controller.compute(
                state,
                reference.x,
                reference.y,
                reference.vx,
                reference.vy
            )
        )

        thrust = (
            self.altitude_controller.compute(
                state,
                reference.z
            )
        )

        tau_phi, tau_theta, tau_psi = (
            self.attitude_controller.compute(
                state,
                phi_ref,
                theta_ref,
                reference.yaw
            )
        )

        control = np.array([
            thrust,
            tau_phi,
            tau_theta,
            tau_psi
        ])

        self.control_history.append(
            control.copy()
        )

        return self.drone.dynamics.derivatives(
            t,
            state_vector,
            control
        )

    def run(self) -> SimulationResult:

        self.control_history = []

        state0 = (
            self.drone.state.to_vector()
        )

        t_eval = np.linspace(
            0,
            self.t_final,
            self.num_points
        )

        solution = solve_ivp(
            self.closed_loop_dynamics,
            (0.0, self.t_final),
            state0,
            t_eval=t_eval
        )

        controls = np.array(
            self.control_history
        )

        return SimulationResult(
            time=solution.t,
            states=solution.y,
            controls=controls
        )