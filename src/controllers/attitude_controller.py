from src.drone.state import DroneState


class AttitudeController:

    def __init__(
        self,
        kp_phi=8.0,
        kd_phi=3.0,
        kp_theta=8.0,
        kd_theta=3.0,
        kp_psi=4.0,
        kd_psi=1.5
    ):

        self.kp_phi = kp_phi
        self.kd_phi = kd_phi

        self.kp_theta = kp_theta
        self.kd_theta = kd_theta

        self.kp_psi = kp_psi
        self.kd_psi = kd_psi

    def compute(
        self,
        state: DroneState,
        phi_ref,
        theta_ref,
        psi_ref=0.0
    ):

        phi, theta, psi = (
            state.attitude
        )

        p, q, r = (
            state.angular_velocity
        )

        tau_phi = (
            self.kp_phi
            * (phi_ref - phi)
            - self.kd_phi * p
        )

        tau_theta = (
            self.kp_theta
            * (theta_ref - theta)
            - self.kd_theta * q
        )

        tau_psi = (
            self.kp_psi
            * (psi_ref - psi)
            - self.kd_psi * r
        )

        return (
            tau_phi,
            tau_theta,
            tau_psi
        )