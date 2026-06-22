import numpy as np
from src.drone.state import DroneState


class PositionController:

    def __init__(
        self,
        gravity: float,
        kp: float = 1.2,
        kd: float = 2.0,
        max_angle_deg: float = 15
    ):

        self.gravity = gravity
        self.kp = kp
        self.kd = kd

        self.max_angle = np.deg2rad(max_angle_deg)

    def compute(
        self,
        state: DroneState,
        x_ref,
        y_ref,
        vx_ref=0.0,
        vy_ref=0.0
    ):

        x = state.position[0]
        y = state.position[1]

        vx = state.velocity[0]
        vy = state.velocity[1]

        ex = x_ref - x
        ey = y_ref - y

        evx = vx_ref - vx
        evy = vy_ref - vy

        ax_des = (self.kp * ex + self.kd * evx)
        ay_des = (self.kp * ey + self.kd * evy)
        
        theta_ref = (ax_des / self.gravity)
        phi_ref = (-ay_des / self.gravity)

        theta_ref = np.clip(theta_ref, -self.max_angle, self.max_angle)
        phi_ref = np.clip(phi_ref, -self.max_angle, self.max_angle)

        return phi_ref, theta_ref