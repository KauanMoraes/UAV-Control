class Control:
    def __init__(self, m, g):
        self.m = m
        self.g = g

    def control_z(self, state):
        z = state[2]
        z_dot = state[5]
        phi = state[6]
        theta = state[7]

        e_z = z_d - z
        e_dot_z = - z_dot
        kh = np.cos(phi) * np.cos(theta)

        u_z = KP_Z*e_z + KD_Z*e_dot_z #Control law for altitude

        f = self.m * (self.g + u_z) / kh

        return f

    def xy_controller(self, state, x_d, y_d, vx_d=0.0, vy_d=0.0):
        x = state[0]
        y = state[1]

        vx = state[3]
        vy = state[4]