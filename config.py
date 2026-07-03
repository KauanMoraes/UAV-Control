import numpy as np

g = 9.81

m = 1.5  # drone mass [kg]
d_uav = 0.2
h_uav = 0
J = np.diag([
    0.025,  # Jx
    0.025,  # Jy
    0.045   # Jz
])

J_a = m*4*np.diag([d_uav**2/2, d_uav**2/2, h_uav**2])
J_b = m*4*np.diag([h_uav**2-d_uav**2/2, h_uav**2-d_uav**2/2, -d_uav**2/2])
Jja = J-J_a
Jjb = J-J_b

J_inv = np.linalg.inv(J)