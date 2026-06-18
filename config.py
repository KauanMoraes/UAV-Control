import numpy as np

g = 9.81

m = 1.5  # drone mass [kg]

J = np.diag([
    0.025,  # Jx
    0.025,  # Jy
    0.045   # Jz
])

J_inv = np.linalg.inv(J)