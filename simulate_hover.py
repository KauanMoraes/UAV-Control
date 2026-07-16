import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from config import m, g
from dynamics import drone_dynamics
from control import Controller
from Trajectory import line_trajectory

traj = line_trajectory
controller = Controller(trajectory=traj)
# Initial state
state0 = np.zeros(12)

# Initial position
state0[2] = 0.0  # z = 1 m

f_hover = 1.5*9.81

# control = np.array([
#     f_hover,  # total thrust
#     0.0,      # tau_phi
#     0.0,      # tau_theta
#     0.0       # tau_psi
# ])
control = controller.control_z

def state_dot(t, state):
    return drone_dynamics(t, state, control)


t_span = (0, 5)
t_eval = np.linspace(0, 5, 1000)

sol = solve_ivp(state_dot, t_span, state0, t_eval=t_eval)

t = sol.t
x = sol.y[0]
y = sol.y[1]
z = sol.y[2]

plt.figure()
plt.plot(t, z)
plt.xlabel("Time [s]")
plt.ylabel("Altitude z [m]")
plt.title("Simulation hover")
plt.grid()
plt.show()
