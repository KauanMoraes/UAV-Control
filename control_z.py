import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from config import m, g
from dynamics import drone_dynamics


kiz = 10
kdz = 5

z_d = 1.0
z_dot_d = 0.0
z_ddot_d = 0.0
    
    
def  control_z(t, state):
    z = state[2]
    z_dot = state[5]
    phi = state[6]
    theta = state[7]

    e_z = z_d - z
    e_dot_z = z_dot_d - z_dot
    kh = np.cos(phi) * np.cos(theta)

    u_z = kiz*e_z + kdz*e_dot_z + z_ddot_d

    f_hover = m*g
    f_total = f_hover + u_z*m

    return (np.array([
        f_total,  # poussée totale
        0.0,      # tau_phi
        0.0,      # tau_theta
        0.0       # tau_psi
    ]), u_z)

def state_derivative(t, state):
    control, _ = control_z(t, state)
    return drone_dynamics(t, state, control)

state0 = np.zeros(12)

t_span = (0, 8)
t_eval = np.linspace(0, 8, 1000)

sol = solve_ivp(state_derivative, t_span, state0, t_eval=t_eval)

t = sol.t
z = sol.y[2]
z_dot = sol.y[5]

f_values = []
Uz_values = []

for k in range(len(t)):
    f_k, Uz_k = control_z(t[k], sol.y[:, k])
    f_values.append(f_k)
    Uz_values.append(Uz_k)

f_values = np.array(f_values)
Uz_values = np.array(Uz_values)


plt.figure()
plt.plot(t, z, label="z réel")
plt.plot(t, z_d * np.ones_like(t), "--", label="z désiré")
plt.xlabel("Temps [s]")
plt.ylabel("Altitude z [m]")
plt.title("Contrôle d'altitude z")
plt.grid()
plt.legend()
plt.show()

plt.figure()
plt.plot(t, z_dot)
plt.xlabel("Temps [s]")
plt.ylabel("Vitesse verticale [m/s]")
plt.title("Vitesse verticale")
plt.grid()
plt.show()

plt.figure()
plt.plot(t, f_values)
plt.xlabel("Temps [s]")
plt.ylabel("Poussée f [N]")
plt.title("Commande de poussée")
plt.grid()
plt.show()