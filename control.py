import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from Trajectoire import circular_trajectory
from config import m, g
from dynamics import drone_dynamics


# Order
x_d = 2.0
y_d = 1.0
z_d = 1.0

# Gains altitude
KP_Z = 6.0
KD_Z = 4.0

# Gains attitude inner loop
KP_PHI = 8.0
KD_PHI = 3.0

KP_THETA = 8.0
KD_THETA = 3.0

KP_PSI = 4.0
KD_PSI = 1.5

KP_X = 1.2
KD_X = 2

KP_Y = 1.2
KD_Y = 2

# Hypothèse small angles XY
MAX_ANGLE = np.deg2rad(15)

# Consignes
x_d = 2.0
y_d = 1.0
z_d = 1.0
    
    
def  control_z(state):
    z = state[2]
    z_dot = state[5]
    phi = state[6]
    theta = state[7]

    e_z = z_d - z
    e_dot_z = - z_dot
    kh = np.cos(phi) * np.cos(theta)

    u_z = KP_Z*e_z + KD_Z*e_dot_z #Control law for altitude

    f = m * (g + u_z) / kh

    return f



def xy_controller(state, x_d, y_d, vx_d=0.0, vy_d=0.0):
    x = state[0]
    y = state[1]

    vx = state[3]
    vy = state[4]

    ex = x_d - x
    ey = y_d - y

    evx = vx_d - vx
    evy = vy_d - vy

    ax_des = KP_X * ex + KD_X * evx
    ay_des = KP_Y * ey + KD_Y * evy

    theta_d = ax_des / g
    phi_d = -ay_des / g

    theta_d = np.clip(theta_d, -MAX_ANGLE, MAX_ANGLE)
    phi_d = np.clip(phi_d, -MAX_ANGLE, MAX_ANGLE)

    return phi_d, theta_d

def attitude_controller(state, phi_d, theta_d, psi_d=0.0):
    phi = state[6]
    theta = state[7]
    psi = state[8]

    p = state[9]
    q = state[10]
    r = state[11]
    # Law for attitude control, with a PD controller
    tau_phi = KP_PHI * (phi_d - phi) - KD_PHI * p 
    tau_theta = KP_THETA * (theta_d - theta) - KD_THETA * q
    tau_psi = KP_PSI * (psi_d - psi) - KD_PSI * r

    return tau_phi, tau_theta, tau_psi


def closed_loop_dynamics(t, state):
    x_d_t, y_d_t, vx_d_t, vy_d_t = circular_trajectory(t)
    phi_d, theta_d = xy_controller(
        state,
        x_d_t,
        y_d_t,
        vx_d_t,
        vy_d_t
    )
    f = control_z(state)
    tau_phi, tau_theta, tau_psi = attitude_controller(
        state,
        phi_d,
        theta_d,
        psi_d=0.0
    )

    control = np.array([
        f,
        tau_phi,
        tau_theta,
        tau_psi
    ])
    return drone_dynamics(t, state, control)

state0 = np.zeros(12)

t_span = (0, 25)
t_eval = np.linspace(0, 25, 5000)

sol = solve_ivp(closed_loop_dynamics, t_span, state0, t_eval=t_eval)

t = sol.t

x = sol.y[0]
y = sol.y[1]
z = sol.y[2]

phi = sol.y[6]
theta = sol.y[7]
psi = sol.y[8]

phi_d_values = []
theta_d_values = []
f_values = []

x_d_values = []
y_d_values = []

for k in range(len(t)):

    state_k = sol.y[:, k]

    # trajectoire désirée à l'instant t[k]
    x_d_k, y_d_k, vx_d_k, vy_d_k = circular_trajectory(t[k])

    phi_d_k, theta_d_k = xy_controller(
        state_k,
        x_d_k,
        y_d_k,
        vx_d_k,
        vy_d_k
    )

    f_k = control_z(state_k)

    x_d_values.append(x_d_k)
    y_d_values.append(y_d_k)

    phi_d_values.append(phi_d_k)
    theta_d_values.append(theta_d_k)
    f_values.append(f_k)

x_d_values = np.array(x_d_values)
y_d_values = np.array(y_d_values)

phi_d_values = np.array(phi_d_values)
theta_d_values = np.array(theta_d_values)
f_values = np.array(f_values)


fig, axs = plt.subplots(3, 2, figsize=(14, 10))

# ==========================
# 1. Trajectoire XY
# ==========================
axs[0, 0].plot(
    x,
    y,
    label="Trajectoire réelle"
)

axs[0, 0].plot(
    x_d_values,
    y_d_values,
    "--",
    linewidth=2,
    label="Trajectoire désirée"
)
axs[0, 0].set_title("Trajectoire XY")
axs[0, 0].set_xlabel("x [m]")
axs[0, 0].set_ylabel("y [m]")
axs[0, 0].grid(True)
axs[0, 0].axis("equal")
axs[0, 0].legend()

# ==========================
# 2. Position X et Y
# ==========================
axs[0, 1].plot(
    t,
    x,
    label="x réel"
)

axs[0, 1].plot(
    t,
    x_d_values,
    "--",
    label="x désiré"
)

axs[0, 1].plot(
    t,
    y,
    label="y réel"
)

axs[0, 1].plot(
    t,
    y_d_values,
    "--",
    label="y désiré"
)

axs[0, 1].set_title("Suivi XY")
axs[0, 1].set_xlabel("Temps [s]")
axs[0, 1].set_ylabel("Position [m]")
axs[0, 1].grid(True)
axs[0, 1].legend()

# ==========================
# 3. Altitude
# ==========================
axs[1, 0].plot(t, z, label="z réel")
axs[1, 0].plot(t, z_d*np.ones_like(t), "--", label="z désiré")

axs[1, 0].set_title("Altitude")
axs[1, 0].set_xlabel("Temps [s]")
axs[1, 0].set_ylabel("z [m]")
axs[1, 0].grid(True)
axs[1, 0].legend()

# ==========================
# 4. Attitude
# ==========================
axs[1, 1].plot(
    t,
    np.rad2deg(phi),
    label="φ réel"
)

axs[1, 1].plot(
    t,
    np.rad2deg(phi_d_values),
    "--",
    label="φ désiré"
)

axs[1, 1].plot(
    t,
    np.rad2deg(theta),
    label="θ réel"
)

axs[1, 1].plot(
    t,
    np.rad2deg(theta_d_values),
    "--",
    label="θ désiré"
)

axs[1, 1].set_title("Attitude")
axs[1, 1].set_xlabel("Temps [s]")
axs[1, 1].set_ylabel("Angle [deg]")
axs[1, 1].grid(True)
axs[1, 1].legend()

# ==========================
# 5. Poussée
# ==========================
axs[2, 0].plot(t, f_values)

axs[2, 0].set_title("Commande de poussée")
axs[2, 0].set_xlabel("Temps [s]")
axs[2, 0].set_ylabel("Force [N]")
axs[2, 0].grid(True)

# ==========================
# 5. Error Following
# ==========================
ex = x_d_values - x
ey = y_d_values - y

axs[2, 1].plot(t, ex, label="erreur x")
axs[2, 1].plot(t, ey, label="erreur y")

axs[2, 1].set_title("Erreur de suivi")
axs[2, 1].set_xlabel("Temps [s]")
axs[2, 1].set_ylabel("Erreur [m]")
axs[2, 1].grid(True)
axs[2, 1].legend()

plt.tight_layout()
plt.show()