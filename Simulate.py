import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from control import Controller 
from Trajectory import circular_trajectory, line_trajectory, z_rampa

trajectory = circular_trajectory
dt = 0.005
periodo_controle = 10 # em milissegundos
t_end = 25
cont = 0
controller = Controller(delta_t = dt, trajectory=trajectory)
closed_loop_dynamics, xy_controller= controller.closed_loop_dynamics,controller.xy_controller,
control_z,outer_controller = controller.control_z, controller.outer_controller
DIST_FORCE, DIST_START, DIST_END = controller.DIST_FORCE,controller.DIST_START, controller.DIST_END
state = np.zeros(12)
states_history = []

phi_d_values = []
theta_d_values = []
f_values = []

x_d_values = []
y_d_values = []
z_d_values = []
tau_values = []

l_x = []
l_y = []
l_z = []
l_phi = []
l_theta = []
l_psi = []

t_range = np.arange(0, t_end, controller.delta_t)
for t in t_range:
    states_history.append(state)
    if int(1000*t)%periodo_controle==0:
        controller.calculate_control(state, t)
    controller.update_integrals(state)
    sol = solve_ivp(closed_loop_dynamics, 
                     [t, t + controller.delta_t], state, t_eval=[t + controller.delta_t])
    state = sol.y[:,-1]

    l_x.append(state[0])
    l_y.append(state[1])
    l_z.append(state[2])
    l_phi.append(state[6])
    l_theta.append(state[7])
    l_psi.append(state[8])

    x_d_values.append(controller.x_d)
    y_d_values.append(controller.y_d)
    z_d_values.append(controller.z_d)
    phi_d_values.append(controller.phi_d)
    theta_d_values.append(controller.theta_d)
    f_values.append(controller.control[0])
    tau_values.append(controller.control[1:])


l_x, l_y, l_z = np.array(l_x), np.array(l_y), np.array(l_z)
l_phi, l_theta, l_psi = np.array(l_phi), np.array(l_theta), np.array(l_psi)
x_d_values, y_d_values, z_d_values = np.array(x_d_values), np.array(y_d_values), np.array(z_d_values)
theta_d_values, phi_d_values = np.array(theta_d_values), np.array(phi_d_values)
f_values, tau_values = np.array(f_values), np.array(tau_values)

# ============================================================
# Performance Metrics
# ============================================================
n_ss = int(len(t_range) * 0.2)  # last 20 % for the stationary state

# --- Z : transitory response (0 → z_d) ---
z_d_target = z_d_values[-1]
z_max = np.max(l_z)
z_overshoot = max(0.0, (z_max - z_d_target) / z_d_target * 100) if z_d_target != 0 else 0.0

idx10 = np.where(np.array(l_z) >= 0.1 * z_d_target)[0]
idx90 = np.where(np.array(l_z) >= 0.9 * z_d_target)[0]
tr_z = (t_range[idx90[0]] - t_range[idx10[0]]) if (len(idx10) and len(idx90)) else float('nan')
t_peak_z = t_range[np.argmax(l_z)]

idx_tau = np.where(np.array(l_z) >= 0.632 * z_d_target)[0]
tau_z = t_range[idx_tau[0]] if len(idx_tau) else float('nan')

err_ss_z = abs(z_d_target - np.mean(np.array(l_z)[-n_ss:]))

# --- X, Y : suivi de trajectoire circulaire ---
e_x = x_d_values - np.array(l_x)
e_y = y_d_values - np.array(l_y)
e_xy = np.sqrt(e_x**2 + e_y**2)

err_ss_x = np.mean(np.abs(e_x[-n_ss:]))
err_ss_y = np.mean(np.abs(e_y[-n_ss:]))

e_xy_peak = np.max(e_xy)
SEUIL_XY = 0.2  # m — seuil de convergence XY
idx_conv = np.where(e_xy < SEUIL_XY)[0]
t_conv_xy = t_range[idx_conv[0]] if len(idx_conv) else float('nan')

print("\n========== Performance Metrics ==========")
print(f"\n  [Z]  Altitude   (Target : {z_d_target:.1f} m)")
print(f"       Time Constant (tau)    : {tau_z:.2f} s  (63.2% of z_d)")
print(f"       Rise Time (10%->90%)   : {tr_z:.2f} s  (~2.2*tau)")
print(f"       Overshoot              : {z_overshoot:.1f} %")
print(f"       Steady-State Error     : {err_ss_z*100:.2f} cm")
print(f"\n  [X]  Steady-State Error (mean) : {err_ss_x*100:.2f} cm")
print(f"  [Y]  Steady-State Error (mean) : {err_ss_y*100:.2f} cm")
print(f"\n  [XY] Maximum Tracking Error       : {e_xy_peak:.3f} m")
print(f"  [XY] Convergence Time      : {t_conv_xy:.2f} s  (threshold <= {SEUIL_XY} m)")
print("================================================\n")


fig, axs = plt.subplots(3, 2, figsize=(14, 10))

# ==========================
# 1. Trajectory XY
# ==========================
axs[0, 0].plot(
    np.array(l_x),
    np.array(l_y),
    label="Real Trajectory"
)

axs[0, 0].plot(
    x_d_values,
    y_d_values,
    "--",
    linewidth=2,
    label="Desired Trajectory"
)
axs[0, 0].set_title("Trajectory XY")
axs[0, 0].set_xlabel("x [m]")
axs[0, 0].set_ylabel("y [m]")
axs[0, 0].grid(True)
axs[0, 0].axis("equal")
axs[0, 0].legend()

# ==========================
# 2. Position X et Y
# ==========================
axs[0, 1].plot(t_range,np.array(l_x),label="x real")

axs[0, 1].plot(t_range,x_d_values,"--", label="x desired")

axs[0, 1].plot(
    t_range,
    np.array(l_y),
    label="y real"
)

axs[0, 1].plot(t_range,y_d_values,"--",label="y desired")

axs[0, 1].axvspan(DIST_START, DIST_END, alpha=0.12, color="red", label=f"wind {DIST_FORCE[0]:.0f}N")
axs[0, 1].axvline(DIST_START, color="red", linestyle="--", linewidth=0.9)
axs[0, 1].axvline(DIST_END,   color="red", linestyle="--", linewidth=0.9)
axs[0, 1].set_title("Follow XY")
axs[0, 1].set_xlabel("Time [s]")
axs[0, 1].set_ylabel("Position [m]")
axs[0, 1].grid(True)
axs[0, 1].legend()

# ==========================
# 3. Altitude
# ==========================
axs[1, 0].plot(t_range, np.array(l_z), label="z real")
axs[1, 0].plot(t_range, z_d_values, "--", label=f"z desired ({z_d_target} m)")

axs[1, 0].set_title("Altitude")
axs[1, 0].set_xlabel("Time [s]")
axs[1, 0].set_ylabel("z [m]")
axs[1, 0].grid(True)
axs[1, 0].legend()

# ==========================
# 4. Attitude
# ==========================
axs[1, 1].plot(t_range,np.rad2deg(np.array(l_phi)),label="φ real")

axs[1, 1].plot(t_range,np.rad2deg(phi_d_values),"--",label="φ desired")

axs[1, 1].plot(t_range,np.rad2deg(np.array(l_theta)),label="θ real")

axs[1, 1].plot(t_range,np.rad2deg(theta_d_values),"--",label="θ desired")

axs[1, 1].set_title("Attitude")
axs[1, 1].set_xlabel("Time [s]")
axs[1, 1].set_ylabel("Angle [deg]")
axs[1, 1].grid(True)
axs[1, 1].legend()

# ==========================
# 5. Control Signals
# ==========================
axs[2, 0].plot(t_range, f_values,label = "f")
axs[2,0].plot(t_range, tau_values[:,0], label="M_φ")
axs[2,0].plot(t_range, tau_values[:,1], label="M_θ")
axs[2,0].plot(t_range, tau_values[:,2], label="M_ψ")

axs[2, 0].set_title("Control Signals")
axs[2, 0].set_xlabel("Time [s]")
axs[2, 0].set_ylabel("Force & Torque [N&Nm]")
axs[2, 0].grid(True)
axs[2, 0].legend()
# ==========================
# 5. Error Following
# ==========================
axs[2, 1].plot(t_range, e_x, label="error x")
axs[2, 1].plot(t_range, e_y, label="error y")
axs[2, 1].plot(t_range, e_xy, "k--", linewidth=0.8, label="||e_xy||")

# Disturbance window
axs[2, 1].axvspan(DIST_START, DIST_END, alpha=0.12, color="red")
axs[2, 1].axvline(DIST_START, color="red", linestyle="--", linewidth=0.9)
axs[2, 1].axvline(DIST_END,   color="red", linestyle="--", linewidth=0.9)
axs[2, 1].text(
    (DIST_START + DIST_END) / 2, 0.02,
    f"wind {DIST_FORCE[0]:.0f}N", ha="center", fontsize=8, color="red"
)

axs[2, 1].set_title("Error of follow")
axs[2, 1].set_xlabel("Time [s]")
axs[2, 1].set_ylabel("Error [m]")
axs[2, 1].grid(True)
axs[2, 1].legend()

plt.tight_layout()
plt.show()