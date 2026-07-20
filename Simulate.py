import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from control import Controller 
from Trajectory import circular_trajectory, line_trajectory, z_rampa

trajectory = z_rampa
controller = Controller(trajectory=trajectory)
closed_loop_dynamics, xy_controller= controller.closed_loop_dynamics,controller.xy_controller,
control_z, z_d= controller.control_z,controller.z_d,
DIST_FORCE, DIST_START, DIST_END = controller.DIST_FORCE,controller.DIST_START, controller.DIST_END
state0 = np.zeros(12)

t_span = (0, 25)
t_eval = np.linspace(0, 25, 5000)

sol = solve_ivp(closed_loop_dynamics, t_span, state0, t_eval=t_eval)

t = sol.t

x = sol.y[0]
y = sol.y[1]
z = sol.y[2]
# z_d importé depuis control.py

phi = sol.y[6]
theta = sol.y[7]
psi = sol.y[8]

phi_d_values = []
theta_d_values = []
f_values = []

x_d_values = []
y_d_values = []
z_d_values = []

for k in range(len(t)):

    state_k = sol.y[:, k]

    # Desired trajectory at instant t[k]
    x_d_k, y_d_k, z_d_k = trajectory(t[k])
    vx_d_k,vy_d_k,vz_d_k = controller.outer_controller(state_k,x_d_k, y_d_k, z_d_k)
    phi_d_k, theta_d_k = xy_controller(
        state_k,
        x_d_k,
        y_d_k,
        vx_d_k,
        vy_d_k
    )

    f_k = control_z(state_k,z_d_k,vz_d_k)

    x_d_values.append(x_d_k)
    y_d_values.append(y_d_k)
    z_d_values.append(z_d_k)

    phi_d_values.append(phi_d_k)
    theta_d_values.append(theta_d_k)
    f_values.append(f_k)

x_d_values = np.array(x_d_values)
y_d_values = np.array(y_d_values)
z_d_values = np.array(z_d_values)

phi_d_values = np.array(phi_d_values)
theta_d_values = np.array(theta_d_values)
f_values = np.array(f_values)


# ============================================================
# Performance Metrics
# ============================================================
n_ss = int(len(t) * 0.2)  # last 20 % for the stationary state

# --- Z : transitory response (0 → z_d) ---
z_max = np.max(z)
z_overshoot = max(0.0, (z_max - z_d) / z_d * 100) if z_d != 0 else 0.0

idx10 = np.where(z >= 0.1 * z_d)[0]
idx90 = np.where(z >= 0.9 * z_d)[0]
tr_z = (t[idx90[0]] - t[idx10[0]]) if (len(idx10) and len(idx90)) else float('nan')
t_peak_z = t[np.argmax(z)]

idx_tau = np.where(z >= 0.632 * z_d)[0]
tau_z = t[idx_tau[0]] if len(idx_tau) else float('nan')

err_ss_z = abs(z_d - np.mean(z[-n_ss:]))

# --- X, Y : suivi de trajectoire circulaire ---
e_x = x_d_values - x
e_y = y_d_values - y
e_xy = np.sqrt(e_x**2 + e_y**2)

err_ss_x = np.mean(np.abs(e_x[-n_ss:]))
err_ss_y = np.mean(np.abs(e_y[-n_ss:]))

e_xy_peak = np.max(e_xy)
SEUIL_XY = 0.2  # m — seuil de convergence XY
idx_conv = np.where(e_xy < SEUIL_XY)[0]
t_conv_xy = t[idx_conv[0]] if len(idx_conv) else float('nan')

print("\n========== Performance Metrics ==========")
print(f"\n  [Z]  Altitude   (Target : {z_d:.1f} m)")
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
    x,
    y,
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
axs[0, 1].plot(t,x,label="x real")

axs[0, 1].plot(t,x_d_values,"--", label="x desired")

axs[0, 1].plot(
    t,
    y,
    label="y real"
)

axs[0, 1].plot(t,y_d_values,"--",label="y desired")

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
axs[1, 0].plot(t, z, label="z real")
axs[1, 0].plot(t, z_d_values, "--", label=f"z desired ({z_d} m)")

axs[1, 0].set_title("Altitude")
axs[1, 0].set_xlabel("Time [s]")
axs[1, 0].set_ylabel("z [m]")
axs[1, 0].grid(True)
axs[1, 0].legend()

# ==========================
# 4. Attitude
# ==========================
axs[1, 1].plot(t,np.rad2deg(phi),label="φ real")

axs[1, 1].plot(t,np.rad2deg(phi_d_values),"--",label="φ desired")

axs[1, 1].plot(t,np.rad2deg(theta),label="θ real")

axs[1, 1].plot(t,np.rad2deg(theta_d_values),"--",label="θ desired")

axs[1, 1].set_title("Attitude")
axs[1, 1].set_xlabel("Time [s]")
axs[1, 1].set_ylabel("Angle [deg]")
axs[1, 1].grid(True)
axs[1, 1].legend()

# ==========================
# 5. Thrust
# ==========================
axs[2, 0].plot(t, f_values)

axs[2, 0].set_title("Thrust Command")
axs[2, 0].set_xlabel("Time [s]")
axs[2, 0].set_ylabel("Force [N]")
axs[2, 0].grid(True)

# ==========================
# 5. Error Following
# ==========================
axs[2, 1].plot(t, e_x, label="error x")
axs[2, 1].plot(t, e_y, label="error y")
axs[2, 1].plot(t, e_xy, "k--", linewidth=0.8, label="||e_xy||")

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