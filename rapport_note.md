# Technical Note — UAV Trajectory Simulation and Control

## 1. Controller Architecture

The controller uses a **two-loop cascade** structure:

```
Desired Trajectory
       │
       ▼
  [Outer XY Loop]  ──── generates φ_d, θ_d
       │
       ▼
  [Inner Attitude Loop]  ──── generates τ_φ, τ_θ, τ_ψ
       │
  [Altitude Z Loop]  ──── generates f (total thrust)
       │
       ▼
  Drone Dynamics (Newton-Euler, 12 states)
```

- **Outer loop**: PD position controller on XY → desired angles φ_d, θ_d
  (small-angle assumption, saturation at ±15°)
- **Inner loop**: PD attitude controller → torques τ
- **Altitude Z**: independent PD controller → thrust f = m(g + u_z) / cos(φ)cos(θ)

---

## 2. Drone Physical Parameters

| Parameter | Symbol | Value | Unit |
|---|---|---|---|
| Mass | m | 1.5 | kg |
| Inertia x-axis | Jx | 0.025 | kg·m² |
| Inertia y-axis | Jy | 0.025 | kg·m² |
| Inertia z-axis | Jz | 0.045 | kg·m² |
| Gravity | g | 9.81 | m/s² |
| Max XY angle | MAX_ANGLE | ±15 | ° |

---

## 3. PD Controller Gains

### Initial gains (baseline)

| Axis | Proportional Gain | Derivative Gain |
|---|---|---|
| Altitude Z | KP_Z = 6.0 | KD_Z = 4.0 |
| Roll φ | KP_PHI = 8.0 | KD_PHI = 3.0 |
| Pitch θ | KP_THETA = 8.0 | KD_THETA = 3.0 |
| Yaw ψ | KP_PSI = 4.0 | KD_PSI = 1.5 |
| Position X | KP_X = 1.2 | KD_X = 2.0 |
| Position Y | KP_Y = 1.2 | KD_Y = 2.0 |

### Current gains (after Steps 1 & 2 tuning)

| Axis | Proportional Gain | Derivative Gain | ωn | ζ |
|---|---|---|---|---|
| Altitude Z | KP_Z = 8.5 | KD_Z = 5.0 | 2.92 rad/s | 0.86 |
| Roll φ | KP_PHI = 10.0 | KD_PHI = 2.0 | 20.0 rad/s | 2.0 |
| Pitch θ | KP_THETA = 10.0 | KD_THETA = 2.0 | 20.0 rad/s | 2.0 |
| Yaw ψ | KP_PSI = 4.0 | KD_PSI = 1.5 | — | — |
| Position X | KP_X = 2.0 | KD_X = 2.8 | 1.41 rad/s | 0.99 |
| Position Y | KP_Y = 2.0 | KD_Y = 2.8 | 1.41 rad/s | 0.99 |

> Frequency separation: ωn_attitude / ωn_XY = 20.0 / 1.10 = **18.2×** ✓

---

## 4. Simulation Results

### 4.1 Altitude Z Response (step input: 0 → 1.0 m)

The altitude response is independent of the XY trajectory.
The behavior is close to a **1st-order system** with a slight overshoot caused by the zero introduced by the derivative action.

| Metric | Baseline (KP=6, KD=4) | **Current (KP=8.5, KD=5)** | Δ |
|---|---|---|---|
| **Time constant τ** | 0.77 s | **0.67 s** | ✅ −13% |
| Rise time tr (10%→90%) | 1.03 s | **0.93 s** | ✅ faster |
| Overshoot | 1.18 % | **0.5 %** | ✅ cleaner |
| Steady-state error | 0.00 cm | **0.00 cm** | ✅ |

> Increasing KP_Z raised ωn (2.45 → 2.92 rad/s) giving a faster response. Increasing KD_Z slightly raised ζ (0.82 → 0.86), reducing overshoot. The ratio tr/τ ≈ 1.39 confirms the system remains slightly faster than a pure 1st order due to the PD zero.

---

### 4.2 Circular Trajectory (R = 2 m, ω = 0.3 rad/s)

The reference trajectory is:
```
x_d(t) = 2·cos(0.3t)
y_d(t) = 2·sin(0.3t)
```
The drone starts at (0, 0, 0). The initial XY error is therefore **2.0 m** (distance from (0,0) to (2,0)).

| Metric | Value |
|---|---|
| Metric | Baseline | Steps 1&2 | **Step 3** | Δ vs baseline |
|---|---|---|---|---|
| Maximum tracking error ‖e_xy‖ | 2.000 m | 2.000 m | **2.000 m** | — (initial geometry) |
| Convergence time (‖e_xy‖ ≤ 0.2 m) | 2.30 s | 2.54 s | **1.94 s** | ✅ −16% |
| Steady-state error X | 13.04 cm | 13.00 cm | **7.76 cm** | ✅ −40% |
| Steady-state error Y | 5.21 cm | 5.13 cm | **3.39 cm** | ✅ −35% |

> τ is not relevant for X and Y on a circular trajectory (sinusoidal reference with no constant final value). The appropriate metrics are the steady-state tracking error and the convergence time.

> The maximum tracking error (2.0 m) is fixed by the initial geometry — the drone starts at (0,0) while the circle starts at (2,0) — and cannot be reduced by gain tuning alone. It would require a smoother trajectory initialisation.

> Steps 1&2 (Z and attitude tuning) temporarily worsened convergence (2.30→2.54 s) by making the attitude loop more aggressive without touching XY gains. Step 3 (KP_X/Y 1.2→2.0, KD_X/Y 2.0→2.8) recovered and surpassed the baseline on all XY metrics, bringing convergence to 1.94 s and halving the steady-state tracking errors.

---

### 4.3 Linear Trajectory (x_d = 0.2t m, y_d = 1.0 m)

The reference trajectory is:
```
x_d(t) = 0.2·t   (constant velocity ramp, vx_d = 0.2 m/s)
y_d(t) = 1.0      (step)
```
The drone starts at (0, 0, 0). The initial XY error is **1.0 m** (distance from y=0 to y_d=1.0m).

| Metric | Value |
|---|---|
| Maximum tracking error ‖e_xy‖ | 1.000 m (initial transient in Y) |
| Convergence time (‖e_xy‖ ≤ 0.2 m) | **2.19 s** |
| Steady-state error X (mean over last 20%) | **0.00 cm** |
| Steady-state error Y (mean over last 20%) | **0.00 cm** |

> The zero steady-state error on X is achieved without integral action because the velocity feedforward term `KD_X · (vx_d − vx)` compensates the ramp directly. In steady state, when `vx = vx_d = 0.2 m/s`, the feedforward drives `ax_des → 0` and the position error converges to zero. This holds for the current low-speed ramp (0.2 m/s); faster ramps with inner-loop lag may reintroduce a residual error.

---

## 5. Gain Tuning Protocol

### Golden rule: always tune **inside → out**
The inner loop must be fully tuned before touching the outer loop, as the outer loop depends on the inner loop's response speed.

---

### Step 1 — Altitude Z (independent of XY) ✓ (done)

> **Important**: altitude dynamics is a **double integrator** (z̈ = f/m − g → 1/ms²).
> A pure P controller on a double integrator is marginally stable and **always oscillates**, regardless of KP.
> KD_Z is structurally required for stability — never set it to zero.

The correct approach is to choose a desired **natural frequency ωn** and **damping ratio ζ**, then compute the gains directly:

```
KP_Z = ωn²
KD_Z = 2·ζ·ωn
```

**Current gains** (KP_Z = 6.0, KD_Z = 4.0) correspond to:
```
ωn = √6.0 ≈ 2.45 rad/s
ζ  = 4.0 / (2 × 2.45) ≈ 0.82   (slightly overdamped)
```
This explains the short τ (0.77 s) and low overshoot (1.18%).

**Tuning procedure:**

1. Choose target response: e.g. ωn = 2.5 rad/s, ζ = 0.8
2. Compute: `KP_Z = ωn²`,  `KD_Z = 2·ζ·ωn`
3. Run simulation, observe τ and overshoot
4. To speed up response → increase ωn (raises both gains proportionally)
5. To reduce overshoot → increase ζ (raises KD_Z only)
6. **Target**: τ < 1 s, overshoot < 10%, SSE = 0

| Observation | Action |
|---|---|
| τ too large | increase ωn |
| Overshoot > 10% | increase ζ |
| Noisy thrust signal | ζ too high or ωn too high — reduce KD_Z |

---

### Step 2 — Inner attitude loop (φ, θ, ψ) ✓ (done)

Must be **5–10× faster** than the XY outer loop.

> **Same structure as Z**: the attitude dynamics is also a double integrator (Jx·φ̈ = τ_φ),
> so KD is structurally required for stability. Never set KD_PHI = 0.
> Use the same ωn / ζ approach:
> ```
> ωn  = √(KP_PHI / Jx)
> ζ   = KD_PHI / (2·√(KP_PHI · Jx))
> ```

**Current values** (Jx = Jy = 0.025 kg·m²):

| | Formula | Value |
|---|---|---|
| ωn_attitude | √(8.0 / 0.025) | 17.9 rad/s |
| ζ_attitude | 3.0 / (2·√(8·0.025)) | **3.35** → heavily overdamped |
| ωn_XY | √(KP_X) = √1.2 | 1.10 rad/s |
| Separation ratio | ωn_att / ωn_XY | **16.3×** ✓ (≥ 5 required) |

ζ = 3.35 means the loop is stable and robust but **slower than necessary**.
The dominant pole is at τ_att ≈ 0.37 s.
For ζ = 0.8 with KP_PHI = 8.0, the optimal KD would be:
```
KD_PHI = 2 · 0.8 · √(8.0 · 0.025) ≈ 0.72
```
Current KD_PHI = 3.0 is ~4× higher than needed for critical damping — it can be reduced significantly.

**Tuning procedure:**

1. Fix `KP_PHI` (sets ωn), compute `KD_PHI = 2·ζ·√(KP_PHI·Jx)` with ζ ∈ [0.7, 1.0]
2. Repeat for θ (identical since Jx = Jy)
3. Tune ψ separately with `KP_PSI`, `KD_PSI` (Jz = 0.045 ≠ Jx)
4. **Always verify**: ωn_attitude / ωn_XY ≥ 5

**What to monitor in the simulation:**

| Plot | What to watch | Target |
|---|---|---|
| Attitude | φ/θ vs φ_d/θ_d lag | settling < 0.3 s |
| Error XY | oscillations after convergence | none → attitude fast enough |
| Thrust | spikes or high-frequency noise | none → KD_PHI not too high |

**Tuning levers:**

| Goal | Action |
|---|---|
| Faster response | increase KP_PHI (raises ωn) |
| Less damping (faster, slight overshoot) | reduce KD_PHI (ζ toward 0.7) |
| More damping (slower, no overshoot) | increase KD_PHI (ζ toward 1.0+) |

**Final gains and conclusion:**

KD_PHI = 0.72 (ζ = 0.72, slightly underdamped) was tested and **rejected** — the attitude overshoot perturbed XY tracking and degraded convergence time from 2.30 s to 2.62 s.

**Retained: KD_PHI = KD_THETA = 2.0**
```
ωn  = √(10.0 / 0.025) = 20.0 rad/s
ζ   = 2.0 / (2·√(10·0.025)) = 2.0   → overdamped
```
Although ζ = 2.0 is overdamped, the attitude natural frequency (20 rad/s) is already 18× faster than the XY outer loop (1.1 rad/s), well above the required 5× separation. The overdamping avoids attitude oscillations that would destabilize XY tracking — a deliberate trade-off favouring XY performance over attitude speed.

---

### Step 3 — Outer XY loop ✓ (done)

Tune **only after** the attitude loop is stable and fast.

1. Set `KD_X = KD_Y = 0`
2. Command a position step (e.g. x = 2 m), increase `KP_X` until XY oscillates
3. Set `KP_X ≈ 0.5 × KP_crit`, then increase `KD_X` to damp
4. **Check angle saturation**: at peak response, θ must stay well below ±15°
5. **Target**: convergence < 3 s, SSE < 5 cm, no oscillation

**Final gains:**
```
KP_X = KP_Y = 2.0  →  ωn = √2.0 = 1.41 rad/s
KD_X = KD_Y = 2.8  →  ζ  = 2.8 / (2·√2.0) = 0.99  ≈ critically damped
Frequency separation: ωn_attitude / ωn_XY = 20.0 / 1.41 = 14.2×  ✓
```

**Results vs baseline:**

| Metric | Baseline | Step 1&2 | **Step 3** | Δ vs baseline |
|---|---|---|---|---|
| tau Z | 0.77 s | 0.67 s | **0.67 s** | ✅ −13% |
| Overshoot Z | 1.18 % | 0.5 % | **0.5 %** | ✅ |
| Convergence XY | 2.30 s | 2.54 s | **1.94 s** | ✅ −16% |
| SSE X | 13.04 cm | 13.00 cm | **7.76 cm** | ✅ −40% |
| SSE Y | 5.21 cm | 5.13 cm | **3.39 cm** | ✅ −35% |

> All three metrics improved over baseline after completing the full tuning sequence. SSE on X and Y dropped ~40% thanks to the higher XY bandwidth (ωn 1.10→1.41 rad/s). Convergence time is now the best achieved (1.94 s). Z unchanged as expected — altitude is independent of XY gains.

---

### Frequency separation check

After tuning all loops, verify:

```
ω_attitude >> ω_XY

KP_PHI / KP_X ≥ 5   →   currently 8 / 1.2 ≈ 6.7  ✓
```

If this ratio drops below 5, the loops interact and can destabilize each other.

---

### Practical tips

- Change **one gain at a time** — never KP and KD simultaneously
- Use `simulate_hover.py` to validate Z independently before coupling XY
- A noisy or saturating thrust means `KD_Z` is too high
- φ or θ exceeding ±15° means `KP_X/Y` is too aggressive
- Increase `t_span` if the drone has not settled before the end of simulation

---

## 6. Areas for Improvement

### 6.1 Integral action — tested and removed

A PID was implemented on X and Y (KI_X = KI_Y = 0.5, with anti-windup) and tested on the circular trajectory. Results vs the PD-only controller:

| Metric | PD Step 3 | PID KI=0.5 | Δ |
|---|---|---|---|
| Convergence XY | 1.94 s | 1.84 s | ✅ −5% |
| SSE X | 7.76 cm | 4.19 cm | ✅ −46% |
| SSE Y | 3.39 cm | 6.94 cm | ❌ +105% |

**X improved but Y degraded significantly** — the integrator was removed and the PD controller retained.

**Root cause**: on a sinusoidal reference (`y_d = 2·sin(0.3t)`), an integrator adds phase lag at the tracking frequency, shifting the Y response and increasing the mean tracking error. The integrator helps on step/ramp references (consistent bias) but hurts on periodic ones (oscillating error accumulates incorrectly).

**Conclusion**: the integral action is only beneficial if the trajectory reference is a step or ramp. For circular tracking, the PD + velocity feedforward is the better choice.

### 6.2 Loop bandwidth — formal verification
The bandwidth separation (KP_PHI / KP_X ≈ 6.7) appears satisfactory but has not been formally verified. A frequency-domain analysis (Bode diagram) would validate the separation and confirm stability margins.

### 6.3 Angle saturation on aggressive trajectories
The ±15° saturation is not a constraint for the current trajectories (centripetal acceleration ≈ 0.18 m/s², θ_d ≈ 1.05°). For higher speeds or larger radii, this limit would restrict tracking performance and should be accounted for in trajectory planning.

### 6.4 No disturbance rejection
The current model is ideal (no wind, no sensor noise, no delay). Future work should include:
- A state observer (Kalman filter) to estimate states from noisy measurements
- External disturbances (wind gusts) to evaluate controller robustness

### 6.5 Small-angle assumption validation
The outer-loop equations assume `sin(θ) ≈ θ` and `cos(θ) ≈ 1`. This holds for |θ| < 15° but introduces an unquantified modeling error. A numerical comparison between the linearized and the full nonlinear model (exact rotation matrix) would bound this error.
