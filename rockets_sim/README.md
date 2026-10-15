# rockets_sim — First-person rocket launch (Lagrangian mechanics)

Interactive simulator of a single-engine rocket launch, viewed in **first
person** from the launch pad (free-camera, shooter-style), with a
live-editable parameter panel and real-time flight telemetry. The physics
is derived with the **Lagrangian formalism** (generalized coordinates, not
body-frame forces) and integrated with **4th-order Runge-Kutta**.

It uses the same physics engine as [`unmanned/launch_rockets/`](../unmanned/launch_rockets/)
(`physics.h`/`physics.cpp` are near-identical byte for byte — see
[§6](#6-relationship-with-unmannedlaunch_rockets)); the difference is the
frontend: here the camera is free and first-person, with a mission-design
HUD (T/W, mass ratio, Δv) instead of the oscilloscope-style telemetry
window used in `unmanned/`.

![Launch pad with the parameter and derived-propulsion panels, rocket still on the ground](rocket_launch1.png)
*Setup screen: launch-parameter panel (top left) and derived-propulsion
panel (bottom left) — T/W=3.33 in green means the rocket can lift off.*

![Rocket in the burn phase (BURN PHASE), with live flight telemetry](rocket_launch2.png)
*In flight: telemetry panel (right) with T+, altitude, speed, horizontal
range, remaining mass, and flight phase — T+1.5 s, 24.8 m altitude,
36.6 m/s, still burning fuel.*

## 1. Generalized coordinates and Lagrangian

Rocket state in the inertial **East-North-Up** frame (not the body frame
used by `unmanned/`'s 6DOF engine):

```
q  = (x, y, z)         position [m]
q̇  = (ẋ, ẏ, ż)         velocity [m/s]
m(t)                    mass, decreases while there's fuel [kg]
```

```
T = ½ m(t) (ẋ² + ẏ² + ż²)      kinetic energy
V = m(t) · g · z                 potential energy
L = T − V                        Lagrangian
```

## 2. Euler-Lagrange equations

With non-conservative generalized forces `Qi` (thrust + drag):

```
d/dt(∂L/∂q̇i) − ∂L/∂qi = Qi

→  m·ẍ = Fx_thrust + Fx_drag
→  m·ÿ = Fy_thrust + Fy_drag
→  m·z̈ = −m·g + Fz_thrust + Fz_drag
   ṁ    = −fuel_flow_rate        (burn phase only)
```

## 3. Forces and derived parameters (`physics.h` / `physics.cpp`)

| Force | Model |
|---|---|
| Gravity | `Fg = (0, 0, −m·g)` |
| Thrust | `Ft = thrustDir(θ,φ) · T`, active while `t < burn_time` and fuel remains |
| Aerodynamic drag | `Fd = −½·ρ(z)·Cd·A·\|v\|·v̂` |
| Mass loss | `ṁ = −fuel_mass / burn_time` (linear burn-off during the burn phase) |

**Thrust direction** — standard physics spherical coordinates, `θ` polar
from zenith (+Z) and `φ` azimuthal from East (+X):

```
thrustDir(θ,φ) = ( sin θ·cos φ,  sin θ·sin φ,  cos θ )
```

**Thrust from specific impulse** (rocket equation) instead of asking for
force directly — so the editable parameter is `Isp`, a real engine figure
of merit, not an arbitrary number of Newtons:

```
mdot      = fuel_mass_kg / burn_time_s
thrustN   = Isp_s · g₀ · mdot
deltaV    = Isp_s · g₀ · ln(total_mass_kg / dry_mass_kg)     (Tsiolkovsky)
T/W       = thrustN / (total_mass_kg · g₀)
massRatio = total_mass_kg / dry_mass_kg
```

These 4 derived quantities are exactly the HUD's **PROPULSION (derived)**
panel (section 5), color-coded: T/W red if `<1` (won't lift off), yellow
`1–1.5` (marginal), green `>1.5`; mass ratio orange if `<2`, yellow `2–4`,
green `>4`.

**Atmosphere** — exponential ISA-type density (same as
`unmanned/launch_rockets/`), affects drag with altitude:

```
rho(z) = 1.225 · e^(−z / 8500)        (kg/m³, z in meters)
```

## 4. Integration: 4th-order Runge-Kutta

Same classic 4-stage scheme with 1-2-2-1 weights as the rest of this
repo's simulators (`unmanned/`, `aerodynamical_controls/`), applied here
to the 7 state variables (position, velocity, mass):

```
k1 = f(t,        y)
k2 = f(t+h/2,    y + h/2·k1)
k3 = f(t+h/2,    y + h/2·k2)
k4 = f(t+h,      y + h·k3)

y_{n+1} = y_n + (h/6)(k1 + 2·k2 + 2·k3 + k4)
```

4 sub-steps per render frame (≈240 Hz effective integration rate at 60 fps
on screen). Ground impact is detected via `rocket.pos.z < 0`, at which
point the simulation freezes and the HUD shows `** IMPACT **`.

## 5. HUD and controls (`main.cpp`)

Three on-screen panels, driven directly by the `LaunchParams` /
`RocketState` state:

- **`LAUNCH PARAMETERS`** (top left) — the 8 editable mission parameters,
  with their valid range:

  | # | Parameter | Step | Range | Default |
  |---|---|---|---|---|
  | 1 | Theta (polar angle from zenith) | 5° | 1–179° | 20° |
  | 2 | Phi (azimuth from East) | 10° | 0–359° | 255° |
  | 3 | Isp | 10 s | 100–500 s | 100 s |
  | 4 | Burn time | 1 s | 1–120 s | 2 s |
  | 5 | Total mass | 10 kg | 10–5000 kg | 12 kg |
  | 6 | Fuel mass | 5 kg | 1–4990 kg | 0.8 kg |
  | 7 | Drag Cd | 0.05 | 0.05–2.0 | 0.5 |
  | 8 | Area | 0.005 m² | 0.005–0.5 m² | 0.05 m² |

- **`PROPULSION (derived)`** (bottom left) — Thrust, T/W, Mass Ratio,
  ideal Δv, and the atmospheric density formula in use (section 3). If
  T/W < 1 before launch, a central warning appears:
  `EMPUJE INSUFICIENTE  T/W < 1` (insufficient thrust) along with the
  minimum thrust needed — this string is literal on-screen HUD text, left
  untranslated here to match what the program actually prints.
- **`TELEMETRY`** (top right, only during/after flight) — `T+` (elapsed
  time), altitude, speed, horizontal range from the pad, remaining mass,
  and the **flight phase**: `BURN PHASE` (burning fuel) → `COAST PHASE`
  (coasting, engine off) → `** IMPACT **` (on touching the ground).

| Key | Action |
|---|---|
| `SPACE` | Launch |
| `R` | Reset — back to setup |
| `1`–`8` | Select parameter |
| `+` / `-` | Adjust the selected parameter (step size from the table above) |
| `ENTER` | Edit the selected parameter's value directly (numeric entry) |
| `W A S D` | Move the camera |
| Arrow keys / mouse drag | Rotate the view |
| `T` | Toggle automatic rocket-tracking camera |
| `ESC` | Quit |

With the default values the rocket climbs ~1–2 km and traces a ballistic
arc visible from the ground.

## 6. Relationship with `unmanned/launch_rockets/`

This project and [`unmanned/launch_rockets/`](../unmanned/launch_rockets/)
share the same physics engine (`physics.h` is byte-for-byte identical;
`physics.cpp` differs by a single comment line) — it's the same Lagrangian
mechanics described in [section 13 of the `unmanned/`
README](../unmanned/README.md#13-launch_rockets--rocket-lagrangian-mechanics).
What changes is the wrapping program: `unmanned/launch_rockets/` is a
simpler orbital/tracking-camera viewer; `rockets_sim/` is a first-person
mission-design console, with the derived-propulsion panel (T/W, mass
ratio, Δv) and the liftoff validation (`T/W < 1`) that the other one
lacks.

## Build and run

```bash
cmake -B build && cmake --build build
./build/rocket_sim
```

Requires macOS with OpenGL + GLUT (included by default in the SDK); on
Linux, `freeglut3-dev`.

## Project structure

| File | Role |
|---|---|
| `CMakeLists.txt` | Build system. |
| `physics.h` | `Vec3`, `RocketState`, `LaunchParams` structs (with the derived quantities from §3) + declarations. |
| `physics.cpp` | Euler-Lagrange (`computeDerivs`) + RK4 integrator (`rk4Step`). |
| `main.cpp` | GLUT window, first-person camera, HUD (parameters/propulsion/telemetry), 3D rendering, keyboard handling. |
| `rocket_launch1.png`, `rocket_launch2.png` | Real screenshots of the simulation, used above in this README. |

## Possible next steps

- Air-density variation with altitude is already modeled (ISA); gravity or
  temperature variation with altitude is still missing if more fidelity is
  needed.
- Multi-stage rockets (stage separation).
- Visualize the `T`, `V`, `H = T + V` energy curve in the HUD.
- Export the trajectory to CSV for external analysis.
- Replace GLUT with GLFW + Dear ImGui for a richer UI.

## Reference

Standard Lagrangian mechanics (Goldstein, *Classical Mechanics*) applied to
a variable-mass body; the Tsiolkovsky rocket equation for thrust from
specific impulse.
