# unmanned — Multi-vehicle UAV physics engine (6DOF)

Technical report on the flight-simulation engine contained in this
folder: what was built, why each piece exists, what equations and
numerical methods implement it, and what scenario each program puts to
the test.

The engine simulates the rigid-body dynamics of **three vehicles** — a
delta-wing UAV, a quadcopter in X configuration, and a 4-rotor tilt-rotor
convertiplane — based on Beard & McLain's equations of motion, *Small
Unmanned Aircraft* (Ch. 3 and 4). The physics model is a single source of
truth (`physics_UAV.h` + `simulate_step.h`) consumed by both the console
demo (`physics_UAV.cpp`) and the interactive 3D visualization
(`drone_model.cpp`), so both simulate exactly the same thing. A fourth,
separate module (`launch_rockets/`) simulates a rocket launch with
Lagrangian mechanics.

**Why each scenario matters:** the delta-wing UAV and the quad are the
two "textbook" control cases (longitudinal thrust with wing lift vs.
collective thrust vectored by tilting) — they validate that the Beard &
McLain equations are correctly implemented against two opposite flight
regimes. The convertiplane is the interesting case: it isn't just a third
vehicle, it's a *transition* — its mass geometry, its lift/thrust mix,
and its control-authority split change continuously between the two
regimes above, and that transition is exactly where a real tilt-rotor can
fail (loss of control authority, propeller thrust lapse with airspeed,
loss of lift if the transition is too fast). The 20-obstacle field and
the solid-collision model exist to force the (human) pilot to make 3D
trajectory decisions, not just avoid the ground.

![Delta-wing UAV in flight, with the live telemetry window](ala_delta.png)
*Real view of `drone_model` running the delta-wing UAV: main window (HUD +
obstacles) on the left, live telemetry oscilloscope on the right.*

## 1. Project structure

| File | Role |
|---|---|
| `physics_UAV.h` | Physics engine (header-only): mass/inertia geometry for the 3 vehicles, translational/rotational kinematics and dynamics, fixed-wing aerodynamics, propeller model, convertiplane attitude control, Euler and RK4 integrators, attitude conversion, ground and obstacle collision. |
| `simulate_step.h` | `simulateStep()` — an RK4 variant that uses the **full inertia tensor** (with products of inertia Ixy, Ixz, Iyz), instead of the reduced form that assumes XZ symmetry. |
| `physics_UAV.cpp` | Console demo: prints mass/inertia for 2 vehicles, a convertiplane transition report, a wing-aerodynamics table, an attitude-conversion demo, and runs 5 s of free flight with `simulateStep()`. |
| `drone_model.cpp` | Interactive 3D visualization (OpenGL/GLUT): keyboard-flyable 3 vehicles + live telemetry window + 20-obstacle solid field. |
| `Makefile` | Build and run targets. |
| `launch_rockets/` | Separate project: rocket-launch simulation (Lagrangian mechanics, RK4), first-person view. See [section 13](#13-launch_rockets--rocket-lagrangian-mechanics). |
| `ala_delta.png` | Real screenshot of `drone_model` (delta-wing UAV) in flight, used above in this README. |
| `euler_to_quaternions.jpeg`*, `quaternion_to_rotationMatrix.jpeg`* | Reference for the attitude-conversion formulas (not included in the repo; see the transcribed formulas in [section 9](#9-attitude-conversion-euler--quaternion--rotation-matrix)). |

> Note: the `Makefile` also defines a `rot_trans_kinematics` target, but
> `rot_trans_kinematics.cpp` isn't currently present in this folder — that
> target doesn't build.

## 2. Vehicle state and axis convention

The full state of any of the 3 vehicles (`UAVState`) has **12
components**:

- **Inertial NED position** (`pn, pe, pd`) — North-East-Down frame; `pd`
  positive downward (so "climbing" means `pd` getting more negative).
- **Body-frame velocity** (`u, v, w`).
- **Attitude in Euler angles** (`phi` roll, `theta` pitch, `psi` yaw).
- **Body-frame angular rates** (`p, q, r`).

Body axes: **X** toward the nose, **Y** right wing, **Z** downward.

The equations governing the evolution of these 12 components are
organized into 5 blocks, following the structure of Beard & McLain Ch. 3:

```
Block 0  Mass/inertia geometry        shape      -> mass, CG, inertia tensor
Block 1  Translational kinematics     u,v,w      -> pn_dot,pe_dot,pd_dot
Block 2  Translational dynamics       fx,fy,fz   -> u_dot,v_dot,w_dot
Block 3  Rotational kinematics        p,q,r      -> phi_dot,theta_dot,psi_dot
Block 4  Rotational dynamics          l,m,n      -> p_dot,q_dot,r_dot
```

## 3. Block 0 — Mass and inertia geometry

**Why this block exists:** instead of making up `Ixx, Iyy, Izz`, the
simulator *derives* them from each vehicle's real geometry (fuselages,
wings, arms, nacelles...), applying the parallel-axis theorem to each
part and summing. This makes the rotational dynamics (Block 4) a
consequence of a physical shape, not an arbitrary design number — and it
lets the convertiplane, whose geometry *changes in flight*, recompute its
inertia at every step instead of using a precomputed table.

### 3.1 Inertia of the primitive parts

Each part is modeled as an elementary solid with known inertia about
**its own CG**:

| Part | Function | Formula (about its own CG) |
|---|---|---|
| Solid cylinder (fuselage, arms, nacelles), X axis | `inerciaCilindro` | `Ixx = ½ m r²`,  `Iyy = Izz = (1/12) m (3r² + L²)` |
| Delta triangular plate in the XY plane (delta wing) | `inerciaAlaDelta` | `Ixx = m b²/24`,  `Iyy = m c²/18`,  `Izz = Ixx+Iyy` |
| Rectangular box (compartments, wing panel, cargo bay, landing gear, stabilizer) | `inerciaCaja` | `Ixx = (1/12) m (ly²+lz²)`, analogous for `Iyy, Izz` |
| Point mass (motor) | `inerciaPuntual` | `I = 0` (all its inertia comes from translation) |
| Thin disk in the XY plane, Z axis (rotor) | `inerciaDiscoZ` | `Izz = ½ m r²` (spin axis),  `Ixx=Iyy = ¼ m r²` (diametral) |

### 3.2 Parallel axis theorem

Each part lives at its own CG, distinct from the vehicle's combined CG.
Summing them requires translating each local tensor to the common CG
(`trasladar()`), with displacement vector `r = part_cg − combined_cg`:

```
Ixx' = Ixx + m(ry² + rz²)      Ixy' = Ixy − m·rx·ry
Iyy' = Iyy + m(rx² + rz²)      Ixz' = Ixz − m·rx·rz
Izz' = Izz + m(rx² + ry²)      Iyz' = Iyz − m·ry·rz
```

`combinarComponentes()` does this for an arbitrary list of parts: it
computes the total mass and combined CG by a weighted average, translates
each local tensor to that CG, and sums them. It's the single assembly
routine shared by all 3 vehicles — each one only describes its own list
of parts.

Block 4's equations of motion (reduced form) assume mass symmetry about
the **XZ** plane (`Ixy = Iyz = 0`); all 3 vehicles satisfy it by
construction (the delta wing lives at `y=0`, and the quad and
convertiplane have their parts in `±y` pairs). If that symmetry were ever
broken, `combinarComponentes()` warns via `stderr`.

### 3.3 Rotating an inertia tensor (tilted arms and nacelles)

A quad's arm or a convertiplane's nacelle has an inertia that's easy to
compute **about its own axis** (a cylinder along X), but in the vehicle
body it's rotated. To bring the tensor to body axes, the similarity
transform `I' = R · I · R^T` is used:

- `rotarZ(I, angle)` — rotates about the Z axis; used for the quadcopter's
  4 arms (at 45°, 135°, 225°, 315° from the X axis, "X" configuration).
- `rotarY(I, angle)` — rotates about the Y axis; used for the
  convertiplane's nacelles and rotors, which literally **tilt** about
  that axis when going from hover to cruise.
- `rotarYVec(v, angle)` — the same rotation but on a position vector:
  when tilting, each nacelle's CG actually moves in space, and with it,
  both the whole vehicle's CG and its inertia tensor change.

### 3.4 The three vehicles (Block 0 instantiated)

| | `buildDeltaWingDrone()` | `buildQuadcopter()` | `buildTiltrotor(sigma)` |
|---|---|---|---|
| Parts | fuselage (cylinder) + delta wing (plate) + compartment (box) + motor (point) | body (box) + landing gear + 4×(arm + motor + rotor) | fuselage + cargo bay + 2 wing panels + fin + stabilizer + landing gear + 4×(nacelle + rotor) |
| Total mass | **200.0 kg** | **142.0 kg** | **154.9 kg** (varies with `sigma` only in CG/inertia, not in mass) |
| `Ixx, Iyy, Izz` [kg·m²] | 16.1 / 187.7 / 202.0 | — (X config, symmetric) | 207.0 / 174.6 / 361.4 @ `sigma=0` (verified by running `physics_UAV`) |
| Distinguishing trait | fixed geometry | fixed geometry, 4-branch symmetry | **variable geometry**: `sigma` ∈ [0, π/2] tilts the 4 nacelles+rotors about the Y axis, moving CG and inertia at every step |

By design, the convertiplane is the only vehicle whose Block 0 is
**re-evaluated at every integration step** (`buildTiltrotor(g_tilt)`
inside `stepTiltrotor()`), because its physical shape changes in flight.

## 4. Block 1 — Translational kinematics

Converts body-frame velocity to inertial position rate, via the
body→inertial rotation matrix `R_b^v` (product of 3 Euler rotations):

```
[pn_dot]   [cθcψ   sφsθcψ-cφsψ   cφsθcψ+sφsψ] [u]
[pe_dot] = [cθsψ   sφsθsψ+cφcψ   cφsθsψ-sφcψ] [v]
[pd_dot]   [-sθ    sφcθ           cφcθ        ] [w]
```

(`c`=cos, `s`=sin; subscripts `φ,θ,ψ` = roll, pitch, yaw). Purely
geometric — doesn't depend on mass or forces.

## 5. Block 2 — Translational dynamics

Newton's second law expressed **in the body frame, which rotates** with
the vehicle — hence the Coriolis/centrifugal terms (`rv−qw`, etc.):

```
u_dot = (r·v − q·w) + fx/m
v_dot = (p·w − r·u) + fy/m
w_dot = (q·u − p·v) + fz/m
```

`fx, fy, fz` are input forces (thrust + gravity projected into the body
frame + aerodynamics) that each scenario assembles — Beard & McLain
leave them as external input on purpose.

### 5.1 Simple quadratic drag (`aeroDrag`) — delta-wing UAV and quad

For the delta-wing UAV and the quad (which in `drone_model.cpp` don't use
the full aerodynamic model), drag is `F = -k·v·|v|` per body axis, with
`k` calibrated per vehicle. Without this term, constant thrust
accelerates without limit (Newton's 1st law) and releasing the throttle
doesn't slow anything down; with it, every throttle position has a real
equilibrium point (`thrust = drag`) at a fixed cruise speed.

### 5.2 Full fixed-wing aerodynamics (Block 2b) — convertiplane

The convertiplane genuinely needs real lift, dependent on airspeed and
angle of attack — it's what decides whether a transition completes or
the vehicle falls. Implements Beard & McLain Ch. 4:

```
Air data:  Va = |v_body|,  alpha = atan2(w,u),  beta = asin(v/Va)
           qbar = ½ · rho · Va²                    (dynamic pressure)
```

**Lift coefficient** — blends, with a sigmoid, the *linear* model (valid
before stall) with the *flat-plate* model (valid after, where `alpha` can
routinely pass through ±90° in hover):

```
sigma_blend(alpha) = (1 + e1 + e2) / ((1+e1)(1+e2))
    e1 = exp(-M(alpha - alpha0)),  e2 = exp(M(alpha + alpha0))

CL(alpha) = (1-sigma_blend)·(CL0 + CLalpha·alpha)
          +    sigma_blend ·(2·sign(alpha)·sin²(alpha)·cos(alpha))
```

**Drag coefficient** (parasitic + induced, eq. 4.11):

```
CD(alpha) = CDp + CL_linear² / (pi · e · AR)         AR = b²/S
```

**Forces in the body frame** (eq. 4.6, rotating (L,D) from the wind frame
to the body by `alpha`):

```
fx = qbar·S·(-CD·cos(alpha) + CL·sin(alpha))
fy = qbar·S·( CYbeta·beta )
fz = qbar·S·(-CD·sin(alpha) - CL·cos(alpha))
```

**Airframe's own moments** (eq. 4.7/4.19) — static stability
(`Cl_beta`, `Cm_alpha`, `Cn_beta`) + angular-rate damping (`Clp`, `Cmq`,
`Cnr`), nondimensionalized by span/mean chord:

```
l = qbar·S·b·(Cl0 + Clbeta·beta + Clp·(b/2Va)·p + Clr·(b/2Va)·r)
m = qbar·S·c·(Cm0 + Cmalpha·alpha + Cmq·(c/2Va)·q)
n = qbar·S·b·(Cn0 + Cnbeta·beta + Cnp·(b/2Va)·p + Cnr·(b/2Va)·r)
```

These are the moments the air puts there *by itself*, with no pilot
input: the wing resists rolling, the tail resists pitching, the fin
aligns the nose with the relative wind. In hover, `qbar≈0` and they
vanish — there, rotor control (Block 4b) is the only thing stabilizing
the vehicle.

Default coefficients: those of the **Aerosonde** (Beard & McLain,
Appendix E), a UAV of a size class similar to this simulator's vehicles.
`S`, `b`, `c` are set by each vehicle (not loose constants), so the wing
that lifts, the one that's weighed in Block 0, and the one that's drawn
are literally the same wing.

### 5.3 Propeller thrust lapse with airspeed

A rotor produces less thrust moving forward than standing still: the
faster the air entering the disk, the less it can be accelerated.
Momentum theory (Beard & McLain eq. 4.18):

```
T(delta, Va_ax) = delta · T_static − kLapse · Va_ax²
    kLapse = ½ · rho · S_prop · C_prop     (S_prop = the convertiplane's 4 disks)
    Va_ax  = u·sin(sigma) − w·cos(sigma)   (velocity projected onto the nacelle axis)
```

Only the flow entering from in front of the disk counts (`Va_ax > 0`).
For the convertiplane this isn't a cosmetic detail: its thrust is sized
to hold its weight in hover (T/W=2), far more than it needs to overcome
drag in cruise — without the lapse, in airplane mode it would accelerate
with no practical ceiling; with it, available thrust collapses right as
the wing is already holding the weight, which is a tilt-rotor's real
trade-off.

## 6. Block 3 — Rotational kinematics

Converts body angular rates to Euler-angle rates:

```
[phi_dot  ]   [1  sin(phi)tan(theta)  cos(phi)tan(theta)] [p]
[theta_dot] = [0  cos(phi)           -sin(phi)           ] [q]
[psi_dot  ]   [0  sin(phi)/cos(theta) cos(phi)/cos(theta)] [r]
```

Singularity at `theta = ±90°` (**gimbal lock** — `sec(theta)` diverges);
`rotational_kinematics()` emits a `[WARNING]` via `stderr` if
`cos(theta)` gets close to 0. This is a known limitation of representing
attitude with Euler angles (see also [section 9](#9-attitude-conversion-euler--quaternion--rotation-matrix)).

## 7. Block 4 — Rotational dynamics

### 7.1 Reduced form (`rotational_dynamics`, XZ symmetry)

Valid when `Ixy = Iyz = 0` (all 3 vehicles satisfy it). Γ constants
precomputed from the inertia tensor (Beard & McLain eq. 3.17):

```
Γ  = Jx·Jz - Jxz²
Γ1 = Jxz(Jx-Jy+Jz)/Γ    Γ2 = (Jz(Jz-Jy)+Jxz²)/Γ    Γ3 = Jz/Γ    Γ4 = Jxz/Γ
Γ5 = (Jz-Jx)/Jy         Γ6 = Jxz/Jy
Γ7 = ((Jx-Jy)Jx+Jxz²)/Γ Γ8 = Jx/Γ

p_dot = Γ1·p·q − Γ2·q·r + Γ3·l + Γ4·n
q_dot = Γ5·p·r − Γ6·(p²-r²) + m/Jy
r_dot = Γ7·p·q − Γ1·q·r + Γ4·l + Γ8·n
```

### 7.2 Full form (`rotational_dynamics_full`, `simulate_step.h`)

When the tensor is **not** symmetric about XZ, the unsimplified Euler
equation for a rigid body is used:

```
I · omega_dot = M − omega × (I·omega)
omega_dot     = I⁻¹ · (M − omega × I·omega)
```

with `I` the full symmetric 3×3 matrix. It's inverted on every
evaluation via Cramer's rule / the adjugate matrix (with a check for a
near-zero determinant). It's more expensive than the Γ form (one 3×3
inversion per call instead of 8 precomputed constants), so it lives in a
separate header and is only used when actually needed — in this project,
the console demo uses it to run the delta wing's exact tensor
(`drone.I_full`) instead of the reduced form.

### 7.3 Block 4b — Convertiplane attitude control

The delta-wing UAV and the quad command `p, q, r` **directly** from the
keyboard, skipping Block 4 entirely (see [section 8](#8-numerical-integration)).
The convertiplane can't: its control authority changes completely during
the transition, and modeling that is exactly its interesting physics.
Here `p, q, r` **do** come from integrating Block 4, fed by:

```
total torque = command torque (saturated by available authority)
             + airframe's own aerodynamic moments (section 5.2)
```

**Where the authority comes from** (`tiltrotorAuthority`), for a given
tilt angle `sigma`:

1. **Differential thrust between rotors.** Each rotor pushes along its
   nacelle's axis, `n = (sin σ, 0, −cos σ)`. A differential `dT` between
   rotors produces, via its vertical component (`−cos σ`) or longitudinal
   component (`sin σ`): rotors at `±y` → **roll** in hover /
   **yaw** in cruise; rotors at `±x` → **pitch** always.
2. **Differential tilting.** Tilting some nacelles more than others
   applies a force along the perpendicular `t = (cos σ, 0, sin σ)`: in
   hover (`t=+x` on `±y`) → **yaw**; in cruise (`t=+z` on `±y`) → **roll**.
   This is exactly complementary to (1): between the two, all 3 axes
   stay covered across the whole transition range.
3. **Aerodynamic surfaces** (aileron/elevator/rudder). Scale with `qbar`,
   so they're zero in hover and are the primary control in cruise — the
   rotor/surface split happens automatically, with no explicit
   mode-switching logic.

**Control loop** (`tiltrotorRateControl`) — the pilot commands desired
**rates** (`p_cmd, q_cmd, r_cmd`), not torques. A proportional controller
requests the torque that would drive the current rate to the commanded
one with a time constant `tau`, and saturates it against the available
authority:

```
torque = sat( J·(rate_cmd − rate_actual) / tau ,  ±available_authority )
```

Writing the gain as `J/tau` (rather than as a bare constant) makes the
closed-loop response the same time constant `tau` **regardless of the
flight mode**, even as the inertia tensor changes with tilt. When it
saturates, the vehicle responds more slowly than commanded — which is
exactly what really happens when more control is demanded than the
current flight mode can provide.

## 8. Numerical integration

| Integrator | Order | Where it's used | Cost |
|---|---|---|---|
| `integrate_euler()` | 1st order (forward Euler) | — (available in the header, not called from the current programs) | 1 derivative evaluation per step |
| `integrate_rk4()` | 4th order (classic Runge-Kutta) | — (reduced Γ form; available but `physics_UAV.cpp` uses the full-tensor variant) | 4 evaluations per step |
| `simulateStep()` (`simulate_step.h`) | 4th order, **full inertia tensor** | `physics_UAV.cpp` (console demo) | 4 evaluations per step + 1 3×3 inversion per evaluation |
| "Manual" forward Euler (`step()` / `stepTiltrotor()` in `drone_model.cpp`) | 1st order | `drone_model.cpp` (interactive 3D visualization) | 1 evaluation per frame |

**RK4 (4 stages, classic 1-2-2-1 weights):** evaluates the full state
derivative (`state_derivative` — Blocks 1 through 4 at the same instant)
at 4 points per step and combines them:

```
k1 = f(s,            t)
k2 = f(s + k1·dt/2,  t+dt/2)
k3 = f(s + k2·dt/2,  t+dt/2)
k4 = f(s + k3·dt,    t+dt)

s_{n+1} = s_n + (dt/6)·(k1 + 2·k2 + 2·k3 + k4)
```

implemented component by component in `rk4_combine()`, shared between
`integrate_rk4()` and `simulateStep()`. Forces and torques are assumed
constant during the step in both cases; only the integration's order of
accuracy changes, not the force model.

**Why the console demo uses RK4+full tensor and the 3D one uses plain
Euler:** `physics_UAV.cpp` runs a fixed offline scenario (5 s, `dt=0.01`,
no human input) where it's worth maximizing numerical accuracy and
verifying the engine with the exact inertia tensor, without a real-time
constraint. `drone_model.cpp` integrates at a **fixed ~60 Hz**
(`DT = 0.016 s`) inside a render loop with live keyboard input: there,
per-frame cost matters, and Euler's truncation error at such a small step
is acceptable for an interactive simulation — plus the pilot is
constantly correcting via the keyboard, which acts as a feedback that
bounds the accumulated error. Note that this means the delta-wing UAV and
the quad **skip Block 4 entirely** in the 3D demo (p,q,r are written
directly from the keyboard, section 7.3); the convertiplane is the only
one of the three programs/vehicles that genuinely integrates the full
Block 4 step by step in real time.

## 9. Attitude conversion (Euler ↔ Quaternion ↔ Rotation Matrix)

Implemented in `physics_UAV.cpp` (declared in `physics_UAV.h`), as
reporting/conversion utilities separate from the integration loop (which
keeps working in Euler angles). They're used to display an attitude
representation without Block 3's gimbal-lock singularity.

**Euler → Quaternion** (`e_b^i = (e0,e1,e2,e3)`, scalar part `e0`):

```
e0 = cy2·ct2·cp2 + sy2·st2·sp2
e1 = cy2·ct2·sp2 − sy2·st2·cp2
e2 = cy2·st2·cp2 + sy2·ct2·sp2
e3 = sy2·ct2·cp2 − cy2·st2·sp2
   (c_2/s_2 = cos/sin of half of each angle: phi/2, theta/2, psi/2)
```

**Quaternion → Euler:**

```
phi   = atan2( 2(e0e1+e2e3), e0²+e3²-e1²-e2² )
theta = asin ( 2(e0e2-e1e3) )
psi   = atan2( 2(e0e3+e1e2), e0²+e1²-e2²-e3² )
```

**Quaternion → Rotation matrix** `R_b^i` (body→inertial): standard 3×3
matrix as a function of `(e0,e1,e2,e3)`, implemented in
`quaternionToRotationMatrix()`.

## 10. Collisions

### 10.1 Ground (`checkGroundCollision` / `resolveGroundCollision`)

The ground is the horizontal plane `pd = pisoPD` (0 by default). Crossing
it (`pd > pisoPD`, recall `pd` positive is downward) triggers a **minimal
inelastic response**: it pins `pd` to the ground and zeroes `u,v,w` — a
hard stop, no bounce or deformation. The drone is treated as a point (its
CG).

### 10.2 Solid obstacles (`checkObstacleCollision` / `resolveObstacleCollision`)

Fixed rectangular blocks (`BoxObstacle`), same simplifications as the
ground: a hard constraint, no deformation, drone treated as a point.

- **Collision test:** AABB (axis-aligned bounding box) — compares
  `pn, pe, pd` against the block's range on the 3 axes.
- **Response:** pushes the drone to the nearest face along the
  **least-penetration** axis (MTV — *minimum translation vector*), same
  as the ground but extended to 3 axes. Without this repositioning, just
  zeroing velocity on the impact frame isn't enough, because
  thrust/gravity rebuild it on the next step and the drone ends up
  passing through the block.

### 10.3 The obstacle field in `drone_model.cpp`

20 blocks (`g_obstacles[]`) spread within the ±200 m grid, with heights
from **6 to 50 m**. The first is the reference block, 100 m north of the
origin and on the initial cruise axis, meant to be hit head-on without
maneuvering. The rest force a choice of **altitude** to cross each zone,
not just dodging head-on. No pair overlaps, and a radius of **≈32.9 m**
around the origin is kept clear (geometrically verified value — the real
minimum distance from the origin to any box), so the vehicle never spawns
inside a block on startup or reset.

Each block is shaded with a gradient by height (amber for low ones, cold
violet for tall ones); the block that was hit is marked red.

### 10.4 "Signal loss" on crash (`g_crashed`)

On hitting the ground or an obstacle: the physics freezes (`step()` stops
integrating), flight controls lock, the window shows a TV-static-style
interference overlay (noise, glitches, red flicker), and the HUD replaces
the numeric telemetry with `--.-` / `[SIN SEÑAL]` (literal on-screen text
— Spanish for "NO SIGNAL", kept as printed by the program). The only
available action is `0` (full reset) — simulating total loss of
communication with the drone after impact.

## 11. `physics_UAV.cpp` — what the console demo runs

An offline validation/report program, no interaction. Actual sequence
(verified by running `make sim`):

1. **Delta-wing mass/inertia** (Block 0) — table of mass, CG, and the
   full inertia tensor.
2. **Convertiplane report** (`reporteConvertiplano()`): mass/inertia with
   vertical nacelles, real dimensions (5.6 m span, wing S≈5.1 m²,
   AR≈6.15), maximum static thrust (T/W=2.0), the airspeed at which the
   propeller lapse zeroes out thrust at full throttle — and a
   **transition table** from `sigma=0°` to `90°` in 15° steps, showing
   how CG and inertia move, how available thrust drops with the typical
   airspeed at each point in the transition, and how control authority
   `l, m, n` splits between rotors and surfaces (section 7.3). This is
   the centerpiece of this report: it numerically shows the classic
   tilt-rotor "power bucket," where propeller thrust collapses just
   before the wing has enough dynamic pressure to hold the weight.
3. **Wing aerodynamics table** (Block 2b) — for airspeeds from 5 to
   32 m/s, the angle of attack that would balance the weight, `CL`,
   `CD`, lift, drag, and the lift/weight ratio (reaches 1.0 from
   ~20 m/s on with this vehicle).
4. **Attitude-conversion demo** — Euler→Quaternion→Euler (round trip, to
   verify there's no loss) and the resulting rotation matrix.
5. **5 s free-flight simulation** of the delta-wing UAV: thrust ≈ weight,
   no torques, integrated with `simulateStep()` (RK4 + the full inertia
   tensor `drone.I_full`, not the reduced form), `dt=0.01 s`. Prints
   `pn, pe, pd, u, phi, theta, p` every 0.5 s and detects/reports the
   instant of ground collision, if it occurs.

## 12. `drone_model.cpp` — interactive visual simulation

OpenGL/GLUT window with the chosen vehicle in 6DOF flight, plus a
**secondary telemetry window** (live oscilloscope) with 5 panels:
altitude, body velocity (u,v,w), attitude (phi,theta,psi), angular rates
(p,q,r), and horizontal position (pn,pe). History in a circular buffer
(~48 s). Fixed integration step at ~60 fps (`DT = 0.016 s`, forward
Euler — section 8).

### 12.1 Vehicle selection

Selection screen on startup:

| Key | Vehicle | Mass | Control distinguishing trait |
|---|---|---|---|
| `1` | **Delta-wing UAV** | 200 kg | longitudinal thrust (+X body); the throttle controls **airspeed**, altitude holds itself |
| `2` | **Quadcopter (X config)** | 142 kg | collective thrust (−Z body); the throttle controls **climb/descent**, moving forward requires tilting the vehicle |
| `3` | **Convertiplane (4 tilting rotors)** | 154.9 kg | thrust vectored along the nacelle axis + real aerodynamics; the only one with variable mass/inertia and **torque-based** control (full Block 4) |

It can also be skipped from the command line
(`./drone_model uav|quad|tilt`), and returned to mid-flight with `v`.
Optional initial `p q r` rates as positional arguments.

All three share **all** of the simulation: the same world, the same
obstacles, the same ground, the same telemetry, and the same Blocks 0-4
of `physics_UAV.h`. Only what defines each vehicle changes — mass/inertia
(Block 0), drag model, and which body axis thrust enters Block 2 on:

| | Delta-wing UAV | Quadcopter | Convertiplane |
|---|---|---|---|
| Thrust | longitudinal, **+X** body axis | collective, **−Z** body axis | vectored, along the nacelle axis: `n=(sin σ,0,−cos σ)` |
| Lift | vertical force = weight (fixed) | none; thrust itself provides it | real (Block 2b): a function of `Va` and `alpha` |
| Drag | calibrated quadratic `aeroDrag` | quadratic `aeroDrag`, near-isotropic | Block 2b: `CD(alpha)·qbar·S` (doesn't use `aeroDrag`) |
| Rotation (p,q,r) | **direct from keyboard**, Block 3 only | **direct from keyboard**, Block 3 only | **full Block 4**: keyboard requests rates, control (7.3) computes torques |
| What the throttle controls | airspeed | climb/descent | total thrust of the 4 rotors (power) |
| Extra state | — | — | tilt angle `sigma` (`[`/`]`/`t`) |
| Startup throttle | 3% (equilibrium at 5 m/s cruise) | 45% (hover, max T/W=2.2) | 50% (hover, T/W=2.0) |
| Initial state | 50 m, cruise at 5 m/s | 50 m, hover | 50 m, hover, vertical nacelles |

### 12.2 Controls

| Key | Action (delta-wing UAV / quad) | Action (convertiplane) |
|---|---|---|
| `← →` | Direct roll rate `p` (±0.02 rad/s) | **Commanded** roll rate `p_cmd` (max 0.60 rad/s) |
| `↑ ↓` | Direct pitch rate `q` (±0.02 rad/s) | **Commanded** pitch rate `q_cmd` (max 0.50 rad/s) |
| `A` / `D` | Direct yaw rate `r` (±0.02 rad/s) | **Commanded** yaw rate `r_cmd` (max 0.40 rad/s) |
| `W` (hold) | Throttle up | Throttle up (power of all 4 rotors) |
| `S` (hold) | Throttle down / reverse | Throttle down |
| `[` / `]` | — | Tilt nacelles toward hover / toward cruise |
| `T` | — | Automatic transition to cruise (90°) |
| `Space` | Pause / resume | |
| `0` | Full reset | (includes `sigma=0`, commanded rates to 0) |
| `V` | Back to selection screen | |
| `R` | Camera reset | |
| `Shift+W` | Wireframe on/off | |
| `+` / `-` | Zoom | |
| `Q` / `Esc` | Quit | |

```
./drone_model                  # selection screen
./drone_model quad              # starts directly in quadcopter
./drone_model tilt               # starts directly in convertiplane
./drone_model uav 0.01 0 0      # delta-wing UAV with initial roll rate
```

## 13. `launch_rockets/` — rocket Lagrangian mechanics

Physically independent project (generalized coordinates `x,y,z` in the
inertial East-North-Up frame, not the body frame), included in the same
repository. Formulates motion via the **Lagrangian** `L = T − V` instead
of body-frame forces like the UAV engine:

```
T = ½ m(t) (x_dot² + y_dot² + z_dot²)        (kinetic energy)
V = m(t)·g·z                                  (potential energy)

Euler-Lagrange equations with non-conservative generalized forces Qi:
  d/dt(dL/dq_dot_i) - dL/dq_i = Qi
  ->  m·x_dot_dot = Fx_thrust + Fx_drag
      m·y_dot_dot = Fy_thrust + Fy_drag
      m·z_dot_dot = -m·g + Fz_thrust + Fz_drag
      m_dot       = -fuel_flow_rate            (during the burn)
```

**Thrust** — rocket equation (Tsiolkovsky), from the specific impulse
`Isp` and the fuel flow rate `mdot = fuel_mass/burn_time`:

```
T_thrust = Isp · g0 · mdot          (fixed direction: thrustDir(theta,phi))
```

plus derived quantities characterizing the launch: dry mass, mass ratio,
**Δv** (`Isp·g0·ln(total_mass/dry_mass)`), and thrust/weight ratio.

**Aerodynamic drag** with a simplified exponential ISA atmosphere:

```
rho(z) = rho0 · e^(-z/H)              H = 8500 m (scale height)
F_drag = -½ · rho(z) · Cd · A · |v| · v_hat
```

**Integration:** the same **classic 4-stage RK4** scheme (`rk4Step()`,
1-2-2-1 weights) as the UAV engine, applied to the 7 state variables
(position, velocity, mass).

Build with CMake (independent of the UAV engine's `Makefile`):

```
cd launch_rockets
cmake -B build && cmake --build build
./build/rocket_sim
```

Controls: `SPACE` launches, `R` resets, `1-7` selects a parameter,
`+`/`-` adjusts it, `WASD`/arrow keys move the free camera, `T` toggles
the rocket-tracking camera, `ESC` quits.

## 14. Known limitations / deliberate simplifications

- **Point-mass collision:** both the ground and the obstacles treat the
  vehicle as its CG — there's no real fuselage/wing collision geometry.
- **No wind or turbulence:** `Va = |v_body|` always (air data with no
  wind component).
- **Delta-wing UAV and quad skip Block 4** in the 3D demo: `p,q,r` are
  direct keyboard input, not the output of integrating torques — a
  deliberate control simplification for those two vehicles (see
  section 8); the convertiplane is the only one that integrates the full
  chain.
- **`aeroDrag` is a drag model, not a lift model:** the delta-wing UAV
  "holds" altitude by fixing the vertical force equal to the weight, not
  deriving it from `CL(alpha)` like the convertiplane.
- **Gimbal lock unmitigated in the integration:** the engine warns via
  `stderr` near `theta=±90°`, but keeps integrating in Euler angles (the
  quaternions are only a conversion/reporting utility, they don't
  replace the integrator).
- **`rot_trans_kinematics`, referenced in the `Makefile`, doesn't exist**
  as a source file in this folder.

## 15. Build and run

```
make view     # builds and opens the interactive 3D simulation (drone_model)
make sim      # builds and runs the console demo (physics_UAV)
make clean    # removes the compiled binaries
```

Requires `clang++` with C++17 support and macOS's `OpenGL`/`GLUT`
frameworks (used only by `drone_model`). For `launch_rockets/`, see
[section 13](#13-launch_rockets--rocket-lagrangian-mechanics) (CMake).

## 16. Reference

Beard, R. W. & McLain, T. W., *Small Unmanned Aircraft: Theory and
Practice*. Ch. 3 (rigid-body equations of motion), Ch. 4 (aerodynamic
forces and moments, propeller model), Appendix E (Aerosonde
coefficients).
