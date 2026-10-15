# aerodynamical_controls — what was done and how to run it

Date: 2026-09-02 (aeroelasticity added 2026-09-03)

The whole module lives in `simulacion_alerones/`. It's an interactive
simulator of the **attitude** dynamics of a rocket with four cruciform
tail fins (Blakelock Fig. 8-2 convention). The full documentation —
derivations, formulas, key-binding table — is in
`simulacion_alerones/README.md`.

---

## How to run it

```bash
cd simulacion_alerones

make run        # interactive 3D window (OpenGL/GLUT)
make test       # 55 checks, no graphics
make paridad    # C++ vs Python, byte for byte
make figuras    # generates figuras/*.csv for the write-up
make help       # all targets
```

Requires `clang++` with C++17 and OpenGL/GLUT (bundled with the SDK on
macOS; `freeglut3-dev` on Linux). The Python prototype needs `numpy` and
`matplotlib`.

### Headless mode (no graphics)

A `stdin` REPL that ignores `#` comments, so it can be fed any file. There
is one per theory section:

```bash
./alerones --headless < scripts/desacoplo.txt        # §2  symmetry decoupling
./alerones --headless < scripts/amortiguamiento.txt  # §4  with and without damping
./alerones --headless < scripts/aeroelasticidad.txt  # §5  flexible fin
./alerones --headless < scripts/actuador.txt         # §6  actuator lag and rate limit
./alerones --headless < scripts/autopiloto.txt       # §6  attitude step response
./alerones --headless < scripts/regresion.txt        # regression trace
```

`help` inside the REPL lists the available commands.

### Main keys in graphics mode

`A` autopilot · `C` hold · `D` damping · `f` flexible/rigid fin ·
`F` exaggerate deformation · `S` servo · arrow keys and `Q`/`E` for
command input (or references if the autopilot is active) · `1`…`4` +
`+`/`-` single fin · `SPACE` pause · `R` reset · `,`/`.` speed ·
`T` tail view · `ESC` quit.

---

## What was done (2026-09-02)

Starting state: the physics engine, rendering, and 23 tests already
worked. What was missing, and got completed:

**1. Documentation.** There was no `README.md`, and the code referenced
§2, §7, and §8 of a document that didn't exist. It now exists, with
§1–§9 and an explicit scope section ("What's deliberately missing").

**2. Aerodynamic damping (§4)** — this was the real physics gap. Each fin
sees the incidence induced by `ω × r_i`. Without it, a constant deflection
produced an angular velocity that grew linearly forever. Central result:
**the derivative matrix is diagonal in the same irrep basis that
diagonalizes the command** (`L_p`, `M_q`, `N_r`, zero cross terms), and
`ω` converges to `V·u/arm`.

**3. Actuator dynamics (§6)** — `delta_cmd` and `delta` are distinct
states; first-order lag plus a rate limit.

**4. Autopilot (§6)** — three SISO PIDs, one per irrep, tuned by pole
placement on the linearized model, so they retune themselves automatically
when `V` or the geometry changes. The yaw gains come out **negative**:
that's `N_du < 0` under the Blakelock convention, not a typo.

**5. Inertia tensor with `Ixz`** (0 by default), to show the *inertial*
roll-yaw coupling, distinct from the aerodynamic one.

**6. Bugs closed** — `set S_fin`/`b_fin` weren't recomputing
`AR → CL_delta`; incomplete `.PHONY` in the Makefile; missing
`.gitignore`; the Python REPL didn't have the same commands as the C++
one.

**7. Tests: 23 → 55**, and the `alerones_py/main.py` prototype ported in
full.

---

## What was done (2026-09-03): flexible fins

Previously the fin was an infinitely rigid plate that went exactly where
the servo commanded. Now each fin is a **two-degree-of-freedom system
excited by its own aerodynamic force** (§5 of the README):

- **Torsion** about the pivot. This is the one that matters: it feeds
  into the angle of attack and closes a loop — the load twists the fin,
  and the twisted fin changes the load. That produces a single number,
  the **aeroelastic efficiency** `eta = 1/(1 + qbar/qbar_car)`, which
  tells you what fraction of the command actually reaches the air.
- **Bending** of the tip, with the exact deflected shape of a cantilever.
  It doesn't change the incidence (the fin stays straight): it's the mode
  you *see*.
- The `e_ea` parameter (distance from the pivot to the aerodynamic
  center) splits the problem in two: with the aerodynamic center
  **behind** the pivot, command effectiveness is lost at high speed; with
  the pivot behind, the fin **diverges** above a certain dynamic
  pressure.
- The autopilot is tuned on `eta · M_du`, so it compensates only for the
  loss of authority.

Two things worth knowing:

- **The default values don't exaggerate the effect.** At 30 m/s, 0.4% of
  the command is lost: it exists and isn't noticeable. At 486 m/s, half
  of it is lost. The takeaway is that *aeroelasticity is a high-speed
  phenomenon*, and overstating it would misrepresent which regime it
  belongs to. To see it at low speed, soften the fin: `set K_theta 5`.
- **The drawing does exaggerate it**, because the real deflection is
  tenths of a millimeter. The scale factor is shown in the HUD and the
  `F` key cycles it (×1 … ×150).

The structural modes (37 and 65 Hz) are now what sets the integration
step: at `dt = 10 ms`, RK4 diverges to NaN. There's a guard
(`structural_cfl`) that warns in the HUD, at startup, and after any `set`
that worsens the number.

**Tests: 55 → 79.** Cases 15–18 cover static efficiency, falloff with
dynamic pressure, divergence, and the step-size limit.

---

## Verification

- 79/79 in both C++ and Python.
- `make paridad` confirms both implementations produce the exact same
  trace **byte for byte**, across the 79 cases and the six scripts. These
  are two independent implementations: they can't share a numerical bug.
- Clean compile with `-Wall`, no warnings.

Two print-formatting adjustments were needed for exact parity, both
explained in §7 of the README: the antisymmetric combinations (`u_roll`,
`L`) are total cancellation of four nearly-equal numbers and, once
converged, land at ~1e-31 with the sign set by rounding (flushed to zero
below 1e-9); and `qbar` is printed with 4 decimals because `ρV²/2` lands
on exact ties (`x.xxx5`) that C++ and Python round in opposite
directions.

---

## Scope: what it does NOT do

Simulates **attitude only**. There's no position, translational velocity,
gravity, or thrust: `V` is a parameter, not a state. For the full 6DOF
flight, see the sibling module `../unmanned/`. It also doesn't model fin
drag, fuselage damping, compressibility, or stall hysteresis. The
aeroelasticity in §5 uses quasi-steady aerodynamics: it gets the
efficiency and the divergence right, since those are static, but not a
reliable flutter speed. The detail is at the end of the README.
