# rockets_sim — Lanzamiento de cohetes en primera persona (mecánica Lagrangiana)

Simulador interactivo de un lanzamiento de cohete de un solo motor, vista en
**primera persona** desde la plataforma de lanzamiento (estilo cámara libre
tipo shooter), con un panel de parámetros editable en vivo y telemetría de
vuelo en tiempo real. La física se deduce con el **formalismo Lagrangiano**
(coordenadas generalizadas, no fuerzas en marco cuerpo) e integra con
**Runge-Kutta de 4º orden**.

Usa el mismo motor físico que [`unmanned/launch_rockets/`](../unmanned/launch_rockets/)
(`physics.h`/`physics.cpp` son prácticamente idénticos byte a byte — ver
[§6](#6-relación-con-unmannedlaunch_rockets)); la diferencia es el frontend:
acá la cámara es libre y en primera persona, con un HUD de diseño de misión
(T/W, *mass ratio*, Δv) en vez de la ventana de telemetría tipo osciloscopio
de `unmanned/`.

![Plataforma de lanzamiento con los paneles de parámetros y propulsión derivada, cohete aún en tierra](rocket_launch1.png)
*Pantalla de configuración: panel de parámetros de lanzamiento (izquierda,
arriba) y panel de propulsión derivada (izquierda, abajo) — T/W=3.33 en
verde indica que el cohete puede despegar.*

![Cohete en fase de quema (BURN PHASE), con telemetría de vuelo en vivo](rocket_launch2.png)
*En vuelo: panel de telemetría (derecha) con T+, altitud, velocidad, rango
horizontal, masa restante y fase de vuelo — T+1.5 s, 24.8 m de altitud,
36.6 m/s, todavía quemando combustible.*

## 1. Coordenadas generalizadas y Lagrangiano

Estado del cohete en el marco inercial **Este-Norte-Arriba** (no marco
cuerpo, a diferencia del motor 6DOF de `unmanned/`):

```
q  = (x, y, z)         posicion [m]
q̇  = (ẋ, ẏ, ż)         velocidad [m/s]
m(t)                    masa, decrece mientras hay combustible [kg]
```

```
T = ½ m(t) (ẋ² + ẏ² + ż²)      energia cinetica
V = m(t) · g · z                 energia potencial
L = T − V                        Lagrangiano
```

## 2. Ecuaciones de Euler-Lagrange

Con fuerzas generalizadas no conservativas `Qi` (empuje + arrastre):

```
d/dt(∂L/∂q̇i) − ∂L/∂qi = Qi

→  m·ẍ = Fx_empuje + Fx_arrastre
→  m·ÿ = Fy_empuje + Fy_arrastre
→  m·z̈ = −m·g + Fz_empuje + Fz_arrastre
   ṁ    = −flujo_combustible        (solo durante la quema)
```

## 3. Fuerzas y parámetros derivados (`physics.h` / `physics.cpp`)

| Fuerza | Modelo |
|---|---|
| Gravedad | `Fg = (0, 0, −m·g)` |
| Empuje | `Ft = thrustDir(θ,φ) · T`, activo mientras `t < burn_time` y queda combustible |
| Arrastre aerodinámico | `Fd = −½·ρ(z)·Cd·A·\|v\|·v̂` |
| Pérdida de masa | `ṁ = −fuel_mass / burn_time` (consumo lineal durante la quema) |

**Dirección del empuje** — coordenadas esféricas estándar de física,
`θ` polar desde el cenit (+Z) y `φ` azimutal desde el Este (+X):

```
thrustDir(θ,φ) = ( sin θ·cos φ,  sin θ·sin φ,  cos θ )
```

**Empuje a partir del impulso específico** (ecuación del cohete) en vez de
pedir la fuerza directamente — así el parámetro editable es `Isp`, una
cifra de mérito real del motor, no un número arbitrario de Newtons:

```
mdot      = fuel_mass_kg / burn_time_s
thrustN   = Isp_s · g₀ · mdot
deltaV    = Isp_s · g₀ · ln(total_mass_kg / dry_mass_kg)     (Tsiolkovsky)
T/W       = thrustN / (total_mass_kg · g₀)
massRatio = total_mass_kg / dry_mass_kg
```

Estas 4 cantidades derivadas son exactamente el panel **PROPULSION
(derived)** del HUD (sección 5), con código de color: T/W en rojo si
`<1` (no despega), amarillo `1–1.5` (marginal), verde `>1.5`; *mass ratio*
en naranja si `<2`, amarillo `2–4`, verde `>4`.

**Atmósfera** — densidad exponencial tipo ISA (igual que
`unmanned/launch_rockets/`), afecta el arrastre con la altitud:

```
rho(z) = 1.225 · e^(−z / 8500)        (kg/m³, z en metros)
```

## 4. Integración: Runge-Kutta 4

Mismo esquema clásico de 4 etapas y pesos 1-2-2-1 que el resto de los
simuladores de este repositorio (`unmanned/`, `aerodynamical_controls/`),
aplicado aquí a las 7 variables de estado (posición, velocidad, masa):

```
k1 = f(t,        y)
k2 = f(t+h/2,    y + h/2·k1)
k3 = f(t+h/2,    y + h/2·k2)
k4 = f(t+h,      y + h·k3)

y_{n+1} = y_n + (h/6)(k1 + 2·k2 + 2·k3 + k4)
```

4 subpasos por frame de render (≈240 Hz efectivo de integración a 60 fps de
pantalla). El impacto contra el piso se detecta por `rocket.pos.z < 0`,
momento en el que la simulación se congela y el HUD muestra `** IMPACT **`.

## 5. HUD y controles (`main.cpp`)

Tres paneles en pantalla, consumiendo directamente el estado de
`LaunchParams` / `RocketState`:

- **`LAUNCH PARAMETERS`** (arriba izquierda) — los 8 parámetros editables
  de la misión, con su rango válido:

  | # | Parámetro | Paso | Rango | Default |
  |---|---|---|---|---|
  | 1 | Theta (ángulo polar desde el cenit) | 5° | 1–179° | 20° |
  | 2 | Phi (azimut desde el Este) | 10° | 0–359° | 255° |
  | 3 | Isp | 10 s | 100–500 s | 100 s |
  | 4 | Burn time | 1 s | 1–120 s | 2 s |
  | 5 | Total mass | 10 kg | 10–5000 kg | 12 kg |
  | 6 | Fuel mass | 5 kg | 1–4990 kg | 0.8 kg |
  | 7 | Drag Cd | 0.05 | 0.05–2.0 | 0.5 |
  | 8 | Area | 0.005 m² | 0.005–0.5 m² | 0.05 m² |

- **`PROPULSION (derived)`** (abajo izquierda) — Thrust, T/W, Mass Ratio,
  Δv ideal y la fórmula de densidad atmosférica en uso (sección 3). Si
  T/W < 1 antes de lanzar, aparece una advertencia central
  `EMPUJE INSUFICIENTE  T/W < 1` con el empuje mínimo necesario.
- **`TELEMETRY`** (arriba derecha, solo durante/después del vuelo) — `T+`
  (tiempo transcurrido), altitud, velocidad, rango horizontal desde la
  plataforma, masa restante, y la **fase de vuelo**: `BURN PHASE`
  (quemando combustible) → `COAST PHASE` (en inercia, motor apagado) →
  `** IMPACT **` (al tocar el piso).

| Tecla | Acción |
|---|---|
| `SPACE` | Lanzar |
| `R` | Reset — vuelve a la configuración |
| `1`–`8` | Seleccionar parámetro |
| `+` / `-` | Ajustar el parámetro seleccionado (paso de la tabla de arriba) |
| `ENTER` | Editar el valor del parámetro seleccionado directamente (teclado numérico) |
| `W A S D` | Mover la cámara |
| Flechas / arrastrar con el mouse | Rotar la vista |
| `T` | Alternar cámara de seguimiento automático del cohete |
| `ESC` | Salir |

Con los valores por defecto el cohete sube ~1–2 km y traza una curva
balística visible desde el suelo.

## 6. Relación con `unmanned/launch_rockets/`

Este proyecto y [`unmanned/launch_rockets/`](../unmanned/launch_rockets/)
comparten el mismo motor físico (`physics.h` es idéntico byte a byte;
`physics.cpp` difiere en una sola línea de comentario) — es la misma
mecánica Lagrangiana descrita en la [sección 13 del README de
`unmanned/`](../unmanned/README.md#13-launch_rockets--mecánica-lagrangiana-de-un-cohete).
Lo que cambia es el programa que lo envuelve: `unmanned/launch_rockets/`
es un visor con cámara orbital/de seguimiento más simple; `rockets_sim/`
es una consola de diseño de misión en primera persona, con el panel de
propulsión derivada (T/W, *mass ratio*, Δv) y la validación de
despegue (`T/W < 1`) que el otro no tiene.

## Compilar y ejecutar

```bash
cmake -B build && cmake --build build
./build/rocket_sim
```

Requiere macOS con OpenGL + GLUT (incluidos por defecto en el SDK); en
Linux, `freeglut3-dev`.

## Estructura del proyecto

| Archivo | Rol |
|---|---|
| `CMakeLists.txt` | Build system. |
| `physics.h` | Structs `Vec3`, `RocketState`, `LaunchParams` (con los derivados de §3) + declaraciones. |
| `physics.cpp` | Euler-Lagrange (`computeDerivs`) + integrador RK4 (`rk4Step`). |
| `main.cpp` | Ventana GLUT, cámara en primera persona, HUD (parámetros/propulsión/telemetría), renderizado 3D, manejo de teclado. |
| `rocket_launch1.png`, `rocket_launch2.png` | Capturas reales de la simulación, usadas arriba en este README. |

## Próximos pasos posibles

- Variación de densidad del aire con altitud ya modelada (ISA); falta
  variar también la gravedad o la temperatura con la altura si se
  necesita más fidelidad.
- Etapas múltiples (separación de cohetes).
- Visualizar la curva de energía `T`, `V`, `H = T + V` en el HUD.
- Exportar la trayectoria a CSV para análisis externo.
- Reemplazar GLUT por GLFW + Dear ImGui para una UI más rica.

## Referencia

Mecánica Lagrangiana estándar (Goldstein, *Classical Mechanics*) aplicada a
un cuerpo de masa variable; ecuación del cohete de Tsiolkovsky para el
empuje a partir del impulso específico.
