# unmanned — Motor de física UAV (6DOF)

Simulación de la dinámica de vuelo de un dron ala delta, basada en las
ecuaciones de movimiento de cuerpo rígido de **Beard & McLain, "Small Unmanned
Aircraft" (Cap. 3)**. El modelo físico es una única fuente de verdad
(`physics_UAV.h`) que consumen tanto la demo de consola como la visualización
3D, para que ambas simulen exactamente lo mismo.

## Estructura del proyecto

| Archivo | Rol |
|---|---|
| `physics_UAV.h` | Motor de física (header-only): geometría de masa/inercia, cinemática/dinámica traslacional y rotacional, integradores Euler y RK4, conversiones de actitud, colisión con el piso. |
| `simulate_step.h` | `simulateStep()` — variante de RK4 que usa el **tensor de inercia completo** (con productos de inercia Ixy, Ixz, Iyz), en vez de la forma reducida que asume simetría XZ. |
| `physics_UAV.cpp` | Demo de consola: imprime propiedades de masa/inercia, corre una simulación de 5 s y muestra la trayectoria en tabla. |
| `drone_model.cpp` | Visualización 3D interactiva (OpenGL/GLUT): vuelo controlable por teclado (UAV ala delta o quadcopter) + ventana de telemetría en vivo. |
| `Makefile` | Targets de compilación y ejecución. |
| `euler_to_quaternions.jpeg`, `quaternion_to_rotationMatrix.jpeg` | Referencia de las fórmulas de conversión de actitud implementadas en `physics_UAV.cpp`. |

> Nota: el `Makefile` también define un target `rot_trans_kinematics`, pero
> `rot_trans_kinematics.cpp` no está presente en la carpeta actualmente — ese
> target no compila.

## El modelo físico (`physics_UAV.h`)

Estado completo del UAV, 12 componentes (`UAVState`):

- **Posición inercial NED** (`pn, pe, pd`) — `pd` positivo hacia abajo.
- **Velocidad en marco cuerpo** (`u, v, w`).
- **Actitud en ángulos de Euler** (`phi` roll, `theta` pitch, `psi` yaw).
- **Tasas angulares en marco cuerpo** (`p, q, r`).

Ejes cuerpo: X hacia la nariz, Y ala derecha, Z hacia abajo.

Las ecuaciones se organizan en 5 bloques:

- **Bloque 0 — Geometría de masa/inercia.** Arma la masa, el centro de
  gravedad y el tensor de inercia completo a partir de componentes
  geométricos reales (no valores inventados), con el teorema de ejes
  paralelos para trasladar cada inercia local al CG conjunto
  (`combinarComponentes()`). Hay dos vehículos:
  - `buildDeltaWingDrone()` — fuselaje (cilindro), ala delta (placa
    triangular), compartimento delantero (caja) y motor trasero (masa
    puntual). **200 kg.**
  - `buildQuadcopter()` — cuerpo central (caja), 4 brazos (barras a 45° del
    eje `x`, config X), 4 motores (masas puntuales), 4 rotores (discos) y
    tren de aterrizaje. **142 kg.**

  Ambos cumplen por construcción la simetría respecto al plano `XZ`
  (`Ixy = Iyz = 0`) que asume el Bloque 4; si alguna vez dejaran de
  cumplirla, `combinarComponentes()` lo avisa por `stderr`.
- **Bloque 1 — Cinemática traslacional.** Convierte velocidad en marco cuerpo
  a tasas de posición inercial (matriz de rotación cuerpo→inercial).
- **Bloque 2 — Dinámica traslacional.** Convierte fuerzas en marco cuerpo
  (`fx, fy, fz`) a aceleración de `u, v, w`.
- **Bloque 3 — Cinemática rotacional.** Convierte tasas angulares `p, q, r`
  a tasas de los ángulos de Euler (con aviso de *gimbal lock* cerca de
  `theta = ±90°`).
- **Bloque 4 — Dinámica rotacional.** Convierte torques (`l, m, n`) a
  derivadas de `p, q, r`. Dos variantes:
  - `rotational_dynamics()` en `physics_UAV.h`: forma reducida con
    constantes Γ precalculadas, asume simetría de masa respecto al plano XZ.
  - `rotational_dynamics_full()` en `simulate_step.h`: tensor de inercia
    3×3 completo (invierte la matriz en cada paso), para geometrías sin esa
    simetría.

**Integración:** `integrate_euler()` (1er orden) e `integrate_rk4()` /
`simulateStep()` (Runge-Kutta 4, mismo esquema de 4 etapas k1-k4 con pesos
1-2-2-1).

**Otras utilidades:**
- Conversión Euler ↔ Cuaternión ↔ Matriz de rotación (`physics_UAV.cpp`).
- Colisión con el piso: `checkGroundCollision()` / `resolveGroundCollision()`
  — plano rígido en `pd = 0`, respuesta inelástica (anula velocidad, no hay
  rebote).
- Colisión con obstáculos fijos: `checkObstacleCollision()` /
  `resolveObstacleCollision()` — ver [Escenario: obstáculos sólidos y pérdida
  de señal](#escenario-obstáculos-sólidos-y-pérdida-de-señal).

## Escenario: obstáculos sólidos y pérdida de señal

Además del piso, el motor modela **obstáculos fijos** — bloques
rectangulares (`BoxObstacle` en `physics_UAV.h`). Son restricciones duras,
sin deformación ni rebote, con la misma simplificación que el piso: el dron
se trata como un punto (su CG).

`drone_model.cpp` define un campo de **20 bloques** (`g_obstacles[]`)
repartidos por el plano, dentro de los ±200 m de la grilla, con alturas de
**6 a 50 m**. El primero es el bloque de referencia 100 m al norte del origen
(`pnCenter=100, peCenter=0`, 2×3 m en planta y 6 m de altura), justo sobre el
eje de crucero inicial; el resto obliga a elegir a qué altura se cruza cada
zona en vez de esquivar solo de frente. Ningún par se solapa y queda libre un
radio de ~33 m alrededor del origen, para que el vehículo nunca aparezca
dentro de un bloque al arrancar o reiniciar.

Cada bloque se pinta con un degradado según su altura (ámbar los bajos,
violeta frío los altos) para poder estimarla de un vistazo; el bloque contra
el que se chocó se marca en rojo.

- **`checkObstacleCollision()`** — test AABB (caja alineada a ejes): compara
  `pn, pe, pd` del dron contra el rango del bloque en los 3 ejes.
- **`resolveObstacleCollision()`** — respuesta inelástica en 3 ejes: calcula
  la penetración en `pn`, `pe` y altitud, y empuja al dron a la cara más
  cercana según el eje de **menor penetración** (MTV — *minimum translation
  vector*), igual que `resolveGroundCollision()` clava `pd` al piso pero
  extendido a una caja. Esto es lo que hace al bloque realmente sólido: sin
  el reposicionamiento, frenar la velocidad en el frame del impacto no
  alcanza, porque el empuje/gravedad la reconstruyen en el siguiente paso y
  el dron termina atravesando el bloque.

En `drone_model.cpp`, el mundo se traslada con la convención NED→GL
`pn → -Z` (la nariz del dron apunta a `-Z`), así que los obstáculos —
definidos con `pn > 0` — se dibujan **por delante** del dron a medida
que avanza, nunca detrás.

**Al chocar** (contra el piso o contra un obstáculo, bandera `g_crashed`):
- La física se congela (`step()` deja de integrarse).
- Los controles de vuelo (`w/s`, flechas, `A`/`D`) quedan bloqueados.
- La ventana muestra una superposición de **interferencia** (ruido tipo
  "sin señal" de TV, franjas de glitch, parpadeo rojo) sobre la escena.
- El HUD reemplaza la telemetría numérica por `--.-` / `[SIN SEÑAL]`.
- La única acción posible es `0` (reset completo) — simula la pérdida total
  de comunicación con el dron tras el impacto.

## `physics_UAV.cpp` — demo de consola

Ejecuta un escenario de vuelo libre (empuje ≈ peso, sin torques) de 5 s con
`simulateStep()` (tensor de inercia completo) y muestra `pn, pe, pd, u, phi,
theta, p` cada 0.5 s. También corre una demo de ida y vuelta
Euler → Cuaternión → Euler y muestra la matriz de rotación resultante.
Detecta e imprime el instante de colisión contra el piso, si ocurre.

## `drone_model.cpp` — simulación visual interactiva

Ventana principal en OpenGL/GLUT con el vehículo elegido en vuelo 6DOF, más una
**ventana secundaria de telemetría** (osciloscopio en vivo) con 5 paneles:
altitud, velocidad body (u,v,w), actitud (phi,theta,psi), tasas angulares
(p,q,r) y posición horizontal (pn,pe). Guarda hasta ~48 s de historial en un
buffer circular.

Paso de integración fijo a ~60 fps (`DT = 0.016 s`).

### Elección de vehículo

Al arrancar aparece una **pantalla de selección**:

| Tecla | Vehículo |
|---|---|
| `1` | **UAV ala delta** — `buildDeltaWingDrone()`, 200 kg |
| `2` | **Quadcopter (config X)** — `buildQuadcopter()`, 142 kg |

También se puede saltear desde la línea de comandos
(`./drone_model uav` / `./drone_model quad`), y volver a la pantalla en pleno
vuelo con `v`.

Los dos vehículos comparten **toda** la simulación: el mismo mundo, los
mismos obstáculos, el mismo piso, la misma telemetría y los mismos Bloques
0-4 de `physics_UAV.h`. Lo único que cambia es lo que define a cada vehículo
— sus propiedades de masa (Bloque 0), sus coeficientes de arrastre y **sobre
qué eje del cuerpo entra el empuje** al Bloque 2:

| | UAV ala delta | Quadcopter |
|---|---|---|
| Masa / inercia | 200 kg; `Ixx=16` `Iyy=188` `Izz=202` | 142 kg; `Ixx=72` `Iyy=72` `Izz=140` |
| Empuje | longitudinal, eje **+X** cuerpo (hélice) | colectivo, eje **−Z** cuerpo (4 rotores) |
| Sustentación | fuerza vertical = peso (el ala la sostiene) | no hay: la da el propio empuje |
| Qué controla la palanca | **velocidad** (la altitud se sostiene sola) | **ascenso/descenso** |
| Cómo se avanza | acelerando | **inclinando** el vehículo |
| Palanca de arranque | 3% (equilibrio a 5 m/s) | 45% (hover; T/W máx = 2.2) |
| Estado inicial | 50 m, crucero a 5 m/s | 50 m, vuelo estacionario |

El quad sale de las mismas ecuaciones, sin tocarlas: al inclinarlo, la
proyección del empuje colectivo sobre el marco inercial es la que lo empuja
hacia adelante. Su geometría de masa se arma con las mismas piezas del Bloque
0 (cuerpo + 4 brazos + 4 motores + 4 rotores + tren), usando dos ayudas
nuevas: `inerciaDiscoZ()` (disco delgado, para los rotores) y `rotarZ()`
(lleva la inercia de cada brazo, calculada sobre su propio eje, a los ejes
del cuerpo).

**Modelo de control usado en esta demo** (simplificado respecto al modelo
completo de `physics_UAV.h`):
- `p, q, r` (roll/pitch/yaw rate) se comandan **directamente** con el teclado
  y se integran vía Bloque 3 — no pasan por torques ni por el Bloque 4.
- El empuje (`W`/`S`) sí recorre la cadena completa Bloque 2 (dinámica) →
  Bloque 1 (cinemática): acelera la velocidad body y eso mueve la posición
  inercial.
- En el ala delta el empuje vertical se fija igual al peso (altitud
  "sostenida" salvo por pitch/roll); en el quad no hay tal cosa, el empuje
  colectivo es lo único que lo sostiene. En ambos casos la gravedad se
  proyecta al marco cuerpo en cada paso según la actitud actual.
- Colisión contra el piso sólido o contra un obstáculo fijo: el dron queda
  restringido en el borde del choque (no lo atraviesa) y la simulación
  entra en modo "señal perdida" — ver [Escenario: obstáculos sólidos y
  pérdida de señal](#escenario-obstáculos-sólidos-y-pérdida-de-señal).

### Controles

| Tecla | Acción |
|---|---|
| `← →` | Roll rate `p` (±0.02 rad/s) |
| `↑ ↓` | Pitch rate `q` (±0.02 rad/s) |
| `A` / `D` | Yaw rate `r` (±0.02 rad/s) |
| `W` (mantener) | Subir palanca (ala delta: acelerar; quad: ascender) |
| `S` (mantener) | Bajar palanca (ala delta: frenar / reversa; quad: descender) |
| `Espacio` | Pausa / reanuda |
| `0` | Reset completo (posición, velocidad, actitud, empuje, telemetría) |
| `V` | Volver a la pantalla de selección de vehículo |
| `R` | Reset de cámara |
| `Shift+W` | Wireframe on/off |
| `+` / `-` | Zoom |
| `Q` / `Esc` | Salir |

Se le pueden pasar `p q r` iniciales (rad/s) como argumentos, con el vehículo
opcionalmente delante:

```
./drone_model                  # pantalla de selección
./drone_model quad             # arranca directo en quadcopter
./drone_model uav 0.01 0 0     # ala delta con roll rate inicial
./drone_model 0.01 0 0         # selección + roll rate inicial
```

## Compilar y ejecutar

```
make view     # compila y abre la simulación 3D interactiva (drone_model)
make sim      # compila y corre la demo de consola (physics_UAV)
make clean    # borra los binarios compilados
```

Requiere `clang++` con soporte de C++17 y los frameworks `OpenGL`/`GLUT` de
macOS (usados solo por `drone_model`).

## `launch_rockets/` — simulación de lanzamiento de cohetes

Proyecto aparte dentro de este mismo repositorio: simulación de dinámica
Lagrangiana de un cohete, con vista en primera persona (OpenGL/GLUT) y
controles de cámara independientes del motor de física del UAV. Se compila
con CMake:

```
cd launch_rockets
cmake -B build && cmake --build build
./build/rocket_sim
```

## Referencia

Beard, R. W. & McLain, T. W., *Small Unmanned Aircraft: Theory and
Practice*, Cap. 3 (ecuaciones de movimiento de cuerpo rígido).
