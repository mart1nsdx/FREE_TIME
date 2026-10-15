# simulacion_alerones — Control aerodinámico de un cohete con aletas cruciformes

Simulador interactivo de la dinámica de actitud de un cohete gobernado por
**cuatro aletas traseras en cruz**. El hilo conductor del módulo es un
resultado de teoría de grupos: la simetría del arreglo diagonaliza a la vez la
matriz de mando y la de amortiguamiento, y de ahí sale un desacoplo *exacto*
entre alabeo, cabeceo y guiñada. Ese desacoplo es lo que permite pilotar el
cohete con tres lazos SISO independientes en lugar de un controlador MIMO.

Convención de ejes y numeración de aletas: **Blakelock, Fig. 8-2**. Las
inercias por defecto son las de `buildDeltaWingDrone()` en
[`unmanned/physics_UAV.h`](../../unmanned/physics_UAV.h), para que los dos
módulos hablen del mismo vehículo.

## Estructura

| Archivo | Rol |
|---|---|
| `fisica_alerones.hpp` / `.cpp` | Motor físico: mixing, aerodinámica de la aleta, amortiguamiento, aeroelasticidad, actuador, Euler + cuaternión, RK4 y autopiloto. Sin dependencias gráficas. |
| `alerones_graficos.hpp` / `.cpp` | Render 3D (OpenGL/GLUT), vista de cola, HUD y traducción teclado → comandos. Solo dibuja. |
| `main.cpp` | Arranque: modo gráfico o REPL `--headless`. |
| `tests.cpp` | Los 18 casos de validación de §7 (79 comprobaciones). |
| `alerones_py/main.py` | Prototipo de referencia en Python (§8): mismo modelo, animación matplotlib, REPL y los mismos 79 casos. |
| `scripts/*.txt` | Guiones para el modo headless, uno por sección teórica. |
| `Makefile` | `make help` lista todos los targets. |

**Alcance deliberado:** este módulo simula **solo la actitud**. No hay posición
ni velocidad de traslación: `V` es un parámetro, no un estado, y no hay
gravedad, empuje ni resistencia. Para el vuelo 6DOF completo está el módulo
hermano [`unmanned/`](../../unmanned/README.md), que integra las ecuaciones de
Beard & McLain. Aquí el cohete es un banco de ensayos de actitud: es lo que
hace falta para ver el desacoplo con claridad, y nada más.

---

## §1 — El modelo de un vistazo

Estado: cuaternión de actitud `q`, velocidad angular `omega = (p, q, r)` en
ejes cuerpo, y las cuatro deflexiones `delta`. La cadena completa de un paso:

```
  referencias           canales                deflexiones            momentos
  de actitud            virtuales              por aleta

  phi_ref  ──┐        ┌─ u_roll  ─┐          ┌─ d1 ─┐              ┌─ L ─┐
  theta_ref ─┼─ §6 ──►├─ u_pitch ─┼── §2 ───►├─ d2 ─┼─ §3,§4,§5 ──►├─ M ─┼─► RK4
  psi_ref  ──┘  PID   └─ u_yaw   ─┘  mixing  ├─ d3 ─┤   aero       └─ N ─┘  Euler
                 ▲                           └─ d4 ─┘  + servo §6      │
                 │                              ▲      + aleta         │
                 └──────── actitud ─────────────┴──────  flexible ──────┘
```

Ejes cuerpo: **x adelante, y derecha, z abajo**. Las aletas van *detrás* del
CG, en `x_t = -l_t` con `l_t > 0`:

| Aleta | Posición | Offset radial | Fuerza para `delta > 0` |
|---|---|---|---|
| `d1` | derecha | `(x_t, +d, 0)` | `+z` (abajo) |
| `d2` | izquierda | `(x_t, -d, 0)` | `+z` (abajo) |
| `d3` | arriba | `(x_t, 0, -d)` | `+y` (derecha) |
| `d4` | abajo | `(x_t, 0, +d)` | `+y` (derecha) |

Parámetros por defecto (`params_default()`): aleta de 0.25 × 0.16 m
(`S = 0.04 m²`), `l_t = 1.75 m`, `d = 0.285 m`, perdida a ±15°, `V = 30 m/s`,
`rho = 1.225`, `dt = 2 ms`.

---

## §2 — Simetría V4 y desacoplo

El arreglo cruciforme tiene grupo de simetría **V4 = Z2 × Z2** (el grupo de
Klein), generado por dos reflexiones:

- `sigma_v` — plano *xz*, intercambia el par horizontal `d1 ↔ d2`
- `sigma_h` — plano *xy*, intercambia el par vertical `d3 ↔ d4`

Las cuatro deflexiones forman una representación *permutacional* de V4. Esa
representación es reducible, y descomponerla en irreps es un cambio de base:

```
  / d1 \   / 1  0   1 \ / u_pitch \          u_pitch = (d1 + d2)/2
  | d2 | = | 1  0  -1 | | u_yaw   |          u_yaw   = (d3 + d4)/2
  | d3 |   | 0  1   1 | \ u_roll  /          u_roll  = (d1 - d2 + d3 - d4)/4
  \ d4 /   \ 0  1  -1 /
```

> **La matriz de mixing no es un generador del grupo.** Es el cambio de base
> que diagonaliza la acción del grupo. Los eigenvectores de autovalor +1
> (modos simétricos) dan cabeceo y guiñada; los de autovalor −1 (modo
> antisimétrico) dan alabeo.

Los momentos que salen son entonces, con `k = qbar · S · CL_delta`:

```
  L =  d   (f1 - f2 + f3 - f4)  =  +4 k d   · u_roll      ← solo el modo antisimétrico
  M =  l_t (f1 + f2)            =  +2 k l_t · u_pitch     ← solo el par horizontal
  N = -l_t (f3 + f4)            =  -2 k l_t · u_yaw       ← solo el par vertical
```

**El signo de `N` es la convención de Blakelock**, no una errata: con
`u_yaw > 0` las aletas verticales empujan la cola hacia `+y`, así que la nariz
se va hacia `-y` y el momento de guiñada es negativo. Ese signo reaparece más
tarde en las ganancias del autopiloto, que salen negativas en el canal de
guiñada (§6).

Valores por defecto (`gains` en el modo headless):

```
k = qbar*S*CL_delta = 47.7029 N/rad
autoridad   L_du   +54.3813   M_du  +166.9601   N_du  -166.9601  N*m/rad
```

El alabeo tiene **tres veces menos autoridad** que los otros dos ejes, porque
su brazo es el offset radial `d = 0.285 m` en vez del brazo de cola
`l_t = 1.75 m`. Es la razón de que el autopiloto lo sintonice más rápido: hay
que compensar con ancho de banda lo que falta de palanca.

---

## §3 — Aerodinámica de la aleta

Aleta *all-moving* (gira la superficie entera), así que la deflexión **es** el
ángulo de ataque y `CL_delta = CL_alpha`. Para el alargamiento bajo típico de
una aleta de cohete se usa la fórmula de **Helmbold**, que interpola entre la
teoría de línea sustentadora (AR alto) y la de ala esbelta (AR bajo):

```
  AR = b² / S                    CL_alpha = 2·pi·AR / (2 + sqrt(AR² + 4))
```

Con los valores por defecto, `AR = 1.5625` y `CL_delta = 2.1634 1/rad`.

La fuerza satura por pérdida:

```
  f_i = qbar · S · CL_delta · clamp(delta_i, ±delta_stall)
```

Dos decisiones que conviene subrayar:

1. **El clamp vive en `fin_forces()`, no en el mixing.** Saturar los canales
   virtuales rompería el desacoplo; saturar las fuerzas no.
2. **El clamp se aplica a la incidencia *efectiva*** (§4), no a la comandada.
   Una aleta puede entrar en pérdida por la rotación del propio cohete aunque
   el mando sea pequeño.

Post-pérdida la fuerza se mantiene constante. Es un modelo deliberadamente
simple: sirve para que la saturación exista y sea visible, no para predecir el
comportamiento real de un perfil en pérdida profunda.

---

## §4 — Amortiguamiento aerodinámico

**Este es el resultado central del módulo.** Cuando el cohete gira, cada aleta
se mueve con él y ve un ángulo de ataque distinto del comandado. La velocidad
del punto de aplicación es `v_i = omega × r_i`; del par horizontal cuenta la
componente `z` y del vertical la componente `y`:

```
  (omega × r)_z = p·r_y − q·r_x           (omega × r)_y = r·r_x − p·r_z
```

Si la aleta se mueve hacia `+z`, el aire la ataca desde `-z` y su incidencia
efectiva **baja**. Sustituyendo las cuatro posiciones:

```
  d1:  delta1 − (q·l_t + p·d)/V           d3:  delta3 + (r·l_t − p·d)/V
  d2:  delta2 − (q·l_t − p·d)/V           d4:  delta4 + (r·l_t + p·d)/V
```

Linealizando alrededor de `omega = 0`, las derivadas de estabilidad son:

```
  L_p = −4 k d²   / V          M_q = −2 k l_t² / V          N_r = M_q
```

y **no hay ningún término cruzado**. La misma simetría V4 que diagonaliza el
mando diagonaliza la respuesta a la rotación:

- una `p` pura desplaza `d1` y `d3` hacia abajo y `d2` y `d4` hacia arriba —
  el modo antisimétrico, que solo produce `L`;
- una `q` pura desplaza `d1` y `d2` por igual — el modo simétrico horizontal,
  que solo produce `M` (sus contribuciones a `L` entran con signo opuesto y se
  cancelan);
- una `r` pura hace lo propio con `d3` y `d4`, y solo produce `N`.

El caso 8 de §7 verifica las tres cosas numéricamente. Con los valores por
defecto:

```
amortig.    L_p     -0.5166   M_q     -9.7393   N_r     -9.7393  N*m/(rad/s)
```

### Consecuencia: el régimen permanente

Sin amortiguamiento, una deflexión constante da una velocidad angular que
**crece linealmente para siempre**. Con él, `omega` converge, y el equilibrio
tiene una interpretación limpia: es el punto donde la incidencia efectiva se
anula, es decir, **la aleta acaba alineada con el flujo local**.

```
  p_ss = +V · u_roll  / d          q_ss = +V · u_pitch / l_t          r_ss = −V · u_yaw / l_t
```

Con los valores por defecto: `p = 105.26 · u_roll`, `q = 17.14 · u_pitch`,
`r = −17.14 · u_yaw`. El alabeo converge seis veces más alto que el cabeceo,
otra vez por el brazo corto.

Las constantes de tiempo son largas — `Iyy/|M_q| ≈ 19 s` en cabeceo,
`Ixx/|L_p| ≈ 31 s` en alabeo — porque las aletas son la *única* superficie
aerodinámica del modelo. Un cohete real añade el amortiguamiento del fuselaje.

Compáralo tú mismo con la tecla `D` en el modo gráfico, o con
`scripts/amortiguamiento.txt` en headless.

---

## §5 — Aeroelasticidad: la aleta deja de ser rígida

Hasta aquí la aleta era una placa infinitamente rígida que iba exactamente
donde el servo la mandaba. No lo es. Cada aleta es un **sistema de dos grados
de libertad excitado por su propia fuerza aerodinámica**:

```
  [ m_bend   S_theta ] [ w'' ]   [ f - c_w  w'  - K_bend  w  ]
  [ S_theta  I_theta ] [ th''] = [ H - c_th th' - K_theta th ]

  con  H    = -e_ea · f                        (momento de la carga sobre el pivote)
       c_w  = 2 zeta_bend  sqrt(K_bend  m_bend)
       c_th = 2 zeta_theta sqrt(K_theta I_theta)
```

- **`w`** es la flecha de la punta. La flexión **no cambia la incidencia** —
  aleta recta, sin flecha en planta — así que es el modo que *se ve*, no el que
  manda.
- **`th`** es la torsión sobre el pivote. `K_theta` es rigidez concentrada (eje
  más rigidez del servo), así que la aleta se retuerce **en bloque**: la
  torsión es uniforme a lo largo de la envergadura. Y ésta **sí** entra en el
  ángulo de ataque.

`e_ea` es la distancia del **pivote** al **centro aerodinámico**, positiva
hacia atrás. Es el parámetro que decide todo.

### El lazo aeroelástico

La torsión cambia la carga, y la carga cambia la torsión. En equilibrio:

```
  f = k (delta + th)          K_theta · th = -e_ea · f

  =>  delta_eff = delta / (1 + lambda)       lambda = qbar · S · CL_delta · e_ea / K_theta
```

De donde la **eficiencia aeroelástica** y la presión dinámica característica:

```
  eta = 1 / (1 + qbar/qbar_car)        qbar_car = K_theta / (S · CL_delta · |e_ea|)
```

Un solo número, `eta`, dice qué fracción del mando llega de verdad al aire. Y
el **signo de `e_ea`** parte el problema en dos regímenes opuestos:

| `e_ea` | Diseño | Qué hace la carga | Consecuencia |
|---|---|---|---|
| **> 0** | normal: c.a. por detrás del pivote | **destuerce** la aleta | `eta < 1`, cae como `1/qbar`. Siempre estable: se pierde eficacia de mando al ir rápido. |
| **< 0** | sobrebalanceada: pivote por detrás del c.a. | **aumenta** la incidencia | `eta > 1`, amplifica. Hay una `qbar` de **divergencia** donde `eta → ∞` y la aleta se rompe. |

Con los valores por defecto (`e_ea = 2 cm`, `K_theta = 250 N·m/rad`):

```
flex        eta 0.996198   qbar_car 144448.8 Pa (V = 485.6 m/s)
modos       flexion 230.9 rad/s (36.8 Hz)   torsion 408.2 rad/s (65.0 Hz)
```

**La aeroelasticidad es cosa de ir rápido.** A 30 m/s se pierde un 0.4% del
mando: existe, es real y no se nota. A 200 m/s ya se pierde el 14%, y a 486 m/s
la mitad. Ésa es la lección, y por eso los valores por defecto **no** exageran
el efecto: exagerarlo sería mentir sobre a qué régimen pertenece.

Para verlo sin ir a Mach 1.5, ablanda la aleta — que es lo que pasa de verdad
en un cohete pequeño, donde la rigidez la pone el servo y no la placa:

```
set K_theta 5     # eta = 0.8398
pitch 10          # el servo manda 10 deg...
step 6
state             # ...pero "eff" dice 8.39763, que es exactamente eta * 10
```

### Los modos estructurales fijan el paso

Los dos modos van a decenas de Hz; el lazo de actitud, a ~1 Hz. Son ellos, y no
la dinámica del cohete, quienes deciden el `dt`:

```
  CFL = max(w_bend, w_tors) · dt          RK4 es inestable por encima de 2.83
```

Con `dt = 2 ms` el CFL es 0.82 y todo va bien. Con `dt = 10 ms` sube a 4.08 y
la integración **diverge a NaN**. Por eso los casos 9, 10 y 13 de §7, que
integran cientos de segundos con `dt = 10 ms`, ponen `flex_on = false`
explícitamente: no es una trampa, es que a ese paso la aleta flexible no se
puede simular. El código avisa antes de reventar — en el HUD, al arrancar, y
después de cualquier `set` que empeore el CFL.

### Lo que la simetría no rompe

La respuesta elástica es ella misma simétrica: un mando de cabeceo carga `d1` y
`d2` por igual, así que ambas se retuercen y flexan lo mismo, y el momento de
alabeo sigue siendo **cero exacto**. **El desacoplo de §2 sobrevive a la
flexibilidad** — el caso 15 de §7 lo comprueba. Lo que sí cambia es la
ganancia: el autopiloto se sintoniza sobre `eta · M_du`, y por eso sus
ganancias suben al bajar `eta`.

### Flexión y torsión acopladas: flameo

`S_theta` (desequilibrio estático entre los dos modos) es el **único** término
que los acopla, y por defecto vale 0. Ponlo distinto de cero y aparece el
modelo binario clásico de **flutter**: por encima de cierta velocidad los dos
modos se acoplan y el conjunto se vuelve inestable. Con aerodinámica
cuasi-estacionaria como la de aquí el fenómeno sale cualitativamente bien pero
no cuantitativamente — para una velocidad de flameo fiable hace falta
aerodinámica no estacionaria (Theodorsen), que está fuera del alcance de este
módulo.

### Cómo se dibuja

La flecha real de la punta a 30 m/s es de **décimas de milímetro**: físicamente
correcta e invisible. El render la exagera, como cualquier post de elementos
finitos, y **anuncia el factor en el HUD** para que nadie lo confunda con la
deformación real. La tecla `F` lo cicla entre ×1, ×20, ×60 y ×150; la tecla `f`
devuelve la aleta a rígida.

La placa se trocea en tiras a lo largo de la envergadura y cada estación se
desplaza según la deformada estática exacta de un voladizo con carga en la
punta, `phi(xi) = (3xi² − xi³)/2`, normalizada a 1 en la punta. Las normales se
inclinan con la pendiente de esa curva; sin eso la iluminación sale plana y la
curvatura no se ve. La torsión, al ser uniforme, va en el ángulo de rotación de
la aleta, no en la forma.

---

## §6 — Actuador y autopiloto

### Actuador

`delta_cmd` (lo que pide el mixing) y `delta` (dónde está de verdad la aleta)
son estados distintos. `servo_step()` lleva el segundo hacia el primero con:

- un **retardo de primer orden** de constante `tau_servo` (0.05 s por defecto),
  integrado en forma exacta para no depender de que `dt << tau`;
- un **límite de velocidad** `rate_limit` (240 °/s por defecto).

Con `tau_servo = rate_limit = 0` el actuador es ideal y `delta = delta_cmd`.
Tecla `S` en el modo gráfico, `servo on|off` en headless.

### Autopiloto

Tres PID independientes, uno por irrep. **Que esto funcione es precisamente lo
que compran §2 y §4**: si el mando o el amortiguamiento acoplaran los ejes,
haría falta un controlador MIMO.

Cada canal se sintoniza por asignación de polos sobre el modelo linealizado.
Para cabeceo (los otros dos son idénticos cambiando la inercia y las
derivadas):

```
  Iyy · theta̎ = M_du · u + M_q · q          con  u = kp·err − kd·q

  =>   kp = wn² · Iyy / M_du
       kd = (2·zeta·wn·Iyy + M_q) / M_du
       ki = kp · wn / 5
```

`M_q` **ya es negativo**, de ahí el `+` en `kd`: el controlador solo aporta el
amortiguamiento que la aerodinámica no da. La integral es lenta a propósito —
sirve de trim, no domina el lazo — y lleva anti-windup contra la saturación de
salida.

Por defecto `wn = 3.0 / 1.5 / 1.5 rad/s` y `zeta = 0.8`. Las ganancias
resultantes:

```
PID roll    kp   +2.6584  ki   +1.5950  kd   +1.4083  (wn 3.00)
PID pitch   kp   +2.5292  ki   +0.7587  kd   +2.6394  (wn 1.50)
PID yaw     kp   -2.7222  ki   -0.8167  kd   -2.8453  (wn 1.50)
```

> **Las ganancias de guiñada salen negativas.** No es un error de signo: es
> `N_du < 0` de §2 propagándose hasta el controlador. Merece la pena dejarlas
> negativas en vez de esconder el signo en el mixing, porque así la convención
> de Blakelock es visible de punta a punta.

La derivada se toma de la **velocidad angular medida**, no de `d(err)/dt`, para
que un salto en la referencia no dé patada derivativa. Cada canal satura en
`0.7 · delta_stall`; como el mixing suma cabeceo y alabeo sobre la misma aleta,
un escalón agresivo en dos ejes a la vez sí puede pedir más de `delta_stall`
en una aleta concreta — el clamp de §3 se encarga, y el lazo converge igual
(caso 12 de §7).

Teclas `A` (activar), `C` (mantener la actitud actual). Con el autopiloto
activo, las flechas y `Q`/`E` mueven **referencias de actitud** en vez de
mando directo: la misma palanca, una capa más arriba del lazo.

### El tensor de inercia

`rotational_dynamics()` usa el tensor con simetría XZ completo:

```
  I = [ Ixx   0   -Ixz ]                     Gamma = Ixx·Izz − Ixz²
      [  0   Iyy    0  ]        I⁻¹|xz = (1/Gamma) [ Izz  Ixz ]
      [-Ixz   0    Izz ]                            [ Ixz  Ixx ]
```

Por defecto `Ixz = 0`. Ponlo a algo distinto (`set Ixz 2.0`) y aparece el
acoplo alabeo–guiñada clásico: un mando de alabeo puro genera `r`, **aunque el
momento aerodinámico `N` de mando siga siendo exactamente cero**. El acoplo es
inercial, del término `omega × I·omega`, no aerodinámico. El caso 13 de §7
separa las dos cosas.

---

## §7 — Casos de validación

`make test` (C++) y `make python-test` (Python) corren los mismos 18 casos, 79
comprobaciones:

| # | Qué comprueba |
|---|---|
| 1–3 | Cabeceo, guiñada y alabeo puros: un solo momento, ceros exactos en los otros dos ejes, signo correcto. |
| 4 | `demix(mix(u)) == u` en cuatro casos (error < 1e-16). |
| 5 | Saturación por pérdida: simétrica, plana por encima, lineal por debajo. |
| 6 | Norma del cuaternión tras 1e4 pasos con los tres canales excitados. |
| 7 | Comando combinado: los momentos coinciden con las fórmulas cerradas de §2. |
| 8 | **Amortiguamiento diagonal**: `p` → solo `L`, `q` → solo `M`, `r` → solo `N`; los tres se oponen al giro; coinciden con `L_p`, `M_q`, `N_r`. |
| 9 | **Régimen permanente** `omega_ss = V·u/brazo` en los tres ejes, y `delta_eff → 0` en equilibrio. |
| 10 | Contraste con `damping_on = false`: rampa lineal exacta, sin equilibrio. |
| 11 | Actuador: `rate_limit·dt` en un paso; `1 − 1/e` en `t = tau`; salto instantáneo con ambos a cero. |
| 12 | Autopiloto: converge a (10°, 5°, −8°) con error < 0.2° y velocidades residuales nulas; `kp` de guiñada negativa. |
| 13 | `Ixz`: aparece `r` con mando de alabeo puro, pero el `N` **de mando** sigue siendo cero. |
| 14 | `params_update_aero()`: cambiar `S_fin` recalcula `AR` y `CL_delta`. |
| 15 | **Aeroelasticidad estática**: `delta_eff/delta == eta`; la aleta se destuerce; la flecha vale `f/K_bend`; `theta1 == theta2`, así que el desacoplo de §2 aguanta. |
| 16 | `eta` cae monótona con `V`, vale exactamente 1/2 en `qbar_car`, y es 1 **exacto** con la aleta rígida. |
| 17 | **Divergencia** con `e_ea < 0`: `eta > 1`, vale 10 al 90% de `qbar_div`, y la torsión cambia de signo. |
| 18 | Modos estructurales y CFL: `dt = 2 ms` los resuelve, `dt = 10 ms` no (y avisa antes de dar NaN). |

**`make paridad`** es la prueba más fuerte del módulo: comprueba que la
implementación C++ y la de Python producen la misma traza **byte a byte**, en
los 79 casos y en los seis scripts. Son dos implementaciones independientes
que no pueden copiarse un error numérico la una a la otra.

Un detalle que cuesta encontrar la primera vez: las combinaciones
antisimétricas (`u_roll`, `L`) son la cancelación total de cuatro números casi
iguales, así que al converger el lazo quedan en ~1e-31 y su **signo** es ruido
de redondeo. Por eso la impresión de estado aplasta a cero todo lo que baja de
1e-9 (`z()` en `main.cpp`, `_z()` en el Python): sin eso, las dos trazas
difieren en un `-0.00000` que no significa nada. Por lo mismo `qbar` se imprime
con cuatro decimales y no tres: `rho·V²/2` cae en empates exactos (`x.xxx5`) y
C++ y Python los rompen en direcciones opuestas.

---

## §8 — Prototipo de referencia en Python

`alerones_py/main.py` implementa el **mismo modelo** de forma independiente
(NumPy + matplotlib) y ofrece tres modos:

```bash
python3 alerones_py/main.py            # animación 3D interactiva
python3 alerones_py/main.py --test     # los 55 casos de §7
python3 alerones_py/main.py --repl     # REPL, equivalente a --headless
```

No es un juguete: es la red de seguridad del módulo. Cualquier cambio en la
física tiene que hacerse en los dos sitios, y `make paridad` no pasa hasta que
coinciden dígito a dígito.

Donde el Python se aparta del C++ es solo en el orden de las operaciones
cuando eso cambiaría el resultado: `mix_commands` y `demix` están escritos en
forma desarrollada, no como el producto matricial `MIX @ u`, porque acumular
los sumandos en otro orden produce ceros con signo distinto. Las matrices
`MIX` y `DEMIX` siguen en el fichero como documentación de §2.

---

## §9 — Uso

### Compilar y ejecutar

```bash
make            # compila ./alerones
make run        # compila y abre la ventana 3D
make test       # los 55 casos, sin gráfica
make paridad    # C++ vs Python, byte a byte
make figuras    # genera figuras/*.csv para el libro
make help       # todos los targets
```

Requiere `clang++` con C++17 y OpenGL/GLUT (en macOS vienen con el SDK; en
Linux hace falta `freeglut3-dev`). El Python necesita `numpy` y `matplotlib`.

### Teclado (modo gráfico)

| Tecla | Acción |
|---|---|
| ↑ / ↓ | `u_pitch` −/+ · con el AP activo, `theta_ref` |
| ← / → | `u_yaw` −/+ · con el AP activo, `psi_ref` |
| `Q` / `E` | `u_roll` −/+ · con el AP activo, `phi_ref` |
| `1`…`4` | selecciona aleta · `+` / `-` la deflecta directamente |
| `A` | autopiloto on/off (engancha sin salto: la referencia arranca en la actitud actual) |
| `C` | *hold*: fija la referencia en la actitud de este instante |
| `D` | amortiguamiento aerodinámico on/off (§4) |
| `f` | aleta flexible / rígida (§5) |
| `F` | exageración del dibujo de la deformación: ×1, ×20, ×60, ×150 |
| `S` | dinámica de actuador on/off (§6) |
| `0` | centra aletas y referencias · `R` reset · `SPACE` pausa |
| `,` / `.` | velocidad de vuelo −/+ (resintoniza el AP: la autoridad va con `qbar`) |
| `T` | vista de cola · `X` ejes · `W` wireframe · `ESC` salir |
| ratón | arrastrar para orbitar, rueda para zoom |

### Modo headless

REPL por `stdin` que ignora comentarios con `#`, así que se le puede dar de
comer cualquier fichero:

```bash
./alerones --headless < scripts/desacoplo.txt
./alerones --headless < scripts/amortiguamiento.txt
./alerones --headless < scripts/aeroelasticidad.txt
./alerones --headless < scripts/actuador.txt
./alerones --headless < scripts/autopiloto.txt
./alerones --headless < scripts/regresion.txt
```

Cada script está comentado y corresponde a una sección de este documento.
`help` dentro del REPL lista los comandos: `pitch/yaw/roll`, `fin`, `ap`,
`ref`, `hold`, `tune`, `gains`, `damp`, `flex`, `servo`, `set`, `step`,
`state`, `csv`, `reset`.

### Figuras para el libro

`csv <seg> [cada] [archivo]` vuelca la trayectoria con columnas
`t, phi, theta, psi, p, q, r, d1..d4, dc1..dc4, L, M, N` (ángulos en grados).
`make figuras` genera los cuatro casos de referencia:

| Archivo | Qué muestra |
|---|---|
| `figuras/sin_amortiguar.csv` | La rampa de §4 sin amortiguamiento. |
| `figuras/amortiguado.csv` | La misma entrada convergiendo a `V·u/l_t`. |
| `figuras/actuador.csv` | `dc` contra `d`: retardo y límite de velocidad. |
| `figuras/autopiloto.csv` | Escalón de actitud a (10°, 5°, −8°). |
| `figuras/aeroelastico.csv` | Aleta blanda: `d` contra la incidencia efectiva mientras el modo de torsión se asienta. |

---

## Qué falta a propósito

Para que el alcance quede escrito y no haya que deducirlo del código:

- **Traslación.** No hay posición, velocidad ni gravedad; `V` es parámetro.
  Está en [`unmanned/`](../../unmanned/README.md).
- **Resistencia de las aletas y fuerza normal sobre el fuselaje.** Solo se
  modelan momentos.
- **Amortiguamiento del fuselaje.** Las aletas son la única superficie
  aerodinámica, y por eso las constantes de tiempo salen largas (§4).
- **Interferencia entre aletas y efectos de compresibilidad.** `CL_delta` es
  constante con el Mach.
- **Histéresis de pérdida.** El modelo post-pérdida mantiene la fuerza de
  saturación (§3).
- **Aerodinámica no estacionaria.** La aeroelasticidad de §5 usa aerodinámica
  cuasi-estacionaria: basta para la eficiencia y la divergencia, que son
  estáticas, pero no da una velocidad de flameo fiable. Para eso hace falta
  Theodorsen.
- **Flexión del fuselaje.** Solo se deforman las aletas; el cuerpo es rígido.

## Referencias

- J. H. Blakelock, *Automatic Control of Aircraft and Missiles*, 2ª ed. —
  convención de ejes y numeración de aletas (Fig. 8-2), control de misiles
  cruciformes.
- R. W. Beard y T. W. McLain, *Small Unmanned Aircraft* — cinemática del
  cuaternión y dinámica de cuerpo rígido con `Ixz` (usadas en `unmanned/`).
- B. Etkin y L. D. Reid, *Dynamics of Flight* — derivadas de estabilidad
  `L_p`, `M_q`, `N_r`.
- H. B. Helmbold (1942) — pendiente de sustentación a alargamiento bajo.
- R. L. Bisplinghoff, H. Ashley y R. L. Halfman, *Aeroelasticity* — eficiencia
  de mando, divergencia y el modelo binario flexión–torsión de §5.
