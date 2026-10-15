# unmanned — Motor de física UAV multi-vehículo (6DOF)

Reporte técnico del motor de simulación de vuelo contenido en esta carpeta:
qué se construyó, por qué cada pieza existe, qué ecuaciones y métodos
numéricos la implementan, y qué escenario pone a prueba cada programa.

El motor simula la dinámica de cuerpo rígido de **tres vehículos** —un UAV
ala delta, un quadcopter en configuración X y un convertiplano de 4 rotores
basculantes (tiltrotor)— sobre la base de las ecuaciones de movimiento de
Beard & McLain, *Small Unmanned Aircraft* (Cap. 3 y 4). El modelo físico es
una única fuente de verdad (`physics_UAV.h` + `simulate_step.h`) que
consumen tanto la demo de consola (`physics_UAV.cpp`) como la visualización
3D interactiva (`drone_model.cpp`), para que ambas simulen exactamente lo
mismo. Un cuarto módulo, aparte (`launch_rockets/`), simula el lanzamiento
de un cohete con mecánica Lagrangiana.

**Por qué importa cada escenario:** el ala delta y el quad son los dos casos
de control "de libro" (empuje longitudinal con sustentación de ala vs.
empuje colectivo vectorizado por inclinación) — sirven para validar que las
ecuaciones de Beard & McLain están bien implementadas contra dos regímenes
de vuelo opuestos. El convertiplano es el caso interesante: no es un tercer
vehículo más, es una *transición* — su geometría de masa, su mezcla de
sustentación/empuje y su reparto de autoridad de control cambian
continuamente entre los dos regímenes anteriores, y esa transición es
exactamente donde un tiltrotor real puede fallar (pérdida de autoridad de
control, caída de empuje de la hélice con la velocidad, pérdida de
sustentación si la transición es muy rápida). El campo de 20 obstáculos y la
colisión sólida existen para forzar al piloto (humano) a tomar decisiones
de trayectoria en 3D, no solo evitar el suelo.

![UAV ala delta en vuelo, con la ventana de telemetría en vivo](ala_delta.png)
*Vista real de `drone_model` corriendo el UAV ala delta: ventana principal
(HUD + obstáculos) a la izquierda, osciloscopio de telemetría en vivo a la
derecha.*

## 1. Estructura del proyecto

| Archivo | Rol |
|---|---|
| `physics_UAV.h` | Motor de física (header-only): geometría de masa/inercia de los 3 vehículos, cinemática/dinámica traslacional y rotacional, aerodinámica de ala fija, modelo de hélice, control de actitud del convertiplano, integradores Euler y RK4, conversión de actitud, colisión con el piso y con obstáculos. |
| `simulate_step.h` | `simulateStep()` — variante de RK4 que usa el **tensor de inercia completo** (con productos de inercia Ixy, Ixz, Iyz), en vez de la forma reducida que asume simetría XZ. |
| `physics_UAV.cpp` | Demo de consola: imprime masa/inercia de 2 vehículos, un reporte de transición del convertiplano, una tabla de aerodinámica del ala, una demo de conversión de actitud, y corre 5 s de vuelo libre con `simulateStep()`. |
| `drone_model.cpp` | Visualización 3D interactiva (OpenGL/GLUT): vuelo controlable por teclado de los 3 vehículos + ventana de telemetría en vivo + campo de 20 obstáculos sólidos. |
| `Makefile` | Targets de compilación y ejecución. |
| `launch_rockets/` | Proyecto aparte: simulación de lanzamiento de un cohete (mecánica Lagrangiana, RK4), con vista en primera persona. Ver [sección 13](#13-launch_rockets--mecánica-lagrangiana-de-un-cohete). |
| `ala_delta.png` | Captura real de `drone_model` (UAV ala delta) en vuelo, usada arriba en este README. |
| `euler_to_quaternions.jpeg`*, `quaternion_to_rotationMatrix.jpeg`* | Referencia de las fórmulas de conversión de actitud (no incluidas en el repo; ver fórmulas transcritas en la [sección 9](#9-conversión-de-actitud-euler--cuaternión--matriz-de-rotación)). |

> Nota: el `Makefile` también define un target `rot_trans_kinematics`, pero
> `rot_trans_kinematics.cpp` no está presente en la carpeta actualmente — ese
> target no compila.

## 2. Estado del vehículo y convención de ejes

El estado completo de cualquiera de los 3 vehículos (`UAVState`) tiene
**12 componentes**:

- **Posición inercial NED** (`pn, pe, pd`) — marco North-East-Down; `pd`
  positivo hacia abajo (así que "subir" es `pd` cada vez más negativo).
- **Velocidad en marco cuerpo** (`u, v, w`).
- **Actitud en ángulos de Euler** (`phi` roll, `theta` pitch, `psi` yaw).
- **Tasas angulares en marco cuerpo** (`p, q, r`).

Ejes cuerpo: **X** hacia la nariz, **Y** ala derecha, **Z** hacia abajo.

Las ecuaciones que gobiernan la evolución de estos 12 componentes se
organizan en 5 bloques, siguiendo la estructura de Beard & McLain Cap. 3:

```
Bloque 0  Geometria de masa/inercia      forma      -> masa, CG, tensor I
Bloque 1  Cinematica traslacional        u,v,w      -> pn_dot,pe_dot,pd_dot
Bloque 2  Dinamica traslacional          fx,fy,fz   -> u_dot,v_dot,w_dot
Bloque 3  Cinematica rotacional          p,q,r      -> phi_dot,theta_dot,psi_dot
Bloque 4  Dinamica rotacional            l,m,n      -> p_dot,q_dot,r_dot
```

## 3. Bloque 0 — Geometría de masa e inercia

**Por qué existe este bloque:** en vez de inventar `Ixx, Iyy, Izz`, el
simulador los *deriva* de la geometría real de cada vehículo (fuselajes,
alas, brazos, góndolas...), aplicando el teorema de ejes paralelos a cada
pieza y sumando. Esto hace que la dinámica rotacional (Bloque 4) sea
consecuencia de una forma física, no un número de diseño arbitrario — y
permite que el convertiplano, cuya geometría *cambia en vuelo*, recalcule su
inercia en cada paso en vez de usar una tabla precalculada.

### 3.1 Inercia de las piezas primitivas

Cada pieza se modela como un sólido elemental con inercia conocida respecto
a **su propio CG**:

| Pieza | Función | Fórmula (sobre su propio CG) |
|---|---|---|
| Cilindro sólido (fuselaje, brazos, góndolas), eje X | `inerciaCilindro` | `Ixx = ½ m r²`,  `Iyy = Izz = (1/12) m (3r² + L²)` |
| Placa triangular delta en plano XY (ala delta) | `inerciaAlaDelta` | `Ixx = m b²/24`,  `Iyy = m c²/18`,  `Izz = Ixx+Iyy` |
| Caja rectangular (compartimentos, panel de ala, bahía, tren, estabilizador) | `inerciaCaja` | `Ixx = (1/12) m (ly²+lz²)`, análogo para `Iyy, Izz` |
| Masa puntual (motor) | `inerciaPuntual` | `I = 0` (toda su inercia es por traslación) |
| Disco delgado en plano XY, eje Z (rotor) | `inerciaDiscoZ` | `Izz = ½ m r²` (eje de giro),  `Ixx=Iyy = ¼ m r²` (diametral) |

### 3.2 Teorema de ejes paralelos

Cada pieza vive en un CG propio distinto del CG conjunto del vehículo. Para
sumarlas hace falta trasladar cada tensor local al CG común
(`trasladar()`), vector de desplazamiento `r = cg_pieza − cg_conjunto`:

```
Ixx' = Ixx + m(ry² + rz²)      Ixy' = Ixy − m·rx·ry
Iyy' = Iyy + m(rx² + rz²)      Ixz' = Ixz − m·rx·rz
Izz' = Izz + m(rx² + ry²)      Iyz' = Iyz − m·ry·rz
```

`combinarComponentes()` hace esto para una lista arbitraria de piezas:
calcula masa total y CG conjunto por promedio pesado, traslada cada tensor
local a ese CG y los suma. Es la única rutina de ensamblado y la comparten
los 3 vehículos — cada uno solo describe su propia lista de piezas.

Las ecuaciones de movimiento del Bloque 4 (forma reducida) asumen simetría
de masa respecto al plano **XZ** (`Ixy = Iyz = 0`); los 3 vehículos la
cumplen por construcción (el ala delta vive en `y=0`, el quad y el
convertiplano tienen sus piezas en pares `±y`). Si alguna vez dejara de
cumplirse, `combinarComponentes()` avisa por `stderr`.

### 3.3 Rotación de un tensor de inercia (brazos y góndolas inclinadas)

Un brazo de quad o una góndola de convertiplano tiene su inercia fácil de
calcular **sobre su propio eje** (un cilindro a lo largo de X), pero en el
cuerpo del vehículo está rotado. Para llevar el tensor a los ejes del
cuerpo se usa la transformación de similitud `I' = R · I · R^T`:

- `rotarZ(I, ángulo)` — rota sobre el eje Z; usada para los 4 brazos del
  quadcopter (a 45°, 135°, 225°, 315° del eje X, configuración "X").
- `rotarY(I, ángulo)` — rota sobre el eje Y; usada para las góndolas y
  rotores del convertiplano, que bascular literalmente **inclina** sobre
  ese eje al pasar de vuelo estacionario a crucero.
- `rotarYVec(v, ángulo)` — mismo giro pero sobre un vector de posición: al
  bascular, el CG de cada góndola se mueve de verdad en el espacio, y con
  él cambian tanto el CG del vehículo entero como su tensor de inercia.

### 3.4 Los tres vehículos (Bloque 0 instanciado)

| | `buildDeltaWingDrone()` | `buildQuadcopter()` | `buildTiltrotor(sigma)` |
|---|---|---|---|
| Piezas | fuselaje (cilindro) + ala delta (placa) + compartimento (caja) + motor (puntual) | cuerpo (caja) + tren + 4×(brazo + motor + rotor) | fuselaje + bahía de carga + 2 paneles de ala + deriva + estabilizador + tren + 4×(góndola + rotor) |
| Masa total | **200.0 kg** | **142.0 kg** | **154.9 kg** (varía con `sigma` solo en CG/inercia, no en masa) |
| `Ixx, Iyy, Izz` [kg·m²] | 16.1 / 187.7 / 202.0 | — (config X, simétrico) | 207.0 / 174.6 / 361.4 @ `sigma=0` (verificado corriendo `physics_UAV`) |
| Particularidad | geometría fija | geometría fija, simetría de 4 ramas | **geometría variable**: `sigma` ∈ [0, π/2] bascula 4 góndolas+rotores alrededor del eje Y, moviendo CG e inercia en cada paso |

El convertiplano es, por diseño, el único cuyo Bloque 0 se **reevalúa en
cada paso de integración** (`buildTiltrotor(g_tilt)` dentro de
`stepTiltrotor()`), porque su forma física cambia en vuelo.

## 4. Bloque 1 — Cinemática traslacional

Convierte la velocidad en marco cuerpo a tasa de posición inercial, vía la
matriz de rotación cuerpo→inercial `R_b^v` (producto de 3 rotaciones de
Euler):

```
[pn_dot]   [cθcψ   sφsθcψ-cφsψ   cφsθcψ+sφsψ] [u]
[pe_dot] = [cθsψ   sφsθsψ+cφcψ   cφsθsψ-sφcψ] [v]
[pd_dot]   [-sθ    sφcθ           cφcθ        ] [w]
```

(`c`=cos, `s`=sin; subíndices `φ,θ,ψ` = roll, pitch, yaw). Es puramente
geométrica — no depende de masa ni de fuerzas.

## 5. Bloque 2 — Dinámica traslacional

Segunda ley de Newton expresada **en el marco cuerpo, que rota** con el
vehículo — de ahí los términos de Coriolis/centrífugos (`rv−qw`, etc.):

```
u_dot = (r·v − q·w) + fx/m
v_dot = (p·w − r·u) + fy/m
w_dot = (q·u − p·v) + fz/m
```

`fx, fy, fz` son fuerzas de entrada (empuje + gravedad proyectada al cuerpo
+ aerodinámica) que arma cada escenario — Beard & McLain las dejan como
entrada externa a propósito.

### 5.1 Arrastre cuadrático simple (`aeroDrag`) — UAV ala delta y quad

Para el ala delta y el quad (que en `drone_model.cpp` no usan el modelo
aerodinámico completo), el arrastre es `F = -k·v·|v|` por eje del cuerpo,
con `k` calibrado por vehículo. Sin este término, un empuje constante
acelera sin límite (1ª ley de Newton) y soltar el acelerador no frena nada;
con él, cada posición de palanca tiene un punto de equilibrio real
(`empuje = drag`) a una velocidad de crucero fija.

### 5.2 Aerodinámica de ala fija completa (Bloque 2b) — convertiplano

El convertiplano sí necesita sustentación real, dependiente de la velocidad
y del ángulo de ataque — es lo que decide si una transición se completa o
el vehículo cae. Implementa Beard & McLain Cap. 4:

```
Datos de aire:  Va = |v_body|,  alpha = atan2(w,u),  beta = asin(v/Va)
                qbar = ½ · rho · Va²                    (presion dinamica)
```

**Coeficiente de sustentación** — mezcla con una sigmoide el modelo
*lineal* (válido antes de pérdida) con el de *placa plana* (válido después,
donde `alpha` puede pasar rutinariamente por ±90° en vuelo estacionario):

```
sigma_blend(alpha) = (1 + e1 + e2) / ((1+e1)(1+e2))
    e1 = exp(-M(alpha - alpha0)),  e2 = exp(M(alpha + alpha0))

CL(alpha) = (1-sigma_blend)·(CL0 + CLalpha·alpha)
          +    sigma_blend ·(2·sign(alpha)·sin²(alpha)·cos(alpha))
```

**Coeficiente de arrastre** (parásito + inducido, eq. 4.11):

```
CD(alpha) = CDp + CL_lineal² / (pi · e · AR)         AR = b²/S
```

**Fuerzas al marco cuerpo** (eq. 4.6, rotando (L,D) del marco viento al
cuerpo por `alpha`):

```
fx = qbar·S·(-CD·cos(alpha) + CL·sin(alpha))
fy = qbar·S·( CYbeta·beta )
fz = qbar·S·(-CD·sin(alpha) - CL·cos(alpha))
```

**Momentos propios de la célula** (eq. 4.7/4.19) — estabilidad estática
(`Cl_beta`, `Cm_alpha`, `Cn_beta`) + amortiguamiento por velocidad angular
(`Clp`, `Cmq`, `Cnr`), adimensionalizados por envergadura/cuerda media:

```
l = qbar·S·b·(Cl0 + Clbeta·beta + Clp·(b/2Va)·p + Clr·(b/2Va)·r)
m = qbar·S·c·(Cm0 + Cmalpha·alpha + Cmq·(c/2Va)·q)
n = qbar·S·b·(Cn0 + Cnbeta·beta + Cnp·(b/2Va)·p + Cnr·(b/2Va)·r)
```

Son los momentos que el aire pone *solo*, sin mando del piloto: el ala se
opone a alabear, la cola se opone a cabecear, la deriva alinea la nariz con
el viento relativo. En vuelo estacionario `qbar≈0` y desaparecen — ahí el
único que estabiliza es el control de rotores (Bloque 4b).

Coeficientes por defecto: los del **Aerosonde** (Beard & McLain, Apéndice
E), un UAV de clase de tamaño similar a los de este simulador. `S`, `b`,
`c` los pone cada vehículo (no son constantes sueltas), para que el ala que
sustenta, la que pesa en el Bloque 0 y la que se dibuja sean literalmente
la misma ala.

### 5.3 Caída de empuje de la hélice con la velocidad

Un rotor da menos empuje avanzando que parado: a más velocidad de aire
entrando por el disco, menos puede acelerarlo. Teoría de cantidad de
movimiento (Beard & McLain eq. 4.18):

```
T(delta, Va_ax) = delta · T_estatico − kLapse · Va_ax²
    kLapse = ½ · rho · S_prop · C_prop     (S_prop = 4 discos del convertiplano)
    Va_ax  = u·sin(sigma) − w·cos(sigma)   (velocidad proyectada sobre el eje de la gondola)
```

Solo cuenta el flujo que entra por delante del disco (`Va_ax > 0`). Para el
convertiplano esto no es un detalle estético: su empuje está dimensionado
para sostener el peso en estacionario (T/W=2), muchísimo más de lo que hace
falta para vencer el arrastre en crucero — sin la caída, en modo avión
aceleraría sin techo útil; con ella, el empuje se desploma justo cuando el
ala ya sostiene el peso, que es el compromiso real de un tiltrotor.

## 6. Bloque 3 — Cinemática rotacional

Convierte tasas angulares del cuerpo a tasas de los ángulos de Euler:

```
[phi_dot  ]   [1  sin(phi)tan(theta)  cos(phi)tan(theta)] [p]
[theta_dot] = [0  cos(phi)           -sin(phi)           ] [q]
[psi_dot  ]   [0  sin(phi)/cos(theta) cos(phi)/cos(theta)] [r]
```

Singularidad en `theta = ±90°` (**gimbal lock** — `sec(theta)` diverge);
`rotational_kinematics()` emite un `[WARNING]` por `stderr` si `cos(theta)`
se acerca a 0. Es una limitación conocida de representar la actitud con
ángulos de Euler (ver también la [sección 9](#9-conversión-de-actitud-euler--cuaternión--matriz-de-rotación)).

## 7. Bloque 4 — Dinámica rotacional

### 7.1 Forma reducida (`rotational_dynamics`, simetría XZ)

Válida cuando `Ixy = Iyz = 0` (los 3 vehículos la cumplen). Constantes Γ
precalculadas del tensor de inercia (Beard & McLain eq. 3.17):

```
Γ  = Jx·Jz - Jxz²
Γ1 = Jxz(Jx-Jy+Jz)/Γ    Γ2 = (Jz(Jz-Jy)+Jxz²)/Γ    Γ3 = Jz/Γ    Γ4 = Jxz/Γ
Γ5 = (Jz-Jx)/Jy         Γ6 = Jxz/Jy
Γ7 = ((Jx-Jy)Jx+Jxz²)/Γ Γ8 = Jx/Γ

p_dot = Γ1·p·q − Γ2·q·r + Γ3·l + Γ4·n
q_dot = Γ5·p·r − Γ6·(p²-r²) + m/Jy
r_dot = Γ7·p·q − Γ1·q·r + Γ4·l + Γ8·n
```

### 7.2 Forma completa (`rotational_dynamics_full`, `simulate_step.h`)

Cuando el tensor **no** es simétrico respecto a XZ, se usa la ecuación de
Euler para cuerpo rígido sin simplificar:

```
I · omega_dot = M − omega × (I·omega)
omega_dot     = I⁻¹ · (M − omega × I·omega)
```

con `I` la matriz simétrica 3×3 completa. Se invierte en cada evaluación
por la regla de Cramer / matriz adjunta (con chequeo de determinante casi
nulo). Es más cara que la forma Γ (una inversión 3×3 por llamada en vez de
8 constantes precalculadas), por eso vive en un header separado y solo se
usa cuando realmente hace falta — en este proyecto, la trae la demo de
consola para usar el tensor exacto (`drone.I_full`) del ala delta en vez de
la forma reducida.

### 7.3 Bloque 4b — Control de actitud del convertiplano

El ala delta y el quad comandan `p, q, r` **directamente** con el teclado y
se saltean el Bloque 4 por completo (ver [sección 8](#8-integración-numérica)).
El convertiplano no puede: su autoridad de control cambia por completo
durante la transición, y modelar eso es justamente su física interesante.
Aquí `p, q, r` **sí** salen de integrar el Bloque 4, alimentado por:

```
torque total = torque de mando (saturado por la autoridad disponible)
             + momentos aerodinamicos propios de la celula (seccion 5.2)
```

**De dónde sale la autoridad** (`tiltrotorAuthority`), para una
basculación `sigma`:

1. **Empuje diferencial entre rotores.** Cada rotor empuja sobre el eje de
   su góndola, `n = (sin σ, 0, −cos σ)`. Un diferencial `dT` entre rotores
   da, según su componente vertical (`−cos σ`) o longitudinal (`sin σ`):
   rotores a `±y` → **alabeo** en estacionario / **guiñada** en crucero;
   rotores a `±x` → **cabeceo** siempre.
2. **Basculación diferencial.** Inclinar unas góndolas más que otras aplica
   una fuerza sobre la perpendicular `t = (cos σ, 0, sin σ)`: en
   estacionario (`t=+x` en `±y`) → **guiñada**; en crucero (`t=+z` en
   `±y`) → **alabeo**. Es exactamente complementaria a (1): entre las dos,
   los 3 ejes quedan cubiertos en todo el rango de la transición.
3. **Superficies aerodinámicas** (alerón/elevador/timón). Escalan con
   `qbar`, así que valen 0 en estacionario y son el mando principal en
   crucero — el reparto rotores/superficies ocurre solo, sin lógica de
   modos explícita.

**Lazo de control** (`tiltrotorRateControl`) — el piloto comanda **tasas**
deseadas (`p_cmd, q_cmd, r_cmd`), no torques. Un controlador proporcional
pide el torque que llevaría la tasa actual a la comandada con una constante
de tiempo `tau`, y lo satura contra la autoridad disponible:

```
torque = sat( J·(tasa_cmd − tasa_actual) / tau ,  ±autoridad_disponible )
```

Escribir la ganancia como `J/tau` (y no como una constante suelta) hace que
la respuesta en lazo cerrado sea la misma constante de tiempo `tau` **sea
cual sea el modo de vuelo**, aunque el tensor de inercia cambie con la
basculación. Cuando satura, el vehículo responde más lento que lo pedido —
que es lo que de verdad pasa cuando se exige más mando del que el modo de
vuelo puede dar en ese instante.

## 8. Integración numérica

| Integrador | Orden | Dónde se usa | Costo |
|---|---|---|---|
| `integrate_euler()` | 1er orden (Euler explícito hacia adelante) | — (disponible en el header, no se llama desde los programas actuales) | 1 evaluación de derivada por paso |
| `integrate_rk4()` | 4to orden (Runge-Kutta clásico) | — (forma reducida Γ; disponible pero `physics_UAV.cpp` usa la variante de tensor completo) | 4 evaluaciones por paso |
| `simulateStep()` (`simulate_step.h`) | 4to orden, **tensor de inercia completo** | `physics_UAV.cpp` (demo de consola) | 4 evaluaciones por paso + 1 inversión 3×3 por evaluación |
| Euler explícito "manual" (`step()` / `stepTiltrotor()` en `drone_model.cpp`) | 1er orden | `drone_model.cpp` (visualización 3D interactiva) | 1 evaluación por frame |

**RK4 (4 etapas, pesos clásicos 1-2-2-1):** evalúa la derivada completa del
estado (`state_derivative` — Bloques 1 a 4 en un mismo instante) en 4 puntos
por paso y los combina:

```
k1 = f(s,            t)
k2 = f(s + k1·dt/2,  t+dt/2)
k3 = f(s + k2·dt/2,  t+dt/2)
k4 = f(s + k3·dt,    t+dt)

s_{n+1} = s_n + (dt/6)·(k1 + 2·k2 + 2·k3 + k4)
```

implementado componente por componente en `rk4_combine()`, compartido entre
`integrate_rk4()` y `simulateStep()`. Las fuerzas y torques se asumen
constantes durante el paso en ambos casos; solo cambia el orden de
precisión de la integración, no el modelo de fuerzas.

**Por qué la demo de consola usa RK4+tensor completo y la 3D usa Euler
simple:** `physics_UAV.cpp` corre un escenario fijo offline (5 s, `dt=0.01`,
sin input humano) donde conviene maximizar precisión numérica y verificar
el motor con el tensor de inercia exacto, sin la restricción de tiempo
real. `drone_model.cpp` integra a **~60 Hz fijo** (`DT = 0.016 s`) dentro de
un bucle de render con input de teclado en vivo: ahí el costo por frame
importa y el error de truncamiento de Euler con un paso tan chico es
aceptable para una simulación interactiva — además el piloto está
constantemente corrigiendo con el teclado, lo cual actúa como una
realimentación que acota el error acumulado. Nótese que esto significa que
el UAV ala delta y el quad **saltean el Bloque 4 por completo** en la
demo 3D (p,q,r se escriben directamente desde el teclado, sección 7.3); el
convertiplano es el único de los tres programas/vehículos que de verdad
integra el Bloque 4 completo paso a paso en tiempo real.

## 9. Conversión de actitud (Euler ↔ Cuaternión ↔ Matriz de rotación)

Implementadas en `physics_UAV.cpp` (declaradas en `physics_UAV.h`), como
utilidades de reporte/conversión separadas del lazo de integración (que
sigue trabajando en ángulos de Euler). Sirven para mostrar una
representación de actitud sin la singularidad de gimbal lock del Bloque 3.

**Euler → Cuaternión** (`e_b^i = (e0,e1,e2,e3)`, parte escalar `e0`):

```
e0 = cy2·ct2·cp2 + sy2·st2·sp2
e1 = cy2·ct2·sp2 − sy2·st2·cp2
e2 = cy2·st2·cp2 + sy2·ct2·sp2
e3 = sy2·ct2·cp2 − cy2·st2·sp2
   (c_2/s_2 = cos/sin de la mitad de cada angulo: phi/2, theta/2, psi/2)
```

**Cuaternión → Euler:**

```
phi   = atan2( 2(e0e1+e2e3), e0²+e3²-e1²-e2² )
theta = asin ( 2(e0e2-e1e3) )
psi   = atan2( 2(e0e3+e1e2), e0²+e1²-e2²-e3² )
```

**Cuaternión → Matriz de rotación** `R_b^i` (cuerpo→inercial): matriz 3×3
estándar en función de `(e0,e1,e2,e3)`, implementada en
`quaternionToRotationMatrix()`.

## 10. Colisiones

### 10.1 Piso (`checkGroundCollision` / `resolveGroundCollision`)

El piso es el plano horizontal `pd = pisoPD` (por defecto 0). Cruzarlo
(`pd > pisoPD`, recordar que `pd` positivo es hacia abajo) dispara una
**respuesta inelástica mínima**: clava `pd` al piso y anula `u,v,w` — frena
en seco, sin rebote ni deformación. El dron se trata como un punto (su CG).

### 10.2 Obstáculos sólidos (`checkObstacleCollision` / `resolveObstacleCollision`)

Bloques rectangulares fijos (`BoxObstacle`), mismas simplificaciones que el
piso: restricción dura, sin deformación, dron tratado como punto.

- **Test de colisión:** AABB (caja alineada a ejes) — compara `pn, pe, pd`
  contra el rango del bloque en los 3 ejes.
- **Respuesta:** empuja al dron a la cara más cercana según el eje de
  **menor penetración** (MTV — *minimum translation vector*), igual que el
  piso pero extendido a 3 ejes. Sin este reposicionamiento, frenar la
  velocidad en el frame del impacto no alcanza, porque empuje/gravedad la
  reconstruyen al paso siguiente y el dron termina atravesando el bloque.

### 10.3 El campo de obstáculos de `drone_model.cpp`

20 bloques (`g_obstacles[]`) repartidos dentro de los ±200 m de la grilla,
con alturas de **6 a 50 m**. El primero es el bloque de referencia, 100 m al
norte del origen y sobre el eje de crucero inicial, pensado para poder ir a
chocar de frente sin maniobrar. El resto obliga a elegir a qué **altura**
se cruza cada zona, no solo a esquivar de frente. Ningún par se solapa y
queda libre un radio de **≈32.9 m** alrededor del origen (valor verificado
geométricamente — distancia mínima real del origen a cualquier caja), para
que el vehículo nunca aparezca dentro de un bloque al arrancar o reiniciar.

Cada bloque se pinta con un degradado según su altura (ámbar los bajos,
violeta frío los altos); el bloque contra el que se chocó se marca en rojo.

### 10.4 "Pérdida de señal" al chocar (`g_crashed`)

Al chocar contra el piso o un obstáculo: la física se congela (`step()`
deja de integrarse), los controles de vuelo quedan bloqueados, la ventana
muestra una superposición de interferencia tipo "sin señal" de TV (ruido,
glitches, parpadeo rojo) y el HUD reemplaza la telemetría por `--.-` /
`[SIN SEÑAL]`. La única acción posible es `0` (reset completo) — simula la
pérdida total de comunicación con el dron tras el impacto.

## 11. `physics_UAV.cpp` — qué corre la demo de consola

Programa de validación/reporte offline, sin interacción. Secuencia real
(verificada ejecutando `make sim`):

1. **Masa/inercia del ala delta** (Bloque 0) — tabla de masa, CG y tensor
   de inercia completo.
2. **Reporte del convertiplano** (`reporteConvertiplano()`): masa/inercia
   en góndolas verticales, dimensiones reales (envergadura 5.6 m, ala
   S≈5.1 m², AR≈6.15), empuje estático máximo (T/W=2.0), velocidad a la que
   la caída de hélice anula el empuje a palanca máxima — y una **tabla de
   transición** de `sigma=0°` a `90°` en pasos de 15°, mostrando cómo se
   mueven CG e inercia, cómo cae el empuje disponible con la velocidad
   típica de cada punto de la transición, y cómo se reparte la autoridad de
   control `l, m, n` entre rotores y superficies (sección 7.3). Es la pieza
   central de este reporte: muestra numéricamente el "valle de potencia"
   clásico de un tiltrotor, donde el empuje de hélice se desploma justo
   antes de que el ala tenga presión dinámica suficiente para sostener el
   peso.
3. **Tabla de aerodinámica del ala** (Bloque 2b) — para velocidades de 5 a
   32 m/s, el ángulo de ataque que equilibraría el peso, `CL`, `CD`,
   sustentación, arrastre y la relación sustentación/peso (llega a 1.0 a
   partir de ~20 m/s con este vehículo).
4. **Demo de conversión de actitud** — Euler→Cuaternión→Euler (ida y
   vuelta, para verificar que no hay pérdida) y la matriz de rotación
   resultante.
5. **Simulación de 5 s de vuelo libre** del ala delta: empuje ≈ peso, sin
   torques, integrada con `simulateStep()` (RK4 + tensor de inercia
   completo `drone.I_full`, no la forma reducida), `dt=0.01 s`. Imprime
   `pn, pe, pd, u, phi, theta, p` cada 0.5 s y detecta/reporta el instante
   de colisión contra el piso, si ocurre.

## 12. `drone_model.cpp` — simulación visual interactiva

Ventana OpenGL/GLUT con el vehículo elegido en vuelo 6DOF, más una
**ventana secundaria de telemetría** (osciloscopio en vivo) con 5 paneles:
altitud, velocidad body (u,v,w), actitud (phi,theta,psi), tasas angulares
(p,q,r) y posición horizontal (pn,pe). Historial en buffer circular
(~48 s). Paso de integración fijo a ~60 fps (`DT = 0.016 s`, Euler
explícito — sección 8).

### 12.1 Elección de vehículo

Pantalla de selección al arrancar:

| Tecla | Vehículo | Masa | Particularidad de control |
|---|---|---|---|
| `1` | **UAV ala delta** | 200 kg | empuje longitudinal (+X cuerpo); la palanca controla **velocidad**, la altitud se sostiene sola |
| `2` | **Quadcopter (config X)** | 142 kg | empuje colectivo (−Z cuerpo); la palanca controla **ascenso/descenso**, avanzar requiere inclinar el vehículo |
| `3` | **Convertiplano (4 rotores basculantes)** | 154.9 kg | empuje vectorizado sobre el eje de góndola + aerodinámica real; único con masa/inercia variable y control por **torques** (Bloque 4 completo) |

También se puede saltear desde línea de comandos
(`./drone_model uav|quad|tilt`), y volver a la pantalla en pleno vuelo con
`v`. Rates iniciales `p q r` opcionales como argumentos posicionales.

Los tres comparten **toda** la simulación: mismo mundo, mismos obstáculos,
mismo piso, misma telemetría y los mismos Bloques 0-4 de `physics_UAV.h`.
Solo cambia lo que define a cada vehículo — masa/inercia (Bloque 0), modelo
de arrastre, y sobre qué eje del cuerpo entra el empuje al Bloque 2:

| | UAV ala delta | Quadcopter | Convertiplano |
|---|---|---|---|
| Empuje | longitudinal, eje **+X** cuerpo | colectivo, eje **−Z** cuerpo | vectorizado, eje de góndola: `n=(sin σ,0,−cos σ)` |
| Sustentación | fuerza vertical = peso (fija) | no hay; la da el propio empuje | real (Bloque 2b): función de `Va` y `alpha` |
| Arrastre | `aeroDrag` cuadrático calibrado | `aeroDrag` cuadrático, casi isótropo | Bloque 2b: `CD(alpha)·qbar·S` (no usa `aeroDrag`) |
| Rotación (p,q,r) | **directa por teclado**, Bloque 3 únicamente | **directa por teclado**, Bloque 3 únicamente | **Bloque 4 completo**: teclado pide tasas, control (7.3) calcula torques |
| Qué controla la palanca | velocidad | ascenso/descenso | empuje total de los 4 rotores (potencia) |
| Estado adicional | — | — | ángulo de basculación `sigma` (`[`/`]`/`t`) |
| Palanca de arranque | 3% (equilibrio a 5 m/s crucero) | 45% (hover, T/W máx=2.2) | 50% (hover, T/W=2.0) |
| Estado inicial | 50 m, crucero a 5 m/s | 50 m, vuelo estacionario | 50 m, vuelo estacionario, góndolas verticales |

### 12.2 Controles

| Tecla | Acción (ala delta / quad) | Acción (convertiplano) |
|---|---|---|
| `← →` | Roll rate `p` directo (±0.02 rad/s) | Tasa de alabeo **comandada** `p_cmd` (máx 0.60 rad/s) |
| `↑ ↓` | Pitch rate `q` directo (±0.02 rad/s) | Tasa de cabeceo **comandada** `q_cmd` (máx 0.50 rad/s) |
| `A` / `D` | Yaw rate `r` directo (±0.02 rad/s) | Tasa de guiñada **comandada** `r_cmd` (máx 0.40 rad/s) |
| `W` (mantener) | Subir palanca | Subir palanca (potencia de los 4 rotores) |
| `S` (mantener) | Bajar palanca / reversa | Bajar palanca |
| `[` / `]` | — | Bascular góndolas hacia estacionario / hacia crucero |
| `T` | — | Transición automática a crucero (90°) |
| `Espacio` | Pausa / reanuda | |
| `0` | Reset completo | (incluye `sigma=0`, tasas comandadas a 0) |
| `V` | Volver a pantalla de selección | |
| `R` | Reset de cámara | |
| `Shift+W` | Wireframe on/off | |
| `+` / `-` | Zoom | |
| `Q` / `Esc` | Salir | |

```
./drone_model                  # pantalla de selección
./drone_model quad              # arranca directo en quadcopter
./drone_model tilt               # arranca directo en convertiplano
./drone_model uav 0.01 0 0      # ala delta con roll rate inicial
```

## 13. `launch_rockets/` — mecánica Lagrangiana de un cohete

Proyecto físicamente independiente (coordenadas generalizadas `x,y,z` en el
marco inercial Este-Norte-Arriba, no marco cuerpo), incluido en el mismo
repositorio. Formula el movimiento vía el **Lagrangiano** `L = T − V` en
vez de fuerzas en marco cuerpo como el motor UAV:

```
T = ½ m(t) (x_dot² + y_dot² + z_dot²)        (energia cinetica)
V = m(t)·g·z                                  (energia potencial)

Ecuaciones de Euler-Lagrange con fuerzas generalizadas no conservativas Qi:
  d/dt(dL/dq_dot_i) - dL/dq_i = Qi
  ->  m·x_dot_dot = Fx_empuje + Fx_arrastre
      m·y_dot_dot = Fy_empuje + Fy_arrastre
      m·z_dot_dot = -m·g + Fz_empuje + Fz_arrastre
      m_dot       = -fuel_flow_rate            (durante la combustion)
```

**Empuje** — ecuación del cohete (Tsiolkovsky), a partir del impulso
específico `Isp` y el caudal de combustible `mdot = fuel_mass/burn_time`:

```
T_empuje = Isp · g0 · mdot          (direccion fija: thrustDir(theta,phi))
```

y cantidades derivadas para caracterizar el lanzamiento: masa seca,
relación de masas, **Δv** (`Isp·g0·ln(masa_total/masa_seca)`) y relación
empuje/peso.

**Arrastre aerodinámico** con atmósfera exponencial ISA simplificada:

```
rho(z) = rho0 · e^(-z/H)              H = 8500 m (altura de escala)
F_drag = -½ · rho(z) · Cd · A · |v| · v_hat
```

**Integración:** mismo esquema **RK4 clásico de 4 etapas** (`rk4Step()`,
pesos 1-2-2-1) que el motor UAV, aplicado a las 7 variables de estado
(posición, velocidad, masa).

Compilación con CMake (independiente del `Makefile` del motor UAV):

```
cd launch_rockets
cmake -B build && cmake --build build
./build/rocket_sim
```

Controles: `SPACE` lanza, `R` resetea, `1-7` selecciona parámetro,
`+`/`-` lo ajusta, `WASD`/flechas mueven la cámara libre, `T` activa cámara
de seguimiento del cohete, `ESC` sale.

## 14. Limitaciones conocidas / simplificaciones deliberadas

- **Colisión como punto:** tanto el piso como los obstáculos tratan al
  vehículo como su CG — no hay geometría de colisión real del fuselaje/ala.
- **Sin viento ni turbulencia:** `Va = |v_body|` siempre (datos de aire sin
  componente de viento).
- **Ala delta y quad saltean el Bloque 4** en la demo 3D: `p,q,r` son
  input directo de teclado, no la salida de integrar torques — es una
  simplificación deliberada de control para esos dos vehículos (ver
  sección 8); el convertiplano es el único que integra la cadena completa.
- **`aeroDrag` es un modelo de arrastre, no de sustentación:** el ala delta
  "sostiene" la altitud fijando la fuerza vertical igual al peso, no
  derivándola de `CL(alpha)` como el convertiplano.
- **Gimbal lock sin mitigar en la integración:** el motor advierte por
  `stderr` cerca de `theta=±90°`, pero sigue integrando en ángulos de
  Euler (los cuaterniones son solo una utilidad de conversión/reporte, no
  reemplazan al integrador).
- **`rot_trans_kinematics` referenciado en el `Makefile` no existe** como
  archivo fuente en esta carpeta.

## 15. Compilar y ejecutar

```
make view     # compila y abre la simulación 3D interactiva (drone_model)
make sim      # compila y corre la demo de consola (physics_UAV)
make clean    # borra los binarios compilados
```

Requiere `clang++` con soporte de C++17 y los frameworks `OpenGL`/`GLUT` de
macOS (usados solo por `drone_model`). Para `launch_rockets/`, ver
[sección 13](#13-launch_rockets--mecánica-lagrangiana-de-un-cohete) (CMake).

## 16. Referencia

Beard, R. W. & McLain, T. W., *Small Unmanned Aircraft: Theory and
Practice*. Cap. 3 (ecuaciones de movimiento de cuerpo rígido), Cap. 4
(fuerzas y momentos aerodinámicos, modelo de hélice), Apéndice E
(coeficientes del Aerosonde).
