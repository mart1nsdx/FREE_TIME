# UAV-s- — Simulación, control y detección de vehículos aéreos no tripulados

Repositorio con cuatro proyectos independientes: tres de simulación física
en C++ (dos basados en las ecuaciones de Beard & McLain, *Small Unmanned
Aircraft: Theory and Practice*, y uno de mecánica Lagrangiana) y uno de
diseño/análisis de radar en MATLAB, pensados para compilarse/correrse tal
cual al clonar este repo.

## Proyectos

| Carpeta | Qué hace | README |
|---|---|---|
| [`unmanned/`](unmanned/README.md) | Motor de vuelo **6DOF completo** (posición + actitud) de tres vehículos — UAV ala delta, quadcopter y convertiplano de rotores basculantes — con visualización 3D interactiva, telemetría en vivo, campo de obstáculos y una demo de consola. Incluye además, como módulo aparte, una simulación de lanzamiento de cohete con mecánica Lagrangiana (`unmanned/launch_rockets/`). | [`unmanned/README.md`](unmanned/README.md) |
| [`aerodynamical_controls/`](aerodynamical_controls/README.md) | Dinámica de **actitud** (sin posición ni traslación) de un cohete con cuatro aletas traseras en cruz: desacoplo por simetría, amortiguamiento aerodinámico, aeroelasticidad de aleta flexible, dinámica de actuador y autopiloto PID. Validado con 79 casos de test y paridad byte a byte entre una implementación en C++ y otra en Python. | [`aerodynamical_controls/README.md`](aerodynamical_controls/README.md) |
| [`rockets_sim/`](rockets_sim/README.md) | Lanzamiento de cohetes en **primera persona** con mecánica Lagrangiana (mismo motor que `unmanned/launch_rockets/`): panel de parámetros de misión editable en vivo (ángulo, Isp, masas, arrastre), panel de propulsión derivada (T/W, *mass ratio*, Δv) y telemetría de vuelo en tiempo real. | [`rockets_sim/README.md`](rockets_sim/README.md) |
| [`MATLAB_RADAR/`](MATLAB_RADAR/README.md) | Diseño y análisis de un **radar PMCW** para detección de UAVs pequeños (presupuesto de detectabilidad, Pd vs. rango, cobertura rango-Doppler, dos modelos de refracción atmosférica) más el diseño físico preliminar del arreglo de antenas de 64Tx/64Rx a 79 GHz que lo alimentaría. | [`MATLAB_RADAR/README.md`](MATLAB_RADAR/README.md) |

Se complementan: `unmanned/` cubre el vuelo completo de un vehículo rígido
(dónde está y hacia dónde apunta), `aerodynamical_controls/` profundiza en
el lazo de control de **actitud** de las superficies aerodinámicas que
hacen posible ese vuelo, `rockets_sim/` es la consola de diseño de misión
del mismo motor de lanzamiento de cohete que usa `unmanned/`, y
`MATLAB_RADAR/` resuelve el problema inverso — **detectar** un vehículo
como esos desde un radar en tierra. Cada README de proyecto referencia a
los otros donde corresponde.

| `unmanned/` — UAV ala delta en vuelo, telemetría en vivo | `aerodynamical_controls/` — cohete con aletas cruciformes, vista de cola y HUD |
|---|---|
| ![UAV ala delta en vuelo, con la ventana de telemetría en vivo](unmanned/ala_delta.png) | ![Cohete con aletas cruciformes en vuelo, con vista de cola y HUD de actitud](aerodynamical_controls/simulacion_alerones/aerodynamical_controls.png) |

| `rockets_sim/` — plataforma de lanzamiento, panel de propulsión derivada | `rockets_sim/` — cohete en fase de quema, telemetría en vivo |
|---|---|
| ![Plataforma de lanzamiento con paneles de parámetros y propulsión derivada](rockets_sim/rocket_launch1.png) | ![Cohete en vuelo durante la fase de quema, con telemetría en vivo](rockets_sim/rocket_launch2.png) |

## Requisitos

`clang++` con soporte C++17. `unmanned/drone_model.cpp` y
`aerodynamical_controls/simulacion_alerones` usan OpenGL/GLUT (en macOS
vienen con el SDK; en Linux, `freeglut3-dev`). `unmanned/launch_rockets/` y
`rockets_sim/` compilan con CMake (mismo requisito de OpenGL/GLUT). El
prototipo Python de `aerodynamical_controls` necesita `numpy` y
`matplotlib`. `MATLAB_RADAR/` necesita MATLAB con Radar Toolbox, Phased
Array System Toolbox y Antenna Toolbox.

Cada carpeta de proyecto trae su propio `Makefile` (o `CMakeLists.txt`) y
su propio README con las instrucciones de compilación y ejecución
detalladas.

## Referencia

Beard, R. W. & McLain, T. W., *Small Unmanned Aircraft: Theory and
Practice*.
