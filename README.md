# UAV-s- — Simulación, control y detección de vehículos aéreos no tripulados

Repositorio con tres proyectos independientes: dos de simulación física en
C++ (basados en las ecuaciones de Beard & McLain, *Small Unmanned Aircraft:
Theory and Practice*) y uno de diseño/análisis de radar en MATLAB, pensados
para compilarse/correrse tal cual al clonar este repo.

## Proyectos

| Carpeta | Qué hace | README |
|---|---|---|
| [`unmanned/`](unmanned/README.md) | Motor de vuelo **6DOF completo** (posición + actitud) de tres vehículos — UAV ala delta, quadcopter y convertiplano de rotores basculantes — con visualización 3D interactiva, telemetría en vivo, campo de obstáculos y una demo de consola. Incluye además, como módulo aparte, una simulación de lanzamiento de cohete con mecánica Lagrangiana (`unmanned/launch_rockets/`). | [`unmanned/README.md`](unmanned/README.md) |
| [`aerodynamical_controls/`](aerodynamical_controls/README.md) | Dinámica de **actitud** (sin posición ni traslación) de un cohete con cuatro aletas traseras en cruz: desacoplo por simetría, amortiguamiento aerodinámico, aeroelasticidad de aleta flexible, dinámica de actuador y autopiloto PID. Validado con 79 casos de test y paridad byte a byte entre una implementación en C++ y otra en Python. | [`aerodynamical_controls/README.md`](aerodynamical_controls/README.md) |
| [`MATLAB/`](MATLAB/README.md) | Diseño y análisis de un **radar PMCW** para detección de UAVs pequeños (presupuesto de detectabilidad, Pd vs. rango, cobertura rango-Doppler, dos modelos de refracción atmosférica) más el diseño físico preliminar del arreglo de antenas de 64Tx/64Rx a 79 GHz que lo alimentaría. | [`MATLAB/README.md`](MATLAB/README.md) |

Los tres se complementan: `unmanned/` cubre el vuelo completo de un vehículo
rígido (dónde está y hacia dónde apunta), `aerodynamical_controls/`
profundiza en el lazo de control de **actitud** de las superficies
aerodinámicas que hacen posible ese vuelo, y `MATLAB/` resuelve el problema
inverso — **detectar** un vehículo como esos desde un radar en tierra —
cada README de proyecto referencia a los otros donde corresponde.

| `unmanned/` — UAV ala delta en vuelo, telemetría en vivo | `aerodynamical_controls/` — cohete con aletas cruciformes, vista de cola y HUD |
|---|---|
| ![UAV ala delta en vuelo, con la ventana de telemetría en vivo](unmanned/ala_delta.png) | ![Cohete con aletas cruciformes en vuelo, con vista de cola y HUD de actitud](aerodynamical_controls/simulacion_alerones/aerodynamical_controls.png) |

## Requisitos

`clang++` con soporte C++17. `unmanned/drone_model.cpp` y
`aerodynamical_controls/simulacion_alerones` usan OpenGL/GLUT (en macOS
vienen con el SDK; en Linux, `freeglut3-dev`). El módulo
`unmanned/launch_rockets/` compila con CMake. El prototipo Python de
`aerodynamical_controls` necesita `numpy` y `matplotlib`. `MATLAB/` necesita
MATLAB con Radar Toolbox, Phased Array System Toolbox y Antenna Toolbox.

Cada carpeta de proyecto trae su propio `Makefile` (o `CMakeLists.txt`) y
su propio README con las instrucciones de compilación y ejecución
detalladas.

## Referencia

Beard, R. W. & McLain, T. W., *Small Unmanned Aircraft: Theory and
Practice*.
