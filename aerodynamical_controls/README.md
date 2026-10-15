# aerodynamical_controls — qué se hizo y cómo correrlo

Fecha: 2026-09-02 (aeroelasticidad anadida el 2026-09-03)

Todo el módulo vive en `simulacion_alerones/`. Es un simulador interactivo de
la dinámica de **actitud** de un cohete con cuatro aletas traseras en cruz
(convención Blakelock Fig. 8-2). La documentación completa —deducciones,
fórmulas, tabla de teclas— está en `simulacion_alerones/README.md`.

---

## Cómo correrlo

```bash
cd simulacion_alerones

make run        # ventana 3D interactiva (OpenGL/GLUT)
make test       # 55 comprobaciones, sin gráfica
make paridad    # C++ vs Python, byte a byte
make figuras    # genera figuras/*.csv para el libro
make help       # todos los targets
```

Requiere `clang++` con C++17 y OpenGL/GLUT (en macOS vienen con el SDK; en
Linux, `freeglut3-dev`). El prototipo Python necesita `numpy` y `matplotlib`.

### Modo headless (sin gráfica)

REPL por `stdin` que ignora comentarios con `#`, así que se le da de comer
cualquier fichero. Hay uno por sección teórica:

```bash
./alerones --headless < scripts/desacoplo.txt        # §2  desacoplo por simetría
./alerones --headless < scripts/amortiguamiento.txt  # §4  con y sin amortiguar
./alerones --headless < scripts/aeroelasticidad.txt  # §5  aleta flexible
./alerones --headless < scripts/actuador.txt         # §6  retardo y rate limit
./alerones --headless < scripts/autopiloto.txt       # §6  escalón de actitud
./alerones --headless < scripts/regresion.txt        # traza de regresión
```

`help` dentro del REPL lista los comandos.

### Teclas principales del modo gráfico

`A` autopiloto · `C` hold · `D` amortiguamiento · `f` aleta flexible/rígida ·
`F` exagerar la deformación · `S` servo · flechas y `Q`/`E` mando (o
referencias si el AP está activo) · `1`…`4` + `+`/`-` aleta suelta · `SPACE`
pausa · `R` reset · `,`/`.` velocidad · `T` vista de cola · `ESC` salir.

---

## Qué se hizo (2026-09-02)

Estado de partida: el motor físico, el render y 23 tests ya funcionaban. Lo
que faltaba, y se completó:

**1. Documentación.** No había `README.md`, y el código citaba §2, §7 y §8 de
un documento inexistente. Ahora existe, con §1–§9 y una sección explícita de
alcance ("Qué falta a propósito").

**2. Amortiguamiento aerodinámico (§4)** — era el hueco físico real. Cada
aleta ve la incidencia inducida por `ω × r_i`. Sin esto, una deflexión
constante daba una velocidad angular que crecía linealmente para siempre.
Resultado central: **la matriz de derivadas es diagonal en la misma base de
irreps que diagonaliza el mando** (`L_p`, `M_q`, `N_r`, cero términos
cruzados), y `ω` converge a `V·u/brazo`.

**3. Dinámica de actuador (§6)** — `delta_cmd` y `delta` son estados
distintos; retardo de primer orden más límite de velocidad.

**4. Autopiloto (§6)** — tres PID SISO, uno por irrep, sintonizados por
asignación de polos sobre el modelo linealizado, así que se resintonizan solos
al cambiar `V` o la geometría. Las ganancias de guiñada salen **negativas**:
es `N_du < 0` de la convención Blakelock, no una errata.

**5. Tensor de inercia con `Ixz`** (por defecto 0), para poder mostrar el
acoplo alabeo–guiñada *inercial*, distinto del aerodinámico.

**6. Bugs cerrados** — `set S_fin`/`b_fin` no recalculaban `AR → CL_delta`;
`.PHONY` incompleto en el Makefile; faltaba `.gitignore`; el REPL de Python no
tenía los mismos comandos que el de C++.

**7. Tests: 23 → 55**, y el prototipo `alerones_py/main.py` porteado entero.

---

## Qué se hizo (2026-09-03): aletas flexibles

Antes la aleta era una placa infinitamente rígida que iba exactamente donde el
servo la mandaba. Ahora cada aleta es un **sistema de dos grados de libertad
excitado por su propia fuerza aerodinámica** (§5 del README):

- **Torsión** sobre el pivote. Es la que importa: entra en el ángulo de ataque
  y cierra un lazo — la carga retuerce la aleta, y la aleta retorcida cambia la
  carga. De ahí sale un solo número, la **eficiencia aeroelástica**
  `eta = 1/(1 + qbar/qbar_car)`, que dice qué fracción del mando llega de
  verdad al aire.
- **Flexión** de la punta, con la deformada exacta de un voladizo. No cambia la
  incidencia (aleta recta): es el modo que *se ve*.
- El parámetro `e_ea` (distancia del pivote al centro aerodinámico) parte el
  problema en dos: con el centro aerodinámico por **detrás** se pierde eficacia
  de mando al ir rápido; con el pivote por detrás la aleta **diverge** por
  encima de cierta presión dinámica.
- El autopiloto se sintoniza sobre `eta · M_du`, así que compensa solo la
  pérdida de autoridad.

Dos cosas que conviene saber:

- **Los valores por defecto no exageran el efecto.** A 30 m/s se pierde un 0.4%
  del mando: existe y no se nota. A 486 m/s se pierde la mitad. La lección es
  que *la aeroelasticidad es cosa de ir rápido*, y falsearla sería mentir sobre
  a qué régimen pertenece. Para verla a baja velocidad, ablanda la aleta:
  `set K_theta 5`.
- **El dibujo sí exagera**, porque la flecha real es de décimas de milímetro.
  El factor se anuncia en el HUD y la tecla `F` lo cicla (×1 … ×150).

Los modos estructurales (37 y 65 Hz) son ahora quienes fijan el paso de
integración: con `dt = 10 ms` RK4 diverge a NaN. Hay una guarda (`structural_cfl`)
que avisa en el HUD, al arrancar y tras cualquier `set` que empeore el número.

**Tests: 55 → 79.** Los casos 15-18 cubren eficiencia estática, caída con la
presión dinámica, divergencia y el límite de paso.

---

## Verificación

- 79/79 en C++ y en Python.
- `make paridad` confirma que las dos implementaciones producen la misma traza
  **byte a byte**, en los 79 casos y en los seis scripts. Son dos
  implementaciones independientes: no pueden copiarse un error numérico.
- Compilación limpia con `-Wall`, sin warnings.

Dos ajustes de impresión hicieron falta para la paridad exacta, ambos
explicados en §7 del README: las combinaciones antisimétricas (`u_roll`, `L`)
son cancelación total de cuatro números casi iguales y al converger quedan en
~1e-31 con el signo dictado por el redondeo (se aplastan a cero por debajo de
1e-9); y `qbar` se imprime con 4 decimales porque `ρV²/2` cae en empates
exactos (`x.xxx5`) que C++ y Python rompen en direcciones opuestas.

---

## Alcance: qué NO hace

Simula **solo actitud**. No hay posición, velocidad de traslación, gravedad ni
empuje: `V` es un parámetro, no un estado. Para el vuelo 6DOF completo está el
módulo hermano `../unmanned/`. Tampoco modela resistencia de las aletas,
amortiguamiento del fuselaje, compresibilidad ni histéresis de pérdida. La
aeroelasticidad de §5 usa aerodinámica cuasi-estacionaria: da bien la
eficiencia y la divergencia, que son estáticas, pero no una velocidad de flameo
fiable. El detalle está al final del README.
