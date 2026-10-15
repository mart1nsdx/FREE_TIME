# MATLAB — Diseño y análisis de radar para detección de drones

Colección de scripts de MATLAB (Radar Toolbox, Phased Array System Toolbox,
Antenna Toolbox) para el diseño y análisis de un radar **PMCW** (Phase-
Modulated Continuous Wave) orientado a la detección de UAVs pequeños, más el
diseño físico preliminar del arreglo de antenas en 79 GHz que lo alimentaría.
Es, junto con `unmanned/` y `aerodynamical_controls/`, la tercera pieza del
problema: los otros dos simulan cómo vuela y se controla el vehículo; esto
simula **cómo detectarlo** desde tierra.

A diferencia de `unmanned/` y `aerodynamical_controls/`, este no es un único
programa con una demo clara: es una colección de iteraciones de diseño
(v1 → v2 → PMCW corregido → arreglo físico en 79 GHz) guardadas tal como se
fueron generando, varias de ellas exportadas directamente desde las apps
*Radar Designer* y *Sensor Array Analyzer* de MATLAB. Se documentan todas
para que se entienda qué prueba cada una y en qué orden aparecieron.

## Estructura

| Archivo / carpeta | Rol |
|---|---|
| `PMCW.m`, `v3_PMCW.m` | Script principal de análisis de detectabilidad del radar PMCW (idénticos byte a byte — `v3_PMCW.m` es la copia de la última iteración guardada con nombre de versión). Ver [§1](#1-pmcwm--v3_pmcwm-análisis-de-detectabilidad-pmcw). |
| `array_script.m` | Export de la app *Sensor Array Analyzer*: arreglo URA 8×8 con taper de Taylor, replicado 2×2, a 81 GHz. Estudio preliminar de patrón de array, anterior al diseño físico de `PMCW_79GHz_Array_Design_Package/`. Ver [§2](#2-array_scriptm--estudio-de-patrón-del-arreglo). |
| `PMCW_79GHz_Array_Design_Package/` | Diseño físico preliminar del arreglo de antenas de 79 GHz (parche inset-fed, red de alimentación, 4 paneles basculados, 64 canales Tx/64 Rx) que alimentaría al radar PMCW. Trae su propio `README_PMCW_79GHz_Array.txt`. Ver [§3](#3-pmcw_79ghz_array_design_package--diseño-físico-del-arreglo-de-antenas). |
| `v2_solution/` | Iteración anterior del análisis de radar: exports crudos de *Radar Designer* (`radar1_*.m`, preset genérico `AirborneRadar`) junto a las versiones ya adaptadas a detección de drones (`Pmcw_*.m`), con dos modelos de propagación atmosférica distintos. Ver [§4](#4-v2_solution--iteración-anterior-del-análisis-de-radar). |
| `radar_scrips/PatchFisico_&_array_ComplexGeometries/array_v1_porfin.mat` | Datos de una primera iteración ("v1") de geometría de arreglo/parche con geometrías complejas — sin script asociado, solo el `.mat` con los resultados. |
| `RADAR_BUENO.mat` | Export de *Radar Toolbox* con los parámetros del radar ya ajustado ("bueno" = la versión validada). Lo importa opcionalmente `PMCW_79GHz_Array_Design_Package/PMCW_79GHz_Array_Design_LiveScript_Source.m` (§3) para listar sus campos si está presente. |

## 1. `PMCW.m` / `v3_PMCW.m` — análisis de detectabilidad PMCW

Adaptación de un export de *Radar Toolbox* (`TrackingRadar`) de pulsos a un
radar de **onda continua modulada en fase** (PMCW), mapeando los parámetros
de un radar pulsado a los de PMCW:

```
Duracion de un chip (1/B)          -> Pulsewidth   (energia de una sola muestra)
Chips por CPI (B x T_int)          -> NumCoherentPulses  (ganancia de correlacion)
Potencia promedio (CW, duty=100%)  -> PeakPower
```

**Forma de onda:** ancho de banda del código 15 MHz (chip ≈ 66.7 ns),
tiempo de integración coherente 10 ms (150 000 chips/CPI), código Gold de
1023 chips, potencia promedio 125 W, ganancia de procesamiento
`10·log10(NumChips) ≈ 51.8 dB`.

**Radar equivalente (`designRadarPMCW`):** antena de 4×[8×8] = 256
elementos (patrón sinc), frecuencia 10 GHz, CFAR GOCA (*Greatest-Of Cell
Averaging*), M-de-N binario (detecta en ≥1 de 2 CPIs), STC (*sensitivity
time control*) con corte a 1000 m.

**Blanco:** RCS = 1e-3 m² (micro-UAV), fluctuación **Swerling 1**,
elevación 5°.

**Presupuesto de detectabilidad** (`effectiveDetectability` /
`detectabilityComponents`): descompone el SNR requerido en 7 términos —
detectabilidad de un solo chip (Swerling 0), pérdida por fluctuación
Swerling 1, ganancia de correlación PMCW, pérdida CFAR, pérdida de forma de
haz, pérdida de barrido electrónico del array y pérdidas propias
(aislación Tx/Rx) — más una ganancia adicional por integración binaria
entre 2 CPIs.

**Pérdidas de propagación:** `radarpropfactor` (multipath/difracción sobre
tierra curva), más pérdidas atmosféricas (`tropopl`, gas) y de lente
(`lenspl`) por el modelo de **radio efectivo de la Tierra** (4/3).

**Salida (4 figuras):** SNR vs. rango, Pd vs. rango (contra umbral 70% /
objetivo 95%), presupuesto de enlace (*link budget*, barra apilada por
componente) y cobertura rango-Doppler (mapa de calor de SNR + máscara de
detección, con pérdida por desajuste Doppler tipo `sinc²`).

## 2. `array_script.m` — estudio de patrón del arreglo

Generado por la app *Sensor Array Analyzer* (Phased Array System Toolbox).
Arreglo rectangular uniforme (URA) 8×8 con taper de Taylor (−30 dB de lóbulo
lateral, `nbar=5`) por fila y columna, elemento de patrón sinc (ancho de haz
120°×120°), replicado en una grilla 2×2 (`ReplicatedSubarray`) con desfasador
de 3 bits a 81 GHz. Genera 4 gráficos: geometría del arreglo, directividad
3D, corte en azimut y corte en elevación. Es el estudio preliminar de patrón
que precede al diseño físico del parche en `PMCW_79GHz_Array_Design_Package/`.

## 3. `PMCW_79GHz_Array_Design_Package/` — diseño físico del arreglo de antenas

Síntesis preliminar (no lista para fabricación) de un arreglo físico de
**64 canales Tx / 64 Rx** a 79 GHz, inspirado en el nivel de potencia por
canal de RF de un chipset tipo AWR2943/AWR2944. Contenido real del paquete
(ver también su propio `README_PMCW_79GHz_Array.txt`):

- **Sustrato:** clase RT/duroid 5880 (`er=2.2`, `h=0.127 mm`).
- **Síntesis del parche** inset-fed: dimensiones de primera pasada
  `W≈1.50 mm`, `L≈1.20 mm`, offset de alimentación `≈0.42 mm`; tabla de
  anchos de línea de microcinta para impedancias características de 25 a
  150 Ω.
- **Apertura mecánica:** 150×150 mm máximo, repartida en **4 paneles de
  75×75 mm basculados 10°** en arreglo tipo pirámide.
- Cada canal de RF alimenta una sub-tila fija de parches 4×4 con
  ponderación de amplitud (red de alimentación corporativa); total
  **1024 parches Tx + 1024 parches Rx**.
- Incluye primera pasada de dimensiones de valla EBG/metamaterial y
  visualización 3D de la apertura completa (`PMCW_79GHz_Array_3D_Preview.png`,
  radiadores Tx en azul / Rx en rojo).
- Salida: `radarAntennaUpdate_79GHz.mat` (generado al correr el script) con
  frecuencia, potencias, ganancias, anchos de haz, dimensiones del parche,
  pérdidas de alimentación y temperatura de sistema, para integrar al radar
  de la sección 1.
- `make_mlx_in_matlab.m` — convierte la fuente `.m` seccionada a
  `.mlx` (Live Script) corriéndolo dentro de MATLAB; no ejecutable fuera de
  MATLAB.

> Nota de ingeniería explícita en el propio paquete: es una síntesis de
> primera pasada y una simulación de factor de arreglo, **no** un diseño de
> PCB de milimétricas listo para manufactura — falta validación con
> solver electromagnético de onda completa (HFSS, CST, FEKO) y medición.

## 4. `v2_solution/` — iteración anterior del análisis de radar

Snapshot de una iteración previa a `PMCW.m`, con dos familias de scripts:

- **`radar1_*.m`** — exports crudos de la app *Radar Designer* sobre el
  preset `AirborneRadar` (genérico, sin adaptar a dron): detectabilidad,
  reporte de métricas, cobertura vertical. Sirven de línea base / plantilla.
- **`Pmcw_*.m`** — las mismas tres salidas ya **corregidas para detección de
  drones pequeños**: RCS = 0.01 m² (−20 dBsm) en vez del valor genérico,
  elevación del blanco llevada a 0° (peor caso de propagación), fluctuación
  Swerling 1 (antes Swerling 0), PMCW con `Pulsewidth = 1/PRF` (duty 100% →
  +3 dB) y lluvia moderada (10 mm/h) en vez de torrencial. Hay **dos
  variantes del modelo de refracción atmosférica** evaluadas en paralelo:
  `Pmcw_EffectiveEarthRadius.m` (aproximación de radio efectivo 4/3, la
  misma que usa `PMCW.m`) y `Pmcw_CRLExponential.m` (modelo exponencial
  CRPL: `N(h) = Ns·exp(-ce·h)`, más preciso para atmósferas no estándar).
- `radar1.mat` — datos asociados a esta iteración.

Es, en esencia, el "antes" de `PMCW.m`: muestra el camino desde un preset
genérico de la toolbox hasta un radar corregido específicamente para el
caso de uso (drones pequeños a baja elevación).

## Requisitos

MATLAB con **Radar Toolbox**, **Phased Array System Toolbox** y
**Antenna Toolbox** (generado y probado en R2026a). Los scripts son
`.m` sueltos, no un proyecto de MATLAB (`.mlx`/`.prj`) empaquetado —
se corren abriéndolos y ejecutándolos directamente, o con `run('archivo.m')`
desde la carpeta correspondiente (algunos, como los de
`PMCW_79GHz_Array_Design_Package/`, esperan sus datos de entrada —
`RADAR_BUENO.mat`— en la misma carpeta de trabajo).

## Referencia

MATLAB Radar Toolbox, Phased Array System Toolbox y Antenna Toolbox
(MathWorks, R2026a). Modelo de refracción CRPL exponencial y de radio
efectivo de la Tierra: documentación de `radarpropfactor` / `tropopl` /
`lenspl` de Radar Toolbox.
