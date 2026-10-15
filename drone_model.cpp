/**
 * drone_model.cpp
 *
 * 3D visualization — motor de vuelo completo (6DOF)
 *
 * Usa el mismo modelo fisico que physics_UAV.cpp (physics_UAV.h: Bloques
 * 0-4 de Beard & McLain Cap. 3) con la masa/inercia real del dron ala
 * delta (buildDeltaWingDrone()). El dron se dibuja centrado en pantalla;
 * el mundo (ejes, horizonte, grilla de suelo) se traslada en sentido
 * opuesto a su posicion inercial para que el vuelo se vea (camara
 * "chase" centrada en el dron).
 *
 * Controles:
 *   <- ->  arrows    p  (roll rate  ±0.02 rad/s, directo, Bloque 3)
 *   up dn  arrows    q  (pitch rate ±0.02 rad/s, directo, Bloque 3)
 *   A / D            r  (yaw rate   ±0.02 rad/s, directo, Bloque 3)
 *   w  (mantener)    palanca de gases + (throttle up,  Bloques 1-2)
 *   s  (mantener)    palanca de gases - (throttle down / reversa, Bloques 1-2)
 *   Space            pause / resume
 *   0                reset estado completo (posicion, velocidad, actitud, palanca)
 *   R                reset camara
 *   Shift+W          wireframe toggle
 *   + / -            zoom
 *   Q / Esc          quit
 */

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#include <GLUT/glut.h>
#pragma clang diagnostic pop

#include <OpenGL/glu.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "physics_UAV.h"

// ═══════════════════════════════════════════════════════════════════════
// FISICA — masa/inercia real (Bloque 0) + dinamica 6DOF (Bloques 1-4)
// ═══════════════════════════════════════════════════════════════════════

static const double GRAVITY = 9.81;
static const double PISO_PD = 0.0;   // suelo solido en el plano X-Y (pd=0, NED)

// ─────────────────────────────────────────────────────────────────────────
// PALANCA DE GASES (throttle) — W/S mueven una posicion de palanca 0..1
// (negativa hasta THROTTLE_MIN = reversa/freno), no una fuerza directa.
// Mientras se mantiene la tecla, rampa a THROTTLE_RATE fraccion/s (no salta
// por evento, para no depender del auto-repeat del teclado); se sostiene al
// soltar, como el acelerador de un avion. fx_thrust = throttle * FX_MAX.
//
// Por si sola la palanca no basta para "sentirse" como un avion: hace falta
// arrastre aerodinamico (mas abajo, K_DRAG_*) para que cada posicion tenga
// un punto de equilibrio real (empuje = drag) en vez de acelerar sin limite.
// ─────────────────────────────────────────────────────────────────────────
static const double THROTTLE_RATE =    0.4;          // fraccion/s mientras se mantiene w/s
static const double FX_MAX        = 4000.0;          // N — empuje maximo (throttle=1.0)
static const double FX_MIN        = -1500.0;         // N — empuje minimo (throttle negativo, reversa/freno)
static const double THROTTLE_MIN  = FX_MIN / FX_MAX;  // ~ -0.375

// Arrastre aerodinamico (Bloque 2, aeroDrag() en physics_UAV.h) — cuadratico
// por eje del marco cuerpo. K_DRAG_U se calibra para que a throttle=1.0 el
// dron se estabilice en V_CRUISE_MAX m/s de crucero longitudinal; K_DRAG_V/W
// son mayores porque el area expuesta de costado / arriba-abajo (fuselaje +
// ala) es mucho mayor que la frontal (solo la nariz) — tambien frena la
// deriva lateral/vertical que antes quedaba sin oponerse a nada.
static const double V_CRUISE_MAX = 28.0;                              // m/s @ throttle=1.0
static const double K_DRAG_U     = FX_MAX / (V_CRUISE_MAX*V_CRUISE_MAX);  // ~5.10 N/(m/s)^2
static const double K_DRAG_V     = K_DRAG_U * 4.0;
static const double K_DRAG_W     = K_DRAG_U * 4.0;
static const DragCoeffs g_dragCoeffs{ K_DRAG_U, K_DRAG_V, K_DRAG_W };

static const double U_INIT = 5.0;   // m/s crucero inicial (coincide con g_state.u en main())
// Throttle que sostiene U_INIT en equilibrio (empuje = drag a esa velocidad)
static const double THROTTLE_INIT = (K_DRAG_U * U_INIT * U_INIT) / FX_MAX;

// ─────────────────────────────────────────────────────────────────────────
// VEHICULOS — el ala fija de siempre y un quadcopter
//
// Los dos corren EXACTAMENTE la misma fisica: Bloque 0 (masa/inercia derivada
// de geometria real), Bloques 1-4 de physics_UAV.h, el mismo aeroDrag(), el
// mismo piso y los mismos obstaculos. Lo unico que cambia es lo que define a
// cada vehiculo: sus propiedades de masa, sus coeficientes de arrastre y —lo
// importante— COMO entra el empuje al Bloque 2:
//
//   Ala fija : el empuje es longitudinal (eje +X cuerpo, la helice tira hacia
//              adelante) y la sustentacion del ala se modela como una fuerza
//              vertical que iguala al peso, asi que la altitud se sostiene
//              sola y la palanca controla velocidad. Es el modelo original,
//              intacto.
//   Quad     : el empuje es COLECTIVO, sobre el eje -Z cuerpo (los cuatro
//              rotores empujan "hacia arriba" del cuerpo). No hay ala que
//              sostenga nada: la palanca controla ascenso/descenso y para
//              desplazarse hay que inclinar el vehiculo, que es lo que hace
//              que el vector de empuje se proyecte hacia adelante. Sale de
//              las mismas ecuaciones, sin tocarlas.
//   Convertiplano : las 4 gondolas BASCULAN. El empuje sale sobre el eje de la
//              gondola, n = (sin s, 0, -cos s): con s=0 es colectivo como el
//              quad y con s=90 grados es longitudinal como el ala fija. Entre
//              medio, las dos cosas a la vez. Ademas es el unico que usa
//              aerodinamica de verdad (Bloque 2b: sustentacion y arrastre
//              dependientes de Va y del angulo de ataque, no un drag
//              cuadratico fijo) y el unico que pasa por el Bloque 4: sus
//              p,q,r salen de TORQUES, porque su autoridad de control cambia
//              durante la transicion y esa es justamente su fisica.
// ─────────────────────────────────────────────────────────────────────────
enum VehicleType { VEHICLE_UAV = 0, VEHICLE_QUAD = 1, VEHICLE_TILT = 2 };

struct VehicleConfig {
    VehicleType tipo;
    const char* nombre;          // nombre corto, para titulos de ventana
    const char* hudTitulo;       // linea de titulo del HUD
    const char* hudControles;    // linea de ayuda de controles
    DragCoeffs  drag;
    double      thrustMax;       // N — empuje a palanca = 1.0
    double      throttleMin;     // posicion minima de palanca
    double      throttleInit;    // palanca de arranque (punto de equilibrio)
    bool        empujeColectivo; // true = quad (empuje por -Z cuerpo)
    double      uInit;           // m/s — velocidad de arranque en el eje x cuerpo
    double      kDragU;          // para el calculo de Veq del HUD (solo ala fija)
    bool        basculante;      // true = convertiplano (empuje vectorizado + Bloques 2b/4)
};

// ─────────────────────────────────────────────────────────────────────────
// CONVERTIPLANO — parametros de transicion y de control de actitud
//
// La basculacion sigma es un estado mas del vehiculo: 0 = rotores verticales
// (vuelo estacionario), 90 grados = rotores horizontales (crucero). El piloto
// la mueve con [ y ] (o la deja a la transicion automatica con t), y todo lo
// demas —la direccion del empuje, la geometria de masa del Bloque 0 y el
// reparto de autoridad de control del Bloque 4b— es funcion de ella.
//
// Se permite bascular un poco por detras de la vertical (TILT_MIN < 0) para
// poder frenar en el aire, como hacen los convertiplanos reales.
// ─────────────────────────────────────────────────────────────────────────
static const double TILT_MIN   = -10.0 * M_PI / 180.0;   // rad — gondolas hacia atras (frenado)
static const double TILT_MAX   =  95.0 * M_PI / 180.0;   // rad — un poco pasado el modo avion
static const double TILT_RATE  =  22.0 * M_PI / 180.0;   // rad/s — velocidad de basculacion
static const double TILT_CRUISE = 90.0 * M_PI / 180.0;   // objetivo de la transicion automatica

// Tasas maximas comandadas por el teclado (el piloto pide TASA, no torque)
static const double TILT_P_CMD_MAX = 0.60;   // rad/s — alabeo
static const double TILT_Q_CMD_MAX = 0.50;   // rad/s — cabeceo
static const double TILT_R_CMD_MAX = 0.40;   // rad/s — guinada

// Constantes de tiempo del lazo de tasa (Bloque 4b): sin saturar, la tasa
// alcanza la comandada con este tau, sea cual sea el modo de vuelo
static const double TILT_TAU_P = 0.35;   // s
static const double TILT_TAU_Q = 0.40;   // s
static const double TILT_TAU_R = 0.60;   // s

// Geometria de los rotores respecto al CG + margen de mando disponible.
// Los brazos salen de tiltrotorGeom(), no de numeros sueltos.
static TiltRotorLayout tiltLayout()
{
    const TiltrotorGeom g = tiltrotorGeom();
    TiltRotorLayout lay;
    lay.armX_fwd = std::abs(g.xGondFwd);
    lay.armX_aft = std::abs(g.xGondAft);
    lay.armY_fwd = g.yGondFwd;
    lay.armY_aft = g.yGondAft;
    lay.diffFrac    = 0.35;                      // hasta +-35% de T/4 por rotor
    lay.diffTiltMax = 8.0 * M_PI / 180.0;        // basculacion diferencial
    return lay;
}

// Coeficientes aerodinamicos del convertiplano (Bloque 2b). El ala que
// sustenta es la MISMA que pesa en el Bloque 0 y la misma que se dibuja:
// S, b y c salen de tiltrotorGeom().
static AeroCoeffs tiltAeroCoeffs()
{
    const TiltrotorGeom g = tiltrotorGeom();
    return aeroCoeffsReferencia(tiltrotorAreaAlar(g),
                                g.envergadura,
                                tiltrotorCuerdaMedia(g));
}

// Construye la configuracion de vuelo a partir de la masa que devolvio el
// Bloque 0 (el empuje del quad se define como relacion empuje/peso, asi que
// depende de la masa que salio de su geometria, no de un numero suelto).
static VehicleConfig makeVehicleConfig(VehicleType tipo, double mass)
{
    if (tipo == VEHICLE_TILT) {
        // Relacion empuje/peso 2.0: puede despegar en vertical con margen, y
        // el vuelo estacionario cae en 1/2.0 = 50% de palanca. En crucero ese
        // mismo empuje apunta hacia adelante y el ala hace el resto.
        const double peso      = mass * GRAVITY;
        const double thrustMax = 2.0 * peso;

        // El arrastre del convertiplano NO se modela con aeroDrag(): sale del
        // Bloque 2b (CD(alpha) * qbar * S), que es lo unico que representa bien
        // el salto entre volar de costado en estacionario y volar de frente en
        // crucero. Estos coeficientes quedan en cero para dejarlo claro.
        return { VEHICLE_TILT, "Convertiplano",
                 "CONVERTIPLANO  —  4 rotores basculantes (transicion VTOL <-> crucero)",
                 "<- ->: alabeo  |  up dn: cabeceo  |  A D: guinada  |  w/s: potencia  |  [ ]: basculacion  "
                 "|  t: transicion auto  |  Space: pausa  |  0: reset  |  v: vehiculo  |  Q: salir",
                 { 0.0, 0.0, 0.0 },
                 thrustMax, /*throttleMin*/ 0.0, /*throttleInit*/ peso / thrustMax,
                 /*empujeColectivo*/ false, /*uInit*/ 0.0, /*kDragU*/ 0.0,
                 /*basculante*/ true };
    }

    if (tipo == VEHICLE_QUAD) {
        // Relacion empuje/peso 2.2 (tipico de un multirrotor de carga): a
        // palanca 1.0 puede acelerar hacia arriba a ~1.2 g, y el hover cae en
        // 1/2.2 = 45% de palanca, que es la posicion de arranque.
        const double peso      = mass * GRAVITY;
        const double thrustMax = 2.2 * peso;

        // Arrastre casi isotropo en el plano (el quad no tiene "frente"
        // aerodinamico) y mas alto en vertical, por el area de los rotores.
        // kx=3.6 deja el crucero en ~15 m/s con 30 grados de inclinacion.
        return { VEHICLE_QUAD, "Quadcopter",
                 "QUADCOPTER  —  Vuelo 6DOF (fisica real: masa/inercia config X)",
                 "<- ->: roll  |  up dn: pitch  |  A D: yaw  |  w/s: colectivo  |  Space: pausa  "
                 "|  0: reset  |  v: vehiculo  |  Shift+W: wire  |  Q: salir",
                 { 3.6, 3.6, 8.0 },
                 thrustMax, /*throttleMin*/ 0.0, /*throttleInit*/ peso / thrustMax,
                 /*empujeColectivo*/ true, /*uInit*/ 0.0, /*kDragU*/ 3.6,
                 /*basculante*/ false };
    }

    // Ala delta — exactamente los mismos numeros de siempre
    return { VEHICLE_UAV, "UAV ala delta",
             "UAV  —  Vuelo 6DOF (fisica real: masa/inercia ala delta)",
             "<- ->: roll  |  up dn: pitch  |  A D: yaw  |  w/s: gases  |  Space: pausa  "
             "|  0: reset  |  v: vehiculo  |  Shift+W: wire  |  Q: salir",
             g_dragCoeffs,
             FX_MAX, THROTTLE_MIN, THROTTLE_INIT,
             /*empujeColectivo*/ false, U_INIT, K_DRAG_U,
             /*basculante*/ false };
}

// Obstaculos — bloques rectangulares fijos (colision AABB, la misma
// checkObstacleCollision() de physics_UAV.h evaluada bloque por bloque),
// dimensiones fisicas reales en metros. El primero es el bloque original:
// centrado 100 m adelante del origen (pn) y sobre el eje de crucero inicial
// (pe=0), para que se pueda ir a chocar contra el descendiendo mientras se
// vuela hacia adelante. El resto forma un campo repartido por el plano del
// piso (dentro de los +-200 m de la grilla) con alturas que van de 6 a 50 m,
// asi que ya no basta con esquivar de frente: hay que elegir a que altura se
// cruza cada zona. Se deja libre un radio de ~25 m alrededor del origen para
// que el dron nunca aparezca dentro de un bloque al arrancar o reiniciar.
static const BoxObstacle g_obstacles[] = {
    //   pnCenter  peCenter   sizePn  sizePe   height   pisoPD
    {     100.0,      0.0,      2.0,    3.0,     6.0,   PISO_PD },   // bloque original
    {      60.0,    -35.0,      8.0,    8.0,    14.0,   PISO_PD },
    {      45.0,     40.0,      6.0,   10.0,    25.0,   PISO_PD },
    {     140.0,    -20.0,     10.0,    6.0,    38.0,   PISO_PD },
    {     120.0,     55.0,      7.0,    7.0,    18.0,   PISO_PD },
    {     -50.0,     30.0,      9.0,    9.0,    30.0,   PISO_PD },
    {     -80.0,    -60.0,     12.0,    8.0,    10.0,   PISO_PD },
    {      20.0,    -70.0,      6.0,    6.0,    45.0,   PISO_PD },
    {     -30.0,    -25.0,      5.0,   14.0,     8.0,   PISO_PD },
    {     170.0,     90.0,     14.0,   14.0,    22.0,   PISO_PD },
    {      80.0,    130.0,      8.0,   12.0,    33.0,   PISO_PD },
    {    -120.0,     70.0,     10.0,   10.0,    16.0,   PISO_PD },
    {    -150.0,   -110.0,     16.0,    9.0,    28.0,   PISO_PD },
    {      10.0,    100.0,      7.0,    7.0,    12.0,   PISO_PD },
    {     -95.0,    150.0,     11.0,   11.0,    40.0,   PISO_PD },
    {     150.0,   -140.0,      9.0,   15.0,    20.0,   PISO_PD },
    {    -170.0,    -10.0,      8.0,    8.0,     9.0,   PISO_PD },
    {      55.0,   -160.0,     12.0,    6.0,    35.0,   PISO_PD },
    {     -40.0,   -140.0,     10.0,   10.0,    24.0,   PISO_PD },
    {     190.0,     30.0,      6.0,   18.0,    50.0,   PISO_PD },
};
static const int N_OBSTACLES = (int)(sizeof(g_obstacles) / sizeof(g_obstacles[0]));

// Altura del bloque mas alto — solo para el degradado de color de
// drawObstacles() (bajo = ambar, alto = violeta frio), asi la altura se lee
// de un vistazo sin tener que acercarse.
static const double OBSTACLE_HEIGHT_MAX = 50.0;

// Marca por bloque: cual fue golpeado (se pinta rojo). g_obstacleHit es el
// "alguno fue golpeado", que usa el HUD y el reinicio con la tecla 0.
static bool g_obstacleHitFlags[N_OBSTACLES] = { false };
static bool g_obstacleHit = false;

static DroneMassProperties g_drone;   // masa, cg, tensor de inercia (Bloque 0)
static VehicleConfig g_cfg;           // vehiculo activo (ala fija o quad)
static bool     g_selecting = true;   // pantalla inicial de eleccion de vehiculo
static UAVState g_state;
static UAVState g_init;
static bool     g_crashed  = false;
static double   g_throttle = THROTTLE_INIT;   // posicion de palanca [throttleMin, 1.0]
static double   g_thrustN  = 0.0;             // N — empuje comandado (longitudinal o colectivo), para HUD
static bool     g_keyW = false, g_keyS = false;

// Rates iniciales tomados de la linea de comandos (se reaplican en cada
// eleccion de vehiculo y en cada reset)
static double   g_argP = 0.0, g_argQ = 0.0, g_argR = 0.0;

// ── Estado exclusivo del convertiplano ───────────────────────────────────
// La basculacion es un estado mas (no una opcion de configuracion): se mueve
// en el tiempo, cambia la geometria de masa en cada paso y con ella el
// reparto de autoridad de control.
static double g_tilt      = 0.0;     // rad — basculacion actual de las gondolas
static bool   g_keyTiltF  = false;   // ']' — bascular hacia crucero
static bool   g_keyTiltB  = false;   // '[' — bascular hacia estacionario
static bool   g_autoTilt  = false;   // 't' — transicion automatica a crucero

// Mandos del piloto: para el convertiplano el teclado pide TASAS, y el lazo
// del Bloque 4b las convierte en torques (a diferencia de los otros dos
// vehiculos, donde las flechas escriben p,q,r directamente).
static double g_pCmd = 0.0, g_qCmd = 0.0, g_rCmd = 0.0;

// Diagnostico del ultimo paso, para el HUD
static AirData         g_air  { 0.0, 0.0, 0.0, 0.0 };
static ControlAuthority g_auth { 0.0, 0.0, 0.0 };
static ControlTorques   g_ctrl { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
static const AeroCoeffs g_aero     = tiltAeroCoeffs();
static const TiltRotorLayout g_lay = tiltLayout();
static PropModel g_prop { 0.0, 0.0 };   // se arma en startVehicle (depende de la masa)
static double    g_thrustCmd = 0.0;      // N — empuje que pide la palanca (estatico)

// ═══════════════════════════════════════════════════════════════════════
// TELEMETRY — historial de estado para la ventana secundaria de graficas
// (buffer circular; step() empuja una muestra por cada paso de fisica)
// ═══════════════════════════════════════════════════════════════════════

struct TelemetrySample {
    double t;
    double pn, pe, pd;
    double u, v, w;
    double phi, theta, psi;
    double p, q, r;
};

static const int TELEMETRY_CAPACITY = 3000;   // ~48 s de historial @ 60fps
static TelemetrySample g_telemetry[TELEMETRY_CAPACITY];
static int    g_telemetryHead  = 0;   // proximo indice a escribir
static int    g_telemetryCount = 0;   // muestras validas (satura en CAPACITY)
static double g_simTime        = 0.0;

static void pushTelemetry()
{
    TelemetrySample smp;
    smp.t     = g_simTime;
    smp.pn    = g_state.pn;    smp.pe    = g_state.pe;    smp.pd  = g_state.pd;
    smp.u     = g_state.u;     smp.v     = g_state.v;     smp.w   = g_state.w;
    smp.phi   = g_state.phi;   smp.theta = g_state.theta; smp.psi = g_state.psi;
    smp.p     = g_state.p;     smp.q     = g_state.q;     smp.r   = g_state.r;

    g_telemetry[g_telemetryHead] = smp;
    g_telemetryHead = (g_telemetryHead + 1) % TELEMETRY_CAPACITY;
    if (g_telemetryCount < TELEMETRY_CAPACITY) g_telemetryCount++;
}

static void resetTelemetry()
{
    g_telemetryHead = 0; g_telemetryCount = 0; g_simTime = 0.0;
}

// Muestra i-esima en orden cronologico (0 = la mas vieja aun en el buffer)
static const TelemetrySample& telemetryAt(int i)
{
    int start = (g_telemetryHead - g_telemetryCount + TELEMETRY_CAPACITY) % TELEMETRY_CAPACITY;
    return g_telemetry[(start + i) % TELEMETRY_CAPACITY];
}

// Cierre comun de todos los vehiculos: colision contra el piso y contra los
// bloques fijos (misma restriccion dura para los tres) + una muestra de
// telemetria. Se factoriza aparte porque el convertiplano integra distinto
// que los otros dos pero choca exactamente igual.
static void finishStep(double dt)
{
    GroundContact gc = checkGroundCollision(g_state, PISO_PD);   // piso solido
    if (gc.colision) {
        g_state  = resolveGroundCollision(g_state, PISO_PD);
        g_crashed = true;
    }

    for (int i = 0; i < N_OBSTACLES; ++i) {                              // bloques fijos
        ObstacleContact oc = checkObstacleCollision(g_state, g_obstacles[i]);
        if (oc.colision) {
            g_state       = resolveObstacleCollision(g_state, g_obstacles[i]);
            g_crashed     = true;
            g_obstacleHit = true;
            g_obstacleHitFlags[i] = true;
        }
    }

    g_simTime += dt;
    pushTelemetry();
}

// ─────────────────────────────────────────────────────────────────────────
// Paso de integracion del CONVERTIPLANO — la cadena completa de los 5 bloques
//
//   Bloque 0  masa/inercia RECALCULADAS en cada paso, porque bascular mueve
//             las gondolas y con ellas el CG y el tensor de inercia
//   Bloque 2b aerodinamica: sustentacion y arrastre de un ala real, funcion
//             de Va y del angulo de ataque (en estacionario valen ~0 solas)
//   Bloque 2  empuje VECTORIZADO sobre el eje de la gondola + gravedad
//             proyectada al cuerpo + fuerzas aerodinamicas -> u,v,w
//   Bloque 1  u,v,w -> pn,pe,pd
//   Bloque 4b torque de mando (saturado por la autoridad del modo de vuelo)
//             + momentos propios de la celula
//   Bloque 4  torques -> p_dot,q_dot,r_dot  (el unico vehiculo que lo usa)
//   Bloque 3  p,q,r -> phi_dot,theta_dot,psi_dot
//
// A diferencia de los otros dos, aca la rotacion recorre dinamica ->
// cinematica igual que la traslacion: no se escriben p,q,r a mano.
// ─────────────────────────────────────────────────────────────────────────
static void stepTiltrotor(double dt)
{
    // --- Basculacion de las gondolas (la transicion propiamente dicha) ---
    if (g_autoTilt) {
        const double err = TILT_CRUISE - g_tilt;
        const double mov = TILT_RATE * dt;
        g_tilt += (std::abs(err) <= mov) ? err : ((err > 0) ? mov : -mov);
        if (std::abs(TILT_CRUISE - g_tilt) < 1e-6) g_autoTilt = false;
    } else {
        if (g_keyTiltF) g_tilt += TILT_RATE * dt;
        if (g_keyTiltB) g_tilt -= TILT_RATE * dt;
    }
    if (g_tilt > TILT_MAX) g_tilt = TILT_MAX;
    if (g_tilt < TILT_MIN) g_tilt = TILT_MIN;

    // --- Bloque 0: la geometria de masa depende de la basculacion ---
    g_drone = buildTiltrotor(g_tilt);

    // --- Palanca de potencia (empuje total de los 4 rotores) ---
    if (g_keyW) g_throttle += THROTTLE_RATE * dt;
    if (g_keyS) g_throttle -= THROTTLE_RATE * dt;
    if (g_throttle > 1.0)               g_throttle = 1.0;
    if (g_throttle < g_cfg.throttleMin) g_throttle = g_cfg.throttleMin;

    // El empuje ENTREGADO no es el que pide la palanca: la helice lo pierde a
    // medida que crece la velocidad que atraviesa el disco (tiltThrust()).
    // g_thrustCmd es lo comandado y g_thrustN lo que realmente sale.
    g_thrustCmd = g_throttle * g_cfg.thrustMax;
    g_thrustN   = tiltThrust(g_prop, g_throttle, g_state, g_tilt);

    // --- Empuje vectorizado: sobre el eje de la gondola, no sobre un eje fijo.
    // Es LA diferencia del convertiplano: con tilt=0 es el quad (todo por -Z)
    // y con tilt=90 es el ala fija (todo por +X); en el medio, las dos cosas.
    const Vec3 nThrust = tiltThrustAxis(g_tilt);
    const double fx_thrust = g_thrustN * nThrust.x;
    const double fz_thrust = g_thrustN * nThrust.z;

    // --- Gravedad proyectada al marco cuerpo segun la actitud actual ---
    const double W = g_drone.mass * GRAVITY;
    const double fg_x = -W * std::sin(g_state.theta);
    const double fg_y =  W * std::cos(g_state.theta) * std::sin(g_state.phi);
    const double fg_z =  W * std::cos(g_state.theta) * std::cos(g_state.phi);

    // --- Bloque 2b: aerodinamica del ala (sustentacion + arrastre) ---
    g_air = computeAirData(g_state, g_aero.rho);
    const Vec3 fa = aeroForces(g_air, g_aero);

    // --- Bloque 2 -> Bloque 1 (traslacion) ---
    Vec3 vel_dot = translational_dynamics(g_state,
                                          fx_thrust + fg_x + fa.x,
                                                      fg_y + fa.y,
                                          fz_thrust + fg_z + fa.z,
                                          g_drone.mass);
    Vec3 pos_dot = translational_kinematics(g_state);

    // --- Bloque 4b: cuanto torque hay disponible y cuanto pide el piloto ---
    g_auth = tiltrotorAuthority(g_thrustN, g_tilt, g_lay, g_aero, g_air.qbar);
    g_ctrl = tiltrotorRateControl(g_state, g_pCmd, g_qCmd, g_rCmd,
                                  g_drone.J, g_auth,
                                  TILT_TAU_P, TILT_TAU_Q, TILT_TAU_R);

    // Momentos propios de la celula (estabilidad estatica + amortiguamiento):
    // el piloto no los comanda, los pone el aire sobre el ala y la cola.
    const Vec3 ma = aeroMomentsAirframe(g_state, g_air, g_aero);

    // --- Bloque 4 -> Bloque 3 (rotacion) ---
    Vec3 rate_dot  = rotational_dynamics(g_state,
                                         g_ctrl.l + ma.x,
                                         g_ctrl.m + ma.y,
                                         g_ctrl.n + ma.z,
                                         g_drone.J);
    Vec3 euler_dot = rotational_kinematics(g_state);

    g_state.pn += pos_dot.x * dt;
    g_state.pe += pos_dot.y * dt;
    g_state.pd += pos_dot.z * dt;

    g_state.u  += vel_dot.x * dt;
    g_state.v  += vel_dot.y * dt;
    g_state.w  += vel_dot.z * dt;

    g_state.p  += rate_dot.x * dt;
    g_state.q  += rate_dot.y * dt;
    g_state.r  += rate_dot.z * dt;

    g_state.phi   += euler_dot.x * dt;
    g_state.theta += euler_dot.y * dt;
    g_state.psi   += euler_dot.z * dt;

    finishStep(dt);
}

// Bloque 3 (igual que antes): el usuario comanda p,q,r directamente con el
// teclado y se integran las tasas de Euler — control directo, sin pasar por
// torques/Bloque 4.
//
// Bloque 1-2 (cadena completa dinamica+cinematica): W/S mueven la palanca de
// gases g_throttle -> g_fxThrust; aeroDrag() (physics_UAV.h) le opone
// arrastre a la velocidad en los 3 ejes del marco cuerpo, dandole a cada
// posicion de palanca un punto de equilibrio real (empuje=drag) en vez de
// acelerar sin limite. translational_dynamics() convierte la fuerza neta en
// aceleracion de u,v,w (Bloque 2) y translational_kinematics() esa velocidad
// en desplazamiento inercial (Bloque 1) — a diferencia de la rotacion, aca
// si se recorre dinamica -> cinematica completa.
static void step(double dt)
{
    if (g_cfg.basculante) { stepTiltrotor(dt); return; }

    if (g_keyW) g_throttle += THROTTLE_RATE * dt;
    if (g_keyS) g_throttle -= THROTTLE_RATE * dt;
    if (g_throttle > 1.0)              g_throttle = 1.0;
    if (g_throttle < g_cfg.throttleMin) g_throttle = g_cfg.throttleMin;
    g_thrustN = g_throttle * g_cfg.thrustMax;

    // Unico punto donde los dos vehiculos difieren: sobre que eje del cuerpo
    // entra el empuje al Bloque 2. De ahi para abajo, ecuaciones identicas.
    double fx_thrust, fz_thrust;
    if (g_cfg.empujeColectivo) {
        fx_thrust =  0.0;                      // quad: no hay empuje longitudinal...
        fz_thrust = -g_thrustN;                // ...todo el empuje va por -Z cuerpo (arriba)
    } else {
        fx_thrust =  g_thrustN;                // ala fija: helice tirando por +X cuerpo
        fz_thrust = -g_drone.mass * GRAVITY;   // sustentacion del ala = peso -> altitud estable
    }
    const double fy_thrust = 0.0;

    double fg_x = -g_drone.mass * GRAVITY * std::sin(g_state.theta);
    double fg_y =  g_drone.mass * GRAVITY * std::cos(g_state.theta) * std::sin(g_state.phi);
    double fg_z =  g_drone.mass * GRAVITY * std::cos(g_state.theta) * std::cos(g_state.phi);

    Vec3 drag = aeroDrag(g_state, g_cfg.drag);     // Bloque 2 — arrastre por eje

    Vec3 pos_dot = translational_kinematics(g_state);                              // Bloque 1
    Vec3 vel_dot = translational_dynamics(g_state, fx_thrust + fg_x + drag.x,
                                           fy_thrust + fg_y + drag.y,
                                           fz_thrust + fg_z + drag.z,
                                           g_drone.mass);                           // Bloque 2
    Vec3 euler_dot = rotational_kinematics(g_state);                               // Bloque 3

    g_state.pn += pos_dot.x * dt;
    g_state.pe += pos_dot.y * dt;
    g_state.pd += pos_dot.z * dt;

    g_state.u += vel_dot.x * dt;
    g_state.v += vel_dot.y * dt;
    g_state.w += vel_dot.z * dt;

    g_state.phi   += euler_dot.x * dt;
    g_state.theta += euler_dot.y * dt;
    g_state.psi   += euler_dot.z * dt;

    // p,q,r NO se integran via Bloque 4 — son el input directo del usuario

    finishStep(dt);
}

// ═══════════════════════════════════════════════════════════════════════
// SIMULATION STATE
// ═══════════════════════════════════════════════════════════════════════

static const double DT = 0.016;   // ~60 fps integration step

static bool     g_paused = false;

// ═══════════════════════════════════════════════════════════════════════
// CAMERA
// ═══════════════════════════════════════════════════════════════════════

static float g_rotX =  20.0f;
static float g_rotY = -35.0f;
static float g_zoom = -22.0f;
static int   g_lastX, g_lastY;
static bool  g_drag = false;
static bool  g_wire = false;

// ═══════════════════════════════════════════════════════════════════════
// HUD
// ═══════════════════════════════════════════════════════════════════════

static void hudTextFont(float x, float y, const char* s, void* font)
{
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
    glOrtho(0, 900, 0, 600, -1, 1);
    glMatrixMode(GL_MODELVIEW);  glPushMatrix(); glLoadIdentity();
    glRasterPos2f(x, y);
    while (*s) glutBitmapCharacter(font, *s++);
    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW);  glPopMatrix();
}

static void hudText(float x, float y, const char* s)
{
    hudTextFont(x, y, s, GLUT_BITMAP_8_BY_13);
}

static void drawHUD()
{
    const double r2d = 180.0 / M_PI;
    char buf[128];

    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // Title
    glColor3f(0.90f, 0.90f, 0.90f);
    hudText(10, 585, g_cfg.hudTitulo);

    if (g_paused) {
        glColor3f(1.0f, 0.80f, 0.10f);
        hudText(430, 585, "[PAUSED]");
    }

    // Posicion e altitud (Bloque 1)
    glColor3f(0.55f, 0.80f, 1.0f);
    if (g_crashed) {
        hudText(10, 560, "pn=--.-  pe=--.-  alt=--.-  m   [SIN SEÑAL]");
    } else {
        snprintf(buf, sizeof(buf), "pn=%.1f  pe=%.1f  alt=%.1f  m",
                 g_state.pn, g_state.pe, -g_state.pd);
        hudText(10, 560, buf);
    }

    // Velocidad (Bloque 2)
    double airspeed = std::sqrt(g_state.u*g_state.u + g_state.v*g_state.v + g_state.w*g_state.w);
    glColor3f(0.70f, 0.90f, 0.60f);
    if (g_crashed) {
        hudText(10, 540, "u=--.-  v=--.-  w=--.-  |V|=--.-  m/s   |  palanca=---%  [SIN SEÑAL]");
    } else if (g_cfg.basculante) {
        // Convertiplano: lo que importa no es una Veq ni un T/W suelto, sino
        // cuanto del peso lo sostiene el ala y cuanto los rotores — que es lo
        // que decide si la transicion se puede completar.
        const double tw = g_thrustN / (g_drone.mass * GRAVITY);
        snprintf(buf, sizeof(buf), "u=%.2f  v=%.2f  w=%.2f  |V|=%.2f  m/s   |  potencia=%.0f%%  (%.0f de %.0f N -> T/W=%.2f)",
                 g_state.u, g_state.v, g_state.w, airspeed, g_throttle*100.0,
                 g_thrustN, g_thrustCmd, tw);
        hudText(10, 540, buf);
    } else if (g_cfg.empujeColectivo) {
        // Quad: la referencia util no es una velocidad de equilibrio sino la
        // relacion empuje/peso — abajo de 1.00 desciende, arriba sube.
        const double tw = g_thrustN / (g_drone.mass * GRAVITY);
        snprintf(buf, sizeof(buf), "u=%.2f  v=%.2f  w=%.2f  |V|=%.2f  m/s   |  colectivo=%.0f%%  (%.0f N -> T/W=%.2f)",
                 g_state.u, g_state.v, g_state.w, airspeed, g_throttle*100.0, g_thrustN, tw);
        hudText(10, 540, buf);
    } else {
        const double vEq = (g_thrustN > 0.0) ? std::sqrt(g_thrustN / g_cfg.kDragU) : 0.0;
        snprintf(buf, sizeof(buf), "u=%.2f  v=%.2f  w=%.2f  |V|=%.2f  m/s   |  palanca=%.0f%%  (%.0f N -> Veq=%.1f m/s)",
                 g_state.u, g_state.v, g_state.w, airspeed, g_throttle*100.0, g_thrustN, vEq);
        hudText(10, 540, buf);
    }

    // Euler angles (Bloque 3)
    glColor3f(0.50f, 1.0f, 0.55f);
    if (g_crashed) {
        hudText(10, 520, "phi=--.-  theta=--.-  psi=--.-  deg   [SIN SEÑAL]");
    } else {
        snprintf(buf, sizeof(buf), "phi=%.2f  theta=%.2f  psi=%.2f  deg",
                 g_state.phi*r2d, g_state.theta*r2d, g_state.psi*r2d);
        hudText(10, 520, buf);
    }

    // Angular rates — input directo del usuario (Bloque 3)
    glColor3f(1.0f, 0.75f, 0.35f);
    snprintf(buf, sizeof(buf), "p=%.3f  q=%.3f  r=%.3f  rad/s",
             g_state.p, g_state.q, g_state.r);
    hudText(10, 500, buf);

    // Masa / inercia (Bloque 0)
    glColor3f(0.65f, 0.65f, 0.70f);
    snprintf(buf, sizeof(buf), "%s:  masa=%.0f kg   Ixx=%.0f  Iyy=%.0f  Izz=%.0f  kg*m^2",
             g_cfg.nombre, g_drone.mass, g_drone.I_full.Ixx, g_drone.I_full.Iyy, g_drone.I_full.Izz);
    hudText(10, 480, buf);

    // ── Bloque exclusivo del convertiplano ──────────────────────────────
    // Basculacion, estado aerodinamico y reparto de sustentacion: las tres
    // cosas que hay que mirar durante una transicion.
    float yTilt = 460.0f;
    if (g_cfg.basculante && !g_crashed) {
        const double r2d_ = 180.0 / M_PI;
        const double tiltDeg = g_tilt * r2d_;

        // Sustentacion vertical que da el ala (componente -Z cuerpo de la
        // fuerza aerodinamica) contra el peso: 0% en estacionario, ~100% en
        // crucero. Es el indicador de "ya puedo soltar los rotores".
        const Vec3 fa = aeroForces(g_air, g_aero);
        const double sustFrac = (-fa.z) / (g_drone.mass * GRAVITY);

        glColor3f(0.60f, 1.00f, 0.55f);
        snprintf(buf, sizeof(buf),
                 "basculacion=%.1f deg  (%s)%s   |   Va=%.1f m/s  alfa=%.1f deg  beta=%.1f deg  qbar=%.0f Pa",
                 tiltDeg,
                 (tiltDeg < 15.0) ? "sustentado por rotores"
                                  : ((tiltDeg > 75.0) ? "modo avion" : "TRANSICION"),
                 g_autoTilt ? "  [AUTO]" : "",
                 g_air.Va, g_air.alpha*r2d_, g_air.beta*r2d_, g_air.qbar);
        hudText(10, yTilt, buf);
        yTilt -= 20.0f;

        glColor3f(0.85f, 0.80f, 0.55f);
        snprintf(buf, sizeof(buf),
                 "ala sostiene %.0f%% del peso   |   autoridad l=%.0f m=%.0f n=%.0f N*m   uso %.0f%%/%.0f%%/%.0f%%",
                 sustFrac*100.0, g_auth.l, g_auth.m, g_auth.n,
                 g_ctrl.satL*100.0, g_ctrl.satM*100.0, g_ctrl.satN*100.0);
        hudText(10, yTilt, buf);
        yTilt -= 20.0f;

        glColor3f(0.60f, 0.62f, 0.68f);
        snprintf(buf, sizeof(buf),
                 "tasa comandada  p*=%.2f  q*=%.2f  r*=%.2f rad/s  (Bloque 4b -> torques -> Bloque 4)",
                 g_pCmd, g_qCmd, g_rCmd);
        hudText(10, yTilt, buf);
        yTilt -= 20.0f;

        // Aviso de transicion peligrosa: gondolas casi horizontales sin
        // velocidad suficiente = el ala no sustenta y los rotores ya no
        // apuntan hacia arriba.
        if (tiltDeg > 55.0 && sustFrac < 0.55 && -g_state.pd > 1.0) {
            glColor3f(1.0f, 0.55f, 0.15f);
            hudText(10, yTilt, "CUIDADO: basculado sin velocidad — el ala no sostiene el peso ([ para volver a rotores)");
            yTilt -= 20.0f;
        }
    }

    // Colision con el piso solido o con el obstaculo (bloque fijo)
    if (g_crashed) {
        glColor3f(1.0f, 0.25f, 0.20f);
        hudText(10, 460, "SEÑAL PERDIDA — INTERFERENCIA. Presiona 0 para reiniciar.");
    }

    // Singularity warning
    if (std::abs(std::cos(g_state.theta)) < 0.1) {
        glColor3f(1.0f, 0.2f, 0.2f);
        hudText(10, yTilt, "WARNING: Gimbal Lock — theta near ±90°");
    }

    // Axis legend
    glColor3f(1.0f, 0.35f, 0.35f); hudText(10,  30, "X_b");
    glColor3f(0.35f, 1.0f, 0.35f); hudText(45,  30, "Y_b");
    glColor3f(0.45f, 0.60f, 1.0f); hudText(80,  30, "Z_b (fwd)");

    // Controls
    glColor3f(0.50f, 0.50f, 0.50f);
    hudText(10, 10, g_cfg.hudControles);

    glEnable(GL_LIGHTING);
}

// ═══════════════════════════════════════════════════════════════════════
// REFERENCE GEOMETRY
// ═══════════════════════════════════════════════════════════════════════

// Fixed world axes (do not rotate with drone)
static void drawWorldAxes()
{
    glDisable(GL_LIGHTING);
    glLineWidth(1.2f);
    glBegin(GL_LINES);
        glColor3f(0.35f,0.10f,0.10f); glVertex3f(0,0,0); glVertex3f(8,0,0);
        glColor3f(0.10f,0.35f,0.10f); glVertex3f(0,0,0); glVertex3f(0,8,0);
        glColor3f(0.10f,0.10f,0.35f); glVertex3f(0,0,0); glVertex3f(0,0,-8);
    glEnd();
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

// Body-frame axes (rotate with drone)
static void drawBodyAxes()
{
    glDisable(GL_LIGHTING);
    glLineWidth(2.5f);
    glBegin(GL_LINES);
        glColor3f(1.0f, 0.25f, 0.25f); glVertex3f(0,0,0); glVertex3f(7, 0, 0);
        glColor3f(0.25f, 1.0f, 0.25f); glVertex3f(0,0,0); glVertex3f(0,-7, 0);
        glColor3f(0.35f, 0.55f, 1.0f); glVertex3f(0,0,0); glVertex3f(0, 0,-7);
    glEnd();
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

// Horizontal reference circle — helps see the roll visually
static void drawHorizonRing()
{
    glDisable(GL_LIGHTING);
    glColor3f(0.30f, 0.30f, 0.40f);
    glLineWidth(1.0f);
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 64; i++) {
        float a = (float)i * 2.0f * (float)M_PI / 64.0f;
        glVertex3f(12.0f*std::cos(a), 0.0f, 12.0f*std::sin(a));
    }
    glEnd();
    glEnable(GL_LIGHTING);
}

// Piso solido en el plano X-Y (pd=PISO_PD, local y=0 en GL) — no es solo
// referencia visual: es la misma restriccion que checkGroundCollision()
// evalua en step(). Se pinta rojo cuando hay colision.
static void drawGroundGrid()
{
    const float extent = 200.0f, step_ = 10.0f;

    glDisable(GL_LIGHTING);

    // Relleno solido semitransparente del piso
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    if (g_crashed) glColor4f(0.75f, 0.15f, 0.12f, 0.55f);
    else           glColor4f(0.16f, 0.22f, 0.30f, 0.35f);
    glBegin(GL_QUADS);
        glVertex3f(-extent, 0.0f, -extent);
        glVertex3f( extent, 0.0f, -extent);
        glVertex3f( extent, 0.0f,  extent);
        glVertex3f(-extent, 0.0f,  extent);
    glEnd();
    glDisable(GL_BLEND);

    // Grilla encima, para dar referencia de escala/desplazamiento
    if (g_crashed) glColor3f(0.95f, 0.30f, 0.25f);
    else           glColor3f(0.30f, 0.30f, 0.40f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for (float v = -extent; v <= extent; v += step_) {
        glVertex3f(v, 0.0f, -extent); glVertex3f(v, 0.0f, extent);
        glVertex3f(-extent, 0.0f, v); glVertex3f(extent, 0.0f, v);
    }
    glEnd();

    glEnable(GL_LIGHTING);
}

// Obstaculos fijos — los mismos bloques que evalua checkObstacleCollision()
// en step(). Coordenadas locales: x=pe, z=-pn (NED pn -> GL -Z, igual que el
// traslado de mundo en display() y drawBodyAxes(); la nariz del dron mira
// hacia -Z, asi que un obstaculo con pn>0 queda por delante, no detras),
// y=altura sobre el piso (pd=PISO_PD -> local y=0, techo del bloque -> local y=height).
// El color va de ambar (bloque bajo) a violeta frio (bloque alto) para leer
// la altura a distancia, y las caras llevan un factor de sombra distinto para
// que se distinga el volumen. Se pinta rojo el bloque contra el que se choco.
static void drawBox(const BoxObstacle& box, bool hit)
{
    const float halfPn = (float)(box.sizePn / 2.0);
    const float halfPe = (float)(box.sizePe / 2.0);
    const float x0 = (float)box.peCenter - halfPe, x1 = (float)box.peCenter + halfPe;
    const float z0 = -(float)box.pnCenter - halfPn, z1 = -(float)box.pnCenter + halfPn;
    const float y0 = 0.0f, y1 = (float)box.height;

    // Degradado por altura: t=0 bloque bajo, t=1 bloque tan alto como el mayor
    float t = (float)(box.height / OBSTACLE_HEIGHT_MAX);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    float r, g, b;
    if (hit) { r = 0.95f; g = 0.30f; b = 0.25f; }
    else     { r = 0.85f - 0.35f*t;  g = 0.60f - 0.15f*t;  b = 0.15f + 0.45f*t; }

    // Sombra por cara (frente, fondo, izquierda, derecha, techo) — solo para
    // dar relieve; el bloque sigue siendo una AABB perfecta en la fisica.
    const float sh[5] = { 0.85f, 0.60f, 0.72f, 1.00f, 1.12f };

    glDisable(GL_LIGHTING);

    glBegin(GL_QUADS);
        glColor3f(r*sh[0], g*sh[0], b*sh[0]);   // cara -pn
        glVertex3f(x0,y0,z0); glVertex3f(x1,y0,z0); glVertex3f(x1,y1,z0); glVertex3f(x0,y1,z0);
        glColor3f(r*sh[1], g*sh[1], b*sh[1]);   // cara +pn
        glVertex3f(x1,y0,z1); glVertex3f(x0,y0,z1); glVertex3f(x0,y1,z1); glVertex3f(x1,y1,z1);
        glColor3f(r*sh[2], g*sh[2], b*sh[2]);   // cara -pe
        glVertex3f(x0,y0,z1); glVertex3f(x0,y0,z0); glVertex3f(x0,y1,z0); glVertex3f(x0,y1,z1);
        glColor3f(r*sh[3], g*sh[3], b*sh[3]);   // cara +pe
        glVertex3f(x1,y0,z0); glVertex3f(x1,y0,z1); glVertex3f(x1,y1,z1); glVertex3f(x1,y1,z0);
        glColor3f(r*sh[4], g*sh[4], b*sh[4]);   // techo
        glVertex3f(x0,y1,z0); glVertex3f(x1,y1,z0); glVertex3f(x1,y1,z1); glVertex3f(x0,y1,z1);
    glEnd();

    // Contorno para que se distinga el volumen desde lejos
    glColor3f(0.15f, 0.15f, 0.15f);
    glLineWidth(1.5f);
    glBegin(GL_LINE_LOOP);
        glVertex3f(x0,y0,z0); glVertex3f(x1,y0,z0); glVertex3f(x1,y1,z0); glVertex3f(x0,y1,z0);
    glEnd();
    glBegin(GL_LINE_LOOP);
        glVertex3f(x0,y0,z1); glVertex3f(x1,y0,z1); glVertex3f(x1,y1,z1); glVertex3f(x0,y1,z1);
    glEnd();
    glBegin(GL_LINES);
        glVertex3f(x0,y0,z0); glVertex3f(x0,y0,z1);
        glVertex3f(x1,y0,z0); glVertex3f(x1,y0,z1);
        glVertex3f(x0,y1,z0); glVertex3f(x0,y1,z1);
        glVertex3f(x1,y1,z0); glVertex3f(x1,y1,z1);
    glEnd();
    glLineWidth(1.0f);

    glEnable(GL_LIGHTING);
}

static void drawObstacles()
{
    for (int i = 0; i < N_OBSTACLES; ++i)
        drawBox(g_obstacles[i], g_obstacleHitFlags[i]);
}

// ═══════════════════════════════════════════════════════════════════════
// DRONE GEOMETRY  (nose at -Z, right wing at +X, up at +Y)
// ═══════════════════════════════════════════════════════════════════════

static void drawFuselage(GLUquadric* q)
{
    glColor3f(0.72f, 0.75f, 0.78f);
    glPushMatrix(); glTranslatef(0,0,-4.5f); glRotatef(180,0,1,0);
    gluDisk(q,0,0.05f,16,1); glPopMatrix();

    glPushMatrix(); glTranslatef(0,0,-4.5f);
    gluCylinder(q,0.05f,0.50f,1.5f,20,4); glPopMatrix();

    glPushMatrix(); glTranslatef(0,0,-3.0f);
    gluCylinder(q,0.50f,0.50f,5.0f,20,4); glPopMatrix();

    glPushMatrix(); glTranslatef(0,0, 2.0f);
    gluCylinder(q,0.50f,0.28f,2.0f,20,4); glPopMatrix();

    glColor3f(0.60f, 0.63f, 0.66f);
    glPushMatrix(); glTranslatef(0,0,4.0f);
    gluCylinder(q,0.30f,0.30f,0.8f,20,4); glPopMatrix();

    glPushMatrix(); glTranslatef(0,0,4.8f);
    gluDisk(q,0,0.30f,20,1); glPopMatrix();
}

static void drawWing(float side)
{
    float rx=side*0.50f, rzle=-0.8f, rzte=1.8f, ry=0.05f;
    float tx=side*10.0f, tzle= 0.5f, tzte=2.2f, ty=0.90f;

    glColor3f(0.72f, 0.75f, 0.78f);
    glNormal3f(0,1,0);
    glBegin(GL_QUADS);
        glVertex3f(rx,ry,     rzle); glVertex3f(rx,ry,     rzte);
        glVertex3f(tx,ty,     tzte); glVertex3f(tx,ty,     tzle);
    glEnd();
    glColor3f(0.62f, 0.65f, 0.68f);
    glNormal3f(0,-1,0);
    glBegin(GL_QUADS);
        glVertex3f(rx,ry-0.07f,rzle); glVertex3f(tx,ty-0.07f,tzle);
        glVertex3f(tx,ty-0.07f,tzte); glVertex3f(rx,ry-0.07f,rzte);
    glEnd();
    glColor3f(0.68f, 0.71f, 0.73f);
    glNormal3f(0,0,-1);
    glBegin(GL_QUADS);
        glVertex3f(rx,ry,      rzle); glVertex3f(tx,ty,      tzle);
        glVertex3f(tx,ty-0.07f,tzle); glVertex3f(rx,ry-0.07f,rzle);
    glEnd();
    glNormal3f(0,0,1);
    glBegin(GL_QUADS);
        glVertex3f(rx,ry,      rzte); glVertex3f(rx,ry-0.07f,rzte);
        glVertex3f(tx,ty-0.07f,tzte); glVertex3f(tx,ty,      tzte);
    glEnd();
    glNormal3f(side,0,0);
    glBegin(GL_QUADS);
        glVertex3f(tx,ty,      tzle); glVertex3f(tx,ty,      tzte);
        glVertex3f(tx,ty-0.07f,tzte); glVertex3f(tx,ty-0.07f,tzle);
    glEnd();
}

static void drawTailFin(float side)
{
    const float ang = 52.0f * (float)M_PI / 180.0f;
    const float span=3.2f, cr=1.8f, ct_val=0.85f, z0=3.8f;
    float tx=side*span*std::cos(ang), ty=span*std::sin(ang);   // ty>0: alerones hacia arriba
    float ax=0,ay=0,az=z0,  bx=0,by=0,bz=z0+cr;
    float cx=tx,cy=ty,cz=z0+0.5f, dx=tx,dy=ty,dz=z0+0.5f+ct_val;
    float nx=side*std::sin(ang), ny=-std::cos(ang);
    glColor3f(0.70f, 0.73f, 0.75f);
    glNormal3f(-nx,ny,0);
    glBegin(GL_QUADS);
        glVertex3f(ax,ay,az); glVertex3f(bx,by,bz);
        glVertex3f(dx,dy,dz); glVertex3f(cx,cy,cz);
    glEnd();
    glNormal3f(nx,-ny,0);
    glBegin(GL_QUADS);
        glVertex3f(ax,ay,az); glVertex3f(cx,cy,cz);
        glVertex3f(dx,dy,dz); glVertex3f(bx,by,bz);
    glEnd();
}

static void drawSensorBall()
{
    glColor3f(0.12f,0.12f,0.12f);
    glPushMatrix(); glTranslatef(0,-0.65f,-3.6f);
    glutSolidSphere(0.38,20,20); glPopMatrix();
}

static void drawSatcomDome()
{
    glColor3f(0.88f,0.88f,0.88f);
    glPushMatrix(); glTranslatef(0,0.60f,-0.8f);
    glScalef(0.48f,0.32f,0.48f);
    glutSolidSphere(1.0,20,20); glPopMatrix();
}

static void drawPropeller()
{
    glDisable(GL_LIGHTING);
    glColor3f(0.15f,0.15f,0.15f);
    glPushMatrix(); glTranslatef(0,0,4.85f);
    glutSolidSphere(0.12,12,8); glPopMatrix();
    for (int i = 0; i < 2; i++) {
        glPushMatrix(); glTranslatef(0,0,4.85f);
        glRotatef(i*90.0f,0,0,1);
        glBegin(GL_QUADS);
            glVertex3f(-0.07f,-1.9f,0.05f); glVertex3f(0.07f,-1.9f,0.05f);
            glVertex3f(0.15f,  1.9f,0.05f); glVertex3f(-0.15f,1.9f,0.05f);
        glEnd();
        glPopMatrix();
    }
    glEnable(GL_LIGHTING);
}

static void drawDeltaWing()
{
    GLUquadric* q = gluNewQuadric();
    gluQuadricNormals(q, GLU_SMOOTH);
    drawFuselage(q);
    drawWing(+1.0f); drawWing(-1.0f);
    drawTailFin(+1.0f); drawTailFin(-1.0f);
    drawSensorBall(); drawSatcomDome();
    drawPropeller();
    gluDeleteQuadric(q);
}

// ═══════════════════════════════════════════════════════════════════════
// QUADCOPTER GEOMETRY  (config X — misma convencion de ejes que el ala
// delta: frente hacia -Z, derecha hacia +X, arriba hacia +Y)
//
// Igual que el ala delta, el modelo se dibuja mas grande que su tamano
// fisico real (el quad mide 3.2 m de punta a punta pero se dibuja de ~16
// unidades) — si no, al lado de una grilla de 400 m seria un punto. Las
// proporciones entre brazos, cuerpo y rotores si son las de buildQuadcopter().
// ═══════════════════════════════════════════════════════════════════════

static const float QUAD_ARM   = 8.0f;    // largo visual del brazo, del centro a la punta
static const float QUAD_ROTOR = 3.0f;    // radio visual del rotor

// Angulo de cada brazo medido desde el eje x del cuerpo (config X), en el
// mismo orden que buildQuadcopter(): frente-der, atras-der, atras-izq, frente-izq
static const float QUAD_ANG[4] = { 45.0f, 135.0f, 225.0f, 315.0f };

// Direccion GL del brazo i (el frente del cuerpo es -Z, la derecha +X)
static void quadArmDir(int i, float& gx, float& gz)
{
    const float a = QUAD_ANG[i] * (float)M_PI / 180.0f;
    gx =  std::sin(a);        // componente y del cuerpo -> +X en GL
    gz = -std::cos(a);        // componente x del cuerpo -> -Z en GL
}

// Cuerpo central: caja achaflanada, con una proa marcada al frente para que
// la orientacion se lea de un vistazo desde cualquier angulo de camara
static void drawQuadBody()
{
    const float hx = 2.1f, hz = 2.1f, yb = -0.75f, yt = 0.85f, inset = 0.6f;

    glColor3f(0.28f, 0.30f, 0.34f);
    glBegin(GL_QUADS);
        // laterales (de la base ancha a la tapa mas angosta)
        glNormal3f(0,0,-1);
        glVertex3f(-hx,yb,-hz); glVertex3f(hx,yb,-hz);
        glVertex3f(hx-inset,yt,-hz+inset); glVertex3f(-hx+inset,yt,-hz+inset);
        glNormal3f(0,0,1);
        glVertex3f(hx,yb,hz); glVertex3f(-hx,yb,hz);
        glVertex3f(-hx+inset,yt,hz-inset); glVertex3f(hx-inset,yt,hz-inset);
        glNormal3f(-1,0,0);
        glVertex3f(-hx,yb,hz); glVertex3f(-hx,yb,-hz);
        glVertex3f(-hx+inset,yt,-hz+inset); glVertex3f(-hx+inset,yt,hz-inset);
        glNormal3f(1,0,0);
        glVertex3f(hx,yb,-hz); glVertex3f(hx,yb,hz);
        glVertex3f(hx-inset,yt,hz-inset); glVertex3f(hx-inset,yt,-hz+inset);
    glEnd();

    // Tapa superior
    glColor3f(0.38f, 0.41f, 0.46f);
    glBegin(GL_QUADS);
        glNormal3f(0,1,0);
        glVertex3f(-hx+inset,yt,-hz+inset); glVertex3f(hx-inset,yt,-hz+inset);
        glVertex3f(hx-inset,yt,hz-inset);   glVertex3f(-hx+inset,yt,hz-inset);
    glEnd();

    // Panza (donde va la carga / el sensor)
    glColor3f(0.20f, 0.21f, 0.24f);
    glBegin(GL_QUADS);
        glNormal3f(0,-1,0);
        glVertex3f(-hx,yb,hz); glVertex3f(hx,yb,hz);
        glVertex3f(hx,yb,-hz); glVertex3f(-hx,yb,-hz);
    glEnd();

    // Proa: cuna hacia -Z (el frente), en naranja
    glColor3f(0.90f, 0.45f, 0.12f);
    glBegin(GL_TRIANGLES);
        glNormal3f(0,0,-1);
        glVertex3f(-1.1f,yb+0.2f,-hz); glVertex3f(1.1f,yb+0.2f,-hz); glVertex3f(0,0.1f,-hz-1.5f);
        glNormal3f(0,1,0);
        glVertex3f(-1.1f,yt-0.1f,-hz+inset); glVertex3f(0,0.1f,-hz-1.5f); glVertex3f(1.1f,yt-0.1f,-hz+inset);
    glEnd();
}

// Brazo i: tubo del centro a la punta + pod del motor
static void drawQuadArm(GLUquadric* q, int i)
{
    float gx, gz; quadArmDir(i, gx, gz);
    const bool delantero = (QUAD_ANG[i] < 90.0f || QUAD_ANG[i] > 270.0f);

    // Orientar el cilindro (que gluCylinder dibuja sobre +Z) hacia el brazo
    const float yaw = std::atan2(gx, gz) * 180.0f / (float)M_PI;

    glColor3f(0.34f, 0.36f, 0.40f);
    glPushMatrix();
        glRotatef(yaw, 0, 1, 0);
        gluCylinder(q, 0.42f, 0.34f, QUAD_ARM, 12, 1);
    glPopMatrix();

    // Pod del motor en la punta: cilindro vertical (+Y)
    const float px = gx * QUAD_ARM, pz = gz * QUAD_ARM;
    glColor3f(delantero ? 0.85f : 0.30f, delantero ? 0.30f : 0.32f, delantero ? 0.22f : 0.36f);
    glPushMatrix();
        glTranslatef(px, 0.0f, pz);
        glRotatef(-90, 1, 0, 0);
        gluCylinder(q, 0.62f, 0.55f, 1.35f, 14, 1);
    glPopMatrix();
    glColor3f(0.18f, 0.19f, 0.22f);
    glPushMatrix();
        glTranslatef(px, 1.35f, pz);
        glRotatef(-90, 1, 0, 0);
        gluDisk(q, 0.0f, 0.55f, 14, 1);
    glPopMatrix();
}

// Rotor i: dos palas girando + el disco barrido difuminado. El sentido de
// giro alterna entre rotores contiguos, como en un quad real (dos CW y dos
// CCW para que los pares de reaccion se cancelen), y la velocidad sigue a la
// palanca de colectivo.
static void drawQuadRotor(int i)
{
    float gx, gz; quadArmDir(i, gx, gz);
    const float px = gx * QUAD_ARM, py = 1.45f, pz = gz * QUAD_ARM;

    const float sentido = (i % 2 == 0) ? 1.0f : -1.0f;
    const float rpm     = 220.0f + 900.0f * (float)g_throttle;
    const float ang     = sentido * (float)g_simTime * rpm;

    glDisable(GL_LIGHTING);

    // Disco barrido (semitransparente) — da la sensacion de rotor girando
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.65f, 0.72f, 0.85f, g_crashed ? 0.06f : 0.16f);
    glBegin(GL_TRIANGLE_FAN);
        glVertex3f(px, py, pz);
        for (int k = 0; k <= 24; ++k) {
            const float a = (float)k * 2.0f * (float)M_PI / 24.0f;
            glVertex3f(px + QUAD_ROTOR*std::cos(a), py, pz + QUAD_ROTOR*std::sin(a));
        }
    glEnd();
    glDisable(GL_BLEND);

    // Palas
    glColor3f(0.12f, 0.12f, 0.14f);
    glPushMatrix();
        glTranslatef(px, py, pz);
        glRotatef(ang, 0, 1, 0);
        for (int b = 0; b < 2; ++b) {
            glPushMatrix();
            glRotatef(b * 180.0f, 0, 1, 0);
            glBegin(GL_QUADS);
                glVertex3f(-0.10f, 0.0f, 0.0f);       glVertex3f( 0.10f, 0.0f, 0.0f);
                glVertex3f( 0.22f, 0.0f, QUAD_ROTOR); glVertex3f(-0.22f, 0.0f, QUAD_ROTOR);
            glEnd();
            glPopMatrix();
        }
    glPopMatrix();

    glEnable(GL_LIGHTING);
}

// Patines de aterrizaje (el "tren" que pesa 6 kg en buildQuadcopter)
static void drawQuadSkids()
{
    const float y0 = -0.8f, y1 = -2.6f, sx = 1.7f, sz = 2.4f;

    glDisable(GL_LIGHTING);
    glColor3f(0.22f, 0.23f, 0.26f);
    glLineWidth(4.0f);
    glBegin(GL_LINES);
        for (int sgn = -1; sgn <= 1; sgn += 2) {
            // dos patas por lado, del cuerpo al patin
            glVertex3f(sgn*0.9f, y0, -sz*0.6f); glVertex3f(sgn*sx, y1, -sz*0.7f);
            glVertex3f(sgn*0.9f, y0,  sz*0.6f); glVertex3f(sgn*sx, y1,  sz*0.7f);
            // patin longitudinal
            glVertex3f(sgn*sx, y1, -sz); glVertex3f(sgn*sx, y1, sz);
        }
    glEnd();
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

static void drawQuadcopter()
{
    GLUquadric* q = gluNewQuadric();
    gluQuadricNormals(q, GLU_SMOOTH);

    drawQuadBody();
    for (int i = 0; i < 4; ++i) drawQuadArm(q, i);
    drawQuadSkids();
    // los rotores van al final: son semitransparentes
    for (int i = 0; i < 4; ++i) drawQuadRotor(i);

    gluDeleteQuadric(q);
}

// ═══════════════════════════════════════════════════════════════════════
// CONVERTIPLANO — GEOMETRIA 3D (4 gondolas basculantes)
//
// A diferencia de los otros dos vehiculos, que estan dibujados con numeros
// sueltos en unidades de pantalla, este se dibuja A ESCALA: cada vertice se
// escribe en METROS del marco cuerpo (los MISMOS que usa tiltrotorGeom() para
// el Bloque 0) y una unica constante, TILT_VIS, los pasa a unidades GL. Asi
// las proporciones en pantalla son las proporciones fisicas, y cambiar una
// dimension en tiltrotorGeom() mueve a la vez la masa, la inercia, el area
// alar y el dibujo.
//
// TILT_VIS = 5 u/m es la misma escala del quadcopter (1.60 m de brazo ->
// 8 unidades), asi que la diferencia de tamano en pantalla entre los dos es
// la diferencia de tamano real.
//
// Conversion de ejes:  cuerpo (x adelante, y derecha, z abajo)
//                  ->  GL     (+X derecha, +Y arriba, -Z adelante)
//                      GL = ( y, -z, -x ) * TILT_VIS
// ═══════════════════════════════════════════════════════════════════════

static const float TILT_VIS = 5.0f;   // unidades GL por metro real

static inline void vBody(double x, double y, double z)
{
    glVertex3f((float)(y * TILT_VIS), (float)(-z * TILT_VIS), (float)(-x * TILT_VIS));
}
static inline void nBody(double x, double y, double z)
{
    glNormal3f((float)y, (float)(-z), (float)(-x));
}

// Placa sustentadora generica: cuatro esquinas en el marco cuerpo (borde de
// ataque y de fuga, raiz y punta) extruidas a un espesor. Sirve para el ala,
// la deriva y el estabilizador — las tres son placas trapezoidales en flecha,
// que es exactamente como las modela el Bloque 0.
struct PlateCorner { double x, y, z; };

static void drawPlate(const PlateCorner& rLE, const PlateCorner& rTE,
                      const PlateCorner& tLE, const PlateCorner& tTE,
                      double th, double nx, double ny, double nz)
{
    // th se aplica sobre la normal de la placa (espesor simetrico)
    const double hx = nx*th*0.5, hy = ny*th*0.5, hz = nz*th*0.5;

    glBegin(GL_QUADS);
        // cara "superior" (hacia la normal)
        nBody(nx, ny, nz);
        vBody(rLE.x+hx, rLE.y+hy, rLE.z+hz); vBody(tLE.x+hx, tLE.y+hy, tLE.z+hz);
        vBody(tTE.x+hx, tTE.y+hy, tTE.z+hz); vBody(rTE.x+hx, rTE.y+hy, rTE.z+hz);
        // cara opuesta
        nBody(-nx, -ny, -nz);
        vBody(rLE.x-hx, rLE.y-hy, rLE.z-hz); vBody(rTE.x-hx, rTE.y-hy, rTE.z-hz);
        vBody(tTE.x-hx, tTE.y-hy, tTE.z-hz); vBody(tLE.x-hx, tLE.y-hy, tLE.z-hz);
        // borde de ataque
        nBody(1, 0, 0);
        vBody(rLE.x+hx, rLE.y+hy, rLE.z+hz); vBody(rLE.x-hx, rLE.y-hy, rLE.z-hz);
        vBody(tLE.x-hx, tLE.y-hy, tLE.z-hz); vBody(tLE.x+hx, tLE.y+hy, tLE.z+hz);
        // borde de fuga
        nBody(-1, 0, 0);
        vBody(rTE.x+hx, rTE.y+hy, rTE.z+hz); vBody(tTE.x+hx, tTE.y+hy, tTE.z+hz);
        vBody(tTE.x-hx, tTE.y-hy, tTE.z-hz); vBody(rTE.x-hx, rTE.y-hy, rTE.z-hz);
        // punta
        nBody(0, (tLE.y >= 0 ? 1 : -1), 0);
        vBody(tLE.x+hx, tLE.y+hy, tLE.z+hz); vBody(tTE.x+hx, tTE.y+hy, tTE.z+hz);
        vBody(tTE.x-hx, tTE.y-hy, tTE.z-hz); vBody(tLE.x-hx, tLE.y-hy, tLE.z-hz);
    glEnd();
}

// Fuselaje: cono de nariz + barril central + cono de cola, sobre el eje x del
// cuerpo (que en GL es -Z). gluCylinder dibuja sobre +Z, asi que se arranca en
// la nariz y se avanza hacia la cola.
static void drawTiltFuselage(GLUquadric* q, const TiltrotorGeom& g)
{
    const float V  = TILT_VIS;
    const float R  = (float)g.fusRadio * V;
    const float xN = (float)( g.fusLargo * 0.5) * V;   // nariz  (GL z = -xN)
    const float lNose = 0.80f * V, lTail = 0.95f * V;
    const float lMid  = (float)g.fusLargo * V - lNose - lTail;

    glColor3f(0.70f, 0.73f, 0.77f);
    glPushMatrix(); glTranslatef(0, 0, -xN);
        gluCylinder(q, 0.04f*V, R, lNose, 24, 3);
        glTranslatef(0, 0, lNose);
        gluCylinder(q, R, R, lMid, 24, 2);
        glTranslatef(0, 0, lMid);
        glColor3f(0.62f, 0.65f, 0.69f);
        gluCylinder(q, R, 0.09f*V, lTail, 24, 3);
        glTranslatef(0, 0, lTail);
        gluDisk(q, 0, 0.09f*V, 16, 1);
    glPopMatrix();

    // Cupula de sensores arriba de la nariz (cuerpo: z negativa = arriba)
    glColor3f(0.86f, 0.87f, 0.89f);
    glPushMatrix();
        glTranslatef(0, (float)(0.20*V), -(float)(1.05*V));
        glScalef(0.30f*V, 0.18f*V, 0.55f*V);
        glutSolidSphere(1.0, 20, 16);
    glPopMatrix();

    // Bola giroestabilizada bajo la nariz (la carga util)
    glColor3f(0.11f, 0.11f, 0.13f);
    glPushMatrix();
        glTranslatef(0, -(float)(0.24*V), -(float)(1.25*V));
        glutSolidSphere(0.20*V, 18, 14);
    glPopMatrix();

    // Marca de proa naranja, para leer la orientacion de un vistazo
    glDisable(GL_LIGHTING);
    glColor3f(0.92f, 0.46f, 0.12f);
    glBegin(GL_TRIANGLES);
        vBody( g.fusLargo*0.5 - 0.05,  0.0,  -g.fusRadio*0.9);
        vBody( g.fusLargo*0.5 - 0.55,  0.16, -g.fusRadio*0.5);
        vBody( g.fusLargo*0.5 - 0.55, -0.16, -g.fusRadio*0.5);
    glEnd();
    glEnable(GL_LIGHTING);
}

// Ala trapezoidal en flecha, un panel por lado. Las cuatro esquinas salen de
// tiltrotorGeom(): cuerda de raiz, cuerda de punta, barrido y diedro.
static void drawTiltWing(const TiltrotorGeom& g, double side)
{
    const double yR = 0.20 * side, yT = (g.envergadura * 0.5) * side;
    const double xR = g.xAla + 0.25 * g.cuerdaRaiz;            // borde de ataque en la raiz
    const double xT = xR - g.barrido;                          // borde de ataque en la punta
    const double zT = -g.diedro;                               // punta mas alta (z negativa)

    glColor3f(0.70f, 0.73f, 0.77f);
    drawPlate({xR, yR, 0.0}, {xR - g.cuerdaRaiz, yR, 0.0},
              {xT, yT, zT},  {xT - g.cuerdaPunta, yT, zT},
              0.06, 0.0, 0.0, -1.0);

    // Aleron: banda en el borde de fuga del tramo exterior. Es la superficie
    // cuya autoridad entra al Bloque 4b (Cl_delta_a) cuando hay presion
    // dinamica, asi que conviene verla.
    const double y1 = 0.55 * yT, y2 = 0.96 * yT;
    const double f1 = (std::abs(y1) - std::abs(yR)) / (std::abs(yT) - std::abs(yR));
    const double f2 = (std::abs(y2) - std::abs(yR)) / (std::abs(yT) - std::abs(yR));
    auto leAt = [&](double f) { return xR + (xT - xR) * f; };
    auto crAt = [&](double f) { return g.cuerdaRaiz + (g.cuerdaPunta - g.cuerdaRaiz) * f; };
    auto zAt  = [&](double f) { return zT * f; };

    glColor3f(0.86f, 0.55f, 0.16f);
    drawPlate({leAt(f1) - crAt(f1)*0.78, y1, zAt(f1) - 0.04},
              {leAt(f1) - crAt(f1),      y1, zAt(f1) - 0.04},
              {leAt(f2) - crAt(f2)*0.78, y2, zAt(f2) - 0.04},
              {leAt(f2) - crAt(f2),      y2, zAt(f2) - 0.04},
              0.03, 0.0, 0.0, -1.0);
}

// Empenaje: deriva vertical en flecha (la aleta grande del modelo) +
// estabilizador horizontal
static void drawTiltTail(const TiltrotorGeom& g)
{
    const double xr = g.xDeriva + 0.55 * g.derivaCuerda;   // borde de ataque en la raiz
    const double h  = -g.derivaAltura;                     // hacia arriba (z negativa)

    glColor3f(0.68f, 0.71f, 0.75f);
    // Deriva: placa en el plano XZ -> su normal es el eje y
    drawPlate({xr,                    0.0, -g.fusRadio*0.6},
              {xr - g.derivaCuerda,   0.0, -g.fusRadio*0.6},
              {xr - 0.42*g.derivaCuerda,          0.0, h},
              {xr - 0.42*g.derivaCuerda - 0.46*g.derivaCuerda, 0.0, h},
              0.06, 0.0, 1.0, 0.0);

    // Timon de direccion (borde de fuga de la deriva)
    glColor3f(0.86f, 0.55f, 0.16f);
    drawPlate({xr - 0.80*g.derivaCuerda, 0.0, -g.fusRadio*0.6 - 0.10},
              {xr - g.derivaCuerda,      0.0, -g.fusRadio*0.6 - 0.10},
              {xr - 0.42*g.derivaCuerda - 0.30*g.derivaCuerda, 0.0, h*0.94},
              {xr - 0.42*g.derivaCuerda - 0.46*g.derivaCuerda, 0.0, h*0.94},
              0.035, 0.0, 1.0, 0.0);

    // Estabilizador horizontal (con su elevador)
    const double xe = g.xEstab + 0.5 * g.estabCuerda, ye = g.estabEnverg * 0.5;
    glColor3f(0.68f, 0.71f, 0.75f);
    for (int s = -1; s <= 1; s += 2) {
        drawPlate({xe, 0.10*s, -0.05}, {xe - g.estabCuerda, 0.10*s, -0.05},
                  {xe - 0.18, ye*s, -0.09}, {xe - 0.18 - 0.72*g.estabCuerda, ye*s, -0.09},
                  0.05, 0.0, 0.0, -1.0);
    }
}

// Pilon: une el pivote de cada gondola con el ala (o con el fuselaje, en el
// par delantero, que va montado por delante del borde de ataque)
static void drawTiltPylon(const TiltrotorGeom& g, int i)
{
    const bool   del = (i < 2);
    const double xp  = del ? g.xGondFwd : g.xGondAft;
    const double yp  = (i % 2 == 0 ? +1.0 : -1.0) * (del ? g.yGondFwd : g.yGondAft);

    // Punto del ala al que se ancla: mismo y, sobre la linea de 1/4 de cuerda
    const double f   = (std::abs(yp) - 0.20) / (g.envergadura*0.5 - 0.20);
    const double xw  = (g.xAla + 0.25*g.cuerdaRaiz) - g.barrido * std::max(0.0, std::min(1.0, f))
                     - 0.35 * g.cuerdaRaiz;
    const double zw  = -g.diedro * std::max(0.0, std::min(1.0, f));

    glColor3f(0.46f, 0.49f, 0.54f);
    glBegin(GL_QUADS);
        nBody(0, (yp >= 0 ? 1 : -1), 0);
        vBody(xp + 0.30, yp, g.zGond);  vBody(xp - 0.30, yp, g.zGond);
        vBody(xw - 0.22, yp, zw);       vBody(xw + 0.26, yp, zw);
        nBody(0, (yp >= 0 ? -1 : 1), 0);
        vBody(xp + 0.30, yp - 0.001, g.zGond);  vBody(xw + 0.26, yp - 0.001, zw);
        vBody(xw - 0.22, yp - 0.001, zw);       vBody(xp - 0.30, yp - 0.001, g.zGond);
    glEnd();
}

// Gondola basculante i + su rotor. Toda la gondola gira alrededor del eje y
// del cuerpo con el angulo g_tilt — el mismo angulo con el que se evalua
// buildTiltrotor(), asi que lo que se ve y lo que se simula coinciden.
static void drawTiltNacelle(GLUquadric* q, const TiltrotorGeom& g, int i)
{
    const float  V   = TILT_VIS;
    const bool   del = (i < 2);
    const double xp  = del ? g.xGondFwd : g.xGondAft;
    const double yp  = (i % 2 == 0 ? +1.0 : -1.0) * (del ? g.yGondFwd : g.yGondAft);
    const float  sig = (float)(g_tilt * 180.0 / M_PI);

    glPushMatrix();
        // Al pivote, en coordenadas GL
        glTranslatef((float)(yp*V), (float)(-g.zGond*V), (float)(-xp*V));
        // Alinear el +Z local con el eje de empuje (sin s, 0, -cos s) del cuerpo:
        // con sigma=0 apunta hacia arriba, con sigma=90 hacia adelante.
        glRotatef(-(90.0f + sig), 1, 0, 0);

        const float lFront = (float)(g.gondLargo * 0.40) * V;   // delante del pivote
        const float lBack  = (float)(g.gondLargo * 0.60) * V;   // detras del pivote
        const float R      = (float)g.gondRadio * V;

        // Cuerpo de la gondola
        glColor3f(del ? 0.80f : 0.34f, del ? 0.36f : 0.37f, del ? 0.24f : 0.42f);
        glPushMatrix();
            glTranslatef(0, 0, -lBack);
            gluCylinder(q, 0.55f*R, R, lBack*0.55f, 18, 1);
            glTranslatef(0, 0, lBack*0.55f);
            gluCylinder(q, R, R, lBack*0.45f + lFront, 18, 1);
        glPopMatrix();
        glPushMatrix();
            glTranslatef(0, 0, -lBack);
            glRotatef(180, 0, 1, 0);
            gluDisk(q, 0, 0.55f*R, 18, 1);
        glPopMatrix();

        // Cubo y cono del rotor, por delante de la gondola
        const float zHub = (float)(g.rotorOffset * V);
        glColor3f(0.20f, 0.21f, 0.24f);
        glPushMatrix();
            glTranslatef(0, 0, zHub - 0.22f*R);
            gluCylinder(q, R*0.85f, R*0.45f, 0.30f*R, 16, 1);
        glPopMatrix();

        // Palas: 3 por rotor, en el plano perpendicular al eje de empuje.
        // El sentido alterna entre rotores contiguos (dos CW y dos CCW) para
        // que los pares de reaccion se cancelen, como en el quad.
        const float rR      = (float)g.rotorRadio * V;
        const float sentido = (i % 2 == 0) ? 1.0f : -1.0f;
        const float rpm     = 240.0f + 900.0f * (float)g_throttle;
        const float ang     = sentido * (float)g_simTime * rpm;

        glDisable(GL_LIGHTING);

        // Disco barrido semitransparente
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(0.66f, 0.74f, 0.88f, g_crashed ? 0.05f : 0.14f);
        glBegin(GL_TRIANGLE_FAN);
            glVertex3f(0, 0, zHub);
            for (int k = 0; k <= 28; ++k) {
                const float a = (float)k * 2.0f * (float)M_PI / 28.0f;
                glVertex3f(rR*std::cos(a), rR*std::sin(a), zHub);
            }
        glEnd();
        glDisable(GL_BLEND);

        glColor3f(0.13f, 0.13f, 0.15f);
        glPushMatrix();
            glTranslatef(0, 0, zHub);
            glRotatef(ang, 0, 0, 1);
            for (int b = 0; b < 3; ++b) {
                glPushMatrix();
                glRotatef(b * 120.0f, 0, 0, 1);
                glBegin(GL_QUADS);
                    glVertex3f(0.10f*R, -0.04f*V, 0.0f);
                    glVertex3f(0.10f*R,  0.04f*V, 0.0f);
                    glVertex3f(rR,       0.055f*V, 0.0f);
                    glVertex3f(rR,      -0.055f*V, 0.0f);
                glEnd();
                glPopMatrix();
            }
        glPopMatrix();

        glEnable(GL_LIGHTING);
    glPopMatrix();
}

// Tren de aterrizaje: patines bajo el fuselaje (los 5 kg del Bloque 0)
static void drawTiltGear(const TiltrotorGeom& g)
{
    const double z0 = g.fusRadio * 0.8, z1 = g.zTren + g.trenLz*0.5;
    const double sy = g.trenLy * 0.5,   sx = g.trenLx * 0.5;

    glDisable(GL_LIGHTING);
    glColor3f(0.24f, 0.25f, 0.28f);
    glLineWidth(4.0f);
    glBegin(GL_LINES);
        for (int s = -1; s <= 1; s += 2) {
            vBody( sx*0.55, 0.18*s, z0);  vBody( sx, sy*s, z1);
            vBody(-sx*0.55, 0.18*s, z0);  vBody(-sx, sy*s, z1);
            vBody( sx, sy*s, z1);         vBody(-sx, sy*s, z1);
        }
    glEnd();
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

static void drawTiltrotor()
{
    const TiltrotorGeom g = tiltrotorGeom();
    GLUquadric* q = gluNewQuadric();
    gluQuadricNormals(q, GLU_SMOOTH);

    drawTiltFuselage(q, g);
    drawTiltWing(g, +1.0); drawTiltWing(g, -1.0);
    drawTiltTail(g);
    for (int i = 0; i < 4; ++i) drawTiltPylon(g, i);
    drawTiltGear(g);
    // las gondolas al final: sus discos de rotor son semitransparentes
    for (int i = 0; i < 4; ++i) drawTiltNacelle(q, g, i);

    gluDeleteQuadric(q);
}

// Dibuja el vehiculo activo
static void drawVehicle()
{
    if      (g_cfg.basculante)      drawTiltrotor();
    else if (g_cfg.empujeColectivo) drawQuadcopter();
    else                            drawDeltaWing();
}

// ═══════════════════════════════════════════════════════════════════════
// TELEMETRY WINDOW — ventana secundaria: evolucion temporal del estado
// (osciloscopio de vuelo, separado de la ventana 3D del dron)
// ═══════════════════════════════════════════════════════════════════════

static int g_telW = 520, g_telH = 760;
static int g_mainWin = 0, g_telemetryWin = 0;

// Scratch buffers reusados panel a panel (hasta 3 series por panel)
static double g_scratchA[TELEMETRY_CAPACITY];
static double g_scratchB[TELEMETRY_CAPACITY];
static double g_scratchC[TELEMETRY_CAPACITY];

static void telemetryText(float x, float y, const char* s)
{
    glRasterPos2f(x, y);
    while (*s) glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *s++);
}

// Rango [lo,hi] con margen, evitando escala degenerada cuando la serie es plana
static void minMaxPadded(const double* v, int n, double& lo, double& hi)
{
    lo = v[0]; hi = v[0];
    for (int i = 1; i < n; ++i) {
        if (v[i] < lo) lo = v[i];
        if (v[i] > hi) hi = v[i];
    }
    if (hi - lo < 1e-6) { lo -= 0.5; hi += 0.5; }
    double pad = (hi - lo) * 0.08;
    lo -= pad; hi += pad;
}

// Ancho aproximado (px) de un string en GLUT_BITMAP_8_BY_13 (8px/caracter)
static float textWidth(const char* s) { return 8.0f * (float)std::strlen(s); }

// Traza una serie dentro del area de ploteo [x0,y0,w,h] (autoescalada a [minV,maxV])
static void plotSeries(float x0, float y0, float w, float h,
                        double minV, double maxV,
                        const double* vals, int count,
                        float r, float g, float b)
{
    if (count < 2 || maxV <= minV) return;
    glColor3f(r, g, b);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < count; ++i) {
        float px = x0 + w * (float)i / (float)(count - 1);
        float t  = (float)((vals[i] - minV) / (maxV - minV));
        float py = y0 + t * h;
        glVertex2f(px, py);
    }
    glEnd();
}

struct Series {
    const double* data;
    const char*   label;   // nombre corto para la leyenda (u, v, w, ...)
    float r, g, b;
};

// Panel tipo osciloscopio completo: marco, titulo con unidad, leyenda de
// series, eje Y con valores min/max (+ linea de referencia en 0) y eje X
// con el rango de tiempo [t0,t1] en segundos que cubre el buffer visible.
static void drawTelemetryPanel(float x0, float y0, float w, float h,
                                const char* varName, const char* unit,
                                const Series* series, int nSeries,
                                double t0, double t1)
{
    // Layout interno: title (arriba), area de ploteo, eje X (abajo)
    const float titleH = 16.0f, axisH = 14.0f, padX = 6.0f;
    const float plotX = x0 + padX, plotW = w - 2.0f*padX;
    const float plotY = y0 + axisH, plotH = h - titleH - axisH;

    // Fondo + marco del panel
    glColor3f(0.13f, 0.13f, 0.17f);
    glBegin(GL_QUADS);
        glVertex2f(x0, y0); glVertex2f(x0+w, y0);
        glVertex2f(x0+w, y0+h); glVertex2f(x0, y0+h);
    glEnd();
    glColor3f(0.36f, 0.36f, 0.44f);
    glBegin(GL_LINE_LOOP);
        glVertex2f(x0, y0); glVertex2f(x0+w, y0);
        glVertex2f(x0+w, y0+h); glVertex2f(x0, y0+h);
    glEnd();

    // Titulo: nombre de la variable + unidad
    char title[80];
    snprintf(title, sizeof(title), "%s [%s]", varName, unit);
    glColor3f(0.85f, 0.85f, 0.90f);
    telemetryText(x0 + padX, y0 + h - 12, title);

    // Leyenda de series (u, v, w, ...) en el color de cada linea
    if (nSeries > 1) {
        float lx = x0 + w - padX;
        for (int i = nSeries - 1; i >= 0; --i) {
            float lw = textWidth(series[i].label);
            lx -= lw;
            glColor3f(series[i].r, series[i].g, series[i].b);
            telemetryText(lx, y0 + h - 12, series[i].label);
            lx -= 10.0f;
        }
    }

    if (g_telemetryCount < 2) return;

    // Rango combinado de todas las series (para compartir la misma escala)
    double lo = 0.0, hi = 0.0;
    for (int i = 0; i < nSeries; ++i) {
        double slo, shi;
        minMaxPadded(series[i].data, g_telemetryCount, slo, shi);
        if (i == 0) { lo = slo; hi = shi; }
        else        { lo = std::min(lo, slo); hi = std::max(hi, shi); }
    }

    // Eje Y: valores maximo (arriba) y minimo (abajo), alineados a la derecha
    char buf[32];
    glColor3f(0.60f, 0.60f, 0.66f);
    snprintf(buf, sizeof(buf), "%.2f", hi);
    telemetryText(plotX + plotW - textWidth(buf), plotY + plotH - 11, buf);
    snprintf(buf, sizeof(buf), "%.2f", lo);
    telemetryText(plotX + plotW - textWidth(buf), plotY + 1, buf);

    // Linea de referencia en 0, si el rango la cruza
    if (lo < 0.0 && hi > 0.0 && hi > lo) {
        float t  = (float)((0.0 - lo) / (hi - lo));
        float py = plotY + t * plotH;
        glColor3f(0.32f, 0.32f, 0.38f);
        glBegin(GL_LINES); glVertex2f(plotX, py); glVertex2f(plotX+plotW, py); glEnd();
    }

    // Series de datos
    for (int i = 0; i < nSeries; ++i)
        plotSeries(plotX, plotY, plotW, plotH, lo, hi,
                   series[i].data, g_telemetryCount, series[i].r, series[i].g, series[i].b);

    // Eje X: tiempo — extremos del rango visible, en segundos
    glColor3f(0.55f, 0.55f, 0.60f);
    snprintf(buf, sizeof(buf), "%.1f s", t0);
    telemetryText(plotX, y0 + 1, buf);
    snprintf(buf, sizeof(buf), "%.1f s", t1);
    telemetryText(plotX + plotW - textWidth(buf), y0 + 1, buf);
}

static void telemetryDisplay()
{
    glClear(GL_COLOR_BUFFER_BIT);

    glColor3f(0.85f, 0.85f, 0.90f);
    telemetryText(10, (float)g_telH - 20, "TELEMETRIA -- evolucion de estado");

    const int n = g_telemetryCount;
    if (n < 2) { glutSwapBuffers(); return; }

    const float margin  = 12.0f;
    const float topSkip = 30.0f;
    const int   NPANELS = 5;
    const float gap     = 8.0f;
    const float panelW  = (float)g_telW - 2*margin;
    const float panelH  = ((float)g_telH - topSkip - margin - (NPANELS-1)*gap) / NPANELS;
    const double r2d = 180.0 / M_PI;
    const double t0 = telemetryAt(0).t, t1 = telemetryAt(n-1).t;

    float y = (float)g_telH - topSkip - panelH;

    // Panel 1: Altitud sobre el piso (plano X-Y, pd=PISO_PD) [m]
    for (int i = 0; i < n; ++i) g_scratchA[i] = PISO_PD - telemetryAt(i).pd;
    {
        Series s[] = {{ g_scratchA, "alt", 0.55f, 0.80f, 1.0f }};
        drawTelemetryPanel(margin, y, panelW, panelH, "Altitud sobre el piso", "m", s, 1, t0, t1);
    }
    y -= panelH + gap;

    // Panel 2: Velocidad body u,v,w [m/s]
    for (int i = 0; i < n; ++i) {
        const TelemetrySample& smp = telemetryAt(i);
        g_scratchA[i] = smp.u; g_scratchB[i] = smp.v; g_scratchC[i] = smp.w;
    }
    {
        Series s[] = {{ g_scratchA, "u", 1.0f,0.40f,0.40f },
                      { g_scratchB, "v", 0.40f,1.0f,0.40f },
                      { g_scratchC, "w", 0.45f,0.60f,1.0f }};
        drawTelemetryPanel(margin, y, panelW, panelH, "Velocidad body", "m/s", s, 3, t0, t1);
    }
    y -= panelH + gap;

    // Panel 3: Actitud phi,theta,psi [deg]
    for (int i = 0; i < n; ++i) {
        const TelemetrySample& smp = telemetryAt(i);
        g_scratchA[i] = smp.phi*r2d; g_scratchB[i] = smp.theta*r2d; g_scratchC[i] = smp.psi*r2d;
    }
    {
        Series s[] = {{ g_scratchA, "phi", 1.0f,0.40f,0.40f },
                      { g_scratchB, "theta", 0.40f,1.0f,0.40f },
                      { g_scratchC, "psi", 0.45f,0.60f,1.0f }};
        drawTelemetryPanel(margin, y, panelW, panelH, "Actitud", "deg", s, 3, t0, t1);
    }
    y -= panelH + gap;

    // Panel 4: Tasas angulares p,q,r [deg/s]
    for (int i = 0; i < n; ++i) {
        const TelemetrySample& smp = telemetryAt(i);
        g_scratchA[i] = smp.p*r2d; g_scratchB[i] = smp.q*r2d; g_scratchC[i] = smp.r*r2d;
    }
    {
        Series s[] = {{ g_scratchA, "p", 1.0f,0.40f,0.40f },
                      { g_scratchB, "q", 0.40f,1.0f,0.40f },
                      { g_scratchC, "r", 0.45f,0.60f,1.0f }};
        drawTelemetryPanel(margin, y, panelW, panelH, "Tasas angulares", "deg/s", s, 3, t0, t1);
    }
    y -= panelH + gap;

    // Panel 5: Posicion horizontal pn,pe [m]
    for (int i = 0; i < n; ++i) {
        const TelemetrySample& smp = telemetryAt(i);
        g_scratchA[i] = smp.pn; g_scratchB[i] = smp.pe;
    }
    {
        Series s[] = {{ g_scratchA, "pn", 1.0f,0.40f,0.40f },
                      { g_scratchB, "pe", 0.40f,1.0f,0.40f }};
        drawTelemetryPanel(margin, y, panelW, panelH, "Posicion horizontal", "m", s, 2, t0, t1);
    }

    glutSwapBuffers();
}

static void telemetryReshape(int w, int h)
{
    if (h == 0) h = 1;
    g_telW = w; g_telH = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    glOrtho(0, w, 0, h, -1, 1);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
}

// Interferencia estilo "senal de TV perdida" — se dibuja cuando g_crashed
// esta activo (choque contra el piso o contra el obstaculo), sobre el
// HUD/escena ya renderizados, en coordenadas de pantalla (mismo ortho que
// hudText). Saturada a proposito: comunicacion totalmente perdida, no un
// glitch leve.
static void drawSignalLossOverlay()
{
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
    glOrtho(0, 900, 0, 600, -1, 1);
    glMatrixMode(GL_MODELVIEW);  glPushMatrix(); glLoadIdentity();
    glDisable(GL_LIGHTING); glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Fondo oscurecido — tapa casi toda la escena, comunicacion perdida
    glColor4f(0.0f, 0.0f, 0.0f, 0.55f);
    glBegin(GL_QUADS);
        glVertex2f(0,0); glVertex2f(900,0); glVertex2f(900,600); glVertex2f(0,600);
    glEnd();

    // "Nieve" densa — bloques de ruido aleatorio, estilo TV sin senal
    glBegin(GL_QUADS);
    for (int i = 0; i < 200; i++) {
        float x = (float)(rand() % 900), y = (float)(rand() % 600);
        float s = 3.0f + (float)(rand() % 5);
        float g = (float)(rand() % 100) / 100.0f;
        glColor4f(g, g, g, 0.55f);
        glVertex2f(x,y); glVertex2f(x+s,y); glVertex2f(x+s,y+s); glVertex2f(x,y+s);
    }
    glEnd();

    // Franjas de glitch horizontal (barras anchas desplazadas)
    glBegin(GL_QUADS);
    for (int i = 0; i < 25; i++) {
        float y     = (float)(rand() % 600);
        float h     = 3.0f + (float)(rand() % 14);
        float xOff  = (float)((rand() % 160) - 80);
        float shade = 0.2f + 0.7f * (float)(rand() % 100) / 100.0f;
        glColor4f(shade, shade, shade, 0.55f);
        glVertex2f(xOff,      y);   glVertex2f(900+xOff, y);
        glVertex2f(900+xOff,  y+h); glVertex2f(xOff,     y+h);
    }
    glEnd();

    // Parpadeo rojo de alarma — cada frame, mas caotico que un timer fijo
    if (rand() % 2 == 0) {
        glColor4f(0.65f, 0.0f, 0.0f, 0.20f);
        glBegin(GL_QUADS);
            glVertex2f(0,0); glVertex2f(900,0); glVertex2f(900,600); glVertex2f(0,600);
        glEnd();
    }

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST); glEnable(GL_LIGHTING);
    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW);  glPopMatrix();
}

// ═══════════════════════════════════════════════════════════════════════
// GLUT CALLBACKS
// ═══════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════
// SELECCION DE VEHICULO — pantalla inicial
//
// Los dos vehiculos comparten toda la simulacion (mundo, obstaculos, piso,
// telemetria y los Bloques 0-4); elegir uno solo cambia sus propiedades de
// masa (Bloque 0) y su configuracion de vuelo. Se puede volver aca en
// cualquier momento con la tecla v.
// ═══════════════════════════════════════════════════════════════════════

static void startVehicle(VehicleType tipo)
{
    // Bloque 0 — masa/inercia derivadas de la geometria de cada vehiculo.
    // El convertiplano arranca con las gondolas verticales (tilt = 0), asi
    // que su geometria de masa se evalua en ese angulo.
    g_tilt     = 0.0;
    g_autoTilt = false;
    g_keyTiltF = g_keyTiltB = false;

    if      (tipo == VEHICLE_QUAD) g_drone = buildQuadcopter();
    else if (tipo == VEHICLE_TILT) g_drone = buildTiltrotor(g_tilt);
    else                           g_drone = buildDeltaWingDrone();
    g_cfg   = makeVehicleConfig(tipo, g_drone.mass);

    g_state    = UAVState{};
    g_state.pd = -50.0;            // 50 m de altitud inicial (NED: negativo = arriba)
    g_state.u  = g_cfg.uInit;      // el ala fija arranca en crucero; el quad, en vuelo estacionario
    g_state.p  = g_argP;
    g_state.q  = g_argQ;
    g_state.r  = g_argR;
    g_init     = g_state;

    // El convertiplano no escribe p,q,r a mano: los rates de la linea de
    // comandos se interpretan como la TASA COMANDADA inicial (Bloque 4b).
    g_pCmd = g_qCmd = g_rCmd = 0.0;

    g_prop        = tiltPropModel(tiltrotorGeom(), g_cfg.thrustMax, g_aero.rho);
    g_throttle    = g_cfg.throttleInit;
    g_thrustCmd   = g_throttle * g_cfg.thrustMax;
    g_thrustN     = g_thrustCmd;
    g_crashed     = false;
    g_obstacleHit = false;
    for (int i = 0; i < N_OBSTACLES; ++i) g_obstacleHitFlags[i] = false;
    resetTelemetry();
    g_paused    = false;
    g_keyW      = g_keyS = false;
    g_selecting = false;

    printf("\n[%s]  masa=%.1f kg  |  Ixx=%.1f Iyy=%.1f Izz=%.1f kg*m^2  |  "
           "empuje max=%.0f N  |  palanca inicial=%.0f%%\n",
           g_cfg.nombre, g_drone.mass,
           g_drone.I_full.Ixx, g_drone.I_full.Iyy, g_drone.I_full.Izz,
           g_cfg.thrustMax, g_cfg.throttleInit * 100.0);

    if (g_cfg.basculante) {
        const TiltrotorGeom tg = tiltrotorGeom();
        printf("        ala S=%.2f m^2  b=%.2f m  c=%.2f m  (AR=%.2f)  |  "
               "gondolas verticales (basculacion 0 deg)  |  [ ] basculan, t = transicion auto\n",
               tiltrotorAreaAlar(tg), tg.envergadura, tiltrotorCuerdaMedia(tg),
               tg.envergadura*tg.envergadura / tiltrotorAreaAlar(tg));
    }

    if (g_mainWin) {
        char titulo[96];
        snprintf(titulo, sizeof(titulo), "%s — Vuelo 6DOF", g_cfg.nombre);
        int prev = glutGetWindow();
        glutSetWindow(g_mainWin);
        glutSetWindowTitle(titulo);
        if (prev) glutSetWindow(prev);
    }
}

static void drawSelectionScreen()
{
    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glColor3f(0.92f, 0.92f, 0.95f);
    hudTextFont(190, 545, "ELEGI VEHICULO", GLUT_BITMAP_TIMES_ROMAN_24);

    glColor3f(0.50f, 0.52f, 0.58f);
    hudText(190, 518, "misma simulacion, mismos obstaculos, misma fisica (Bloques 0-4)");

    // Opcion 1 — ala delta
    glColor3f(1.00f, 0.80f, 0.25f);
    hudTextFont(190, 468, "[1]  UAV ala delta", GLUT_BITMAP_9_BY_15);
    glColor3f(0.72f, 0.74f, 0.78f);
    hudText(220, 446, "empuje longitudinal (+X cuerpo) y sustentacion de ala");
    hudText(220, 430, "la palanca controla VELOCIDAD; la altitud se sostiene sola");
    hudText(220, 414, "200 kg  |  3.5 m de largo  |  arranca en crucero a 5 m/s");

    // Opcion 2 — quadcopter
    glColor3f(0.45f, 0.85f, 1.00f);
    hudTextFont(190, 370, "[2]  Quadcopter (config X)", GLUT_BITMAP_9_BY_15);
    glColor3f(0.72f, 0.74f, 0.78f);
    hudText(220, 348, "empuje colectivo de 4 rotores (-Z cuerpo, hacia arriba)");
    hudText(220, 332, "la palanca controla ASCENSO; para avanzar hay que inclinarlo");
    hudText(220, 316, "142 kg  |  3.2 m punta a punta  |  estacionario (T/W=2.2, hover 45%)");

    // Opcion 3 — convertiplano
    glColor3f(0.60f, 1.00f, 0.55f);
    hudTextFont(190, 272, "[3]  Convertiplano (4 rotores basculantes)", GLUT_BITMAP_9_BY_15);
    glColor3f(0.72f, 0.74f, 0.78f);
    hudText(220, 250, "las gondolas BASCULAN: despega como multirrotor y vuela como avion");
    hudText(220, 234, "empuje vectorizado + ala real (sustentacion y arrastre vs Va y alfa)");
    hudText(220, 218, "unico con control por TORQUES (Bloque 4) y masa/inercia variable");
    hudText(220, 202, "155 kg  |  4.2 m de largo, 5.6 m de envergadura  |  T/W=2.0");

    glColor3f(0.55f, 0.57f, 0.62f);
    hudText(190, 150, "Presiona 1, 2 o 3 para despegar.   Q sale.");
    hudText(190, 130, "Durante el vuelo, v vuelve a esta pantalla.");

    glEnable(GL_LIGHTING);
}

static void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    if (g_selecting) {
        drawSelectionScreen();
        glutSwapBuffers();
        return;
    }

    glTranslatef(0, 0, g_zoom);
    glRotatef(g_rotX, 1, 0, 0);
    glRotatef(g_rotY, 0, 1, 0);

    // World reference — se traslada en sentido opuesto a la posicion inercial
    // del dron (camara "chase"): el dron queda centrado en pantalla y se ve
    // el mundo recular a medida que vuela. NED (pn,pe,pd) -> GL (pe,-pd,-pn).
    glPushMatrix();
    glTranslatef(-(float)g_state.pe, (float)g_state.pd, (float)g_state.pn);
    drawWorldAxes();
    drawHorizonRing();
    drawGroundGrid();
    drawObstacles();
    glPopMatrix();

    // Drone centrado en pantalla, orientado con la actitud actual
    // Rotation order ZYX (yaw → pitch → roll) — NED convention
    glPushMatrix();
    glRotatef(-(float)(g_state.psi   * 180.0/M_PI), 0, 1, 0);  // yaw
    glRotatef( (float)(g_state.theta * 180.0/M_PI), 1, 0, 0);  // pitch
    glRotatef(-(float)(g_state.phi   * 180.0/M_PI), 0, 0, 1);  // roll

    glPolygonMode(GL_FRONT_AND_BACK, g_wire ? GL_LINE : GL_FILL);
    drawBodyAxes();
    drawVehicle();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPopMatrix();

    drawHUD();
    if (g_crashed) drawSignalLossOverlay();
    glutSwapBuffers();
}

static void timerStep(int)
{
    if (!g_selecting && !g_paused && !g_crashed)
        step(DT);

    glutSetWindow(g_mainWin);       glutPostRedisplay();
    glutSetWindow(g_telemetryWin);  glutPostRedisplay();
    glutTimerFunc(16, timerStep, 0);
}

static void reshape(int w, int h)
{
    if (h == 0) h = 1;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    gluPerspective(45.0, (double)w/h, 0.1, 500.0);
    glMatrixMode(GL_MODELVIEW);
}

static void mouseButton(int btn, int state, int x, int y)
{
    if (btn == GLUT_LEFT_BUTTON) { g_drag=(state==GLUT_DOWN); g_lastX=x; g_lastY=y; }
    if (btn == 3) { g_zoom += 1.2f; glutPostRedisplay(); }
    if (btn == 4) { g_zoom -= 1.2f; glutPostRedisplay(); }
}

static void mouseMotion(int x, int y)
{
    if (!g_drag) return;
    g_rotY += (x - g_lastX) * 0.5f;
    g_rotX += (y - g_lastY) * 0.5f;
    g_lastX=x; g_lastY=y;
    glutPostRedisplay();
}

static void keyboard(unsigned char key, int, int)
{
    const double dp = 0.02;

    // Pantalla inicial: solo elegir vehiculo o salir
    if (g_selecting) {
        switch (key) {
            case '1': startVehicle(VEHICLE_UAV);  break;
            case '2': startVehicle(VEHICLE_QUAD); break;
            case '3': startVehicle(VEHICLE_TILT); break;
            case 'q': case 'Q': case 27: exit(0);
        }
        glutPostRedisplay();
        return;
    }

    if (g_crashed && key != '0' && key != 'v' && key != 'q' && key != 'Q' && key != 27) {
        g_keyTiltF = g_keyTiltB = false;
        glutPostRedisplay();
        return;
    }
    switch (key) {
        // Guinada: los dos primeros vehiculos escriben r directamente; el
        // convertiplano mueve la TASA COMANDADA y deja que el Bloque 4b
        // decida el torque (y si hay autoridad para darlo).
        case 'a': case 'A':
            if (g_cfg.basculante) g_rCmd = std::max(-TILT_R_CMD_MAX, g_rCmd - dp);
            else                  g_state.r -= dp;
            break;
        case 'd': case 'D':
            if (g_cfg.basculante) g_rCmd = std::min( TILT_R_CMD_MAX, g_rCmd + dp);
            else                  g_state.r += dp;
            break;
        case '[': case '{':                            // bascular hacia estacionario
            if (g_cfg.basculante) { g_keyTiltB = true; g_autoTilt = false; }
            break;
        case ']': case '}':                            // bascular hacia crucero
            if (g_cfg.basculante) { g_keyTiltF = true; g_autoTilt = false; }
            break;
        case 't': case 'T':                            // transicion automatica a crucero
            if (g_cfg.basculante) g_autoTilt = !g_autoTilt;
            break;
        case 'w': g_keyW = true; break;              // palanca +  (Bloques 1-2)
        case 's': case 'S': g_keyS = true; break;     // palanca -  (Bloques 1-2)
        case ' ': g_paused = !g_paused;    break;
        case '0':
            g_state = g_init; g_crashed = false; g_obstacleHit = false;
            for (int i = 0; i < N_OBSTACLES; ++i) g_obstacleHitFlags[i] = false;
            g_throttle = g_cfg.throttleInit; resetTelemetry();
            g_tilt = 0.0; g_autoTilt = false; g_keyTiltF = g_keyTiltB = false;
            g_pCmd = g_qCmd = g_rCmd = 0.0;
            if (g_cfg.basculante) g_drone = buildTiltrotor(g_tilt);
            break;
        case 'v': case 'V':                            // volver a elegir vehiculo
            g_selecting = true; g_keyW = g_keyS = false; break;
        case 'W': g_wire = !g_wire; break;             // Shift+W: wireframe
        case 'r': case 'R': g_rotX=20; g_rotY=-35; g_zoom=-22; break;
        case '+': case '=': g_zoom += 1.5f; break;
        case '-':           g_zoom -= 1.5f; break;
        case 'q': case 'Q': case 27: exit(0);
    }
    glutPostRedisplay();
}

// Suelta W/S — el empuje se sostiene en el valor alcanzado, no vuelve a cero
static void keyboardUp(unsigned char key, int, int)
{
    switch (key) {
        case 'w': g_keyW = false; break;
        case 's': case 'S': g_keyS = false; break;
        case '[': case '{': g_keyTiltB = false; break;
        case ']': case '}': g_keyTiltF = false; break;
    }
}

static void specialKey(int key, int, int)
{
    const double dp = 0.02;
    if (g_selecting || g_crashed) { glutPostRedisplay(); return; }
    if (g_cfg.basculante) {
        // Convertiplano: las flechas mueven la tasa COMANDADA (Bloque 4b),
        // no la tasa real — esa sale de integrar el Bloque 4.
        switch (key) {
            case GLUT_KEY_LEFT:  g_pCmd = std::max(-TILT_P_CMD_MAX, g_pCmd - dp); break;
            case GLUT_KEY_RIGHT: g_pCmd = std::min( TILT_P_CMD_MAX, g_pCmd + dp); break;
            case GLUT_KEY_UP:    g_qCmd = std::min( TILT_Q_CMD_MAX, g_qCmd + dp); break;
            case GLUT_KEY_DOWN:  g_qCmd = std::max(-TILT_Q_CMD_MAX, g_qCmd - dp); break;
        }
        glutPostRedisplay();
        return;
    }
    switch (key) {
        case GLUT_KEY_LEFT:  g_state.p -= dp; break;
        case GLUT_KEY_RIGHT: g_state.p += dp; break;
        case GLUT_KEY_UP:    g_state.q += dp; break;
        case GLUT_KEY_DOWN:  g_state.q -= dp; break;
    }
    glutPostRedisplay();
}

// ═══════════════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
    // Linea de comandos:  ./drone_model [uav|quad] [p q r]
    // Sin vehiculo, arranca en la pantalla de seleccion. Los rates iniciales
    // siguen siendo posicionales, como siempre.
    bool  vehiculoPorCLI = false;
    VehicleType tipoCLI  = VEHICLE_UAV;
    int   arg            = 1;

    if (argc > 1) {
        const char* a = argv[1];
        if (!strcmp(a,"uav") || !strcmp(a,"--uav") || !strcmp(a,"ala")) {
            vehiculoPorCLI = true; tipoCLI = VEHICLE_UAV;  arg = 2;
        } else if (!strcmp(a,"quad") || !strcmp(a,"--quad") || !strcmp(a,"quadcopter")) {
            vehiculoPorCLI = true; tipoCLI = VEHICLE_QUAD; arg = 2;
        } else if (!strcmp(a,"tilt") || !strcmp(a,"--tilt") || !strcmp(a,"tiltrotor")
                || !strcmp(a,"convertiplano")) {
            vehiculoPorCLI = true; tipoCLI = VEHICLE_TILT; arg = 2;
        }
    }

    g_argP = (argc > arg)     ? atof(argv[arg])     : 0.0;
    g_argQ = (argc > arg + 1) ? atof(argv[arg + 1]) : 0.0;
    g_argR = (argc > arg + 2) ? atof(argv[arg + 2]) : 0.0;

    printf("Rates iniciales: p=%.3f  q=%.3f  r=%.3f rad/s\n", g_argP, g_argQ, g_argR);

    // Estado valido antes de que corra el primer display() (la pantalla de
    // seleccion no lo usa, pero nada debe leer basura)
    g_drone = buildDeltaWingDrone();
    g_cfg   = makeVehicleConfig(VEHICLE_UAV, g_drone.mass);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(900, 600);
    glutInitWindowPosition(50, 50);
    g_mainWin = glutCreateWindow("Unmanned — Vuelo 6DOF");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glShadeModel(GL_SMOOTH);

    GLfloat lpos[]={10,20,-5,0}, lamb[]={0.25f,0.25f,0.25f,1}, ldif[]={0.85f,0.85f,0.85f,1};
    glLightfv(GL_LIGHT0, GL_POSITION, lpos);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  lamb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  ldif);
    glClearColor(0.07f, 0.07f, 0.11f, 1.0f);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouseButton);
    glutMotionFunc(mouseMotion);
    glutKeyboardFunc(keyboard);
    glutKeyboardUpFunc(keyboardUp);
    glutSpecialFunc(specialKey);

    // Ventana secundaria — telemetria (osciloscopio 2D, sin luces/profundidad)
    glutInitWindowSize(g_telW, g_telH);
    glutInitWindowPosition(970, 50);
    g_telemetryWin = glutCreateWindow("Unmanned — Telemetria");

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glClearColor(0.05f, 0.05f, 0.07f, 1.0f);

    glutDisplayFunc(telemetryDisplay);
    glutReshapeFunc(telemetryReshape);

    glutSetWindow(g_mainWin);

    // Con vehiculo en la linea de comandos se saltea la pantalla de seleccion
    if (vehiculoPorCLI) startVehicle(tipoCLI);

    glutTimerFunc(16, timerStep, 0);

    glutMainLoop();
    return 0;
}
