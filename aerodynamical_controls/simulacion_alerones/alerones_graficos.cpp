// ═══════════════════════════════════════════════════════════════════════════
//  alerones_graficos.cpp — dibujo 3D del cohete + aletas, HUD, vista de cola,
//  y traduccion de teclado a comandos.
//
//  Convencion de dibujo
//  ────────────────────
//  La geometria se define en EJES CUERPO (x adelante, y derecha, z abajo) y se
//  lleva a OpenGL con la matriz  C = B * R_b^i,  donde R_b^i sale del cuaternion
//  y B es el mismo cambio de base que usa unmanned/drone_model.cpp:
//
//      body/inercial -> GL :   (ix, iy, iz)  ->  (iy, -iz, -ix)
//
//  es decir  x_cuerpo -> -z_GL (la nariz apunta hacia el observador),
//            y_cuerpo -> +x_GL,  z_cuerpo -> -y_GL (abajo es abajo en pantalla).
//
//  Sentido de giro de cada aleta (deducido de la tabla de signos del header):
//    · Aletas 1-2 (horizontales): giran -delta alrededor del eje y del cuerpo.
//      delta>0 baja el borde de ataque -> incidencia negativa -> fuerza +z
//      (hacia abajo) en la cola -> morro arriba -> M>0.
//    · Aletas 3-4 (verticales): giran +delta alrededor del eje z del cuerpo.
//      delta>0 lleva el borde de ataque hacia +y -> fuerza +y en la cola ->
//      morro hacia -y -> N<0.
// ═══════════════════════════════════════════════════════════════════════════
#include "alerones_graficos.hpp"

#include <GLUT/glut.h>
#include <OpenGL/glu.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace alerones {

// ─────────────────────────────────────────────────────────────────────────────
// Geometria del cohete (ejes cuerpo, metros)
// ─────────────────────────────────────────────────────────────────────────────
static const float FUS_R      = 0.16f;   // radio del fuselaje
static const float NOSE_LEN   = 0.55f;   // longitud del cono de nariz
static const float FIN_CHORD  = 0.16f;   // cuerda de la aleta
static const float FIN_THICK  = 0.012f;  // espesor de la aleta
static const float FIN_TIP_C  = 0.10f;   // cuerda en la punta (aleta trapezoidal)

// ─────────────────────────────────────────────────────────────────────────────
// Estado global del render (GLUT obliga a callbacks sin contexto)
// ─────────────────────────────────────────────────────────────────────────────
static Sim   g_sim;
static Sim   g_sim0;

static float g_rotX = -18.0f, g_rotY = 28.0f;
static float g_zoom = -6.5f;
static int   g_lastX = 0, g_lastY = 0;
static bool  g_drag = false;
static bool  g_wire = false;
static bool  g_tailView = true;
static bool  g_showAxes = true;

// La flecha real de la punta es de decimas de milimetro a 30 m/s: fisicamente
// correcta e invisible.  Se exagera para dibujar, como en cualquier post de
// elementos finitos, y el factor se anuncia en el HUD para que nadie lo
// confunda con la deformacion real.  La tecla F lo cicla.
static const float g_flexScales[] = { 1.0f, 20.0f, 60.0f, 150.0f };
static int         g_flexScaleIdx = 2;
static float flexScale() { return g_flexScales[g_flexScaleIdx]; }

static int   g_mainWin = 0;
static int   g_winW = 820, g_winH = 740;

// Ventana secundaria: todos los numeros y parametros viven aqui, para que la
// ventana principal quede solo con el cohete y sus aletas (mismo patron de dos
// ventanas GLUT que usa unmanned/drone_model.cpp).
static int   g_telWin = 0;
static int   g_telW = 620, g_telH = 740;

// Incrementos de mando por pulsacion
static const double DCMD = 0.5 * M_PI / 180.0;   // 0.5 deg  (mando directo)
static const double DREF = 2.0 * M_PI / 180.0;   // 2.0 deg  (referencia del AP)
static const double DVEL = 2.0;                  // m/s

// Servo guardado al desactivarlo con la tecla S, para poder restaurarlo.
static double g_tau_saved  = 0.0;
static double g_rate_saved = 0.0;

// ═══════════════════════════════════════════════════════════════════════════
// SIMULACION
// ═══════════════════════════════════════════════════════════════════════════

Sim sim_init()
{
    Sim sim{};
    sim.P       = params_default();
    sim.s       = state_init();
    sim.C       = commands_zero();
    sim.A       = autopilot_default(sim.P);
    sim.t       = 0.0;
    sim.paused  = false;
    sim.fin_sel = 0;
    apply_commands_instant(sim.C, sim.s);
    return sim;
}

void sim_reset(Sim& sim)
{
    Params keep    = sim.P;       // conserva rho/V/geometria elegidos en runtime
    bool   ap_on   = sim.A.enabled;
    double wn_r    = sim.A.wn_roll, wn_p = sim.A.wn_pitch, wn_y = sim.A.wn_yaw;
    double zeta    = sim.A.zeta;

    sim   = sim_init();
    sim.P = keep;

    sim.A.enabled  = ap_on;
    sim.A.wn_roll  = wn_r; sim.A.wn_pitch = wn_p; sim.A.wn_yaw = wn_y;
    sim.A.zeta     = zeta;
    autopilot_tune(sim.P, sim.A);
}

// El orden importa: el autopiloto ve la actitud de ESTE instante, su salida
// pasa por el mixing, el actuador la retarda y solo entonces se integra.
void sim_step(Sim& sim, double dt)
{
    autopilot_update(sim.P, sim.A, sim.s, sim.C, dt);
    apply_commands(sim.C, sim.s);
    servo_step(sim.P, sim.s, dt);
    step_rk4(sim.P, sim.s, dt);
    sim.t += dt;
}

// ═══════════════════════════════════════════════════════════════════════════
// UTILIDADES
// ═══════════════════════════════════════════════════════════════════════════

static double clampd(double v, double lo, double hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static double deg(double r) { return r * 180.0 / M_PI; }

// Matriz GL (columna-mayor) = B * R_b^i, con B: (x,y,z) -> (y,-z,-x)
static void bodyToGLMatrix(const double q[4], GLfloat m[16])
{
    double R[9];
    rotmat_from_quat(q, R);       // fila-mayor, cuerpo -> inercial

    // C = B * R.  B tiene filas (0,1,0), (0,0,-1), (-1,0,0).
    double C[9];
    for (int c = 0; c < 3; ++c) {
        C[0*3 + c] =  R[1*3 + c];   // fila 0 de C =  fila 1 de R
        C[1*3 + c] = -R[2*3 + c];   // fila 1 de C = -fila 2 de R
        C[2*3 + c] = -R[0*3 + c];   // fila 2 de C = -fila 0 de R
    }

    for (int i = 0; i < 16; ++i) m[i] = 0.0f;
    for (int col = 0; col < 3; ++col)
        for (int row = 0; row < 3; ++row)
            m[col * 4 + row] = (GLfloat)C[row * 3 + col];
    m[15] = 1.0f;
}

static void text2D(float x, float y, const char* s, void* font = GLUT_BITMAP_8_BY_13)
{
    glRasterPos2f(x, y);
    while (*s) glutBitmapCharacter(font, *s++);
}

// ═══════════════════════════════════════════════════════════════════════════
// DIBUJO 3D — geometria en ejes cuerpo
// ═══════════════════════════════════════════════════════════════════════════

// Cilindro/cono a lo largo del eje x del cuerpo.
static void solidAlongX(GLUquadric* q, float baseR, float topR, float len)
{
    glPushMatrix();
    glRotatef(90.0f, 0, 1, 0);          // +z_GLU -> +x_cuerpo
    gluCylinder(q, baseR, topR, len, 24, 1);
    glPopMatrix();
}

static void drawFuselage(GLUquadric* q, float x_tail, float x_nose)
{
    const float bodyLen = (x_nose - NOSE_LEN) - x_tail;

    // Cuerpo cilindrico
    glColor3f(0.78f, 0.80f, 0.84f);
    glPushMatrix();
    glTranslatef(x_tail, 0, 0);
    solidAlongX(q, FUS_R, FUS_R, bodyLen);
    glPopMatrix();

    // Cono de nariz
    glColor3f(0.90f, 0.45f, 0.25f);
    glPushMatrix();
    glTranslatef(x_nose - NOSE_LEN, 0, 0);
    solidAlongX(q, FUS_R, 0.02f, NOSE_LEN);
    glPopMatrix();

    // Tapa de cola (tobera)
    glColor3f(0.35f, 0.35f, 0.40f);
    glPushMatrix();
    glTranslatef(x_tail - 0.10f, 0, 0);
    solidAlongX(q, FUS_R * 0.55f, FUS_R, 0.10f);
    glPopMatrix();

    // Banda de referencia: marca el plano de las aletas
    glColor3f(0.20f, 0.22f, 0.26f);
    glPushMatrix();
    glTranslatef(x_tail + 0.02f, 0, 0);
    solidAlongX(q, FUS_R * 1.02f, FUS_R * 1.02f, 0.03f);
    glPopMatrix();
}

// Modo de flexion de un voladizo con carga en la punta, normalizado a 1 en la
// punta:   phi(xi) = (3 xi^2 - xi^3) / 2,   xi = s/span.
// Es la deformada estatica exacta, no una interpolacion cualquiera.
static float bendShape(float xi)  { return 0.5f * (3.0f * xi * xi - xi * xi * xi); }
static float bendSlope(float xi)  { return 0.5f * (6.0f * xi - 3.0f * xi * xi); }

// Placa trapezoidal delgada: raiz en s=0, punta en s=span, cuerda de FIN_CHORD
// a FIN_TIP_C.  Se dibuja en el plano local xy y luego se orienta; la flexion
// va en +z local, que en las cuatro aletas coincide con la direccion de la
// fuerza aerodinamica.
//
// wtip es la flecha YA escalada para dibujar (metros de dibujo).
static void drawFinPlate(float span, float thick, float wtip)
{
    const int   NSEG = 10;
    const float h    = thick * 0.5f;
    const float cr   = FIN_CHORD * 0.5f;
    const float ct   = FIN_TIP_C * 0.5f;

    float y[NSEG + 1], c[NSEG + 1], zc[NSEG + 1], ny[NSEG + 1];
    for (int j = 0; j <= NSEG; ++j) {
        const float xi = (float)j / (float)NSEG;
        y[j]  = xi * span;
        c[j]  = cr + (ct - cr) * xi;             // semicuerda local
        zc[j] = wtip * bendShape(xi);            // linea elastica
        // Normal de la cara: inclinada por la pendiente de la deformada.  Sin
        // esto la iluminacion es plana y la curvatura no se ve.
        const float sl = (span > 1e-6f) ? (wtip * bendSlope(xi) / span) : 0.0f;
        ny[j] = -sl;
    }

    // Caras superior (+z) e inferior (-z)
    for (int sgn = 0; sgn < 2; ++sgn) {
        const float zs = (sgn == 0) ? -h : +h;
        const float ns = (sgn == 0) ? -1.0f : +1.0f;
        glBegin(GL_QUADS);
        for (int j = 0; j < NSEG; ++j) {
            float nn = std::sqrt(ny[j] * ny[j] + 1.0f);
            glNormal3f(0.0f, ns * ny[j] / nn, ns / nn);
            if (sgn == 0) {
                glVertex3f(+c[j],   y[j],   zc[j]   + zs);
                glVertex3f(-c[j],   y[j],   zc[j]   + zs);
                glVertex3f(-c[j+1], y[j+1], zc[j+1] + zs);
                glVertex3f(+c[j+1], y[j+1], zc[j+1] + zs);
            } else {
                glVertex3f(+c[j+1], y[j+1], zc[j+1] + zs);
                glVertex3f(-c[j+1], y[j+1], zc[j+1] + zs);
                glVertex3f(-c[j],   y[j],   zc[j]   + zs);
                glVertex3f(+c[j],   y[j],   zc[j]   + zs);
            }
        }
        glEnd();
    }

    // Cantos de ataque (+x) y de salida (-x)
    for (int side = 0; side < 2; ++side) {
        const float sx = (side == 0) ? +1.0f : -1.0f;
        glNormal3f(sx, 0.0f, 0.0f);
        glBegin(GL_QUADS);
        for (int j = 0; j < NSEG; ++j) {
            glVertex3f(sx * c[j],   y[j],   zc[j]   - h);
            glVertex3f(sx * c[j+1], y[j+1], zc[j+1] - h);
            glVertex3f(sx * c[j+1], y[j+1], zc[j+1] + h);
            glVertex3f(sx * c[j],   y[j],   zc[j]   + h);
        }
        glEnd();
    }

    // Tapas de raiz y de punta
    for (int cap = 0; cap < 2; ++cap) {
        const int   j  = (cap == 0) ? 0 : NSEG;
        const float nn = (cap == 0) ? -1.0f : +1.0f;
        glNormal3f(0.0f, nn, 0.0f);
        glBegin(GL_QUADS);
        glVertex3f(+c[j], y[j], zc[j] - h);
        glVertex3f(-c[j], y[j], zc[j] - h);
        glVertex3f(-c[j], y[j], zc[j] + h);
        glVertex3f(+c[j], y[j], zc[j] + h);
        glEnd();
    }
}

// Color de la aleta segun la magnitud de su deflexion (frio -> caliente).
static void finColor(double delta, double delta_stall, bool selected)
{
    double t = clampd(std::fabs(delta) / std::fmax(delta_stall, 1e-9), 0.0, 1.0);
    if (selected) glColor3f(1.0f, 0.95f, 0.30f);
    else          glColor3f((float)(0.30 + 0.65 * t), (float)(0.62 - 0.30 * t),
                            (float)(0.85 - 0.55 * t));
}

// Las cuatro aletas, cada una rotada por su propio angulo alrededor de su eje
// de charnela, situada en x = x_t (detras del CG, en el origen del modelo).
//
// El angulo que se dibuja es delta + theta, no delta: la aleta gira lo que le
// manda el servo MAS lo que se retuerce sobre la rigidez del eje (§5).  Como
// K_theta es una rigidez concentrada en el pivote, la torsion es uniforme a lo
// largo de la envergadura y entra en la rotacion; la flexion, en cambio, tiene
// forma de modo y se dibuja dentro de la placa.
static void drawFins(const Sim& sim, float x_t, float span)
{
    const double* d  = sim.s.delta;
    const double  ds = sim.P.delta_stall;
    const bool    fx = sim.P.flex_on;
    const float   fs = flexScale();

    for (int i = 0; i < 4; ++i) {
        const double ang  = d[i] + (fx ? sim.s.theta[i] : 0.0);
        const float  wtip = fx ? (float)(sim.s.w[i]) * fs : 0.0f;

        finColor(ang, ds, sim.fin_sel == i);
        glPushMatrix();

        if (i < 2) {
            // Aletas 1 (derecha, +y) y 2 (izquierda, -y): giro -ang sobre y
            const float ysgn = (i == 0) ? +1.0f : -1.0f;
            glTranslatef(x_t, ysgn * FUS_R, 0.0f);
            glRotatef((float)(-deg(ang)), 0, 1, 0);     // charnela = eje y
            glScalef(1.0f, ysgn, 1.0f);                 // espeja la de babor
        } else {
            // Aletas 3 (arriba, -z) y 4 (abajo, +z): giro +ang sobre z
            const float zsgn = (i == 2) ? -1.0f : +1.0f;
            glTranslatef(x_t, 0.0f, zsgn * FUS_R);
            glRotatef((float)(+deg(ang)), 0, 0, 1);     // charnela = eje z
            // La placa se dibuja en el plano xy: la llevamos al plano xz.
            glRotatef(-90.0f, 1, 0, 0);
            glScalef(1.0f, -zsgn, 1.0f);
        }

        // Ojo: glScalef(1,-1,1) espeja la ENVERGADURA, no el eje de flexion.
        // El +z local sigue apuntando donde empuja el aire en las cuatro
        // aletas, asi que wtip entra con el mismo signo en todas.
        drawFinPlate(span, FIN_THICK, wtip);
        glPopMatrix();
    }
}

// Vectores de fuerza aerodinamica de cada aleta (escalados para verse).
static void drawFinForces(const Sim& sim, float x_t, float span)
{
    // La fuerza real sale de la incidencia EFECTIVA: mando + torsion elastica
    // + termino de rotacion.  Dibujar fin_forces(delta) mentiria.
    double de[4], f[4];
    fin_effective_deflection(sim.P, sim.s.delta, sim.s.theta, sim.s.omega, de);
    fin_forces(sim.P, de, f);

    const float rc = FUS_R + 0.55f * span;          // centro de presion aprox.
    const float k  = 0.045f;                         // N -> m de dibujo

    struct { float px, py, pz, dx, dy, dz; } arr[4] = {
        { x_t, +rc, 0,  0, 0, 1 },   // f1 -> +z
        { x_t, -rc, 0,  0, 0, 1 },   // f2 -> +z
        { x_t, 0, -rc,  0, 1, 0 },   // f3 -> +y
        { x_t, 0, +rc,  0, 1, 0 }    // f4 -> +y
    };

    glDisable(GL_LIGHTING);
    glLineWidth(2.5f);
    for (int i = 0; i < 4; ++i) {
        float L = (float)(f[i] * k);
        if (std::fabs(L) < 1e-4f) continue;
        glColor3f(1.0f, 0.35f, 0.35f);
        glBegin(GL_LINES);
        glVertex3f(arr[i].px, arr[i].py, arr[i].pz);
        glVertex3f(arr[i].px + arr[i].dx * L,
                   arr[i].py + arr[i].dy * L,
                   arr[i].pz + arr[i].dz * L);
        glEnd();
        // Punta
        glPushMatrix();
        glTranslatef(arr[i].px + arr[i].dx * L,
                     arr[i].py + arr[i].dy * L,
                     arr[i].pz + arr[i].dz * L);
        glutSolidSphere(0.028, 8, 6);
        glPopMatrix();
    }
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

// Ejes del cuerpo: x rojo (adelante), y verde (derecha), z azul (abajo).
static void drawBodyAxes(float len)
{
    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.30f, 0.30f); glVertex3f(0,0,0); glVertex3f(len,0,0);
    glColor3f(0.30f, 1.0f, 0.40f); glVertex3f(0,0,0); glVertex3f(0,len,0);
    glColor3f(0.40f, 0.55f, 1.0f); glVertex3f(0,0,0); glVertex3f(0,0,len);
    glEnd();
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

// Rejilla de referencia inercial (en GL, plano horizontal y = -1.6).
static void drawGrid()
{
    glDisable(GL_LIGHTING);
    glColor3f(0.20f, 0.22f, 0.28f);
    glBegin(GL_LINES);
    for (int i = -8; i <= 8; ++i) {
        glVertex3f((float)i * 0.5f, -1.6f, -4.0f);
        glVertex3f((float)i * 0.5f, -1.6f,  4.0f);
        glVertex3f(-4.0f, -1.6f, (float)i * 0.5f);
        glVertex3f( 4.0f, -1.6f, (float)i * 0.5f);
    }
    glEnd();
    glEnable(GL_LIGHTING);
}

// ═══════════════════════════════════════════════════════════════════════════
// OVERLAY 2D — HUD y vista de cola (replica de Blakelock Fig. 8-2)
// ═══════════════════════════════════════════════════════════════════════════

static void begin2D(int w, int h)
{
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
    gluOrtho2D(0, w, 0, h);
    glMatrixMode(GL_MODELVIEW);  glPushMatrix(); glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
}

static void end2D()
{
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW);  glPopMatrix();
}

// Vista de cola: cuatro flechas con el signo de cada delta.  Permite leer de
// un vistazo si el modo activo es simetrico (pitch/yaw) o antisimetrico (roll).
static void drawTailView(const Sim& sim)
{
    const float cx = g_winW - 130.0f, cy = 150.0f, R = 78.0f;
    const double ds = sim.P.delta_stall;

    // Fondo
    glColor4f(0.10f, 0.11f, 0.15f, 1.0f);
    glBegin(GL_QUADS);
    glVertex2f(cx - 108, cy - 108); glVertex2f(cx + 108, cy - 108);
    glVertex2f(cx + 108, cy + 118); glVertex2f(cx - 108, cy + 118);
    glEnd();

    // Seccion del fuselaje
    glColor3f(0.45f, 0.48f, 0.55f);
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 40; ++i) {
        float a = 2.0f * (float)M_PI * i / 40.0f;
        glVertex2f(cx + 22.0f * std::cos(a), cy + 22.0f * std::sin(a));
    }
    glEnd();

    // Vista DESDE LA COLA mirando hacia la nariz: +y del cuerpo (derecha) va a
    // la izquierda de la pantalla, +z (abajo) va abajo.  Se dibuja en cambio la
    // vista "desde atras del observador" (y a la derecha) para que coincida con
    // la vista 3D por defecto.
    struct { float ux, uy; const char* lbl; float ax, ay; } fin[4] = {
        { +1,  0, "d1", 0,  1 },   // derecha, flecha segun +z -> abajo
        { -1,  0, "d2", 0,  1 },   // izquierda
        {  0, +1, "d3", 1,  0 },   // arriba (z negativo -> arriba en pantalla)
        {  0, -1, "d4", 1,  0 }    // abajo
    };

    // Escala de dibujo de la flecha: la flecha real de la punta es de decimas
    // de milimetro (§5), asi que aqui va exagerada como en la vista 3D.
    const float wpix = (R - 22.0f) * 0.45f * flexScale() / 60.0f;

    for (int i = 0; i < 4; ++i) {
        // Aleta como barra radial, curvada por la flexion.  La flecha se
        // aplica perpendicular al radio, en la direccion de la fuerza (ax,-ay),
        // con la deformada de voladizo phi(xi) = (3 xi^2 - xi^3)/2.
        const float wtip = sim.P.flex_on ? (float)sim.s.w[i] * wpix / 0.001f : 0.0f;

        glColor3f(0.55f, 0.60f, 0.70f);
        glLineWidth(3.0f);
        glBegin(GL_LINE_STRIP);
        for (int j = 0; j <= 12; ++j) {
            const float xi = (float)j / 12.0f;
            const float rr = 22.0f + (R - 22.0f) * xi;
            const float bw = wtip * bendShape(xi);
            glVertex2f(cx + fin[i].ux * rr + fin[i].ax * bw,
                       cy + fin[i].uy * rr - fin[i].ay * bw);
        }
        glEnd();

        // Flecha de fuerza: longitud proporcional a delta_i, sentido por signo.
        double t = clampd(sim.s.delta[i] / std::fmax(ds, 1e-9), -1.5, 1.5);
        float  Lp = (float)(t * 34.0);
        float  bx = cx + fin[i].ux * (R * 0.72f);
        float  by = cy + fin[i].uy * (R * 0.72f);
        // La flecha apunta en -ay en pantalla porque +z_cuerpo (abajo) es -y_GL.
        float  ex = bx + fin[i].ax * Lp;
        float  ey = by - fin[i].ay * Lp;

        if (std::fabs(Lp) > 0.8f) {
            glColor3f(1.0f, 0.40f, 0.35f);
            glLineWidth(2.5f);
            glBegin(GL_LINES); glVertex2f(bx, by); glVertex2f(ex, ey); glEnd();
            glBegin(GL_TRIANGLES);
            float dx = ex - bx, dy = ey - by;
            float n = std::sqrt(dx*dx + dy*dy); dx /= n; dy /= n;
            glVertex2f(ex, ey);
            glVertex2f(ex - 8*dx + 4*dy, ey - 8*dy - 4*dx);
            glVertex2f(ex - 8*dx - 4*dy, ey - 8*dy + 4*dx);
            glEnd();
        }

        glColor3f(sim.fin_sel == i ? 1.0f : 0.70f,
                  sim.fin_sel == i ? 0.95f : 0.75f,
                  sim.fin_sel == i ? 0.30f : 0.80f);
        char buf[32];
        if (sim.P.flex_on)
            std::snprintf(buf, sizeof buf, "%s %+.1f%+.2f", fin[i].lbl,
                          deg(sim.s.delta[i]), deg(sim.s.theta[i]));
        else
            std::snprintf(buf, sizeof buf, "%s %+.1f", fin[i].lbl, deg(sim.s.delta[i]));
        text2D(cx + fin[i].ux * (R + 12.0f) - (fin[i].ux < 0 ? 44.0f : 0.0f)
                  - (fin[i].ux == 0 ? 24.0f : 0.0f),
               cy + fin[i].uy * (R + 14.0f) - 4.0f, buf);
    }
    glLineWidth(1.0f);

    // Modo dominante segun las irreps
    double up, uy, ur;
    demix(sim.s.delta, up, uy, ur);
    const char* modo = "neutro";
    double m = std::fmax(std::fabs(up), std::fmax(std::fabs(uy), std::fabs(ur)));
    if (m > 1e-6) {
        if (std::fabs(ur) >= std::fabs(up) && std::fabs(ur) >= std::fabs(uy))
            modo = "ANTISIMETRICO (roll)";
        else if (std::fabs(up) >= std::fabs(uy)) modo = "SIMETRICO h (pitch)";
        else                                     modo = "SIMETRICO v (yaw)";
    }
    glColor3f(0.85f, 0.85f, 0.90f);
    text2D(cx - 100, cy + 104, sim.P.flex_on ? "Vista de cola   servo+torsion"
                                             : "Vista de cola  (Fig. 8-2)");
    glColor3f(0.55f, 0.85f, 1.0f);
    text2D(cx - 100, cy - 100, modo);
}

static void drawHUD(const Sim& sim)
{
    char buf[192];
    double phi, th, psi;
    euler_from_quat(sim.s.q, phi, th, psi);

    double up, uy, ur;
    demix(sim.s.delta, up, uy, ur);

    double f[4], M[3];
    fin_forces(sim.P, sim.s.delta, f);
    fin_moments(sim.P, sim.s.delta, M);

    float y = (float)g_telH - 22.0f;
    const float x = 14.0f;
    const float dy = 15.0f;

    glColor3f(0.95f, 0.95f, 1.00f);
    std::snprintf(buf, sizeof buf, "SIMULACION ALERONES  —  t = %7.2f s   %s",
                  sim.t, sim.paused ? "[PAUSA]" : "");
    text2D(x, y, buf); y -= dy * 1.6f;

    glColor3f(0.60f, 0.90f, 1.00f);
    text2D(x, y, "ACTITUD"); y -= dy;
    glColor3f(0.85f, 0.88f, 0.95f);
    std::snprintf(buf, sizeof buf, "  phi %+8.2f   theta %+8.2f   psi %+8.2f   deg",
                  deg(phi), deg(th), deg(psi));
    text2D(x, y, buf); y -= dy;
    std::snprintf(buf, sizeof buf, "  p   %+8.3f   q     %+8.3f   r   %+8.3f   rad/s",
                  sim.s.omega[0], sim.s.omega[1], sim.s.omega[2]);
    text2D(x, y, buf); y -= dy * 1.6f;

    glColor3f(0.60f, 0.90f, 1.00f);
    text2D(x, y, "DEFLEXIONES  [deg]"); y -= dy;
    glColor3f(0.85f, 0.88f, 0.95f);
    std::snprintf(buf, sizeof buf, "  d1 %+7.2f   d2 %+7.2f   d3 %+7.2f   d4 %+7.2f   (stall +-%.1f)",
                  deg(sim.s.delta[0]), deg(sim.s.delta[1]),
                  deg(sim.s.delta[2]), deg(sim.s.delta[3]), deg(sim.P.delta_stall));
    text2D(x, y, buf); y -= dy;
    std::snprintf(buf, sizeof buf, "  cmd%+7.2f  %+7.2f  %+7.2f  %+7.2f   (servo tau %.3f s, %.0f deg/s)",
                  deg(sim.s.delta_cmd[0]), deg(sim.s.delta_cmd[1]),
                  deg(sim.s.delta_cmd[2]), deg(sim.s.delta_cmd[3]),
                  sim.P.tau_servo, deg(sim.P.rate_limit));
    text2D(x, y, buf); y -= dy;
    std::snprintf(buf, sizeof buf, "  f1 %+7.2f   f2 %+7.2f   f3 %+7.2f   f4 %+7.2f   N",
                  f[0], f[1], f[2], f[3]);
    text2D(x, y, buf); y -= dy;

    // Incidencia efectiva: lo que la aleta ve de verdad cuando el cohete gira.
    double de[4];
    fin_effective_deflection(sim.P, sim.s.delta, sim.s.theta, sim.s.omega, de);
    std::snprintf(buf, sizeof buf, "  eff%+7.2f  %+7.2f  %+7.2f  %+7.2f   %s",
                  deg(de[0]), deg(de[1]), deg(de[2]), deg(de[3]),
                  sim.P.damping_on ? "(amortiguamiento ON)" : "(amortiguamiento OFF)");
    text2D(x, y, buf); y -= dy * 1.6f;

    glColor3f(0.60f, 0.90f, 1.00f);
    text2D(x, y, "CANALES VIRTUALES  (irreps de V4)  [deg]"); y -= dy;
    glColor3f(0.85f, 0.88f, 0.95f);
    std::snprintf(buf, sizeof buf, "  u_pitch %+7.2f   u_yaw %+7.2f   u_roll %+7.2f",
                  deg(up), deg(uy), deg(ur));
    text2D(x, y, buf); y -= dy;
    std::snprintf(buf, sizeof buf, "  L %+8.3f   M %+8.3f   N %+8.3f   N*m",
                  M[0], M[1], M[2]);
    text2D(x, y, buf); y -= dy * 1.6f;

    double L_p, M_q, N_r;
    damping_coeffs(sim.P, L_p, M_q, N_r);
    glColor3f(0.60f, 0.90f, 1.00f);
    text2D(x, y, "AMORTIGUAMIENTO  (matriz diagonal en la base de irreps)"); y -= dy;
    glColor3f(0.85f, 0.88f, 0.95f);
    std::snprintf(buf, sizeof buf, "  L_p %+8.4f   M_q %+8.4f   N_r %+8.4f   N*m/(rad/s)",
                  L_p, M_q, N_r);
    text2D(x, y, buf); y -= dy * 1.6f;

    // ── Aeroelasticidad (§5) ────────────────────────────────────────────────
    {
        const double eta   = aeroelastic_efficiency(sim.P);
        const double q_car = qbar_caracteristica(sim.P);
        const bool   div   = aeroelastic_divergence(sim.P);
        const bool   cfl   = structural_unstable(sim.P);

        if (!sim.P.flex_on)   glColor3f(0.55f, 0.60f, 0.70f);
        else if (div || cfl)  glColor3f(1.00f, 0.35f, 0.30f);
        else                  glColor3f(0.60f, 0.90f, 1.00f);
        std::snprintf(buf, sizeof buf, "ALETA %s  (x%.0f en el dibujo)",
                      sim.P.flex_on ? "FLEXIBLE" : "RIGIDA", flexScale());
        text2D(x, y, buf); y -= dy;

        glColor3f(0.85f, 0.88f, 0.95f);
        if (sim.P.flex_on) {
            std::snprintf(buf, sizeof buf,
                          "  torsion  %+6.3f %+6.3f %+6.3f %+6.3f deg",
                          deg(sim.s.theta[0]), deg(sim.s.theta[1]),
                          deg(sim.s.theta[2]), deg(sim.s.theta[3]));
            text2D(x, y, buf); y -= dy;
            std::snprintf(buf, sizeof buf,
                          "  flecha   %+6.3f %+6.3f %+6.3f %+6.3f mm",
                          1000.0 * sim.s.w[0], 1000.0 * sim.s.w[1],
                          1000.0 * sim.s.w[2], 1000.0 * sim.s.w[3]);
            text2D(x, y, buf); y -= dy;

            if (div) glColor3f(1.00f, 0.35f, 0.30f);
            std::snprintf(buf, sizeof buf,
                          "  eta %.4f   qbar/qbar_car %.3f%s",
                          eta, (q_car > 0.0) ? qbar(sim.P) / q_car : 0.0,
                          div ? "   *** DIVERGENCIA ***" : "");
            text2D(x, y, buf); y -= dy;

            if (cfl) {
                glColor3f(1.00f, 0.35f, 0.30f);
                std::snprintf(buf, sizeof buf,
                              "  *** dt demasiado grande: CFL %.2f > 2 (bajalo o pulsa f) ***",
                              structural_cfl(sim.P));
                text2D(x, y, buf); y -= dy;
            }
        }
        y -= dy * 0.6f;
    }

    if (sim.A.enabled) glColor3f(0.55f, 1.00f, 0.60f);
    else               glColor3f(0.60f, 0.90f, 1.00f);
    std::snprintf(buf, sizeof buf, "AUTOPILOTO  [%s]  (3 lazos SISO, uno por irrep)",
                  sim.A.enabled ? "ON" : "OFF");
    text2D(x, y, buf); y -= dy;
    glColor3f(0.85f, 0.88f, 0.95f);
    std::snprintf(buf, sizeof buf, "  ref  phi %+7.2f   theta %+7.2f   psi %+7.2f   deg",
                  deg(sim.A.phi_ref), deg(sim.A.theta_ref), deg(sim.A.psi_ref));
    text2D(x, y, buf); y -= dy;
    std::snprintf(buf, sizeof buf, "  wn   roll %.2f  pitch %.2f  yaw %.2f rad/s   zeta %.2f",
                  sim.A.wn_roll, sim.A.wn_pitch, sim.A.wn_yaw, sim.A.zeta);
    text2D(x, y, buf); y -= dy * 1.6f;

    glColor3f(0.60f, 0.90f, 1.00f);
    text2D(x, y, "AIRE"); y -= dy;
    glColor3f(0.85f, 0.88f, 0.95f);
    std::snprintf(buf, sizeof buf,
                  "  V %6.1f m/s   qbar %8.1f Pa   CL_delta %.4f 1/rad   S %.4f m2",
                  sim.P.V, qbar(sim.P), sim.P.CL_delta, sim.P.S_fin);
    text2D(x, y, buf); y -= dy;
    std::snprintf(buf, sizeof buf, "  l_t %.3f m   d %.3f m   Ixx %.2f  Iyy %.2f  Izz %.2f",
                  sim.P.l_t, sim.P.d, sim.P.Ixx, sim.P.Iyy, sim.P.Izz);
    text2D(x, y, buf);

    // Ayuda de teclado, abajo a la izquierda
    float yy = 115.0f;
    glColor3f(0.55f, 0.60f, 0.70f);
    if (sim.A.enabled) {
        text2D(x, yy, "flechas arr/aba : theta_ref -/+    Q / E : phi_ref -/+"); yy -= dy;
        text2D(x, yy, "flechas izq/der : psi_ref   -/+    C     : mantener actitud actual"); yy -= dy;
    } else {
        text2D(x, yy, "flechas arr/aba : u_pitch -/+      Q / E : u_roll -/+"); yy -= dy;
        text2D(x, yy, "flechas izq/der : u_yaw   -/+      1..4  : selecciona aleta"); yy -= dy;
    }
    text2D(x, yy, "+ / -           : deflexion directa de la aleta seleccionada"); yy -= dy;
    text2D(x, yy, "A autopiloto   D amortiguamiento   S servo   C hold   0 centrar   R reset"); yy -= dy;
    text2D(x, yy, "f aleta flexible/rigida   F exagerar deformacion   , . velocidad"); yy -= dy;
    text2D(x, yy, "SPACE pausa   T cola   X ejes   W wire   ESC salir"); yy -= dy;
    std::snprintf(buf, sizeof buf, "aleta seleccionada: d%d", g_sim.fin_sel + 1);
    glColor3f(1.0f, 0.95f, 0.30f);
    text2D(x, yy, buf);
}

// ═══════════════════════════════════════════════════════════════════════════
// CALLBACKS DE GLUT
// ═══════════════════════════════════════════════════════════════════════════

static void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glTranslatef(0, 0, g_zoom);
    glRotatef(g_rotX, 1, 0, 0);
    glRotatef(g_rotY, 0, 1, 0);

    drawGrid();

    // Cohete: geometria en ejes cuerpo, llevada a GL por B * R_b^i
    GLfloat m[16];
    bodyToGLMatrix(g_sim.s.q, m);

    glPushMatrix();
    glMultMatrixf(m);

    // El origen del modelo es el CG.  Cola en x = -l_t, nariz en +x.
    const float x_t    = -(float)g_sim.P.l_t;
    const float x_nose =  (float)g_sim.P.l_t * 0.85f;
    const float span   =  (float)g_sim.P.b_fin;

    glPolygonMode(GL_FRONT_AND_BACK, g_wire ? GL_LINE : GL_FILL);

    GLUquadric* q = gluNewQuadric();
    gluQuadricNormals(q, GLU_SMOOTH);
    drawFuselage(q, x_t - 0.05f, x_nose);
    drawFins(g_sim, x_t, span);
    gluDeleteQuadric(q);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    drawFinForces(g_sim, x_t, span);
    if (g_showAxes) drawBodyAxes(0.9f);

    glPopMatrix();

    begin2D(g_winW, g_winH);
    if (g_tailView) drawTailView(g_sim);
    end2D();

    glutSwapBuffers();
}

static void timerStep(int)
{
    if (!g_sim.paused) {
        // ~60 Hz de video, paso fisico fijo P.dt
        const double frame = 1.0 / 60.0;
        int n = (int)std::lround(frame / g_sim.P.dt);
        if (n < 1) n = 1;
        for (int i = 0; i < n; ++i) sim_step(g_sim, g_sim.P.dt);
    } else {
        apply_commands(g_sim.C, g_sim.s);   // refleja los mandos aun en pausa
    }
    glutSetWindow(g_mainWin); glutPostRedisplay();
    glutSetWindow(g_telWin);  glutPostRedisplay();
    glutTimerFunc(16, timerStep, 0);
}

static void reshape(int w, int h)
{
    if (h == 0) h = 1;
    g_winW = w; g_winH = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    gluPerspective(45.0, (double)w / h, 0.05, 200.0);
    glMatrixMode(GL_MODELVIEW);
}

// ── Ventana de telemetria ───────────────────────────────────────────────────
// Solo texto 2D.  No comparte contexto GL con la principal, asi que aqui la
// iluminacion y el z-buffer quedan apagados de forma permanente (si estuvieran
// activos, el color del texto lo decidiria la luz y no glColor).
static void telemetryDisplay()
{
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    gluOrtho2D(0, g_telW, 0, g_telH);
    glMatrixMode(GL_MODELVIEW);  glLoadIdentity();

    drawHUD(g_sim);

    glutSwapBuffers();
}

static void telemetryReshape(int w, int h)
{
    if (h == 0) h = 1;
    g_telW = w; g_telH = h;
    glViewport(0, 0, w, h);
}

static const double CMD_MAX = 3.0;   // multiplos de delta_stall

static void clampCommands()
{
    const double lim = CMD_MAX * g_sim.P.delta_stall;

    // Con el autopiloto activo los canales virtuales los escribe el, ya
    // saturados por PID.u_max; aqui solo se limitan las referencias.
    if (g_sim.A.enabled) {
        const double lref = 80.0 * M_PI / 180.0;   // evita el gimbal lock
        g_sim.A.phi_ref   = clampd(g_sim.A.phi_ref,   -M_PI, M_PI);
        g_sim.A.theta_ref = clampd(g_sim.A.theta_ref, -lref, lref);
        g_sim.A.psi_ref   = clampd(g_sim.A.psi_ref,   -M_PI, M_PI);
    } else {
        g_sim.C.u_pitch = clampd(g_sim.C.u_pitch, -lim, lim);
        g_sim.C.u_yaw   = clampd(g_sim.C.u_yaw,   -lim, lim);
        g_sim.C.u_roll  = clampd(g_sim.C.u_roll,  -lim, lim);
    }
    for (int i = 0; i < 4; ++i)
        g_sim.C.fin_trim[i] = clampd(g_sim.C.fin_trim[i], -lim, lim);
}

// Con el autopiloto activo el piloto mueve REFERENCIAS de actitud; con el
// autopiloto apagado mueve directamente los canales virtuales.  Es la misma
// palanca en las dos capas del lazo.
static bool apOn() { return g_sim.A.enabled; }

static void keyboard(unsigned char k, int, int)
{
    switch (k) {
        case 'q': case 'Q':
            if (apOn()) g_sim.A.phi_ref -= DREF; else g_sim.C.u_roll -= DCMD;
            break;
        case 'e': case 'E':
            if (apOn()) g_sim.A.phi_ref += DREF; else g_sim.C.u_roll += DCMD;
            break;

        case '1': g_sim.fin_sel = 0; break;
        case '2': g_sim.fin_sel = 1; break;
        case '3': g_sim.fin_sel = 2; break;
        case '4': g_sim.fin_sel = 3; break;

        case '+': case '=': g_sim.C.fin_trim[g_sim.fin_sel] += DCMD; break;
        case '-': case '_': g_sim.C.fin_trim[g_sim.fin_sel] -= DCMD; break;

        case '0':   // trim neutro: centra aletas y referencias
            g_sim.C = commands_zero();
            g_sim.A.phi_ref = g_sim.A.theta_ref = g_sim.A.psi_ref = 0.0;
            g_sim.A.roll.i_state = g_sim.A.pitch.i_state = g_sim.A.yaw.i_state = 0.0;
            break;

        case 'a': case 'A':   // autopiloto on/off
            g_sim.A.enabled = !g_sim.A.enabled;
            if (g_sim.A.enabled) {
                // Engancha sin salto: la referencia arranca en la actitud actual.
                euler_from_quat(g_sim.s.q, g_sim.A.phi_ref,
                                g_sim.A.theta_ref, g_sim.A.psi_ref);
                autopilot_tune(g_sim.P, g_sim.A);
            }
            break;

        case 'c': case 'C':   // "hold": congela la actitud de este instante
            euler_from_quat(g_sim.s.q, g_sim.A.phi_ref,
                            g_sim.A.theta_ref, g_sim.A.psi_ref);
            break;

        case 'd': case 'D':   // amortiguamiento aerodinamico on/off (§4)
            g_sim.P.damping_on = !g_sim.P.damping_on;
            autopilot_tune(g_sim.P, g_sim.A);
            break;

        case 'f':             // aleta flexible / rigida (§5)
            g_sim.P.flex_on = !g_sim.P.flex_on;
            if (!g_sim.P.flex_on)         // al volver a rigida, borra el estado
                for (int i = 0; i < 4; ++i) {
                    g_sim.s.theta[i] = g_sim.s.theta_dot[i] = 0.0;
                    g_sim.s.w[i]     = g_sim.s.w_dot[i]     = 0.0;
                }
            autopilot_tune(g_sim.P, g_sim.A);
            break;

        case 'F':             // cicla la exageracion del dibujo
            g_flexScaleIdx = (g_flexScaleIdx + 1) %
                             (int)(sizeof g_flexScales / sizeof g_flexScales[0]);
            break;

        case 's': case 'S':   // dinamica de actuador on/off (§6)
            if (g_sim.P.tau_servo > 0.0 || g_sim.P.rate_limit > 0.0) {
                g_tau_saved  = g_sim.P.tau_servo;
                g_rate_saved = g_sim.P.rate_limit;
                g_sim.P.tau_servo = g_sim.P.rate_limit = 0.0;
            } else {
                g_sim.P.tau_servo  = (g_tau_saved  > 0.0) ? g_tau_saved  : 0.05;
                g_sim.P.rate_limit = (g_rate_saved > 0.0) ? g_rate_saved
                                                          : 240.0 * M_PI / 180.0;
            }
            break;

        case 'r': case 'R':
            sim_reset(g_sim);
            g_sim.P = g_sim0.P;
            break;

        case ' ': g_sim.paused = !g_sim.paused; break;
        case 'w': case 'W': g_wire     = !g_wire;     break;
        case 't': case 'T': g_tailView = !g_tailView; break;
        case 'x': case 'X': g_showAxes = !g_showAxes; break;

        case ',': case '<':
            g_sim.P.V = std::fmax(1.0, g_sim.P.V - DVEL);
            autopilot_tune(g_sim.P, g_sim.A);   // la autoridad va con qbar
            break;
        case '.': case '>':
            g_sim.P.V += DVEL;
            autopilot_tune(g_sim.P, g_sim.A);
            break;

        case 27: std::exit(0);
    }
    clampCommands();
    glutPostRedisplay();
}

static void specialKey(int k, int, int)
{
    switch (k) {
        case GLUT_KEY_UP:
            if (apOn()) g_sim.A.theta_ref += DREF; else g_sim.C.u_pitch += DCMD;
            break;
        case GLUT_KEY_DOWN:
            if (apOn()) g_sim.A.theta_ref -= DREF; else g_sim.C.u_pitch -= DCMD;
            break;
        case GLUT_KEY_LEFT:
            if (apOn()) g_sim.A.psi_ref -= DREF; else g_sim.C.u_yaw -= DCMD;
            break;
        case GLUT_KEY_RIGHT:
            if (apOn()) g_sim.A.psi_ref += DREF; else g_sim.C.u_yaw += DCMD;
            break;
    }
    clampCommands();
    glutPostRedisplay();
}

static void mouseButton(int btn, int state, int x, int y)
{
    if (btn == GLUT_LEFT_BUTTON) { g_drag = (state == GLUT_DOWN); g_lastX = x; g_lastY = y; }
    else if (btn == 3) { g_zoom += 0.35f; glutPostRedisplay(); }   // rueda arriba
    else if (btn == 4) { g_zoom -= 0.35f; glutPostRedisplay(); }   // rueda abajo
}

static void mouseMotion(int x, int y)
{
    if (!g_drag) return;
    g_rotY += (x - g_lastX) * 0.4f;
    g_rotX += (y - g_lastY) * 0.4f;
    g_lastX = x; g_lastY = y;
    glutPostRedisplay();
}

// ═══════════════════════════════════════════════════════════════════════════
void run_gui(int argc, char** argv, const Sim& sim0)
{
    g_sim  = sim0;
    g_sim0 = sim0;

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(g_winW, g_winH);
    glutInitWindowPosition(60, 60);
    g_mainWin = glutCreateWindow("Simulacion Alerones — control cruciforme (Blakelock Fig. 8-2)");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    glShadeModel(GL_SMOOTH);

    GLfloat lpos[] = {6, 10, 8, 0}, lamb[] = {0.30f, 0.30f, 0.34f, 1},
            ldif[] = {0.85f, 0.85f, 0.85f, 1};
    glLightfv(GL_LIGHT0, GL_POSITION, lpos);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  lamb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  ldif);
    glClearColor(0.06f, 0.07f, 0.10f, 1.0f);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKey);
    glutMouseFunc(mouseButton);
    glutMotionFunc(mouseMotion);

    // ── Ventana secundaria: numeros y parametros ────────────────────────────
    glutInitWindowSize(g_telW, g_telH);
    glutInitWindowPosition(60 + g_winW + 10, 60);
    g_telWin = glutCreateWindow("Simulacion Alerones — telemetria");

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glClearColor(0.05f, 0.05f, 0.07f, 1.0f);

    glutDisplayFunc(telemetryDisplay);
    glutReshapeFunc(telemetryReshape);
    // El mismo teclado en las dos ventanas: se pilota desde la que tenga el foco.
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKey);

    glutSetWindow(g_mainWin);
    glutTimerFunc(16, timerStep, 0);

    glutMainLoop();
}

} // namespace alerones
