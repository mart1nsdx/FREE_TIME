// main.cpp — Rocket Simulation: First-Person View + Lagrangian Dynamics
// Build:  cmake -B build && cmake --build build
// Run:    ./build/rocket_sim
//
// Controls:
//   SPACE        launch           R            reset
//   1-7          select param     +/-          adjust value
//   WASD         move camera      Arrow keys   look around
//   Left drag    look around      T            toggle track-rocket cam
//   ESC          quit

#ifdef __APPLE__
  #include <GLUT/glut.h>
  #include <OpenGL/gl.h>
  #include <OpenGL/glu.h>
#else
  #include <GL/glut.h>
  #include <GL/gl.h>
  #include <GL/glu.h>
#endif

#include "physics.h"
#include <vector>

// physics: x=East, y=North, z=Up
// render:  x=East, y=Up,    z=North  (OpenGL Y-up convention)
static Vec3 physToRender(const Vec3& v) { return Vec3(v.x, v.z, v.y); }
#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <cstdlib>

// ─────────────────────────────────────────────────────────────────
// Global state
// ─────────────────────────────────────────────────────────────────
static int WIN_W = 1280, WIN_H = 800;

// Launch pad position in world space
static const float PAD_X = 0.f, PAD_Y = 0.f, PAD_Z = 80.f;

// Camera
static float camX = -20.f, camY = 2.f, camZ = 5.f;
static float camYaw   = 0.30f;    // horizontal angle (radians)
static float camPitch = 0.05f;    // vertical angle
static bool  keys[256];

// Mouse look
static int  mLastX = -1, mLastY = -1;
static bool mDown  = false;

// Simulation
static LaunchParams  params;
static RocketState   rocket;
static std::vector<Vec3> trail;   // world-space trail positions
static bool simRunning  = false;
static bool simFinished = false;
static bool trackCam    = true;   // auto-aim at rocket when flying

// UI
static int  selParam  = 0;        // selected parameter index [0..7]
static bool editMode  = false;    // typing a value directly
static char editBuf[32] = "";
static int  editLen   = 0;

// ─────────────────────────────────────────────────────────────────
// Camera
// ─────────────────────────────────────────────────────────────────
static void applyCamera() {
    float lx = sinf(camYaw) * cosf(camPitch);
    float ly = sinf(camPitch);
    float lz = cosf(camYaw) * cosf(camPitch);
    gluLookAt(camX, camY, camZ,
              camX+lx, camY+ly, camZ+lz,
              0.f, 1.f, 0.f);
}

static void moveCamera(float dt) {
    float spd = 25.f * dt;
    float fx = sinf(camYaw), fz = cosf(camYaw);   // forward (XZ plane)
    if (keys['w']||keys['W']) { camX += fx*spd; camZ += fz*spd; }
    if (keys['s']||keys['S']) { camX -= fx*spd; camZ -= fz*spd; }
    if (keys['a']||keys['A']) { camX -= fz*spd; camZ += fx*spd; }
    if (keys['d']||keys['D']) { camX += fz*spd; camZ -= fx*spd; }
}

static void aimAtRocket() {
    Vec3  rpr = physToRender(rocket.pos);
    float rx = rpr.x + PAD_X;
    float ry = rpr.y + PAD_Y;
    float rz = rpr.z + PAD_Z;
    float dx = rx - camX, dy = ry - camY, dz = rz - camZ;
    camYaw   = atan2f(dx, dz);
    camPitch = atan2f(dy, sqrtf(dx*dx + dz*dz));
}

// ─────────────────────────────────────────────────────────────────
// 2D overlay helpers
// ─────────────────────────────────────────────────────────────────
static void enter2D() {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix(); glLoadIdentity();
    gluOrtho2D(0, WIN_W, 0, WIN_H);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix(); glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_FOG);
}

static void exit2D() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_FOG);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

static void drawText(float x, float y, const char* s, void* font = GLUT_BITMAP_8_BY_13) {
    glRasterPos2f(x, y);
    for (; *s; ++s) glutBitmapCharacter(font, *s);
}

static void filledRect(float x, float y, float w, float h, float r, float g, float b, float a) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(r, g, b, a);
    glBegin(GL_QUADS);
    glVertex2f(x,   y);   glVertex2f(x+w, y);
    glVertex2f(x+w, y-h); glVertex2f(x,   y-h);
    glEnd();
    glDisable(GL_BLEND);
}

static void outlineRect(float x, float y, float w, float h, float r, float g, float b) {
    glColor3f(r, g, b);
    glLineWidth(1.5f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(x,   y);   glVertex2f(x+w, y);
    glVertex2f(x+w, y-h); glVertex2f(x,   y-h);
    glEnd();
}

// ─────────────────────────────────────────────────────────────────
// 3D scene
// ─────────────────────────────────────────────────────────────────
static void drawGround() {
    // Solid terrain plane
    glColor3f(0.14f, 0.28f, 0.10f);
    glBegin(GL_QUADS);
    glVertex3f(-4000, 0, -4000); glVertex3f( 4000, 0, -4000);
    glVertex3f( 4000, 0,  4000); glVertex3f(-4000, 0,  4000);
    glEnd();

    // Metric grid (every 20 m, ±500 m)
    glColor3f(0.22f, 0.42f, 0.16f);
    glLineWidth(1.f);
    glBegin(GL_LINES);
    for (int i = -25; i <= 25; i++) {
        float f = i * 20.f;
        glVertex3f(f,  0.08f, -500); glVertex3f(f,  0.08f,  500);
        glVertex3f(-500, 0.08f, f); glVertex3f(500, 0.08f, f);
    }
    glEnd();
}

static void drawLaunchPad() {
    glPushMatrix();
    glTranslatef(PAD_X, PAD_Y, PAD_Z);

    // Concrete apron
    glColor3f(0.42f, 0.40f, 0.37f);
    glBegin(GL_QUADS);
    glVertex3f(-5, 0.06f,  5); glVertex3f( 5, 0.06f,  5);
    glVertex3f( 5, 0.06f, -5); glVertex3f(-5, 0.06f, -5);
    glEnd();

    // Support truss (4 corner pillars + top frame)
    glColor3f(0.52f, 0.54f, 0.58f);
    glLineWidth(3.f);
    float c[4][2] = {{-2,-2},{2,-2},{2,2},{-2,2}};
    glBegin(GL_LINES);
    for (int i = 0; i < 4; i++) {
        glVertex3f(c[i][0], 0.1f, c[i][1]);
        glVertex3f(c[i][0], 9.f,  c[i][1]);
    }
    for (int i = 0; i < 4; i++) {
        int j = (i+1)%4;
        glVertex3f(c[i][0], 9.f, c[i][1]);
        glVertex3f(c[j][0], 9.f, c[j][1]);
    }
    // Diagonal braces
    for (int i = 0; i < 4; i++) {
        int j = (i+1)%4;
        glVertex3f(c[i][0], 0.1f, c[i][1]);
        glVertex3f(c[j][0], 5.f,  c[j][1]);
    }
    glEnd();

    // Rocket on pad (pre-launch visualization)
    if (!simRunning && !simFinished) {
        glColor3f(0.92f, 0.92f, 0.90f);
        glLineWidth(5.f);
        glBegin(GL_LINES);
        glVertex3f(0, 0.2f, 0); glVertex3f(0, 7.5f, 0);
        glEnd();
        // Nose cone tip
        glColor3f(0.85f, 0.18f, 0.18f);
        glPointSize(7.f);
        glBegin(GL_POINTS);
        glVertex3f(0, 8.2f, 0);
        glEnd();
        // Fins
        glColor3f(0.7f, 0.7f, 0.75f);
        glLineWidth(2.f);
        float fins[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
        glBegin(GL_LINES);
        for (int i = 0; i < 4; i++) {
            glVertex3f(0, 0.5f, 0);
            glVertex3f(fins[i][0]*2.f, 0.5f, fins[i][1]*2.f);
            glVertex3f(fins[i][0]*2.f, 0.5f, fins[i][1]*2.f);
            glVertex3f(0, 2.5f, 0);
        }
        glEnd();
    }

    glPopMatrix();
}

static void drawTrailAndRocket() {
    // Trail as colored line strip
    if (trail.size() >= 2) {
        glLineWidth(3.f);
        glBegin(GL_LINE_STRIP);
        for (size_t i = 0; i < trail.size(); i++) {
            float t = (float)i / (float)trail.size();
            // Yellow (ignition) → orange → red (burnout) → dim (descent)
            float r = 1.f;
            float g = (t < 0.45f) ? 1.f - t * 1.6f : 0.f;
            float b = (t > 0.75f) ? (t - 0.75f) * 1.6f * 0.4f : 0.f;
            glColor3f(r, g, b);
            Vec3 rp = physToRender(trail[i]);
            glVertex3f(rp.x + PAD_X, rp.y + PAD_Y, rp.z + PAD_Z);
        }
        glEnd();
    }

    if (!simRunning && !simFinished) return;

    Vec3  rp  = physToRender(rocket.pos);
    float rx  = rp.x + PAD_X;
    float ry  = rp.y + PAD_Y;
    float rz  = rp.z + PAD_Z;

    // Rocket body (bright point)
    glPointSize(12.f);
    glColor3f(1.f, 1.f, 1.f);
    glBegin(GL_POINTS);
    glVertex3f(rx, ry, rz);
    glEnd();

    // Engine plume during burn phase
    float dry = params.total_mass_kg - params.fuel_mass_kg;
    if (simRunning && rocket.t < params.burn_time_s && rocket.mass > dry + 0.1f) {
        Vec3 td = physToRender(thrustDir(params));
        float flicker = 6.f + 4.f * ((float)(rand() % 100) / 100.f);

        // Outer plume (orange)
        glLineWidth(6.f);
        glColor3f(1.f, 0.55f, 0.05f);
        glBegin(GL_LINES);
        glVertex3f(rx, ry, rz);
        glVertex3f(rx - td.x*flicker, ry - td.y*flicker, rz - td.z*flicker);
        glEnd();

        // Inner core (bright white-yellow)
        glLineWidth(2.5f);
        glColor3f(1.f, 1.f, 0.75f);
        glBegin(GL_LINES);
        glVertex3f(rx, ry, rz);
        glVertex3f(rx - td.x*flicker*0.45f, ry - td.y*flicker*0.45f, rz - td.z*flicker*0.45f);
        glEnd();
    }
}

// ─────────────────────────────────────────────────────────────────
// HUD
// ─────────────────────────────────────────────────────────────────

// Parameter table — mirrors the LaunchParams fields in order
struct PEntry {
    const char* label;
    float*      val;
    float       step;
    const char* unit;
    float       lo, hi;
};

static void drawHUD() {
    enter2D();

    // ── Parameter panel (top-left) — 8 configurable entries ────
    float px = 10, py = (float)WIN_H - 10, pw = 318, ph = 300;
    filledRect(px, py, pw, ph, 0.f, 0.02f, 0.07f, 0.82f);
    outlineRect(px, py, pw, ph, 0.f, 0.80f, 1.f);

    glColor3f(0.f, 1.f, 1.f);
    drawText(px+8, py-22, "=== LAUNCH PARAMETERS ===", GLUT_BITMAP_9_BY_15);

    PEntry entries[8] = {
        {"[1] Theta th",   &params.theta_deg,     5.f,    "deg",  1.f,   179.f },
        {"[2] Phi ph",     &params.phi_deg,       10.f,   "deg",  0.f,   359.f },
        {"[3] Isp",        &params.Isp_s,         10.f,   "s",    100.f, 500.f },
        {"[4] Burn Time",  &params.burn_time_s,   1.f,    "s",    1.f,   120.f },
        {"[5] Total Mass", &params.total_mass_kg, 10.f,   "kg",   10.f,  5000.f},
        {"[6] Fuel Mass",  &params.fuel_mass_kg,  5.f,    "kg",   1.f,   4990.f},
        {"[7] Drag Cd",    &params.Cd,            0.05f,  "",     0.05f, 2.f   },
        {"[8] Area",       &params.area_m2,       0.005f, "m2",   0.005f,0.5f  },
    };

    for (int i = 0; i < 8; i++) {
        float ly = py - 48 - i * 27;

        if (i == selParam) {
            float hr = editMode ? 0.15f : 0.f;
            float hg = editMode ? 0.40f : 0.55f;
            float hb = editMode ? 0.f   : 1.f;
            filledRect(px+4, ly+17, pw-8, 24, hr, hg, hb, 0.30f);
            glColor3f(editMode ? 1.f : 0.1f,
                      editMode ? 1.f : 1.f,
                      editMode ? 0.2f: 1.f);
        } else {
            glColor3f(0.76f, 0.76f, 0.76f);
        }

        char buf[64];
        if (i == selParam && editMode) {
            bool cur = (glutGet(GLUT_ELAPSED_TIME) % 700) < 350;
            char tmp[24];
            snprintf(tmp, sizeof(tmp), "%s%s", editBuf, cur ? "_" : " ");
            snprintf(buf, sizeof(buf), "%-15s [%8s] %s",
                     entries[i].label, tmp, entries[i].unit);
        } else {
            snprintf(buf, sizeof(buf), "%-15s %9.3f %s",
                     entries[i].label, *entries[i].val, entries[i].unit);
        }
        drawText(px+12, ly, buf, GLUT_BITMAP_8_BY_13);
    }

    float fy = py - ph + 52;
    if (editMode) {
        glColor3f(1.f, 1.f, 0.25f);
        drawText(px+10, fy,    "Escribe el valor  ENTER: confirmar");
        glColor3f(0.55f, 0.55f, 0.55f);
        drawText(px+10, fy-16, "ESC: cancelar edicion");
        drawText(px+10, fy-32, "+/-: ajuste fino");
    } else {
        glColor3f(0.50f, 0.50f, 0.50f);
        drawText(px+10, fy,    "ENTER: editar valor  +/-: ajuste fino");
        drawText(px+10, fy-16, "SPACE: launch   R: reset");
        drawText(px+10, fy-32, "WASD: move   Arrows/drag: look   T: track");
    }

    // ── Propulsion derived values panel ────────────────────────
    float dp_y = py - ph - 8.f;   // top of derived panel
    float dp_h = 116.f;
    filledRect(px, dp_y, pw, dp_h, 0.f, 0.07f, 0.02f, 0.82f);
    outlineRect(px, dp_y, pw, dp_h, 1.f, 0.65f, 0.f);

    glColor3f(1.f, 0.75f, 0.f);
    drawText(px+8, dp_y-20, "=== PROPULSION (derived) ===", GLUT_BITMAP_9_BY_15);

    char dbuf[64];
    float tw_ratio = params.thrustToWeight();
    float mr       = params.massRatio();

    // Thrust — blanco
    glColor3f(0.90f, 0.90f, 0.90f);
    snprintf(dbuf,sizeof(dbuf),"Thrust       %8.1f N", params.thrustN());
    drawText(px+12, dp_y-40, dbuf, GLUT_BITMAP_8_BY_13);

    // T/W — rojo <1, amarillo 1-1.5, verde >1.5
    if      (tw_ratio < 1.0f) glColor3f(1.f, 0.20f, 0.20f);
    else if (tw_ratio < 1.5f) glColor3f(1.f, 0.85f, 0.10f);
    else                      glColor3f(0.20f, 1.f, 0.40f);
    snprintf(dbuf,sizeof(dbuf),"Thrust/Weight%8.2f %s", tw_ratio,
             tw_ratio < 1.f ? "<- NO DESPEGA" : tw_ratio < 1.5f ? "<- MARGINAL" : "<- OK");
    drawText(px+12, dp_y-56, dbuf, GLUT_BITMAP_8_BY_13);

    // Mass Ratio — rojo <2, amarillo 2-4, verde >4
    if      (mr < 2.0f) glColor3f(1.f, 0.45f, 0.20f);
    else if (mr < 4.0f) glColor3f(1.f, 0.85f, 0.10f);
    else                glColor3f(0.20f, 1.f, 0.40f);
    snprintf(dbuf,sizeof(dbuf),"Mass Ratio   %8.2f", mr);
    drawText(px+12, dp_y-72, dbuf, GLUT_BITMAP_8_BY_13);

    // Delta-V — blanco
    glColor3f(0.90f, 0.90f, 0.90f);
    snprintf(dbuf,sizeof(dbuf),"Delta-V ideal%8.1f m/s", params.deltaV());
    drawText(px+12, dp_y-88, dbuf, GLUT_BITMAP_8_BY_13);

    glColor3f(0.45f, 0.65f, 0.45f);
    drawText(px+12, dp_y-104, "Atm: ISA rho(z)=1.225*e^(-z/8500)", GLUT_BITMAP_8_BY_13);

    // ── Advertencia central si T/W < 1 ─────────────────────────
    if (!simRunning && !simFinished && tw_ratio < 1.f) {
        float wx = WIN_W * 0.5f - 140.f, wy = WIN_H * 0.5f - 40.f;
        filledRect(wx, wy+28, 280, 44, 0.5f, 0.f, 0.f, 0.82f);
        outlineRect(wx, wy+28, 280, 44, 1.f, 0.f, 0.f);
        glColor3f(1.f, 0.25f, 0.25f);
        drawText(wx+10, wy+14, "EMPUJE INSUFICIENTE  T/W < 1", GLUT_BITMAP_9_BY_15);
        glColor3f(1.f, 0.70f, 0.f);
        snprintf(dbuf,sizeof(dbuf),"Necesitas Thrust > %.0f N", params.total_mass_kg * GRAVITY);
        drawText(wx+18, wy-2, dbuf, GLUT_BITMAP_8_BY_13);
    }

    // ── Telemetry panel (top-right) ─────────────────────────────
    if (simRunning || simFinished) {
        float tx = (float)WIN_W - 225, ty = (float)WIN_H - 10, tw = 215, th = 152;
        filledRect(tx, ty, tw, th, 0.f, 0.02f, 0.f, 0.80f);
        outlineRect(tx, ty, tw, th, 1.f, 0.60f, 0.f);

        glColor3f(1.f, 0.82f, 0.f);
        drawText(tx+8, ty-22, "=== TELEMETRY ===", GLUT_BITMAP_9_BY_15);

        char buf[64];
        float spd   = rocket.vel.mag();
        float alt   = rocket.pos.z;                                          // z=altitud
        float range = sqrtf(rocket.pos.x*rocket.pos.x + rocket.pos.y*rocket.pos.y); // plano XY
        float dry   = params.total_mass_kg - params.fuel_mass_kg;

        glColor3f(0.88f, 0.88f, 0.88f);
        snprintf(buf,sizeof(buf),"T+ %7.1f s",     rocket.t);  drawText(tx+10, ty-44, buf);
        snprintf(buf,sizeof(buf),"Alt   %7.1f m",  alt);       drawText(tx+10, ty-60, buf);
        snprintf(buf,sizeof(buf),"Speed %7.1f m/s",spd);       drawText(tx+10, ty-76, buf);
        snprintf(buf,sizeof(buf),"Range %7.1f m",  range);     drawText(tx+10, ty-92, buf);
        snprintf(buf,sizeof(buf),"Mass  %7.1f kg", rocket.mass);drawText(tx+10, ty-108,buf);

        if (simFinished) {
            glColor3f(1.f, 0.22f, 0.22f);
            drawText(tx+10, ty-130, "** IMPACT **", GLUT_BITMAP_9_BY_15);
        } else if (rocket.t < params.burn_time_s && rocket.mass > dry + 0.05f) {
            glColor3f(1.f, 0.72f, 0.f);
            drawText(tx+10, ty-130, "BURN PHASE", GLUT_BITMAP_9_BY_15);
        } else {
            glColor3f(0.45f, 0.85f, 1.f);
            drawText(tx+10, ty-130, "COAST PHASE", GLUT_BITMAP_9_BY_15);
        }
    }

    // ── Center crosshair ────────────────────────────────────────
    float cx = WIN_W * 0.5f, cy = WIN_H * 0.5f;
    glColor3f(1.f, 1.f, 1.f);
    glLineWidth(1.f);
    glBegin(GL_LINES);
    glVertex2f(cx-14, cy); glVertex2f(cx+14, cy);
    glVertex2f(cx, cy-14); glVertex2f(cx, cy+14);
    glEnd();
    glPointSize(2.f);
    glBegin(GL_POINTS);
    glVertex2f(cx, cy);
    glEnd();

    // Track indicator
    if (trackCam && (simRunning || simFinished)) {
        glColor3f(0.f, 1.f, 0.5f);
        drawText((float)WIN_W - 105, 22, "[T] TRACKING");
    }

    // Launch prompt (when idle)
    if (!simRunning && !simFinished) {
        glColor3f(1.f, 1.f, 0.6f);
        drawText((float)WIN_W*0.5f - 80, 30, "SPACE to launch", GLUT_BITMAP_9_BY_15);
    }

    exit2D();
}

// ─────────────────────────────────────────────────────────────────
// Simulation control
// ─────────────────────────────────────────────────────────────────
static void launchRocket() {
    if (simRunning) return;
    params.fuel_mass_kg = std::min(params.fuel_mass_kg, params.total_mass_kg * 0.90f);

    rocket      = RocketState();
    rocket.pos  = Vec3(0.f, 0.f, 0.f);
    rocket.vel  = Vec3(0.f, 0.f, 0.2f);  // tiny initial nudge vertical (z=arriba)
    rocket.mass = params.total_mass_kg;
    rocket.t    = 0.f;

    trail.clear();
    trail.push_back(rocket.pos);

    simRunning  = true;
    simFinished = false;
    trackCam    = true;
}

static void resetSim() {
    simRunning  = false;
    simFinished = false;
    rocket      = RocketState();
    trail.clear();
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// ─────────────────────────────────────────────────────────────────
// GLUT callbacks
// ─────────────────────────────────────────────────────────────────
static void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(75.0, (double)WIN_W / WIN_H, 0.3, 12000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    applyCamera();

    drawGround();
    drawLaunchPad();
    drawTrailAndRocket();
    drawHUD();

    glutSwapBuffers();
}

static void reshape(int w, int h) {
    WIN_W = w; WIN_H = h;
    glViewport(0, 0, w, h);
}

static void timerCB(int) {
    static int prevMs = 0;
    int   nowMs = glutGet(GLUT_ELAPSED_TIME);
    float dt    = (nowMs - prevMs) / 1000.f;
    prevMs = nowMs;
    if (dt <= 0.f || dt > 0.1f) dt = 0.016f;

    moveCamera(dt);

    if (simRunning) {
        int   substeps = 4;
        float subDt    = dt / substeps;

        for (int i = 0; i < substeps && simRunning; i++) {
            rocket = rk4Step(rocket, params, subDt);

            // Append trail point when rocket has moved enough
            float dist = trail.empty() ? 999.f : (rocket.pos - trail.back()).mag();
            if (dist > 2.f) trail.push_back(rocket.pos);

            // Ground impact
            if (rocket.pos.z < 0.f) {
                rocket.pos.z = 0.f;
                simRunning   = false;
                simFinished  = true;
            }
        }

        if (trackCam) aimAtRocket();
    }

    glutPostRedisplay();
    glutTimerFunc(16, timerCB, 0);
}

static void keyDown(unsigned char key, int, int) {
    keys[(unsigned char)key] = true;

    // Tabla de parámetros — usada tanto en edición como en +/-
    float* vals[] = {
        &params.theta_deg, &params.phi_deg, &params.Isp_s,
        &params.burn_time_s,   &params.total_mass_kg, &params.fuel_mass_kg,
        &params.Cd, &params.area_m2
    };
    float steps[] = {  5.f, 10.f, 10.f,  1.f,  10.f,  5.f,  0.05f, 0.005f };
    float  los[]  = {  1.f,  0.f,100.f,  1.f,  10.f,  1.f,  0.05f, 0.005f };
    float  his[]  = {179.f,359.f,500.f,120.f,5000.f,4990.f,  2.f,   0.5f  };

    // ESC: cancela edición primero, luego sale
    if (key == 27) {
        if (editMode) { editMode = false; editLen = 0; editBuf[0] = '\0'; return; }
        exit(0);
    }

    // ── Modo edición directa ──────────────────────────────────
    if (editMode) {
        if ((key >= '0' && key <= '9') || (key == '.' && editLen < 10)) {
            if (editLen < 11) { editBuf[editLen++] = (char)key; editBuf[editLen] = '\0'; }
        } else if (key == 8 || key == 127) {          // Backspace
            if (editLen > 0) editBuf[--editLen] = '\0';
        } else if (key == 13 || key == '\r') {        // Enter = confirmar
            if (editLen > 0) {
                float v = (float)atof(editBuf);
                *vals[selParam] = clampf(v, los[selParam], his[selParam]);
            }
            editMode = false; editLen = 0; editBuf[0] = '\0';
        }
        return;  // bloquea todo lo demás mientras se edita
    }

    // ── Modo normal ───────────────────────────────────────────
    if (key == ' ')            { launchRocket(); return; }
    if (key == 'r' || key=='R'){ resetSim();     return; }
    if (key == 't' || key=='T'){ trackCam = !trackCam; return; }

    // Enter: entrar en modo edición del parámetro seleccionado
    if ((key == 13 || key == '\r') && !simRunning) {
        editMode = true; editLen = 0; editBuf[0] = '\0';
        return;
    }

    if (key >= '1' && key <= '8') { selParam = key - '1'; return; }

    if (simRunning) return;   // no se ajustan parámetros durante el vuelo

    if (key == '+' || key == '=') {
        *vals[selParam] = clampf(*vals[selParam] + steps[selParam], los[selParam], his[selParam]);
    }
    if (key == '-') {
        *vals[selParam] = clampf(*vals[selParam] - steps[selParam], los[selParam], his[selParam]);
    }
}

static void keyUp(unsigned char key, int, int) {
    keys[(unsigned char)key] = false;
}

static void specialKey(int key, int, int) {
    const float ROT = 0.04f;
    switch (key) {
        case GLUT_KEY_LEFT:  camYaw   -= ROT; trackCam = false; break;
        case GLUT_KEY_RIGHT: camYaw   += ROT; trackCam = false; break;
        case GLUT_KEY_UP:    camPitch  = std::min(camPitch+ROT,  1.55f); trackCam = false; break;
        case GLUT_KEY_DOWN:  camPitch  = std::max(camPitch-ROT, -1.55f); trackCam = false; break;
    }
}

static void mouseBtn(int btn, int state, int x, int y) {
    if (btn == GLUT_LEFT_BUTTON) {
        mDown  = (state == GLUT_DOWN);
        mLastX = x; mLastY = y;
        if (mDown) trackCam = false;
    }
}

static void mouseMove(int x, int y) {
    if (mDown && mLastX >= 0) {
        camYaw   += (x - mLastX) * 0.005f;
        camPitch -= (y - mLastY) * 0.005f;
        camPitch  = std::max(-1.55f, std::min(1.55f, camPitch));
    }
    mLastX = x; mLastY = y;
}

// ─────────────────────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(WIN_W, WIN_H);
    glutCreateWindow("Rocket Simulation — Lagrangian Dynamics | First Person");

    memset(keys, 0, sizeof(keys));

    // Night sky / twilight background
    glClearColor(0.06f, 0.10f, 0.24f, 1.f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

    // Atmospheric depth fog (matches sky color)
    glEnable(GL_FOG);
    glFogi(GL_FOG_MODE, GL_LINEAR);
    float fogCol[] = {0.06f, 0.10f, 0.24f, 1.f};
    glFogfv(GL_FOG_COLOR, fogCol);
    glFogf(GL_FOG_START, 1200.f);
    glFogf(GL_FOG_END,   9000.f);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyDown);
    glutKeyboardUpFunc(keyUp);
    glutSpecialFunc(specialKey);
    glutMouseFunc(mouseBtn);
    glutMotionFunc(mouseMove);
    glutTimerFunc(16, timerCB, 0);

    printf("╔══════════════════════════════════════════════════╗\n");
    printf("║   ROCKET SIMULATION — LAGRANGIAN DYNAMICS        ║\n");
    printf("╠══════════════════════════════════════════════════╣\n");
    printf("║  Physics: Euler-Lagrange + RK4 integration       ║\n");
    printf("║  L = T − V = ½m(ẋ²+ẏ²+ż²) − mgz               ║\n");
    printf("║  Forces: gravity + thrust + aero drag            ║\n");
    printf("╠══════════════════════════════════════════════════╣\n");
    printf("║  SPACE: launch   R: reset   T: track cam         ║\n");
    printf("║  1-7: select param   +/-: adjust value           ║\n");
    printf("║  WASD: move   Arrows/drag: look   ESC: quit      ║\n");
    printf("╚══════════════════════════════════════════════════╝\n\n");

    glutMainLoop();
    return 0;
}
