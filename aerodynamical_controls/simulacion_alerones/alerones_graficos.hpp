// ═══════════════════════════════════════════════════════════════════════════
//  alerones_graficos.hpp — render 3D + main loop de la simulacion.
//
//  Backend: GLUT / OpenGL (frameworks nativos de macOS, mismo stack que
//  unmanned/drone_model.cpp).  Toda la fisica vive en fisica_alerones.*;
//  este modulo solo dibuja y traduce teclado -> Commands.
// ═══════════════════════════════════════════════════════════════════════════
#ifndef ALERONES_GRAFICOS_HPP
#define ALERONES_GRAFICOS_HPP

#include "fisica_alerones.hpp"

namespace alerones {

// Estado completo de la simulacion interactiva.
struct Sim {
    Params    P;
    State     s;
    Commands  C;
    Autopilot A;

    double t;            // tiempo simulado [s]
    bool   paused;
    int    fin_sel;      // aleta seleccionada por las teclas 1..4 (0..3)
};

Sim sim_init();
void sim_reset(Sim& sim);

// Un paso completo, en este orden:
//   1. autopiloto  -> comandos virtuales (si esta activo)
//   2. mixing      -> delta_cmd
//   3. actuador    -> delta
//   4. RK4         -> actitud
void sim_step(Sim& sim, double dt);

// Abre la ventana y entra en el bucle de GLUT (no retorna).
void run_gui(int argc, char** argv, const Sim& sim0);

} // namespace alerones

#endif // ALERONES_GRAFICOS_HPP
