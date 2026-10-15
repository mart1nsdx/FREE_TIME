// ═══════════════════════════════════════════════════════════════════════════
//  main.cpp — instancia el estado y arranca el bucle.
//
//      ./alerones              modo grafico (GLUT)
//      ./alerones --headless   REPL por stdin, para scripting y regresion
//
//  El modo headless acepta comentarios con '#', asi que se le puede dar de
//  comer cualquier fichero de scripts/:
//
//      ./alerones --headless < scripts/amortiguamiento.txt
// ═══════════════════════════════════════════════════════════════════════════
#include "alerones_graficos.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

using namespace alerones;

static double deg(double r) { return r * 180.0 / M_PI; }
static double rad(double d) { return d * M_PI / 180.0; }

// Solo para imprimir.  Las combinaciones antisimetricas (u_roll, L) son la
// cancelacion total de cuatro numeros casi iguales: al converger el lazo
// quedan en ~1e-31, veinte y pico ordenes por debajo de cualquier escala
// fisica del problema, y su SIGNO es ruido de redondeo.  Sin esto, la traza
// de C++ y la de Python difieren en un "-0.00000" que no significa nada.
static double z(double v) { return (std::fabs(v) < 1e-9) ? 0.0 : v; }

// ─────────────────────────────────────────────────────────────────────────────
// MODO HEADLESS
// ─────────────────────────────────────────────────────────────────────────────

static bool parseOnOff(std::istringstream& in, bool& out)
{
    std::string v;
    if (!(in >> v)) return false;
    if (v == "on"  || v == "1" || v == "si") { out = true;  return true; }
    if (v == "off" || v == "0" || v == "no") { out = false; return true; }
    return false;
}

static void printState(const Sim& sim)
{
    double phi, th, psi;
    euler_from_quat(sim.s.q, phi, th, psi);

    double up, uy, ur;
    demix(sim.s.delta, up, uy, ur);

    double f[4], M[3], de[4];
    fin_forces(sim.P, sim.s.delta, f);
    fin_moments_rate(sim.P, sim.s.delta, sim.s.theta, sim.s.omega, M);
    fin_effective_deflection(sim.P, sim.s.delta, sim.s.theta, sim.s.omega, de);

    std::printf("t      %.4f s\n", sim.t);
    std::printf("euler  phi %+10.5f  theta %+10.5f  psi %+10.5f  deg\n",
                deg(phi), deg(th), deg(psi));
    std::printf("rates  p   %+10.6f  q     %+10.6f  r   %+10.6f  rad/s\n",
                z(sim.s.omega[0]), z(sim.s.omega[1]), z(sim.s.omega[2]));
    std::printf("quat   w %+.9f  x %+.9f  y %+.9f  z %+.9f  |e| %.12f\n",
                sim.s.q[0], sim.s.q[1], sim.s.q[2], sim.s.q[3],
                std::sqrt(sim.s.q[0]*sim.s.q[0] + sim.s.q[1]*sim.s.q[1] +
                          sim.s.q[2]*sim.s.q[2] + sim.s.q[3]*sim.s.q[3]));
    std::printf("cmd    d1 %+10.5f  d2 %+10.5f  d3 %+10.5f  d4 %+10.5f  deg\n",
                z(deg(sim.s.delta_cmd[0])), z(deg(sim.s.delta_cmd[1])),
                z(deg(sim.s.delta_cmd[2])), z(deg(sim.s.delta_cmd[3])));
    std::printf("delta  d1 %+10.5f  d2 %+10.5f  d3 %+10.5f  d4 %+10.5f  deg\n",
                z(deg(sim.s.delta[0])), z(deg(sim.s.delta[1])),
                z(deg(sim.s.delta[2])), z(deg(sim.s.delta[3])));
    std::printf("eff    d1 %+10.5f  d2 %+10.5f  d3 %+10.5f  d4 %+10.5f  deg\n",
                z(deg(de[0])), z(deg(de[1])), z(deg(de[2])), z(deg(de[3])));
    std::printf("virt   up %+10.5f  uy %+10.5f  ur %+10.5f  deg\n",
                z(deg(up)), z(deg(uy)), z(deg(ur)));
    std::printf("force  f1 %+10.5f  f2 %+10.5f  f3 %+10.5f  f4 %+10.5f  N\n",
                z(f[0]), z(f[1]), z(f[2]), z(f[3]));
    std::printf("moment L  %+10.5f  M  %+10.5f  N  %+10.5f  N*m\n",
                z(M[0]), z(M[1]), z(M[2]));
    // 4 decimales, no 3: qbar = rho*V^2/2 cae en empates exactos (x.xxx5) y
    // C++ y Python los rompen en direcciones distintas.
    std::printf("air    V %.3f m/s   qbar %.4f Pa   CL_delta %.6f 1/rad\n",
                sim.P.V, qbar(sim.P), sim.P.CL_delta);
    if (sim.P.flex_on) {
        std::printf("torsion t1 %+10.5f  t2 %+10.5f  t3 %+10.5f  t4 %+10.5f  deg\n",
                    z(deg(sim.s.theta[0])), z(deg(sim.s.theta[1])),
                    z(deg(sim.s.theta[2])), z(deg(sim.s.theta[3])));
        std::printf("flecha w1 %+10.5f  w2 %+10.5f  w3 %+10.5f  w4 %+10.5f  mm\n",
                    z(1000.0 * sim.s.w[0]), z(1000.0 * sim.s.w[1]),
                    z(1000.0 * sim.s.w[2]), z(1000.0 * sim.s.w[3]));
        std::printf("flex   eta %.6f   qbar/qbar_car %.5f%s\n",
                    aeroelastic_efficiency(sim.P),
                    (qbar_caracteristica(sim.P) > 0.0)
                        ? qbar(sim.P) / qbar_caracteristica(sim.P) : 0.0,
                    aeroelastic_divergence(sim.P) ? "   DIVERGIDA" : "");
    }
    std::printf("modo   amortiguamiento %s   servo tau %.4f s / %.1f deg/s   AP %s   aleta %s\n",
                sim.P.damping_on ? "ON" : "OFF",
                sim.P.tau_servo, deg(sim.P.rate_limit),
                sim.A.enabled ? "ON" : "OFF",
                sim.P.flex_on ? "FLEXIBLE" : "RIGIDA");
}

// Derivadas de estabilidad y ganancias: la tabla que acompana a §4 y §6.
static void printGains(const Sim& sim)
{
    double L_p, M_q, N_r, L_du, M_du, N_du;
    damping_coeffs(sim.P, L_p, M_q, N_r);
    control_effectiveness(sim.P, L_du, M_du, N_du);

    std::printf("k = qbar*S*CL_delta = %.4f N/rad\n", fin_gain(sim.P));
    std::printf("autoridad   L_du %+10.4f   M_du %+10.4f   N_du %+10.4f  N*m/rad\n",
                L_du, M_du, N_du);
    std::printf("amortig.    L_p  %+10.4f   M_q  %+10.4f   N_r  %+10.4f  N*m/(rad/s)\n",
                L_p, M_q, N_r);

    // Velocidad angular de equilibrio ante mando constante: la aleta acaba
    // alineada con el flujo local, luego  omega_ss = V*u/brazo.
    std::printf("omega_ss    p = %+.4f*u_roll   q = %+.4f*u_pitch   r = %+.4f*u_yaw\n",
                 v_eff(sim.P) / sim.P.d,
                 v_eff(sim.P) / sim.P.l_t,
                -v_eff(sim.P) / sim.P.l_t);

    std::printf("PID roll    kp %+9.4f  ki %+9.4f  kd %+9.4f  (wn %.2f)\n",
                sim.A.roll.kp,  sim.A.roll.ki,  sim.A.roll.kd,  sim.A.wn_roll);
    std::printf("PID pitch   kp %+9.4f  ki %+9.4f  kd %+9.4f  (wn %.2f)\n",
                sim.A.pitch.kp, sim.A.pitch.ki, sim.A.pitch.kd, sim.A.wn_pitch);
    std::printf("PID yaw     kp %+9.4f  ki %+9.4f  kd %+9.4f  (wn %.2f)\n",
                sim.A.yaw.kp,   sim.A.yaw.ki,   sim.A.yaw.kd,   sim.A.wn_yaw);
    std::printf("            (las ganancias de guinada son negativas: N_du < 0)\n");

    if (sim.P.flex_on) {
        double wb, wt;
        structural_freqs(sim.P, wb, wt);
        std::printf("flex        eta %.6f   qbar_car %.1f Pa (V = %.1f m/s)%s\n",
                    aeroelastic_efficiency(sim.P), qbar_caracteristica(sim.P),
                    std::sqrt(2.0 * qbar_caracteristica(sim.P) / sim.P.rho),
                    aeroelastic_divergence(sim.P) ? "   DIVERGIDA" : "");
        std::printf("modos       flexion %.1f rad/s (%.1f Hz)   torsion %.1f rad/s (%.1f Hz)\n",
                    wb, wb / (2.0 * M_PI), wt, wt / (2.0 * M_PI));
        std::printf("            CFL = w_max*dt = %.3f%s\n", structural_cfl(sim.P),
                    structural_unstable(sim.P)
                        ? "   *** > 2: baja dt o pon 'flex off', o esto acaba en NaN ***"
                        : "   (RK4 revienta en 2.83)");
    }
}

// Volcado CSV para las figuras del libro.
static void dumpCsv(Sim& sim, double T, double cada, const std::string& archivo)
{
    std::FILE* out = stdout;
    if (!archivo.empty()) {
        out = std::fopen(archivo.c_str(), "w");
        if (!out) { std::printf("error: no puedo escribir '%s'\n", archivo.c_str()); return; }
    }

    std::fprintf(out, "t,phi,theta,psi,p,q,r,d1,d2,d3,d4,dc1,dc2,dc3,dc4,L,M,N\n");

    const int n     = (int)std::lround(T / sim.P.dt);
    const int cada_n = (cada > 0.0) ? (int)std::lround(cada / sim.P.dt) : 1;

    for (int i = 0; i <= n; ++i) {
        if (i % (cada_n > 0 ? cada_n : 1) == 0) {
            double phi, th, psi, M[3];
            euler_from_quat(sim.s.q, phi, th, psi);
            fin_moments_rate(sim.P, sim.s.delta, sim.s.theta, sim.s.omega, M);
            std::fprintf(out,
                "%.5f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                sim.t, deg(phi), deg(th), deg(psi),
                sim.s.omega[0], sim.s.omega[1], sim.s.omega[2],
                deg(sim.s.delta[0]), deg(sim.s.delta[1]),
                deg(sim.s.delta[2]), deg(sim.s.delta[3]),
                deg(sim.s.delta_cmd[0]), deg(sim.s.delta_cmd[1]),
                deg(sim.s.delta_cmd[2]), deg(sim.s.delta_cmd[3]),
                M[0], M[1], M[2]);
        }
        if (i < n) sim_step(sim, sim.P.dt);
    }

    if (out != stdout) {
        std::fclose(out);
        std::printf("ok csv %.3f s -> %s\n", T, archivo.c_str());
    }
}

static void printHelp()
{
    std::printf(
        "mando:\n"
        "  pitch <deg>          fija u_pitch          (ignorado si el AP esta ON)\n"
        "  yaw   <deg>          fija u_yaw\n"
        "  roll  <deg>          fija u_roll\n"
        "  fin <n> <deg>        deflexion directa de la aleta n (1..4)\n"
        "\n"
        "autopiloto (§6):\n"
        "  ap on|off            activa/desactiva los tres lazos SISO\n"
        "  ref <phi> <th> <psi> referencias de actitud en grados\n"
        "  hold                 fija la referencia en la actitud actual\n"
        "  tune <wnR> <wnP> <wnY> <zeta>   reasigna polos y resintoniza\n"
        "  gains                derivadas de estabilidad y ganancias PID\n"
        "\n"
        "modelo:\n"
        "  damp on|off          amortiguamiento aerodinamico (§4)\n"
        "  flex on|off          aleta flexible / rigida (§5)\n"
        "  servo on|off         dinamica de actuador (§6)\n"
        "  set <k> <v>          k = V | rho | V_min | l_t | d | S_fin | b_fin\n"
        "                           | stall(deg) | dt | Ixx | Iyy | Izz | Ixz\n"
        "                           | tau_servo | rate_limit(deg/s)\n"
        "                           | e_ea | K_theta | I_theta | zeta_theta\n"
        "                           | K_bend | m_bend | zeta_bend | S_theta\n"
        "\n"
        "simulacion:\n"
        "  step <seg>           integra <seg> segundos con paso P.dt\n"
        "  state                imprime el estado completo\n"
        "  csv <seg> [cada] [archivo]   volcado CSV para figuras\n"
        "  reset | help | quit\n");
}

static int runHeadless()
{
    Sim sim = sim_init();

    std::printf("# simulacion_alerones — modo headless.  'help' para la lista.\n");

    std::string line;
    while (std::getline(std::cin, line)) {
        // Ignora comentarios y lineas vacias (permite alimentar scripts)
        size_t h = line.find('#');
        if (h != std::string::npos) line = line.substr(0, h);
        std::istringstream in(line);
        std::string cmd;
        if (!(in >> cmd)) continue;

        if (cmd == "quit" || cmd == "exit") break;

        if (cmd == "help") { printHelp(); continue; }

        if (cmd == "state") { printState(sim); continue; }

        if (cmd == "gains") { printGains(sim); continue; }

        if (cmd == "reset") {
            sim_reset(sim);
            apply_commands_instant(sim.C, sim.s);
            std::printf("ok reset\n");
            continue;
        }

        if (cmd == "pitch" || cmd == "yaw" || cmd == "roll") {
            double v;
            if (!(in >> v)) { std::printf("error: falta el valor en grados\n"); continue; }
            if      (cmd == "pitch") sim.C.u_pitch = rad(v);
            else if (cmd == "yaw")   sim.C.u_yaw   = rad(v);
            else                     sim.C.u_roll  = rad(v);
            apply_commands(sim.C, sim.s);
            if (sim.A.enabled)
                std::printf("aviso: el autopiloto esta ON y pisara este mando\n");
            std::printf("ok %s %.4f deg\n", cmd.c_str(), v);
            continue;
        }

        if (cmd == "fin") {
            int n; double v;
            if (!(in >> n >> v) || n < 1 || n > 4) {
                std::printf("error: uso 'fin <1..4> <deg>'\n"); continue;
            }
            sim.C.fin_trim[n - 1] = rad(v);
            apply_commands(sim.C, sim.s);
            std::printf("ok fin %d %.4f deg\n", n, v);
            continue;
        }

        if (cmd == "ap") {
            bool on;
            if (!parseOnOff(in, on)) { std::printf("error: uso 'ap on|off'\n"); continue; }
            sim.A.enabled = on;
            if (on) {
                euler_from_quat(sim.s.q, sim.A.phi_ref, sim.A.theta_ref, sim.A.psi_ref);
                autopilot_tune(sim.P, sim.A);
            }
            std::printf("ok ap %s\n", on ? "on" : "off");
            continue;
        }

        if (cmd == "ref") {
            double a, b, c;
            if (!(in >> a >> b >> c)) {
                std::printf("error: uso 'ref <phi> <theta> <psi>' en grados\n"); continue;
            }
            sim.A.phi_ref = rad(a); sim.A.theta_ref = rad(b); sim.A.psi_ref = rad(c);
            std::printf("ok ref %.3f %.3f %.3f deg\n", a, b, c);
            continue;
        }

        if (cmd == "hold") {
            euler_from_quat(sim.s.q, sim.A.phi_ref, sim.A.theta_ref, sim.A.psi_ref);
            std::printf("ok hold  phi %.3f  theta %.3f  psi %.3f deg\n",
                        deg(sim.A.phi_ref), deg(sim.A.theta_ref), deg(sim.A.psi_ref));
            continue;
        }

        if (cmd == "tune") {
            double wr, wp, wy, z;
            if (!(in >> wr >> wp >> wy >> z)) {
                std::printf("error: uso 'tune <wnR> <wnP> <wnY> <zeta>'\n"); continue;
            }
            sim.A.wn_roll = wr; sim.A.wn_pitch = wp; sim.A.wn_yaw = wy; sim.A.zeta = z;
            autopilot_tune(sim.P, sim.A);
            std::printf("ok tune  wn = %.3f / %.3f / %.3f rad/s   zeta = %.3f\n", wr, wp, wy, z);
            continue;
        }

        if (cmd == "damp") {
            bool on;
            if (!parseOnOff(in, on)) { std::printf("error: uso 'damp on|off'\n"); continue; }
            sim.P.damping_on = on;
            autopilot_tune(sim.P, sim.A);
            std::printf("ok damp %s\n", on ? "on" : "off");
            continue;
        }

        if (cmd == "flex") {
            bool on;
            if (!parseOnOff(in, on)) { std::printf("error: uso 'flex on|off'\n"); continue; }
            sim.P.flex_on = on;
            if (!on)                       // al volver a rigida, borra el estado
                for (int i = 0; i < 4; ++i) {
                    sim.s.theta[i] = sim.s.theta_dot[i] = 0.0;
                    sim.s.w[i]     = sim.s.w_dot[i]     = 0.0;
                }
            autopilot_tune(sim.P, sim.A);
            std::printf("ok flex %s", on ? "on" : "off");
            if (on) std::printf("  (eta = %.6f, CFL = %.3f)",
                                aeroelastic_efficiency(sim.P), structural_cfl(sim.P));
            std::printf("\n");
            if (structural_unstable(sim.P))
                std::printf("aviso: CFL %.3f > 2 con dt = %.4f s: la integracion divergira\n",
                            structural_cfl(sim.P), sim.P.dt);
            continue;
        }

        if (cmd == "servo") {
            bool on;
            if (!parseOnOff(in, on)) { std::printf("error: uso 'servo on|off'\n"); continue; }
            if (on) {
                if (sim.P.tau_servo  <= 0.0) sim.P.tau_servo  = 0.05;
                if (sim.P.rate_limit <= 0.0) sim.P.rate_limit = rad(240.0);
            } else {
                sim.P.tau_servo = sim.P.rate_limit = 0.0;
            }
            std::printf("ok servo %s (tau %.4f s, %.1f deg/s)\n",
                        on ? "on" : "off", sim.P.tau_servo, deg(sim.P.rate_limit));
            continue;
        }

        if (cmd == "step") {
            double T;
            if (!(in >> T)) { std::printf("error: uso 'step <seg>'\n"); continue; }
            int n = (int)std::lround(T / sim.P.dt);
            if (n < 0) n = 0;
            for (int i = 0; i < n; ++i) sim_step(sim, sim.P.dt);
            std::printf("ok step %d pasos (%.6f s)  t = %.6f s\n", n, n * sim.P.dt, sim.t);
            continue;
        }

        if (cmd == "csv") {
            double T, cada = 0.0;
            std::string archivo;
            if (!(in >> T)) { std::printf("error: uso 'csv <seg> [cada] [archivo]'\n"); continue; }
            in >> cada;
            in >> archivo;
            dumpCsv(sim, T, cada, archivo);
            continue;
        }

        if (cmd == "set") {
            std::string k; double v;
            if (!(in >> k >> v)) { std::printf("error: uso 'set <clave> <valor>'\n"); continue; }

            bool geom = false;
            if      (k == "V")          sim.P.V = v;
            else if (k == "rho")        sim.P.rho = v;
            else if (k == "V_min")      sim.P.V_min = v;
            else if (k == "l_t")        sim.P.l_t = v;
            else if (k == "d")          sim.P.d = v;
            else if (k == "S_fin")    { sim.P.S_fin = v; geom = true; }
            else if (k == "b_fin")    { sim.P.b_fin = v; geom = true; }
            else if (k == "stall")      sim.P.delta_stall = rad(v);
            else if (k == "dt")         sim.P.dt = v;
            else if (k == "Ixx")        sim.P.Ixx = v;
            else if (k == "Iyy")        sim.P.Iyy = v;
            else if (k == "Izz")        sim.P.Izz = v;
            else if (k == "Ixz")        sim.P.Ixz = v;
            else if (k == "tau_servo")  sim.P.tau_servo = v;
            else if (k == "rate_limit") sim.P.rate_limit = rad(v);
            else if (k == "e_ea")       sim.P.e_ea = v;
            else if (k == "K_theta")    sim.P.K_theta = v;
            else if (k == "I_theta")    sim.P.I_theta = v;
            else if (k == "zeta_theta") sim.P.zeta_theta = v;
            else if (k == "K_bend")     sim.P.K_bend = v;
            else if (k == "m_bend")     sim.P.m_bend = v;
            else if (k == "zeta_bend")  sim.P.zeta_bend = v;
            else if (k == "S_theta")    sim.P.S_theta = v;
            else { std::printf("error: clave desconocida '%s'\n", k.c_str()); continue; }

            // Tocar la geometria de la aleta cambia AR, y AR fija CL_delta.
            if (geom) params_update_aero(sim.P);
            autopilot_tune(sim.P, sim.A);   // la sintonia depende de todo esto

            if (geom)
                std::printf("ok set %s %.6f  (AR = %.4f -> CL_delta = %.6f)\n",
                            k.c_str(), v, aspect_ratio(sim.P.b_fin, sim.P.S_fin),
                            sim.P.CL_delta);
            else
                std::printf("ok set %s %.6f\n", k.c_str(), v);

            // Cambiar dt o la rigidez puede dejar el paso corto para los modos
            // estructurales; mas vale decirlo aqui que ver NaN dentro de 200
            // pasos y no saber de donde salio.
            if (structural_unstable(sim.P))
                std::printf("aviso: CFL %.3f > 2 con dt = %.4f s: la integracion divergira "
                            "(baja dt o usa 'flex off')\n",
                            structural_cfl(sim.P), sim.P.dt);
            continue;
        }

        std::printf("error: comando desconocido '%s' (prueba 'help')\n", cmd.c_str());
    }
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    bool headless = false;
    for (int i = 1; i < argc; ++i)
        if (std::strcmp(argv[i], "--headless") == 0) headless = true;

    if (headless) return runHeadless();

    Sim sim = sim_init();

    std::printf("simulacion_alerones — arreglo cruciforme, convencion Blakelock Fig. 8-2\n");
    std::printf("  qbar = %.1f Pa | CL_delta = %.4f 1/rad (AR = %.3f, Helmbold)\n",
                qbar(sim.P), sim.P.CL_delta, aspect_ratio(sim.P.b_fin, sim.P.S_fin));
    std::printf("  l_t = %.3f m | d = %.3f m | stall = %.1f deg | dt = %.4f s\n",
                sim.P.l_t, sim.P.d, deg(sim.P.delta_stall), sim.P.dt);
    std::printf("  Ixx = %.4f  Iyy = %.4f  Izz = %.4f  Ixz = %.4f kg m^2\n",
                sim.P.Ixx, sim.P.Iyy, sim.P.Izz, sim.P.Ixz);

    double L_p, M_q, N_r;
    damping_coeffs(sim.P, L_p, M_q, N_r);
    std::printf("  amortiguamiento: L_p = %.4f  M_q = %.4f  N_r = %.4f N*m/(rad/s)\n",
                L_p, M_q, N_r);
    std::printf("  servo: tau = %.3f s, %.0f deg/s\n",
                sim.P.tau_servo, deg(sim.P.rate_limit));

    if (sim.P.flex_on) {
        double wb, wt;
        structural_freqs(sim.P, wb, wt);
        std::printf("  aleta FLEXIBLE: eta = %.4f | modos %.1f Hz (flexion) y %.1f Hz (torsion)\n",
                    aeroelastic_efficiency(sim.P), wb / (2.0 * M_PI), wt / (2.0 * M_PI));
        if (structural_unstable(sim.P))
            std::printf("  *** CFL = %.2f > 2: dt demasiado grande, la aleta divergira ***\n",
                        structural_cfl(sim.P));
    }
    std::printf("  teclas: A autopiloto | D amortiguamiento | f aleta rigida | F exagerar\n");

    run_gui(argc, argv, sim);
    return 0;
}
