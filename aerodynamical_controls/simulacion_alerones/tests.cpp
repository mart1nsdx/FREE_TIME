// ═══════════════════════════════════════════════════════════════════════════
//  tests.cpp — casos de validacion (§7).  Sin dependencias graficas.
//      make test
//
//  Los casos 1-7 comprueban el mixing y el desacoplo por simetria (§2-§3);
//  los casos 8-13 el amortiguamiento (§4), el actuador y el autopiloto (§6).
//  El prototipo Python de §8 corre exactamente los mismos casos y debe dar
//  los mismos numeros:  make test  vs  make python-test.
// ═══════════════════════════════════════════════════════════════════════════
#include "fisica_alerones.hpp"

#include <cmath>
#include <cstdio>

using namespace alerones;

static int g_fail = 0;
static int g_total = 0;

static void check(bool ok, const char* nombre, const char* detalle = "")
{
    ++g_total;
    if (ok) {
        std::printf("  [ ok ] %s\n", nombre);
    } else {
        ++g_fail;
        std::printf("  [FAIL] %s  %s\n", nombre, detalle);
    }
}

static bool close(double a, double b, double tol) { return std::fabs(a - b) < tol; }
static bool zero(double a, double tol = 1e-12)    { return std::fabs(a) < tol; }

static double deg(double r) { return r * 180.0 / M_PI; }
static double rad(double d) { return d * M_PI / 180.0; }

// ─────────────────────────────────────────────────────────────────────────────
int main()
{
    const Params P = params_default();

    std::printf("\nParametros:  qbar = %.1f Pa | CL_delta = %.4f 1/rad (AR = %.3f)\n",
                qbar(P), P.CL_delta, aspect_ratio(P.b_fin, P.S_fin));
    std::printf("             l_t = %.3f m | d = %.3f m | delta_stall = %.1f deg\n",
                P.l_t, P.d, deg(P.delta_stall));
    std::printf("             Ixx = %.4f  Iyy = %.4f  Izz = %.4f kg m^2\n\n",
                P.Ixx, P.Iyy, P.Izz);

    // ── 1. Pitch puro ────────────────────────────────────────────────────────
    std::printf("1. Pitch puro (u_pitch > 0)\n");
    {
        double d[4], M[3];
        mix_commands(rad(5.0), 0.0, 0.0, d);
        fin_moments(P, d, M);

        check(M[1] > 0.0,  "M > 0 (momento de cabeceo)");
        check(zero(M[0]),  "L == 0 (sin roll)");
        check(zero(M[2]),  "N == 0 (sin yaw)");

        State s = state_init();
        mix_commands(rad(5.0), 0.0, 0.0, s.delta);
        for (int i = 0; i < 500; ++i) step_rk4(P, s, P.dt);   // 1 s

        check(s.omega[1] > 0.0, "q crece (pitch rate)");
        check(zero(s.omega[0], 1e-10) && zero(s.omega[2], 1e-10),
              "p == r == 0 (canales desacoplados)");
        std::printf("         -> tras 1 s:  p=%.3e  q=%+.5f  r=%.3e rad/s\n",
                    s.omega[0], s.omega[1], s.omega[2]);
    }

    // ── 2. Yaw puro ──────────────────────────────────────────────────────────
    std::printf("\n2. Yaw puro (u_yaw > 0)\n");
    {
        double d[4], M[3];
        mix_commands(0.0, rad(5.0), 0.0, d);
        fin_moments(P, d, M);

        check(std::fabs(M[2]) > 1e-6, "N != 0 (momento de guinada)");
        check(zero(M[0]), "L == 0");
        check(zero(M[1]), "M == 0");
        // Signo: delta3,4 > 0 empujan la cola hacia +y -> la nariz va a -y.
        check(M[2] < 0.0, "N < 0 (convencion Blakelock: cola a +y, nariz a -y)");

        State s = state_init();
        mix_commands(0.0, rad(5.0), 0.0, s.delta);
        for (int i = 0; i < 500; ++i) step_rk4(P, s, P.dt);

        check(zero(s.omega[0], 1e-10) && zero(s.omega[1], 1e-10), "p == q == 0");
        std::printf("         -> tras 1 s:  p=%.3e  q=%.3e  r=%+.5f rad/s\n",
                    s.omega[0], s.omega[1], s.omega[2]);
    }

    // ── 3. Roll puro (modo antisimetrico) ────────────────────────────────────
    std::printf("\n3. Roll puro (u_roll > 0)\n");
    {
        double d[4], M[3];
        mix_commands(0.0, 0.0, rad(5.0), d);
        fin_moments(P, d, M);

        check(M[0] > 0.0, "L > 0 (momento de alabeo)");
        check(zero(M[1]), "M == 0");
        check(zero(M[2]), "N == 0");

        State s = state_init();
        mix_commands(0.0, 0.0, rad(5.0), s.delta);
        for (int i = 0; i < 500; ++i) step_rk4(P, s, P.dt);

        check(s.omega[0] > 0.0, "p crece (roll rate)");
        check(zero(s.omega[1], 1e-10) && zero(s.omega[2], 1e-10), "q == r == 0");
        std::printf("         -> tras 1 s:  p=%+.5f  q=%.3e  r=%.3e rad/s\n",
                    s.omega[0], s.omega[1], s.omega[2]);
    }

    // ── 4. Round-trip del mixing ─────────────────────────────────────────────
    std::printf("\n4. Round-trip del mixing:  demix(mix(u)) == u\n");
    {
        const double casos[][3] = {
            { rad( 3.0), rad(-2.0), rad( 1.5) },
            { rad(-7.0), rad( 4.0), rad(-6.0) },
            { 0.0,       0.0,       0.0       },
            { rad(12.0), rad(12.0), rad(12.0) }
        };
        bool ok = true;
        double maxerr = 0.0;
        for (auto& c : casos) {
            double d[4], a, b, r;
            mix_commands(c[0], c[1], c[2], d);
            demix(d, a, b, r);
            maxerr = std::fmax(maxerr, std::fabs(a - c[0]));
            maxerr = std::fmax(maxerr, std::fabs(b - c[1]));
            maxerr = std::fmax(maxerr, std::fabs(r - c[2]));
            ok = ok && close(a, c[0], 1e-15) && close(b, c[1], 1e-15)
                    && close(r, c[2], 1e-15);
        }
        check(ok, "identidad en los 4 casos");
        std::printf("         -> error maximo: %.3e rad\n", maxerr);
    }

    // ── 5. Saturacion por perdida ────────────────────────────────────────────
    std::printf("\n5. Saturacion (delta > delta_stall)\n");
    {
        double d_stall[4] = { P.delta_stall, P.delta_stall, P.delta_stall, P.delta_stall };
        double d_big[4]   = { 3.0 * P.delta_stall, 3.0 * P.delta_stall,
                              3.0 * P.delta_stall, 3.0 * P.delta_stall };
        double d_neg[4]   = { -3.0 * P.delta_stall, 0, 0, 0 };

        double f_stall[4], f_big[4], f_neg[4];
        fin_forces(P, d_stall, f_stall);
        fin_forces(P, d_big,   f_big);
        fin_forces(P, d_neg,   f_neg);

        check(close(f_big[0], f_stall[0], 1e-12), "f satura por arriba (no crece)");
        check(close(f_neg[0], -f_stall[0], 1e-12), "f satura por abajo (simetrico)");

        // Debajo del stall la respuesta sigue siendo lineal.
        double d_half[4] = { 0.5 * P.delta_stall, 0, 0, 0 };
        double f_half[4];
        fin_forces(P, d_half, f_half);
        check(close(f_half[0], 0.5 * f_stall[0], 1e-12), "lineal por debajo del stall");
        std::printf("         -> f(stall) = %.3f N   f(3x stall) = %.3f N\n",
                    f_stall[0], f_big[0]);
    }

    // ── 6. Norma del cuaternion tras 1e4 pasos ───────────────────────────────
    std::printf("\n6. Norma del cuaternion tras 1e4 pasos\n");
    {
        State s = state_init();
        // Rates iniciales no triviales + mando mixto: excita los tres canales.
        s.omega[0] = 0.8; s.omega[1] = -0.4; s.omega[2] = 0.3;
        mix_commands(rad(4.0), rad(-3.0), rad(2.0), s.delta);

        for (int i = 0; i < 10000; ++i) step_rk4(P, s, P.dt);

        double n = std::sqrt(s.q[0]*s.q[0] + s.q[1]*s.q[1] + s.q[2]*s.q[2] + s.q[3]*s.q[3]);
        check(std::fabs(n - 1.0) < 1e-9, "|e| - 1 < 1e-9");

        double phi, th, psi;
        euler_from_quat(s.q, phi, th, psi);
        std::printf("         -> |e| - 1 = %.3e   |   phi=%+.2f  theta=%+.2f  psi=%+.2f deg\n",
                    n - 1.0, deg(phi), deg(th), deg(psi));
    }

    // ── Extra: desacoplo cruzado del mixing con comando combinado ────────────
    std::printf("\n7. Extra: comando combinado -> momentos coherentes con las irreps\n");
    {
        const double up = rad(4.0), uy = rad(-3.0), ur = rad(2.0);
        double d[4], M[3];
        mix_commands(up, uy, ur, d);
        fin_moments(P, d, M);

        const double k = qbar(P) * P.S_fin * P.CL_delta;
        check(close(M[0],  4.0 * P.d   * k * ur, 1e-9), "L = 4 d k u_roll");
        check(close(M[1],  2.0 * P.l_t * k * up, 1e-9), "M = 2 l_t k u_pitch");
        check(close(M[2], -2.0 * P.l_t * k * uy, 1e-9), "N = -2 l_t k u_yaw");
        std::printf("         -> L=%+.3f  M=%+.3f  N=%+.3f  N*m\n", M[0], M[1], M[2]);
    }


    // ── 8. Amortiguamiento: la matriz de derivadas es DIAGONAL (§4) ──────────
    std::printf("\n8. Amortiguamiento aerodinamico: matriz diagonal en la base de irreps\n");
    {
        const double d0[4] = { 0, 0, 0, 0 };       // sin mando: solo la rotacion
        const double w = 0.05;                     // rad/s, bien dentro del lineal

        double L_p, M_q, N_r;
        damping_coeffs(P, L_p, M_q, N_r);

        double Mp[3], Mq[3], Mr[3];
        const double wp[3] = { w, 0, 0 };
        const double wq[3] = { 0, w, 0 };
        const double wr[3] = { 0, 0, w };
        fin_moments_rate(P, d0, nullptr, wp, Mp);
        fin_moments_rate(P, d0, nullptr, wq, Mq);
        fin_moments_rate(P, d0, nullptr, wr, Mr);

        check(zero(Mp[1], 1e-12) && zero(Mp[2], 1e-12), "p pura -> solo L (sin M ni N)");
        check(zero(Mq[0], 1e-12) && zero(Mq[2], 1e-12), "q pura -> solo M (sin L ni N)");
        check(zero(Mr[0], 1e-12) && zero(Mr[1], 1e-12), "r pura -> solo N (sin L ni M)");

        check(Mp[0] < 0.0 && Mq[1] < 0.0 && Mr[2] < 0.0, "los tres momentos se oponen al giro");

        check(close(Mp[0], L_p * w, 1e-9), "L = L_p * p   (coincide con la linealizacion)");
        check(close(Mq[1], M_q * w, 1e-9), "M = M_q * q");
        check(close(Mr[2], N_r * w, 1e-9), "N = N_r * r");
        check(close(M_q, N_r, 1e-12), "M_q == N_r (los dos pares tienen el mismo brazo)");

        std::printf("         -> L_p=%+.4f  M_q=%+.4f  N_r=%+.4f  N*m/(rad/s)\n", L_p, M_q, N_r);
    }

    // ── 9. Regimen permanente: la aleta acaba alineada con el flujo local ────
    std::printf("\n9. Regimen permanente con mando constante:  omega_ss = V*u/brazo\n");
    {
        Params Q = P;
        Q.dt = 0.01;                     // tau ~ 20-30 s: no hace falta 2 ms
        Q.flex_on = false;               // ...pero 10 ms NO resuelve los modos
        const int  N   = 20000;          // estructurales de §5 (65 Hz): con la
                                         // aleta flexible RK4 diverge a NaN.
                                         // 200 s (6-10 constantes de tiempo)
        const double u = rad(3.0);

        // Cabeceo:  q_ss = V*u_pitch/l_t
        State sp = state_init();
        mix_commands(u, 0.0, 0.0, sp.delta);
        for (int i = 0; i < N; ++i) step_rk4(Q, sp, Q.dt);
        const double q_teo = v_eff(Q) * u / Q.l_t;
        check(close(sp.omega[1], q_teo, 0.01 * q_teo), "q -> V*u_pitch/l_t");

        // Alabeo:  p_ss = V*u_roll/d
        State sr = state_init();
        mix_commands(0.0, 0.0, u, sr.delta);
        for (int i = 0; i < N; ++i) step_rk4(Q, sr, Q.dt);
        const double p_teo = v_eff(Q) * u / Q.d;
        check(close(sr.omega[0], p_teo, 0.01 * p_teo), "p -> V*u_roll/d");

        // Guinada:  r_ss = -V*u_yaw/l_t   (signo de la convencion Blakelock)
        State sy = state_init();
        mix_commands(0.0, u, 0.0, sy.delta);
        for (int i = 0; i < N; ++i) step_rk4(Q, sy, Q.dt);
        const double r_teo = -v_eff(Q) * u / Q.l_t;
        check(close(sy.omega[2], r_teo, 0.01 * std::fabs(r_teo)), "r -> -V*u_yaw/l_t");

        // En equilibrio la incidencia efectiva se anula: la aleta "no ve" nada.
        double de[4];
        fin_effective_deflection(Q, sp.delta, sp.theta, sp.omega, de);
        check(zero(de[0], 1e-3) && zero(de[1], 1e-3), "delta_eff -> 0 en equilibrio");

        std::printf("         -> q=%+.5f (teo %+.5f)   p=%+.5f (teo %+.5f)   r=%+.5f (teo %+.5f)\n",
                    sp.omega[1], q_teo, sr.omega[0], p_teo, sy.omega[2], r_teo);
    }

    // ── 10. Sin amortiguamiento la velocidad angular no converge ─────────────
    std::printf("\n10. Contraste: damping_on = false -> rampa sin limite\n");
    {
        Params Q = P;
        Q.dt = 0.01;
        Q.flex_on = false;               // igual que en el caso 9
        Q.damping_on = false;

        State s = state_init();
        mix_commands(rad(3.0), 0.0, 0.0, s.delta);

        for (int i = 0; i < 1000; ++i) step_rk4(Q, s, Q.dt);   // 10 s
        const double q10 = s.omega[1];
        for (int i = 0; i < 1000; ++i) step_rk4(Q, s, Q.dt);   // 20 s
        const double q20 = s.omega[1];

        check(close(q20, 2.0 * q10, 1e-6), "q crece lineal (q(20s) = 2*q(10s))");

        double L_p, M_q, N_r;
        damping_coeffs(Q, L_p, M_q, N_r);
        check(zero(L_p) && zero(M_q) && zero(N_r), "las derivadas se anulan");
        std::printf("         -> q(10s)=%+.5f   q(20s)=%+.5f rad/s  (sin equilibrio)\n", q10, q20);
    }

    // ── 11. Actuador: limite de velocidad y retardo de primer orden (§6) ─────
    std::printf("\n11. Dinamica del actuador\n");
    {
        // (a) Solo limite de velocidad: el primer paso avanza exactamente R*dt.
        Params Q = P;
        Q.tau_servo  = 0.0;
        Q.rate_limit = rad(100.0);

        State s = state_init();
        for (int i = 0; i < 4; ++i) s.delta_cmd[i] = rad(10.0);
        servo_step(Q, s, Q.dt);
        check(close(s.delta[0], Q.rate_limit * Q.dt, 1e-15), "un paso avanza rate_limit*dt");

        // Y nunca sobrepasa el comando.
        for (int i = 0; i < 5000; ++i) servo_step(Q, s, Q.dt);
        check(close(s.delta[0], rad(10.0), 1e-12), "converge al comando sin sobrepasarlo");

        // (b) Solo retardo: tras t = tau se ha recorrido 1 - 1/e del salto.
        Params R = P;
        R.tau_servo  = 0.05;
        R.rate_limit = 0.0;

        State s2 = state_init();
        for (int i = 0; i < 4; ++i) s2.delta_cmd[i] = rad(10.0);
        const int n_tau = (int)std::lround(R.tau_servo / R.dt);
        for (int i = 0; i < n_tau; ++i) servo_step(R, s2, R.dt);
        check(close(s2.delta[0], rad(10.0) * (1.0 - std::exp(-1.0)), 1e-9),
              "en t = tau lleva 1 - 1/e del recorrido");

        // (c) Actuador ideal: salto instantaneo.
        Params I = P;
        I.tau_servo = I.rate_limit = 0.0;
        State s3 = state_init();
        for (int i = 0; i < 4; ++i) s3.delta_cmd[i] = rad(10.0);
        servo_step(I, s3, I.dt);
        check(close(s3.delta[0], rad(10.0), 1e-15), "tau = rate = 0 -> salto instantaneo");

        std::printf("         -> rate: %.6f deg tras 1 paso   |   lag: %.4f de %.1f deg en t=tau\n",
                    deg(Q.rate_limit * Q.dt), deg(s2.delta[0]), 10.0);
    }

    // ── 12. Autopiloto: tres lazos SISO, error estacionario nulo (§6) ────────
    std::printf("\n12. Autopiloto de actitud\n");
    {
        Params Q = P;
        Q.dt = 0.005;

        Autopilot A = autopilot_default(Q);
        A.enabled   = true;
        A.phi_ref   = rad(10.0);
        A.theta_ref = rad( 5.0);
        A.psi_ref   = rad(-8.0);

        check(A.pitch.kp > 0.0 && A.roll.kp > 0.0, "kp de alabeo y cabeceo positivas");
        check(A.yaw.kp   < 0.0, "kp de guinada NEGATIVA (N_du < 0)");
        check(A.pitch.kd > 0.0, "kd de cabeceo positiva (anade amortiguamiento)");

        State    s = state_init();
        Commands C = commands_zero();

        for (int i = 0; i < 12000; ++i) {          // 60 s
            autopilot_update(Q, A, s, C, Q.dt);
            apply_commands(C, s);
            servo_step(Q, s, Q.dt);
            step_rk4(Q, s, Q.dt);
        }

        double phi, th, psi;
        euler_from_quat(s.q, phi, th, psi);

        check(std::fabs(deg(phi) - 10.0) < 0.2, "phi -> phi_ref  (< 0.2 deg)");
        check(std::fabs(deg(th)  -  5.0) < 0.2, "theta -> theta_ref");
        check(std::fabs(deg(psi) + 8.0)  < 0.2, "psi -> psi_ref");
        check(zero(s.omega[0], 1e-3) && zero(s.omega[1], 1e-3) && zero(s.omega[2], 1e-3),
              "velocidades angulares residuales ~ 0");

        std::printf("         -> phi=%+.4f (ref +10)   theta=%+.4f (ref +5)   psi=%+.4f (ref -8) deg\n",
                    deg(phi), deg(th), deg(psi));
    }

    // ── 13. Ixz: el producto de inercia acopla alabeo y guinada ──────────────
    std::printf("\n13. Producto de inercia Ixz: acoplo roll-yaw\n");
    {
        Params Q = P;
        Q.dt = 0.01;
        Q.flex_on = false;               // aisla el acoplo inercial del elastico

        // Con Ixz = 0 el alabeo puro no genera nada de guinada.
        State s0 = state_init();
        mix_commands(0.0, 0.0, rad(3.0), s0.delta);
        for (int i = 0; i < 200; ++i) step_rk4(Q, s0, Q.dt);   // 2 s
        check(zero(s0.omega[2], 1e-12), "Ixz = 0 -> r == 0 con mando de alabeo");

        // Con Ixz != 0 aparece guinada aunque el momento N siga siendo cero.
        Q.Ixz = 2.0;
        Q.flex_on = false;
        State s1 = state_init();
        mix_commands(0.0, 0.0, rad(3.0), s1.delta);
        for (int i = 0; i < 200; ++i) step_rk4(Q, s1, Q.dt);
        check(std::fabs(s1.omega[2]) > 1e-6, "Ixz != 0 -> aparece r (acoplo inercial)");

        // Las aletas no mandan NADA de guinada: el mixing sigue desacoplado.
        // La r que aparece es puramente inercial (el termino omega x I*omega),
        // y solo DESPUES la aerodinamica reacciona amortiguandola.
        double Mc[3], Mt[3];
        fin_moments(Q, s1.delta, Mc);                     // solo mando
        fin_moments_rate(Q, s1.delta, s1.theta, s1.omega, Mt);      // mando + rotacion
        check(std::fabs(Mc[2]) < 1e-12, "el mando de guinada sigue siendo N = 0");
        check(Mt[2] * s1.omega[2] < 0.0, "el N que aparece es amortiguamiento, se opone a r");

        std::printf("         -> Ixz=0: r=%.3e    Ixz=2: r=%+.6f rad/s  (p=%+.5f)\n",
                    s0.omega[2], s1.omega[2], s1.omega[0]);
        std::printf("            N de mando = %.3e N*m   |   N total = %+.6f N*m\n",
                    Mc[2], Mt[2]);
    }

    // ── 14. Coherencia de la geometria: AR fija CL_delta ─────────────────────
    std::printf("\n14. params_update_aero: cambiar la aleta recalcula CL_delta\n");
    {
        Params Q = P;
        const double cl0 = Q.CL_delta;

        Q.S_fin = 2.0 * P.S_fin;          // aleta el doble de grande
        params_update_aero(Q);

        check(!close(Q.CL_delta, cl0, 1e-9), "CL_delta cambia al cambiar S_fin");
        check(close(Q.CL_delta,
                    cl_alpha_helmbold(aspect_ratio(Q.b_fin, Q.S_fin)), 1e-15),
              "CL_delta == CL_alpha(AR(b_fin, S_fin))");
        check(Q.CL_delta < cl0, "mas superficie con la misma envergadura -> AR menor -> CL menor");

        std::printf("         -> AR %.4f -> %.4f   |   CL_delta %.4f -> %.4f 1/rad\n",
                    aspect_ratio(P.b_fin, P.S_fin), aspect_ratio(Q.b_fin, Q.S_fin),
                    cl0, Q.CL_delta);
    }


    // ── 15. Aeroelasticidad estatica: eficiencia eta = 1/(1+lambda) (§5) ─────
    std::printf("\n15. Aeroelasticidad: la aleta flexible entrega solo eta del mando\n");
    {
        Params Q = P;
        Q.damping_on = false;      // aisla el lazo elastico del de §4

        const double u   = rad(5.0);
        const double eta = aeroelastic_efficiency(Q);

        State s = state_init();
        mix_commands(u, 0.0, 0.0, s.delta);
        for (int i = 0; i < 500; ++i) step_rk4(Q, s, Q.dt);   // 1 s: el modo
                                                              // de torsion ya
                                                              // esta asentado
        double de[4];
        fin_effective_deflection(Q, s.delta, s.theta, s.omega, de);

        check(eta < 1.0, "eta < 1 con e_ea > 0 (el centro aerodinamico va detras)");
        check(s.theta[0] < 0.0, "la aleta se DESTUERCE: theta se opone al mando");
        check(close(de[0] / u, eta, 1e-6), "delta_eff/delta == eta");

        // Flexion: en equilibrio la punta cede f/K_bend.
        double f[4];
        fin_forces(Q, de, f);
        check(close(s.w[0], f[0] / Q.K_bend, 1e-6), "flecha en punta == f/K_bend");

        // Las dos aletas del par horizontal se deforman IGUAL: la respuesta
        // elastica es ella misma simetrica, asi que el desacoplo de §2 aguanta.
        double M[3];
        fin_moments(Q, de, M);
        check(close(s.theta[0], s.theta[1], 1e-15), "theta1 == theta2 (simetria V4)");
        check(zero(M[0], 1e-12) && zero(M[2], 1e-12), "sigue sin haber L ni N");

        std::printf("         -> eta=%.6f   theta=%+.5f deg   flecha=%.4f mm   "
                    "delta_eff=%.5f de %.1f deg\n",
                    eta, deg(s.theta[0]), 1000.0 * s.w[0], deg(de[0]), 5.0);
    }

    // ── 16. La eficiencia cae con la presion dinamica ────────────────────────
    std::printf("\n16. eta cae como 1/qbar: la aeroelasticidad es cosa de ir rapido\n");
    {
        Params Q = P;
        const double q_car = qbar_caracteristica(Q);

        // A la presion dinamica caracteristica se pierde justo la mitad.
        Params R = Q;
        R.V = std::sqrt(2.0 * q_car / R.rho);
        check(close(aeroelastic_efficiency(R), 0.5, 1e-9), "eta = 1/2 en qbar_car");

        // Y a velocidad de crucero el efecto es casi inapreciable.
        check(aeroelastic_efficiency(Q) > 0.99, "eta > 0.99 a 30 m/s");

        // Monotona decreciente.
        Params S1 = Q, S2 = Q;
        S1.V = 100.0; S2.V = 200.0;
        check(aeroelastic_efficiency(S2) < aeroelastic_efficiency(S1) &&
              aeroelastic_efficiency(S1) < aeroelastic_efficiency(Q),
              "eta decrece monotona con V");

        // Aleta rigida: eta == 1 exacto, sin excusas.
        Params RG = Q; RG.flex_on = false;
        check(aeroelastic_efficiency(RG) == 1.0, "flex_on = false -> eta == 1 exacto");
        check(qbar_caracteristica(RG) == 0.0,    "...y no hay qbar caracteristica");

        std::printf("         -> qbar_car = %.0f Pa (V = %.0f m/s)   |   "
                    "eta: 30 m/s %.4f | 100 %.4f | 200 %.4f\n",
                    q_car, R.V, aeroelastic_efficiency(Q),
                    aeroelastic_efficiency(S1), aeroelastic_efficiency(S2));
    }

    // ── 17. Divergencia: aleta sobrebalanceada (e_ea < 0) ────────────────────
    std::printf("\n17. Divergencia aeroelastica con el pivote por detras del c.a.\n");
    {
        Params Q = P;
        Q.e_ea = -0.02;            // pivote DETRAS del centro aerodinamico

        check(aeroelastic_efficiency(Q) > 1.0, "eta > 1: ahora el lazo AMPLIFICA");
        check(!aeroelastic_divergence(Q), "a 30 m/s todavia no diverge");

        const double q_div = qbar_caracteristica(Q);

        // Justo por debajo: amplificacion grande pero finita.
        Params R = Q;
        R.V = std::sqrt(2.0 * 0.9 * q_div / R.rho);
        check(!aeroelastic_divergence(R), "al 90% de qbar_div sigue acotada");
        check(close(aeroelastic_efficiency(R), 10.0, 1e-6), "eta = 1/(1-0.9) = 10");

        // Por encima: divergida.
        Params S1 = Q;
        S1.V = std::sqrt(2.0 * 1.1 * q_div / S1.rho);
        check(aeroelastic_divergence(S1), "por encima de qbar_div: divergida");

        // Y el signo de la torsion es el contrario que en el caso 15.
        Params T = Q; T.damping_on = false;
        State s = state_init();
        mix_commands(rad(5.0), 0.0, 0.0, s.delta);
        for (int i = 0; i < 500; ++i) step_rk4(T, s, T.dt);
        check(s.theta[0] > 0.0, "la aleta se retuerce A FAVOR del mando");

        std::printf("         -> qbar_div = %.0f Pa (V = %.0f m/s)   eta(30 m/s) = %.4f   "
                    "theta = %+.5f deg\n",
                    q_div, std::sqrt(2.0 * q_div / Q.rho),
                    aeroelastic_efficiency(Q), deg(s.theta[0]));
    }

    // ── 18. Modos estructurales y estabilidad del integrador ────────────────
    std::printf("\n18. Modos estructurales: son ELLOS quienes fijan el paso\n");
    {
        double wb, wt;
        structural_freqs(P, wb, wt);

        check(close(wb, std::sqrt(P.K_bend  / P.m_bend),  1e-12), "w_bend = sqrt(K/m)");
        check(close(wt, std::sqrt(P.K_theta / P.I_theta), 1e-12), "w_tors = sqrt(K/I)");

        // El modo mas rapido es cientos de veces mas rapido que la actitud.
        double L_p, M_q, N_r;
        damping_coeffs(P, L_p, M_q, N_r);
        check(wt > 100.0 * (std::fabs(M_q) / P.Iyy), "la torsion es >100x el modo de cabeceo");

        check(close(structural_cfl(P), std::fmax(wb, wt) * P.dt, 1e-12), "CFL = w_max*dt");
        check(!structural_unstable(P), "dt = 2 ms resuelve los modos");

        Params Q = P; Q.dt = 0.01;
        check(structural_unstable(Q), "dt = 10 ms NO los resuelve (avisa antes de NaN)");

        Params RG = P; RG.flex_on = false;
        check(structural_cfl(RG) == 0.0, "aleta rigida: sin restriccion de paso");

        std::printf("         -> flexion %.1f rad/s (%.1f Hz)   torsion %.1f rad/s (%.1f Hz)\n",
                    wb, wb / (2.0 * M_PI), wt, wt / (2.0 * M_PI));
        std::printf("            CFL: dt=2ms -> %.3f    dt=10ms -> %.3f    (RK4 revienta en 2.83)\n",
                    structural_cfl(P), structural_cfl(Q));
    }

    // ── Resumen ──────────────────────────────────────────────────────────────
    std::printf("\n────────────────────────────────────────────────\n");
    if (g_fail == 0) std::printf("TODOS LOS TESTS PASARON  (%d/%d)\n\n", g_total, g_total);
    else             std::printf("FALLARON %d de %d\n\n", g_fail, g_total);

    return (g_fail == 0) ? 0 : 1;
}
