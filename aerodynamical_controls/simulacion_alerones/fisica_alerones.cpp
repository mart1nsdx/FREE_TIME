// ═══════════════════════════════════════════════════════════════════════════
//  fisica_alerones.cpp — mixing, fuerzas, momentos, amortiguamiento, actuador,
//  Euler + cuaternion, RK4 y autopiloto.
//
//  Ver fisica_alerones.hpp para la convencion de signos y README.md para la
//  justificacion por simetria del desacoplo.
// ═══════════════════════════════════════════════════════════════════════════
#include "fisica_alerones.hpp"

#include <cmath>

namespace alerones {

static double clamp(double v, double lo, double hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// ─────────────────────────────────────────────────────────────────────────────
// Parametros
// ─────────────────────────────────────────────────────────────────────────────

double aspect_ratio(double b_fin, double S_fin)
{
    return (S_fin > 0.0) ? (b_fin * b_fin) / S_fin : 0.0;
}

double cl_alpha_helmbold(double AR)
{
    // Helmbold: interpola entre la teoria de linea sustentadora (AR alto) y
    // la teoria de ala esbelta (AR bajo).  Coherente con el tratamiento de
    // superficies finitas del resto del proyecto.
    return (2.0 * M_PI * AR) / (2.0 + std::sqrt(AR * AR + 4.0));
}

void params_update_aero(Params& P)
{
    P.CL_delta = cl_alpha_helmbold(aspect_ratio(P.b_fin, P.S_fin));
}

Params params_default()
{
    Params P{};

    P.rho   = 1.225;    // atmosfera estandar a nivel del mar
    P.V     = 30.0;     // velocidad de vuelo
    P.V_min = 1.0;      // por debajo de esto el modelo lineal no dice nada util

    // Aleta: 0.25 m de envergadura x 0.16 m de cuerda
    P.b_fin = 0.25;
    P.S_fin = 0.25 * 0.16;                                   // 0.040 m^2
    params_update_aero(P);

    // Geometria: aletas en la cola; el offset radial es el radio del fuselaje
    // (0.16 m) mas la mitad de la envergadura -> centro de presion de la aleta.
    P.l_t = 1.75;
    P.d   = 0.16 + 0.5 * P.b_fin;                            // 0.285 m

    P.delta_stall = 15.0 * M_PI / 180.0;

    // Inercias de buildDeltaWingDrone() — unmanned/physics_UAV.h
    P.Ixx = 16.0629;
    P.Iyy = 187.6754;
    P.Izz = 201.9983;
    P.Ixz = 0.0;        // simetria XZ perfecta; ponlo != 0 para ver el acoplo

    P.damping_on = true;

    // Servo de aleta tipico: 0.05 s de retardo, 240 deg/s de velocidad maxima.
    P.tau_servo  = 0.05;
    P.rate_limit = 240.0 * M_PI / 180.0;

    // ── Aeroelasticidad (§5) ────────────────────────────────────────────────
    // Aleta de aluminio de 3 mm: 0.35 kg, voladizo de 0.25 m.
    //   K_bend = 3EI/L^3 con EI = E*c*h^3/12 = 70e9*0.16*2.7e-8/12 ~ 25 N*m^2
    //   m_bend ~ 0.25*m_aleta (masa generalizada del primer modo de voladizo)
    // K_theta es sobre todo la rigidez del SERVO y del eje, no de la placa.
    P.flex_on    = true;
    P.e_ea       = 0.02;      // centro aerodinamico 2 cm por detras del pivote
    P.K_theta    = 250.0;
    P.I_theta    = 1.5e-3;    // -> w_tors = 408 rad/s = 65 Hz
    P.zeta_theta = 0.04;
    P.K_bend     = 4800.0;
    P.m_bend     = 0.09;      // -> w_bend = 231 rad/s = 37 Hz
    P.zeta_bend  = 0.02;
    P.S_theta    = 0.0;       // sin acoplo flexion-torsion (sin flameo)

    P.dt = 0.002;

    return P;
}

double qbar(const Params& P)
{
    return 0.5 * P.rho * P.V * P.V;
}

double v_eff(const Params& P)
{
    const double vmin = (P.V_min > 1e-9) ? P.V_min : 1e-9;
    return (P.V > vmin) ? P.V : vmin;
}

// ─────────────────────────────────────────────────────────────────────────────
// Estado y comandos
// ─────────────────────────────────────────────────────────────────────────────

State state_init()
{
    State s{};
    s.q[0] = 1.0; s.q[1] = 0.0; s.q[2] = 0.0; s.q[3] = 0.0;
    for (int i = 0; i < 3; ++i) s.omega[i] = 0.0;
    for (int i = 0; i < 4; ++i) {
        s.delta[i] = s.delta_cmd[i] = 0.0;
        s.theta[i] = s.theta_dot[i] = 0.0;
        s.w[i] = s.w_dot[i] = 0.0;
    }
    return s;
}

Commands commands_zero()
{
    Commands c{};
    c.u_pitch = c.u_yaw = c.u_roll = 0.0;
    for (int i = 0; i < 4; ++i) c.fin_trim[i] = 0.0;
    return c;
}

void apply_commands(const Commands& C, State& s)
{
    double d[4];
    mix_commands(C.u_pitch, C.u_yaw, C.u_roll, d);
    for (int i = 0; i < 4; ++i) s.delta_cmd[i] = d[i] + C.fin_trim[i];
}

void apply_commands_instant(const Commands& C, State& s)
{
    apply_commands(C, s);
    for (int i = 0; i < 4; ++i) s.delta[i] = s.delta_cmd[i];
}

// ─────────────────────────────────────────────────────────────────────────────
// Mixing  (cambio de base a las irreps de V4)
// ─────────────────────────────────────────────────────────────────────────────

void mix_commands(double u_pitch, double u_yaw, double u_roll, double delta_out[4])
{
    delta_out[0] = u_pitch + u_roll;   // d1  derecha
    delta_out[1] = u_pitch - u_roll;   // d2  izquierda
    delta_out[2] = u_yaw   + u_roll;   // d3  arriba
    delta_out[3] = u_yaw   - u_roll;   // d4  abajo
}

void demix(const double delta[4], double& u_pitch, double& u_yaw, double& u_roll)
{
    u_pitch = 0.5  * (delta[0] + delta[1]);
    u_yaw   = 0.5  * (delta[2] + delta[3]);
    u_roll  = 0.25 * (delta[0] - delta[1] + delta[2] - delta[3]);
}

// ─────────────────────────────────────────────────────────────────────────────
// Aerodinamica de superficie
// ─────────────────────────────────────────────────────────────────────────────

double fin_gain(const Params& P)
{
    return qbar(P) * P.S_fin * P.CL_delta;
}

void fin_forces(const Params& P, const double delta[4], double f[4])
{
    const double k = fin_gain(P);
    for (int i = 0; i < 4; ++i)
        f[i] = k * clamp(delta[i], -P.delta_stall, +P.delta_stall);
}

// §4 — incidencia inducida por la rotacion del cohete.
//
// Punto de aplicacion de cada aleta:  r1=(-l_t,+d,0)  r2=(-l_t,-d,0)
//                                     r3=(-l_t,0,-d)  r4=(-l_t,0,+d)
//
// v_i = omega x r_i.  Para el par horizontal (fuerza en +z) cuenta (v_i)_z,
// para el par vertical (fuerza en +y) cuenta (v_i)_y:
//
//     (omega x r)_z = p*r_y - q*r_x        (omega x r)_y = r*r_x - p*r_z
//
// Si la aleta se mueve hacia +z, el aire la ataca desde -z: la incidencia
// efectiva BAJA.  De ahi el signo negativo.
void fin_effective_deflection(const Params& P, const double delta[4],
                              const double theta[4], const double omega[3],
                              double delta_eff[4])
{
    // Lo que el servo pone en la raiz, mas lo que la aleta se retuerce (§5).
    const bool flex = (P.flex_on && theta != nullptr);
    for (int i = 0; i < 4; ++i)
        delta_eff[i] = delta[i] + (flex ? theta[i] : 0.0);

    if (!P.damping_on) return;

    const double p = omega[0], q = omega[1], r = omega[2];
    const double invV = 1.0 / v_eff(P);

    delta_eff[0] -= (q * P.l_t + p * P.d) * invV;   // d1 derecha
    delta_eff[1] -= (q * P.l_t - p * P.d) * invV;   // d2 izquierda
    delta_eff[2] += (r * P.l_t - p * P.d) * invV;   // d3 arriba
    delta_eff[3] += (r * P.l_t + p * P.d) * invV;   // d4 abajo
}

void fin_moments(const Params& P, const double delta[4], double M[3])
{
    double f[4];
    fin_forces(P, delta, f);

    M[0] =  P.d   * (f[0] - f[1] + f[2] - f[3]);   // L (roll)
    M[1] =  P.l_t * (f[0] + f[1]);                 // M (pitch)
    M[2] = -P.l_t * (f[2] + f[3]);                 // N (yaw)
}

void fin_moments_rate(const Params& P, const double delta[4],
                      const double theta[4], const double omega[3], double M[3])
{
    double delta_eff[4];
    fin_effective_deflection(P, delta, theta, omega, delta_eff);
    fin_moments(P, delta_eff, M);
}

// Linealizacion de lo anterior alrededor de omega = 0, sin perdida.
//
//   p:  d1 pierde k*p*d/V, d2 gana k*p*d/V  ->  Delta L = -2 k d^2 p / V
//       d3 pierde k*p*d/V, d4 gana k*p*d/V  ->  Delta L = -2 k d^2 p / V
//       ...y sus contribuciones a M y N se cancelan por pares.
//   q:  d1 y d2 pierden k*q*l_t/V por igual ->  Delta M = -2 k l_t^2 q / V
//       su contribucion a L se cancela (entran con signo opuesto en L).
//   r:  d3 y d4 ganan k*r*l_t/V por igual   ->  Delta N = -2 k l_t^2 r / V
//
// Cero terminos cruzados: la matriz de amortiguamiento es diagonal en la
// misma base de irreps que diagonaliza el mando.  Ese es el resultado de §4.
void damping_coeffs(const Params& P, double& L_p, double& M_q, double& N_r)
{
    if (!P.damping_on) { L_p = M_q = N_r = 0.0; return; }

    const double k    = fin_gain(P);
    const double invV = 1.0 / v_eff(P);

    L_p = -4.0 * k * P.d   * P.d   * invV;
    M_q = -2.0 * k * P.l_t * P.l_t * invV;
    N_r = M_q;
}

void control_effectiveness(const Params& P, double& L_du, double& M_du, double& N_du)
{
    const double k = fin_gain(P);

    L_du =  4.0 * k * P.d;
    M_du =  2.0 * k * P.l_t;
    N_du = -2.0 * k * P.l_t;   // convencion Blakelock: u_yaw > 0 -> N < 0
}

// ─────────────────────────────────────────────────────────────────────────────
// Aeroelasticidad (§5)
// ─────────────────────────────────────────────────────────────────────────────

// lambda = qbar S CL_delta e_ea / K_theta = k e_ea / K_theta.  Es el unico
// numero adimensional del problema aeroelastico estatico.
static double aeroelastic_lambda(const Params& P)
{
    if (!P.flex_on || P.K_theta <= 0.0) return 0.0;
    return fin_gain(P) * P.e_ea / P.K_theta;
}

double aeroelastic_efficiency(const Params& P)
{
    const double den = 1.0 + aeroelastic_lambda(P);
    // den <= 0 es la aleta divergida: el modelo estatico ya no significa nada.
    return (den > 1e-9) ? (1.0 / den) : 0.0;
}

double qbar_caracteristica(const Params& P)
{
    if (!P.flex_on) return 0.0;
    const double den = P.S_fin * P.CL_delta * std::fabs(P.e_ea);
    return (den > 1e-12) ? (P.K_theta / den) : 0.0;
}

bool aeroelastic_divergence(const Params& P)
{
    // Solo la aleta sobrebalanceada (e_ea < 0) diverge, y solo por encima de
    // qbar_car.  Con e_ea > 0 el lazo siempre resta, nunca suma.
    return P.flex_on && (P.e_ea < 0.0) && (1.0 + aeroelastic_lambda(P) <= 0.0);
}

void structural_freqs(const Params& P, double& w_bend, double& w_tors)
{
    w_bend = (P.m_bend  > 1e-12) ? std::sqrt(P.K_bend  / P.m_bend)  : 0.0;
    w_tors = (P.I_theta > 1e-12) ? std::sqrt(P.K_theta / P.I_theta) : 0.0;
}

double structural_cfl(const Params& P)
{
    if (!P.flex_on) return 0.0;
    double wb, wt;
    structural_freqs(P, wb, wt);
    return std::fmax(wb, wt) * P.dt;
}

bool structural_unstable(const Params& P)
{
    // 2.0 y no 2.83: el margen que queda entre ambos ya no es simulacion, es
    // una oscilacion numerica que parece fisica y no lo es.
    return structural_cfl(P) > 2.0;
}

void fin_structural_accel(const Params& P, double f,
                          double theta, double theta_dot,
                          double w, double w_dot,
                          double& theta_dd, double& w_dd)
{
    theta_dd = w_dd = 0.0;
    if (!P.flex_on) return;

    // Amortiguamiento estructural en forma de fraccion del critico.
    const double c_w  = 2.0 * P.zeta_bend  * std::sqrt(std::fmax(P.K_bend  * P.m_bend,  0.0));
    const double c_th = 2.0 * P.zeta_theta * std::sqrt(std::fmax(P.K_theta * P.I_theta, 0.0));

    // Momento de la carga sobre el pivote.  Con e_ea > 0 (centro aerodinamico
    // por detras) tiende a destorcer: se opone a la incidencia que lo creo.
    const double H = -P.e_ea * f;

    const double bw = f - c_w  * w_dot     - P.K_bend  * w;
    const double bt = H - c_th * theta_dot - P.K_theta * theta;

    // Inversion de la matriz de masa 2x2.  S_theta es el unico acoplo.
    const double det = P.m_bend * P.I_theta - P.S_theta * P.S_theta;
    if (std::fabs(det) < 1e-15) return;      // matriz degenerada: no integres

    w_dd     = (P.I_theta * bw - P.S_theta * bt) / det;
    theta_dd = (P.m_bend  * bt - P.S_theta * bw) / det;
}

// ─────────────────────────────────────────────────────────────────────────────
// Actuador
// ─────────────────────────────────────────────────────────────────────────────

void servo_step(const Params& P, State& s, double dt)
{
    for (int i = 0; i < 4; ++i) {
        double target = s.delta_cmd[i];

        // Retardo de primer orden:  delta' = (cmd - delta)/tau.
        // Se integra en forma exacta para no depender de dt << tau.
        if (P.tau_servo > 1e-9) {
            const double a = 1.0 - std::exp(-dt / P.tau_servo);
            target = s.delta[i] + a * (s.delta_cmd[i] - s.delta[i]);
        }

        // Limite de velocidad: se aplica al desplazamiento total del paso.
        if (P.rate_limit > 1e-9) {
            const double dmax = P.rate_limit * dt;
            target = s.delta[i] + clamp(target - s.delta[i], -dmax, +dmax);
        }

        s.delta[i] = target;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Dinamica y cinematica
// ─────────────────────────────────────────────────────────────────────────────

void rotational_dynamics(const Params& P, const double w[3],
                         const double M[3], double wdot[3])
{
    // I * omega  con  I = [[Ixx,0,-Ixz],[0,Iyy,0],[-Ixz,0,Izz]]
    const double Iw[3] = {
        P.Ixx * w[0] - P.Ixz * w[2],
        P.Iyy * w[1],
        -P.Ixz * w[0] + P.Izz * w[2]
    };

    // omega x (I omega)  — termino giroscopico
    const double gyro[3] = {
        w[1] * Iw[2] - w[2] * Iw[1],
        w[2] * Iw[0] - w[0] * Iw[2],
        w[0] * Iw[1] - w[1] * Iw[0]
    };

    const double b[3] = { M[0] - gyro[0], M[1] - gyro[1], M[2] - gyro[2] };

    // I^-1 en el bloque xz:  1/Gamma * [[Izz, Ixz],[Ixz, Ixx]],
    // con Gamma = Ixx*Izz - Ixz^2.  El eje y queda desacoplado.
    const double Gamma = P.Ixx * P.Izz - P.Ixz * P.Ixz;

    if (std::fabs(Gamma) < 1e-12) {          // tensor degenerado: no integres
        wdot[0] = wdot[1] = wdot[2] = 0.0;
        return;
    }

    wdot[0] = (P.Izz * b[0] + P.Ixz * b[2]) / Gamma;
    wdot[1] =  b[1] / P.Iyy;
    wdot[2] = (P.Ixz * b[0] + P.Ixx * b[2]) / Gamma;
}

void quat_kinematics(const double q[4], const double w[3], double qdot[4])
{
    const double e0 = q[0], e1 = q[1], e2 = q[2], e3 = q[3];
    const double p = w[0], qq = w[1], r = w[2];

    qdot[0] = 0.5 * (-p * e1 - qq * e2 - r * e3);
    qdot[1] = 0.5 * ( p * e0 + r  * e2 - qq * e3);
    qdot[2] = 0.5 * (qq * e0 - r  * e1 + p  * e3);
    qdot[3] = 0.5 * ( r * e0 + qq * e1 - p  * e2);
}

void quat_normalize(double q[4])
{
    double n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n > 1e-12) { for (int i = 0; i < 4; ++i) q[i] /= n; }
    else           { q[0] = 1.0; q[1] = q[2] = q[3] = 0.0; }
}

// Disposicion del vector de estado del integrador.
//
//   rigido (N_RIG = 7):   [0..3] cuaternion   [4..6] omega
//   flexible (N_FLEX = 23): + [7..10] theta   [11..14] theta_dot
//                             [15..18] w      [19..22] w_dot
//
// Que la version rigida use solo 7 no es una optimizacion cosmetica: garantiza
// que con flex_on = false el modelo de §1-§4 se reproduce bit a bit.
enum { N_RIG = 7, N_FLEX = 23, IT = 7, ITD = 11, IW = 15, IWD = 19 };

// Derivada del estado.  La deflexion de RAIZ esta congelada durante el paso
// (el servo va fuera del RK4), pero los momentos se reevaluan en cada etapa:
// dependen de omega (§4) y de la torsion elastica (§5).
static void derivative(const Params& P, const double delta[4],
                       const double y[], double dy[], bool flex)
{
    const double* theta = flex ? (y + IT) : nullptr;

    double delta_eff[4], f[4], M[3];
    fin_effective_deflection(P, delta, theta, y + 4, delta_eff);
    fin_forces(P, delta_eff, f);

    M[0] =  P.d   * (f[0] - f[1] + f[2] - f[3]);
    M[1] =  P.l_t * (f[0] + f[1]);
    M[2] = -P.l_t * (f[2] + f[3]);

    quat_kinematics(y, y + 4, dy);
    rotational_dynamics(P, y + 4, M, dy + 4);

    if (!flex) return;

    // Cada aleta responde a SU propia fuerza: cuatro sistemas de 2 gdl.
    for (int i = 0; i < 4; ++i) {
        double theta_dd, w_dd;
        fin_structural_accel(P, f[i], y[IT + i], y[ITD + i], y[IW + i], y[IWD + i],
                             theta_dd, w_dd);
        dy[IT  + i] = y[ITD + i];
        dy[ITD + i] = theta_dd;
        dy[IW  + i] = y[IWD + i];
        dy[IWD + i] = w_dd;
    }
}

void step_rk4(const Params& P, State& s, double dt)
{
    const bool flex = P.flex_on;
    const int  n    = flex ? N_FLEX : N_RIG;

    double y[N_FLEX] = { s.q[0], s.q[1], s.q[2], s.q[3],
                         s.omega[0], s.omega[1], s.omega[2] };
    if (flex) {
        for (int i = 0; i < 4; ++i) {
            y[IT  + i] = s.theta[i];
            y[ITD + i] = s.theta_dot[i];
            y[IW  + i] = s.w[i];
            y[IWD + i] = s.w_dot[i];
        }
    }

    double k1[N_FLEX], k2[N_FLEX], k3[N_FLEX], k4[N_FLEX], tmp[N_FLEX];

    derivative(P, s.delta, y, k1, flex);
    for (int i = 0; i < n; ++i) tmp[i] = y[i] + 0.5 * dt * k1[i];
    derivative(P, s.delta, tmp, k2, flex);
    for (int i = 0; i < n; ++i) tmp[i] = y[i] + 0.5 * dt * k2[i];
    derivative(P, s.delta, tmp, k3, flex);
    for (int i = 0; i < n; ++i) tmp[i] = y[i] + dt * k3[i];
    derivative(P, s.delta, tmp, k4, flex);

    for (int i = 0; i < n; ++i)
        y[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);

    for (int i = 0; i < 4; ++i) s.q[i] = y[i];
    for (int i = 0; i < 3; ++i) s.omega[i] = y[4 + i];

    if (flex) {
        for (int i = 0; i < 4; ++i) {
            s.theta[i]     = y[IT  + i];
            s.theta_dot[i] = y[ITD + i];
            s.w[i]         = y[IW  + i];
            s.w_dot[i]     = y[IWD + i];
        }
    }

    quat_normalize(s.q);
}

// ─────────────────────────────────────────────────────────────────────────────
// Autopiloto
// ─────────────────────────────────────────────────────────────────────────────

PID pid_zero()
{
    PID c{};
    c.kp = c.ki = c.kd = 0.0;
    c.i_state = 0.0;
    c.i_max   = 0.0;
    c.u_max   = 0.0;
    return c;
}

double pid_update(PID& c, double err, double rate, double dt)
{
    c.i_state += err * dt;
    if (c.i_max > 0.0) c.i_state = clamp(c.i_state, -c.i_max, +c.i_max);

    double u = c.kp * err + c.ki * c.i_state - c.kd * rate;

    if (c.u_max > 0.0) {
        const double u_sat = clamp(u, -c.u_max, +c.u_max);
        // Anti-windup: si la salida satura, deshaz la parte de integral que
        // no ha llegado a la planta.
        if (u_sat != u && std::fabs(c.ki) > 1e-12)
            c.i_state += (u_sat - u) / c.ki;
        u = u_sat;
    }
    return u;
}

// Asignacion de polos sobre  I*x_dd = C_du*u + C_rate*x_d,  con
// u = kp*err - kd*x_d.  Ver la deduccion en el header.
static void tune_channel(PID& c, double I, double C_du, double C_rate,
                         double wn, double zeta, double u_max)
{
    c = pid_zero();
    if (std::fabs(C_du) < 1e-12) return;

    c.kp = (wn * wn * I) / C_du;
    c.kd = (2.0 * zeta * wn * I + C_rate) / C_du;   // C_rate <= 0
    c.ki = c.kp * wn / 5.0;                         // trim lento, sin dominar

    c.u_max   = u_max;
    c.i_max   = (std::fabs(c.ki) > 1e-12) ? u_max / std::fabs(c.ki) : 0.0;
    c.i_state = 0.0;
}

void autopilot_tune(const Params& P, Autopilot& A)
{
    double L_p, M_q, N_r, L_du, M_du, N_du;
    damping_coeffs(P, L_p, M_q, N_r);
    control_effectiveness(P, L_du, M_du, N_du);

    // La aleta flexible entrega solo una fraccion eta del mando (§5), y por la
    // misma razon amortigua solo eta.  Los modos estructurales (>30 Hz) son
    // mil veces mas rapidos que el lazo de actitud (~1 Hz), asi que para
    // sintonizar basta con su valor cuasi-estatico.  Si no se corrige, el
    // autopiloto queda sistematicamente lento al subir la velocidad.
    const double eta = aeroelastic_efficiency(P);
    L_du *= eta; M_du *= eta; N_du *= eta;
    L_p  *= eta; M_q  *= eta; N_r  *= eta;

    // Deja margen: el mixing suma pitch/yaw y roll sobre la misma aleta, asi
    // que cada canal se queda por debajo de la perdida.
    const double u_max = 0.7 * P.delta_stall;

    tune_channel(A.roll,  P.Ixx, L_du, L_p, A.wn_roll,  A.zeta, u_max);
    tune_channel(A.pitch, P.Iyy, M_du, M_q, A.wn_pitch, A.zeta, u_max);
    tune_channel(A.yaw,   P.Izz, N_du, N_r, A.wn_yaw,   A.zeta, u_max);
}

Autopilot autopilot_default(const Params& P)
{
    Autopilot A{};
    A.enabled   = false;
    A.phi_ref   = A.theta_ref = A.psi_ref = 0.0;
    A.wn_roll   = 3.0;    // el alabeo tiene mucha menos inercia: mas rapido
    A.wn_pitch  = 1.5;
    A.wn_yaw    = 1.5;
    A.zeta      = 0.8;
    autopilot_tune(P, A);
    return A;
}

void autopilot_update(const Params& P, Autopilot& A, const State& s,
                      Commands& C, double dt)
{
    (void)P;
    if (!A.enabled) return;

    double phi, th, psi;
    euler_from_quat(s.q, phi, th, psi);

    // Tres lazos SISO independientes: eso es lo que compra la simetria V4.
    C.u_roll  = pid_update(A.roll,  wrap_pi(A.phi_ref   - phi), s.omega[0], dt);
    C.u_pitch = pid_update(A.pitch, wrap_pi(A.theta_ref - th),  s.omega[1], dt);
    C.u_yaw   = pid_update(A.yaw,   wrap_pi(A.psi_ref   - psi), s.omega[2], dt);
}

// ─────────────────────────────────────────────────────────────────────────────
// Utilidades de actitud
// ─────────────────────────────────────────────────────────────────────────────

double wrap_pi(double a)
{
    while (a >   M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

void euler_from_quat(const double q[4], double& phi, double& th, double& psi)
{
    const double e0 = q[0], e1 = q[1], e2 = q[2], e3 = q[3];

    phi = std::atan2(2.0 * (e0*e1 + e2*e3), 1.0 - 2.0 * (e1*e1 + e2*e2));

    double sth = 2.0 * (e0*e2 - e3*e1);
    sth = clamp(sth, -1.0, 1.0);          // protege asin() en gimbal lock
    th  = std::asin(sth);

    psi = std::atan2(2.0 * (e0*e3 + e1*e2), 1.0 - 2.0 * (e2*e2 + e3*e3));
}

void rotmat_from_quat(const double q[4], double R[9])
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];

    R[0] = 1.0 - 2.0*(y*y + z*z);  R[1] = 2.0*(x*y - w*z);        R[2] = 2.0*(x*z + w*y);
    R[3] = 2.0*(x*y + w*z);        R[4] = 1.0 - 2.0*(x*x + z*z);  R[5] = 2.0*(y*z - w*x);
    R[6] = 2.0*(x*z - w*y);        R[7] = 2.0*(y*z + w*x);        R[8] = 1.0 - 2.0*(x*x + y*y);
}

} // namespace alerones
