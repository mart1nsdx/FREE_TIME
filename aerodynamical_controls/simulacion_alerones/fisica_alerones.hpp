// ═══════════════════════════════════════════════════════════════════════════
//  fisica_alerones.hpp — Modelo fisico del control aerodinamico de las cuatro
//  aletas traseras (arreglo cruciforme) de un cohete.
//
//  Sin dependencias graficas: este header se puede compilar y testear en
//  aislamiento (ver tests.cpp).  Ver README.md para la deduccion completa.
//
//  Ejes cuerpo (Blakelock Fig. 8-2):  x adelante, y derecha, z abajo.
//
//  Numeracion de aletas (vista de cola) y convencion de signo:
//
//      Aleta   Posicion   Offset radial      Fuerza para delta > 0
//      -----   --------   ----------------   ---------------------
//      d1      derecha    (x_t, +d,  0)      +z   (abajo)
//      d2      izquierda  (x_t, -d,  0)      +z   (abajo)
//      d3      arriba     (x_t,  0, -d)      +y   (derecha)
//      d4      abajo      (x_t,  0, +d)      +y   (derecha)
//
//  con x_t = -l_t  (las aletas van DETRAS del CG, brazo l_t > 0).
//
//  Simetria: el arreglo tiene grupo V4 = Z2 x Z2, generado por las reflexiones
//  sigma_v (plano xz, intercambia el par horizontal d1<->d2) y sigma_h (plano
//  xy, intercambia el par vertical d3<->d4).  La matriz de mixing no es un
//  generador del grupo: es el cambio de base que descompone la representacion
//  permutacional en irreps.  Los eigenvectores +1 (simetricos) dan pitch/yaw;
//  los -1 (antisimetricos) dan roll.  De ahi el desacoplo exacto de §2.
//
//  El amortiguamiento aerodinamico (§4) respeta ese mismo desacoplo: la matriz
//  de derivadas de estabilidad es DIAGONAL en la base de irreps (L_p, M_q, N_r
//  sin terminos cruzados).  Es el resultado central del modulo.
// ═══════════════════════════════════════════════════════════════════════════
#ifndef FISICA_ALERONES_HPP
#define FISICA_ALERONES_HPP

namespace alerones {

// ─────────────────────────────────────────────────────────────────────────────
// Parametros del modelo
// ─────────────────────────────────────────────────────────────────────────────
struct Params {
    // Aire y velocidad de vuelo
    double rho;          // densidad del aire            [kg/m^3]
    double V;            // velocidad de vuelo           [m/s]
    double V_min;        // piso para el cociente 1/V    [m/s]  (evita 1/0)

    // Aerodinamica de la aleta
    double S_fin;        // superficie de una aleta      [m^2]
    double b_fin;        // envergadura de una aleta     [m]   (solo para AR)
    double CL_delta;     // derivada de control          [1/rad]

    // Geometria del arreglo
    double l_t;          // brazo longitudinal CG->aletas [m]  (positivo)
    double d;            // offset radial del centro de presion de la aleta [m]

    // Limite de deflexion (perdida)
    double delta_stall;  // [rad]

    // Tensor de inercia en ejes cuerpo, simetria XZ:
    //     I = [ Ixx   0   -Ixz ]
    //         [  0   Iyy    0  ]
    //         [-Ixz   0    Izz ]
    double Ixx, Iyy, Izz, Ixz;   // [kg m^2]   (Ixz = 0 -> simetria perfecta)

    // Amortiguamiento aerodinamico por velocidad angular (§4)
    bool   damping_on;   // false reproduce el modelo sin amortiguar

    // Dinamica del actuador (§6).  <= 0 desactiva cada efecto.
    double tau_servo;    // constante de tiempo de primer orden [s]
    double rate_limit;   // velocidad maxima de deflexion       [rad/s]

    // ── Aeroelasticidad (§5): la aleta deja de ser rigida ───────────────────
    bool   flex_on;      // false -> aleta rigida (modelo de §1-§4 intacto)

    // Distancia del PIVOTE al CENTRO AERODINAMICO, positiva hacia ATRAS.
    // e_ea > 0 (diseno normal): la carga tiende a DESTORCER la aleta -> se
    //          pierde eficacia de mando al subir qbar.  Siempre estable.
    // e_ea < 0 (sobrebalanceada): la carga AUMENTA la incidencia -> hay una
    //          presion dinamica de DIVERGENCIA a la que la aleta se rompe.
    double e_ea;         // [m]

    // Modo de torsion alrededor del pivote (eje + rigidez del servo)
    double K_theta;      // rigidez torsional          [N*m/rad]
    double I_theta;      // inercia torsional          [kg*m^2]
    double zeta_theta;   // amortiguamiento estructural [-]

    // Modo de flexion (voladizo, carga en la punta)
    double K_bend;       // rigidez a flexion          [N/m]
    double m_bend;       // masa generalizada del modo [kg]
    double zeta_bend;    // amortiguamiento estructural [-]

    // Desequilibrio estatico entre los dos modos: es el UNICO termino que los
    // acopla.  Con S_theta = 0 flexion y torsion son independientes; distinto
    // de cero abre la puerta al flameo (flutter) binario clasico.
    double S_theta;      // [kg*m]

    // Integracion
    double dt;           // paso fijo [s]
};

// Valores por defecto: cohete/UAV de 200 kg, inercias de buildDeltaWingDrone()
// en unmanned/physics_UAV.h  (Ixx=16.06, Iyy=187.68, Izz=202.00, Ixz~0).
Params params_default();

// Recalcula CL_delta = CL_alpha(AR(b_fin, S_fin)).  Hay que llamarla despues de
// tocar b_fin o S_fin en caliente, o AR y CL_delta quedan incoherentes.
void params_update_aero(Params& P);

// Presion dinamica  qbar = 1/2 rho V^2   [Pa].  La autoridad de control cae
// con el cuadrado de la velocidad.
double qbar(const Params& P);

// Velocidad efectiva para los cocientes 1/V:  max(V, V_min).
double v_eff(const Params& P);

// Pendiente de sustentacion de un ala finita (Helmbold, valido a AR bajo):
//     CL_alpha = 2*pi*AR / (2 + sqrt(AR^2 + 4))     [1/rad]
// Para una aleta "all-moving" (toda la superficie gira) la deflexion ES el
// angulo de ataque, luego CL_delta = CL_alpha.
double cl_alpha_helmbold(double AR);

// Alargamiento de la aleta: AR = b^2 / S
double aspect_ratio(double b_fin, double S_fin);

// ─────────────────────────────────────────────────────────────────────────────
// Estado
// ─────────────────────────────────────────────────────────────────────────────
struct State {
    double q[4];          // cuaternion de actitud (w,x,y,z), cuerpo -> inercial
    double omega[3];      // p, q, r   [rad/s]
    double delta[4];      // deflexiones REALES d1..d4        [rad]  (raiz)
    double delta_cmd[4];  // deflexiones COMANDADAS d1..d4    [rad]

    // Estado elastico de cada aleta (§5).  delta es lo que impone el servo en
    // la RAIZ; theta es lo que la aleta se retuerce por encima de eso, y es la
    // suma de los dos lo que ve el aire.
    double theta[4];      // torsion elastica          [rad]
    double theta_dot[4];  //                           [rad/s]
    double w[4];          // flecha en la punta        [m]
    double w_dot[4];      //                           [m/s]
};

// Estado inicial: actitud identidad, rates y deflexiones a cero.
State state_init();

// ─────────────────────────────────────────────────────────────────────────────
// Comandos: canales virtuales (irreps) + trim directo por aleta
// ─────────────────────────────────────────────────────────────────────────────
struct Commands {
    double u_pitch;    // [rad]  irrep simetrica del par horizontal
    double u_yaw;      // [rad]  irrep simetrica del par vertical
    double u_roll;     // [rad]  irrep antisimetrica de las cuatro
    double fin_trim[4];// [rad]  deflexion directa adicional por aleta
};

Commands commands_zero();

// delta_cmd = mix(u_pitch, u_yaw, u_roll) + fin_trim.
// El actuador (servo_step) es quien lleva delta hacia delta_cmd.
void apply_commands(const Commands& C, State& s);

// Fuerza delta = delta_cmd de golpe (actuador ideal).  Util en tests y al
// inicializar, para no arrastrar el transitorio del servo.
void apply_commands_instant(const Commands& C, State& s);

// ─────────────────────────────────────────────────────────────────────────────
// Mixing (comandos -> aletas) y su inversa (proyeccion sobre irreps)
//
//   / d1 \   / 1  0   1 \ / u_pitch \        u_pitch = (d1 + d2)/2
//   | d2 | = | 1  0  -1 | | u_yaw   |        u_yaw   = (d3 + d4)/2
//   | d3 |   | 0  1   1 | \ u_roll  /        u_roll  = (d1-d2+d3-d4)/4
//   \ d4 /   \ 0  1  -1 /
// ─────────────────────────────────────────────────────────────────────────────
void mix_commands(double u_pitch, double u_yaw, double u_roll, double delta_out[4]);
void demix(const double delta[4], double& u_pitch, double& u_yaw, double& u_roll);

// ─────────────────────────────────────────────────────────────────────────────
// Aerodinamica
// ─────────────────────────────────────────────────────────────────────────────

// Fuerza de cada aleta, con saturacion por perdida:
//     f_i = qbar * S_fin * CL_delta * clamp(delta_i, +-delta_stall)
// Fuera de +-delta_stall la fuerza mantiene el valor de saturacion
// (modelo post-perdida simple).  El clamp vive aqui, NO en el mixing.
void fin_forces(const Params& P, const double delta[4], double f[4]);

// Ganancia de la aleta:  k = qbar * S_fin * CL_delta   [N/rad]
double fin_gain(const Params& P);

// §4 — Incidencia efectiva de cada aleta cuando el cohete gira.
//
// La velocidad del punto de aplicacion de la aleta es v_i = omega x r_i.  Su
// componente normal a la aleta se suma al angulo de ataque:
//
//     d1:  delta - ( q*l_t + p*d) / V        d3:  delta + ( r*l_t - p*d) / V
//     d2:  delta - ( q*l_t - p*d) / V        d4:  delta + ( r*l_t + p*d) / V
//
// A eso se le suma la torsion elastica theta de §5, si la hay.  El clamp por
// perdida se aplica DESPUES, sobre delta_eff: una aleta puede entrar en
// perdida por la rotacion del cohete o por su propia torsion, aunque el mando
// sea pequeno.
//
// theta puede ser nullptr: entonces la aleta se trata como rigida.
void fin_effective_deflection(const Params& P, const double delta[4],
                              const double theta[4], const double omega[3],
                              double delta_eff[4]);

// Momentos sobre el CG SIN amortiguamiento (omega = 0).  Este es el desacoplo
// impuesto por la simetria:
//     L = d  * (f1 - f2 + f3 - f4)     <- solo el modo antisimetrico
//     M = l_t* (f1 + f2)               <- solo el par horizontal simetrico
//     N = -l_t*(f3 + f4)               <- solo el par vertical simetrico
void fin_moments(const Params& P, const double delta[4], double M[3]);

// Momentos incluyendo el amortiguamiento por velocidad angular y la torsion
// elastica.  Con P.damping_on = false y theta = nullptr es identico a
// fin_moments().
void fin_moments_rate(const Params& P, const double delta[4],
                      const double theta[4], const double omega[3], double M[3]);

// Derivadas de estabilidad linealizadas (regimen sin perdida), §4:
//     L_p = -4 k d^2   / V        M_q = -2 k l_t^2 / V        N_r = M_q
// Son NEGATIVAS (amortiguan) y la matriz es diagonal: la simetria V4 que
// desacopla el mando desacopla tambien la respuesta a la rotacion.
void damping_coeffs(const Params& P, double& L_p, double& M_q, double& N_r);

// Autoridad de control linealizada [N*m/rad] sobre cada canal virtual:
//     L_du = +4 k d       M_du = +2 k l_t       N_du = -2 k l_t
// Ojo al signo de N_du: es la convencion de Blakelock (u_yaw > 0 -> N < 0).
void control_effectiveness(const Params& P, double& L_du, double& M_du, double& N_du);

// ─────────────────────────────────────────────────────────────────────────────
// Aeroelasticidad (§5)
//
// Cada aleta es un sistema de DOS grados de libertad excitado por su propia
// fuerza aerodinamica:
//
//   [ m_bend   S_theta ] [ w'' ]   [ f - c_w  w'     - K_bend w ]
//   [ S_theta  I_theta ] [ th''] = [ H - c_th th'    - K_theta th]
//
//   con  H = -e_ea * f     (momento de la carga sobre el pivote)
//        c_w  = 2 zeta_bend  sqrt(K_bend  m_bend)
//        c_th = 2 zeta_theta sqrt(K_theta I_theta)
//
// La FLEXION w no cambia la incidencia (aleta recta, sin flecha): es el modo
// que se ve, no el que manda.  La TORSION th SI entra en el angulo de ataque,
// y ahi se cierra el lazo aeroelastico:
//
//     f = k (delta + th)        K_theta th = -e_ea f
//  => delta_eff = delta / (1 + lambda),     lambda = qbar S CL_delta e_ea / K_theta
//
// De donde la eficiencia aeroelastica y la presion dinamica caracteristica:
//
//     eta       = 1 / (1 + qbar/qbar_car)   con   qbar_car = K_theta/(S CL_delta |e_ea|)
//     e_ea > 0  ->  eta < 1, cae como 1/qbar:  perdida de eficacia de mando
//     e_ea < 0  ->  eta > 1 y DIVERGE en qbar = qbar_car
// ─────────────────────────────────────────────────────────────────────────────

// Eficiencia aeroelastica estatica: cuanto del mando llega de verdad al aire.
// Vale 1 exacto con flex_on = false o e_ea = 0.
double aeroelastic_efficiency(const Params& P);

// Presion dinamica caracteristica  qbar_car = K_theta / (S CL_delta |e_ea|).
// Con e_ea > 0 es donde eta = 1/2; con e_ea < 0 es la de divergencia.
// Devuelve 0 si no aplica (aleta rigida o e_ea = 0).
double qbar_caracteristica(const Params& P);

// true si la aleta esta divergida (e_ea < 0 y qbar >= qbar_car): a partir de
// ahi el lazo aeroelastico es inestable y el modelo lineal no dice nada.
bool aeroelastic_divergence(const Params& P);

// Frecuencias naturales de los dos modos estructurales, en vacio [rad/s].
void structural_freqs(const Params& P, double& w_bend, double& w_tors);

// max(w_bend, w_tors) * dt — el numero de estabilidad del integrador.
//
// Los modos estructurales son cientos de veces mas rapidos que la actitud del
// cohete, y son ELLOS quienes fijan el paso.  RK4 es inestable por encima de
// 2.83 y ya es impreciso bastante antes.  Vale 0 con la aleta rigida, que es
// justamente por lo que §1-§4 podian permitirse dt = 10 ms.
double structural_cfl(const Params& P);

// true si el paso actual no puede resolver los modos: la simulacion va a
// divergir a NaN.  Conviene avisar antes, no despues.
bool structural_unstable(const Params& P);

// Aceleraciones estructurales de UNA aleta sometida a la fuerza f.
void fin_structural_accel(const Params& P, double f,
                          double theta, double theta_dot,
                          double w, double w_dot,
                          double& theta_dd, double& w_dd);

// ─────────────────────────────────────────────────────────────────────────────
// Actuador (§6)
// ─────────────────────────────────────────────────────────────────────────────

// Lleva delta hacia delta_cmd durante dt: primero limite de velocidad, luego
// retardo de primer orden.  Con tau_servo <= 0 y rate_limit <= 0 es un salto.
void servo_step(const Params& P, State& s, double dt);

// ─────────────────────────────────────────────────────────────────────────────
// Dinamica y cinematica
// ─────────────────────────────────────────────────────────────────────────────

// Dinamica de Euler con tensor constante en ejes cuerpo (simetria XZ, Ixz):
//     omega_dot = I^-1 (M - omega x I omega)
void rotational_dynamics(const Params& P, const double omega[3],
                         const double M[3], double omega_dot[3]);

// Cinematica del cuaternion:  e_dot = 1/2 Omega(omega) e
void quat_kinematics(const double q[4], const double omega[3], double q_dot[4]);

// Un paso de RK4 con la deflexion de RAIZ congelada durante el paso (el servo
// va fuera).  Los momentos SI se reevaluan en cada etapa: dependen de omega
// (§4) y de la torsion elastica (§5).  Renormaliza el cuaternion al final.
//
// Con flex_on integra 23 estados (q, omega, theta, theta_dot, w, w_dot); sin
// el, solo 7 — asi la aleta rigida reproduce §1-§4 bit a bit.
void step_rk4(const Params& P, State& s, double dt);

// ─────────────────────────────────────────────────────────────────────────────
// Autopiloto (§6): tres lazos SISO, uno por irrep.
//
// El desacoplo de §2 y la diagonalidad del amortiguamiento de §4 son lo que
// permite sintonizar tres PID independientes en vez de un controlador MIMO.
// ─────────────────────────────────────────────────────────────────────────────
struct PID {
    double kp, ki, kd;   // ganancias  (pueden ser negativas: ver N_du)
    double i_state;      // integral del error [rad*s]
    double i_max;        // anti-windup sobre i_state
    double u_max;        // saturacion de la salida [rad]
};

PID pid_zero();

// u = kp*err + ki*I(err) - kd*rate.   La derivada se toma de la VELOCIDAD
// medida, no de d(err)/dt: asi un salto en la referencia no da patada.
double pid_update(PID& c, double err, double rate, double dt);

struct Autopilot {
    bool   enabled;
    double phi_ref, theta_ref, psi_ref;   // referencias de actitud [rad]
    double wn_roll, wn_pitch, wn_yaw;     // frecuencias naturales deseadas [rad/s]
    double zeta;                          // amortiguamiento deseado [-]
    PID    roll, pitch, yaw;
};

// Autopiloto desactivado, referencias a cero, sintonizado para P.
Autopilot autopilot_default(const Params& P);

// Asignacion de polos sobre el modelo linealizado de §4.  Para el canal de
// cabeceo (los otros dos son identicos cambiando I y las derivadas):
//
//     Iyy*theta_dd = M_du*u + M_q*q       u = kp*err - kd*q
//  => kp = wn^2 * Iyy / M_du              kd = (2*zeta*wn*Iyy + M_q) / M_du
//
// M_q ya es negativo, de ahi el "+": el controlador solo aporta el
// amortiguamiento que la aerodinamica no da.  ki se toma como kp*wn/5.
void autopilot_tune(const Params& P, Autopilot& A);

// Un paso del autopiloto: lee la actitud de s, escribe u_pitch/u_yaw/u_roll
// en C.  No toca fin_trim (el trim manual sigue disponible).
void autopilot_update(const Params& P, Autopilot& A, const State& s,
                      Commands& C, double dt);

// ─────────────────────────────────────────────────────────────────────────────
// Utilidades de actitud
// ─────────────────────────────────────────────────────────────────────────────

// Angulos de Euler 3-2-1 (yaw-pitch-roll) a partir del cuaternion.
void euler_from_quat(const double q[4], double& phi, double& th, double& psi);

// Matriz de rotacion cuerpo -> inercial, fila-mayor R[3][3].
void rotmat_from_quat(const double q[4], double R[9]);

void quat_normalize(double q[4]);

// Envuelve un angulo a (-pi, pi].  Necesario en el error del autopiloto.
double wrap_pi(double a);

} // namespace alerones

#endif // FISICA_ALERONES_HPP
