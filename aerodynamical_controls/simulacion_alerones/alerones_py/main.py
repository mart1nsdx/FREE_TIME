#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simulacion_alerones — prototipo de referencia en Python (§8).

Mismo modelo fisico que fisica_alerones.cpp: sirve como referencia numerica
independiente.  Los casos de validacion de §7 deben dar exactamente lo mismo
que la version C++ (`make test` vs `python3 alerones_py/main.py --test`).

Uso:
    python3 main.py            animacion 3D interactiva (matplotlib)
    python3 main.py --test     casos de validacion de §7
    python3 main.py --repl     REPL equivalente al modo --headless de C++

Ejes cuerpo (Blakelock Fig. 8-2): x adelante, y derecha, z abajo.

    Aleta   Posicion    Offset radial     Fuerza para delta > 0
    d1      derecha     (x_t, +d,  0)     +z
    d2      izquierda   (x_t, -d,  0)     +z
    d3      arriba      (x_t,  0, -d)     +y
    d4      abajo       (x_t,  0, +d)     +y
"""

import sys
import math
from dataclasses import dataclass, field, replace

import numpy as np

DEG = 180.0 / math.pi
RAD = math.pi / 180.0


# ─────────────────────────────────────────────────────────────────────────────
# Parametros
# ─────────────────────────────────────────────────────────────────────────────

def aspect_ratio(b_fin, S_fin):
    return (b_fin * b_fin) / S_fin if S_fin > 0 else 0.0


def cl_alpha_helmbold(AR):
    """Helmbold: interpola entre linea sustentadora (AR alto) y ala esbelta."""
    return (2.0 * math.pi * AR) / (2.0 + math.sqrt(AR * AR + 4.0))


@dataclass
class Params:
    rho: float = 1.225
    V: float = 30.0
    V_min: float = 1.0             # piso para los cocientes 1/V

    b_fin: float = 0.25
    S_fin: float = 0.25 * 0.16
    CL_delta: float = 0.0          # se rellena en __post_init__

    l_t: float = 1.75
    d: float = 0.16 + 0.5 * 0.25

    delta_stall: float = 15.0 * RAD

    # Inercias de buildDeltaWingDrone() — unmanned/physics_UAV.h
    # I = [[Ixx, 0, -Ixz], [0, Iyy, 0], [-Ixz, 0, Izz]]
    Ixx: float = 16.0629
    Iyy: float = 187.6754
    Izz: float = 201.9983
    Ixz: float = 0.0               # 0 = simetria XZ perfecta

    damping_on: bool = True        # §4
    tau_servo: float = 0.05        # §6, <= 0 desactiva
    rate_limit: float = 240.0 * RAD

    # ── Aeroelasticidad (§5): la aleta deja de ser rigida ───────────────────
    flex_on: bool = True           # False -> modelo de §1-§4 intacto
    # Distancia del PIVOTE al CENTRO AERODINAMICO, positiva hacia ATRAS.
    #   e_ea > 0 (normal): la carga destuerce -> se pierde eficacia con qbar.
    #   e_ea < 0 (sobrebalanceada): la carga aumenta la incidencia -> diverge.
    e_ea: float = 0.02
    # Torsion alrededor del pivote (eje + rigidez del servo)
    K_theta: float = 250.0         # [N*m/rad]
    I_theta: float = 1.5e-3        # [kg*m^2]  -> 408 rad/s = 65 Hz
    zeta_theta: float = 0.04
    # Flexion (voladizo, carga en la punta): K = 3EI/L^3 de una placa de 3 mm
    K_bend: float = 4800.0         # [N/m]
    m_bend: float = 0.09           # [kg]      -> 231 rad/s = 37 Hz
    zeta_bend: float = 0.02
    # Unico termino que acopla los dos modos; != 0 abre la puerta al flameo.
    S_theta: float = 0.0           # [kg*m]

    dt: float = 0.002

    def __post_init__(self):
        if self.CL_delta == 0.0:
            self.CL_delta = cl_alpha_helmbold(aspect_ratio(self.b_fin, self.S_fin))

    @property
    def qbar(self):
        return 0.5 * self.rho * self.V ** 2

    @property
    def v_eff(self):
        """Velocidad efectiva para 1/V: nunca por debajo de V_min."""
        vmin = self.V_min if self.V_min > 1e-9 else 1e-9
        return self.V if self.V > vmin else vmin

    @property
    def k_fin(self):
        """Ganancia de la aleta:  k = qbar * S * CL_delta   [N/rad]."""
        return self.qbar * self.S_fin * self.CL_delta


def params_update_aero(P):
    """Recalcula CL_delta tras tocar b_fin o S_fin (AR es quien fija CL)."""
    P.CL_delta = cl_alpha_helmbold(aspect_ratio(P.b_fin, P.S_fin))
    return P


# ─────────────────────────────────────────────────────────────────────────────
# Estado y comandos
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class State:
    q: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    omega: np.ndarray = field(default_factory=lambda: np.zeros(3))
    delta: np.ndarray = field(default_factory=lambda: np.zeros(4))      # raiz
    delta_cmd: np.ndarray = field(default_factory=lambda: np.zeros(4))  # comandada

    # Estado elastico (§5): delta es lo que impone el servo en la RAIZ, theta
    # lo que la aleta se retuerce por encima, y la suma es lo que ve el aire.
    theta: np.ndarray = field(default_factory=lambda: np.zeros(4))      # [rad]
    theta_dot: np.ndarray = field(default_factory=lambda: np.zeros(4))  # [rad/s]
    w: np.ndarray = field(default_factory=lambda: np.zeros(4))          # [m]
    w_dot: np.ndarray = field(default_factory=lambda: np.zeros(4))      # [m/s]

    def copy(self):
        return State(self.q.copy(), self.omega.copy(),
                     self.delta.copy(), self.delta_cmd.copy(),
                     self.theta.copy(), self.theta_dot.copy(),
                     self.w.copy(), self.w_dot.copy())


@dataclass
class Commands:
    u_pitch: float = 0.0
    u_yaw: float = 0.0
    u_roll: float = 0.0
    fin_trim: np.ndarray = field(default_factory=lambda: np.zeros(4))


def apply_commands(C, s):
    """delta_cmd = mix(u) + fin_trim.  El actuador lleva delta hasta ahi."""
    s.delta_cmd = mix_commands(C.u_pitch, C.u_yaw, C.u_roll) + C.fin_trim


def apply_commands_instant(C, s):
    """Actuador ideal: delta = delta_cmd de golpe."""
    apply_commands(C, s)
    s.delta = s.delta_cmd.copy()


# ─────────────────────────────────────────────────────────────────────────────
# Mixing: cambio de base a las irreps de V4 = Z2 x Z2
#
#   / d1 \   / 1  0   1 \ / u_pitch \      u_pitch = (d1 + d2)/2
#   | d2 | = | 1  0  -1 | | u_yaw   |      u_yaw   = (d3 + d4)/2
#   | d3 |   | 0  1   1 | \ u_roll  /      u_roll  = (d1-d2+d3-d4)/4
#   \ d4 /   \ 0  1  -1 /
# ─────────────────────────────────────────────────────────────────────────────

MIX = np.array([[1.0, 0.0,  1.0],
                [1.0, 0.0, -1.0],
                [0.0, 1.0,  1.0],
                [0.0, 1.0, -1.0]])

DEMIX = np.array([[0.5,   0.5,   0.0,   0.0 ],
                  [0.0,   0.0,   0.5,   0.5 ],
                  [0.25, -0.25,  0.25, -0.25]])


# MIX y DEMIX de arriba son la forma compacta, util para razonar.  Aqui se
# usa la forma desarrollada: el producto matricial acumula los sumandos en
# otro orden y produce ceros con signo distinto, lo justo para que el diff
# con la version C++ deje de ser vacio.
def mix_commands(u_pitch, u_yaw, u_roll):
    return np.array([u_pitch + u_roll,     # d1  derecha
                     u_pitch - u_roll,     # d2  izquierda
                     u_yaw   + u_roll,     # d3  arriba
                     u_yaw   - u_roll])    # d4  abajo


def demix(delta):
    d = np.asarray(delta)
    return (0.5  * (d[0] + d[1]),
            0.5  * (d[2] + d[3]),
            0.25 * (d[0] - d[1] + d[2] - d[3]))


# ─────────────────────────────────────────────────────────────────────────────
# Aerodinamica y dinamica
# ─────────────────────────────────────────────────────────────────────────────

def fin_forces(P, delta):
    """f_i = qbar * S * CL_delta * clamp(delta_i, +-delta_stall).

    Fuera de +-delta_stall la fuerza satura (post-perdida simple).  El clamp
    vive aqui, no en el mixing.
    """
    return P.k_fin * np.clip(np.asarray(delta), -P.delta_stall, +P.delta_stall)


def fin_effective_deflection(P, delta, theta, omega):
    """Incidencia que ve cada aleta de verdad: mando + torsion + rotacion.

    §4 — cuando el cohete gira, v_i = omega x r_i con r1=(-l_t,+d,0)
    r2=(-l_t,-d,0) r3=(-l_t,0,-d) r4=(-l_t,0,+d).  Del par horizontal cuenta
    (v_i)_z = p*r_y - q*r_x; del vertical (v_i)_y = r*r_x - p*r_z.  Si la
    aleta se mueve hacia +z el aire la ataca desde -z y su incidencia efectiva
    BAJA: de ahi los signos.

    §5 — a eso se le suma la torsion elastica theta.  theta puede ser None:
    entonces la aleta se trata como rigida.
    """
    delta = np.asarray(delta, dtype=float)
    flex = P.flex_on and theta is not None
    de = delta + np.asarray(theta, dtype=float) if flex else delta.copy()

    if not P.damping_on:
        return de

    p, q, r = omega
    invV = 1.0 / P.v_eff
    return np.array([
        de[0] - (q * P.l_t + p * P.d) * invV,   # d1 derecha
        de[1] - (q * P.l_t - p * P.d) * invV,   # d2 izquierda
        de[2] + (r * P.l_t - p * P.d) * invV,   # d3 arriba
        de[3] + (r * P.l_t + p * P.d) * invV,   # d4 abajo
    ])


def fin_moments(P, delta):
    """Momentos sobre el CG con omega = 0: el desacoplo por simetria."""
    f = fin_forces(P, delta)
    L =  P.d   * (f[0] - f[1] + f[2] - f[3])
    M =  P.l_t * (f[0] + f[1])
    N = -P.l_t * (f[2] + f[3])
    return np.array([L, M, N])


def fin_moments_rate(P, delta, theta, omega):
    """Momentos con amortiguamiento (§4) y torsion elastica (§5)."""
    return fin_moments(P, fin_effective_deflection(P, delta, theta, omega))


def damping_coeffs(P):
    """Derivadas de estabilidad linealizadas:  (L_p, M_q, N_r), todas <= 0.

    La matriz es DIAGONAL: la simetria V4 que desacopla el mando desacopla
    tambien la respuesta a la rotacion.  Ese es el resultado de §4.
    """
    if not P.damping_on:
        return 0.0, 0.0, 0.0
    invV = 1.0 / P.v_eff
    L_p = -4.0 * P.k_fin * P.d * P.d * invV
    M_q = -2.0 * P.k_fin * P.l_t * P.l_t * invV
    return L_p, M_q, M_q


def control_effectiveness(P):
    """Autoridad de control (L_du, M_du, N_du) [N*m/rad].

    N_du es NEGATIVA: es la convencion de Blakelock (u_yaw > 0 -> N < 0).
    """
    k = P.k_fin
    return 4.0 * k * P.d, 2.0 * k * P.l_t, -2.0 * k * P.l_t


# ─────────────────────────────────────────────────────────────────────────────
# §5 — aeroelasticidad
#
# Cada aleta es un sistema de DOS grados de libertad excitado por su propia
# fuerza aerodinamica:
#
#   [ m_bend   S_theta ] [ w'' ]   [ f - c_w  w'  - K_bend  w  ]
#   [ S_theta  I_theta ] [ th''] = [ H - c_th th' - K_theta th ]
#
#   con  H = -e_ea * f        (momento de la carga sobre el pivote)
#
# La FLEXION w no cambia la incidencia (aleta recta, sin flecha): es el modo
# que se ve, no el que manda.  La TORSION th SI entra en el angulo de ataque,
# y ahi se cierra el lazo:
#
#     f = k (delta + th)      K_theta th = -e_ea f
#  => delta_eff = delta / (1 + lambda),   lambda = qbar S CL_delta e_ea/K_theta
#     eta = 1/(1 + qbar/qbar_car),        qbar_car = K_theta/(S CL_delta |e_ea|)
# ─────────────────────────────────────────────────────────────────────────────

def _aeroelastic_lambda(P):
    """Unico numero adimensional del problema aeroelastico estatico."""
    if not P.flex_on or P.K_theta <= 0.0:
        return 0.0
    return P.k_fin * P.e_ea / P.K_theta


def aeroelastic_efficiency(P):
    """Cuanto del mando llega de verdad al aire.  1 exacto si la aleta es rigida."""
    den = 1.0 + _aeroelastic_lambda(P)
    # den <= 0 es la aleta divergida: el modelo estatico ya no significa nada.
    return 1.0 / den if den > 1e-9 else 0.0


def qbar_caracteristica(P):
    """qbar_car = K_theta/(S CL_delta |e_ea|).

    Con e_ea > 0 es donde eta = 1/2; con e_ea < 0, la de divergencia.
    """
    if not P.flex_on:
        return 0.0
    den = P.S_fin * P.CL_delta * abs(P.e_ea)
    return P.K_theta / den if den > 1e-12 else 0.0


def aeroelastic_divergence(P):
    """Solo la aleta sobrebalanceada (e_ea < 0) diverge, y solo sobre qbar_car."""
    return P.flex_on and P.e_ea < 0.0 and (1.0 + _aeroelastic_lambda(P)) <= 0.0


def structural_freqs(P):
    """Frecuencias naturales en vacio (w_bend, w_tors) [rad/s]."""
    wb = math.sqrt(P.K_bend / P.m_bend) if P.m_bend > 1e-12 else 0.0
    wt = math.sqrt(P.K_theta / P.I_theta) if P.I_theta > 1e-12 else 0.0
    return wb, wt


def structural_cfl(P):
    """max(w)*dt — el numero de estabilidad del integrador.

    Los modos estructurales son cientos de veces mas rapidos que la actitud del
    cohete, y son ELLOS quienes fijan el paso.  RK4 es inestable por encima de
    2.83 y ya es impreciso bastante antes.  Vale 0 con la aleta rigida, que es
    justamente por lo que §1-§4 podian permitirse dt = 10 ms.
    """
    if not P.flex_on:
        return 0.0
    wb, wt = structural_freqs(P)
    return max(wb, wt) * P.dt


def structural_unstable(P):
    """El paso actual no resuelve los modos: esto va a acabar en NaN."""
    # 2.0 y no 2.83: el margen que queda entre ambos ya no es simulacion, es
    # una oscilacion numerica que parece fisica y no lo es.
    return structural_cfl(P) > 2.0


def fin_structural_accel(P, f, theta, theta_dot, w, w_dot):
    """Aceleraciones (theta_dd, w_dd) de UNA aleta sometida a la fuerza f."""
    if not P.flex_on:
        return 0.0, 0.0

    # Amortiguamiento estructural como fraccion del critico.
    c_w = 2.0 * P.zeta_bend * math.sqrt(max(P.K_bend * P.m_bend, 0.0))
    c_th = 2.0 * P.zeta_theta * math.sqrt(max(P.K_theta * P.I_theta, 0.0))

    # Momento de la carga sobre el pivote.  Con e_ea > 0 (centro aerodinamico
    # por detras) tiende a destorcer: se opone a la incidencia que lo creo.
    H = -P.e_ea * f

    bw = f - c_w * w_dot - P.K_bend * w
    bt = H - c_th * theta_dot - P.K_theta * theta

    # Inversion de la matriz de masa 2x2.  S_theta es el unico acoplo.
    det = P.m_bend * P.I_theta - P.S_theta * P.S_theta
    if abs(det) < 1e-15:
        return 0.0, 0.0

    w_dd = (P.I_theta * bw - P.S_theta * bt) / det
    theta_dd = (P.m_bend * bt - P.S_theta * bw) / det
    return theta_dd, w_dd


# ─────────────────────────────────────────────────────────────────────────────
# §6 — actuador
# ─────────────────────────────────────────────────────────────────────────────

def servo_step(P, s, dt):
    """Lleva delta hacia delta_cmd: primero el retardo, luego el rate limit."""
    target = np.array(s.delta_cmd, dtype=float)

    if P.tau_servo > 1e-9:
        a = 1.0 - math.exp(-dt / P.tau_servo)
        target = s.delta + a * (s.delta_cmd - s.delta)

    if P.rate_limit > 1e-9:
        dmax = P.rate_limit * dt
        target = s.delta + np.clip(target - s.delta, -dmax, +dmax)

    s.delta = target
    return s


def rotational_dynamics(P, w, M):
    """omega_dot = I^-1 (M - omega x I omega), con simetria XZ (Ixz)."""
    Iw = np.array([P.Ixx * w[0] - P.Ixz * w[2],
                   P.Iyy * w[1],
                   -P.Ixz * w[0] + P.Izz * w[2]])
    b = M - np.cross(w, Iw)

    # I^-1 en el bloque xz:  1/Gamma * [[Izz, Ixz], [Ixz, Ixx]]
    Gamma = P.Ixx * P.Izz - P.Ixz * P.Ixz
    if abs(Gamma) < 1e-12:
        return np.zeros(3)

    return np.array([(P.Izz * b[0] + P.Ixz * b[2]) / Gamma,
                     b[1] / P.Iyy,
                     (P.Ixz * b[0] + P.Ixx * b[2]) / Gamma])


def quat_kinematics(q, w):
    """e_dot = 1/2 Omega(omega) e."""
    e0, e1, e2, e3 = q
    p, qq, r = w
    return 0.5 * np.array([
        -p * e1 - qq * e2 - r * e3,
         p * e0 + r  * e2 - qq * e3,
        qq * e0 - r  * e1 + p  * e3,
         r * e0 + qq * e1 - p  * e2,
    ])


def quat_normalize(q):
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


# Disposicion del vector de estado del integrador.
#
#   rigido   (N_RIG  =  7): [0:4] cuaternion   [4:7] omega
#   flexible (N_FLEX = 23): + [7:11] theta   [11:15] theta_dot
#                             [15:19] w      [19:23] w_dot
#
# Que la version rigida use solo 7 no es cosmetica: garantiza que con
# flex_on = False el modelo de §1-§4 se reproduce bit a bit.
N_RIG, N_FLEX = 7, 23
IT, ITD, IW, IWD = 7, 11, 15, 19


def _derivative(P, delta, y, flex):
    """La deflexion de RAIZ esta congelada en el paso, pero los momentos se
    reevaluan en cada etapa: dependen de omega (§4) y de la torsion (§5)."""
    theta = y[IT:IT + 4] if flex else None

    de = fin_effective_deflection(P, delta, theta, y[4:7])
    f = fin_forces(P, de)

    M = np.array([P.d * (f[0] - f[1] + f[2] - f[3]),
                  P.l_t * (f[0] + f[1]),
                  -P.l_t * (f[2] + f[3])])

    dy = np.empty(N_FLEX if flex else N_RIG)
    dy[:4] = quat_kinematics(y[:4], y[4:7])
    dy[4:7] = rotational_dynamics(P, y[4:7], M)

    if flex:
        # Cada aleta responde a SU propia fuerza: cuatro sistemas de 2 gdl.
        for i in range(4):
            theta_dd, w_dd = fin_structural_accel(
                P, f[i], y[IT + i], y[ITD + i], y[IW + i], y[IWD + i])
            dy[IT + i] = y[ITD + i]
            dy[ITD + i] = theta_dd
            dy[IW + i] = y[IWD + i]
            dy[IWD + i] = w_dd
    return dy


def step_rk4(P, s, dt):
    """Un paso de RK4; la deflexion de raiz congelada durante el paso."""
    flex = P.flex_on

    if flex:
        y = np.concatenate([s.q, s.omega, s.theta, s.theta_dot, s.w, s.w_dot])
    else:
        y = np.concatenate([s.q, s.omega])

    k1 = _derivative(P, s.delta, y, flex)
    k2 = _derivative(P, s.delta, y + 0.5 * dt * k1, flex)
    k3 = _derivative(P, s.delta, y + 0.5 * dt * k2, flex)
    k4 = _derivative(P, s.delta, y + dt * k3, flex)

    y = y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    s.q = quat_normalize(y[:4])
    s.omega = y[4:7]
    if flex:
        s.theta = y[IT:IT + 4]
        s.theta_dot = y[ITD:ITD + 4]
        s.w = y[IW:IW + 4]
        s.w_dot = y[IWD:IWD + 4]
    return s


def euler_from_quat(q):
    """Angulos 3-2-1 (yaw-pitch-roll)."""
    e0, e1, e2, e3 = q
    phi = math.atan2(2.0 * (e0 * e1 + e2 * e3), 1.0 - 2.0 * (e1 * e1 + e2 * e2))
    sth = np.clip(2.0 * (e0 * e2 - e3 * e1), -1.0, 1.0)
    th = math.asin(sth)
    psi = math.atan2(2.0 * (e0 * e3 + e1 * e2), 1.0 - 2.0 * (e2 * e2 + e3 * e3))
    return phi, th, psi


def rotmat_from_quat(q):
    """R: cuerpo -> inercial."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def wrap_pi(a):
    """Envuelve un angulo a (-pi, pi].  Necesario en el error del autopiloto."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


# ─────────────────────────────────────────────────────────────────────────────
# §6 — autopiloto: tres lazos SISO, uno por irrep
#
# El desacoplo de §2 y la diagonalidad del amortiguamiento de §4 son lo que
# permite sintonizar tres PID independientes en vez de un controlador MIMO.
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class PID:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0        # las ganancias pueden ser negativas (ver N_du)
    i_state: float = 0.0   # integral del error [rad*s]
    i_max: float = 0.0     # anti-windup
    u_max: float = 0.0     # saturacion de la salida [rad]


def pid_update(c, err, rate, dt):
    """u = kp*err + ki*I(err) - kd*rate.

    La derivada se toma de la VELOCIDAD medida, no de d(err)/dt: asi un salto
    en la referencia no produce patada derivativa.
    """
    c.i_state += err * dt
    if c.i_max > 0.0:
        c.i_state = float(np.clip(c.i_state, -c.i_max, +c.i_max))

    u = c.kp * err + c.ki * c.i_state - c.kd * rate

    if c.u_max > 0.0:
        u_sat = float(np.clip(u, -c.u_max, +c.u_max))
        if u_sat != u and abs(c.ki) > 1e-12:
            c.i_state += (u_sat - u) / c.ki      # anti-windup
        u = u_sat
    return u


@dataclass
class Autopilot:
    enabled: bool = False
    phi_ref: float = 0.0
    theta_ref: float = 0.0
    psi_ref: float = 0.0
    wn_roll: float = 3.0       # el alabeo tiene mucha menos inercia
    wn_pitch: float = 1.5
    wn_yaw: float = 1.5
    zeta: float = 0.8
    roll: PID = field(default_factory=PID)
    pitch: PID = field(default_factory=PID)
    yaw: PID = field(default_factory=PID)


def _tune_channel(c, I, C_du, C_rate, wn, zeta, u_max):
    """Asignacion de polos sobre  I*x_dd = C_du*u + C_rate*x_d,
    con  u = kp*err - kd*x_d:

        kp = wn^2 * I / C_du        kd = (2*zeta*wn*I + C_rate) / C_du

    C_rate ya es negativo, de ahi el '+': el controlador solo aporta el
    amortiguamiento que la aerodinamica no da.
    """
    c.kp = c.ki = c.kd = 0.0
    c.i_state = 0.0
    c.i_max = c.u_max = 0.0
    if abs(C_du) < 1e-12:
        return c

    c.kp = (wn * wn * I) / C_du
    c.kd = (2.0 * zeta * wn * I + C_rate) / C_du
    c.ki = c.kp * wn / 5.0          # trim lento, sin dominar el lazo

    c.u_max = u_max
    c.i_max = u_max / abs(c.ki) if abs(c.ki) > 1e-12 else 0.0
    return c


def autopilot_tune(P, A):
    L_p, M_q, N_r = damping_coeffs(P)
    L_du, M_du, N_du = control_effectiveness(P)

    # La aleta flexible entrega solo una fraccion eta del mando (§5), y por la
    # misma razon amortigua solo eta.  Los modos estructurales (>30 Hz) son mil
    # veces mas rapidos que el lazo de actitud (~1 Hz), asi que para sintonizar
    # basta su valor cuasi-estatico.  Sin esta correccion el autopiloto queda
    # sistematicamente lento al subir la velocidad.
    eta = aeroelastic_efficiency(P)
    L_du *= eta
    M_du *= eta
    N_du *= eta
    L_p *= eta
    M_q *= eta
    N_r *= eta

    # Margen: el mixing suma pitch/yaw y roll sobre la misma aleta.
    u_max = 0.7 * P.delta_stall

    _tune_channel(A.roll,  P.Ixx, L_du, L_p, A.wn_roll,  A.zeta, u_max)
    _tune_channel(A.pitch, P.Iyy, M_du, M_q, A.wn_pitch, A.zeta, u_max)
    _tune_channel(A.yaw,   P.Izz, N_du, N_r, A.wn_yaw,   A.zeta, u_max)
    return A


def autopilot_default(P):
    return autopilot_tune(P, Autopilot())


def autopilot_update(P, A, s, C, dt):
    """Lee la actitud de s y escribe u_pitch/u_yaw/u_roll en C."""
    if not A.enabled:
        return C

    phi, th, psi = euler_from_quat(s.q)

    # Tres lazos SISO independientes: eso es lo que compra la simetria V4.
    C.u_roll  = pid_update(A.roll,  wrap_pi(A.phi_ref   - phi), s.omega[0], dt)
    C.u_pitch = pid_update(A.pitch, wrap_pi(A.theta_ref - th),  s.omega[1], dt)
    C.u_yaw   = pid_update(A.yaw,   wrap_pi(A.psi_ref   - psi), s.omega[2], dt)
    return C


# ─────────────────────────────────────────────────────────────────────────────
# §7 — casos de validacion
# ─────────────────────────────────────────────────────────────────────────────

class _Check:
    def __init__(self):
        self.total = 0
        self.fail = 0

    def __call__(self, ok, nombre, detalle=""):
        self.total += 1
        if ok:
            print(f"  [ ok ] {nombre}")
        else:
            self.fail += 1
            print(f"  [FAIL] {nombre}  {detalle}")


def run_tests():
    P = Params()
    check = _Check()

    print(f"\nParametros:  qbar = {P.qbar:.1f} Pa | CL_delta = {P.CL_delta:.4f} 1/rad "
          f"(AR = {aspect_ratio(P.b_fin, P.S_fin):.3f})")
    print(f"             l_t = {P.l_t:.3f} m | d = {P.d:.3f} m | "
          f"delta_stall = {P.delta_stall*DEG:.1f} deg")
    print(f"             Ixx = {P.Ixx:.4f}  Iyy = {P.Iyy:.4f}  Izz = {P.Izz:.4f} kg m^2\n")

    # 1. Pitch puro
    print("1. Pitch puro (u_pitch > 0)")
    d = mix_commands(5 * RAD, 0, 0)
    M = fin_moments(P, d)
    check(M[1] > 0, "M > 0 (momento de cabeceo)")
    check(abs(M[0]) < 1e-12, "L == 0 (sin roll)")
    check(abs(M[2]) < 1e-12, "N == 0 (sin yaw)")
    s = State(); s.delta = d
    for _ in range(500):
        step_rk4(P, s, P.dt)
    check(s.omega[1] > 0, "q crece (pitch rate)")
    check(abs(s.omega[0]) < 1e-10 and abs(s.omega[2]) < 1e-10,
          "p == r == 0 (canales desacoplados)")
    print(f"         -> tras 1 s:  p={s.omega[0]:.3e}  q={s.omega[1]:+.5f}  "
          f"r={s.omega[2]:.3e} rad/s")

    # 2. Yaw puro
    print("\n2. Yaw puro (u_yaw > 0)")
    d = mix_commands(0, 5 * RAD, 0)
    M = fin_moments(P, d)
    check(abs(M[2]) > 1e-6, "N != 0 (momento de guinada)")
    check(abs(M[0]) < 1e-12, "L == 0")
    check(abs(M[1]) < 1e-12, "M == 0")
    check(M[2] < 0, "N < 0 (convencion Blakelock: cola a +y, nariz a -y)")
    s = State(); s.delta = d
    for _ in range(500):
        step_rk4(P, s, P.dt)
    check(abs(s.omega[0]) < 1e-10 and abs(s.omega[1]) < 1e-10, "p == q == 0")
    print(f"         -> tras 1 s:  p={s.omega[0]:.3e}  q={s.omega[1]:.3e}  "
          f"r={s.omega[2]:+.5f} rad/s")

    # 3. Roll puro
    print("\n3. Roll puro (u_roll > 0)")
    d = mix_commands(0, 0, 5 * RAD)
    M = fin_moments(P, d)
    check(M[0] > 0, "L > 0 (momento de alabeo)")
    check(abs(M[1]) < 1e-12, "M == 0")
    check(abs(M[2]) < 1e-12, "N == 0")
    s = State(); s.delta = d
    for _ in range(500):
        step_rk4(P, s, P.dt)
    check(s.omega[0] > 0, "p crece (roll rate)")
    check(abs(s.omega[1]) < 1e-10 and abs(s.omega[2]) < 1e-10, "q == r == 0")
    print(f"         -> tras 1 s:  p={s.omega[0]:+.5f}  q={s.omega[1]:.3e}  "
          f"r={s.omega[2]:.3e} rad/s")

    # 4. Round-trip del mixing
    print("\n4. Round-trip del mixing:  demix(mix(u)) == u")
    casos = [(3*RAD, -2*RAD, 1.5*RAD), (-7*RAD, 4*RAD, -6*RAD),
             (0.0, 0.0, 0.0), (12*RAD, 12*RAD, 12*RAD)]
    maxerr = 0.0
    for c in casos:
        got = np.array(demix(mix_commands(*c)))
        maxerr = max(maxerr, float(np.max(np.abs(got - np.array(c)))))
    check(maxerr < 1e-15, "identidad en los 4 casos")
    print(f"         -> error maximo: {maxerr:.3e} rad")

    # 5. Saturacion
    print("\n5. Saturacion (delta > delta_stall)")
    f_stall = fin_forces(P, np.full(4, P.delta_stall))
    f_big = fin_forces(P, np.full(4, 3.0 * P.delta_stall))
    f_neg = fin_forces(P, np.array([-3.0 * P.delta_stall, 0, 0, 0]))
    f_half = fin_forces(P, np.array([0.5 * P.delta_stall, 0, 0, 0]))
    check(abs(f_big[0] - f_stall[0]) < 1e-12, "f satura por arriba (no crece)")
    check(abs(f_neg[0] + f_stall[0]) < 1e-12, "f satura por abajo (simetrico)")
    check(abs(f_half[0] - 0.5 * f_stall[0]) < 1e-12, "lineal por debajo del stall")
    print(f"         -> f(stall) = {f_stall[0]:.3f} N   f(3x stall) = {f_big[0]:.3f} N")

    # 6. Norma del cuaternion
    print("\n6. Norma del cuaternion tras 1e4 pasos")
    s = State()
    s.omega = np.array([0.8, -0.4, 0.3])
    s.delta = mix_commands(4 * RAD, -3 * RAD, 2 * RAD)
    for _ in range(10000):
        step_rk4(P, s, P.dt)
    n = float(np.linalg.norm(s.q))
    check(abs(n - 1.0) < 1e-9, "|e| - 1 < 1e-9")
    phi, th, psi = euler_from_quat(s.q)
    print(f"         -> |e| - 1 = {n-1.0:.3e}   |   phi={phi*DEG:+.2f}  "
          f"theta={th*DEG:+.2f}  psi={psi*DEG:+.2f} deg")

    # 7. Extra: comando combinado
    print("\n7. Extra: comando combinado -> momentos coherentes con las irreps")
    up, uy, ur = 4 * RAD, -3 * RAD, 2 * RAD
    M = fin_moments(P, mix_commands(up, uy, ur))
    k = P.qbar * P.S_fin * P.CL_delta
    check(abs(M[0] - 4.0 * P.d * k * ur) < 1e-9, "L = 4 d k u_roll")
    check(abs(M[1] - 2.0 * P.l_t * k * up) < 1e-9, "M = 2 l_t k u_pitch")
    check(abs(M[2] + 2.0 * P.l_t * k * uy) < 1e-9, "N = -2 l_t k u_yaw")
    print(f"         -> L={M[0]:+.3f}  M={M[1]:+.3f}  N={M[2]:+.3f}  N*m")

    # 8. Amortiguamiento: la matriz de derivadas es DIAGONAL (§4)
    print("\n8. Amortiguamiento aerodinamico: matriz diagonal en la base de irreps")
    d0 = np.zeros(4)
    w = 0.05                                   # rad/s, bien dentro del lineal
    L_p, M_q, N_r = damping_coeffs(P)

    Mp = fin_moments_rate(P, d0, None, np.array([w, 0.0, 0.0]))
    Mq = fin_moments_rate(P, d0, None, np.array([0.0, w, 0.0]))
    Mr = fin_moments_rate(P, d0, None, np.array([0.0, 0.0, w]))

    check(abs(Mp[1]) < 1e-12 and abs(Mp[2]) < 1e-12, "p pura -> solo L (sin M ni N)")
    check(abs(Mq[0]) < 1e-12 and abs(Mq[2]) < 1e-12, "q pura -> solo M (sin L ni N)")
    check(abs(Mr[0]) < 1e-12 and abs(Mr[1]) < 1e-12, "r pura -> solo N (sin L ni M)")
    check(Mp[0] < 0 and Mq[1] < 0 and Mr[2] < 0, "los tres momentos se oponen al giro")
    check(abs(Mp[0] - L_p * w) < 1e-9, "L = L_p * p   (coincide con la linealizacion)")
    check(abs(Mq[1] - M_q * w) < 1e-9, "M = M_q * q")
    check(abs(Mr[2] - N_r * w) < 1e-9, "N = N_r * r")
    check(abs(M_q - N_r) < 1e-12, "M_q == N_r (los dos pares tienen el mismo brazo)")
    print(f"         -> L_p={L_p:+.4f}  M_q={M_q:+.4f}  N_r={N_r:+.4f}  N*m/(rad/s)")

    # 9. Regimen permanente: la aleta acaba alineada con el flujo local
    print("\n9. Regimen permanente con mando constante:  omega_ss = V*u/brazo")
    # dt = 10 ms basta para tau ~ 20-30 s, pero NO resuelve los modos
    # estructurales de §5 (65 Hz): con la aleta flexible RK4 diverge a NaN.
    Q = replace(P, dt=0.01, flex_on=False)
    N_STEP = 20000                             # 200 s  (6-10 constantes de tiempo)
    u = 3.0 * RAD

    sp = State(); sp.delta = mix_commands(u, 0.0, 0.0)
    for _ in range(N_STEP):
        step_rk4(Q, sp, Q.dt)
    q_teo = Q.v_eff * u / Q.l_t
    check(abs(sp.omega[1] - q_teo) < 0.01 * q_teo, "q -> V*u_pitch/l_t")

    sr = State(); sr.delta = mix_commands(0.0, 0.0, u)
    for _ in range(N_STEP):
        step_rk4(Q, sr, Q.dt)
    p_teo = Q.v_eff * u / Q.d
    check(abs(sr.omega[0] - p_teo) < 0.01 * p_teo, "p -> V*u_roll/d")

    sy = State(); sy.delta = mix_commands(0.0, u, 0.0)
    for _ in range(N_STEP):
        step_rk4(Q, sy, Q.dt)
    r_teo = -Q.v_eff * u / Q.l_t
    check(abs(sy.omega[2] - r_teo) < 0.01 * abs(r_teo), "r -> -V*u_yaw/l_t")

    de = fin_effective_deflection(Q, sp.delta, sp.theta, sp.omega)
    check(abs(de[0]) < 1e-3 and abs(de[1]) < 1e-3, "delta_eff -> 0 en equilibrio")
    print(f"         -> q={sp.omega[1]:+.5f} (teo {q_teo:+.5f})   "
          f"p={sr.omega[0]:+.5f} (teo {p_teo:+.5f})   "
          f"r={sy.omega[2]:+.5f} (teo {r_teo:+.5f})")

    # 10. Sin amortiguamiento la velocidad angular no converge
    print("\n10. Contraste: damping_on = false -> rampa sin limite")
    Q = replace(P, dt=0.01, flex_on=False, damping_on=False)  # igual que el caso 9
    s = State(); s.delta = mix_commands(3.0 * RAD, 0.0, 0.0)
    for _ in range(1000):
        step_rk4(Q, s, Q.dt)
    q10 = s.omega[1]
    for _ in range(1000):
        step_rk4(Q, s, Q.dt)
    q20 = s.omega[1]
    check(abs(q20 - 2.0 * q10) < 1e-6, "q crece lineal (q(20s) = 2*q(10s))")
    L_p0, M_q0, N_r0 = damping_coeffs(Q)
    check(L_p0 == 0.0 and M_q0 == 0.0 and N_r0 == 0.0, "las derivadas se anulan")
    print(f"         -> q(10s)={q10:+.5f}   q(20s)={q20:+.5f} rad/s  (sin equilibrio)")

    # 11. Actuador: limite de velocidad y retardo de primer orden (§6)
    print("\n11. Dinamica del actuador")
    Q = replace(P, tau_servo=0.0, rate_limit=100.0 * RAD)
    s = State(); s.delta_cmd = np.full(4, 10.0 * RAD)
    servo_step(Q, s, Q.dt)
    check(abs(s.delta[0] - Q.rate_limit * Q.dt) < 1e-15, "un paso avanza rate_limit*dt")
    for _ in range(5000):
        servo_step(Q, s, Q.dt)
    check(abs(s.delta[0] - 10.0 * RAD) < 1e-12, "converge al comando sin sobrepasarlo")

    R = replace(P, tau_servo=0.05, rate_limit=0.0)
    s2 = State(); s2.delta_cmd = np.full(4, 10.0 * RAD)
    for _ in range(round(R.tau_servo / R.dt)):
        servo_step(R, s2, R.dt)
    check(abs(s2.delta[0] - 10.0 * RAD * (1.0 - math.exp(-1.0))) < 1e-9,
          "en t = tau lleva 1 - 1/e del recorrido")

    I = replace(P, tau_servo=0.0, rate_limit=0.0)
    s3 = State(); s3.delta_cmd = np.full(4, 10.0 * RAD)
    servo_step(I, s3, I.dt)
    check(abs(s3.delta[0] - 10.0 * RAD) < 1e-15, "tau = rate = 0 -> salto instantaneo")
    print(f"         -> rate: {Q.rate_limit*Q.dt*DEG:.6f} deg tras 1 paso   |   "
          f"lag: {s2.delta[0]*DEG:.4f} de 10.0 deg en t=tau")

    # 12. Autopiloto: tres lazos SISO, error estacionario nulo (§6)
    print("\n12. Autopiloto de actitud")
    Q = replace(P, dt=0.005)
    A = autopilot_default(Q)
    A.enabled = True
    A.phi_ref, A.theta_ref, A.psi_ref = 10.0 * RAD, 5.0 * RAD, -8.0 * RAD

    check(A.pitch.kp > 0 and A.roll.kp > 0, "kp de alabeo y cabeceo positivas")
    check(A.yaw.kp < 0, "kp de guinada NEGATIVA (N_du < 0)")
    check(A.pitch.kd > 0, "kd de cabeceo positiva (anade amortiguamiento)")

    s, C = State(), Commands()
    for _ in range(12000):                     # 60 s
        autopilot_update(Q, A, s, C, Q.dt)
        apply_commands(C, s)
        servo_step(Q, s, Q.dt)
        step_rk4(Q, s, Q.dt)

    phi, th, psi = euler_from_quat(s.q)
    check(abs(phi * DEG - 10.0) < 0.2, "phi -> phi_ref  (< 0.2 deg)")
    check(abs(th * DEG - 5.0) < 0.2, "theta -> theta_ref")
    check(abs(psi * DEG + 8.0) < 0.2, "psi -> psi_ref")
    check(all(abs(x) < 1e-3 for x in s.omega), "velocidades angulares residuales ~ 0")
    print(f"         -> phi={phi*DEG:+.4f} (ref +10)   theta={th*DEG:+.4f} (ref +5)   "
          f"psi={psi*DEG:+.4f} (ref -8) deg")

    # 13. Ixz: el producto de inercia acopla alabeo y guinada
    print("\n13. Producto de inercia Ixz: acoplo roll-yaw")
    Q = replace(P, dt=0.01, flex_on=False)  # aisla el acoplo inercial del elastico
    s0 = State(); s0.delta = mix_commands(0.0, 0.0, 3.0 * RAD)
    for _ in range(200):
        step_rk4(Q, s0, Q.dt)
    check(abs(s0.omega[2]) < 1e-12, "Ixz = 0 -> r == 0 con mando de alabeo")

    Q = replace(P, dt=0.01, Ixz=2.0, flex_on=False)
    s1 = State(); s1.delta = mix_commands(0.0, 0.0, 3.0 * RAD)
    for _ in range(200):
        step_rk4(Q, s1, Q.dt)
    check(abs(s1.omega[2]) > 1e-6, "Ixz != 0 -> aparece r (acoplo inercial)")

    # Las aletas no mandan NADA de guinada: la r es puramente inercial (el
    # termino omega x I*omega) y solo despues la aerodinamica la amortigua.
    Mc = fin_moments(Q, s1.delta)
    Mt = fin_moments_rate(Q, s1.delta, s1.theta, s1.omega)
    check(abs(Mc[2]) < 1e-12, "el mando de guinada sigue siendo N = 0")
    check(Mt[2] * s1.omega[2] < 0.0, "el N que aparece es amortiguamiento, se opone a r")
    print(f"         -> Ixz=0: r={s0.omega[2]:.3e}    Ixz=2: r={s1.omega[2]:+.6f} rad/s  "
          f"(p={s1.omega[0]:+.5f})")
    print(f"            N de mando = {Mc[2]:.3e} N*m   |   N total = {Mt[2]:+.6f} N*m")

    # 14. Coherencia de la geometria: AR fija CL_delta
    print("\n14. params_update_aero: cambiar la aleta recalcula CL_delta")
    cl0 = P.CL_delta
    Q = params_update_aero(replace(P, S_fin=2.0 * P.S_fin))
    check(abs(Q.CL_delta - cl0) > 1e-9, "CL_delta cambia al cambiar S_fin")
    check(abs(Q.CL_delta - cl_alpha_helmbold(aspect_ratio(Q.b_fin, Q.S_fin))) < 1e-15,
          "CL_delta == CL_alpha(AR(b_fin, S_fin))")
    check(Q.CL_delta < cl0, "mas superficie con la misma envergadura -> AR menor -> CL menor")
    print(f"         -> AR {aspect_ratio(P.b_fin, P.S_fin):.4f} -> "
          f"{aspect_ratio(Q.b_fin, Q.S_fin):.4f}   |   "
          f"CL_delta {cl0:.4f} -> {Q.CL_delta:.4f} 1/rad")

    # 15. Aeroelasticidad estatica: eficiencia eta = 1/(1+lambda) (§5)
    print("\n15. Aeroelasticidad: la aleta flexible entrega solo eta del mando")
    Q = replace(P, damping_on=False)           # aisla el lazo elastico del de §4
    u = 5.0 * RAD
    eta = aeroelastic_efficiency(Q)

    s = State(); s.delta = mix_commands(u, 0.0, 0.0)
    for _ in range(500):                       # 1 s: la torsion ya esta asentada
        step_rk4(Q, s, Q.dt)
    de = fin_effective_deflection(Q, s.delta, s.theta, s.omega)

    check(eta < 1.0, "eta < 1 con e_ea > 0 (el centro aerodinamico va detras)")
    check(s.theta[0] < 0.0, "la aleta se DESTUERCE: theta se opone al mando")
    check(abs(de[0] / u - eta) < 1e-6, "delta_eff/delta == eta")

    f = fin_forces(Q, de)
    check(abs(s.w[0] - f[0] / Q.K_bend) < 1e-6, "flecha en punta == f/K_bend")

    # Las dos aletas del par horizontal se deforman IGUAL: la respuesta
    # elastica es ella misma simetrica, asi que el desacoplo de §2 aguanta.
    M = fin_moments(Q, de)
    check(abs(s.theta[0] - s.theta[1]) < 1e-15, "theta1 == theta2 (simetria V4)")
    check(abs(M[0]) < 1e-12 and abs(M[2]) < 1e-12, "sigue sin haber L ni N")
    print(f"         -> eta={eta:.6f}   theta={s.theta[0]*DEG:+.5f} deg   "
          f"flecha={1000.0*s.w[0]:.4f} mm   delta_eff={de[0]*DEG:.5f} de 5.0 deg")

    # 16. La eficiencia cae con la presion dinamica
    print("\n16. eta cae como 1/qbar: la aeroelasticidad es cosa de ir rapido")
    q_car = qbar_caracteristica(P)
    R = replace(P, V=math.sqrt(2.0 * q_car / P.rho))
    check(abs(aeroelastic_efficiency(R) - 0.5) < 1e-9, "eta = 1/2 en qbar_car")
    check(aeroelastic_efficiency(P) > 0.99, "eta > 0.99 a 30 m/s")

    S1, S2 = replace(P, V=100.0), replace(P, V=200.0)
    check(aeroelastic_efficiency(S2) < aeroelastic_efficiency(S1) <
          aeroelastic_efficiency(P), "eta decrece monotona con V")

    RG = replace(P, flex_on=False)
    check(aeroelastic_efficiency(RG) == 1.0, "flex_on = false -> eta == 1 exacto")
    check(qbar_caracteristica(RG) == 0.0, "...y no hay qbar caracteristica")
    print(f"         -> qbar_car = {q_car:.0f} Pa (V = {R.V:.0f} m/s)   |   "
          f"eta: 30 m/s {aeroelastic_efficiency(P):.4f} | "
          f"100 {aeroelastic_efficiency(S1):.4f} | 200 {aeroelastic_efficiency(S2):.4f}")

    # 17. Divergencia: aleta sobrebalanceada (e_ea < 0)
    print("\n17. Divergencia aeroelastica con el pivote por detras del c.a.")
    Q = replace(P, e_ea=-0.02)                 # pivote DETRAS del centro aerodinamico
    check(aeroelastic_efficiency(Q) > 1.0, "eta > 1: ahora el lazo AMPLIFICA")
    check(not aeroelastic_divergence(Q), "a 30 m/s todavia no diverge")

    q_div = qbar_caracteristica(Q)
    R = replace(Q, V=math.sqrt(2.0 * 0.9 * q_div / Q.rho))
    check(not aeroelastic_divergence(R), "al 90% de qbar_div sigue acotada")
    check(abs(aeroelastic_efficiency(R) - 10.0) < 1e-6, "eta = 1/(1-0.9) = 10")

    S1 = replace(Q, V=math.sqrt(2.0 * 1.1 * q_div / Q.rho))
    check(aeroelastic_divergence(S1), "por encima de qbar_div: divergida")

    T = replace(Q, damping_on=False)
    s = State(); s.delta = mix_commands(5.0 * RAD, 0.0, 0.0)
    for _ in range(500):
        step_rk4(T, s, T.dt)
    check(s.theta[0] > 0.0, "la aleta se retuerce A FAVOR del mando")
    print(f"         -> qbar_div = {q_div:.0f} Pa (V = {math.sqrt(2.0*q_div/Q.rho):.0f} m/s)   "
          f"eta(30 m/s) = {aeroelastic_efficiency(Q):.4f}   theta = {s.theta[0]*DEG:+.5f} deg")

    # 18. Modos estructurales y estabilidad del integrador
    print("\n18. Modos estructurales: son ELLOS quienes fijan el paso")
    wb, wt = structural_freqs(P)
    check(abs(wb - math.sqrt(P.K_bend / P.m_bend)) < 1e-12, "w_bend = sqrt(K/m)")
    check(abs(wt - math.sqrt(P.K_theta / P.I_theta)) < 1e-12, "w_tors = sqrt(K/I)")

    L_p, M_q, N_r = damping_coeffs(P)
    check(wt > 100.0 * (abs(M_q) / P.Iyy), "la torsion es >100x el modo de cabeceo")
    check(abs(structural_cfl(P) - max(wb, wt) * P.dt) < 1e-12, "CFL = w_max*dt")
    check(not structural_unstable(P), "dt = 2 ms resuelve los modos")
    check(structural_unstable(replace(P, dt=0.01)),
          "dt = 10 ms NO los resuelve (avisa antes de NaN)")
    check(structural_cfl(replace(P, flex_on=False)) == 0.0,
          "aleta rigida: sin restriccion de paso")
    print(f"         -> flexion {wb:.1f} rad/s ({wb/(2*math.pi):.1f} Hz)   "
          f"torsion {wt:.1f} rad/s ({wt/(2*math.pi):.1f} Hz)")
    print(f"            CFL: dt=2ms -> {structural_cfl(P):.3f}    "
          f"dt=10ms -> {structural_cfl(replace(P, dt=0.01)):.3f}    "
          f"(RK4 revienta en 2.83)")

    print("\n" + "─" * 48)
    if check.fail == 0:
        print(f"TODOS LOS TESTS PASARON  ({check.total}/{check.total})\n")
    else:
        print(f"FALLARON {check.fail} de {check.total}\n")
    return 0 if check.fail == 0 else 1


# ─────────────────────────────────────────────────────────────────────────────
# REPL (equivalente al modo --headless de C++)
# ─────────────────────────────────────────────────────────────────────────────

def _z(v):
    """Solo para imprimir.  Las combinaciones antisimetricas (u_roll, L) son la
    cancelacion total de cuatro numeros casi iguales: al converger el lazo
    quedan en ~1e-31, veinte y pico ordenes por debajo de cualquier escala
    fisica del problema, y su SIGNO es ruido de redondeo.  Sin esto, la traza
    de Python y la de C++ difieren en un "-0.00000" que no significa nada."""
    return 0.0 if abs(v) < 1e-9 else v


def print_state(P, s, t, A=None):
    phi, th, psi = euler_from_quat(s.q)
    up, uy, ur = demix(s.delta)
    f = fin_forces(P, s.delta)
    M = fin_moments_rate(P, s.delta, s.theta, s.omega)
    de = fin_effective_deflection(P, s.delta, s.theta, s.omega)
    print(f"t      {t:.4f} s")
    print(f"euler  phi {phi*DEG:+10.5f}  theta {th*DEG:+10.5f}  psi {psi*DEG:+10.5f}  deg")
    print(f"rates  p   {_z(s.omega[0]):+10.6f}  q     {_z(s.omega[1]):+10.6f}  "
          f"r   {_z(s.omega[2]):+10.6f}  rad/s")
    print(f"quat   w {s.q[0]:+.9f}  x {s.q[1]:+.9f}  y {s.q[2]:+.9f}  z {s.q[3]:+.9f}  "
          f"|e| {np.linalg.norm(s.q):.12f}")
    print(f"cmd    d1 {_z(s.delta_cmd[0]*DEG):+10.5f}  d2 {_z(s.delta_cmd[1]*DEG):+10.5f}  "
          f"d3 {_z(s.delta_cmd[2]*DEG):+10.5f}  d4 {_z(s.delta_cmd[3]*DEG):+10.5f}  deg")
    print(f"delta  d1 {_z(s.delta[0]*DEG):+10.5f}  d2 {_z(s.delta[1]*DEG):+10.5f}  "
          f"d3 {_z(s.delta[2]*DEG):+10.5f}  d4 {_z(s.delta[3]*DEG):+10.5f}  deg")
    print(f"eff    d1 {_z(de[0]*DEG):+10.5f}  d2 {_z(de[1]*DEG):+10.5f}  "
          f"d3 {_z(de[2]*DEG):+10.5f}  d4 {_z(de[3]*DEG):+10.5f}  deg")
    print(f"virt   up {_z(up*DEG):+10.5f}  uy {_z(uy*DEG):+10.5f}  ur {_z(ur*DEG):+10.5f}  deg")
    print(f"force  f1 {_z(f[0]):+10.5f}  f2 {_z(f[1]):+10.5f}  "
          f"f3 {_z(f[2]):+10.5f}  f4 {_z(f[3]):+10.5f}  N")
    print(f"moment L  {_z(M[0]):+10.5f}  M  {_z(M[1]):+10.5f}  N  {_z(M[2]):+10.5f}  N*m")
    print(f"air    V {P.V:.3f} m/s   qbar {P.qbar:.4f} Pa   CL_delta {P.CL_delta:.6f} 1/rad")
    if P.flex_on:
        print(f"torsion t1 {_z(s.theta[0]*DEG):+10.5f}  t2 {_z(s.theta[1]*DEG):+10.5f}  "
              f"t3 {_z(s.theta[2]*DEG):+10.5f}  t4 {_z(s.theta[3]*DEG):+10.5f}  deg")
        print(f"flecha w1 {_z(1000.0*s.w[0]):+10.5f}  w2 {_z(1000.0*s.w[1]):+10.5f}  "
              f"w3 {_z(1000.0*s.w[2]):+10.5f}  w4 {_z(1000.0*s.w[3]):+10.5f}  mm")
        q_car = qbar_caracteristica(P)
        print(f"flex   eta {aeroelastic_efficiency(P):.6f}   "
              f"qbar/qbar_car {(P.qbar/q_car if q_car > 0 else 0.0):.5f}"
              f"{'   DIVERGIDA' if aeroelastic_divergence(P) else ''}")
    print(f"modo   amortiguamiento {'ON' if P.damping_on else 'OFF'}   "
          f"servo tau {P.tau_servo:.4f} s / {P.rate_limit*DEG:.1f} deg/s   "
          f"AP {'ON' if (A is not None and A.enabled) else 'OFF'}   "
          f"aleta {'FLEXIBLE' if P.flex_on else 'RIGIDA'}")


def print_gains(P, A):
    """Derivadas de estabilidad y ganancias: la tabla que acompana a §4 y §6."""
    L_p, M_q, N_r = damping_coeffs(P)
    L_du, M_du, N_du = control_effectiveness(P)

    print(f"k = qbar*S*CL_delta = {P.k_fin:.4f} N/rad")
    print(f"autoridad   L_du {L_du:+10.4f}   M_du {M_du:+10.4f}   N_du {N_du:+10.4f}  N*m/rad")
    print(f"amortig.    L_p  {L_p:+10.4f}   M_q  {M_q:+10.4f}   N_r  {N_r:+10.4f}  N*m/(rad/s)")
    # En equilibrio la aleta se alinea con el flujo local: omega_ss = V*u/brazo.
    print(f"omega_ss    p = {P.v_eff/P.d:+.4f}*u_roll   q = {P.v_eff/P.l_t:+.4f}*u_pitch   "
          f"r = {-P.v_eff/P.l_t:+.4f}*u_yaw")
    for nombre, c, wn in (("roll ", A.roll, A.wn_roll),
                          ("pitch", A.pitch, A.wn_pitch),
                          ("yaw  ", A.yaw, A.wn_yaw)):
        print(f"PID {nombre}   kp {c.kp:+9.4f}  ki {c.ki:+9.4f}  kd {c.kd:+9.4f}  (wn {wn:.2f})")
    print("            (las ganancias de guinada son negativas: N_du < 0)")

    if P.flex_on:
        wb, wt = structural_freqs(P)
        q_car = qbar_caracteristica(P)
        print(f"flex        eta {aeroelastic_efficiency(P):.6f}   "
              f"qbar_car {q_car:.1f} Pa (V = {math.sqrt(2.0*q_car/P.rho):.1f} m/s)"
              f"{'   DIVERGIDA' if aeroelastic_divergence(P) else ''}")
        print(f"modos       flexion {wb:.1f} rad/s ({wb/(2*math.pi):.1f} Hz)   "
              f"torsion {wt:.1f} rad/s ({wt/(2*math.pi):.1f} Hz)")
        aviso = ("   *** > 2: baja dt o pon 'flex off', o esto acaba en NaN ***"
                 if structural_unstable(P) else "   (RK4 revienta en 2.83)")
        print(f"            CFL = w_max*dt = {structural_cfl(P):.3f}{aviso}")


def dump_csv(P, A, s, t, T, cada, archivo):
    """Volcado CSV para las figuras del libro.  Devuelve (s, t) avanzados."""
    C = Commands()
    demixed = demix(s.delta_cmd)
    C.u_pitch, C.u_yaw, C.u_roll = demixed

    out = open(archivo, "w") if archivo else sys.stdout
    try:
        out.write("t,phi,theta,psi,p,q,r,d1,d2,d3,d4,dc1,dc2,dc3,dc4,L,M,N\n")
        n = round(T / P.dt)
        cada_n = max(1, round(cada / P.dt)) if cada > 0 else 1
        for i in range(n + 1):
            if i % cada_n == 0:
                phi, th, psi = euler_from_quat(s.q)
                M = fin_moments_rate(P, s.delta, s.theta, s.omega)
                out.write(
                    f"{t:.5f},{phi*DEG:.6f},{th*DEG:.6f},{psi*DEG:.6f},"
                    f"{s.omega[0]:.6f},{s.omega[1]:.6f},{s.omega[2]:.6f},"
                    + ",".join(f"{x*DEG:.6f}" for x in s.delta)
                    + "," + ",".join(f"{x*DEG:.6f}" for x in s.delta_cmd)
                    + f",{M[0]:.6f},{M[1]:.6f},{M[2]:.6f}\n")
            if i < n:
                autopilot_update(P, A, s, C, P.dt)
                apply_commands(C, s)
                servo_step(P, s, P.dt)
                step_rk4(P, s, P.dt)
                t += P.dt
    finally:
        if out is not sys.stdout:
            out.close()
            print(f"ok csv {T:.3f} s -> {archivo}")
    return s, t


def _on_off(tok, i):
    """Devuelve True/False/None a partir de on|off|1|0|si|no."""
    if len(tok) <= i:
        return None
    v = tok[i]
    if v in ("on", "1", "si"):
        return True
    if v in ("off", "0", "no"):
        return False
    return None


REPL_HELP = """mando:
  pitch <deg>          fija u_pitch          (ignorado si el AP esta ON)
  yaw   <deg>          fija u_yaw
  roll  <deg>          fija u_roll
  fin <n> <deg>        deflexion directa de la aleta n (1..4)

autopiloto (§6):
  ap on|off            activa/desactiva los tres lazos SISO
  ref <phi> <th> <psi> referencias de actitud en grados
  hold                 fija la referencia en la actitud actual
  tune <wnR> <wnP> <wnY> <zeta>   reasigna polos y resintoniza
  gains                derivadas de estabilidad y ganancias PID

modelo:
  damp on|off          amortiguamiento aerodinamico (§4)
  flex on|off          aleta flexible / rigida (§5)
  servo on|off         dinamica de actuador (§6)
  set <k> <v>          k = V | rho | V_min | l_t | d | S_fin | b_fin
                           | stall(deg) | dt | Ixx | Iyy | Izz | Ixz
                           | tau_servo | rate_limit(deg/s)
                           | e_ea | K_theta | I_theta | zeta_theta
                           | K_bend | m_bend | zeta_bend | S_theta

simulacion:
  step <seg>           integra <seg> segundos con paso P.dt
  state                imprime el estado completo
  csv <seg> [cada] [archivo]   volcado CSV para figuras
  reset | help | quit"""


def run_repl():
    """Equivalente linea a linea del modo --headless de C++."""
    P, s, C, t = Params(), State(), Commands(), 0.0
    A = autopilot_default(P)
    apply_commands_instant(C, s)
    print("# simulacion_alerones — modo headless.  'help' para la lista.")

    for line in sys.stdin:
        line = line.split("#")[0].strip()
        if not line:
            continue
        tok = line.split()
        cmd = tok[0]
        try:
            if cmd in ("quit", "exit"):
                break

            elif cmd == "help":
                print(REPL_HELP)

            elif cmd == "state":
                print_state(P, s, t, A)

            elif cmd == "gains":
                print_gains(P, A)

            elif cmd == "reset":
                # Igual que sim_reset() en C++: conserva P y la sintonia del
                # AP (enabled, wn, zeta) pero borra estado, refs e integrales.
                s, C, t = State(), Commands(), 0.0
                A.phi_ref = A.theta_ref = A.psi_ref = 0.0
                autopilot_tune(P, A)
                apply_commands_instant(C, s)
                print("ok reset")

            elif cmd in ("pitch", "yaw", "roll"):
                v = float(tok[1])
                setattr(C, "u_" + cmd, v * RAD)
                apply_commands(C, s)
                if A.enabled:
                    print("aviso: el autopiloto esta ON y pisara este mando")
                print(f"ok {cmd} {v:.4f} deg")

            elif cmd == "fin":
                n, v = int(tok[1]), float(tok[2])
                C.fin_trim[n - 1] = v * RAD
                apply_commands(C, s)
                print(f"ok fin {n} {v:.4f} deg")

            elif cmd == "ap":
                on = _on_off(tok, 1)
                if on is None:
                    print("error: uso 'ap on|off'")
                    continue
                A.enabled = on
                if on:
                    A.phi_ref, A.theta_ref, A.psi_ref = euler_from_quat(s.q)
                    autopilot_tune(P, A)
                print(f"ok ap {'on' if on else 'off'}")

            elif cmd == "ref":
                a, b, c = float(tok[1]), float(tok[2]), float(tok[3])
                A.phi_ref, A.theta_ref, A.psi_ref = a * RAD, b * RAD, c * RAD
                print(f"ok ref {a:.3f} {b:.3f} {c:.3f} deg")

            elif cmd == "hold":
                A.phi_ref, A.theta_ref, A.psi_ref = euler_from_quat(s.q)
                print(f"ok hold  phi {A.phi_ref*DEG:.3f}  theta {A.theta_ref*DEG:.3f}  "
                      f"psi {A.psi_ref*DEG:.3f} deg")

            elif cmd == "tune":
                A.wn_roll, A.wn_pitch = float(tok[1]), float(tok[2])
                A.wn_yaw, A.zeta = float(tok[3]), float(tok[4])
                autopilot_tune(P, A)
                print(f"ok tune  wn = {A.wn_roll:.3f} / {A.wn_pitch:.3f} / "
                      f"{A.wn_yaw:.3f} rad/s   zeta = {A.zeta:.3f}")

            elif cmd == "damp":
                on = _on_off(tok, 1)
                if on is None:
                    print("error: uso 'damp on|off'")
                    continue
                P.damping_on = on
                autopilot_tune(P, A)
                print(f"ok damp {'on' if on else 'off'}")

            elif cmd == "flex":
                on = _on_off(tok, 1)
                if on is None:
                    print("error: uso 'flex on|off'")
                    continue
                P.flex_on = on
                if not on:                 # al volver a rigida, borra el estado
                    s.theta[:] = s.theta_dot[:] = 0.0
                    s.w[:] = s.w_dot[:] = 0.0
                autopilot_tune(P, A)
                extra = (f"  (eta = {aeroelastic_efficiency(P):.6f}, "
                         f"CFL = {structural_cfl(P):.3f})" if on else "")
                print(f"ok flex {'on' if on else 'off'}{extra}")
                if structural_unstable(P):
                    print(f"aviso: CFL {structural_cfl(P):.3f} > 2 con dt = {P.dt:.4f} s: "
                          f"la integracion divergira")

            elif cmd == "servo":
                on = _on_off(tok, 1)
                if on is None:
                    print("error: uso 'servo on|off'")
                    continue
                if on:
                    if P.tau_servo <= 0.0:
                        P.tau_servo = 0.05
                    if P.rate_limit <= 0.0:
                        P.rate_limit = 240.0 * RAD
                else:
                    P.tau_servo = P.rate_limit = 0.0
                print(f"ok servo {'on' if on else 'off'} "
                      f"(tau {P.tau_servo:.4f} s, {P.rate_limit*DEG:.1f} deg/s)")

            elif cmd == "step":
                T = float(tok[1])
                n = max(0, round(T / P.dt))
                for _ in range(n):
                    autopilot_update(P, A, s, C, P.dt)
                    apply_commands(C, s)
                    servo_step(P, s, P.dt)
                    step_rk4(P, s, P.dt)
                    t += P.dt
                print(f"ok step {n} pasos ({n*P.dt:.6f} s)  t = {t:.6f} s")

            elif cmd == "csv":
                T = float(tok[1])
                cada = float(tok[2]) if len(tok) > 2 else 0.0
                archivo = tok[3] if len(tok) > 3 else ""
                s, t = dump_csv(P, A, s, t, T, cada, archivo)

            elif cmd == "set":
                k, v = tok[1], float(tok[2])
                geom = k in ("S_fin", "b_fin")
                if k == "stall":
                    P.delta_stall = v * RAD
                elif k == "rate_limit":
                    P.rate_limit = v * RAD
                elif hasattr(P, k) and k not in ("qbar", "v_eff", "k_fin"):
                    setattr(P, k, v)
                else:
                    print(f"error: clave desconocida '{k}'")
                    continue
                # Tocar la geometria cambia AR, y AR fija CL_delta.
                if geom:
                    params_update_aero(P)
                autopilot_tune(P, A)
                if geom:
                    print(f"ok set {k} {v:.6f}  "
                          f"(AR = {aspect_ratio(P.b_fin, P.S_fin):.4f} -> "
                          f"CL_delta = {P.CL_delta:.6f})")
                else:
                    print(f"ok set {k} {v:.6f}")

                # Cambiar dt o la rigidez puede dejar el paso corto para los
                # modos estructurales; mas vale decirlo aqui que ver NaN dentro
                # de 200 pasos y no saber de donde salio.
                if structural_unstable(P):
                    print(f"aviso: CFL {structural_cfl(P):.3f} > 2 con dt = {P.dt:.4f} s: "
                          f"la integracion divergira (baja dt o usa 'flex off')")

            else:
                print(f"error: comando desconocido '{cmd}' (prueba 'help')")

        except (IndexError, ValueError) as e:
            print(f"error: {e}")
    return 0


# ─────────────────────────────────────────────────────────────────────────────
# Animacion 3D interactiva
# ─────────────────────────────────────────────────────────────────────────────

FUS_R = 0.16

# La flecha real de la punta es de decimas de milimetro a 30 m/s: fisicamente
# correcta e invisible.  Se exagera para dibujar, como en cualquier post de
# elementos finitos, y el factor se anuncia en el panel.  La tecla F lo cicla.
FLEX_SCALES = (1.0, 20.0, 60.0, 150.0)
FIN_CHORD = 0.16
FIN_TIP_C = 0.10


# Modo de flexion de un voladizo con carga en la punta, normalizado a 1 en la
# punta:  phi(xi) = (3 xi^2 - xi^3)/2.  Es la deformada estatica exacta.
def _bend_shape(xi):
    return 0.5 * (3.0 * xi * xi - xi * xi * xi)


NSEG_FIN = 8            # tiras a lo largo de la envergadura, para ver la curva


def _fin_strips(P, idx, delta, wtip):
    """La aleta idx troceada en NSEG_FIN cuadrilateros, en ejes cuerpo.

    delta es el angulo TOTAL (servo + torsion elastica): como K_theta es una
    rigidez concentrada en el pivote, la aleta se retuerce en bloque y la
    torsion entra en la rotacion, no en la forma.

    wtip es la flecha de la punta ya escalada para dibujar.  Va en el eje
    local de flexion, que en las cuatro aletas apunta donde empuja el aire.

    Aletas 1-2 (horizontales): giro -delta sobre y, flexion en +z del cuerpo.
    Aletas 3-4 (verticales)  : giro +delta sobre z, flexion en +y del cuerpo.
    """
    span = P.b_fin
    cr, ct = FIN_CHORD / 2, FIN_TIP_C / 2
    x_t = -P.l_t

    # Estaciones a lo largo de la envergadura: (semicuerda, s, flecha local)
    est = []
    for j in range(NSEG_FIN + 1):
        xi = j / NSEG_FIN
        est.append((cr + (ct - cr) * xi, xi * span, wtip * _bend_shape(xi)))

    def punto(xc, sp, zb):
        if idx < 2:                   # derecha (+y) / izquierda (-y)
            ysgn = 1.0 if idx == 0 else -1.0
            c, sn = math.cos(-delta), math.sin(-delta)
            # rotacion -delta sobre y:  (x,z) -> (x c + z s, -x s + z c)
            return [x_t + xc * c + zb * sn, ysgn * (FUS_R + sp), -xc * sn + zb * c]
        # arriba (-z) / abajo (+z)
        zsgn = -1.0 if idx == 2 else 1.0
        c, sn = math.cos(delta), math.sin(delta)
        # rotacion +delta sobre z:  (x,y) -> (x c - y s, x s + y c), con el eje
        # local de flexion mapeado a +y del cuerpo
        return [x_t + xc * c - zb * sn, xc * sn + zb * c, zsgn * (FUS_R + sp)]

    quads = []
    for j in range(NSEG_FIN):
        c0, s0, z0 = est[j]
        c1, s1, z1 = est[j + 1]
        quads.append(np.array([punto(+c0, s0, z0), punto(-c0, s0, z0),
                               punto(-c1, s1, z1), punto(+c1, s1, z1)]))
    return quads


def run_animation():
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    P, s, C = Params(), State(), Commands()
    A = autopilot_default(P)
    apply_commands_instant(C, s)
    state = {"t": 0.0, "paused": False, "fin_sel": 0,
             "tau_saved": 0.0, "rate_saved": 0.0, "flex_idx": 2}
    DCMD = 0.5 * RAD      # mando directo
    DREF = 2.0 * RAD      # referencia del autopiloto

    fig = plt.figure(figsize=(11, 7))
    fig.canvas.manager.set_window_title("simulacion_alerones (python)")
    ax = fig.add_subplot(111, projection="3d")
    fig.subplots_adjust(left=0.0, right=0.66, bottom=0.0, top=1.0)
    txt = fig.text(0.68, 0.97, "", va="top", family="monospace", fontsize=8.5)

    # Fuselaje: cilindro a lo largo de x del cuerpo
    th = np.linspace(0, 2 * np.pi, 24)
    x_axis = np.array([-P.l_t - 0.05, P.l_t * 0.85])

    def body_geometry(s):
        """(fuselaje, tiras de las 4 aletas) en ejes cuerpo.

        Las aletas se dibujan con el angulo TOTAL (servo + torsion) y con su
        flecha real exagerada: la de verdad es de decimas de milimetro (§5).
        """
        fus = []
        for a0, a1 in zip(th[:-1], th[1:]):
            for x0, x1 in [(x_axis[0], x_axis[1])]:
                fus.append(np.array([
                    [x0, FUS_R * math.cos(a0), FUS_R * math.sin(a0)],
                    [x1, FUS_R * math.cos(a0), FUS_R * math.sin(a0)],
                    [x1, FUS_R * math.cos(a1), FUS_R * math.sin(a1)],
                    [x0, FUS_R * math.cos(a1), FUS_R * math.sin(a1)],
                ]))
        fs = FLEX_SCALES[state["flex_idx"]]
        fins = []
        for i in range(4):
            ang = s.delta[i] + (s.theta[i] if P.flex_on else 0.0)
            wtip = s.w[i] * fs if P.flex_on else 0.0
            fins.append(_fin_strips(P, i, ang, wtip))
        return fus, fins

    fus_coll = Poly3DCollection([], facecolor="#c9cdd6", edgecolor="none", alpha=0.85)
    fin_coll = Poly3DCollection([], edgecolor="#20242c", linewidths=0.6)
    ax.add_collection3d(fus_coll)
    ax.add_collection3d(fin_coll)

    # Con el autopiloto activo el piloto mueve REFERENCIAS de actitud; con el
    # autopiloto apagado mueve directamente los canales virtuales.  Es la misma
    # palanca en las dos capas del lazo.
    def on_key(ev):
        k = ev.key
        if k == "up":
            if A.enabled:
                A.theta_ref += DREF
            else:
                C.u_pitch += DCMD
        elif k == "down":
            if A.enabled:
                A.theta_ref -= DREF
            else:
                C.u_pitch -= DCMD
        elif k == "left":
            if A.enabled:
                A.psi_ref -= DREF
            else:
                C.u_yaw -= DCMD
        elif k == "right":
            if A.enabled:
                A.psi_ref += DREF
            else:
                C.u_yaw += DCMD
        elif k == "q":
            if A.enabled:
                A.phi_ref -= DREF
            else:
                C.u_roll -= DCMD
        elif k == "e":
            if A.enabled:
                A.phi_ref += DREF
            else:
                C.u_roll += DCMD
        elif k in ("1", "2", "3", "4"):
            state["fin_sel"] = int(k) - 1
        elif k in ("+", "="):
            C.fin_trim[state["fin_sel"]] += DCMD
        elif k in ("-", "_"):
            C.fin_trim[state["fin_sel"]] -= DCMD
        elif k == "0":
            C.u_pitch = C.u_yaw = C.u_roll = 0.0
            C.fin_trim[:] = 0.0
            A.phi_ref = A.theta_ref = A.psi_ref = 0.0
            A.roll.i_state = A.pitch.i_state = A.yaw.i_state = 0.0
        elif k == "a":                      # autopiloto on/off
            A.enabled = not A.enabled
            if A.enabled:                   # engancha sin salto
                A.phi_ref, A.theta_ref, A.psi_ref = euler_from_quat(s.q)
                autopilot_tune(P, A)
        elif k == "c":                      # "hold": congela esta actitud
            A.phi_ref, A.theta_ref, A.psi_ref = euler_from_quat(s.q)
        elif k == "d":                      # amortiguamiento aerodinamico (§4)
            P.damping_on = not P.damping_on
            autopilot_tune(P, A)
        elif k == "f":                      # aleta flexible / rigida (§5)
            P.flex_on = not P.flex_on
            if not P.flex_on:               # al volver a rigida, borra el estado
                s.theta[:] = s.theta_dot[:] = 0.0
                s.w[:] = s.w_dot[:] = 0.0
            autopilot_tune(P, A)
        elif k == "F":                      # cicla la exageracion del dibujo
            state["flex_idx"] = (state["flex_idx"] + 1) % len(FLEX_SCALES)
        elif k == "s":                      # dinamica de actuador (§6)
            if P.tau_servo > 0.0 or P.rate_limit > 0.0:
                state["tau_saved"], state["rate_saved"] = P.tau_servo, P.rate_limit
                P.tau_servo = P.rate_limit = 0.0
            else:
                P.tau_servo = state["tau_saved"] or 0.05
                P.rate_limit = state["rate_saved"] or 240.0 * RAD
        elif k in (",", "<", ".", ">"):     # la autoridad va con qbar
            P.V = max(1.0, P.V + (2.0 if k in (".", ">") else -2.0))
            autopilot_tune(P, A)
        elif k == "r":
            s.q[:] = [1.0, 0.0, 0.0, 0.0]
            s.omega[:] = 0.0
            s.delta[:] = 0.0
            C.u_pitch = C.u_yaw = C.u_roll = 0.0
            C.fin_trim[:] = 0.0
            A.roll.i_state = A.pitch.i_state = A.yaw.i_state = 0.0
            state["t"] = 0.0
        elif k == " ":
            state["paused"] = not state["paused"]

        lim = 3.0 * P.delta_stall
        if A.enabled:
            # Los canales virtuales los escribe el AP, ya saturados por u_max;
            # aqui solo se limitan las referencias.
            lref = 80.0 * RAD                       # evita el gimbal lock
            A.theta_ref = float(np.clip(A.theta_ref, -lref, lref))
            A.phi_ref = wrap_pi(A.phi_ref)
            A.psi_ref = wrap_pi(A.psi_ref)
        else:
            C.u_pitch = float(np.clip(C.u_pitch, -lim, lim))
            C.u_yaw = float(np.clip(C.u_yaw, -lim, lim))
            C.u_roll = float(np.clip(C.u_roll, -lim, lim))
        C.fin_trim[:] = np.clip(C.fin_trim, -lim, lim)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_):
        # Mismo orden que sim_step() en C++: AP -> mixing -> actuador -> RK4.
        if not state["paused"]:
            for _ in range(int(round((1 / 30.0) / P.dt))):
                autopilot_update(P, A, s, C, P.dt)
                apply_commands(C, s)
                servo_step(P, s, P.dt)
                step_rk4(P, s, P.dt)
                state["t"] += P.dt
        else:
            apply_commands(C, s)

        R = rotmat_from_quat(s.q)
        fus, fins = body_geometry(s)
        fus_coll.set_verts([(R @ p.T).T for p in fus])
        # Cada aleta son NSEG_FIN tiras: se aplanan y el color se repite.
        fin_coll.set_verts([(R @ q.T).T for tiras in fins for q in tiras])

        sel = state["fin_sel"]
        ang_tot = s.delta + (s.theta if P.flex_on else 0.0)
        mag = np.clip(np.abs(ang_tot) / P.delta_stall, 0, 1)
        fin_coll.set_facecolor([
            "#ffee55" if i == sel else (0.30 + 0.65 * m, 0.62 - 0.30 * m, 0.85 - 0.55 * m)
            for i, m in enumerate(mag) for _ in range(NSEG_FIN)
        ])

        phi, tht, psi = euler_from_quat(s.q)
        up, uy, ur = demix(s.delta)
        f = fin_forces(P, s.delta)
        M = fin_moments_rate(P, s.delta, s.theta, s.omega)
        de = fin_effective_deflection(P, s.delta, s.theta, s.omega)
        L_p, M_q, N_r = damping_coeffs(P)
        modo = "neutro"
        if max(abs(up), abs(uy), abs(ur)) > 1e-6:
            if abs(ur) >= abs(up) and abs(ur) >= abs(uy):
                modo = "ANTISIMETRICO (roll)"
            elif abs(up) >= abs(uy):
                modo = "SIMETRICO h (pitch)"
            else:
                modo = "SIMETRICO v (yaw)"

        txt.set_text(
            f"t = {state['t']:7.2f} s  {'[PAUSA]' if state['paused'] else ''}\n\n"
            f"ACTITUD\n"
            f"  phi {phi*DEG:+8.2f}  theta {tht*DEG:+8.2f}  psi {psi*DEG:+8.2f} deg\n"
            f"  p   {s.omega[0]:+8.3f}  q     {s.omega[1]:+8.3f}  r   {s.omega[2]:+8.3f} rad/s\n\n"
            f"DEFLEXIONES [deg]   (stall +-{P.delta_stall*DEG:.1f})\n"
            f"  d1 {s.delta[0]*DEG:+7.2f}  d2 {s.delta[1]*DEG:+7.2f}"
            f"  d3 {s.delta[2]*DEG:+7.2f}  d4 {s.delta[3]*DEG:+7.2f}\n"
            f"  cmd{s.delta_cmd[0]*DEG:+7.2f} {s.delta_cmd[1]*DEG:+7.2f}"
            f" {s.delta_cmd[2]*DEG:+7.2f} {s.delta_cmd[3]*DEG:+7.2f}"
            f"   (servo {P.tau_servo:.3f} s)\n"
            f"  eff{de[0]*DEG:+7.2f} {de[1]*DEG:+7.2f}"
            f" {de[2]*DEG:+7.2f} {de[3]*DEG:+7.2f}\n"
            f"  f1 {f[0]:+7.2f}  f2 {f[1]:+7.2f}"
            f"  f3 {f[2]:+7.2f}  f4 {f[3]:+7.2f}   N\n\n"
            f"CANALES (irreps de V4) [deg]\n"
            f"  u_pitch {up*DEG:+7.2f}  u_yaw {uy*DEG:+7.2f}  u_roll {ur*DEG:+7.2f}\n"
            f"  L {M[0]:+8.3f}  M {M[1]:+8.3f}  N {M[2]:+8.3f} N*m\n"
            f"  modo: {modo}\n\n"
            f"AMORTIGUAMIENTO [{'ON' if P.damping_on else 'OFF'}]  (matriz diagonal)\n"
            f"  L_p {L_p:+8.4f}  M_q {M_q:+8.4f}  N_r {N_r:+8.4f}\n\n"
            f"ALETA {'FLEXIBLE' if P.flex_on else 'RIGIDA'}"
            f"  (x{FLEX_SCALES[state['flex_idx']]:.0f} en el dibujo)\n"
            + ((f"  torsion {s.theta[0]*DEG:+6.3f} {s.theta[1]*DEG:+6.3f}"
                f" {s.theta[2]*DEG:+6.3f} {s.theta[3]*DEG:+6.3f} deg\n"
                f"  flecha  {1000*s.w[0]:+6.3f} {1000*s.w[1]:+6.3f}"
                f" {1000*s.w[2]:+6.3f} {1000*s.w[3]:+6.3f} mm\n"
                f"  eta {aeroelastic_efficiency(P):.4f}"
                f"   qbar/qbar_car {(P.qbar/qbar_caracteristica(P) if qbar_caracteristica(P) > 0 else 0):.3f}"
                f"{'  *** DIVERGENCIA ***' if aeroelastic_divergence(P) else ''}\n"
                + ("  *** dt grande: CFL %.2f > 2 ***\n" % structural_cfl(P)
                   if structural_unstable(P) else ""))
               if P.flex_on else "")
            + "\n"
            f"AUTOPILOTO [{'ON' if A.enabled else 'OFF'}]  (3 lazos SISO)\n"
            f"  ref phi {A.phi_ref*DEG:+7.2f}  theta {A.theta_ref*DEG:+7.2f}"
            f"  psi {A.psi_ref*DEG:+7.2f}\n"
            f"  wn {A.wn_roll:.2f}/{A.wn_pitch:.2f}/{A.wn_yaw:.2f} rad/s"
            f"   zeta {A.zeta:.2f}\n\n"
            f"AIRE\n"
            f"  V {P.V:.1f} m/s   qbar {P.qbar:.1f} Pa\n"
            f"  CL_delta {P.CL_delta:.4f} 1/rad\n\n"
            + (f"teclas: arr/aba theta_ref | izq/der psi_ref | q/e phi_ref\n"
               if A.enabled else
               f"teclas: arr/aba u_pitch | izq/der u_yaw | q/e u_roll\n")
            + f"        1..4 aleta (sel: d{sel+1}) | +/- deflexion\n"
            f"        a autopiloto | d amortiguam. | s servo | c hold\n"
            f"        f aleta flexible/rigida | F exagerar deformacion\n"
            f"        0 centrar | r reset | space pausa | , . velocidad"
        )
        return fus_coll, fin_coll

    L = P.l_t
    ax.set_xlim(-L, L); ax.set_ylim(-L, L); ax.set_zlim(-L, L)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("x cuerpo (adelante)")
    ax.set_ylabel("y cuerpo (derecha)")
    ax.set_zlabel("z cuerpo (abajo)")
    ax.invert_zaxis()          # z abajo, como en ejes cuerpo
    ax.view_init(elev=18, azim=-125)

    _anim = FuncAnimation(fig, update, interval=33, blit=False, cache_frame_data=False)
    plt.show()
    return 0


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    if "--test" in sys.argv:
        sys.exit(run_tests())
    elif "--repl" in sys.argv:
        sys.exit(run_repl())
    else:
        sys.exit(run_animation())
