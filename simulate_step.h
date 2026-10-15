/*
simulate_step.h

simulateStep() — integrador RK4 que expone el TENSOR DE INERCIA COMPLETO
(momentos Ixx,Iyy,Izz + productos de inercia Ixy,Ixz,Iyz, no necesariamente
nulos) al integrador, en lugar de la forma reducida {Jx,Jy,Jz,Jxz} que usa
integrate_rk4() en physics_UAV.h y que asume simetria de masa respecto al
plano XZ (Ixy = Iyz = 0, solo Ixz sobrevive).

Requisito que cubre (adaptado de Beard & McLain a nuestro dron):
  - inputs de la funcion: fuerzas y momentos aplicados al dron en el marco
    cuerpo (fx,fy,fz,l,m_torque,n) — Bloques 2 y 4.
  - parametros modificables: masa, tensor de inercia completo (Inertia3x3),
    y la condicion inicial de cada uno de los 12 estados (UAVState s).

Ecuacion general de Euler para un cuerpo rigido (sin asumir Ixy=Iyz=0):
    I * omega_dot = M - omega x (I * omega)
    omega_dot     = I^-1 * (M - omega x I*omega)

donde I es la matriz 3x3 [[Ixx,Ixy,Ixz],[Ixy,Iyy,Iyz],[Ixz,Iyz,Izz]] (el
signo de los productos de inercia ya viene resuelto asi en Inertia3x3 —
ver trasladar() en physics_UAV.h, teorema de ejes paralelos).

Se implementa en un subfile aparte de physics_UAV.h porque generaliza el
Bloque 4 (rotational_dynamics) a costa de una inversion de matriz 3x3 en
cada evaluacion — mas caro que las constantes Gamma precalculadas que usa
rotational_dynamics(), y solo hace falta cuando la geometria no es simetrica
respecto al plano XZ.
*/

#ifndef SIMULATE_STEP_H
#define SIMULATE_STEP_H

#include "physics_UAV.h"

// ─────────────────────────────────────────────────────────────────────────────
// Bloque 4 generalizado — dinamica rotacional con tensor de inercia completo
// ─────────────────────────────────────────────────────────────────────────────
inline Vec3 rotational_dynamics_full(const UAVState& s,
                                      double l, double m_torque, double n,
                                      const Inertia3x3& I)
{
    const Vec3 omega(s.p, s.q, s.r);

    // I * omega
    const Vec3 Iw(
        I.Ixx*omega.x + I.Ixy*omega.y + I.Ixz*omega.z,
        I.Ixy*omega.x + I.Iyy*omega.y + I.Iyz*omega.z,
        I.Ixz*omega.x + I.Iyz*omega.y + I.Izz*omega.z
    );

    // omega x (I*omega) — termino giroscopico (acopla los 3 ejes cuando
    // hay productos de inercia no nulos)
    const Vec3 gyro(
        omega.y*Iw.z - omega.z*Iw.y,
        omega.z*Iw.x - omega.x*Iw.z,
        omega.x*Iw.y - omega.y*Iw.x
    );

    const Vec3 rhs(l - gyro.x, m_torque - gyro.y, n - gyro.z);

    // Inversion de la matriz de inercia 3x3 simetrica (regla de Cramer /
    // matriz adjunta) — I es la matriz completa, no la forma reducida.
    const double a=I.Ixx, b=I.Ixy, c=I.Ixz;
    const double d=I.Ixy, e=I.Iyy, f=I.Iyz;
    const double g=I.Ixz, h=I.Iyz, i=I.Izz;

    const double det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);

    if (std::abs(det) < 1e-9) {
        std::cerr << "[WARNING] simulateStep(): tensor de inercia singular, "
                     "no se puede invertir.\n";
        return Vec3(0.0, 0.0, 0.0);
    }

    const double invDet = 1.0 / det;
    const double m00 =  (e*i - f*h) * invDet;
    const double m01 = -(b*i - c*h) * invDet;
    const double m02 =  (b*f - c*e) * invDet;
    const double m11 =  (a*i - c*g) * invDet;
    const double m12 = -(a*f - c*d) * invDet;
    const double m22 =  (a*e - b*d) * invDet;

    return Vec3(m00*rhs.x + m01*rhs.y + m02*rhs.z,
                m01*rhs.x + m11*rhs.y + m12*rhs.z,
                m02*rhs.x + m12*rhs.y + m22*rhs.z);
}

// Derivada completa del estado (Bloques 1-3 igual que siempre + Bloque 4
// generalizado) usando el tensor de inercia completo
inline StateDot state_derivative_full(const UAVState& s,
                                       double fx, double fy, double fz,
                                       double l,  double m_torque, double n,
                                       const Inertia3x3& I, double mass)
{
    const Vec3 pos_dot   = translational_kinematics(s);                    // Bloque 1
    const Vec3 vel_dot   = translational_dynamics(s, fx, fy, fz, mass);    // Bloque 2
    const Vec3 euler_dot = rotational_kinematics(s);                      // Bloque 3
    const Vec3 rate_dot  = rotational_dynamics_full(s, l, m_torque, n, I); // Bloque 4 (completo)

    return { pos_dot.x, pos_dot.y, pos_dot.z,
             vel_dot.x, vel_dot.y, vel_dot.z,
             euler_dot.x, euler_dot.y, euler_dot.z,
             rate_dot.x, rate_dot.y, rate_dot.z };
}

// ─────────────────────────────────────────────────────────────────────────────
// simulateStep — un paso de integracion RK4 con tensor de inercia completo
//
//   inputs:      fuerzas (fx,fy,fz) y momentos (l,m_torque,n) en marco cuerpo
//   modificable: mass, I (Inertia3x3 — momentos + productos de inercia), dt
//   estado:      s — los 12 componentes; la condicion inicial la fija el
//                llamador (misma UAVState que usa el resto del motor)
// ─────────────────────────────────────────────────────────────────────────────
inline UAVState simulateStep(const UAVState& s,
                              double fx, double fy, double fz,
                              double l,  double m_torque, double n,
                              const Inertia3x3& I,
                              double mass, double dt)
{
    const StateDot k1 = state_derivative_full(s,  fx,fy,fz, l,m_torque,n, I, mass);
    const UAVState s2 = state_add_scaled(s, k1, dt / 2.0);

    const StateDot k2 = state_derivative_full(s2, fx,fy,fz, l,m_torque,n, I, mass);
    const UAVState s3 = state_add_scaled(s, k2, dt / 2.0);

    const StateDot k3 = state_derivative_full(s3, fx,fy,fz, l,m_torque,n, I, mass);
    const UAVState s4 = state_add_scaled(s, k3, dt);

    const StateDot k4 = state_derivative_full(s4, fx,fy,fz, l,m_torque,n, I, mass);

    return rk4_combine(s, k1, k2, k3, k4, dt);
}

#endif // SIMULATE_STEP_H
