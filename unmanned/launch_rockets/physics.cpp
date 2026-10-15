#include "physics.h"
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Vec3 thrustDir(const LaunchParams& p) {
    // inclination respecto to horizontal line...
    // cardinal point you point out ...
    // from normal degrees -> radians (c++ works on it)
    float theta = p.theta_deg * (float)M_PI / 180.f;
    float phi   = p.phi_deg   * (float)M_PI / 180.f;

    // Standard physics spherical → Cartesian (x=East, y=North, z=Up)
    // θ ∈ [0, π]   — polar angle from zenith (+Z)
    // φ ∈ [0, 2π)  — azimuthal angle from East (+X)
    return Vec3(sinf(theta) * cosf(phi),   // x = r·sin(θ)·cos(φ)
                sinf(theta) * sinf(phi),   // y = r·sin(θ)·sin(φ)
                cosf(theta));              // z = r·cos(θ)
}

// Rates of change for all state variables:  implements the Euler-Lagrange equations
struct Deriv { Vec3 dp, dv; float dm; };

static Deriv computeDerivs(const RocketState& s, const LaunchParams& p) {
    Deriv d;
    d.dp = s.vel;   // dq/dt = q̇

    float dry_mass = p.total_mass_kg - p.fuel_mass_kg;

    // ── Gravity: ∂V/∂z = m·g  → generalized force in -z direction ──────────
    Vec3 Fg(0.f, 0.f, -GRAVITY * s.mass);

    // ── Thrust: F = Isp · g₀ · ṁ  (rocket equation, active during burn) ──────
    Vec3 Ft;
    d.dm = 0.f;
    if (s.t < p.burn_time_s && s.mass > dry_mass + 0.01f) {
        d.dm = -p.mdot();
        Ft   = thrustDir(p) * p.thrustN();
    }

    // ── Aerodynamic drag with ISA exponential atmosphere ─────────────────────
    // ρ(y) = ρ₀ · e^(−y/H),  H = 8500 m (scale height)
    float rho  = AIR_DENSITY * expf(-s.pos.z / 8500.f);
    float spd  = s.vel.mag();
    Vec3  Fd;
    if (spd > 0.01f) {
        float drag_mag = 0.5f * rho * p.Cd * p.area_m2 * spd * spd;
        Fd = s.vel.norm() * (-drag_mag);
    }

    // Euler-Lagrange: m·q̈ = Fg + Ft + Fd
    Vec3 Fnet = Fg + Ft + Fd;
    d.dv = Fnet / s.mass;   // dq̇/dt = q̈

    return d;
}

static RocketState stateAdvance(const RocketState& s, const Deriv& d, float dt) {
    RocketState ns;
    ns.pos  = s.pos + d.dp * dt;
    ns.vel  = s.vel + d.dv * dt;
    ns.mass = s.mass + d.dm * dt;
    ns.t    = s.t + dt;
    return ns;
}

// Classical 4th-order Runge-Kutta integration of the Euler-Lagrange equations
RocketState rk4Step(const RocketState& s, const LaunchParams& p, float dt) {
    // s -> current state
    // p -> design paramters
    Deriv k1 = computeDerivs(s, p);

    RocketState s2 = stateAdvance(s, k1, dt * 0.5f);
    Deriv k2 = computeDerivs(s2, p);

    RocketState s3 = stateAdvance(s, k2, dt * 0.5f);
    Deriv k3 = computeDerivs(s3, p);

    RocketState s4 = stateAdvance(s, k3, dt);
    Deriv k4 = computeDerivs(s4, p);

    // Weighted combination: y_{n+1} = y_n + (dt/6)(k1 + 2k2 + 2k3 + k4)
    RocketState r;
    r.t = s.t + dt;
    r.pos.x = s.pos.x + (k1.dp.x + 2*k2.dp.x + 2*k3.dp.x + k4.dp.x) * dt/6.f;
    r.pos.y = s.pos.y + (k1.dp.y + 2*k2.dp.y + 2*k3.dp.y + k4.dp.y) * dt/6.f;
    r.pos.z = s.pos.z + (k1.dp.z + 2*k2.dp.z + 2*k3.dp.z + k4.dp.z) * dt/6.f;
    r.vel.x = s.vel.x + (k1.dv.x + 2*k2.dv.x + 2*k3.dv.x + k4.dv.x) * dt/6.f;
    r.vel.y = s.vel.y + (k1.dv.y + 2*k2.dv.y + 2*k3.dv.y + k4.dv.y) * dt/6.f;
    r.vel.z = s.vel.z + (k1.dv.z + 2*k2.dv.z + 2*k3.dv.z + k4.dv.z) * dt/6.f;

    float dry_mass = p.total_mass_kg - p.fuel_mass_kg;
    r.mass = std::max(
        s.mass + (k1.dm + 2*k2.dm + 2*k3.dm + k4.dm) * dt/6.f,
        dry_mass
    );

    return r;
}
