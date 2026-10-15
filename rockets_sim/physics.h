#pragma once
#include <cmath>

// ============================================================
// Lagrangian Mechanics - Rocket in 3D
//
// Generalized coordinates: q = (x, y, z)  — x=East, y=North, z=Up
// Kinetic energy:   T = ½ m(t) (ẋ² + ẏ² + ż²)
// Potential energy: V = m(t) g z
// Lagrangian:       L = T - V
//
// Euler-Lagrange equations with non-conservative generalized forces Qi:
//   d/dt(∂L/∂q̇i) − ∂L/∂qi = Qi
//
//   m ẍ = Qx = Fx_thrust + Fx_drag
//   m ÿ = Qy = Fy_thrust + Fy_drag
//   m z̈ = −mg + Qz = −mg + Fz_thrust + Fz_drag
//   ṁ    = −fuel_flow_rate  (during burn; mass consumption modifies T)
// ============================================================

const float GRAVITY     = 9.81f;    // m/s²
const float AIR_DENSITY = 1.225f;   // kg/m³ at sea level

// 3-vector with arithmetic ops
struct Vec3 {
    float x, y, z;

    Vec3() : x(0.f), y(0.f), z(0.f) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return Vec3(x+o.x, y+o.y, z+o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x-o.x, y-o.y, z-o.z); }
    Vec3 operator*(float s)       const { return Vec3(x*s,   y*s,   z*s);   }
    Vec3 operator/(float s)       const { return Vec3(x/s,   y/s,   z/s);   }

    float mag()  const { return sqrtf(x*x + y*y + z*z); }
    Vec3  norm() const { float m = mag(); return m > 1e-7f ? *this/m : Vec3(); }
};

// Full rocket state — generalized coordinates q and velocities q̇
struct RocketState {
    Vec3  pos;           // q  = (x, y, z)  [m], origin at launch pad
    Vec3  vel;           // q̇  = (ẋ, ẏ, ż)  [m/s]
    float mass;          // m(t) [kg], decreases as fuel burns
    float t;             // elapsed time [s]

    RocketState() : mass(1.f), t(0.f) {}
};

// Launch parameters — define the initial conditions and forcing
struct LaunchParams {
    float theta_deg;       // polar angle from +Z (zenith) [°]: 0=up, 90=horizontal, 180=down
    float phi_deg;         // azimuthal angle from +X (East) [°]: 0=East, 90=North, 180=West
    float Isp_s;           // specific impulse [s]  — replaces direct thrust input
    float burn_time_s;     // active burn duration [s]
    float total_mass_kg;   // wet mass (structure + propellant) [kg]
    float fuel_mass_kg;    // propellant mass only [kg]
    float Cd;              // aerodynamic drag coefficient
    float area_m2;         // rocket cross-sectional area [m²]
  

    LaunchParams()
        : theta_deg(20.f), phi_deg(255.f),  // 20° from zenith, toward camera
          Isp_s(100.f),
          burn_time_s(2.f),
          total_mass_kg(12.f), fuel_mass_kg(0.8f),
          Cd(0.5f), area_m2(0.05f) {}

    // Derived propulsion values
    float mdot()          const { return fuel_mass_kg / burn_time_s; }
    float thrustN()       const { return Isp_s * GRAVITY * mdot(); }
    float dryMass()       const { return total_mass_kg - fuel_mass_kg; }
    float massRatio()     const { float d = dryMass(); return d > 0.1f ? total_mass_kg / d : 9999.f; }
    float deltaV()        const { return Isp_s * GRAVITY * logf(massRatio()); }
    float thrustToWeight() const { return thrustN() / (total_mass_kg * GRAVITY); }
};

// Unit vector in thrust direction derived from launch angles
Vec3 thrustDir(const LaunchParams& p);

// One RK4 step integrating the Euler-Lagrange equations of motion
RocketState rk4Step(const RocketState& s, const LaunchParams& p, float dt);
