/*
physics_UAV.cpp

Demo de consola de la fisica real del UAV. Las ecuaciones de movimiento
(Beard & McLain Cap. 3) y la geometria de masa/inercia viven en
physics_UAV.h, para que drone_model.cpp (el motor de visualizacion) consuma
exactamente el mismo modelo.
*/

#include <iostream>
#include <iomanip>
#include "physics_UAV.h"
#include "simulate_step.h"

void printMassProperties(const DroneMassProperties& mp,
                         const char* titulo = "dron ala delta")
{
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "=== Geometria de masa / inercia (" << titulo << ") ===\n";
    std::cout << "Masa total : " << mp.mass << " kg\n";
    std::cout << "CG (x,y,z) : (" << mp.cg.x << ", " << mp.cg.y << ", " << mp.cg.z
              << ") m  [respecto al origen del fuselaje]\n";
    std::cout << "Tensor de inercia respecto al CG [kg*m^2]:\n";
    std::cout << "  Ixx=" << mp.I_full.Ixx << "  Ixy=" << mp.I_full.Ixy << "  Ixz=" << mp.I_full.Ixz << "\n";
    std::cout << "  Iyx=" << mp.I_full.Ixy << "  Iyy=" << mp.I_full.Iyy << "  Iyz=" << mp.I_full.Iyz << "\n";
    std::cout << "  Izx=" << mp.I_full.Ixz << "  Izy=" << mp.I_full.Iyz << "  Izz=" << mp.I_full.Izz << "\n\n";
}


// ─────────────────────────────────────────────────────────────────────────────
// Conversiones de actitud: Euler <-> Cuaternion <-> Matriz de rotacion
// (declaradas en physics_UAV.h; formulas en euler_to_quaternions.jpeg y
// quaternion_to_rotationMatrix.jpeg)
// ─────────────────────────────────────────────────────────────────────────────

// Angulos de Euler (phi=roll, theta=pitch, psi=yaw) -> cuaternion e_b^i
Quaternion eulerToQuaternion(double phi, double theta, double psi)
{
    const double cp2 = std::cos(phi   / 2.0), sp2 = std::sin(phi   / 2.0);
    const double ct2 = std::cos(theta / 2.0), st2 = std::sin(theta / 2.0);
    const double cy2 = std::cos(psi   / 2.0), sy2 = std::sin(psi   / 2.0);

    Quaternion e;
    e.e0 = cy2*ct2*cp2 + sy2*st2*sp2;
    e.e1 = cy2*ct2*sp2 - sy2*st2*cp2;
    e.e2 = cy2*st2*cp2 + sy2*ct2*sp2;
    e.e3 = sy2*ct2*cp2 - cy2*st2*sp2;
    return e;
}

// Cuaternion e_b^i -> angulos de Euler, devueltos como Vec3(phi, theta, psi)
Vec3 quaternionToEuler(const Quaternion& e)
{
    const double e0 = e.e0, e1 = e.e1, e2 = e.e2, e3 = e.e3;

    double phi   = std::atan2(2.0*(e0*e1 + e2*e3), e0*e0 + e3*e3 - e1*e1 - e2*e2);
    double theta = std::asin (2.0*(e0*e2 - e1*e3));
    double psi   = std::atan2(2.0*(e0*e3 + e1*e2), e0*e0 + e1*e1 - e2*e2 - e3*e3);

    return Vec3(phi, theta, psi);
}

// Cuaternion e_b^i -> matriz de rotacion R_b^i (cuerpo -> inercial)
Mat3x3 quaternionToRotationMatrix(const Quaternion& e)
{
    const double e0 = e.e0, e1 = e.e1, e2 = e.e2, e3 = e.e3;

    Mat3x3 R;
    R.m[0][0] = e1*e1 + e0*e0 - e2*e2 - e3*e3;
    R.m[0][1] = 2.0*(e1*e2 - e3*e0);
    R.m[0][2] = 2.0*(e1*e3 + e2*e0);

    R.m[1][0] = 2.0*(e1*e2 + e3*e0);
    R.m[1][1] = e2*e2 + e0*e0 - e1*e1 - e3*e3;
    R.m[1][2] = 2.0*(e2*e3 - e1*e0);

    R.m[2][0] = 2.0*(e1*e3 - e2*e0);
    R.m[2][1] = 2.0*(e2*e3 + e1*e0);
    R.m[2][2] = e3*e3 + e0*e0 - e1*e1 - e2*e2;

    return R;
}

void printQuaternion(const Quaternion& e)
{
    std::cout << "Cuaternion (e0,e1,e2,e3) = ("
              << e.e0 << ", " << e.e1 << ", " << e.e2 << ", " << e.e3 << ")\n";
}

void printRotationMatrix(const Mat3x3& R)
{
    std::cout << "Matriz de rotacion R_b^i:\n";
    for (int i = 0; i < 3; ++i) {
        std::cout << "  [" << R.m[i][0] << "  " << R.m[i][1] << "  " << R.m[i][2] << "]\n";
    }
}


// ─────────────────────────────────────────────────────────────────────────────
// Reporte del convertiplano de 4 rotores basculantes
//
// Recorre la basculacion de 0 (vuelo estacionario) a 90 grados (crucero) y
// muestra las dos cosas que cambian con ella y que no cambian en ningun otro
// vehiculo del simulador:
//
//   1. La geometria de masa (Bloque 0): al bascular, las gondolas y sus
//      rotores rotan alrededor del eje y, asi que el CG se corre y el tensor
//      de inercia cambia. buildTiltrotor(sigma) lo recalcula.
//   2. El reparto de autoridad de control (Bloque 4b): el empuje diferencial
//      entre rotores alabea/cabecea en estacionario y guina en modo avion; la
//      basculacion diferencial hace lo complementario; y las superficies
//      aerodinamicas aparecen recien cuando hay presion dinamica.
// ─────────────────────────────────────────────────────────────────────────────
void reporteConvertiplano()
{
    const TiltrotorGeom G = tiltrotorGeom();
    const AeroCoeffs    A = aeroCoeffsReferencia(tiltrotorAreaAlar(G), G.envergadura,
                                                 tiltrotorCuerdaMedia(G));
    const DroneMassProperties base = buildTiltrotor(0.0);
    const double peso   = base.mass * 9.81;
    const double Tmax   = 2.0 * peso;
    const PropModel prop = tiltPropModel(G, Tmax, A.rho);

    TiltRotorLayout lay;
    lay.armX_fwd = std::abs(G.xGondFwd); lay.armX_aft = std::abs(G.xGondAft);
    lay.armY_fwd = G.yGondFwd;           lay.armY_aft = G.yGondAft;
    lay.diffFrac = 0.35;                 lay.diffTiltMax = 8.0 * M_PI / 180.0;

    printMassProperties(base, "convertiplano, gondolas verticales");

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Dimensiones: fuselaje " << G.fusLargo << " m  |  envergadura "
              << G.envergadura << " m  |  ala S=" << tiltrotorAreaAlar(G)
              << " m^2  c=" << tiltrotorCuerdaMedia(G) << " m  AR="
              << (G.envergadura*G.envergadura / tiltrotorAreaAlar(G)) << "\n";
    std::cout << "Empuje estatico maximo: " << std::setprecision(0) << Tmax
              << " N  (T/W = 2.0, vuelo estacionario al 50% de palanca)\n";
    std::cout << "Caida de empuje de la helice: se anula a Va_ax = "
              << std::sqrt(Tmax / prop.kLapse) << " m/s con palanca 1.0\n\n";

    std::cout << "=== Transicion: geometria de masa y autoridad de control vs basculacion ===\n";
    std::cout << "  Autoridad maxima (palanca 1.0) en la Va tipica de cada punto de la transicion.\n";
    std::cout << "  Mirar como el empuje disponible se desploma al final (caida de la helice) justo\n";
    std::cout << "  cuando las superficies aerodinamicas ya tienen presion dinamica para reemplazarlo.\n\n";
    std::cout << "  tilt    cg_x    cg_z      Ixx     Iyy     Izz     Ixz  |"
                 "     Va       T  |      l       m       n   (N*m)\n";
    std::cout << "  " << std::string(101, '-') << "\n";

    for (int deg = 0; deg <= 90; deg += 15) {
        const double sigma = deg * M_PI / 180.0;
        const DroneMassProperties d = buildTiltrotor(sigma);

        // Velocidad tipica de ese punto de la transicion (interpolacion lineal
        // hasta la de crucero) — solo para evaluar la presion dinamica
        UAVState st{};
        st.u = (deg / 90.0) * 32.0;
        const AirData ad = computeAirData(st, A.rho);
        const double  T  = tiltThrust(prop, 1.0, st, sigma);
        const ControlAuthority au = tiltrotorAuthority(T, sigma, lay, A, ad.qbar);

        std::cout << std::setprecision(3)
                  << "  " << std::setw(3) << deg << "  "
                  << std::setw(7) << d.cg.x << " " << std::setw(7) << d.cg.z << "  "
                  << std::setprecision(1)
                  << std::setw(7) << d.I_full.Ixx << " " << std::setw(7) << d.I_full.Iyy << " "
                  << std::setw(7) << d.I_full.Izz << " " << std::setw(7) << d.I_full.Ixz << "  |"
                  << std::setw(7) << ad.Va << std::setw(8) << T << "  |"
                  << std::setw(8) << au.l << std::setw(8) << au.m << std::setw(8) << au.n << "\n";
    }

    std::cout << "\n=== Aerodinamica del ala (Bloque 2b): sustentacion vs peso ===\n";
    std::cout << "  Va(m/s)  alfa(deg)     CL      CD    L(N)    D(N)   L/peso\n";
    std::cout << "  " << std::string(58, '-') << "\n";
    for (double Va : {5.0, 10.0, 15.0, 20.0, 25.0, 32.0}) {
        // Angulo de ataque que equilibraria el peso a esa velocidad
        const double qbar = 0.5 * A.rho * Va * Va;
        const double CLreq = peso / (qbar * A.S);
        const double alpha = std::max(-0.35, std::min(0.35, (CLreq - A.CL0) / A.CLalpha));
        const double CL = liftCoefficient(alpha, A), CD = dragCoefficient(alpha, A);
        std::cout << std::setprecision(1)
                  << "  " << std::setw(7) << Va << std::setw(11) << alpha*180.0/M_PI
                  << std::setprecision(3) << std::setw(8) << CL << std::setw(8) << CD
                  << std::setprecision(1) << std::setw(8) << qbar*A.S*CL
                  << std::setw(8) << qbar*A.S*CD
                  << std::setprecision(2) << std::setw(9) << qbar*A.S*CL/peso << "\n";
    }
    std::cout << "\n";
}


// ─────────────────────────────────────────────────────────────────────────────
// Main — simulation example
// ─────────────────────────────────────────────────────────────────────────────
int main()
{
    const double g = 9.81;

    // Masa, CG e inercia derivados de la geometria real del dron ala delta
    // (Bloque 0) — no son valores inventados, son consecuencia de la forma.
    const DroneMassProperties drone = buildDeltaWingDrone();
    printMassProperties(drone);

    // ── Convertiplano: lo que lo distingue es que su geometria de masa NO es
    // fija — basculando las gondolas se mueven el CG y el tensor de inercia —
    // y que su autoridad de control se reparte entre rotores y superficies a
    // lo largo de la transicion. Las dos cosas se ven en una tabla.
    reporteConvertiplano();

    const double mass = drone.mass;

    UAVState state{};
    state.pn    =  0.0;
    state.pe    =  0.0;
    state.pd    = -10.0;   // 10 m altitude (NED: negative = up)
    state.u     =  5.0;    // m/s forward
    state.phi   =  0.1;    // ~5.7 deg roll
    state.theta =  0.05;   // ~2.9 deg pitch
    state.p     =  0.01;   // rad/s roll rate

    // Demo de conversion de actitud: Euler -> Cuaternion -> Matriz de rotacion
    std::cout << "=== Conversion de actitud (attitude en t=0) ===\n";
    Quaternion e0 = eulerToQuaternion(state.phi, state.theta, state.psi);
    printQuaternion(e0);

    Vec3 eulerBack = quaternionToEuler(e0);
    std::cout << "Euler recuperado (phi,theta,psi) = ("
              << eulerBack.x * 180.0 / M_PI << ", "
              << eulerBack.y * 180.0 / M_PI << ", "
              << eulerBack.z * 180.0 / M_PI << ") deg\n";

    Mat3x3 R = quaternionToRotationMatrix(e0);
    printRotationMatrix(R);
    std::cout << "\n";

    const double fx_thrust =  0.5;
    const double fy_thrust =  0.0;
    const double fz_thrust = -mass * g;  // full-weight upward thrust

    // Zero torques — pure free-flight / gravity scenario
    const double l_torque = 0.0;
    const double m_torque = 0.0;
    const double n_torque = 0.0;

    const double dt    = 0.01;
    const int    steps = 500;   // 5 seconds

    const double PISO_PD = 0.0;   // suelo solido en el plano X-Y (pd=0, NED)
    bool crashed = false;

    const int W = 12;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << std::setw(W) << "t(s)"
              << std::setw(W) << "pn(m)"
              << std::setw(W) << "pe(m)"
              << std::setw(W) << "pd(m)"
              << std::setw(W) << "u(m/s)"
              << std::setw(W) << "phi(deg)"
              << std::setw(W) << "theta(deg)"
              << std::setw(W) << "p(deg/s)"
              << "\n";
    std::cout << std::string(W * 8, '-') << "\n";

    for (int i = 0; i <= steps; ++i) {
        if (i % 50 == 0) {
            std::cout << std::setw(W) << i * dt
                      << std::setw(W) << state.pn
                      << std::setw(W) << state.pe
                      << std::setw(W) << state.pd
                      << std::setw(W) << state.u
                      << std::setw(W) << state.phi   * 180.0 / M_PI
                      << std::setw(W) << state.theta * 180.0 / M_PI
                      << std::setw(W) << state.p     * 180.0 / M_PI
                      << "\n";
        }

        // Gravity projected into body frame — updates each step as attitude changes
        double fg_x = -mass * g * std::sin(state.theta);
        double fg_y =  mass * g * std::cos(state.theta) * std::sin(state.phi);
        double fg_z =  mass * g * std::cos(state.theta) * std::cos(state.phi);

        // simulateStep() (simulate_step.h): mismo RK4, pero con el tensor de
        // inercia COMPLETO (drone.I_full: momentos + productos de inercia),
        // en vez de la forma reducida {Jx,Jy,Jz,Jxz} que usa integrate_rk4().
        state = simulateStep(state,
                              fx_thrust + fg_x,
                              fy_thrust + fg_y,
                              fz_thrust + fg_z,
                              l_torque, m_torque, n_torque,
                              drone.I_full, mass, dt);

        GroundContact gc = checkGroundCollision(state, PISO_PD);
        if (gc.colision) {
            state = resolveGroundCollision(state, PISO_PD);
            if (!crashed)
                std::cout << "\n[COLISION] impacto contra el piso en t=" << (i+1)*dt
                          << " s  (pn=" << state.pn << ", pe=" << state.pe << ")\n\n";
            crashed = true;
        }
    }

    return 0;
}
